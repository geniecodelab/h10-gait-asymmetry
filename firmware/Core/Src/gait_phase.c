#include "gait_phase.h"
#include <math.h>

#define TWO_PI_F 6.28318530718f
#define BIG_F    1.0e30f

static void set_coeff(GaitPhase_t *g)
{
    g->lp_a = 1.0f - expf(-g->alpha * g->w_gait * g->dt);   /* 1차 LPF 이산화 계수 */
}

void gait_phase_reset(GaitPhase_t *g)
{
    g->started = 0; g->armed_pos = g->armed_neg = 0;
    g->have_max = g->have_min = g->have_vel = 0;
    g->n = g->n_last_max = g->n_last_event = g->n_max_events = 0;
    g->run_thd_max = -BIG_F; g->run_thd_min = BIG_F;
    g->valid = 0;
}

void gait_phase_init(GaitPhase_t *g, float fs_hz)
{
    g->dt = 1.0f / fs_hz;
    g->alpha = 5.0f;  g->sign = 1.0f;
    g->min_amp_deg = 3.0f;  g->min_vel_dps = 4.0f;
    g->f_min_hz = 0.25f;    g->f_max_hz = 2.5f;
    g->w_gait = TWO_PI_F * 0.8f;               /* 초기 보행주파수 추정 0.8 Hz */
    set_coeff(g);
    g->phase_pct = 0; g->theta_f = g->thetadot = 0;
    g->theta_n = g->thetadot_n = 0; g->f_gait_hz = 0.8f;
    gait_phase_reset(g);
}

float gait_phase_update(GaitPhase_t *g, float theta_deg)
{
    const float x = g->sign * theta_deg;
    if (!g->started) { g->th1 = g->th2 = g->th_prev = x; g->started = 1; }
    g->n++;

    /* 1) GFAF : 2차 LPF (1차 2단), 차단 = alpha * w_gait */
    g->th1 += g->lp_a * (x    - g->th1);
    g->th2 += g->lp_a * (g->th1 - g->th2);
    const float th  = g->th2;
    /* 2) 미분 (필터된 각도의 미분 -> 각도와 같은 지연) */
    const float thd = (th - g->th_prev) / g->dt;
    g->th_prev = th;
    g->theta_f = th;  g->thetadot = thd;

    if (thd > g->run_thd_max) g->run_thd_max = thd;
    if (thd < g->run_thd_min) g->run_thd_min = thd;

    /* 3) 극값 이벤트 (theta_dot 영점교차) -> 중심/진폭/주기 갱신 */
    const float T_est = TWO_PI_F / g->w_gait;
    float thr = g->min_vel_dps;
    if (g->have_vel) {
        float t2 = 0.3f * 0.5f * (g->thd_max - g->thd_min);
        if (t2 > thr) thr = t2;
    }
    if (thd >  thr) g->armed_pos = 1;
    if (thd < -thr) g->armed_neg = 1;
    const uint32_t refr = (uint32_t)(0.4f * T_est / g->dt);   /* 불응기: 이중 검출 방지 */

    if (g->armed_pos && thd <= 0.0f) {                         /* 각도 최대 = phase 0 % */
        g->armed_pos = 0;
        if ((g->n - g->n_last_max) > refr) {
            g->th_max = th;  g->have_max = 1;
            if (g->n_max_events >= 1) {
                float period = (float)(g->n - g->n_last_max) * g->dt;
                if (period > 1.0f / g->f_max_hz && period < 1.0f / g->f_min_hz) {
                    float w_new = TWO_PI_F / period;
                    g->w_gait = (g->n_max_events == 1) ? w_new : 0.6f * g->w_gait + 0.4f * w_new;
                    set_coeff(g);
                    g->f_gait_hz = g->w_gait / TWO_PI_F;
                }
                g->thd_max = g->run_thd_max;  g->thd_min = g->run_thd_min;  g->have_vel = 1;
            }
            g->run_thd_max = -BIG_F;  g->run_thd_min = BIG_F;
            g->n_last_max = g->n;  g->n_last_event = g->n;  g->n_max_events++;
        }
    }
    if (g->armed_neg && thd >= 0.0f) {                         /* 각도 최소 */
        g->armed_neg = 0;
        g->th_min = th;  g->have_min = 1;  g->n_last_event = g->n;
    }
    /* 멈춤 감지: 3주기 이상 극값이 없으면 무효화하고 재시작 */
    if (g->n_last_event && (float)(g->n - g->n_last_event) * g->dt > 3.0f * T_est) {
        g->have_max = g->have_min = g->have_vel = 0; g->n_max_events = 0; g->n_last_event = 0;
    }

    /* 4) 정규화 + phase 계산 */
    g->valid = 0;
    if (g->have_max && g->have_min && g->have_vel) {
        if (th > g->th_max) g->th_max = th;                    /* 진폭 증가 즉시 반영 */
        if (th < g->th_min) g->th_min = th;
        const float A_th  = 0.5f * (g->th_max  - g->th_min);
        const float A_thd = 0.5f * (g->thd_max - g->thd_min);
        if (A_th >= g->min_amp_deg && A_thd > 1e-3f) {
            const float O_th  = 0.5f * (g->th_max  + g->th_min);
            const float O_thd = 0.5f * (g->thd_max + g->thd_min);
            g->theta_n    = (th  - O_th ) / A_th;
            g->thetadot_n = (thd - O_thd) / A_thd;
            /* 필터 지연(2차: 2*atan(1/alpha))은 상수 -> 보상 */
            float phi = atan2f(-g->thetadot_n, g->theta_n) + 2.0f * atanf(1.0f / g->alpha);
            if (phi < 0.0f)      phi += TWO_PI_F;
            if (phi >= TWO_PI_F) phi -= TWO_PI_F;
            g->phase_pct = phi * (100.0f / TWO_PI_F);
            g->valid = 1;
        }
    }
    return g->phase_pct;
}
