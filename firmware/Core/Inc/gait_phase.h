/**
 * gait_phase.h  -  Phase-portrait based gait phase estimator (STM32 / any C99 MCU)
 *
 * 입력 : 한쪽 다리(고관절/허벅지) 각도 theta [deg], 고정 주기(dt)로 호출
 * 출력 : phase_pct (0~100 %), 정규화된 phase portrait 좌표, 보행 주파수, valid 플래그
 *
 * 처리 흐름 (강의자료 GPEM 블록도와 동일)
 *   theta -> [GFAF: 보행주파수 적응 2차 LPF] -> theta_f -> d/dt -> theta_dot
 *         -> [정규화: 중심 O, 진폭 A] -> (theta_n, thetadot_n)
 *         -> phi = atan2(-thetadot_n, theta_n)  -> phase% = phi / 2pi * 100
 *   phase 0 % = 각도 최대(고관절 굴곡 최대, heel strike 근처 / 왕복운동의 한쪽 끝점)
 *   보행주파수는 각도 최대점(theta_dot 의 + -> - 영점 교차) 사이 간격으로 추정 -> LPF 차단주파수에 되먹임
 */
#ifndef GAIT_PHASE_H
#define GAIT_PHASE_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

typedef struct {
    /* ---------- 설정 (init 후 필요하면 수정) ---------- */
    float dt;            /* 샘플 주기 [s]                                   */
    float alpha;         /* LPF 차단 wc = alpha * w_gait  (기본 5)           */
    float sign;          /* +1 / -1 : 각도 부호가 반대면 -1                  */
    float min_amp_deg;   /* 이 값보다 진폭이 작으면 valid=0 (기본 3 deg)     */
    float min_vel_dps;   /* 영점교차 검출 최소 속도 [deg/s] (기본 4)         */
    float f_min_hz;      /* 허용 보행주파수 범위 (기본 0.25 ~ 2.5 Hz)        */
    float f_max_hz;

    /* ---------- 내부 상태 ---------- */
    float w_gait, lp_a;
    float th1, th2, th_prev;
    float th_max, th_min, thd_max, thd_min;
    float run_thd_max, run_thd_min;
    uint32_t n, n_last_max, n_last_event, n_max_events;
    uint8_t started, armed_pos, armed_neg, have_max, have_min, have_vel;

    /* ---------- 출력 (Live Expressions 로 보기) ---------- */
    float   phase_pct;   /* 0 ~ 100 [%]                                     */
    float   theta_f;     /* 필터된 각도 [deg]                               */
    float   thetadot;    /* 필터된 각속도 [deg/s]                           */
    float   theta_n;     /* 정규화 각도   (원 위의 x)                       */
    float   thetadot_n;  /* 정규화 각속도 (원 위의 y)                       */
    float   f_gait_hz;   /* 추정 보행(왕복) 주파수 [Hz]                     */
    uint8_t valid;       /* 1: phase 신뢰 가능                              */
} GaitPhase_t;

void  gait_phase_init  (GaitPhase_t *g, float fs_hz);
void  gait_phase_reset (GaitPhase_t *g);
/* 반환값 = phase_pct. 고정 주기 타이머 인터럽트/제어 루프에서 호출 */
float gait_phase_update(GaitPhase_t *g, float theta_deg);

#ifdef __cplusplus
}
#endif
#endif
