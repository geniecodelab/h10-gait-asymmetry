# -*- coding: utf-8 -*-
"""
H10 로그 현장 분석 (KW_decoder 형식 그대로 해석)  -- numpy, scipy 만 사용

창(GUI)으로 쓰기:  h10_gui.py 를 더블클릭 (또는 python h10_gui.py)
명령으로 쓰기:
  python h10_analyze.py 로그.csv --pid A --cond 0kg_OFF_1
  python h10_analyze.py 로그.csv --pid A --cond 3kg_OFF            (기본 왼다리 LH)
  python h10_analyze.py 로그.csv --pid A --cond 3kg_TUNE1 --gain 0.5 --min-pairs 5
결과: results 폴더의 results.csv 에 한 줄씩 누적, baseline_<pid>.json 에 정상값 저장
"""
import argparse, csv, json, os
import numpy as np
from scipy.signal import butter, filtfilt, find_peaks

# ------------------------- 1. 디코딩 (KW_decoder 와 동일한 필드/상수) -------------------------
C16 = 65535
DEG = C16 / 720; VEL = C16 / 6000; CUR = C16 / 360; GYR = C16 / 1000
ACC = C16 / 78.4532; EMG = C16 / 2; UIN = C16 / 30
FIELDS = [
    ('assist_level', 'f16', 100),
    ('thighDeg_RH', 'f16', DEG), ('incPosDeg_RH', 'f16', DEG), ('MotorActCurrent_RH', 'f16', CUR),
    ('accX_Calib_RH', 'f16', ACC), ('accY_Calib_RH', 'f16', ACC), ('gyroZ_Calib_RH', 'f16', GYR),
    ('thighDeg_LH', 'f16', DEG), ('incPosDeg_LH', 'f16', DEG), ('MotorActCurrent_LH', 'f16', CUR),
    ('accX_Calib_LH', 'f16', ACC), ('accY_Calib_LH', 'f16', ACC), ('gyroZ_Calib_LH', 'f16', GYR),
    ('u_RH', 'f16', UIN), ('u_LH', 'f16', UIN), ('Pvector_ref_RH', 'f16', DEG), ('Pvector_ref_LH', 'f16', DEG),
    ('extension_control_mode', 'u8', None), ('emg_R1', 'f16', EMG), ('emg_L1', 'f16', EMG),
    ('fsr_R1', 'u16', None), ('fsr_R2', 'u16', None), ('fsr_L1', 'u16', None), ('fsr_L2', 'u16', None),
    ('free_var1', 'f16', VEL), ('gp_phase_LH', 'f16', VEL),   # free_var1: 옛 로그=오른발 phase, 새 로그=1000+1000*split_asym / free_var2: 왼발 phase
]

def decode(path):
    cols = {'loopCnt': []}
    for n, _, _ in FIELDS:
        cols[n] = []
    with open(path, 'rb') as f:
        lines = f.read().split(b'\n')
    for ln in lines:
        h = ln.replace(b'\x00', b'').decode('utf-8', 'ignore').strip()
        if len(h) < 110:
            continue
        try:
            vals = [int(h[0:8], 16)]; c = 8
            for name, t, k in FIELDS:
                if t == 'f16':
                    v = int(h[c:c+4], 16); v = v - 0x10000 if v > 0x7FFF else v
                    vals.append(v / k); c += 4
                elif t == 'u16':
                    vals.append(int(h[c:c+4], 16)); c += 4
                else:
                    vals.append(int(h[c:c+2], 16)); c += 2
        except ValueError:
            continue
        for key, v in zip(cols, vals):
            cols[key].append(v)
    d = {k: np.array(v, dtype=float) for k, v in cols.items()}
    if len(d['loopCnt']) == 0:
        raise ValueError('로그에서 읽을 수 있는 줄이 없습니다. H10 로그 파일이 맞는지 확인하세요.')
    resets = np.where(np.diff(d['loopCnt']) < 0)[0]       # 이어 붙은 이전 기록 제거
    if len(resets):
        s = resets[-1] + 1
        d = {k: v[s:] for k, v in d.items()}
    # free_var1 해석: 1000 근처면 새 펌웨어(벌어짐 비대칭 기록), 아니면 옛 펌웨어(오른발 phase)
    fv = d['free_var1']
    if len(fv) and np.median(fv) > 200:
        d['split_asym_fw'] = (fv - 1000.0) / 1000.0
        d['gp_phase_RH'] = None
    else:
        d['split_asym_fw'] = None
        d['gp_phase_RH'] = fv
    return d

# ------------------------- 1-2. 두 다리 벌어짐 비대칭 (펌웨어와 같은 계산) -------------------------
def split_profile(d, fs=1000.0):
    """걸음마다 (오른발 앞 최대 벌어짐, 왼발 앞 최대 벌어짐, 비대칭)을 계산. 펌웨어 split_asym 과 같은 방법."""
    D = d['thighDeg_RH'] - d['thighDeg_LH']
    a = (1 / fs) / (0.04 + 1 / fs)
    s1 = s2 = mid = D[0]; hi, lo = -1e9, 1e9; state = 0; A, H, Lo = [], [], []
    for x in D:
        s1 += a * (x - s1); s2 += a * (s1 - s2); mid += (1 / fs / 3.0) * (s2 - mid)
        hi = max(hi, s2); lo = min(lo, s2)
        st = 1 if s2 > mid else -1
        if st == 1 and state == -1:
            if hi - lo > 15:
                A.append((hi + lo) / (hi - lo)); H.append(hi); Lo.append(lo)
            hi, lo = -1e9, 1e9
        state = st
    A, H, Lo = np.array(A), np.array(H), np.array(Lo)
    if len(A) < 5:
        return None
    return dict(asym=float(np.median(A)), asym_iqr=np.percentile(A, [25, 75]),
                r_front=float(np.median(H)), l_front=float(-np.median(Lo)), n=len(A), all=A)

# ------------------------- 1-3. 실제로 나간 보조 (u_LH) 요약 -------------------------
GEAR, KT = 18.75, 0.085
def assist_summary(d, side='LH', fs=1000.0):
    u = np.abs(d['u_' + side]); on = u > 0.02
    if on.sum() < 20:
        return None
    e = np.diff(np.concatenate([[0], on.astype(int), [0]]))
    st, en = np.where(e == 1)[0], np.where(e == -1)[0]
    keep = (en - st) > 30                       # 30 ms 미만 잡음 제외
    st, en = st[keep], en[keep]
    if len(st) == 0:
        return None
    dur = (en - st) / fs * 1000
    peak = np.array([u[a:b].max() for a, b in zip(st, en)]) * GEAR * KT
    # 보조 시작이 모래주머니 쪽 다리의 "가장 뒤(최대 신전)"보다 얼마나 먼저/늦게인가
    b, a = butter(2, 6 / (fs / 2)); th = filtfilt(b, a, d['thighDeg_' + side])
    ext, _ = find_peaks(-th, distance=int(0.6 * fs), prominence=5)
    lead = []
    for s0 in st:
        if len(ext):
            j = int(np.argmin(np.abs(ext - s0)))
            if abs(ext[j] - s0) < 0.6 * fs:
                lead.append((s0 - ext[j]) / fs * 1000)
    return dict(n=len(st), dur=float(np.median(dur)), peak=float(np.median(peak)),
                lead=float(np.median(lead)) if lead else float('nan'),
                per_stride=len(st) / max(len(ext), 1))

# ------------------------- 2. 보행주기 분할과 지표 -------------------------
FS = 1000.0
def SI(a, b):
    return np.abs(a - b) / (0.5 * (a + b)) * 100

def strides(theta, fs=FS):
    b, a = butter(2, 6 / (fs / 2))
    th = filtfilt(b, a, theta)
    pk, _ = find_peaks(th, distance=int(0.6 * fs), prominence=8)   # 최대 굴곡 = 주기 시작
    if len(pk) < 3:
        return None
    t0 = pk[:-1] / fs; T = np.diff(pk) / fs
    fmax = np.array([th[a0:a1].max() for a0, a1 in zip(pk[:-1], pk[1:])])
    emin = np.array([th[a0:a1].min() for a0, a1 in zip(pk[:-1], pk[1:])])
    rom = fmax - emin
    ok = (np.abs(T - np.median(T)) < 0.25 * np.median(T)) & (np.abs(rom - np.median(rom)) < 0.4 * np.median(rom))
    # 짧은 복도 왕복 대응: 정상 주기가 연달아 이어진 "직선 구간"마다 첫 주기와 마지막 주기를 버린다
    # (출발 가속, 멈춤 감속 구간 제거). 돌아서거나 멈춰 선 구간은 위 조건에서 이미 빠진다.
    keep = np.zeros_like(ok)
    i = 0
    while i < len(ok):
        if not ok[i]:
            i += 1; continue
        j = i
        while j + 1 < len(ok) and ok[j + 1]:
            j += 1
        if j - i + 1 >= 4:                       # 직선 구간이 4주기 이상일 때만 사용
            keep[i + 1:j] = True                 # 첫 주기(i)와 마지막 주기(j) 제외
        i = j + 1
    s = dict(t0=t0[keep], T=T[keep], fmax=fmax[keep], emin=emin[keep], rom=rom[keep])
    s['sl'] = np.sin(np.radians(s['fmax'])) + np.sin(np.radians(-s['emin']))   # SL_est (L=1)
    s['ctr'] = (s['fmax'] + s['emin']) / 2
    s['runs'] = int(np.sum(np.diff(np.concatenate([[0], keep.astype(int)])) == 1))  # 사용한 직선 구간 수
    return s

METRICS = ['SI_strideT', 'SI_ROM', 'SI_SL', 'dCenter']
NAME = {'SI_strideT': '보행주기 시간 SI', 'SI_ROM': '각도 범위 SI', 'SI_SL': '보폭 대리값 SI', 'dCenter': '중심 차이'}
UNIT = {'SI_strideT': '%', 'SI_ROM': '%', 'SI_SL': '%', 'dCenter': '°'}

def paired(R, L):
    out = {k: [] for k in METRICS}
    for i in range(len(R['t0'])):
        j = int(np.argmin(np.abs(L['t0'] - R['t0'][i])))
        if abs(L['t0'][j] - R['t0'][i]) > 0.8 * R['T'][i]:
            continue
        out['SI_strideT'].append(SI(R['T'][i], L['T'][j]))
        out['SI_ROM'].append(SI(R['rom'][i], L['rom'][j]))
        out['SI_SL'].append(SI(R['sl'][i], L['sl'][j]))
        out['dCenter'].append(abs(R['ctr'][i] - L['ctr'][j]))
    return {k: np.array(v) for k, v in out.items()}

# ------------------------- 3. 분석 (GUI/명령 공용) -------------------------
def analyze(log, pid, cond, side='LH', gain=0.0, K=10.0, outdir='results', min_pairs=10, max_gain=10.0):
    """결과를 (문자열, 종류) 목록으로 돌려준다. 종류: 'h'제목 'n'보통 'ok'정상 'bad'경고 'act'다음행동"""
    out = []
    P = lambda s, k='n': out.append((s, k))
    d = decode(log)
    P(f'[{pid} / {cond}]  기록 길이 {len(d["loopCnt"])/FS:.1f}초', 'h')
    R, L = strides(d['thighDeg_RH']), strides(d['thighDeg_LH'])
    if R is None or L is None or len(R['T']) == 0 or len(L['T']) == 0:
        P('보행주기를 찾지 못했습니다. 실제로 걸은 로그인지 확인하세요.', 'bad'); return out, None
    M = paired(R, L); n = len(M['SI_SL'])
    if n < min_pairs:
        P(f'쓸 수 있는 좌우 주기가 {n}개뿐입니다 (필요 {min_pairs}개). 직선으로 더 길게 걸어 다시 측정하세요.', 'bad')
        return out, None
    cad = 120 / np.mean(np.concatenate([R['T'], L['T']]))
    P(f'분석한 주기: 오른쪽 {len(R["T"])}개 / 왼쪽 {len(L["T"])}개 (좌우 짝 {n}개), '
      f'사용한 직선 구간 {max(R["runs"], L["runs"])}개')
    P(f'케이던스: 분당 {cad:.0f}걸음  → 메트로놈 BPM과 ±5% 이내인지 확인하세요')
    P(f'고관절 각도 범위: 오른쪽 {np.mean(R["rom"]):.1f}° / 왼쪽 {np.mean(L["rom"]):.1f}°')
    vL = np.mean(d['gp_phase_LH'] >= 0) * 100
    if d['gp_phase_RH'] is not None:
        vR = np.mean(d['gp_phase_RH'] >= 0) * 100
        P(f'로봇이 기록한 phase 유효 비율: 오른쪽 {vR:.0f}% / 왼쪽 {vL:.0f}%')
    else:
        P(f'로봇이 기록한 왼발 phase 유효 비율: {vL:.0f}%  (새 펌웨어: 1번 칸에 벌어짐 비대칭 기록)')
    # 두 다리 벌어짐 비대칭 (중증도 지표)
    SP = split_profile(d)
    if SP:
        P(f'두 다리 벌어짐: 오른발이 앞 {SP["r_front"]:.1f}° / 왼발이 앞 {SP["l_front"]:.1f}°   '
          f'비대칭 split_asym {SP["asym"]:+.3f} (걸음 {SP["n"]}개, 0=대칭, +면 왼발이 덜 나감)')
        if d['split_asym_fw'] is not None:
            fwv = d['split_asym_fw'][len(d['split_asym_fw']) // 5:]
            P(f'  펌웨어가 기록한 split_asym 중앙값 {np.median(fwv):+.3f} (위 계산과 비슷하면 정상)')
    # 실제로 나간 보조
    AS = assist_summary(d, side)
    if AS:
        P(f'보조 실제 출력: {AS["n"]}회 (걸음당 {AS["per_stride"]:.2f}회), 길이 {AS["dur"]:.0f} ms, '
          f'최대 약 {AS["peak"]:.1f} Nm, 시작 = 다리가 가장 뒤로 간 순간 {AS["lead"]:+.0f} ms (음수=먼저)')
    # 보조가 실제로 나갔는지: 모터 전류와 제어 입력(u)의 크기 (상위 1% 값)
    iR = np.percentile(np.abs(d['MotorActCurrent_RH']), 99); iL = np.percentile(np.abs(d['MotorActCurrent_LH']), 99)
    uR = np.percentile(np.abs(d['u_RH']), 99); uL = np.percentile(np.abs(d['u_LH']), 99)
    on = max(iL, uL) > 0.05
    P(f'보조 확인 (상위 1% 크기): 왼쪽 전류 {iL:.2f} A, 입력 {uL:.2f} / 오른쪽 전류 {iR:.2f} A, 입력 {uR:.2f}'
      f'  → {"보조가 나가고 있음" if on else "보조 없음"}', 'ok' if on else 'n')
    if ('TUNE' in cond or 'ON' in cond) and not on:
        P('보조 측정인데 모터 출력이 거의 없습니다. 케이블을 뺄 때 값이 초기화됐거나 assist_on이 0이었을 수 있습니다.', 'bad')
    if cond.endswith('_OFF') or cond.startswith('0kg') or cond == 'check':
        if on:
            P('보조 OFF 측정인데 모터 출력이 있습니다. assist_on이 1로 남아 있지 않았는지 확인하세요.', 'bad')
    P('')

    os.makedirs(outdir, exist_ok=True)
    bpath = os.path.join(outdir, f'baseline_{pid}.json')
    base = json.load(open(bpath, encoding='utf-8')) if os.path.exists(bpath) else {}
    mean = {k: float(np.mean(M[k])) for k in METRICS}
    deficit = None

    if cond == 'check':
        P('착용 확인', 'h')
        P(f'  각도 범위 SI {mean["SI_ROM"]:.1f}%  (공개 데이터 정상 상한 약 13%)')
        if mean['SI_ROM'] > 13:
            P('→ 착용 문제 의심: H10 벨트를 다시 조이고 재측정하세요.', 'bad')
        else:
            P('→ 착용 상태 양호. 0kg_OFF_1 측정으로 넘어가세요.', 'ok')
    elif cond.startswith('0kg_OFF'):
        base[cond] = {k: M[k].tolist() for k in METRICS}
        if SP:
            base[cond]['split_asym'] = SP['all'].tolist()
        json.dump(base, open(bpath, 'w', encoding='utf-8'))
        P('정상값 저장 (지금까지 0kg 측정을 모두 합친 기준)', 'h')
        for k in METRICS:
            allv = np.concatenate([np.array(v[k]) for v in base.values()])
            P(f'  {NAME[k]:10s}  이번 {mean[k]:6.2f}{UNIT[k]}   기준 상한(평균+2SD) {allv.mean()+2*allv.std():6.2f}{UNIT[k]}')
        if mean['SI_ROM'] > 13:
            P('각도 범위 SI가 공개 데이터 정상 상한(약 13%)보다 큽니다 → H10 벨트를 다시 조이고 재측정하세요.', 'bad')
        else:
            P('착용 상태 양호 (각도 범위 SI가 공개 데이터 정상 범위 안)', 'ok')
        if len(base) >= 2:
            a, b = list(base)[:2]
            P('재현성 (1차 vs 2차 평균): ' + ', '.join(
                f'{NAME[k]} {np.mean(base[a][k]):.1f} vs {np.mean(base[b][k]):.1f}' for k in METRICS))
    else:
        if not base:
            P('이 참가자의 정상값(0kg_OFF)이 없습니다. 먼저 0kg_OFF_1, 0kg_OFF_2를 분석하세요.', 'bad')
        P('정상값 대비 판정', 'h')
        for k in METRICS:
            if base:
                allv = np.concatenate([np.array(v[k]) for v in base.values()])
                up = allv.mean() + 2 * allv.std()
                over = mean[k] > up
                P(f'  {NAME[k]:10s}  이번 {mean[k]:6.2f}{UNIT[k]}   상한 {up:6.2f}{UNIT[k]}   '
                  f'{"→ 비대칭 (기준 초과)" if over else "→ 정상 범위"}', 'bad' if over else 'ok')
            else:
                P(f'  {NAME[k]:10s}  이번 {mean[k]:6.2f}{UNIT[k]}')
        bs = [np.array(v['split_asym']) for v in base.values() if 'split_asym' in v]
        bs = np.concatenate(bs) if bs else None
        if SP and bs is not None:
            ref, up = bs.mean(), bs.mean() + 2 * bs.std()
            over = SP['asym'] > up
            P(f'  {"벌어짐 비대칭":10s}  이번 {SP["asym"]:+6.3f}    정상 {ref:+.3f} (상한 {up:+.3f})   '
              f'{"→ 비대칭 (기준 초과)" if over else "→ 정상 범위"}', 'bad' if over else 'ok')
            P(f'  펌웨어 assist_sym_ref 에 넣을 이 사람의 정상값: {ref:+.3f}')
        elif SP:
            P(f'  벌어짐 비대칭 {SP["asym"]:+.3f}  (0kg_OFF 를 새 분석기로 다시 분석하면 정상값과 비교됩니다)')
        aff, una = (R, L) if side == 'RH' else (L, R)
        deficit = (np.mean(una['sl']) - np.mean(aff['sl'])) / np.mean(una['sl'])
        P('')
        P(f'보폭 결손 비율 ({"왼다리" if side == "LH" else "오른다리"}): {deficit*100:+.1f}%   '
          f'(양수 = 모래주머니 쪽 보폭이 짧음, 0에 가까울수록 좋음)', 'h')
        if 'ON' in cond:
            P('ON 본 측정입니다. 보조량을 바꾸지 마세요.', 'act')
        else:
            # 판단 기준: 벌어짐 비대칭이 이 사람의 정상 범위(0kg 평균+2SD)로 돌아왔는가 (세기는 1 Nm 단위)
            if 'TUNE' not in cond:
                P('보조 OFF 측정: 이 값이 이 조건의 중증도(진단)입니다. 세기 조정은 TUNE 측정에서 합니다.', 'act')
            elif SP and bs is not None:
                ref, up = bs.mean(), bs.mean() + 2 * bs.std()
                if SP['asym'] <= ref - 2 * bs.std():
                    P(f'보조가 넘칩니다 (왼발이 오히려 더 나감) → assist_gain_Nm을 {max(gain - 1, 0):.0f} Nm로 낮추세요.', 'act')
                elif SP['asym'] <= up:
                    P(f'벌어짐 비대칭이 정상 범위로 돌아왔습니다 → {gain:.0f} Nm로 확정하고 ON 본 측정을 하세요.', 'act')
                elif gain >= max_gain:
                    P(f'최대 세기({max_gain:g} Nm)에서도 정상 범위에 못 미칩니다 → {max_gain:g} Nm로 ON 측정하고, '
                      f'"고관절 보조만으로는 부족한 중증도"로 기록하세요.', 'act')
                else:
                    P(f'아직 비대칭이 큽니다 → assist_gain_Nm을 {min(gain + 1, max_gain):.0f} Nm로 올려 다시 측정하세요 '
                      f'(목표 split_asym {up:+.3f} 이하).', 'act')
            else:
                P('0kg_OFF 를 이 분석기로 다시 분석하면 세기 판단이 나옵니다.', 'bad')

    rp = os.path.join(outdir, 'results_v2.csv')      # 벌어짐/보조 출력 칸이 추가된 새 결과 파일
    new = not os.path.exists(rp)
    with open(rp, 'a', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f)
        if new:
            w.writerow(['pid', 'cond', 'side', 'gain', 'pairs', 'cadence'] + METRICS + ['deficit_pct',
                        'split_asym', 'R_front_deg', 'L_front_deg', 'assist_n', 'assist_per_stride',
                        'assist_dur_ms', 'assist_peak_Nm', 'assist_lead_ms'])
        f3 = lambda x: '' if x is None else round(x, 3)
        w.writerow([pid, cond, side, gain, n, round(cad, 1)] + [round(mean[k], 3) for k in METRICS]
                   + ['' if deficit is None else round(deficit * 100, 2)]
                   + ([f3(SP['asym']), f3(SP['r_front']), f3(SP['l_front'])] if SP else ['', '', ''])
                   + ([AS['n'], f3(AS['per_stride']), f3(AS['dur']), f3(AS['peak']), f3(AS['lead'])] if AS else ['', '', '', '', '']))
    P('')
    P(f'결과를 {os.path.abspath(rp)} 에 기록했습니다.')
    return out, dict(mean=mean, deficit=deficit, cad=cad, n=n)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log'); ap.add_argument('--pid', required=True); ap.add_argument('--cond', required=True)
    ap.add_argument('--side', choices=['RH', 'LH'], default='LH')
    ap.add_argument('--gain', type=float, default=0.0); ap.add_argument('--K', type=float, default=10.0)
    ap.add_argument('--max-gain', type=float, default=10.0); ap.add_argument('--outdir', default='results'); ap.add_argument('--min-pairs', type=int, default=10)
    a = ap.parse_args()
    lines, _ = analyze(a.log, a.pid, a.cond, a.side, a.gain, a.K, a.outdir, a.min_pairs, a.max_gain)
    for s, _k in lines:
        print(s)

if __name__ == '__main__':
    main()
