# 데이터 수집 절차 (팀원용)

H10 + 확장보드(STM32H743)로 **왼다리 보조 OFF / ON 보행 데이터**를 모으는 방법입니다.
처음 하는 사람도 이 순서대로만 하면 됩니다. 순서는 **[1] 펌웨어 넣기 → [2] Live Expressions 준비 → [3] 한 번 측정하기 → [4] 분석하기** 입니다.

> 측정 조건 (모두 같게): 왼발목 모래주머니 · 메트로놈 **82 BPM** (딱 1번 = 한 걸음) · 복도 **4회 왕복**, 돌아설 때 2~3초 멈춤 · 앱 보조 레벨 **100 %**

---

## 1. 펌웨어 넣기 (처음 한 번)

STM32CubeIDE에서 수업 프로젝트 `Expansion_Board_H743AII6` 를 연 상태에서 시작합니다.

| 저장소 파일 | 프로젝트 안 위치 | 방법 |
|---|---|---|
| `firmware/Algorithm_Ctrl/Src/algorithm_ctrl.c` | `AGR_Library_MD/Apps/Tasks/Algorithm_Ctrl/Src/algorithm_ctrl.c` | **기존 파일에 그대로 덮어쓰기** |
| `firmware/Core/Inc/gait_phase.h` | `Core/Inc/gait_phase.h` | 새로 붙여넣기 (`main.h` 가 있는 폴더) |
| `firmware/Core/Src/gait_phase.c` | `Core/Src/gait_phase.c` | 새로 붙여넣기 (`main.c` 가 있는 폴더) |

1. 위 세 파일을 탐색기에서 해당 폴더에 복사합니다. (덮어쓰기 전에 원래 `algorithm_ctrl.c` 를 `algorithm_ctrl.c.bak` 으로 복사해 두면 안전합니다. 확장자를 `.c` 로 남기면 같이 빌드되어 오류가 나니 꼭 `.bak` 으로)
2. CubeIDE 왼쪽 **Project Explorer** 에서 프로젝트를 클릭하고 **F5** (새로고침). `Core/Src` 에 `gait_phase.c` 가 보여야 합니다.
3. 열려 있던 `algorithm_ctrl.c` 탭이 있으면 닫습니다 (저장하겠냐고 물으면 **No**).
4. **Ctrl+B** 빌드 → Console 에 `0 errors` 확인.
5. 로봇 전원 ON, ST-LINK 연결 → **Run → Debug Configurations → `Expansion_Board_H743AII6 Debug`** (Download 포함) → **Debug**.
6. Console 에 `Download verified successfully` → `main()` 에서 멈추면 **F8 (Resume)**.

> 다음부터 같은 빌드가 이미 올라가 있으면 **`Attach_only`** 설정으로 붙이면 됩니다 (보드가 리셋되지 않음).
> 펌웨어를 새로 빌드했는데 Attach_only 로 붙이면 값이 이상하게(255, nan) 보입니다 → Download 설정으로 다시 올리세요.

---

## 2. Live Expressions 준비

Debug 화면 오른쪽 위 **Live Expressions** 탭 → **Add new expression** 에 아래 이름을 하나씩 입력합니다.
(한 번 넣으면 다음에도 목록은 남지만, **값은 전원/Download 때마다 기본값으로 돌아갑니다.**)

### 2-1. 매번 바꾸는 값 (Resume 후 조작)

| 변수 | 넣는 값 | 의미 |
|---|---|---|
| `CM_connect_signal` | **1** | 로봇 본체와 연결 요청. 넣으면 스스로 0으로 돌아감 (정상) |
| `assist_gain_Nm` | OFF: 0 · **3 kg ON: 6** · **6 kg ON: 10** | 왼다리 보조 세기 [Nm] (최대 10) |
| `assist_on` | OFF: **0** · ON: **1** (마지막에) | 보조 켜기/끄기. 켜고 끌 때 1초에 걸쳐 서서히 |

### 2-2. 확인만 하는 값

| 변수 | 정상 값 | 의미 |
|---|---|---|
| `SUIT_State_curr` | **3~17 또는 33 근처** (보조 시작 후) | H10 상태. 0 = 연결 안 됨, 1 = 대기 |
| `controlMode` | `USER_DEFINED_CTRL` | 우리 제어 모드 |
| `assist_pulse_cnt` | 왼발 한 걸음마다 +1 | 보조가 시작된 횟수 |
| `assist_trigS_cnt` | 위와 같이 증가 | 두 다리 벌어짐 기준으로 시작된 횟수 |
| `assist_stop_cnt` | 거의 안 오름 | 안전 조건(허벅지 35° 이상, 250°/s 이상)으로 중단된 횟수 |
| `split_asym` | 0 kg ≈ 0, 무게 달면 + | 두 다리 벌어짐 비대칭 (+ = 왼발이 덜 나감) |
| `assist_dur_eff_ms` | 450~730 | 이번 걸음에 미는 시간 [ms] |
| `gp_theta_LH` | 서 있으면 0° 근처 | 왼 허벅지 각도 |

### 2-3. 기본값 그대로 두는 값 (바꿀 필요 없음)

`assist_trig_src` = 3 (벌어짐 기준) · `assist_dur_frac` = 0.40 · `assist_lead_frac` = 0.3 · `assist_arm_dps` = 10 · `assist_k_dur` = 2 · `assist_sym_ref` = −0.04 · `assist_max_deg` = 35 · `assist_max_dps` = 250 · `assist_warm_n` = 3

> Live Expressions 값은 갱신이 느립니다. **판단은 SD 로그 분석으로** 하고, 여기서는 "켜졌나 / 정상인가"만 확인하세요.

---

## 3. 한 번 측정하기 (전원은 끝날 때까지 켜둔 채로)

측정 하나 = 조건 하나 (예: `C_3kg_ON`). 조건마다 아래를 반복합니다.

1. **SD 카드의 이전 로그 파일을 지운다** (기록이 같은 파일에 이어 붙기 때문)
2. **전원 ON** → 노트북 연결 → Debug(또는 Attach_only) → **F8 Resume**
3. Live Expressions: `CM_connect_signal = 1` → `SUIT_State_curr` 가 0이 아닌지 확인
4. 착용한 채 **차렷 자세로 서서 중립 각도 설정** (`gp_theta_LH` 가 0° 근처인지 확인)
5. **"보조를 시작합니다"** (앱/버튼) → `SUIT_State_curr` 3~17 / 33 근처 확인
6. Live Expressions: `assist_gain_Nm` 입력 → **마지막에** `assist_on` (OFF 측정은 0 그대로, ON 은 1)
7. 제자리에서 왼다리 몇 번 스윙 → ON 이면 뒤에서 앞으로 당겨주는 힘이 느껴지는지, `assist_pulse_cnt` 가 오르는지 확인
8. **Ctrl+F2** (디버그 종료) → **ST-LINK 케이블을 보드에서 뽑는다** (전원은 끄지 않음)
9. 다시 제자리 스윙으로 보조가 그대로 들어오는지 확인 (안 들어오면 값이 초기화된 것 → 아래 문제 해결)
10. **82 BPM 메트로놈에 맞춰 4회 왕복** (돌아설 때 2~3초 멈춤)
11. **"보조를 종료합니다"** → 전원 OFF
12. SD 카드를 빼서 로그를 노트북 `data/raw/` 에 복사하고 **이름을 바꾼다**: `<참가자>_<조건>.csv`
    예) `C_0kg_1.csv`, `C_0kg_2.csv`, `C_3kg_OFF.csv`, `C_3kg_ON.csv`, `C_6kg_OFF.csv`, `C_6kg_ON.csv`

### 측정 순서 (한 사람)

| 순서 | 조건 | 모래주머니 | `assist_on` | `assist_gain_Nm` |
|---|---|---|---|---|
| 1 | `0kg_1` | 없음 | 0 | 0 |
| 2 | `0kg_2` | 없음 | 0 | 0 |
| 3 | `3kg_OFF` | 왼발목 3 kg | 0 | 0 |
| 4 | `3kg_ON` | 왼발목 3 kg | 1 | 6 |
| 5 | `6kg_OFF` | 왼발목 6 kg | 0 | 0 |
| 6 | `6kg_ON` | 왼발목 6 kg | 1 | 10 |

처음 10 Nm 로 걸을 때는 8 Nm 로 한 번 걸어보고 올리세요. 걷기 시작 후 처음 3걸음은 힘이 40→100 % 로 서서히 올라갑니다.

---

## 4. 분석하기 (bat 파일)

1. `analysis/run_gui.bat` 더블클릭 (처음 한 번: 명령창에서 `pip install -r analysis/requirements.txt`)
2. 창에서
   - **로그 파일**: 찾기 → `data/raw/C_0kg_1.csv`
   - **참가자**: `C`
   - **조건**: `0kg_OFF_1` (파일과 맞게)
   - **넣어 둔 보조량 [Nm]**: ON 이면 6 또는 10 (OFF 는 자동 0)
   - **모래주머니 쪽**: 왼다리
3. **분석 실행** → 결과가 창에 나오고 `analysis/results/results_v2.csv` 에 한 줄씩 쌓입니다.
4. **반드시 `0kg_OFF_1`, `0kg_OFF_2` 를 먼저** 분석하세요 (그 사람의 정상값이 `baseline_<참가자>.json` 으로 저장되어야 비교가 됩니다).
5. 결과 창에서 확인할 것
   - `케이던스`: 분당 80~85걸음 근처인가 (82 BPM)
   - `보조 실제 출력`: ON 이면 **걸음당 약 1.0회**, 최대 세기가 넣은 값과 같은가
   - `두 다리 벌어짐` / `split_asym`: OFF 보다 ON 이 0 에 가까워졌나
6. 그림: `python scripts/make_figures.py --logs data/raw --pid C --out results/figures_C`

---

## 문제 해결

| 증상 | 원인 / 해결 |
|---|---|
| `SUIT_State_curr` 가 0 | `CM_connect_signal = 1` 을 다시 넣기 |
| 케이블 뽑은 뒤 보조가 안 들어옴 | 분리하면서 값이 초기화됨 → `algorithm_ctrl.c` 의 `assist_on`, `assist_gain_Nm` 기본값을 바꿔 빌드·Download |
| `Target is not responding` / `HardFault_Handler` 에서 멈춤 | 리셋 전에 **Fault Analyzer** 화면과 Expressions 의 `hdma_adc2` 를 캡처해 공유 → 전원 껐다 켜고 다시 Download |
| 다리가 확 튀거나 프레임이 걸림 | 바로 `assist_on = 0` → 보조 종료 → 전원 OFF 후 손으로 천천히 차렷 자세. **안 입은 채로는 2 Nm 이하로만 시험** |
| 전원이 안 꺼짐 / 블루투스 안 됨 | ST-LINK 분리 → 전원 버튼 10~15초 → 안 되면 배터리 분리 |
| 결과 창에 "보조 OFF 측정인데 모터 출력이 있습니다" | `assist_on` 이 1 로 남아 있었음 → 다시 측정 |
