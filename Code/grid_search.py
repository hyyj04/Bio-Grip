import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

# 1. 경로 설정 (절대 경로를 사용하여 에러 방지)
file_path = r'C:\Users\ee\Desktop\Bio Grip REPO\Bio-Grip\ble_csv_logs\ppg_gsr_ble_20260505_083415.csv'

if not os.path.exists(file_path):
    print(f"에러: {file_path} 파일을 찾을 수 없습니다. 경로를 확인하세요.")
    exit()

# 2. 데이터 로드
data = pd.read_csv(file_path)

# 3. 펌웨어 헤더와 파이썬 변수 매핑 (제공된 펌웨어 기준)
# 펌웨어 의 헤더 명칭을 그대로 사용합니다.
try:
    data['s_ppg'] = data['final_ppg_stress_score']
    data['s_gsr'] = data['final_gsr_stress_score']
    data['r_ppg'] = data['valid_window_flag']
    data['r_gsr'] = data['gsr_valid_sample_flag']
    data['mode']  = data['ppg_mode']
except KeyError as e:
    print(f"에러: CSV에 {e} 컬럼이 없습니다. 펌웨어의 ENABLE_SERIAL_CSV 설정을 확인하세요.")
    exit()

# 4. Ground Truth 설정 (설문 결과 입력)
REST_SURVEY = 20.0    # 예: 피험자가 REST 구간 후 답한 점수
STRESS_SURVEY = 80.0  # 예: 피험자가 STRESS 구간 후 답한 점수

def get_ground_truth(mode):
    if mode == "REST": return REST_SURVEY
    if mode == "STRESS": return STRESS_SURVEY
    return np.nan

data['y_true'] = data['mode'].apply(get_ground_truth)
valid_data = data.dropna(subset=['y_true']).copy()

# 5. Grid Search를 통한 alpha, beta 최적화
best_alpha = 0
best_rmse = float('inf')

for alpha in np.linspace(0, 1, 101):
    beta = 1.0 - alpha
    
    # 융합 수식 적용
    denom = (valid_data['r_ppg'] * alpha) + (valid_data['r_gsr'] * beta)
    # 분모가 0이 되는 구간(둘 다 미접촉)은 NaN 처리
    w_ppg = (valid_data['r_ppg'] * alpha) / denom.replace(0, np.nan)
    w_gsr = (valid_data['r_gsr'] * beta) / denom.replace(0, np.nan)
    
    # 최종 점수 시뮬레이션
    s_final = (w_ppg * valid_data['s_ppg']) + (w_gsr * valid_data['s_gsr'])
    
    # RMSE 계산 (실제값과 예측값의 차이)
    rmse = np.sqrt(((s_final - valid_data['y_true']) ** 2).mean())
    
    if not np.isnan(rmse) and rmse < best_rmse:
        best_rmse = rmse
        best_alpha = alpha

best_beta = 1.0 - best_alpha
print(f"최적의 결과 - Alpha: {best_alpha:.2f}, Beta: {best_beta:.2f}")
print(f"최소 오차(RMSE): {best_rmse:.4f}")

# 6. 결과 시각화
best_denom = (data['r_ppg'] * best_alpha) + (data['r_gsr'] * best_beta)
data['s_fused'] = ((data['r_ppg'] * best_alpha * data['s_ppg']) + 
                   (data['r_gsr'] * best_beta * data['s_gsr'])) / best_denom.replace(0, np.nan)

plt.figure(figsize=(12, 6))
plt.plot(data['time_ms']/1000, data['s_fused'], label='Fused Stress Score', color='blue', linewidth=1.5)
plt.step(data['time_ms']/1000, data['y_true'], where='post', label='Survey Ground Truth', color='red', linestyle='--')
plt.title(f'Stress Index Calibration (Alpha={best_alpha:.2f}, Beta={best_beta:.2f})')
plt.ylabel('Stress Level')
plt.xlabel('Time (sec)')
plt.legend()
plt.grid(alpha=0.3)
plt.show()