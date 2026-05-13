import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# 1. 가상의 실험 데이터 로드 (실제 수집하신 CSV 파일로 대체하세요)
# columns: ['time_ms', 'mode', 's_ppg', 's_gsr', 'r_ppg', 'r_gsr']
data = pd.read_csv('experimental_log.csv')

# 2. Ground Truth (설문 점수) 설정
# 피험자가 REST 후 15점, STRESS 후 85점이라고 응답했다고 가정
REST_SURVEY = 15.0
STRESS_SURVEY = 85.0

def get_ground_truth(mode):
    if mode == "REST": return REST_SURVEY
    if mode == "STRESS": return STRESS_SURVEY
    return np.nan

data['y_true'] = data['mode'].apply(get_ground_truth)

# 분석을 위해 REST와 STRESS 구간만 추출
valid_data = data.dropna(subset=['y_true']).copy()

# 3. 최적화 알고리즘 (Grid Search)
best_alpha = 0
best_rmse = float('inf')
results = []

# alpha를 0.0에서 1.0까지 탐색
for alpha in np.linspace(0, 1, 101):
    beta = 1.0 - alpha
    
    # 가중치 계산 식 적용
    denom = (valid_data['r_ppg'] * alpha) + (valid_data['r_gsr'] * beta)
    # 분모가 0인 경우 방지
    w_ppg = (valid_data['r_ppg'] * alpha) / denom.replace(0, np.nan)
    w_gsr = (valid_data['r_gsr'] * beta) / denom.replace(0, np.nan)
    
    # 최종 점수 계산
    s_final = (w_ppg * valid_data['s_ppg']) + (w_gsr * valid_data['s_gsr'])
    
    # RMSE 계산
    rmse = np.sqrt(((s_final - valid_data['y_true']) ** 2).mean())
    
    if rmse < best_rmse:
        best_rmse = rmse
        best_alpha = alpha
    
    results.append((alpha, rmse))

# 최적 값 도출
best_beta = 1.0 - best_alpha
print(f"최적의 Alpha: {best_alpha:.2f}, Beta: {best_beta:.2f}")
print(f"최소 RMSE: {best_rmse:.4f}")

# 4. 결과 시각화 (개형 비교)
best_denom = (data['r_ppg'] * best_alpha) + (data['r_gsr'] * best_beta)
data['w_ppg_final'] = (data['r_ppg'] * best_alpha) / best_denom.replace(0, np.nan)
data['w_gsr_final'] = (data['r_gsr'] * best_beta) / best_denom.replace(0, np.nan)
data['s_final_best'] = (data['w_ppg_final'] * data['s_ppg']) + (data['w_gsr_final'] * data['s_gsr'])

plt.figure(figsize=(12, 6))
plt.plot(data['time_ms']/1000, data['s_final_best'], label='BioGrip Stress Index (Fused)', color='blue')
plt.step(data['time_ms']/1000, data['y_true'], where='post', label='Ground Truth (Survey)', color='red', linestyle='--')
plt.title(f'Stress Index Optimization (Alpha={best_alpha:.2f}, Beta={best_beta:.2f})')
plt.xlabel('Time (sec)')
plt.ylabel('Stress Score')
plt.legend()
plt.grid(True)
plt.show()