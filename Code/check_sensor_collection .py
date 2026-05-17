import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def verify_biosignals(csv_filepath):
    print(f"데이터 로딩 중: {csv_filepath}")
    # 데이터 로드 (첫 번째 행이 헤더임을 가정)
    df = pd.read_csv(csv_filepath)
    
    # ---------------------------------------------------------
    # 1. 기본 정보 및 시간축 변환
    # ---------------------------------------------------------
    # time_ms를 초(초) 단위로 변환
    df['time_sec'] = df['time_ms'] / 1000.0
    total_time_sec = df['time_sec'].iloc[-1] - df['time_sec'].iloc[0]
    total_samples = len(df)
    
    # ---------------------------------------------------------
    # 2. 샘플링률 (Sampling Rate) 및 끊김률 (Dropout Rate) 검증
    # ---------------------------------------------------------
    # 시간 차이(delta) 계산
    df['time_delta_ms'] = df['time_ms'].diff()
    
    # 평균 샘플링 주기 및 샘플링률 계산
    avg_delta_ms = df['time_delta_ms'].mean()
    actual_sampling_rate = 1000.0 / avg_delta_ms if avg_delta_ms > 0 else 0
    
    # 끊김률: 예상 측정 주기보다 1.5배 이상 지연된 구간을 끊김(Dropout)으로 간주
    # (참고: 아두이노 코드 기준 PRINT_INTERVAL_MS = 2000 이면 2초 주기로 로깅됨)
    expected_interval = df['time_delta_ms'].median()
    dropout_threshold = expected_interval * 1.5
    dropouts = df[df['time_delta_ms'] > dropout_threshold]
    dropout_rate = (len(dropouts) / total_samples) * 100

    # ---------------------------------------------------------
    # 3. 접촉 여부 (Contact Status)
    # ---------------------------------------------------------
    # PPG 접촉: 아두이노 코드의 NO_FINGER_IR_THRESHOLD (10000) 기준 적용
    df['ppg_contact'] = df['ir_raw'] >= 10000
    ppg_contact_ratio = (df['ppg_contact'].sum() / total_samples) * 100
    
    # GSR 접촉: gsr_contact_ok 플래그 사용
    gsr_contact_ratio = (df['gsr_contact_ok'].sum() / total_samples) * 100

    # ---------------------------------------------------------
    # 4. 신호 품질 (Signal Quality)
    # ---------------------------------------------------------
    # PPG SQI 점수 평균 (신호가 유효할 때만 계산)
    valid_sqi_df = df[df['sqi_score'].notna()]
    avg_ppg_sqi = valid_sqi_df['sqi_score'].mean() if not valid_sqi_df.empty else 0.0

    # ---------------------------------------------------------
    # 5. 유효 신호 비율 (Valid Signal Ratio)
    # ---------------------------------------------------------
    # PPG 유효 비율: valid_window_flag == 1
    ppg_valid_ratio = (df['valid_window_flag'] == 1).sum() / total_samples * 100
    
    # GSR 유효 비율: gsr_valid_sample_flag == 1
    gsr_valid_ratio = (df['gsr_valid_sample_flag'] == 1).sum() / total_samples * 100

    # =========================================================
    # 📈 결과 출력
    # =========================================================
    print("=" * 50)
    print("🔬 BioGrip 센서 수집 검증 리포트")
    print("=" * 50)
    print(f"⏱ 총 측정 시간: {total_time_sec:.2f} 초")
    print(f"📊 총 기록 샘플 수: {total_samples} 개")
    print(f"🔄 실제 로깅 샘플링률: {actual_sampling_rate:.2f} Hz (평균 주기: {avg_delta_ms:.0f} ms)")
    print(f"⚠️ 데이터 끊김률 (Dropout): {dropout_rate:.2f}% ({len(dropouts)}회 발생)")
    print("-" * 50)
    print("[PPG 센서 (MAX30102)]")
    print(f" 👉 접촉 유지율 (IR > 10k): {ppg_contact_ratio:.2f}%")
    print(f" 👉 평균 신호 품질 (SQI): {avg_ppg_sqi:.2f} / 100")
    print(f" 👉 유효 신호 비율 (Valid Flag): {ppg_valid_ratio:.2f}%")
    print("-" * 50)
    print("[GSR 센서 (전극)]")
    print(f" 👉 접촉 유지율 (Contact OK): {gsr_contact_ratio:.2f}%")
    print(f" 👉 유효 신호 비율 (Valid Flag): {gsr_valid_ratio:.2f}%")
    print("=" * 50)

    # =========================================================
    # 📉 데이터 시각화
    # =========================================================
    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    
    # 1. PPG IR 신호 및 접촉 상태
    axes[0].plot(df['time_sec'], df['ir_raw'], label='IR Raw', color='red', alpha=0.7)
    axes[0].axhline(y=10000, color='gray', linestyle='--', label='Contact Threshold (10k)')
    axes[0].set_title('PPG IR Raw Signal & Contact Status')
    axes[0].set_ylabel('Amplitude')
    axes[0].legend(loc='upper right')
    
    # 2. PPG SQI (신호 품질)
    axes[1].plot(df['time_sec'], df['sqi_score'], label='SQI Score', color='orange')
    axes[1].axhline(y=50, color='red', linestyle='--', label='Min SQI Threshold (50)')
    axes[1].set_title('PPG Signal Quality Index (SQI)')
    axes[1].set_ylabel('Score (0-100)')
    axes[1].set_ylim(0, 110)
    axes[1].legend(loc='upper right')

    # 3. GSR Raw 및 Phasic 신호
    axes[2].plot(df['time_sec'], df['raw_gsr'], label='Raw GSR', color='blue', alpha=0.5)
    axes[2].plot(df['time_sec'], df['phasic'], label='Phasic (SCR)', color='purple')
    axes[2].set_title('GSR Signal (Raw & Phasic)')
    axes[2].set_ylabel('ADC Value')
    axes[2].legend(loc='upper right')

    # 4. 센서 유효성 플래그 (Valid Flags)
    axes[3].plot(df['time_sec'], df['valid_window_flag'], label='PPG Valid Flag', color='red', drawstyle='steps-post')
    axes[3].plot(df['time_sec'], df['gsr_valid_sample_flag'] * 0.9, label='GSR Valid Flag', color='blue', drawstyle='steps-post', alpha=0.7)
    axes[3].set_title('Sensor Valid Status (1 = Valid, 0 = Invalid)')
    axes[3].set_xlabel('Time (Seconds)')
    axes[3].set_ylabel('Flag Status')
    axes[3].set_yticks([0, 1])
    axes[3].legend(loc='upper right')

    plt.tight_layout()
    plt.show()

# 실행 예시 (저장된 CSV 파일 경로를 입력하세요)
# verify_biosignals('biogrip_data_log.csv')