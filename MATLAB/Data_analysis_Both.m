clc; clear; close all;

%% =========================
% 1. 데이터 로드
%% =========================
pickle = py.importlib.import_module('pickle');

file = py.open('S2.pkl', 'rb');
data = pickle.load(file, pyargs('encoding','latin1'));
file.close();

signal = data{'signal'};
wrist = signal{'wrist'};

ppg = double(wrist{'BVP'});
gsr = double(wrist{'EDA'});
label = double(data{'label'});

%% =========================
% 2. 샘플링
%% =========================
fs_ppg = 64;
fs_gsr = 4;
fs_label = 700;

t_ppg = (0:length(ppg)-1)/fs_ppg;
t_gsr = (0:length(gsr)-1)/fs_gsr;
t_label = (0:length(label)-1)/fs_label;

label_ppg = interp1(t_label, label, t_ppg, 'nearest');

%% =========================
% 3. 분석 함수 (안정 구간)
%% =========================
function results = analyze_stable(ppg, gsr, t_ppg, t_gsr, label_ppg, target_label, offset, fs_ppg, fs_gsr)

    idx_all = (label_ppg == target_label);
    t_all = t_ppg(idx_all);

    t_start = t_all(1) + offset;
    t_end = t_start + 60;

    idx_ppg = (t_ppg >= t_start) & (t_ppg <= t_end);
    idx_gsr = (t_gsr >= t_start) & (t_gsr <= t_end);

    ppg_seg = ppg(idx_ppg);
    gsr_seg = gsr(idx_gsr);

    t_ppg_seg = t_ppg(idx_ppg) - t_start;
    t_gsr_seg = t_gsr(idx_gsr) - t_start;

    %% PPG
    ppg_f = bandpass(ppg_seg, [0.5 5], fs_ppg);

    [peaks, locs] = findpeaks(ppg_f, ...
        'MinPeakDistance', round(fs_ppg*0.5));

    RR = diff(locs)/fs_ppg;
    RR_ms = RR * 1000;

    HR = 60 ./ RR;

    Mean_HR = mean(HR);
    SDNN = std(RR_ms);
    RMSSD = sqrt(mean(diff(RR_ms).^2));

    %% GSR
    gsr_f = lowpass(gsr_seg, 0.5, fs_gsr);
    gsr_detrend = detrend(gsr_f);

    [gsr_peaks, gsr_locs] = findpeaks(gsr_detrend, ...
        'MinPeakProminence', 0.001);

    SCR_count = length(gsr_peaks);

    baseline = movmean(gsr_f, 5);

    if SCR_count > 0
        SCR_amp = mean(gsr_f(gsr_locs) - baseline(gsr_locs));
    else
        SCR_amp = 0;
    end

    SCL = mean(gsr_f);

    results = struct('t_ppg', t_ppg_seg, 't_gsr', t_gsr_seg, ...
        'ppg_f', ppg_f, 'gsr_f', gsr_f, ...
        'locs', locs, 'peaks', peaks, ...
        'gsr_locs', gsr_locs, ...
        'Mean_HR', Mean_HR, ...
        'SDNN', SDNN, ...
        'RMSSD', RMSSD, ...
        'SCL', SCL, ...
        'SCR_count', SCR_count, ...
        'SCR_amp', SCR_amp);
end

%% =========================
% 4. 분석 실행
%% =========================

stress = analyze_stable(ppg, gsr, t_ppg, t_gsr, label_ppg, 2, 60, fs_ppg, fs_gsr);
meditation = analyze_stable(ppg, gsr, t_ppg, t_gsr, label_ppg, 4, 30, fs_ppg, fs_gsr);

%% =========================
% 5. 결과 출력
%% =========================
fprintf('\n=== 안정 구간 결과 ===\n');

fprintf('\n[Stress]\n');
fprintf('Mean HR: %.2f bpm\n', stress.Mean_HR);
fprintf('SDNN: %.2f ms\n', stress.SDNN);
fprintf('RMSSD: %.2f ms\n', stress.RMSSD);
fprintf('SCL: %.4f µS\n', stress.SCL);
fprintf('SCR Count: %d\n', stress.SCR_count);
fprintf('SCR Amp: %.4f µS\n', stress.SCR_amp);

fprintf('\n[Meditation]\n');
fprintf('Mean HR: %.2f bpm\n', meditation.Mean_HR);
fprintf('SDNN: %.2f ms\n', meditation.SDNN);
fprintf('RMSSD: %.2f ms\n', meditation.RMSSD);
fprintf('SCL: %.4f µS\n', meditation.SCL);
fprintf('SCR Count: %d\n', meditation.SCR_count);
fprintf('SCR Amp: %.4f µS\n', meditation.SCR_amp);

%% =========================
% 6. 그래프 (단위 포함)
%% =========================
figure;

% PPG - Stress
subplot(2,2,1);
plot(stress.t_ppg, stress.ppg_f); hold on;
plot(stress.t_ppg(stress.locs), stress.peaks,'r*');
title('PPG - Stress (Stable)');
xlabel('Time (s)');
ylabel('Amplitude (a.u.)'); % arbitrary unit

% PPG - Meditation
subplot(2,2,2);
plot(meditation.t_ppg, meditation.ppg_f); hold on;
plot(meditation.t_ppg(meditation.locs), meditation.peaks,'r*');
title('PPG - Meditation (Stable)');
xlabel('Time (s)');
ylabel('Amplitude (a.u.)');

% GSR - Stress
subplot(2,2,3);
plot(stress.t_gsr, stress.gsr_f, 'k'); hold on;
scatter(stress.t_gsr(stress.gsr_locs), ...
        stress.gsr_f(stress.gsr_locs), ...
        60, 'r', 'filled');
title('GSR - Stress (Stable)');
xlabel('Time (s)');
ylabel('Conductance (µS)');

% GSR - Meditation
subplot(2,2,4);
plot(meditation.t_gsr, meditation.gsr_f, 'k'); hold on;
scatter(meditation.t_gsr(meditation.gsr_locs), ...
        meditation.gsr_f(meditation.gsr_locs), ...
        60, 'b', 'filled');
title('GSR - Meditation (Stable)');
xlabel('Time (s)');
ylabel('Conductance (µS)');

sgtitle('Stable State Comparison (Stress vs Meditation)');
