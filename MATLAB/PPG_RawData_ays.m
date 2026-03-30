clc; clear; close all;

%% 1. CSV 불러오기 (안전하게)
data = readmatrix('5.csv');

% NaN 제거
data = data(~any(isnan(data),2), :);

t = data(:,1) / 1000;   % ms → s
ir = data(:,2);

%% 2. Sampling Rate 강제 계산
dt = diff(t);
dt = dt(dt > 0);  % 이상값 제거

fs = 1 / mean(dt);

fprintf('Sampling Rate: %.2f Hz\n', fs);

%% ❗ fs가 이상하면 강제 지정
if isnan(fs) || fs < 10
    fs = 400;  % 너 설정값
    fprintf('Sampling Rate manually set to 400 Hz\n');
end

%% 3. DC 제거
ir_dc_removed = ir - movmean(ir, round(fs));

%% 4. 필터
[b, a] = butter(3, [0.5 4] / (fs/2), 'bandpass');
ir_filtered = filtfilt(b, a, ir_dc_removed);

%% 5. Peak 검출
[peaks, locs] = findpeaks(ir_filtered, ...
    'MinPeakHeight', 0.3 * max(ir_filtered), ...
    'MinPeakDistance', round(0.4 * fs));

peak_times = t(locs);

%% 6. IBI
ibi = diff(peak_times) * 1000;

%% 7. HRV
SDNN = std(ibi);
RMSSD = sqrt(mean(diff(ibi).^2));
HR = 60 ./ (ibi / 1000);

fprintf('\n=== HRV RESULT ===\n');
fprintf('Mean HR: %.2f bpm\n', mean(HR));
fprintf('SDNN: %.2f ms\n', SDNN);
fprintf('RMSSD: %.2f ms\n', RMSSD);

%% 8. Plot
figure;
subplot(2,1,1);
plot(t, ir);
title('Raw');

subplot(2,1,2);
plot(t, ir_filtered);
hold on;
plot(t(locs), peaks, 'ro');
title('Filtered + Peaks');
