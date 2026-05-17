clear; clc; close all;

[file, path] = uigetfile('*.csv', 'GSR CSV 파일 선택');
if isequal(file, 0)
    error('파일이 선택되지 않았습니다.');
end

filename = fullfile(path, file);
data = readmatrix(filename);

if size(data, 2) < 3
    error('CSV 파일은 최소 3개 열이 필요합니다. (time_s, gsr_raw, gsr_voltage)');
end

t = data(:,1);
raw = data(:,2);
v = data(:,3);

validIdx = isfinite(t) & isfinite(raw) & isfinite(v);
t = t(validIdx);
raw = raw(validIdx);
v = v(validIdx);

if numel(t) < 10
    error('데이터 개수가 너무 적습니다.');
end

[t, sortIdx] = sort(t);
raw = raw(sortIdx);
v = v(sortIdx);

[t, uniqueIdx] = unique(t, 'stable');
raw = raw(uniqueIdx);
v = v(uniqueIdx);

t = t - t(1);

dt = diff(t);
dt = dt(dt > 0);

if isempty(dt)
    error('시간 데이터가 올바르지 않습니다.');
end

fs = 1 / mean(dt);

fprintf('Estimated Sampling Rate : %.3f Hz\n', fs);
fprintf('Duration                : %.3f s\n', t(end));
fprintf('Number of Samples       : %d\n', numel(t));

smoothWinSec = 0.5;
smoothWin = max(3, round(fs * smoothWinSec));
if mod(smoothWin, 2) == 0
    smoothWin = smoothWin + 1;
end
vSmooth = movmean(v, smoothWin);

tonicWinSec = 5.0;
tonicWin = max(3, round(fs * tonicWinSec));
if mod(tonicWin, 2) == 0
    tonicWin = tonicWin + 1;
end
tonic = movmean(vSmooth, tonicWin);
phasic = vSmooth - tonic;

minPeakDistanceSec = 1.0;
minPeakDistanceSamples = max(1, round(fs * minPeakDistanceSec));
minPeakProminence = 0.002;

[peakVals, peakLocs, peakWidths, peakProms] = findpeaks(phasic, ...
    'MinPeakDistance', minPeakDistanceSamples, ...
    'MinPeakProminence', minPeakProminence);

peakTimes = t(peakLocs);

scrAmpThreshold = 0.05;
validSCR = peakVals > scrAmpThreshold;

scrPeakVals = peakVals(validSCR);
scrPeakLocs = peakLocs(validSCR);
scrPeakTimes = t(scrPeakLocs);

riseTimes = [];
onsetLocs = [];

for k = 1:numel(scrPeakLocs)
    p = scrPeakLocs(k);
    leftBound = max(1, p - round(5 * fs));
    seg = phasic(leftBound:p);
    [~, localMinIdx] = min(seg);
    onset = leftBound + localMinIdx - 1;
    onsetLocs(end+1,1) = onset;
    riseTimes(end+1,1) = t(p) - t(onset);
end

durationMin = t(end) / 60;

if durationMin > 0
    NSSCR_Frequency = numel(scrPeakVals) / durationMin;
else
    NSSCR_Frequency = 0;
end

if ~isempty(scrPeakVals)
    SCR_Amplitude = mean(scrPeakVals);
    SCR_Rise_Time = mean(riseTimes);
else
    SCR_Amplitude = 0;
    SCR_Rise_Time = 0;
end

SCL_Mean = mean(tonic);

xPSD = detrend(vSmooth, 'linear');
N = numel(xPSD);
windowLength = min(128, N);
if mod(windowLength, 2) ~= 0
    windowLength = windowLength - 1;
end
windowLength = max(windowLength, 8);
noverlap = floor(windowLength / 2);
nfft = max(256, 2^nextpow2(N));

[pxx, f] = pwelch(xPSD, blackman(windowLength), noverlap, nfft, fs);

edaBandIdx = (f >= 0.045) & (f <= 0.25);
if any(edaBandIdx)
    EDASymp = trapz(f(edaBandIdx), pxx(edaBandIdx));
else
    EDASymp = 0;
end

fsTV = 2;
t2 = (0:1/fsTV:t(end))';
v2 = interp1(t, vSmooth, t2, 'linear', 'extrap');

bpLow = 0.08;
bpHigh = 0.24;

if fsTV <= 2 * bpHigh
    TVSymp = NaN;
    tvBand = nan(size(v2));
    tvAmp = nan(size(v2));
else
    [b, a] = butter(4, [bpLow bpHigh] / (fsTV / 2), 'bandpass');
    tvBand = filtfilt(b, a, detrend(v2, 'linear'));
    tvAmp = abs(hilbert(tvBand));
    TVSymp = mean(tvAmp);
end

fprintf('\n=== Requested GSR Metrics ===\n');
fprintf('NS-SCR Frequency : %.4f peaks/min\n', NSSCR_Frequency);
fprintf('SCR Amplitude    : %.6f uS-equivalent\n', SCR_Amplitude);
fprintf('SCR Rise Time    : %.6f s\n', SCR_Rise_Time);
fprintf('SCL Mean         : %.6f V\n', SCL_Mean);
fprintf('EDASymp          : %.8f\n', EDASymp);
fprintf('TVSymp           : %.8f\n', TVSymp);

Feature = {
    'NS-SCR Frequency'
    'SCR Amplitude'
    'SCR Rise Time'
    'SCL Mean'
    'EDASymp'
    'TVSymp'
    };

Value = [
    NSSCR_Frequency
    SCR_Amplitude
    SCR_Rise_Time
    SCL_Mean
    EDASymp
    TVSymp
    ];

Unit = {
    'peaks/min'
    'V'
    's'
    'V'
    'V^2/Hz-band'
    'a.u.'
    };

ResultTable = table(Feature, Value, Unit);

disp(' ');
disp('=== Result Table ===');
disp(ResultTable);

writetable(ResultTable, 'GSR_Selected_Metrics.csv');
fprintf('\n결과가 GSR_Selected_Metrics.csv 로 저장되었습니다.\n');

figure('Name', 'GSR Selected Metrics Analysis', 'Color', 'w');

subplot(4,1,1);
plot(t, v, 'LineWidth', 1.0); hold on;
plot(t, vSmooth, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Voltage (V)');
title('GSR Signal');
legend('Original', 'Smoothed');

subplot(4,1,2);
plot(t, tonic, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Voltage (V)');
title('Tonic Component');

subplot(4,1,3);
plot(t, phasic, 'LineWidth', 1.2); hold on;
plot(scrPeakTimes, scrPeakVals, 'rv', 'MarkerFaceColor', 'r');
if ~isempty(onsetLocs)
    plot(t(onsetLocs), phasic(onsetLocs), 'ko', 'MarkerFaceColor', 'y');
end
grid on;
xlabel('Time (s)');
ylabel('Voltage (V)');
title('Phasic Component with SCR Peaks');
legend('Phasic', 'SCR Peaks', 'SCR Onsets');

subplot(4,1,4);
if all(isnan(tvAmp))
    plot(t2, zeros(size(t2)), 'LineWidth', 1.2);
    title('TVSymp Approximation Not Available');
else
    plot(t2, tvAmp, 'LineWidth', 1.2);
    title('TVSymp Approximation');
end
grid on;
xlabel('Time (s)');
ylabel('Amplitude');
