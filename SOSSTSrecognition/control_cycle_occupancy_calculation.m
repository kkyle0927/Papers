clear all;
close all;
clc;

%% H10 data load (여기에 여러 파일명을 넣으세요)
cd 'C:\Users\ChanyoungKo\OneDrive - 한국과학기술원 기계공학과\바탕 화면\'
file_H10 = { ...
    'SUIT_LOGGED_DATA-1-decoded.csv', ...
    'SUIT_LOGGED_DATA-2-decoded.csv', ...
    'SUIT_LOGGED_DATA-3-decoded.csv'};

% 결과 저장용
per_file_names = strings(numel(file_H10),1);
per_file_N     = zeros(numel(file_H10),1);
per_file_mean  = nan(numel(file_H10),1);
per_file_std   = nan(numel(file_H10),1);
per_file_max   = nan(numel(file_H10),1);

all_vals = [];  % 전체(통합) 통계용 누적 벡터 (조건 통과한 free_var1 값만 순서대로 저장)

for f = 1:numel(file_H10)
    % ====== CSV 읽기 & 변수명 세팅 ======
    T = readtable(file_H10{f});
    T.Properties.VariableNames = { ...
        'loopCnt','assist_level', ...
        'thighDeg_RH','incPosDeg_RH','MotorActCurrent_RH','accX_Calib_RH','accY_Calib_RH','gyroZ_Calib_RH', ...
        'thighDeg_LH','incPosDeg_LH','MotorActCurrent_LH','accX_Calib_LH','accY_Calib_LH','gyroZ_Calib_LH', ...
        'u_RH','u_LH','Pvector_ref_RH','Pvector_ref_LH','extension_control_mode', ...
        'emg_R1','emg_L1','fsr_R1','fsr_R2','fsr_L1','fsr_L2','free_var1','free_var2'};

    % (옵션) 시간 벡터 (1 kHz 가정)
    t = (0:height(T)-1).' * 0.001; %#ok<NASGU>

    % ====== 조건 계산 ======
    % 기준: free_var2 >= 0.5 인 인덱스들의 "한 tick 뒤" free_var1 값을 사용
    idx = find(T.free_var2 >= 0.5);

    % 한 tick 뒤 인덱스가 유효한 경우만 남김 (idx+1 <= height(T))
    idx = idx(idx < height(T));

    % 한 tick 뒤의 free_var1 값
    vals1ahead = T.free_var1(idx + 1);

    % 그 값들 중 42 이상만 선택
    vals = vals1ahead(vals1ahead >= 42);

    % ====== 파일별 통계 ======
    per_file_names(f) = string(file_H10{f});
    if isempty(vals)
        per_file_N(f)    = 0;
        per_file_mean(f) = NaN;
        per_file_std(f)  = NaN;
        per_file_max(f)  = NaN;
    else
        per_file_N(f)    = numel(vals);
        per_file_mean(f) = mean(vals, 'omitnan');
        per_file_std(f)  = std(vals,  'omitnan');   % N-1 기준
        per_file_max(f)  = max(vals);
        % 통합용 누적
        all_vals = [all_vals; vals]; %#ok<AGROW>
    end

    % ====== (선택) 파일별 플롯이 필요하면 아래 주석 해제 ======
    %{
    figure('Name', file_H10{f});
    subplot(3,1,1);
    plot(t, T.thighDeg_RH,'r'); hold on;
    plot(t, T.thighDeg_LH,'b'); hold on; grid on;
    plot(t, T.free_var2*100,'k'); grid on;
    legend('R Thigh deg', 'L Thigh deg', 'Assistance trigger moment');

    subplot(3,1,2);
    plot(t, T.free_var1,'k'); grid on;
    legend('Resource (us)');

    subplot(3,1,3);
    plot(t, T.MotorActCurrent_RH,'r'); hold on;
    plot(t, T.MotorActCurrent_LH,'b'); grid on;
    legend('R Motor Current', 'L Motor Current');
    %}
end

%% 파일별 결과 테이블 출력
per_file_table = table(per_file_names, per_file_N, per_file_mean, per_file_std, per_file_max, ...
    'VariableNames', {'filename','N_samples','mean_val','std_val','max_val'});
disp('=== Per-file results ===');
disp(per_file_table);

%% 전체(통합) 결과 - 앞 600개만 사용
if isempty(all_vals)
    N_total   = 0;
    mean_all  = NaN;
    std_all   = NaN;
    max_all   = NaN;
else
    head_vals = all_vals(1:min(600, numel(all_vals)));  % 앞 600개 (데이터가 600개 미만이면 전체)
    N_total   = numel(head_vals);
    mean_all  = mean(head_vals, 'omitnan');
    std_all   = std(head_vals,  'omitnan');
    max_all   = max(head_vals);
end

overall_table = table(N_total, mean_all, std_all, max_all, ...
    'VariableNames', {'N_first600','mean_val','std_val','max_val'});
disp('=== Overall (first 600 samples only) results ===');
disp(overall_table);
