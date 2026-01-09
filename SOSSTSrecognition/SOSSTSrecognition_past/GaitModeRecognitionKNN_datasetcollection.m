clear all;
close all;
clc;

% 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 1

%% H10 data load
cd 'C:\Users\ChanyoungKo\OneDrive - 한국과학기술원 기계공학과\바탕 화면\250829_refdataset_exp'

file_H10 = {'CYK_250829_STS_120spm.csv'};

% 데이터 저장용 struct
data_H10 = struct();

for idx = 1:length(file_H10)
    % CSV 읽기
    T = readtable(file_H10{idx});   

    T.Properties.VariableNames = { ...
        'loopCnt','assist_level', ...
        'thighDeg_RH','incPosDeg_RH','MotorActCurrent_RH','accX_Calib_RH','accY_Calib_RH','gyroZ_Calib_RH', ...
        'thighDeg_LH','incPosDeg_LH','MotorActCurrent_LH','accX_Calib_LH','accY_Calib_LH','gyroZ_Calib_LH', ...
        'u_RH','u_LH','Pvector_ref_RH','Pvector_ref_LH','extension_control_mode', ...
        'emg_R1','emg_L1','fsr_R1','fsr_R2','fsr_L1','fsr_L2','free_var1','free_var2'};

    % (옵션) 1 kHz 기준 시간 벡터 생성
    t = (0:height(T)-1).' * 0.001;  % 초 단위
    data_H10.t = t;

    % struct에 각 열 저장
    for p = 1:width(T)
        data_H10.(T.Properties.VariableNames{p}) = T{:, p};
    end
end

Rdeg = data_H10.thighDeg_RH;
Ldeg = data_H10.thighDeg_LH;

N = length(Rdeg);

%%
% ---- 파라미터 (원하면 자유롭게 변경) ----
fs         = 1000;
t_gap      = 400;
thres_up   = 10;
thres_down = 10;
var_crit   = 0;

scaling_X  = 1;
scaling_Y  = 1;

fc = 60;
Rdeg = LPF(Rdeg, fs, fc);
Ldeg = LPF(Ldeg, fs, fc);

% Rdeg_filtered = LPF(Rdeg, fs, fc);
% Ldeg_filtered = LPF(Ldeg, fs, fc);
% 
% figure; plot(t, Rdeg,'r'); hold on; plot(t, Rdeg_filtered,'r--'); hold on;

% [arb, hc_mask, feat_x, feat_y, idx_list] = compute_features_offline( ...
%     Rdeg, Ldeg, fs, t_gap, thres_up, thres_down, var_crit, scaling_X, scaling_Y);

% 함수 호출 예:
[arb, hc_mask, feat_x, feat_y, idx_list, swing_side, ...
 hc_R, hc_L, up_R, up_L, low_R, low_L] = compute_features_offline( ...
    Rdeg, Ldeg, fs, t_gap, thres_up, thres_down, var_crit, scaling_X, scaling_Y);


%% Labeling
figure('Name','수동 라벨링: Rdeg(빨강), Ldeg(파랑)');
plot(t, Rdeg(1:N), 'r'); hold on;
plot(t, Ldeg(1:N), 'b');
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
legend('Rdeg','Ldeg');
title('플롯 확인 후 라벨 벡터를 입력하세요 (1=STS, 2=SOS)');

M = length(feat_x);
label = prompt_labels(M);  % 여기서 호출


%% figure - HC labeling result
figure; plot(t, Rdeg,'r'); hold on; plot(t, Ldeg,'b'); hold on; plot(t, hc_mask*50, 'k', 'LineWidth', 3); grid on;
% hold on; plot(t, arb,'b')

%% figure - calculated indexes
Nlabel = length(label);
lab = label(1:Nlabel);
figure('Name','Features by Label'); hold on; grid on;
xlabel('feat\_x'); ylabel('feat\_y'); title('Feature scatter with labels');

idxSTS = (lab == 1);   % 빨강
idxSOS = (lab == 2);   % 파랑

h = gobjects(0); leg = {};

if any(idxSTS)
    h(end+1) = plot(feat_x(idxSTS), feat_y(idxSTS), 'ro', 'MarkerFaceColor','r', ...
        'DisplayName','STS'); %#ok<AGROW>
    leg{end+1} = 'STS'; %#ok<AGROW>
end
if any(idxSOS)
    h(end+1) = plot(feat_x(idxSOS), feat_y(idxSOS), 'bo', 'MarkerFaceColor','b', ...
        'DisplayName','SOS'); %#ok<AGROW>
    leg{end+1} = 'SOS'; %#ok<AGROW>
end

if ~isempty(h)
    legend(h, leg, 'Location','best');
end

% xlim([0 200]);
% ylim([0 1000]);

%% Check

figure; hold on; grid on;
plot(t, Rdeg, 'r', 'DisplayName','Rdeg');
plot(t, Ldeg, 'b', 'DisplayName','Ldeg');
xlabel('Time (s)'); ylabel('Angle (deg)');
title('HC / Upper / Lower (R/L 구분)');

% HC: Rswing/Lswing
if ~isempty(hc_R)
    plot(t(hc_R), Rdeg(hc_R), 'ms', 'MarkerFaceColor','m', 'DisplayName','HC (R swing)');
end
if ~isempty(hc_L)
    plot(t(hc_L), Ldeg(hc_L), 'gs', 'MarkerFaceColor','g', 'DisplayName','HC (L swing)');
end

% Upper peaks: R/L
if ~isempty(up_R)
    plot(t(up_R), Rdeg(up_R), 'r^', 'MarkerFaceColor','r', 'DisplayName','Upper (R)');
end
if ~isempty(up_L)
    plot(t(up_L), Ldeg(up_L), 'b^', 'MarkerFaceColor','b', 'DisplayName','Upper (L)');
end

% Lower peaks: R/L
if ~isempty(low_R)
    plot(t(low_R), Rdeg(low_R), 'rv', 'MarkerFaceColor','r', 'DisplayName','Lower (R)');
end
if ~isempty(low_L)
    plot(t(low_L), Ldeg(low_L), 'bv', 'MarkerFaceColor','b', 'DisplayName','Lower (L)');
end

legend('Location','best');

%% Reference outlier removal
ref_raw = [feat_x, feat_y, label];
[ref_clean, stats] = preprocess_ref_data(ref_raw);

%% Reference dataset save
% 확장자 제거하고 suffix 붙이기
% ref_data = extract_ref_data(ref_clean, 5);
% Option for stepmill data 5개 speed에 대해서 10개씩만 - gait mode별 총 50개

%% Reference dataset save
ref_data = ref_clean;
[~, baseName, ~] = fileparts(file_H10{1});
varName = [baseName '_refdataset'];     % 예: '250813_SOS_slowV_refdataset'
matFile = [varName '.mat'];             % 예: '250813_SOS_slowV_refdataset.mat'

% ref_data를 해당 변수명으로 저장
S.(varName) = ref_data;                 % 워크스페이스의 ref_data를 새 변수명으로 래핑
save(matFile, '-struct', 'S');          % 현재 경로에 저장

%% 모든 reference dataset combine
clear all; close all; clc;

cd 'C:\Users\ChanyoungKo\OneDrive - 한국과학기술원 기계공학과\바탕 화면\250829_refdataset_exp'

files = dir('*_refdataset.mat');
if isempty(files)
    error('현재 경로에 *_refdataset.mat 파일이 없습니다.');
end

% ===== 파라미터 =====
MAX_RUN = 50;                 % 1/2 연속 허용 최대 길이
TARGET_LABELS = [1 2];        % 규칙 적용 대상 레이블

% 파일 로드 + 세로 결합
ref_all = [];
for k = 1:numel(files)
    S = load(fullfile(files(k).folder, files(k).name));
    fn = fieldnames(S);
    A = [];
    for j = 1:numel(fn)
        val = S.(fn{j});
        if isnumeric(val) && ndims(val)==2 && size(val,2)==3
            A = val; break;
        end
    end
    if isempty(A)
        warning('건너뜀: %s (n×3 숫자배열 없음)', files(k).name);
        continue;
    end
    if ~isa(A,'double'); A = double(A); end
    ref_all = [ref_all; A]; %#ok<AGROW>
end
if isempty(ref_all), error('유효한 n×3 데이터가 없습니다.'); end

% 1) 고립된(길이 1) TARGET_LABELS 제거
g = ref_all(:,3);
N = numel(g);
edges = [1; find(diff(g)~=0)+1; N+1];
runStarts = edges(1:end-1);
runEnds   = edges(2:end)-1;
runLens   = runEnds - runStarts + 1;
runVals   = g(runStarts);

keep = true(N,1);
isTarget = ismember(runVals, TARGET_LABELS);
soloIdx  = find(isTarget & runLens==1);
for ii = 1:numel(soloIdx)
    s = runStarts(soloIdx(ii)); e = runEnds(soloIdx(ii));
    keep(s:e) = false;
end
ref_all = ref_all(keep,:);

% 2) (RLE 재계산) TARGET_LABELS 연속 길이 MAX_RUN 초과분 절단
g = ref_all(:,3);
N = numel(g);
edges = [1; find(diff(g)~=0)+1; N+1];
runStarts = edges(1:end-1);
runEnds   = edges(2:end)-1;
runLens   = runEnds - runStarts + 1;
runVals   = g(runStarts);

keep = true(N,1);
isTarget = ismember(runVals, TARGET_LABELS);
longIdx  = find(isTarget & runLens>MAX_RUN);
for ii = 1:numel(longIdx)
    s = runStarts(longIdx(ii));
    e = runEnds(longIdx(ii));
    remStart = s + MAX_RUN;         % (MAX_RUN+1)번째부터 제거
    keep(remStart:e) = false;
end
ref_all = ref_all(keep,:);

% (검증) 모든 TARGET_LABELS 런 길이 ≤ MAX_RUN 확인
g_chk = ref_all(:,3);
edges_chk = [1; find(diff(g_chk)~=0)+1; numel(g_chk)+1];
lens_chk  = diff(edges_chk);
vals_chk  = g_chk(edges_chk(1:end-1));
viol = find(ismember(vals_chk, TARGET_LABELS) & lens_chk>MAX_RUN);
if ~isempty(viol)
    warning('아직 %d 초과 런 존재 (시작 인덱스): %s', MAX_RUN, mat2str(edges_chk(viol)));
else
    fprintf('검증 통과: 레이블 %s의 모든 연속 구간 길이 ≤ %d\n', mat2str(TARGET_LABELS), MAX_RUN);
end

% 3) 스케일링 (필터 이후)
mx1 = max(ref_all(:,1)); if mx1==0, mx1=1; end
mx2 = max(ref_all(:,2)); if mx2==0, mx2=1; end
ref_all_scaled = [ref_all(:,1)/mx1, ref_all(:,2)/mx2, ref_all(:,3)];
scale_factor   = [mx1 mx2]

% 저장
save('refdataset_merged_scaled.mat', 'ref_all_scaled', 'scale_factor');
disp(size(ref_all_scaled));
disp('저장 완료: refdataset_merged_scaled.mat (변수명: ref_all_scaled)');

%% reference dataset 전부로 KNN model visualize
% ref_all_scaled 사용
x = ref_all_scaled(:,1);
y = ref_all_scaled(:,2);
g = ref_all_scaled(:,3);

N          = size(ref_all_scaled,1);
groupSize  = 50;               % 그룹 크기
subSize    = 10;               % 그룹을 5개로 나눔(10개씩)
numSub     = floor(groupSize / subSize);   % = 5
numGroups  = ceil(N / groupSize);          % 잔여 구간도 포함

% 색상 팔레트: 앞(1) -> 옅은색, 뒤(numSub) -> 진한색
baseRed  = [1 0 0];
baseBlue = [0 0 1];
white    = [1 1 1];
w = linspace(0.6, 0.0, numSub);   % 가중치(화이트 비율)

C = zeros(N,3);

for gi = 1:numGroups
    rStart = (gi-1)*groupSize + 1;
    rEnd   = min(gi*groupSize, N);

    if rStart > N, break; end

    label_g = g(rStart);
    if label_g == 1
        baseColor = baseRed;
    elseif label_g == 2
        baseColor = baseBlue;
    else
        baseColor = [0.4 0.4 0.4];   % 기타 레이블
    end

    for si = 1:numSub
        sStart = rStart + (si-1)*subSize;
        if sStart > rEnd, break; end
        sEnd   = min(sStart + subSize - 1, rEnd);

        % 옅은색 -> 진한색 (앞에서부터)
        Ci = baseColor*(1 - w(si)) + white*w(si);
        C(sStart:sEnd, :) = repmat(Ci, sEnd - sStart + 1, 1);
    end
end

% --- KNN 결정경계 그리기 ---
k = 9;
mdl = fitcknn([x y], g, 'NumNeighbors', k, 'Standardize', true);

mx = range(x); if mx==0, mx=1; end
my = range(y); if my==0, my=1; end
pad = 0.05;
xg = linspace(min(x)-pad*mx, max(x)+pad*mx, 400);
yg = linspace(min(y)-pad*my, max(y)+pad*my, 400);
[Xg, Yg] = meshgrid(xg, yg);

[predLbl, scores] = predict(mdl, [Xg(:) Yg(:)]);
Z = reshape(predLbl, size(Xg));

RGB = zeros([size(Z) 3]);
mask1 = (Z == 1);
mask2 = (Z == 2);
maskO = ~(mask1 | mask2);
for c = 1:3
    RGB(:,:,c) = baseRed(c)*mask1 + baseBlue(c)*mask2 + 0.9*white(c)*maskO;
end

cls = mdl.ClassNames;
idx1 = find(cls==1, 1);
idx2 = find(cls==2, 1);
D = nan(size(Xg));
if ~isempty(idx1) && ~isempty(idx2)
    Sd = reshape(scores(:,idx1) - scores(:,idx2), size(Xg));
    D = Sd;
end

figure;
hold on;

hImg = imagesc(xg, yg, RGB);
set(gca, 'YDir','normal');
set(hImg, 'AlphaData', 0.18);

if ~all(isnan(D), 'all')
    contour(Xg, Yg, D, [0 0], 'k--', 'LineWidth', 1.5);  % 경계선(점수가 0인 등고선)
end

scatter(x, y, 50, C, 'filled');

axis tight;
axis equal;
axis square;
axis off;
hold off;

disp('Red = STS (g=1), Blue = SOS (g=2)');

%% Function

function [arb, hc_mask, feat_x, feat_y, idx_list, swing_side, ...
          hc_idx_R, hc_idx_L, up_idx_R, up_idx_L, low_idx_R, low_idx_L] = ...
    compute_features_offline(Rdeg, Ldeg, fs, t_gap, thres_up, thres_down, var_crit, scaling_X, scaling_Y)

    N  = length(Rdeg);
    dt = 1/fs;

    % ---- 상태/카운터
    R_time_afterupFlag = 0;  L_time_afterupFlag = 0;  HC_time_afterFlag = 0;
    R_swing_time = 0;        L_swing_time = 0;

    R_time_upcond = false;   L_time_upcond = false;   HC_time_upcond = false;

    R_lowpeak_val = thres_down;
    L_lowpeak_val = thres_down;

    % (사용자가 넣었던 보조 변수)
    Rpeakval = 50; 
    Lpeakval = 50;

    % ---- 출력 버퍼
    arb        = zeros(N,1);
    hc_mask    = zeros(N,1);
    feat_x     = [];
    feat_y     = [];
    idx_list   = [];
    swing_side = [];          % 1=Rswing, 2=Lswing

    % 이벤트 인덱스(측면별)
    hc_idx_R = [];  hc_idx_L = [];
    up_idx_R = [];  up_idx_L = [];
    low_idx_R= [];  low_idx_L= [];

    % ---- 보류 중인 HC (상부피크에서 확정)
    pendR.valid = false; pendR.i = []; pendR.vel = []; pendR.T = [];
    pendL.valid = false; pendL.i = []; pendL.vel = []; pendL.T = [];

    for i = 3:N
        % 타이머 업데이트
        R_time_afterupFlag = R_time_afterupFlag + 1;
        L_time_afterupFlag = L_time_afterupFlag + 1;
        HC_time_afterFlag  = HC_time_afterFlag  + 1;
        R_swing_time       = R_swing_time + 1;
        L_swing_time       = L_swing_time + 1;

        % 3-샘플 윈도우
        R0 = Rdeg(i-2); R1 = Rdeg(i-1); R2 = Rdeg(i);
        L0 = Ldeg(i-2); L1 = Ldeg(i-1); L2 = Ldeg(i);

        % 재트리거 허용 조건
        if R_time_afterupFlag >= t_gap, R_time_upcond = true; end
        if L_time_afterupFlag >= t_gap, L_time_upcond = true; end
        if HC_time_afterFlag  >= t_gap, HC_time_upcond  = true; end

        % ================= HC 체크(보류 저장) =================
        if HC_time_upcond && ((R2 - L2)*(R1 - L1) <= 0) && (R2 > thres_down) && (L2 > thres_down)
            % 어느 쪽 스윙인지 분리(사용자가 쓰던 규칙 유지)
            if (R2 - R1) >= (L2 - L1) && Lpeakval > 40
                HC_time_upcond    = false;
                HC_time_afterFlag = 0;
                hc_mask(i) = 1;

                pendR.valid = true;
                pendR.i  = i;
                pendR.vel= (R2 - R1)/dt;      % deg/s
                pendR.T  = R_swing_time;
                hc_idx_R(end+1,1) = i;        % HC (Rswing)
            elseif (L2 - L1) >= (R2 - R1) && Rpeakval > 40
                HC_time_upcond    = false;
                HC_time_afterFlag = 0;
                hc_mask(i) = 1;

                pendL.valid = true;
                pendL.i  = i;
                pendL.vel= (L2 - L1)/dt;
                pendL.T  = L_swing_time;
                hc_idx_L(end+1,1) = i;        % HC (Lswing)
            end
        end

        % ============= Upper peak (상부 피크) =============
        % 오른쪽 상부피크
        if ((R2 - R1)*(R1 - R0) <= var_crit) && ((R2 - R1) < 0) && (R2 >= thres_up) && R_time_upcond && pendR.valid
            Rpeakval = R2;                         % 보조 변수 업데이트
            R_time_upcond       = false;
            R_time_afterupFlag  = 0;
            swing_period_R      = R_swing_time;    % 이번 오른쪽 swing의 period (lower→upper)
            up_idx_R(end+1,1)   = i;               % Upper (R)

            if ~isempty(pendR.vel) && swing_period_R > 0 && isfinite(pendR.vel)
                norm_vel_HC = (pendR.vel * (swing_period_R * dt)) / scaling_X;
                norm_T_HC   = (pendR.T   / (swing_period_R * dt)) / scaling_Y;

                feat_x(end+1,1)   = norm_vel_HC;
                feat_y(end+1,1)   = norm_T_HC;
                idx_list(end+1,1) = pendR.i;
                swing_side(end+1,1)= 1;        % Rswing
            end
            pendR.valid = false;
            pendR.i = []; pendR.vel = []; pendR.T = [];

            arb(i) = R_swing_time;                 % 디버그용
        end

        % 왼쪽 상부피크
        if ((L2 - L1)*(L1 - L0) <= var_crit) && ((L2 - L1) < 0) && (L2 >= thres_up) && L_time_upcond && pendL.valid
            Lpeakval = L2;
            L_time_upcond       = false;
            L_time_afterupFlag  = 0;
            swing_period_L      = L_swing_time;
            up_idx_L(end+1,1)   = i;               % Upper (L)

            if ~isempty(pendL.vel) && swing_period_L > 0 && isfinite(pendL.vel)
                norm_vel_HC = (pendL.vel * (swing_period_L * dt)) / scaling_X;
                norm_T_HC   = (pendL.T   / (swing_period_L * dt)) / scaling_Y;

                feat_x(end+1,1)   = norm_vel_HC;
                feat_y(end+1,1)   = norm_T_HC;
                idx_list(end+1,1) = pendL.i;
                swing_side(end+1,1)= 2;        % Lswing
            end
            pendL.valid = false;
            pendL.i = []; pendL.vel = []; pendL.T = [];

            arb(i) = L_swing_time;
        end

        % ============= Lower peak (하부 피크) =============
        % 오른쪽 하부피크 (사용자 조건 유지)
        if (R2 > R_lowpeak_val)
            R_lowpeak_val = thres_down;
        end
        if ((R2 - R1)*(R1 - R0) <= var_crit) && ((R2 - R1) >= 0) && (R2 <= R_lowpeak_val + 3) && ((R2 - R1) - (L2 - L1) > 0)
            R_lowpeak_val = R2;
            R_swing_time  = 0;                     % 새 스윙 시작
            low_idx_R(end+1,1) = i;                % Lower (R)
        end

        % 왼쪽 하부피크
        if (L2 > L_lowpeak_val)
            L_lowpeak_val = thres_down;
        end
        if ((L2 - L1)*(L1 - L0) <= var_crit) && ((L2 - L1) > 0) && (L2 <= L_lowpeak_val + 3) && ((R2 - R1) - (L2 - L1) < 0)
            L_lowpeak_val = L2;
            L_swing_time  = 0;                     % 새 스윙 시작
            low_idx_L(end+1,1) = i;                % Lower (L)
        end
    end
end

function ref_data_extracted = extract_ref_data(ref_data, num_segments)
% extract_ref_data : ref_data를 num_segments 등분한 후,
% 각 구간에서 7번째 ~ 16번째 데이터를 추출하여 반환
%
% 사용법:
%   ref_data_extracted = extract_ref_data(ref_data, 5)

    % 전체 행 개수
    N = size(ref_data,1);

    % num_segments 등분 구간 인덱스 (구간 경계)
    edges = floor(linspace(1, N+1, num_segments+1));

    % 결과 저장용
    ref_data_extracted = [];

    % 각 구간 반복
    for i = 1:num_segments
        % 구간 시작/끝 인덱스
        idx_start = edges(i);
        idx_end   = edges(i+1) - 1;

        % 구간 데이터
        segment = ref_data(idx_start:idx_end, :);

        % 추출할 구간 (7번째 ~ 16번째)
        row_start = 7;
        row_end   = min(16, size(segment,1));

        % 데이터가 존재할 경우에만 추출
        if row_start <= row_end
            ref_data_extracted = [ref_data_extracted; segment(row_start:row_end,:)];
        end
    end
end

function [ref_clean, stats] = preprocess_ref_data(ref_data, th, do_shuffle_within_group, rng_seed, save_path)
% preprocess_ref_data
%   ref_data를 전처리합니다.
%   1) NaN/Inf 제거
%   2) 라벨(3열) 검증: 1 또는 2
%   3) 라벨별 Z-Score 이상치 제거(|z|>th)
%   4) 라벨=1 → 라벨=2 순으로 이어붙이기
%   5) 저장(선택) 및 리포트 출력
%
% 사용법
%   [ref_clean, stats] = preprocess_ref_data(ref_data)
%   [ref_clean, stats] = preprocess_ref_data(ref_data, 3, true, 0, 'refdataset_scaled_cleaned_grouped.mat')
%
% 입력
%   ref_data (Nx3): [x y label_ref], label_ref ∈ {1,2}
%   th (double, opt): Z-score 임계치 (기본 3)
%   do_shuffle_within_group (logical, opt): 그룹내 셔플 (기본 true)
%   rng_seed (double, opt): rng seed (기본 0; 빈 값이면 설정 안 함)
%   save_path (char, opt): 저장 경로 (비우면 저장하지 않음)
%
% 출력
%   ref_clean (Mx3): 전처리 및 정렬 완료 [x y label_ref]
%   stats (struct 1x2): 각 라벨의 before/removed/after 등

    % 기본값
    if nargin < 2 || isempty(th), th = 3; end
    if nargin < 3 || isempty(do_shuffle_within_group), do_shuffle_within_group = true; end
    if nargin < 4, rng_seed = 0; end
    if nargin < 5, save_path = ''; end

    % 입력 점검
    if size(ref_data,2) < 3
        error('ref_data는 최소 3개의 열([x y label_ref])을 가져야 합니다.');
    end

    % 1) 기본 전처리: NaN/Inf 제거
    valid = all(isfinite(ref_data),2);
    if ~all(valid)
        warning('NaN/Inf %d개 행 제거', sum(~valid));
        ref_data = ref_data(valid,:);
    end

    % 라벨 추출 및 검증
    labels_ref = ref_data(:,3);
    if any(~ismember(labels_ref, [1 2]))
        error('라벨은 1 또는 2여야 합니다.');
    end

    % RNG 설정(선택)
    if ~isempty(rng_seed) && isnumeric(rng_seed)
        rng(rng_seed);
    end

    % 2) 라벨별 이상치 제거 (Z-score 기준 |z|>th 이면 제거)
    ref_clean_groups = cell(2,1);
    stats = struct('label_ref',[],'N_before',[],'N_removed',[],'N_after',[]);
    for L = 1:2
        idxL = (labels_ref == L);
        XY   = ref_data(idxL, 1:2);
        Nb   = size(XY,1);

        % 수동 zscore (분산 0 방지)
        mu = mean(XY,1);
        sg = std(XY,0,1);
        sg(sg==0) = eps;
        Z = (XY - mu) ./ sg;

        keep = all(abs(Z) <= th, 2);
        Na = sum(keep);
        Nr = Nb - Na;

        XYk = XY(keep,:);
        if do_shuffle_within_group && ~isempty(XYk)
            p = randperm(Na);
            XYk = XYk(p,:);
        end

        ref_clean_groups{L} = [XYk, L*ones(size(XYk,1),1)];

        stats(L).label_ref = L;
        stats(L).N_before  = Nb;
        stats(L).N_removed = Nr;
        stats(L).N_after   = Na;
    end

    % 3) 라벨=1 → 라벨=2 순으로 이어 붙이기
    ref_clean = vertcat(ref_clean_groups{:});  % [x y label_ref]

    % 4) 저장(선택) + 리포트
    if ~isempty(save_path)
        save(save_path, 'ref_clean', 'stats');
    end

    fprintf('=== Outlier 제거 결과 (|z|>%g) ===\n', th);
    for L = 1:2
        fprintf('Label_ref %d: before=%d, removed=%d, after=%d\n', ...
            stats(L).label_ref, stats(L).N_before, stats(L).N_removed, stats(L).N_after);
    end
    fprintf('최종 크기: %d x 3 (변수: ref_clean)\n', size(ref_clean,1));
end

function label = prompt_labels(M)
% prompt_labels : HC 라벨 벡터를 수동 입력하거나, 프리셋 버튼으로 자동 생성
%   label = prompt_labels(M)
%
% 입력
%   M : 라벨 벡터 길이 (HC 이벤트 개수)
%
% 출력
%   label (Mx1 double) : 라벨 벡터 (값=1 또는 2)

    % 안내 다이얼로그
    uiwait(msgbox( ...
        sprintf(['HC 이벤트 개수: %d개\n' ...
                 '라벨 벡터 길이도 %d이어야 합니다.\n' ...
                 '1=STS, 2=SOS\n' ...
                 '아래 창에서 직접 입력하거나, 프리셋 버튼을 누르세요.'], M, M), ...
        '안내','modal'));

    % 모달 대화상자
    d = dialog('Name','라벨 수동 입력/프리셋', 'WindowStyle','modal', ...
               'Position',[100 100 640 300]);

    uicontrol(d,'Style','text','String', ...
        sprintf('길이 %d의 label 벡터를 입력하세요 (예: 1 2 2 1 ... 또는 [1,2,2,1,...])', M), ...
        'HorizontalAlignment','left','Position',[20 255 600 30], 'FontSize',10);

    % 입력창
    hEdit = uicontrol(d,'Style','edit','Max',2,'Min',0, ... % 다중줄 허용
        'Position',[20 115 600 130],'HorizontalAlignment','left','FontName','Consolas', ...
        'TooltipString','여기에 직접 라벨을 입력하거나, 아래 프리셋 버튼을 클릭하세요.');

    % 프리셋 버튼
    btnW = 180; btnH = 30; gap = 10; y = 75;
    uicontrol(d,'Style','pushbutton','String','모두 1(STS)로 채우기', ...
        'Position',[20 y btnW btnH], 'Callback', @(~,~) setPreset(1));
    uicontrol(d,'Style','pushbutton','String','모두 2(SOS)로 채우기', ...
        'Position',[20+btnW+gap y btnW btnH], 'Callback', @(~,~) setPreset(2));
    uicontrol(d,'Style','pushbutton','String','처음/마지막=1, 중간=2', ...
        'Position',[20+2*(btnW+gap) y btnW btnH], 'Callback', @(~,~) setPreset(3));

    % 하단 버튼
    uicontrol(d,'Style','pushbutton','String','입력 내용 사용', ...
        'Position',[20 20 120 30], 'Callback', @(~,~) useTyped());
    uicontrol(d,'Style','pushbutton','String','취소', ...
        'Position',[160 20 80 30], 'Callback', @(~,~) cancelDialog());
    uicontrol(d,'Style','pushbutton','String','초기화', ...
        'Position',[250 20 80 30], 'Callback', @(~,~) set(hEdit,'String',''));

    % 내부 상태 저장
    setappdata(d,'label',[]);

    % 대기
    uiwait(d);
    if isvalid(d)
        label_tmp = getappdata(d,'label');
        delete(d);
    else
        label_tmp = [];
    end

    % 결과 반영
    if ~isempty(label_tmp)
        label = label_tmp;
    else
        label = ones(M,1); % fallback: 모두 1
    end

    % ----------------- Nested 함수 -----------------
    function setPreset(mode)
        switch mode
            case 1  % 모두 1
                L = ones(M,1);
            case 2  % 모두 2
                L = 2*ones(M,1);
            case 3  % 처음/마지막=1, 중간=2
                if M >= 2
                    L = 2*ones(M,1);
                    L(1) = 1; L(end) = 1;
                elseif M == 1
                    L = 1;
                else
                    L = zeros(0,1);
                end
        end
        setappdata(d,'label',L);
        uiresume(d);
    end

    function useTyped()
        raw = strtrim(get(hEdit,'String'));
        if isempty(raw)
            L = ones(M,1);
            setappdata(d,'label',L);
            uiresume(d);
            return;
        end
        raw_clean = regexprep(raw, '[\[\],;]', ' ');
        nums = sscanf(raw_clean, '%f');
        if numel(nums) ~= M
            uiwait(errordlg( ...
                sprintf('라벨 길이는 %d이어야 합니다. 현재 %d개가 입력되었습니다.', M, numel(nums)), ...
                '길이 불일치','modal'));
            return;
        end
        if any(~ismember(nums, [1 2]))
            choice = questdlg('1/2 이외의 값이 포함되어 있습니다. 그대로 사용할까요?', ...
                              '값 확인', '사용', '다시 입력', '다시 입력');
            if ~strcmp(choice,'사용')
                return;
            end
        end
        setappdata(d,'label',nums(:));
        uiresume(d);
    end

    function cancelDialog()
        choice = questdlg('입력을 취소했습니다. 어떻게 할까요?', ...
            '취소됨', '모두 1', '모두 2', '다시 입력', '모두 1');
        switch choice
            case '모두 1'
                setPreset(1);
            case '모두 2'
                setPreset(2);
            otherwise
                % 다시 입력 → 아무 것도 안 하고 대화상자 유지
        end
    end
end

function y = LPF(x, fs, fc)
    y = zeros(length(x),1);
    % Define the filter parameters
    T = 1/fs;
    RC = 1/(2*pi*fc);
    a = T / (T + RC);
    for i = 2 : length(x)
        y(i) = (1-a) * y(i-1) + a * x(i);
    end
end