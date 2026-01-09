clear all;
close all;
clc;

% 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 1

%% H10 data load
% cd 'C:\Users\ChanyoungKo\OneDrive - 한국과학기술원 기계공학과\바탕 화면\250829_CYK_validation_exp'
cd 'C:\Users\ChanyoungKo\OneDrive - 한국과학기술원 기계공학과\바탕 화면\250829_PJS_validation_exp'
file_H10 = {'PJS_250829_SOS_selfpaced_validation1.csv'};

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

%% Ref_xy load and process
load('C:\Users\ChanyoungKo\OneDrive - 한국과학기술원 기계공학과\바탕 화면\250829_refdataset_exp\refdataset_merged_scaled.mat');

%%
% ---- 파라미터 (원하면 자유롭게 변경) ----
fs         = 1000;
t_gap      = 300;
thres_up   = 10;
thres_down = 10;
var_crit   = 0;

fc = 60;
Rdeg = LPF(Rdeg, fs, fc);
Ldeg = LPF(Ldeg, fs, fc);

% [hc_mask, feat_x, feat_y, idx_list] = compute_features_RT( ...
%     Rdeg, Ldeg, fs, t_gap, thres_up, thres_down, var_crit, scaling_X, scaling_Y);

[hc_mask, feat_x, feat_y, idx_list, hc_idx_R, hc_idx_L, up_idx_R, up_idx_L, low_idx_R, low_idx_L] = ...
    compute_features_RT(Rdeg, Ldeg, fs, t_gap, thres_up, thres_down, var_crit, 1, 1);

%% Labeling
figure('Name','수동 라벨링: Rdeg(빨강), Ldeg(파랑)');
plot(t, Rdeg(1:N), 'r'); hold on;
plot(t, Ldeg(1:N), 'b');
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
legend('Rdeg','Ldeg');
title('플롯 확인 후 라벨 벡터를 입력하세요 (1=STS, 2=SOS)');

% 라벨 개수는 HC 특징 개수와 동일해야 함
M = length(feat_x);

uiwait(msgbox( ...
    sprintf(['HC 이벤트 개수: %d개\n' ...
             '라벨 벡터 길이도 %d이어야 합니다.\n' ...
             '1=STS, 2=SOS\n' ...
             '(빈칸이면 모두 1(STS)로 자동 채움)'], M, M), ...
    '안내','modal'));

label = [];  % 초기화

while true
    prompt   = {sprintf('길이 %d의 label 벡터를 입력하세요 (예: 1 2 2 1 ... 또는 [1,2,2,1,...])', M)};
    dlgtitle = '라벨 수동 입력';
    definput = {''};  % 빈칸이면 모두 1로 처리
    answer = inputdlg(prompt, dlgtitle, [1 80], definput);

    % 사용자가 창을 닫거나 취소한 경우
    if isempty(answer)
        choice = questdlg('입력을 취소했습니다. 모두 1(STS)로 채울까요?', ...
                          '취소됨', '모두 1로', '다시 입력', '모두 1로');
        if strcmp(choice,'모두 1로')
            label = ones(M,1);
            break;
        else
            continue; % 다시 입력
        end
    end

    raw = strtrim(answer{1});

    % 빈칸이면 모두 1로 자동 채움
    if isempty(raw)
        label = ones(M,1);
        break;
    end

    % 다양한 포맷 허용: [], , ; 제거 후 공백 기준 파싱
    raw_clean = regexprep(raw, '[\[\],;]', ' ');
    nums = sscanf(raw_clean, '%f');

    % 길이 검증
    if numel(nums) ~= M
        uiwait(errordlg( ...
            sprintf('라벨 길이는 %d이어야 합니다. 현재 %d개가 입력되었습니다.', M, numel(nums)), ...
            '길이 불일치','modal'));
        continue;
    end

    % 값 검증(원하면 유지)
    if any(~ismember(nums, [1 2]))
        choice = questdlg('1/2 이외의 값이 포함되어 있습니다. 그대로 사용할까요?', ...
                          '값 확인', '사용', '다시 입력', '다시 입력');
        if ~strcmp(choice,'사용')
            continue;
        end
    end

    label = nums(:);  % 열벡터로 정리
    break;
end

%% figure - HC labeling result
figure; plot(t, Rdeg,'r'); hold on; plot(t, Ldeg,'b'); hold on; plot(t, hc_mask*50, 'k'); grid on;

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
% === RT 이벤트 마커 플롯 (R/L 구분) ===
figure; hold on; grid on;
plot(t, Rdeg, 'r', 'DisplayName','Rdeg');
plot(t, Ldeg, 'b', 'DisplayName','Ldeg');
xlabel('Time (s)'); ylabel('Angle (deg)');
title('RT: HC / Upper / Lower (R/L separated)');

% HC (R/L)
if ~isempty(hc_idx_R)
    plot(t(hc_idx_R), Rdeg(hc_idx_R), 'ms', 'MarkerFaceColor','m', 'DisplayName','HC (R swing)');
end
if ~isempty(hc_idx_L)
    plot(t(hc_idx_L), Ldeg(hc_idx_L), 'gs', 'MarkerFaceColor','g', 'DisplayName','HC (L swing)');
end

% Upper (R/L)
if ~isempty(up_idx_R)
    plot(t(up_idx_R), Rdeg(up_idx_R), 'r^', 'MarkerFaceColor','r', 'DisplayName','Upper (R)');
end
if ~isempty(up_idx_L)
    plot(t(up_idx_L), Ldeg(up_idx_L), 'b^', 'MarkerFaceColor','b', 'DisplayName','Upper (L)');
end

% Lower (R/L)
if ~isempty(low_idx_R)
    plot(t(low_idx_R), Rdeg(low_idx_R), 'rv', 'MarkerFaceColor','r', 'DisplayName','Lower (R)');
end
if ~isempty(low_idx_L)
    plot(t(low_idx_L), Ldeg(low_idx_L), 'bv', 'MarkerFaceColor','b', 'DisplayName','Lower (L)');
end

legend('Location','best');

%% KNN algorithm
test_dataset_knn = [feat_x / scale_factor(1) feat_y / scale_factor(2) label];
ref_dataset_knn = ref_all_scaled;

k = 9;         % K 값 (원하면 조절)
grid_res = 250; % 결정영역 해상도 (200~400 권장)

[pred, acc, CM] = plot_knn_regions(ref_dataset_knn, test_dataset_knn, k, grid_res);
fprintf('K=%d | Accuracy = %.2f%%\n', k, acc*100);
disp('Confusion Matrix (rows=true, cols=pred):'); disp(CM);

%% KNN algorithm result visualization (filled 0~1 range, no white near boundary)
% 입력:
%   ref_all_scaled    = [x y label]  % reference dataset (학습용)
%   test_dataset_knn  = [x y label]  % test dataset (평가/표시)
%   k (optional)                     % K 값 (없으면 9)

% 1) 데이터 분리
rx = ref_all_scaled(:,1); ry = ref_all_scaled(:,2); rlab = ref_all_scaled(:,3);
tx = test_dataset_knn(:,1); ty = test_dataset_knn(:,2); tlab = test_dataset_knn(:,3);

% 2) KNN 학습 (reference만 사용)
if ~exist('k','var') || isempty(k), k = 9; end
mdl_ref = fitcknn([rx ry], rlab, 'NumNeighbors', k, 'Standardize', true);

% 3) test dataset 분류 결과/지표
pred_test = predict(mdl_ref, [tx ty]);
acc_test  = mean(pred_test == tlab);
CM_test   = zeros(2,2);
CM_test(1,1) = sum(tlab==1 & pred_test==1);
CM_test(1,2) = sum(tlab==1 & pred_test==2);
CM_test(2,1) = sum(tlab==2 & pred_test==1);
CM_test(2,2) = sum(tlab==2 & pred_test==2);
err = (pred_test ~= tlab);

fprintf('K=%d | Test Accuracy = %.2f%%\n', k, 100*acc_test);
disp('Confusion Matrix (rows=true, cols=pred):'); disp(CM_test);

% 4) 배경 그리드: **좌표 범위를 0~1로 고정** (원하신 축 범위)
nx = 400; ny = 400;
xv = linspace(0, 1, nx);           % grid "중심"
yv = linspace(0, 1, ny);
[Xg, Yg] = meshgrid(xv, yv);

[pred_grid, scores_grid] = predict(mdl_ref, [Xg(:) Yg(:)]);
Z = reshape(pred_grid, size(Xg));

% 5) 배경색 구성 (흰색 혼합 제거) — 클래스별 단색
baseRed  = [1 0 0];
baseBlue = [0 0 1];
RGB = zeros([size(Z) 3]);
mask1 = (Z == 1); 
mask2 = (Z == 2);
for c = 1:3
    RGB(:,:,c) = baseRed(c)*mask1 + baseBlue(c)*mask2; % 흰색(maskO) 제거!
end

% 6) 배경 렌더 — image로 “엣지”를 [0 1]에 정확히 맞춰서 꽉 채움
figure('Name', sprintf('KNN Result (K=%d): filled 0-1, no white near boundary', k)); hold on;
hImg = image('XData', [0 1], 'YData', [0 1], 'CData', RGB);
set(gca, 'YDir', 'normal');

% 배경 투명도: 상수(경계 부근도 동일하게 칠해져 하얀 띠 안 생김)
set(hImg, 'AlphaData', 0.20);

% 7) 결정경계(두 클래스 점수 동일선) — 선만 표시(채움 없음)
hCt = gobjects(1);
if size(scores_grid,2) >= 2
    Sd = reshape(scores_grid(:,1) - scores_grid(:,2), size(Xg));
    [~, hCt] = contour(Xg, Yg, Sd, [0 0], 'k--', 'LineWidth', 1.0);
end

% 8) reference 점들: 단일 회색, 매우 투명 (학습 샘플 존재만 암시)
refColor = [0.5 0.5 0.5];
hRef = scatter(rx, ry, 14, refColor, 'filled', ...
    'MarkerFaceAlpha', 0.12, 'MarkerEdgeAlpha', 0.12);

% 9) test 점들: 라벨별 선명(정답 라벨 기준 색상)
idxSTS = (tlab==1); idxSOS = (tlab==2);
hSTS = gobjects(1); hSOS = gobjects(1);
if any(idxSTS)
    hSTS = scatter(tx(idxSTS), ty(idxSTS), 48, 'r', 'filled', ...
        'MarkerEdgeColor','k', 'LineWidth',0.5);
end
if any(idxSOS)
    hSOS = scatter(tx(idxSOS), ty(idxSOS), 48, 'b', 'filled', ...
        'MarkerEdgeColor','k', 'LineWidth',0.5);
end

% 10) 오분류는 X 표시
hErr = gobjects(1);
if any(err)
    hErr = plot(tx(err), ty(err), 'kx', 'MarkerSize', 8, 'LineWidth', 1.4);
end

% 11) 축/제목/범례
xlim([0 1]); ylim([0 1]);              % 요청하신 축 범위 고정
grid on; box on;
xlabel('x'); ylabel('y');
title(sprintf('K=%d | Test Acc = %.2f%%', k, 100*acc_test));

axis equal;      % <-- x축: y축 비율을 1:1로 맞춤
% axis square;   % (옵션) 정사각형으로 맞추고 싶을 때

% 안전한 핸들 기반 범례
legH = []; legN = {};
if isgraphics(hCt),  legH(end+1) = hCt;  legN{end+1} = 'Decision boundary'; end %#ok<AGROW>
if isgraphics(hRef), legH(end+1) = hRef; legN{end+1} = 'Reference (faint)'; end %#ok<AGROW>
if isgraphics(hSTS), legH(end+1) = hSTS; legN{end+1} = 'Test true STS'; end %#ok<AGROW>
if isgraphics(hSOS), legH(end+1) = hSOS; legN{end+1} = 'Test true SOS'; end %#ok<AGROW>
if isgraphics(hErr) && ~isempty(hErr)
    legH(end+1) = hErr(1);              legN{end+1} = 'Misclassified (X)'; %#ok<AGROW>
end
if ~isempty(legH)
    legend(legH, legN, 'Location','bestoutside');
end
xlim([0 1]);


%% Function
function [hc_mask, feat_x, feat_y, idx_list, ...
          hc_idx_R, hc_idx_L, up_idx_R, up_idx_L, low_idx_R, low_idx_L] = ...
    compute_features_RT(Rdeg, Ldeg, fs, t_gap, thres_up, thres_down, var_crit, scaling_X, scaling_Y)

    N = length(Rdeg);
    dt = 1/fs;

    % 상태/카운터
    R_time_afterupFlag = 0;  L_time_afterupFlag = 0;  HC_time_afterFlag = 0;
    R_swing_time = 0;        L_swing_time = 0;

    R_time_upcond = false;   L_time_upcond = false;   HC_time_upcond = false;
    HC_Rswingflag4upcond = 0; HC_Lswingflag4upcond = 0;

    swing_period  = 400;    % 초기 1000 tick (1초 가정)
    R_lowpeak_val = thres_down;
    L_lowpeak_val = thres_down;

    Rpeakval = 50; Lpeakval = 50;

    % 출력 버퍼
    hc_mask = zeros(N,1);
    feat_x  = [];  feat_y = [];  idx_list = [];

    % 이벤트 인덱스 (R/L 분리)
    hc_idx_R = []; hc_idx_L = [];
    up_idx_R = []; up_idx_L = [];
    low_idx_R= []; low_idx_L= [];

    for i = 3:N
        % 시간 카운터 증가
        R_time_afterupFlag = R_time_afterupFlag + 1;
        L_time_afterupFlag = L_time_afterupFlag + 1;
        HC_time_afterFlag  = HC_time_afterFlag + 1;
        R_swing_time       = R_swing_time + 1;
        L_swing_time       = L_swing_time + 1;

        % 3-샘플 윈도우
        R0 = Rdeg(i-2); R1 = Rdeg(i-1); R2 = Rdeg(i);
        L0 = Ldeg(i-2); L1 = Ldeg(i-1); L2 = Ldeg(i);

        % 재트리거 허용 조건
        if R_time_afterupFlag >= t_gap, R_time_upcond = true; end
        if L_time_afterupFlag >= t_gap, L_time_upcond = true; end
        if HC_time_afterFlag  >= t_gap, HC_time_upcond  = true; end

        % ================= HC 체크 =================
        if HC_time_upcond && ((R2 - L2)*(R1 - L1) <= 0) && (R2 > thres_down) && (L2 > thres_down)
            HC_time_upcond    = false;
            HC_time_afterFlag = 0;
            hc_mask(i) = 1;  % 디버그 마커

            % 어느 쪽 스윙인지에 따라 속도/시간 선택
            if (R2 - R1) >= (L2 - L1) && Lpeakval > 40
                vel_HC = (R2 - R1)/dt;     % deg/s
                T_HC   = R_swing_time;     % ticks
                HC_Rswingflag4upcond = 1;
                hc_idx_R(end+1,1) = i;     % HC at Rswing
            elseif (L2 - L1) >= (R2 - R1) && Rpeakval > 40
                vel_HC = (L2 - L1)/dt;
                T_HC   = L_swing_time;
                HC_Lswingflag4upcond = 1;
                hc_idx_L(end+1,1) = i;     % HC at Lswing
            else
                % 둘 다 조건 안 맞으면 스킵
                continue;
            end

            % 특징(정규화)
            norm_vel_HC = (vel_HC * (swing_period * dt)) / scaling_X;
            norm_T_HC   = (T_HC   / (swing_period * dt)) / scaling_Y;

            % 이벤트 저장
            feat_x(end+1,1)  = norm_vel_HC; %#ok<AGROW>
            feat_y(end+1,1)  = norm_T_HC;  %#ok<AGROW>
            idx_list(end+1,1)= i;          %#ok<AGROW>
        end

        % ============= Upper peak (상부 피크) =============
        if ((R2 - R1)*(R1 - R0) < var_crit) && ((R2 - R1) < 0) && (R2 >= thres_up) && R_time_upcond && HC_Rswingflag4upcond
%         if ((R2 - R1)*(R1 - R0) < var_crit) && ((R2 - R1) < 0) && (R2 >= thres_up) && R_time_upcond
            Rpeakval = R2;
            R_time_upcond = false;
            swing_period  = R_swing_time;
            R_time_afterupFlag = 0;
            HC_Rswingflag4upcond = 0;

            up_idx_R(end+1,1) = i;   % Upper (R)
        end

        if ((L2 - L1)*(L1 - L0) < var_crit) && ((L2 - L1) < 0) && (L2 >= thres_up) && L_time_upcond && HC_Lswingflag4upcond
%         if ((L2 - L1)*(L1 - L0) < var_crit) && ((L2 - L1) < 0) && (L2 >= thres_up) && L_time_upcond
            Lpeakval = L2;
            L_time_upcond = false;
            swing_period  = L_swing_time;
            L_time_afterupFlag = 0;
            HC_Lswingflag4upcond = 0;

            up_idx_L(end+1,1) = i;   % Upper (L)
        end

        % ============= Lower peak (하부 피크) =============
        if (R2 > R_lowpeak_val)
            R_lowpeak_val = thres_down;
        end
        if ((R2 - R1)*(R1 - R0) < var_crit) && ((R2 - R1) >= 0) && (R2 <= R_lowpeak_val + 3) && (R2 - R1) - (L2 - L1) > 0
            R_lowpeak_val = R2;
            R_swing_time = 0;
            low_idx_R(end+1,1) = i;  % Lower (R)
        end

        if (L2 > L_lowpeak_val)
            L_lowpeak_val = thres_down;
        end
        if ((L2 - L1)*(L1 - L0) < var_crit) && ((L2 - L1) > 0) && (L2 <= L_lowpeak_val + 3) && (R2 - R1) - (L2 - L1) < 0
            L_lowpeak_val = L2;
            L_swing_time = 0;
            low_idx_L(end+1,1) = i;  % Lower (L)
        end
    end
end

% ================== 함수들 ==================
function [pred, acc, CM] = plot_knn_regions(ref_dataset, test_dataset, k, grid_res)
    % 데이터 분리
    rx = ref_dataset(:,1); ry = ref_dataset(:,2); rlab = ref_dataset(:,3);
    tx = test_dataset(:,1); ty = test_dataset(:,2); tlab = test_dataset(:,3);

    % 그리드 생성
    xv = linspace(0, 1.5, grid_res);
    yv = linspace(0, 1.5, grid_res);
    [Xg, Yg] = meshgrid(xv, yv);      % (Ny x Nx)

    % 그리드 예측 (가벼운 다수결 KNN)
    Z = knn_predict_grid(rx, ry, rlab, Xg, Yg, k);  % (Ny x Nx), 1/2

    % 테스트셋 예측 및 평가
    pred = knn_classify_points(rx, ry, rlab, tx, ty, k);
    acc  = mean(pred == tlab);
    CM   = zeros(2,2);
    CM(1,1) = sum(tlab==1 & pred==1);
    CM(1,2) = sum(tlab==1 & pred==2);
    CM(2,1) = sum(tlab==2 & pred==1);
    CM(2,2) = sum(tlab==2 & pred==2);

    % ---- 그림 ----
    figure('Name',sprintf('KNN Decision Regions (K=%d)',k)); hold on;
    % 배경 결정영역
    imagesc(xv, yv, Z); set(gca,'YDir','normal');
    colormap([1 0.92 0.92; 0.90 0.93 1.0]);  % 1=붉은톤, 2=푸른톤
    contour(xv, yv, Z, [1.5 1.5], 'k-', 'LineWidth', 1.0); % 경계선

    % 레퍼런스 점 (채움)
    scatter(rx(rlab==1), ry(rlab==1), 14, 'r.', 'MarkerFaceAlpha',0.7);
    scatter(rx(rlab==2), ry(rlab==2), 14, 'b.', 'MarkerFaceAlpha',0.7);

    % 테스트 점 (정답 라벨 기준 테두리만)
    scatter(tx(tlab==1), ty(tlab==1), 36, 'ro', 'LineWidth',2);
    scatter(tx(tlab==2), ty(tlab==2), 36, 'bo', 'LineWidth',2);

    % 오분류 표시(X)
    err = (pred ~= tlab);
    if any(err)
        scatter(tx(err), ty(err), 40, 'kx', 'LineWidth',1.5);
    end

    grid on; box on; xlabel('x'); ylabel('y');
    title(sprintf('K=%d | Acc=%.2f%%', k, acc*100));
    xlim([0 1.5]); ylim([0 1.5]);

    % 범례
    legend({'Boundary', 'Ref pt STS(1)','Ref pt SOS(2)','Test true STS','Test true SOS','Misclassified'}, ...
            'Location','bestoutside');
end


function Z = knn_predict_grid(rx, ry, rlab, Xg, Yg, k)
    % Xg,Yg: (Ny x Nx). 메모리 절약 위해 배치로 처리
    [Ny, Nx] = size(Xg);
    Q = Ny*Nx;
    Xq = Xg(:); Yq = Yg(:);

    M = numel(rx);
    if k > M, k = M; end

    Zq = zeros(Q,1);
    batch = 3000;                           % 배치 크기 (메모리 여유에 맞춰 조정)
    for s = 1:batch:Q
        e = min(s+batch-1, Q);
        B = e - s + 1;

        % 거리 (MxB)
        Dx = rx - Xq(s:e).';    % implicit expansion
        Dy = ry - Yq(s:e).';
        d2 = Dx.^2 + Dy.^2;

        % 상위 k 이웃
        [~, ord] = sort(d2, 1, 'ascend');   % (M x B)
        nn = rlab(ord(1:k, :));             % (k x B)

        c1 = sum(nn == 1, 1);
        c2 = k - c1;
        Zq(s:e) = 1 + (c1 < c2);            % 다수결, 동률이면 1
    end

    Z = reshape(Zq, Ny, Nx);
end


function pred = knn_classify_points(rx, ry, rlab, Xq, Yq, k)
    % 여러 쿼리 점을 한 번에 예측 (배치)
    M = numel(rx);
    N = numel(Xq);
    if k > M, k = M; end

    pred = zeros(N,1);
    batch = 3000;
    for s = 1:batch:N
        e = min(s+batch-1, N);

        Dx = rx - Xq(s:e).';
        Dy = ry - Yq(s:e).';
        d2 = Dx.^2 + Dy.^2;

        [~, ord] = sort(d2, 1, 'ascend');
        nn = rlab(ord(1:k, :));

        c1 = sum(nn==1, 1);
        c2 = k - c1;
        pred(s:e) = 1 + (c1 < c2);
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