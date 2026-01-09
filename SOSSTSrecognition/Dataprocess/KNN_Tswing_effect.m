%% KNNClassification_Final.m
% 기능:
% 1. Reference 데이터셋 로드 (최적 K값 포함) 및 KNN 모델 학습 (Feature X=Distance)
% 2. 타겟 CSV 파일 로드
% 3. 상세 감지 확인 그래프 (HC, Peak)
% 4. Online Simulation (Method A, B, C)
% 5. Time Domain Analysis
% 6. Quantitative Statistics (Boxplots)
% 7. KNN Classification Visualization (Feature X vs Feature Y)

clear all; close all; clc;

% =========================================================================
% [설정] 데이터 경로 및 파일명
% =========================================================================
targetDir = 'C:\Users\ChanyoungKo\OneDrive - 한국과학기술원 기계공학과\Kyle\KAIST\ExoLab\성과\Publication\Ongoing\1저자_2025_RAL_StairMode\2nd_Submit\Revision_Data\Transient_test';
% Reference Dataset 경로
refDir = 'C:\Users\ChanyoungKo\OneDrive - 한국과학기술원 기계공학과\Kyle\KAIST\ExoLab\성과\Publication\Ongoing\1저자_2025_RAL_StairMode\2nd_Submit\Revision_Data\Ref_dataset';
refFile = fullfile(refDir, 'refdataset_merged_scaled.mat');

%% 1. Reference Dataset 로드 및 KNN 모델 학습 (Only Feature X = Distance)
if ~exist(refFile, 'file')
    error('Reference dataset 파일을 찾을 수 없습니다: %s', refFile);
end
fprintf('Reference Dataset 로드 중: %s\n', refFile);

load(refFile, 'ref_final_scaled', 'scale_factor', 'best_K'); 

% 1-1. Scaling Factor 복원
mx_feat_x = scale_factor(1);
mx_feat_y = scale_factor(2);

% [수정됨] Velocity 계산 부분 삭제 (Case 1 제거)
% Feature X (Distance)는 이미 ref_final_scaled(:,1)에 정규화되어 있음

fprintf('[Scaling Info]\n Max Feat X: %.2f\n Max Feat Y: %.2f\n', mx_feat_x, mx_feat_y);

% 1-2. Training Data 구성 (Case 2 Only)
X_train = ref_final_scaled(:, 1:2); % Feature X (Dist), Feature Y
Y_train = ref_final_scaled(:, 3);   % Labels

% 1-3. KNN 모델 학습 (최적 K 및 Distance Weight 적용)
if exist('best_K', 'var')
    knn_k = best_K;
    fprintf('최적화된 K값 적용: K=%d\n', knn_k);
else
    knn_k = 9;
    warning('최적 K값을 찾을 수 없어 기본값(9)을 사용합니다.');
end

% 모델 학습 (DistanceWeight: inverse, Standardize: false)
knn_model = fitcknn(X_train, Y_train, 'NumNeighbors', knn_k, ...
    'DistanceWeight', 'inverse', 'Standardize', false);

fprintf('KNN 모델 학습 완료 (K=%d)\n', knn_k);


%% 2. 타겟 CSV 다중 선택 및 피험자별 처리
CSV_HEADER = [
    "LoopCnt","H10Mode","H10AssistLevel","SmartAssist", ...
    "LeftHipAngle","RightHipAngle","LeftThighAngle","RightThighAngle", ...
    "LeftHipTorque","RightHipTorque","LeftHipMotorAngle","RightHipMotorAngle", ...
    "LeftHipImuGlobalAccX","LeftHipImuGlobalAccY","LeftHipImuGlobalAccZ", ...
    "LeftHipImuGlobalGyrX","LeftHipImuGlobalGyrY","LeftHipImuGlobalGyrZ", ...
    "RightHipImuGlobalAccX","RightHipImuGlobalAccY","RightHipImuGlobalAccZ", ...
    "RightHipImuGlobalGyrX","RightHipImuGlobalGyrY","RightHipImuGlobalGyrZ", ...
    "is_moving","hc_count","R_count_upeak","L_count_upeak", ...
    "R_count_dpeak","L_count_dpeak", ...
    "tau_max_setting","s_gait_mode","s_g_knn_conf", ...
    "T_swing_ms","T_swing_SOS_ms","T_swing_STS_ms", ...
    "T_swing_SOS_ms_conf1","T_swing_STS_ms_conf1","TswingRecording_ms", ...
    "s_vel_HC","s_T_HC_s", ...
    "s_norm_vel_HC","s_norm_T_HC","s_scaling_X","s_scaling_Y", ...
    "s_t_gap_R_ms","s_t_gap_L_ms","s_hc_deg_thresh","s_thres_up","s_thres_down"
];

% 여러 CSV 선택 (MultiSelect)
[csvFiles, csvPath] = uigetfile(fullfile(targetDir, '*.csv'), ...
    'Select target CSV file(s)', 'MultiSelect', 'on');
if isequal(csvFiles, 0)
    error('CSV 파일 선택이 취소되었습니다.');
end
if ischar(csvFiles)
    csvFiles = {csvFiles};
end
nSub = numel(csvFiles);

feat_A_all = cell(nSub, 1);
feat_B_all = cell(nSub, 1);
feat_C_all = cell(nSub, 1);
subjNames  = cell(nSub, 1);

% 공통 파라미터 (기존과 동일)
fs = 500;
fc = 20;
t_gap_ms = 300;
thres_down = 10;

for sIdx = 1:nSub
    csvFile = csvFiles{sIdx};
    fullCsvPath = fullfile(csvPath, csvFile);
    subjNames{sIdx} = csvFile;

    fprintf('\n=== Processing %s (%d/%d) ===\n', csvFile, sIdx, nSub);

    [feat_A, feat_B, feat_C] = process_one_subject( ...
        fullCsvPath, csvFile, CSV_HEADER, fs, fc, t_gap_ms, thres_down, ...
        scale_factor, knn_model);

    feat_A_all{sIdx} = feat_A;
    feat_B_all{sIdx} = feat_B;
    feat_C_all{sIdx} = feat_C;
end

%% 7. Multi-subject Figure 4/5 및 Subplot3 분리 Figure 생성
% Figure 4:
% - 2x2 배치
%   (1,1) steady-circle: T_cycle 유형1 (현재 코드 기준 subplot1)
%   (1,2) steady-circle: T_cycle 유형2 (현재 코드 기준 subplot2)
%   (2,1) transient-star: T_cycle 유형1
%   (2,2) transient-star: T_cycle 유형2
create_classification_plot_multi(4, 'Figure 4: Classification (Multi-subject)', knn_model, feat_A_all, feat_B_all, subjNames);

% 기존 Figure4 subplot3는 별도 Figure로 이동: 피험자 수 x 1 subplot
create_time_domain_per_subject(44, 'Figure 4 (Subplot3 moved): Time Domain per Subject', feat_A_all, feat_B_all, subjNames);

% Figure 5: 기존 방식(원래 A vs B 비교) 유지, 단 피험자별 색 분리
create_signed_distance_plot_multi(5, 'Figure 5: Signed Distance to Boundary (A vs B, Multi-subject)', knn_model, feat_A_all, feat_B_all, subjNames);


%% ================== Functions ==================
function [feat_A, feat_B, feat_C] = process_one_subject(fullCsvPath, csvFile, CSV_HEADER, fs, fc, t_gap_ms, thres_down, scale_factor, knn_model)
    if ~exist(fullCsvPath, 'file')
        error('CSV 파일을 찾을 수 없습니다: %s', fullCsvPath);
    end

    opts = detectImportOptions(fullCsvPath, 'Encoding','UTF-8');
    opts.DataLines = [2 Inf];
    opts.VariableNamesLine = 0;
    opts.VariableNamingRule = 'preserve';
    T = readtable(fullCsvPath, opts);
    try
        T.Properties.VariableNames = CSV_HEADER;
    catch
        warning('헤더 매칭 실패. 인덱스로 데이터 로드합니다.');
    end

    if ismember('RightThighAngle', T.Properties.VariableNames)
        angle_R = LPF(T.RightThighAngle, fs, fc);
        angle_L = LPF(T.LeftThighAngle, fs, fc);
        is_moving = T.is_moving;
    else
        angle_R = LPF(T{:,8}, fs, fc);
        angle_L = LPF(T{:,7}, fs, fc);
        is_moving = T{:,25};
    end

    N = length(angle_R);
    t = (0:N-1)' / fs;

    % 3. HC 및 Peak 감지 (상세 확인용)
    [hc_idx_list, hc_types, upeak_R, upeak_L, dpeak_R, dpeak_L] = ...
        find_hc_indices(angle_R, angle_L, is_moving, fs, t_gap_ms, thres_down);

    M = length(hc_idx_list);
    fprintf('감지된 HC 개수: %d\n', M);

    % Figure 1: Detailed Detection Check (피험자별)
    figure('Name', ['Figure 1: Detailed Detection Check - ' char(csvFile)], ...
        'NumberTitle', 'off', 'Position', [100, 200, 1400, 600]);
    plot(t, angle_R, 'r', 'DisplayName','Right Thigh'); hold on; grid on;
    plot(t, angle_L, 'b', 'DisplayName','Left Thigh');
    plot(t, is_moving * 10, 'k-', 'LineWidth', 0.5, 'DisplayName', 'is\_moving');

    hc_R_idxs = hc_idx_list(hc_types == 1);
    hc_L_idxs = hc_idx_list(hc_types == 2);
    if ~isempty(hc_R_idxs)
        plot(t(hc_R_idxs), angle_R(hc_R_idxs), 'ms', 'MarkerFaceColor','m', 'MarkerSize', 8, 'DisplayName','HC (R swing)');
    end
    if ~isempty(hc_L_idxs)
        plot(t(hc_L_idxs), angle_L(hc_L_idxs), 'gs', 'MarkerFaceColor','g', 'MarkerSize', 8, 'DisplayName','HC (L swing)');
    end

    if ~isempty(upeak_R), plot(t(upeak_R), angle_R(upeak_R), 'r^', 'MarkerFaceColor','r', 'DisplayName','Upper(R)'); end
    if ~isempty(upeak_L), plot(t(upeak_L), angle_L(upeak_L), 'b^', 'MarkerFaceColor','b', 'DisplayName','Upper(L)'); end
    if ~isempty(dpeak_R), plot(t(dpeak_R), angle_R(dpeak_R), 'rv', 'MarkerFaceColor','r', 'DisplayName','Lower(R)'); end
    if ~isempty(dpeak_L), plot(t(dpeak_L), angle_L(dpeak_L), 'bv', 'MarkerFaceColor','b', 'DisplayName','Lower(L)'); end

    if ~isempty(hc_idx_list)
        text(t(hc_idx_list), max(angle_R(hc_idx_list), angle_L(hc_idx_list)) + 5, string(1:M)', ...
            'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', 'k');
    end

    title(['File: ' char(csvFile) ' (Check HC & Peaks)'], 'Interpreter','none');
    xlabel('Time (s)'); ylabel('Angle (deg)');
    legend('Location', 'best'); xlim([t(1) t(end)]);

    % 라벨 입력
    label_input = prompt_labels(M);
    if isempty(label_input)
        error('라벨 입력이 취소되었습니다.');
    end

    % 4. 시뮬레이션: 세 가지 Method (A, B, C)로 Feature 계산
    mx_vel_dummy = 1;
    [feat_A, feat_B, feat_C] = simulate_normalization_comparison( ...
        angle_R, angle_L, is_moving, label_input, fs, t_gap_ms, thres_down, scale_factor, mx_vel_dummy);

    % 6. Figure 3: Quantitative Comparison (Boxplots) - 피험자별 유지
    bg_pts = get_boundary_points(knn_model);
    stats_A = analyze_method_stats(feat_A, knn_model, bg_pts);
    stats_B = analyze_method_stats(feat_B, knn_model, bg_pts);
    stats_C = analyze_method_stats(feat_C, knn_model, bg_pts);

    figure('Name', ['Figure 3: Quantitative Comparison - ' char(csvFile)], ...
        'Position', [150, 150, 1000, 500], 'Color', 'w');
    sgtitle('Quantitative Comparison (Safety Margin)', 'FontSize', 14, 'FontWeight', 'bold');

    hold on; grid on; box on;
    full_labels_dist = {'A(T)', 'B(T)', 'C(T)', 'A(S)', 'B(S)', 'C(S)'};
    full_ids_dist    = [1, 2, 3, 5, 6, 7];

    data_dist = {stats_A.dist_trans, stats_B.dist_trans, stats_C.dist_trans, ...
                 stats_A.dist_steady, stats_B.dist_steady, stats_C.dist_steady};
    grp_dist = [];
    for i=1:3, grp_dist=[grp_dist; i*ones(size(data_dist{i}))]; end
    for i=4:6, grp_dist=[grp_dist; (i+1)*ones(size(data_dist{i}))]; end

    all_dist_data = vertcat(data_dist{:});

    if ~isempty(all_dist_data)
        existing_groups = unique(grp_dist);
        [~, loc] = ismember(existing_groups, full_ids_dist);
        active_labels = full_labels_dist(loc);
        boxplot(all_dist_data, grp_dist, 'Positions', existing_groups, 'Labels', active_labels);
        ylabel('Signed Distance (Neg=Safe)'); title('Safety Margin'); yline(0, 'k--');
        xlim([0 8]);
    else
        text(0.5, 0.5, 'No Data Available', 'HorizontalAlignment','center');
    end
end

function create_classification_plot_multi(figNum, figTitle, model, fType1_all, fType2_all, subjNames)
    nSub = numel(subjNames);

    % subject-specific marker shapes (repeat if needed)
    markerList = {'o','s','d','^','v','>','<','p','h','x','+','.'};

    % KNN map range
    ng = 220;
    xg = linspace(0, 1.5, ng); yg = linspace(0, 1.5, ng);
    [Xg, Yg] = meshgrid(xg, yg);
    [predGrid, ~] = predict(model, [Xg(:), Yg(:)]);
    Z = reshape(predGrid, size(Xg));

    baseBlue = [0.82 0.86 1.00];
    baseRed  = [1.00 0.84 0.84];
    RGB = zeros([size(Z), 3]);
    mask1 = (Z == 1); mask2 = (Z == 2);
    for c = 1:3
        tmp = zeros(size(Z));
        tmp(mask1) = baseRed(c);
        tmp(mask2) = baseBlue(c);
        RGB(:,:,c) = tmp;
    end

    figure('Name', figTitle, 'NumberTitle', 'off', 'Color', 'w', ...
        'Units','pixels', 'Position', [50, 80, 1300, 780]);
    sgtitle(figTitle, 'FontSize', 15, 'FontWeight', 'bold');

    % 4 subplots:
    % (steady/type1), (steady/type2), (transient/type1), (transient/type2)
    subplot(2,2,1); hold on; box on; grid on;
    imagesc(xg, yg, RGB); set(gca, 'YDir', 'normal');
    contour(Xg, Yg, Z, [1.5 1.5], 'k--', 'LineWidth', 1.5);
    title('Steady-state data points');
    xlabel('Feature X'); ylabel('Feature Y');
    xlim([0 1.5]); ylim([0 1.5]);

    subplot(2,2,2); hold on; box on; grid on;
    imagesc(xg, yg, RGB); set(gca, 'YDir', 'normal');
    contour(Xg, Yg, Z, [1.5 1.5], 'k--', 'LineWidth', 1.5);
    title('Steady-state data points');
    xlabel('Feature X'); ylabel('Feature Y');
    xlim([0 1.5]); ylim([0 1.5]);

    subplot(2,2,3); hold on; box on; grid on;
    imagesc(xg, yg, RGB); set(gca, 'YDir', 'normal');
    contour(Xg, Yg, Z, [1.5 1.5], 'k--', 'LineWidth', 1.5);
    title('Transient-state data points');
    xlabel('Feature X'); ylabel('Feature Y');
    xlim([0 1.5]); ylim([0 1.5]);

    subplot(2,2,4); hold on; box on; grid on;
    imagesc(xg, yg, RGB); set(gca, 'YDir', 'normal');
    contour(Xg, Yg, Z, [1.5 1.5], 'k--', 'LineWidth', 1.5);
    title('Transient-state data points');
    xlabel('Feature X'); ylabel('Feature Y');
    xlim([0 1.5]); ylim([0 1.5]);

    % Pre-make subject legend handles (one per subject) using marker shapes
    lg = gobjects(nSub,1);
    subjTags = cell(nSub,1);
    for s = 1:nSub
        subjTags{s} = get_subject_tag(subjNames{s});
        mk = markerList{mod(s-1, numel(markerList)) + 1};
        subplot(2,2,1);
        lg(s) = plot(NaN, NaN, mk, 'MarkerSize', 7, 'LineStyle','none', ...
            'MarkerFaceColor', [0.6 0.6 0.6], 'MarkerEdgeColor','k', ...
            'DisplayName', subjTags{s});
    end

    % Plot points
    for s = 1:nSub
        f1 = fType1_all{s};   % T_cycle type 1
        f2 = fType2_all{s};   % T_cycle type 2
        mk = markerList{mod(s-1, numel(markerList)) + 1};

        if isempty(f1) || isempty(f2) || ~isfield(f1,'x') || ~isfield(f2,'x')
            continue;
        end

        % Masks
        [is_trans_1, steady_1] = compute_transition_masks(f1.labels);
        [is_trans_2, steady_2] = compute_transition_masks(f2.labels);

        % Color rule by offline label: 1=red, 2=blue
        red = [1 0 0];
        blue = [0 0 1];

        % ---------- Steady: Type 1 (subplot 1) ----------
        subplot(2,2,1);
        idx11 = steady_1 & (f1.labels(:) == 1);
        idx12 = steady_1 & (f1.labels(:) == 2);
        if any(idx11)
            plot(f1.x(idx11), f1.y(idx11), mk, 'MarkerSize', 6, ...
                'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'LineStyle','none');
        end
        if any(idx12)
            plot(f1.x(idx12), f1.y(idx12), mk, 'MarkerSize', 6, ...
                'MarkerFaceColor', blue, 'MarkerEdgeColor', 'k', 'LineStyle','none');
        end

        % ---------- Steady: Type 2 (subplot 2) ----------
        subplot(2,2,2);
        idx21 = steady_2 & (f2.labels(:) == 1);
        idx22 = steady_2 & (f2.labels(:) == 2);
        if any(idx21)
            plot(f2.x(idx21), f2.y(idx21), mk, 'MarkerSize', 6, ...
                'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'LineStyle','none');
        end
        if any(idx22)
            plot(f2.x(idx22), f2.y(idx22), mk, 'MarkerSize', 6, ...
                'MarkerFaceColor', blue, 'MarkerEdgeColor', 'k', 'LineStyle','none');
        end

        % ---------- Transient: Type 1 (subplot 3, star) ----------
        subplot(2,2,3);
        idx31 = is_trans_1 & (f1.labels(:) == 1);
        idx32 = is_trans_1 & (f1.labels(:) == 2);
        if any(idx31)
            plot(f1.x(idx31), f1.y(idx31), mk, 'MarkerSize', 8, ...
                'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'LineWidth', 1.0, 'LineStyle','none');
        end
        if any(idx32)
            plot(f1.x(idx32), f1.y(idx32), mk, 'MarkerSize', 8, ...
                'MarkerFaceColor', blue, 'MarkerEdgeColor', 'k', 'LineWidth', 1.0, 'LineStyle','none');
        end

        % ---------- Transient: Type 2 (subplot 4, star) ----------
        subplot(2,2,4);
        idx41 = is_trans_2 & (f2.labels(:) == 1);
        idx42 = is_trans_2 & (f2.labels(:) == 2);
        if any(idx41)
            plot(f2.x(idx41), f2.y(idx41), mk, 'MarkerSize', 8, ...
                'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'LineWidth', 1.0, 'LineStyle','none');
        end
        if any(idx42)
            plot(f2.x(idx42), f2.y(idx42), mk, 'MarkerSize', 8, ...
                'MarkerFaceColor', blue, 'MarkerEdgeColor', 'k', 'LineWidth', 1.0, 'LineStyle','none');
        end
    end

    % Legends: one per subplot, only subjects (file count entries)
    for k = 1:4
        subplot(2,2,k);
        legend(lg, 'Location','best');
    end
end
function create_time_domain_per_subject(figNum, figTitle, fType1_all, fType2_all, subjNames)
    nSub = numel(subjNames);
    figure('Name', figTitle, 'NumberTitle', 'off', 'Color', 'w', ...
        'Units','pixels', 'Position', [100, 60, 1200, min(250*nSub+120, 900)]);
    sgtitle(figTitle, 'FontSize', 15, 'FontWeight', 'bold');

    for s = 1:nSub
        f1 = fType1_all{s};
        f2 = fType2_all{s};

        subplot(nSub, 1, s); hold on; grid on; box on;
        if ~isempty(f1.measured)
            steps = 1:length(f1.measured);

            plot(steps, f1.measured, '--o', 'Color', [0.5 0.5 0.5], 'LineWidth', 1, ...
                'MarkerSize', 3, 'DisplayName', 'Measured');
            plot(steps, f1.refs, 'r-s', 'LineWidth', 2, 'MarkerSize', 5, 'DisplayName', 'Ref Type 1');
            plot(steps, f2.refs, '-d', 'Color', [0 0.5 0], 'LineWidth', 2, 'MarkerSize', 5, 'DisplayName', 'Ref Type 2');
        else
            text(0.5, 0.5, 'No Data', 'HorizontalAlignment','center');
        end

        xlabel('HC count'); ylabel('Swing time [ms]');
        legend('Location', 'best');
    end
end

function create_signed_distance_plot_multi(figNum, figTitle, model, fType1_all, fType2_all, subjNames)
    nSub = numel(subjNames);
    cols = lines(max(nSub,1));

    ng = 220;
    xg = linspace(0, 1.5, ng); yg = linspace(0, 1.5, ng);
    [Xg, Yg] = meshgrid(xg, yg);
    [predGrid, ~] = predict(model, [Xg(:), Yg(:)]);
    Z = reshape(predGrid, size(Xg));
    C = contourc(xg, yg, double(Z), [1.5 1.5]);
    boundaryXY = parse_contourc_to_xy(C);

    figure('Name', figTitle, 'NumberTitle', 'off', 'Color', 'w', ...
        'Units','pixels', 'Position', [80, 120, 980, 440]);

    subplot(1,2,1); hold on; grid on; box on;
    title('Steady-state data points'); ylabel('Signed min distance to boundary');
    xlim([0.5 2.5]); xticks([1 2]); xticklabels({'Type 1','Type 2'});

    subplot(1,2,2); hold on; grid on; box on;
    title('Transient-state data points'); ylabel('Signed min distance to boundary');
    xlim([0.5 2.5]); xticks([1 2]); xticklabels({'Type 1','Type 2'});

    lg = gobjects(0);
    subjTags = cell(nSub,1);

    for s = 1:nSub
        c = cols(s,:);
        f1 = fType1_all{s};
        f2 = fType2_all{s};
        subjTags{s} = get_subject_tag(subjNames{s});

        if isempty(f1) || isempty(f2) || ~isfield(f1,'x') || ~isfield(f2,'x')
            continue;
        end

        [is_trans_1, steady_1] = compute_transition_masks(f1.labels);
        [is_trans_2, steady_2] = compute_transition_masks(f2.labels);

        sd1 = signed_distance_to_boundary(model, boundaryXY, [f1.x(:), f1.y(:)], f1.labels(:));
        sd2 = signed_distance_to_boundary(model, boundaryXY, [f2.x(:), f2.y(:)], f2.labels(:));

        off = (s - (nSub+1)/2) * 0.10;  % subject offset for mean/std bars

        % ---------------- Steady (subplot 1) ----------------
        subplot(1,2,1);
        d1c = sd1(steady_1); d2c = sd2(steady_2);

        scatter(1 + off + 0.02*randn(size(d1c)), d1c, 18, 'filled', ...
            'MarkerFaceColor', c, 'MarkerEdgeColor','k');
        scatter(2 + off + 0.02*randn(size(d2c)), d2c, 18, 'filled', ...
            'MarkerFaceColor', c, 'MarkerEdgeColor','k');

        mu1 = mean(d1c, 'omitnan'); sdv1 = std(d1c, 'omitnan');
        mu2 = mean(d2c, 'omitnan'); sdv2 = std(d2c, 'omitnan');
        if ~isnan(mu1) && ~isnan(sdv1)
            errorbar(1 + off, mu1, sdv1, 'LineWidth', 1.3, 'CapSize', 10, 'Color', c);
        end
        if ~isnan(mu2) && ~isnan(sdv2)
            errorbar(2 + off, mu2, sdv2, 'LineWidth', 1.3, 'CapSize', 10, 'Color', c);
        end

        % ---------------- Transient (subplot 2) ----------------
        subplot(1,2,2);
        d1s = sd1(is_trans_1); d2s = sd2(is_trans_2);

        scatter(1 + off + 0.02*randn(size(d1s)), d1s, 18, 'filled', ...
            'MarkerFaceColor', c, 'MarkerEdgeColor','k');
        scatter(2 + off + 0.02*randn(size(d2s)), d2s, 18, 'filled', ...
            'MarkerFaceColor', c, 'MarkerEdgeColor','k');

        mu1 = mean(d1s, 'omitnan'); sdv1 = std(d1s, 'omitnan');
        mu2 = mean(d2s, 'omitnan'); sdv2 = std(d2s, 'omitnan');
        if ~isnan(mu1) && ~isnan(sdv1)
            errorbar(1 + off, mu1, sdv1, 'LineWidth', 1.3, 'CapSize', 10, 'Color', c);
        end
        if ~isnan(mu2) && ~isnan(sdv2)
            errorbar(2 + off, mu2, sdv2, 'LineWidth', 1.3, 'CapSize', 10, 'Color', c);
        end

        % legend handle (one per subject)
        subplot(1,2,1);
        lg(end+1) = plot(NaN, NaN, 'o', 'MarkerFaceColor', c, 'MarkerEdgeColor','k', ...
            'LineStyle','none', 'DisplayName', subjTags{s}); %#ok<AGROW>
    end

    subplot(1,2,1);
    legend(lg, 'Location','best');
    yline(0, 'k--');

    subplot(1,2,2);
    yline(0, 'k--');
end
function [is_trans, is_steady] = compute_transition_masks(labels)
    N = length(labels);
    is_trans = false(N,1);
    if N == 0
        is_steady = false(0,1);
        return;
    end
    for i = 2:N
        if labels(i) ~= labels(i-1)
            is_trans(i) = true;
        end
    end
    is_steady = ~is_trans;
end

function sd = signed_distance_to_boundary(model, boundaryXY, pts, trueLabels)
    d = min_distance_to_boundary(boundaryXY, pts);
    [predPt, ~] = predict(model, pts);
    sd = d(:);
    sd(predPt ~= trueLabels) = -sd(predPt ~= trueLabels);
end


function create_classification_plot(figNum, figTitle, model, fA, fB, fC)
    ng = 200;
    xg = linspace(0, 1.2, ng); yg = linspace(0, 1.2, ng);
    [Xg, Yg] = meshgrid(xg, yg);
    [predGrid, ~] = predict(model, [Xg(:), Yg(:)]);
    Z = reshape(predGrid, size(Xg));

    % [색상 설정] SOS(1)=Blue, STS(2)=Red
    baseBlue = [0.8 0.8 1]; baseRed = [1 0.8 0.8];
    RGB = zeros([size(Z), 3]);
    mask1 = (Z == 1); mask2 = (Z == 2);
    for c = 1:3, RGB(:,:,c) = baseBlue(c)*mask1 + baseRed(c)*mask2; end

    figure('Name', figTitle, 'NumberTitle', 'off', 'Units','pixels', 'Position', [50, 80, 1523, 521], 'Color', 'w');

    sgtitle(figTitle, 'FontSize', 15, 'FontWeight', 'bold');

    % subplot(3,1,1~2): Classification (A/B)
    % subplot(3,1,3): Time Domain (A/B)  <-- Figure 2의 (1),(2) 합침
    methods_cls = {fA, fB};
    titles_cls  = {'Method A: Previous', 'Method B: Avg Buffer'};

    C = contourc(xg, yg, Z, [1.5 1.5]);
    boundaryXY = parse_contourc_to_xy(C);

    signedDist = cell(1,2);
    isStar     = cell(1,2);
    
    for i = 1:2
        subplot(1, 3, i); hold on;
        imagesc(xg, yg, RGB); set(gca, 'YDir', 'normal');
        contour(Xg, Yg, Z, [1.5 1.5], 'k--', 'LineWidth', 1.5);

        curr_feat = methods_cls{i};

        is_trans = plot_markers_on_bg(curr_feat.x, curr_feat.y, curr_feat.labels);

        d = min_distance_to_boundary(boundaryXY, [curr_feat.x(:), curr_feat.y(:)]);
        [predPt, ~] = predict(model, [curr_feat.x(:), curr_feat.y(:)]);
        lab_model = curr_feat.labels(:);
        lab_model(lab_model==1) = 2;   % offline SOS(1) -> model SOS(2)
        lab_model(lab_model==2) = 1;   % offline STS(2) -> model STS(1)

        sgn = ones(size(d));
        sgn(predPt ~= lab_model) = -1;
        signedDist{i} = sgn .* d(:);
        isStar{i} = is_trans(:);
    end

    % Legend Dummy (Classification)
    subplot(1, 3, 1);
    p1 = plot(NaN, NaN, 'bo', 'MarkerFaceColor','b', 'MarkerEdgeColor','k', 'DisplayName','SOS');
    p2 = plot(NaN, NaN, 'ro', 'MarkerFaceColor','r', 'MarkerEdgeColor','k', 'DisplayName','STS');
    legend([p1 p2], 'Location','best');

    % subplot(3,1,3): Time Domain (Figure 2의 subplot(2,2,1)+(2,2,2))
    if ~isempty(fA.refs)
        subplot(1, 3, 3); hold on; grid on; box on;

        steps = 1:length(fA.refs);
        labels_seq = fA.labels;
        y_lims_all = [min([fA.measured; fA.refs; fB.refs]) - 50, max([fA.measured; fA.refs; fB.refs]) + 50];

        plot(steps, fA.measured, '--o', 'Color', [0.5 0.5 0.5], 'LineWidth', 1, 'MarkerSize', 3, 'DisplayName', 'Measured');
        plot(steps, fA.refs,     'r-s',  'LineWidth', 2, 'MarkerSize', 5, 'DisplayName', 'Ref A');
        plot(steps, fB.refs,     '-d',  'Color', [0 0.5 0], 'LineWidth', 2, 'MarkerSize', 5, 'DisplayName', 'Ref B');


        xlabel('HC count'); ylabel('Swing time [ms]');
        legend('Location', 'best');
    end
    
    % (create_classification_plot 내부, signedDist/isStar 계산 완료 후에 추가)

    % Figure 5: circle / star 별 Method A vs Method B signed distance 비교
    isStarCommon = isStar{1};          % A/B는 labels 기반이라 동일
    circleMask   = ~isStarCommon;
    starMask     =  isStarCommon;

    figure('Name', 'Figure 5: Signed Distance to Boundary (A vs B)', 'NumberTitle', 'off', ...
           'Units','pixels', 'Position', [80, 120, 900, 420], 'Color', 'w');

    % subplot(1,2,1): circle
    subplot(1,2,1); hold on; grid on; box on;
    dA_c = signedDist{1}(circleMask);
    dB_c = signedDist{2}(circleMask);

    xA = 1 + 0.06*randn(size(dA_c));
    xB = 2 + 0.06*randn(size(dB_c));
    scatter(xA, dA_c, 18, 'filled');
    scatter(xB, dB_c, 18, 'filled');

    muA = mean(dA_c, 'omitnan'); sdA = std(dA_c, 'omitnan');
    muB = mean(dB_c, 'omitnan'); sdB = std(dB_c, 'omitnan');
    errorbar(1, muA, sdA, 'LineWidth', 1.5, 'CapSize', 10);
    errorbar(2, muB, sdB, 'LineWidth', 1.5, 'CapSize', 10);

    xlim([0.5 2.5]); xticks([1 2]); xticklabels({'Method A','Method B'});
    ylabel('Signed min distance to boundary');
    title('Circle points');

    % subplot(1,2,2): star
    subplot(1,2,2); hold on; grid on; box on;
    dA_s = signedDist{1}(starMask);
    dB_s = signedDist{2}(starMask);

    xA = 1 + 0.06*randn(size(dA_s));
    xB = 2 + 0.06*randn(size(dB_s));
    scatter(xA, dA_s, 18, 'filled');
    scatter(xB, dB_s, 18, 'filled');

    muA = mean(dA_s, 'omitnan'); sdA = std(dA_s, 'omitnan');
    muB = mean(dB_s, 'omitnan'); sdB = std(dB_s, 'omitnan');
    errorbar(1, muA, sdA, 'LineWidth', 1.5, 'CapSize', 10);
    errorbar(2, muB, sdB, 'LineWidth', 1.5, 'CapSize', 10);

    xlim([0.5 2.5]); xticks([1 2]); xticklabels({'Method A','Method B'});
    ylabel('Signed min distance to boundary');
    title('Star points');

    
end

function is_trans = plot_markers_on_bg(xvals, yvals, labels)
    % - circle(o): 일반 포인트 (연속 구간)
    % - star(*): 라벨 전이(transition) 포인트
    % 색상: SOS=Blue(1), STS=Red(2)
    N = length(labels);
    if N == 0
        is_trans = false(0,1);
        return;
    end

    is_trans = false(N, 1);
    for i = 2:N
        if labels(i) ~= labels(i-1)
            is_trans(i) = true;
        end
    end

    idx_o1 = (~is_trans) & (labels == 1);
    idx_o2 = (~is_trans) & (labels == 2);
    idx_s1 = ( is_trans) & (labels == 1);
    idx_s2 = ( is_trans) & (labels == 2);

    if any(idx_o1), plot(xvals(idx_o1), yvals(idx_o1), 'bo', 'MarkerFaceColor','b', 'MarkerEdgeColor','k', 'MarkerSize', 6); end
    if any(idx_o2), plot(xvals(idx_o2), yvals(idx_o2), 'ro', 'MarkerFaceColor','r', 'MarkerEdgeColor','k', 'MarkerSize', 6); end
    if any(idx_s1), plot(xvals(idx_s1), yvals(idx_s1), 'b*', 'MarkerEdgeColor','b', 'MarkerSize', 10, 'LineWidth', 1.2); end
    if any(idx_s2), plot(xvals(idx_s2), yvals(idx_s2), 'r*', 'MarkerEdgeColor','r', 'MarkerSize', 10, 'LineWidth', 1.2); end

%     text(xvals, yvals, string(1:N)', 'VerticalAlignment','bottom', 'FontSize',8, 'FontWeight','bold');
end

function draw_background_gait(steps, labels, y_lims)
    ylim(y_lims); xlim([0.5 length(steps)+0.5]);
    for i = 1:length(steps)
        % [색상 수정] SOS(1)=Light Blue, STS(2)=Light Red
        if labels(i) == 1
            fill([i-0.5 i+0.5 i+0.5 i-0.5], [y_lims(1) y_lims(1) y_lims(2) y_lims(2)], [0.9 0.9 1], 'EdgeColor', 'none', 'HandleVisibility', 'off');
        else
            fill([i-0.5 i+0.5 i+0.5 i-0.5], [y_lims(1) y_lims(1) y_lims(2) y_lims(2)], [1 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
        end
    end
end

function [feat_A, feat_B, feat_C] = simulate_normalization_comparison(Rdeg, Ldeg, is_moving, labels, fs, t_gap_ms, thres_down, scale_factor, mx_vel)
    dt = 1/fs; N = length(Rdeg);
    
    cur_hc_deg_thresh = 10.0; cur_stance_angle_gate = 10.0; cur_thres_up = 10.0;
    ALPHA = 0.2; var_crit = 0;
    
    SX = scale_factor(1); SY = scale_factor(2); SV = mx_vel; 
    
    ref_A_ms = 300.0; last_STS_ms = 350.0; last_SOS_ms = 400.0;
    ref_C_ms = 350.0; ALPHA_REF = 0.3; 
    
    feat_A = struct('x',[], 'y',[], 'vel',[], 'labels',[], 'refs',[], 'measured',[]);
    feat_B = feat_A; feat_B.hist_SOS=[]; feat_B.hist_STS=[]; feat_C = feat_A;
    
    R_tm=0; L_tm=0; HC_tm=0; R_swing_tm=0; L_swing_tm=0;
    R_upcond=false; L_upcond=false; HC_upcond=false;
    R_low_val=thres_down; L_low_val=thres_down;
    last_low_idx_R=1; last_low_idx_L=1;
    t_gap_samples = t_gap_ms * (fs/1000);
    
    hc_idx=0; 
    pendR = struct('valid',false, 'vel',0, 'T_curr',0, 'label',0, 'refA',0, 'refB',0, 'refC',0);
    pendL = pendR;
    
    R0=0; R1=0; L0=0; L1=0;

    for i = 3:N
        R_tm=R_tm+1; L_tm=L_tm+1; HC_tm=HC_tm+1; R_swing_tm=R_swing_tm+1; L_swing_tm=L_swing_tm+1;
        R0=Rdeg(i-2); R1=Rdeg(i-1); R2=Rdeg(i); L0=Ldeg(i-2); L1=Ldeg(i-1); L2=Ldeg(i);
        
        if R_tm>=t_gap_samples, R_upcond=true; end
        if L_tm>=t_gap_samples, L_upcond=true; end
        if HC_tm>=t_gap_samples, HC_upcond=true; end
        
        if HC_upcond && ((R2-L2)*(R1-L1) <= 0)
            is_R_swing = ((R2-R1) >= (L2-L1));
            is_valid_hc = false; swing_deg_meas = 0;
            if is_R_swing
                if (R2 >= cur_hc_deg_thresh) && (L2 >= cur_stance_angle_gate), is_valid_hc = true; swing_deg_meas = R2; end
            else
                if (L2 >= cur_hc_deg_thresh) && (R2 >= cur_stance_angle_gate), is_valid_hc = true; swing_deg_meas = L2; end
            end
            
            if is_valid_hc
                hc_idx = hc_idx + 1;
                if hc_idx > length(labels), break; end % CHANGED: 라벨 입력 길이 초과 시 종료
                curr_label = labels(hc_idx);           % CHANGED: 0이면 해당 HC만 제외(continue)
                curr_ref_B = (last_STS_ms + last_SOS_ms) / 2.0; curr_ref_C = ref_C_ms; 
                HC_upcond = false; HC_tm = 0;
                deg_target = max(5.0, min(90.0, 0.55 * swing_deg_meas));
                cur_hc_deg_thresh = (1.0 - ALPHA) * cur_hc_deg_thresh + ALPHA * deg_target;
                if curr_label == 0                    % CHANGED
                    % 0 입력: 해당 HC point는 feature/label에서 제외하고 다음 HC로 진행 % CHANGED
                    continue;                         % CHANGED
                end                                  % CHANGED
                
                if is_R_swing
                    pendR.valid = true; pendR.vel = (R2-R1)/dt; pendR.T_curr = R_swing_tm * dt;
                    pendR.label = curr_label;
                    pendR.refA = ref_A_ms / 1000.0; pendR.refB = curr_ref_B / 1000.0; pendR.refC = curr_ref_C / 1000.0;
                else 
                    pendL.valid = true; pendL.vel = (L2-L1)/dt; pendL.T_curr = L_swing_tm * dt;
                    pendL.label = curr_label;
                    pendL.refA = ref_A_ms / 1000.0; pendL.refB = curr_ref_B / 1000.0; pendL.refC = curr_ref_C / 1000.0;
                end
            end
        end
        
        if ((R2-R1)*(R1-R0)<=var_crit) && ((R2-R1)<0) && (R2>=cur_thres_up) && R_upcond
            R_upcond = false; R_tm = 0;
            cur_stance_angle_gate = (1.0 - ALPHA) * cur_stance_angle_gate + ALPHA * max(5.0, min(120.0, 0.3 * abs(R2)));
            if pendR.valid
                [feat_A, feat_B, feat_C, ref_A_ms, last_STS_ms, last_SOS_ms, ref_C_ms] = ...
                    update_features_and_refs(feat_A, feat_B, feat_C, pendR, i, last_low_idx_R, dt, SX, SY, SV, ...
                    ref_A_ms, last_STS_ms, last_SOS_ms, ref_C_ms, ALPHA_REF);
                pendR.valid = false;
            end
        end
        
        if ((L2-L1)*(L1-L0)<=var_crit) && ((L2-L1)<0) && (L2>=cur_thres_up) && L_upcond
            L_upcond = false; L_tm = 0;
            cur_stance_angle_gate = (1.0 - ALPHA) * cur_stance_angle_gate + ALPHA * max(5.0, min(120.0, 0.3 * abs(L2)));
            if pendL.valid
                [feat_A, feat_B, feat_C, ref_A_ms, last_STS_ms, last_SOS_ms, ref_C_ms] = ...
                    update_features_and_refs(feat_A, feat_B, feat_C, pendL, i, last_low_idx_L, dt, SX, SY, SV, ...
                    ref_A_ms, last_STS_ms, last_SOS_ms, ref_C_ms, ALPHA_REF);
                pendL.valid = false;
            end
        end
        
        if (R2 > R_low_val+10), R_low_val = thres_down; end
        if ((R2-R1)*(R1-R0)<=var_crit) && ((R2-R1)>=0) && (R2<=R_low_val+2) && (is_moving(i)==1)
            R_low_val = R2; R_swing_tm = 0; last_low_idx_R = i;
        end
        if (L2 > L_low_val+10), L_low_val = thres_down; end
        if ((L2-L1)*(L1-L0)<=var_crit) && ((L2-L1)>0) && (L2<=L_low_val+2) && (is_moving(i)==1)
            L_low_val = L2; L_swing_tm = 0; last_low_idx_L = i;
        end
    end
end

function [fA, fB, fC, rA, lSTS, lSOS, rC] = update_features_and_refs(fA, fB, fC, pend, idx, low_idx, dt, SX, SY, SV, rA, lSTS, lSOS, rC, ALPHA)
    meas_T_ms = ((idx - low_idx) * dt) * 1000.0;
    
    fA.refs = [fA.refs; pend.refA * 1000]; fB.refs = [fB.refs; pend.refB * 1000]; fC.refs = [fC.refs; pend.refC * 1000];
    fA.measured = [fA.measured; meas_T_ms]; fB.measured = [fB.measured; meas_T_ms]; fC.measured = [fC.measured; meas_T_ms];
    fB.hist_SOS = [fB.hist_SOS; lSOS]; fB.hist_STS = [fB.hist_STS; lSTS];
    
    norm_vel = pend.vel / SV;
    
    valX = pend.vel * pend.refA; valY = pend.T_curr / pend.refA; 
    fA.x = [fA.x; valX/SX]; fA.y = [fA.y; valY/SY]; fA.vel = [fA.vel; norm_vel]; fA.labels = [fA.labels; pend.label];
    
    valX = pend.vel * pend.refB; valY = pend.T_curr / pend.refB; 
    fB.x = [fB.x; valX/SX]; fB.y = [fB.y; valY/SY]; fB.vel = [fB.vel; norm_vel]; fB.labels = [fB.labels; pend.label];
    
    valX = pend.vel * pend.refC; valY = pend.T_curr / pend.refC; 
    fC.x = [fC.x; valX/SX]; fC.y = [fC.y; valY/SY]; fC.vel = [fC.vel; norm_vel]; fC.labels = [fC.labels; pend.label];
    
    rA = meas_T_ms;
    if pend.label == 1, lSTS = meas_T_ms; else, lSOS = meas_T_ms; end
    rC = (1.0 - ALPHA) * rC + ALPHA * (lSTS + lSOS) / 2.0;
end

function bg_pts = get_boundary_points(mdl)
    ng = 100; xg=linspace(0,1.2,ng); yg=linspace(0,1.2,ng);
    [Xg,Yg]=meshgrid(xg,yg); [pred,~]=predict(mdl,[Xg(:),Yg(:)]); Z=reshape(pred,size(Xg));
    C=contourc(xg,yg,double(Z),[1.5 1.5]);
    bg_pts = []; k = 1;
    while k < size(C, 2), n = C(2, k); bg_pts = [bg_pts; C(:, k+1 : k+n)']; k = k + n + 1; end
end

function s = analyze_method_stats(feat, model, boundary_pts)
    s.dist_trans=[]; s.conf_trans=[]; s.dist_steady=[]; s.conf_steady=[];
    N = length(feat.labels); if N==0, return; end
    % Feature X (Distance), Feature Y 사용
    input_data = [feat.x, feat.y]; 
    [pred_label, scores] = predict(model, input_data); conf_values = max(scores, [], 2);
    dists = zeros(N, 1);
    for i=1:N
        pt = input_data(i,:);
        if ~isempty(boundary_pts)
            raw_dist = min(vecnorm(boundary_pts - pt, 2, 2));
            if pred_label(i) == feat.labels(i), dists(i) = -raw_dist; else, dists(i) = raw_dist; end
        end
    end
    for i=1:N
        is_trans = false; if i>1, if feat.labels(i)~=feat.labels(i-1), is_trans=true; end; end
        if is_trans, s.dist_trans=[s.dist_trans; dists(i)]; s.conf_trans=[s.conf_trans; conf_values(i)];
        else, s.dist_steady=[s.dist_steady; dists(i)]; s.conf_steady=[s.conf_steady; conf_values(i)]; end
    end
end

function [idx_list, type_list, upeak_R, upeak_L, dpeak_R, dpeak_L] = find_hc_indices(Rdeg, Ldeg, is_moving, fs, t_gap_ms, thres_down)
    N = length(Rdeg); idx_list=[]; type_list=[]; upeak_R=[]; upeak_L=[]; dpeak_R=[]; dpeak_L=[];
    cur_hc=10; cur_up=10; cur_gate=10; ALPHA=0.2; t_samp = t_gap_ms*(fs/1000);
    HC_tm=0; R_tm=0; L_tm=0; HC_up=false; R_up=false; L_up=false; R_low=thres_down; L_low=thres_down;
    
    for i=3:N
        HC_tm=HC_tm+1; R_tm=R_tm+1; L_tm=L_tm+1;
        R0=Rdeg(i-2); R1=Rdeg(i-1); R2=Rdeg(i); L0=Ldeg(i-2); L1=Ldeg(i-1); L2=Ldeg(i);
        if HC_tm>=t_samp, HC_up=true; end; if R_tm>=t_samp, R_up=true; end; if L_tm>=t_samp, L_up=true; end
        
        if HC_up && ((R2-L2)*(R1-L1)<=0)
            is_R = ((R2-R1)>=(L2-L1)); is_v=false; s_deg=0;
            if is_R, if (R2>=cur_hc)&&(L2>=cur_gate), is_v=true; s_deg=R2; end
            else,    if (L2>=cur_hc)&&(R2>=cur_gate), is_v=true; s_deg=L2; end; end
            if is_v, HC_up=false; HC_tm=0; idx_list=[idx_list;i]; type_list=[type_list; ternary(is_R,1,2)]; cur_hc=(1-ALPHA)*cur_hc+ALPHA*max(5,min(90,0.55*s_deg)); end
        end
        if ((R2-R1)*(R1-R0)<=0) && ((R2-R1)<0) && (R2>=cur_up) && R_up, R_up=false; R_tm=0; upeak_R=[upeak_R;i]; cur_gate=(1-ALPHA)*cur_gate+ALPHA*max(5,min(120,0.3*abs(R2))); end
        if ((L2-L1)*(L1-L0)<=0) && ((L2-L1)<0) && (L2>=cur_up) && L_up, L_up=false; L_tm=0; upeak_L=[upeak_L;i]; cur_gate=(1-ALPHA)*cur_gate+ALPHA*max(5,min(120,0.3*abs(L2))); end
        if R2>R_low+10, R_low=thres_down; end; if ((R2-R1)*(R1-R0)<=0)&&((R2-R1)>=0)&&(R2<=R_low+2)&&(is_moving(i)==1), R_low=R2; dpeak_R=[dpeak_R;i]; end
        if L2>L_low+10, L_low=thres_down; end; if ((L2-L1)*(L1-L0)<=0)&&((L2-L1)>0)&&(L2<=L_low+2)&&(is_moving(i)==1), L_low=L2; dpeak_L=[dpeak_L;i]; end
    end
end

function y = LPF(x, fs, fc), y=zeros(length(x),1); T=1/fs; RC=1/(2*pi*fc); a=T/(T+RC); for i=2:length(x), y(i)=(1-a)*y(i-1)+a*x(i); end, end

function label = prompt_labels(M)
    % 라벨 입력 다이얼로그 (1=SOS, 2=STS)
    d = dialog('Position',[100 100 520 240],'Name','Label Input');

    uicontrol('Parent',d,'Style','text','Position',[20 190 480 30], ...
        'HorizontalAlignment','left', ...
        'String',sprintf('Total HC: %d (1=SOS, 2=STS)', M));

    e = uicontrol('Parent',d,'Style','edit','Position',[20 140 480 40], ...
        'HorizontalAlignment','left', ...
        'String',num2str(ones(1, M)));

    % 자동 입력 버튼: 1 1 2 2 1 1 2 1
    uicontrol('Parent',d,'Style','pushbutton','Position',[20 95 240 35], ...
        'String','Auto: 1 1 2 2 1 1 2 1', ...
        'Callback', @(~,~) set(e,'String','1 1 2 2 1 1 2 1'));

    uicontrol('Parent',d,'Style','pushbutton','Position',[300 20 100 45], ...
        'String','OK', 'Callback', @(~,~) uiresume(d));
    uicontrol('Parent',d,'Style','pushbutton','Position',[420 20 80 45], ...
        'String','Cancel', 'Callback', @(~,~) delete(d));

    uiwait(d);

    if isvalid(d)
        str = get(e,'String');
        label = str2num(str)'; %#ok<ST2NM>
        delete(d);
    else
        label = [];
    end
end

function out = ternary(cond, a, b), if cond, out=a; else, out=b; end, end

function boundaryXY = parse_contourc_to_xy(C)
    boundaryXY = zeros(0,2);
    k = 1;
    while k < size(C,2)
        n = C(2,k);
        pts = C(:, k+1:k+n).';
        boundaryXY = [boundaryXY; pts]; %#ok<AGROW>
        k = k + n + 1;
    end
end

function d = min_distance_to_boundary(boundaryXY, pts)
    if isempty(boundaryXY) || isempty(pts)
        d = nan(size(pts,1),1);
        return;
    end
    d = zeros(size(pts,1),1);
    for i = 1:size(pts,1)
        dif = boundaryXY - pts(i,:);
        d(i) = min(sqrt(sum(dif.^2, 2)));
    end
end

function print_boundary_distance(titleStr, idxMask, dist)
    idx = find(idxMask);
    fprintf('\n[%s]\n', titleStr);
    if isempty(idx)
        fprintf('  (no points)\n');
        return;
    end
    fprintf('  i\tdist_to_boundary\n');
    for t = 1:numel(idx)
        i = idx(t);
        fprintf('  %d\t%.4f\n', i, dist(i));
    end
    fprintf('  Summary: N=%d, mean=%.4f, max=%.4f\n', numel(idx), mean(dist(idx),'omitnan'), max(dist(idx)));
end



function tag = get_subject_tag(fname)
    % Extract 3 characters between first '_' and second '_'
    tag = fname;
    us = strfind(fname, '_');
    if numel(us) >= 2
        mid = fname(us(1)+1:us(2)-1);
        if numel(mid) >= 3
            tag = mid(1:3);
        else
            tag = mid;
        end
    end
end