%% Refdata_Combine_Visualize_Final_v4.m
% 기능:
% 1. 데이터 로드 및 병합
% 2. Feature X 변환 (Raw Velocity -> Distance)
% 3. LOF 기반 Outlier Removal
% 4. [Analysis 4] Norm. Velocity vs Norm. Feat Y
% 5. 데이터 밸런싱 (Ref 20 / Val 5)
% 6. [Analysis 1] Scatter Plot (SOS=Blue, STS=Red) - [색상 변경됨]
% 7. [Analysis 2] Condition Trajectory (유지)
% 8. [Analysis 3] Subject Avg+Std (유지)
% 9. Optimal K 탐색 (Safety Margin Sum 기준)
% 10. [Analysis 6] KNN Map (SOS=Blue, STS=Red) - [색상 변경됨]
% 11. [Per-Subject] 피험자별 분포 (Points Colored) - [색상 변경됨]

clear all; close all; clc;

% =========================================================================
% 1. 설정 (경로 및 파라미터)
% =========================================================================

% [수정 필요] 데이터 폴더 경로
targetDir = 'C:\Users\ChanyoungKo\OneDrive - 한국과학기술원 기계공학과\Kyle\KAIST\ExoLab\성과\Publication\Ongoing\1저자_2025_RAL_StairMode\2nd_Submit\Revision_Data\Ref_dataset';

% [수정 필요] 피험자 나이 정보 (ID, 나이)
SUBJECT_INFO = {
    'cyk', 30;
    'kso', 60;
    'eek', 57;
    'grandpa', 95;
};

% 파라미터
TARGET_REF_NUM    = 25;   % Ref Dataset 목표 개수
TARGET_VAL_NUM    = 5;    % Validation Dataset 목표 개수
KNN_K_DEFAULT     = 9;    % 기본 K
RNG_SEED          = 42;   % 랜덤 시드

% [설정] 이상치 제거 파라미터 (LOF)
LOF_K             = 20;   % 이웃 수
LOF_THRESHOLD     = 1.5;  % LOF 점수 기준

% [색상 설정] SOS=Blue, STS=Red (요청사항 반영)
color_SOS = [0 0 1]; % Blue
color_STS = [1 0 0]; % Red

% [라벨 매핑] 데이터 라벨 값 기준 (요청: STS=Red, SOS=Blue)
LABEL_STS = 1;
LABEL_SOS = 2;
white     = [1 1 1];

% =========================================================================
% 2. 데이터 로드 및 정보 추출
% =========================================================================

if ~exist(targetDir, 'dir')
    error('경로를 찾을 수 없습니다: %s', targetDir);
end

files = dir(fullfile(targetDir, '*_ref.mat'));
if isempty(files)
    error('파일이 없습니다.');
end

fprintf('------------------------------------------------\n');
fprintf('데이터 병합 시작 (총 %d개 파일)\n', length(files));

ref_all = [];
subjectList = {}; 
conditionList = {}; 

subj_map_keys = SUBJECT_INFO(:,1);
subj_map_ages = cell2mat(SUBJECT_INFO(:,2));

for k = 1:length(files)
    fileName = files(k).name;
    fullPath = fullfile(files(k).folder, fileName);
    S = load(fullPath);
    
    loadedData = []; rawYData = []; 
    if isfield(S, 'ref_raw'), loadedData = S.ref_raw; 
    elseif isfield(S, 'ref_data'), loadedData = S.ref_data;
    else
        fields = fieldnames(S);
        for f = 1:numel(fields)
            val = S.(fields{f});
            if isa(val, 'double') && size(val, 2) == 3 && ~contains(fields{f}, 'type2')
                loadedData = val; break;
            end
        end
    end
    
    if isfield(S, 'ref_type2'), rawYData = S.ref_type2(:, 2); 
    else, if ~isempty(loadedData), rawYData = nan(size(loadedData, 1), 1); end, end
    
    if ~isempty(loadedData)
        if length(rawYData) ~= size(loadedData, 1)
            warning('데이터 길이 불일치: %s', fileName); rawYData = nan(size(loadedData, 1), 1);
        end

        condName = 'unknown';
        tokens_bpm = regexp(fileName, '(\d+)bpm', 'tokens');
        tokens_speed = regexpi(fileName, '(slow|normal|fast)', 'tokens');
        if ~isempty(tokens_bpm), condName = [tokens_bpm{1}{1}, 'bpm']; 
        elseif ~isempty(tokens_speed), condName = lower(tokens_speed{1}{1}); 
        else, condName = 'default'; end
        
        if ~ismember(condName, conditionList), conditionList{end+1} = condName; end
        cond_idx = find(strcmp(conditionList, condName)); 
        
        tokens_subj = regexp(fileName, '_([a-zA-Z]{3})', 'tokens', 'once');
        if ~isempty(tokens_subj), subj_str = tokens_subj{1};
        elseif contains(lower(fileName), 'grandpa'), subj_str = 'grandpa';
        else, subj_str = 'UNK'; end
        
        if ~ismember(subj_str, subjectList), subjectList{end+1} = subj_str; end
        subj_idx = find(strcmp(subjectList, subj_str));
        
        map_idx = find(strcmp(subj_map_keys, subj_str));
        if ~isempty(map_idx), age_val = subj_map_ages(map_idx); else, age_val = NaN; end
        
        N = size(loadedData, 1);
        dataWithInfo = [loadedData, repmat(cond_idx,N,1), repmat(subj_idx,N,1), repmat(age_val,N,1), rawYData];
        ref_all = [ref_all; dataWithInfo];
        fprintf(' + [%d/%d] %s (Subj:%s, Cond:%s, N=%d)\n', k, length(files), fileName, subj_str, condName, N);
    end
end
if isempty(ref_all), error('데이터 없음'); end

% =========================================================================
%  [추가 기능] 50bpm 조건에서 T_swing 평균 계산 (SOS / STS) - (병합 직후, LOF 전)
%  + (추가) ref_all(:,3)에서 이전=2, 현재=1 인 row는 STS 평균에서 제외
% =========================================================================
fprintf('\n------------------------------------------------\n');
fprintf('[추가 기능] 50bpm 조건의 T_swing 평균 계산 (병합 직후)\n');

if size(ref_all, 2) < 7 || all(isnan(ref_all(:,7)))
    warning('T_swing 데이터(ref_type2(:,2))가 없어 평균을 계산할 수 없습니다.');
else
    condName_target = '50bpm';
    cond_idx_target = find(strcmp(conditionList, condName_target), 1);

    if isempty(cond_idx_target)
        warning('조건 "%s"를 conditionList에서 찾지 못했습니다. (파일명에 50bpm이 포함되어야 합니다.)', condName_target);
    else
        idx_cond = (ref_all(:,4) == cond_idx_target);
        t_swing_all = ref_all(:,7);

        % 전이(이전=2, 현재=1)인 row 찾기
        prev_label = [NaN; ref_all(1:end-1, 3)];
        curr_label = ref_all(:, 3);
        idx_trans_2to1 = (prev_label == 2) & (curr_label == 1);

        % SOS는 그대로
        T_swing_SOS = t_swing_all(idx_cond & (ref_all(:,3) == LABEL_SOS));

        % STS는 전이(2->1) row 제외
        T_swing_STS = t_swing_all(idx_cond & (ref_all(:,3) == LABEL_STS) & ~idx_trans_2to1);

        T_swing_SOS_mean = mean(T_swing_SOS, 'omitnan');
        T_swing_STS_mean = mean(T_swing_STS, 'omitnan');

        fprintf(' - 50bpm: T_swing_SOS mean = %.6f (N=%d)\n', T_swing_SOS_mean, numel(T_swing_SOS));
        fprintf(' - 50bpm: T_swing_STS mean = %.6f (N=%d)\n', T_swing_STS_mean, numel(T_swing_STS));
        fprintf('   (참고) STS에서 제외된 2->1 전이 row 수 = %d\n', sum(idx_cond & idx_trans_2to1 & (ref_all(:,3) == LABEL_STS)));
    end
end

% =========================================================================
% 3. Outlier Removal (LOF) & Integrated Visualization
% =========================================================================
fprintf('\n------------------------------------------------\n');
fprintf('Outlier Removal (LOF > %.1f, K=%d)...\n', LOF_THRESHOLD, LOF_K);

ref_clean_groups = cell(2,1);
keep_mask_all = false(size(ref_all,1),1);  % [추가] outlier 제거 후 유지 마스크 (ref_all 기준)

figure('Name', 'LOF Outlier Removal Result (Combined)', 'Color', 'w', 'Position', [100, 100, 900, 600]);
hold on; grid on;

% [색상] SOS(1)=Blue, STS(2)=Red
% labels_str 순서 주의: Label 값에 맞게 매핑
colors = cell(1,2);
markers = cell(1,2);
labels_str = cell(1,2);

% Label 매핑 (STS=LABEL_STS, SOS=LABEL_SOS)
colors{LABEL_STS} = color_STS; markers{LABEL_STS} = 's'; labels_str{LABEL_STS} = 'STS';
colors{LABEL_SOS} = color_SOS; markers{LABEL_SOS} = 'o'; labels_str{LABEL_SOS} = 'SOS';

for L = 1:2
    idxL = (ref_all(:,3) == L); DataL = ref_all(idxL, :); 
    if isempty(DataL), continue; end

    XY_raw = DataL(:, 1:2); N_pts = size(XY_raw, 1);
    current_k = LOF_K;
    if N_pts <= current_k + 1
        if N_pts > 3, current_k = N_pts - 2; 
        else, ref_clean_groups{L} = DataL; continue; end
    end

    med_val = median(XY_raw, 1); mad_val = median(abs(XY_raw - med_val), 1); mad_val(mad_val==0)=1;
    XY_robust = (XY_raw - med_val) ./ mad_val;

    [indices, dists] = knnsearch(XY_robust, XY_robust, 'K', current_k + 1);
    knn_idx = indices(:, 2:end); knn_dist = dists(:, 2:end); k_dist_obj = knn_dist(:, end);
    
    reach_dist = zeros(N_pts, current_k);
    for i = 1:N_pts
        neighbors = knn_idx(i, :);
        dist_to_neighbors = knn_dist(i, :)';
        reach_dist(i, :) = max([k_dist_obj(neighbors), dist_to_neighbors], [], 2)';
    end
    lrd = 1 ./ (mean(reach_dist, 2) + eps);
    
    lof_score = zeros(N_pts, 1);
    for i = 1:N_pts
        lof_score(i) = mean(lrd(knn_idx(i, :))) / lrd(i);
    end
    is_keep = lof_score <= LOF_THRESHOLD;
    keep_mask_all(idxL) = is_keep;  % [추가] ref_all 기준 유지 마스크 저장
    
    scatter(XY_raw(is_keep, 1), XY_raw(is_keep, 2), 40, colors{L}, markers{L}, 'filled', 'MarkerFaceAlpha', 0.5, 'DisplayName', sprintf('%s Normal', labels_str{L}));
    if sum(~is_keep) > 0
        scatter(XY_raw(~is_keep, 1), XY_raw(~is_keep, 2), 80, colors{L}, 'x', 'LineWidth', 2, 'DisplayName', sprintf('%s Outlier', labels_str{L}));
    end
    ref_clean_groups{L} = DataL(is_keep, :);

    % [추가 출력] Subject별 Outlier 제거 전/후 개수 (Gait mode=Label별)
    u_subj_in_L = unique(DataL(:,5));
    fprintf('   Label %d (N=%d): 유지 %d / 제거 %d\n', L, N_pts, sum(is_keep), sum(~is_keep));
    for ss = 1:numel(u_subj_in_L)
        sid = u_subj_in_L(ss);
        idxS = (DataL(:,5) == sid);
        nS_total = sum(idxS);
        nS_keep  = sum(idxS & is_keep);
        nS_rm    = sum(idxS & ~is_keep);
    
        if sid <= length(subjectList), sName = subjectList{sid};
        else, sName = sprintf('ID%d', sid); end
    
        fprintf('      - %s (N=%d): 유지 %d / 제거 %d\n', sName, nS_total, nS_keep, nS_rm);
    end
end

% =========================================================================
% [추가 출력] Subject별 / 속도(Condition)별 / Gait mode별
%   - 초기 개수 (outlier 제거 전)
%   - 제거 개수
%   - 제거 후 유지 개수
% =========================================================================
fprintf('\n[Outlier Removal 상세] Subject / Condition / Gait mode별 개수\n');

u_subj_all = unique(ref_all(:,5));
u_cond_all = unique(ref_all(:,4));

for ss = 1:numel(u_subj_all)
    sid = u_subj_all(ss);
    if sid <= length(subjectList), sName = subjectList{sid};
    else, sName = sprintf('ID%d', sid); end

    for cc = 1:numel(u_cond_all)
        cid = u_cond_all(cc);

        % 해당 subject-condition에 데이터가 있는지 먼저 확인
        idxSC = (ref_all(:,5) == sid) & (ref_all(:,4) == cid);
        if ~any(idxSC), continue; end

        if cid <= length(conditionList), cName = conditionList{cid};
        else, cName = sprintf('COND%d', cid); end

        % Label별로 1줄씩 출력 (STS / SOS)
        for L = 1:2
            idxSCL = idxSC & (ref_all(:,3) == L);
            n0 = sum(idxSCL);
            if n0 == 0, continue; end

            n_keep = sum(idxSCL & keep_mask_all);
            n_rm   = n0 - n_keep;

            fprintf(' [%s] %-8s | %s : 초기 %3d / 제거 %3d / 이후 %3d\n', ...
                sName, cName, labels_str{L}, n0, n_rm, n_keep);
        end
    end
end

xlabel('Feature X (Distance)'); ylabel('Feature Y'); title('Outlier Detection (SOS vs STS)');
legend('show', 'Location', 'best'); hold off;

clean_data_all = vertcat(ref_clean_groups{:});

% Scaling Factors
mx_feat_x = max(clean_data_all(:,1)); if mx_feat_x==0, mx_feat_x=1; end
mx_feat_y = max(clean_data_all(:,2)); if mx_feat_y==0, mx_feat_y=1; end
raw_vel_all = clean_data_all(:,1) ./ clean_data_all(:,7); raw_vel_all(isinf(raw_vel_all)) = NaN;
mx_vel = max(raw_vel_all, [], 'omitnan'); if isempty(mx_vel)||mx_vel==0, mx_vel=1; end


%% =========================================================================
%  [Analysis 4] Norm. Velocity vs Norm. Feature Y (KNN 포함)
% =========================================================================
fprintf('\n[분석 4] Norm. Velocity vs Norm. Feature Y (with KNN)\n');
if size(clean_data_all, 2) < 7 || all(isnan(clean_data_all(:,7)))
    warning('Raw Time 데이터가 없어 분석 4를 건너뜁니다.');
else
    raw_dist_vals = clean_data_all(:,1); raw_t_vals = clean_data_all(:,7);
    raw_y_vals = clean_data_all(:,2); labels = clean_data_all(:,3); cond_vals = clean_data_all(:,4);
    
    norm_vel = (raw_dist_vals ./ raw_t_vals) / mx_vel; norm_y = raw_y_vals / mx_feat_y;
    valid_idx = isfinite(norm_vel) & isfinite(norm_y);
    
    mdl4 = fitcknn([norm_vel(valid_idx), norm_y(valid_idx)], labels(valid_idx), ...
        'NumNeighbors', KNN_K_DEFAULT, 'DistanceWeight', 'inverse');
    
    xg4 = linspace(0, 1, 400); yg4 = linspace(0, 1, 400); [Xg4, Yg4] = meshgrid(xg4, yg4);
    [predLbl4, scores4] = predict(mdl4, [Xg4(:), Yg4(:)]);
    Z4 = reshape(predLbl4, size(Xg4));
    
    RGB4 = zeros([size(Z4), 3]);
    % [색상] STS=Red, SOS=Blue (라벨 매핑 반영)
    mask1 = (Z4==LABEL_SOS); mask2 = (Z4==LABEL_STS); maskO = ~(mask1|mask2);
    for c=1:3, RGB4(:,:,c) = color_SOS(c)*mask1 + color_STS(c)*mask2 + 0.9*white(c)*maskO; end
    
    figure('Name', 'Analysis 4: Norm. Vel vs Norm. Feat Y', 'Color', 'w'); hold on;
    imagesc(xg4, yg4, RGB4, 'AlphaData', 0.18); set(gca, 'YDir', 'normal');
    
    % 데이터 포인트 색상 반영
    scatter(norm_vel(valid_idx & labels==LABEL_SOS), norm_y(valid_idx & labels==LABEL_SOS), 30, color_SOS, 'filled', 'MarkerFaceAlpha', 0.5, 'DisplayName', 'SOS');
    scatter(norm_vel(valid_idx & labels==LABEL_STS), norm_y(valid_idx & labels==LABEL_STS), 30, color_STS, 'filled', 'MarkerFaceAlpha', 0.5, 'DisplayName', 'STS');
    
    xlabel('Norm Velocity'); ylabel('Norm Feature Y'); title('Analysis 4'); axis square;
end


% =========================================================================
% 5. 데이터 밸런싱 (Ref Set 20개 + Validation Set 5개)
% =========================================================================
fprintf('\n데이터 밸런싱 및 검증 세트 추출 (Ref 20 / Val 5)...\n');
rng(RNG_SEED); 
ref_balanced = []; val_balanced = [];
unique_subjs_all = unique(clean_data_all(:, 5)); 

for s_i = 1:length(unique_subjs_all)
    sid = unique_subjs_all(s_i);
    u_conds_s = unique(clean_data_all(clean_data_all(:,5)==sid, 4));
    if sid <= length(subjectList), sName = subjectList{sid}; else, sName = sprintf('ID%d', sid); end
    
    for c_i = 1:length(u_conds_s)
        c_id = u_conds_s(c_i); 
        if c_id <= length(conditionList), cName = conditionList{c_id}; else, cName = sprintf('C%d', c_id); end
        
        for l_val = 1:2
            lStr = ternary(l_val==LABEL_SOS, 'SOS', 'STS');
            mask = (clean_data_all(:,3)==l_val) & (clean_data_all(:,4)==c_id) & (clean_data_all(:,5)==sid);
            subset = clean_data_all(mask, :);
            n_total = size(subset, 1);
            
            perm_idx = randperm(n_total);
            n_ref = min(n_total, TARGET_REF_NUM);
            ref_balanced = [ref_balanced; subset(perm_idx(1:n_ref), :)];
            
            idx_rem = perm_idx(n_ref+1:end);
            if ~isempty(idx_rem)
                n_val = min(length(idx_rem), TARGET_VAL_NUM);
                val_balanced = [val_balanced; subset(idx_rem(1:n_val), :)];
            end
            fprintf(' [%-4s] %-8s | %-3s : %3d -> Ref %2d / Val %2d\n', sName, cName, lStr, n_total, n_ref, min(length(idx_rem), TARGET_VAL_NUM));
        end
    end
end

% Scaling
ref_norm = ref_balanced;
ref_norm(:,1) = ref_balanced(:,1)/mx_feat_x; ref_norm(:,2) = ref_balanced(:,2)/mx_feat_y;
val_norm = val_balanced;
if ~isempty(val_norm)
    val_norm(:,1) = val_balanced(:,1)/mx_feat_x; val_norm(:,2) = val_balanced(:,2)/mx_feat_y;
end


%% =========================================================================
%  Analysis 1, 2, 3
% =========================================================================
fprintf('\n[시각화] Analysis 1 (Scatter), 2 (Trajectory), 3 (Avg+Std)...\n');

X_plot = ref_norm(:, 1);
Y_plot = ref_norm(:, 2);
L_plot = ref_norm(:, 3);
C_plot = ref_norm(:, 4);
S_plot = ref_norm(:, 5);

% [Analysis 1] Group by Gait Mode (Scatter 방식, SOS=Blue, STS=Red)
figure('Name', 'Analysis 1: Group by Gait Mode', 'Color', 'w'); hold on; grid on;
% SOS (L=2) -> Blue
idx1 = (L_plot == LABEL_SOS);
scatter(X_plot(idx1), Y_plot(idx1), 40, color_SOS, 'o', 'filled', 'DisplayName', 'SOS');
% STS (L=1) -> Red
idx2 = (L_plot == LABEL_STS);
scatter(X_plot(idx2), Y_plot(idx2), 40, color_STS, 's', 'filled', 'DisplayName', 'STS');

xlabel('Norm. Feature X (Dist)'); ylabel('Norm. Feature Y');
title('Analysis 1: Gait Mode (Scatter)'); legend('show'); axis square; xlim([0 1]); ylim([0 1]);


% [Analysis 2] Group by Condition (Per Subject Trajectory) - 유지
figure('Name', 'Analysis 2: Group by Condition (Per Subject)', 'Color', 'w'); hold on; grid on;
u_subjs = unique(S_plot);
u_conds = unique(C_plot); 
cmap_cond = [0 1 1; 0 0.8 0; 1 0 1]; % Cyan(Slow), Green(Normal), Magenta(Fast)

for s_i = 1:length(u_subjs)
    sid = u_subjs(s_i);
    for l_val = 1:2 
        x_means = []; y_means = [];
        for c_i = 1:length(u_conds)
            cid = u_conds(c_i);
            idx = (S_plot==sid) & (C_plot==cid) & (L_plot==l_val);
            if sum(idx) > 0
                mx = mean(X_plot(idx), 'omitnan'); my = mean(Y_plot(idx), 'omitnan');
                sx = std(X_plot(idx), 'omitnan'); sy = std(Y_plot(idx), 'omitnan');
                if l_val==LABEL_SOS, mk='o'; else, mk='s'; end
                
                col = cmap_cond(mod(c_i-1, size(cmap_cond,1))+1, :);
                plot(mx, my, mk, 'Color', col, 'MarkerFaceColor', col, 'DisplayName', sprintf('C%d', cid));
                line([mx-sx, mx+sx], [my, my], 'Color', col, 'HandleVisibility', 'off');
                line([mx, mx], [my-sy, my+sy], 'Color', col, 'HandleVisibility', 'off');
                x_means = [x_means, mx]; y_means = [y_means, my];
            end
        end
        if length(x_means) > 1
            plot(x_means, y_means, '-', 'Color', [0.5 0.5 0.5 0.5], 'HandleVisibility', 'off');
        end
    end
end
xlabel('Norm. Feature X'); ylabel('Norm. Feature Y');
title('Analysis 2: Condition Effect (Per Subject)'); 
for c=1:length(u_conds)
    plot(nan, nan, 'ko', 'MarkerFaceColor', cmap_cond(mod(c-1,3)+1,:), 'DisplayName', conditionList{u_conds(c)});
end
legend('show', 'Location', 'best'); axis square; xlim([0 1]); ylim([0 1]);


% [Analysis 3] Group by Subject (Avg + Std) - 유지
figure('Name', 'Analysis 3: Group by Subject', 'Color', 'w'); hold on; grid on;
cmap_subj = lines(length(u_subjs)); 
for i = 1:length(u_subjs)
    idx = (S_plot == u_subjs(i));
    if u_subjs(i) <= length(subjectList), s_name = subjectList{u_subjs(i)}; 
    else, s_name = sprintf('ID%d', u_subjs(i)); end
    
    mx = mean(X_plot(idx), 'omitnan'); my = mean(Y_plot(idx), 'omitnan');
    sx = std(X_plot(idx), 'omitnan'); sy = std(Y_plot(idx), 'omitnan');
    
    color = cmap_subj(i,:);
    plot(mx, my, 'o', 'Color', color, 'MarkerFaceColor', color, 'MarkerSize', 8, 'DisplayName', s_name);
    line([mx-sx, mx+sx], [my, my], 'Color', color, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    line([mx, mx], [my-sy, my+sy], 'Color', color, 'LineWidth', 1.5, 'HandleVisibility', 'off');
end
xlabel('Norm. Feature X'); ylabel('Norm. Feature Y');
title('Analysis 3: Subject Distribution (Avg \pm Std)'); legend('show', 'Location', 'bestoutside'); axis square; xlim([0 1]); ylim([0 1]);


% =========================================================================
% 6. Optimal K 탐색 (기준: Sum of Safety Margins)
% =========================================================================
fprintf('\n[최적 K값 탐색] 기준: Sum of Safety Margins\n');
if isempty(val_norm)
    best_K = KNN_K_DEFAULT;
else
    N_ref = size(ref_norm, 1);
    max_K = floor(sqrt(N_ref)); if mod(max_K, 2)==0, max_K = max_K - 1; end
    candidate_Ks = 1:2:max_K;
    
    log_losses = zeros(size(candidate_Ks)); sum_margins = zeros(size(candidate_Ks));
    X_tr = ref_norm(:, 1:2); Y_tr = ref_norm(:, 3);
    X_val = val_norm(:, 1:2); Y_val = val_norm(:, 3);
    
    dist_enemy = zeros(size(X_val, 1), 1);
    for i=1:size(X_val,1)
        [~,d] = knnsearch(X_tr(Y_tr~=Y_val(i),:), X_val(i,:), 'K', 1); dist_enemy(i)=d;
    end
    
    fprintf('K | Loss | Sum(Margin)\n');
    for i=1:length(candidate_Ks)
        k_val = candidate_Ks(i);
        mdl = fitcknn(X_tr, Y_tr, 'NumNeighbors', k_val, 'DistanceWeight', 'inverse', 'Standardize', false);
        [~, p] = predict(mdl, X_val);
        
        prob_true = zeros(length(Y_val), 1);
        for j=1:length(Y_val), prob_true(j) = p(j, Y_val(j)); end
        log_losses(i) = -mean(log(prob_true + eps));
        
        dist_kth = zeros(size(X_val, 1), 1);
        for j=1:size(X_val,1)
            X_fr = X_tr(Y_tr==Y_val(j),:);
            [~,d] = knnsearch(X_fr, X_val(j,:), 'K', min(k_val, size(X_fr,1)));
            dist_kth(j) = d(end);
        end
        sum_margins(i) = sum(dist_enemy - dist_kth);
        fprintf('%d | %.4f | %.4f\n', k_val, log_losses(i), sum_margins(i));
    end
    
    min_loss = min(log_losses);
    best_cands = candidate_Ks(log_losses <= min_loss + 1e-6);
    safe_cands = best_cands(sum_margins(ismember(candidate_Ks, best_cands)) > 0);
    
    if isempty(safe_cands)
        [~, idx] = max(sum_margins); best_K = candidate_Ks(idx);
    else
        best_K = max(safe_cands);
    end
    fprintf('>> Best K: %d\n', best_K);
end


% =========================================================================
% 7. [Analysis 6] KNN Map (Best K 적용) - [색상 수정: SOS=Blue, STS=Red]
% =========================================================================
fprintf('\n[Analysis 6] Final KNN Map (K=%d)...\n', best_K);

ref_final_scaled = ref_norm;

disp('===== MATLAB ref_dataset DEBUG =====');
disp(['best_K = ', num2str(best_K)]);
disp(['ref count = ', num2str(size(ref_final_scaled,1))]);
disp(['STS count = ', num2str(sum(ref_norm(:,3)==1))]);
disp(['SOS count = ', num2str(sum(ref_norm(:,3)==2))]);

% disp('first 10 ref points [x y label]:');
% disp(ref_norm(1:10, :));
% 
% disp('last 10 ref points [x y label]:');
% disp(ref_norm(end-9:end, :));
% disp('====================================');

scale_factor = [mx_feat_x, mx_feat_y];

save(fullfile(targetDir, 'refdataset_merged_scaled.mat'), ...
    'ref_final_scaled', 'scale_factor', 'subjectList', 'conditionList', 'best_K');

mdl_final = fitcknn(ref_norm(:,1:2), ref_norm(:,3), ...
    'NumNeighbors', best_K, ...
    'Distance', 'euclidean', ...
    'DistanceWeight', 'inverse', ...
    'BreakTies', 'smallest', ...
    'Standardize', false);

xg = linspace(0, 1, 400); yg = linspace(0, 1, 400); [Xg,Yg] = meshgrid(xg,yg);
[predLbl, scores] = predict(mdl_final, [Xg(:),Yg(:)]); Z = reshape(predLbl, size(Xg));

RGB = zeros([size(Z),3]); 
% [색상] SOS=Blue, STS=Red (라벨 매핑 반영)
mask_sos = (Z==LABEL_SOS); mask_sts = (Z==LABEL_STS); maskO = ~(mask_sos|mask_sts);
for c=1:3, RGB(:,:,c) = color_SOS(c)*mask_sos + color_STS(c)*mask_sts + 0.9*white(c)*maskO; end

classes = mdl_final.ClassNames;
idx_sos_cls = find(classes==LABEL_SOS,1);
idx_sts_cls = find(classes==LABEL_STS,1);
D = nan(size(Xg));
if ~isempty(idx_sos_cls) && ~isempty(idx_sts_cls)
    D = reshape(scores(:,idx_sos_cls) - scores(:,idx_sts_cls), size(Xg));
end

% figure('Name', sprintf('Analysis 6: KNN Map (K=%d)', best_K), 'Color','w'); hold on;
% imagesc(xg, yg, RGB, 'AlphaData', 0.18); set(gca,'YDir','normal');
% if ~all(isnan(D),'all'), contour(Xg, Yg, D, [0 0], 'k--', 'LineWidth', 1.5); end
% 
% % [수정됨] 데이터 포인트 색상 반영 (SOS=Blue, STS=Red)
% idx_sos = (ref_norm(:,3)==LABEL_SOS); idx_sts = (ref_norm(:,3)==LABEL_STS);
% scatter(ref_norm(idx_sos,1), ref_norm(idx_sos,2), 40, color_SOS, 'filled', 'MarkerFaceAlpha', 0.6);
% scatter(ref_norm(idx_sts,1), ref_norm(idx_sts,2), 40, color_STS, 'filled', 'MarkerFaceAlpha', 0.6);
% 
% xlabel('Norm Feature X'); ylabel('Norm Feature Y'); title(sprintf('KNN Map (K=%d)', best_K)); axis square;
SHOW_CONTOUR = false;  % 비교용: 일단 점선 끄기

figure('Name', sprintf('Analysis 6: KNN Map (K=%d)', best_K), 'Color','w'); hold on;
imagesc(xg, yg, RGB, 'AlphaData', 0.18); set(gca,'YDir','normal');

if SHOW_CONTOUR
    if ~all(isnan(D),'all')
        contour(Xg, Yg, D, [0 0], 'k--', 'LineWidth', 1.5);
    end
end

% 데이터 포인트
idx_sos = (ref_norm(:,3)==LABEL_SOS); idx_sts = (ref_norm(:,3)==LABEL_STS);
scatter(ref_norm(idx_sos,1), ref_norm(idx_sos,2), 40, color_SOS, 'filled', 'MarkerFaceAlpha', 0.6);
scatter(ref_norm(idx_sts,1), ref_norm(idx_sts,2), 40, color_STS, 'filled', 'MarkerFaceAlpha', 0.6);

xlabel('Norm Feature X'); ylabel('Norm Feature Y');
title(sprintf('KNN Label Map Only (K=%d)', best_K));
axis square;
xlim([0 1]); ylim([0 1]);

testPts = [0.20 0.75;
           0.50 0.50;
           0.80 0.45;
           0.15 0.90;
           0.95 0.49];
testPred = predict(mdl_final, testPts);
% disp('MATLAB testPred (label):'); disp([testPts, testPred]);


% =========================================================================
% 8. [Per-Subject] 피험자별 분포 확인
% =========================================================================
n_subjects = length(subjectList);
n_cols = ceil(sqrt(n_subjects)); n_rows = ceil(n_subjects / n_cols);
figure('Name', 'Per-Subject Distribution', 'Color', 'w');
for s_i = 1:n_subjects
    subplot(n_rows, n_cols, s_i); hold on;
    imagesc(xg,yg,RGB, 'AlphaData', 0.18); set(gca,'YDir','normal');
    if ~all(isnan(D),'all'), contour(Xg,Yg,D,[0 0],'k--','LineWidth',1.0); end
    
    idx_s = (ref_norm(:,5) == s_i);
    % [수정됨] 데이터 포인트 색상 반영
    sub_data = ref_norm(idx_s, :);
    if ~isempty(sub_data)
        scatter(sub_data(sub_data(:,3)==LABEL_SOS,1), sub_data(sub_data(:,3)==LABEL_SOS,2), 30, color_SOS, 'filled');
        scatter(sub_data(sub_data(:,3)==LABEL_STS,1), sub_data(sub_data(:,3)==LABEL_STS,2), 30, color_STS, 'filled');
    end
    title(subjectList{s_i}); axis square; xlim([0 1]); ylim([0 1]);
end

function out = ternary(cond, a, b), if cond, out=a; else, out=b; end, end
