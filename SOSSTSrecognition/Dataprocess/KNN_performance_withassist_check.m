% compare_realtime_vs_offline_HC_with_manual_labels.m
%
% 
% - CSV  realtime (s_norm_vel_HC, s_norm_T_HC, s_gait_mode)
% - Refdata_process_many.m   offline  (s_norm_vel_HC, s_norm_T_HC)
% -  offline  label Refdata_process_many.m  UI  
%   .
%
% / 
% - offline feature    Refdata_process_many.m   
% - HC/upeak/dpeak  plot 
% - offline (outlier)   (Refdata_process_many embedded deletion tool )
% - offline  label  Refdata_process_many.m  embedded labeling UI
%   (preset: all 1, all 2, all 2 last 1 + skip)
%
% (offline feature 0) 
% -   0:
%   (1) CSV threshold/t_gap    () 
%   (2)  0  sweep feature 1   
%
% :   MATLAB 

clear; close all; clc;

%% 1) CSV (multi-select)
[csvFiles, csvPath] = uigetfile({'*.csv','CSV Files (*.csv)'}, 'Select realtime log CSV(s)', 'MultiSelect','on');
if isequal(csvFiles,0), error('CSV not selected.'); end
if ischar(csvFiles)
    csvFiles = {csvFiles};
end


% (added) accumulate results across multiple CSVs
result_all = [];
result_file = {};
result_subject = {};
result_header = { ...
    'offline_feature_X','offline_feature_Y','offline_boundary_dist','offline_label', ...
    'online_feature_X','online_feature_Y','online_boundary_dist','online_label','success_rate'};

for iFile = 1:numel(csvFiles)
    csvFullPath = fullfile(csvPath, csvFiles{iFile});
    fprintf('\n=== (%d/%d) Processing CSV: %s ===\n', iFile, numel(csvFiles), csvFiles{iFile});
    close all;  %#ok<CLALL> % keep per-file figures separate
    [res_i, sr_i] = process_one_csv(csvFullPath);
    if ~isempty(res_i)
        result_all = [result_all; res_i]; %#ok<AGROW>
        result_file = [result_file; repmat({csvFiles{iFile}}, size(res_i,1), 1)]; %#ok<AGROW>
        tok = regexp(csvFiles{iFile}, '^[^_]*_([^_]*)_', 'tokens', 'once');
        if isempty(tok)
            subj = '';
        else
            subj = tok{1};
        end
        result_subject = [result_subject; repmat({subj}, size(res_i,1), 1)]; %#ok<AGROW>
    end
end

% (added) final combined summary across all selected CSVs
if ~isempty(result_all)
    off_all = result_all(:,4);
    on_all  = result_all(:,8);
    valid = ~isnan(off_all) & ~isnan(on_all);
    off_all = off_all(valid);
    on_all  = on_all(valid);
    total_trial = numel(off_all);
    total_success = sum(off_all == on_all);
    if total_trial > 0
        fprintf('\n===== FINAL (ALL CSVs) =====\n');
        fprintf('Success / Trial = %d / %d\n', total_success, total_trial);
        fprintf('Overall success rate = %.2f %%\n', 100 * total_success / total_trial);
        % Confusion matrix (rows = offline label, cols = online label)
        classes = unique([off_all; on_all]);
        try
            [C, order] = confusionmat(off_all, on_all, 'Order', classes);
        catch
            [C, order] = confusionmat(off_all, on_all);
        end
        fprintf('Confusion matrix (rows=offline, cols=online), label order: ');
        fprintf('%g ', order); fprintf('\n');
        disp(C);
        % Optional: confusion chart (if available)
        try
            % Combined summary figure: confusion matrix + KNN map
            figSum = figure('Name','All CSVs Summary (Confusion + KNN Map)', 'Color','w');
            tl = tiledlayout(figSum, 1, 2, 'TileSpacing','compact', 'Padding','compact');

            % (1) Confusion matrix
            axC = nexttile(tl, 1);
            confusionchart(axC, C, order);
            title(axC, 'Confusion Matrix (All CSVs)');

            % (2) KNN map scatter (offline/online features overlaid)
            axM = nexttile(tl, 2);
            hold(axM,'on'); grid(axM,'on'); box(axM,'on');


            % (added) KNN map background + boundary (trained from offline labeled points)
            % Background: label=1 (red), label=2 (blue)
            % Boundary: decision boundary of 1-NN classifier (visual aid)
            
            if exist('result_all','var') && ~isempty(result_all)
                offX = result_all(:,1);
                offY = result_all(:,2);
                offL = result_all(:,4);

                onX  = result_all(:,5);
                onY  = result_all(:,6);
                onL  = result_all(:,8);


                % draw background/boundary from offline labeled points
                plot_knn_background(axM, offX, offY, offL);
                % Offline points
                idxOff1 = (offL == 1);
                idxOff2 = (offL == 2);
                if any(idxOff1), plot(axM, offX(idxOff1), offY(idxOff1), 'ro', 'MarkerFaceColor','r', 'DisplayName','Offline label=1'); end
                if any(idxOff2), plot(axM, offX(idxOff2), offY(idxOff2), 'bo', 'MarkerFaceColor','b', 'DisplayName','Offline label=2'); end

                % Online points (different marker)
                idxOn1 = (onL == 1);
                idxOn2 = (onL == 2);
                if any(idxOn1), plot(axM, onX(idxOn1), onY(idxOn1), 'r.', 'DisplayName','Online label=1'); end
                if any(idxOn2), plot(axM, onX(idxOn2), onY(idxOn2), 'b.', 'DisplayName','Online label=2'); end

                legend(axM, 'Location','best');
            end

            xlabel(axM, 'Feature X');
            ylabel(axM, 'Feature Y');
            title(axM, 'KNN Map (Offline/Online Feature Scatter)');
        catch
            % no confusionchart; disp only
            try
                figSum = figure('Name','All CSVs Summary (KNN Map)', 'Color','w');
                axM = axes('Parent', figSum); hold(axM,'on'); grid(axM,'on'); box(axM,'on');
                if exist('result_all','var') && ~isempty(result_all)
                    offX = result_all(:,1);
                    offY = result_all(:,2);
                    offL = result_all(:,4);
                    % draw background/boundary from offline labeled points
                    plot_knn_background(axM, offX, offY, offL);
                    idxOff1 = (offL == 1);
                    idxOff2 = (offL == 2);
                    if any(idxOff1), plot(axM, offX(idxOff1), offY(idxOff1), 'ro', 'MarkerFaceColor','r', 'DisplayName','Offline label=1'); end
                    if any(idxOff2), plot(axM, offX(idxOff2), offY(idxOff2), 'bo', 'MarkerFaceColor','b', 'DisplayName','Offline label=2'); end
                    legend(axM, 'Location','best');
                end
                xlabel(axM, 'Feature X'); ylabel(axM, 'Feature Y');
                title(axM, 'KNN Map (Offline Feature Scatter)');
            catch
            end
        end
    end
end



% (added) Figure 4: subject-wise KNN map (subplots per subject)
% - Each subject combines all trials (CSVs) for that subject
% - Subject name is extracted from filename: between 1st '_' and 2nd '_'
if ~isempty(result_all) && ~isempty(result_subject)
    subj_list = unique(result_subject);
    % keep empty subject (if any) at the end
    subj_list = [subj_list(~strcmp(subj_list,'')); subj_list(strcmp(subj_list,''))];
    nSubj = numel(subj_list);

    figure(4); clf;
    set(gcf, 'Name', 'Figure 4: KNN Map per Subject', 'Color', 'w');

    % (added) make Figure 4 window square-ish
    try
        set(gcf,'Units','pixels');
        pos = get(gcf,'Position');
        s = min(pos(3), pos(4));
        set(gcf,'Position',[pos(1) pos(2) s s]);
    catch
    end

    nCol = ceil(sqrt(nSubj));
    nRow = ceil(nSubj / nCol);

    % Use a single global offline dataset for the KNN background so that every
    % subplot shows the background/boundary even if a subject has only one class.
    offX_all = result_all(:,1);
    offY_all = result_all(:,2);
    offL_all = result_all(:,4);

    for iS = 1:nSubj
        ax = subplot(nRow, nCol, iS);
        cla(ax); hold(ax, 'off');
        set(ax, 'YDir', 'normal');
        hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');

        subj = subj_list{iS};
        idxS = strcmp(result_subject, subj);

        offX = result_all(idxS,1);
        offY = result_all(idxS,2);
        offL = result_all(idxS,4);

        onX  = result_all(idxS,5);
        onY  = result_all(idxS,6);
        onL  = result_all(idxS,8);

        % background + boundary from offline labeled points (same drawing style as KNN_Tswing_effect.m Figure 4)
        % Use global background so it appears for every subject subplot.
        % (changed) xmax: 1.2 -> 1.0
        plot_knn_background(ax, offX_all, offY_all, offL_all, 1.0);

        % Online feature points colored by OFFLINE label (STS=red, SOS=blue)
        offL_sub = result_all(idxS,4);   % offline label for matched rows (same rows as onX/onY)
        idxSTS = (offL_sub == 1);
        idxSOS = (offL_sub == 2);
        if any(idxSTS), plot(ax, onX(idxSTS), onY(idxSTS), 'ro', 'DisplayName','STS (offline label)'); end
        if any(idxSOS), plot(ax, onX(idxSOS), onY(idxSOS), 'bo', 'DisplayName','SOS (offline label)'); end

        % (changed) axis range: 0~1.2 -> 0~1.0
        xlim(ax, [0 1.0]); ylim(ax, [0 1.0]);

        % (added) square aspect for each subplot
        axis(ax, 'square');

        % (added) ensure grid on (already on above, but keep explicit)
        grid(ax, 'on');

        xlabel(ax, 'Feature X');
        ylabel(ax, 'Feature Y');
        if isempty(subj)
            title(ax, '(no subject name)');
        else
            title(ax, subj);
        end
        legend(ax, 'Location','best');
    end
end

%%
% (added) Figure 5: all-subject combined KNN map (single plot)
if ~isempty(result_all) && ~isempty(result_subject)

    figure(5); clf;
    set(gcf, 'Name', 'Figure 5: KNN Map (All Subjects Combined)', 'Color', 'w');

    % square-ish window
    try
        set(gcf,'Units','pixels');
        pos = get(gcf,'Position');
        s = min(pos(3), pos(4));
        set(gcf,'Position',[pos(1) pos(2) s s]);
    catch
    end

    ax = gca;
    cla(ax); hold(ax, 'off');
    set(ax, 'YDir', 'normal');
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');

    % Global offline dataset (background + boundary)
    offX_all = result_all(:,1);
    offY_all = result_all(:,2);
    offL_all = result_all(:,4);

    % Global online points
    onX_all = result_all(:,5);
    onY_all = result_all(:,6);

    plot_knn_background(ax, offX_all, offY_all, offL_all, 1.0);

    grid(ax,'on');
    ax.Layer = 'top';

    ax.GridAlpha = 0.2;
    ax.MinorGridAlpha = 0.2;

    ax.GridColor = [0.5 0.5 0.5];   % 연한 회색
    ax.LineWidth = 1;             % grid 선 두께 줄이기
    ax.MinorGridLineStyle = 'none'; % minor grid 끄기

    % Online feature points: subject-wise marker, color by OFFLINE label
    subj_list = unique(result_subject);
    subj_list = [subj_list(~strcmp(subj_list,'')); subj_list(strcmp(subj_list,''))];

    marker_list = {'o','s','^','d','v','>','<','p','h','x','+'};  % 필요시 추가
    nMarker = numel(marker_list);

    for iS = 1:numel(subj_list)
        subj = subj_list{iS};
        idxS = strcmp(result_subject, subj);

        if ~any(idxS)
            continue;
        end

        mk = marker_list{mod(iS-1, nMarker) + 1};

        % offline label 기준
        offL_sub = offL_all(idxS);
        onX_sub  = onX_all(idxS);
        onY_sub  = onY_all(idxS);

        idxSTS = (offL_sub == 1);
        idxSOS = (offL_sub == 2);

        if any(idxSTS)
            plot(ax, onX_sub(idxSTS), onY_sub(idxSTS), ...
                ['r' mk], 'MarkerSize', 14, 'DisplayName', ['STS - ' subj]);
        end
        if any(idxSOS)
            plot(ax, onX_sub(idxSOS), onY_sub(idxSOS), ...
                ['b' mk], 'MarkerSize', 14, 'DisplayName', ['SOS - ' subj]);
        end
    end


    xlim(ax, [0 1.0]); ylim(ax, [0 1.0]);
    axis(ax, 'square');

%     xlabel(ax, 'Feature X');
%     ylabel(ax, 'Feature Y');
%     title(ax, 'All subjects combined');
    legend(ax, 'Location','best');
end

%%

% (added) save accumulated results once after all CSVs processed
if ~isempty(result_all)
    ts = datestr(now,'yyyymmdd_HHMMSS');
    defaultMat = fullfile(csvPath, ['KNN_perf_result_' ts '.mat']);
% Old save removed
    % Also write CSV (same folder)
    defaultCsv = fullfile(csvPath, ['KNN_perf_result_' ts '.csv']);
    % Write numeric matrix with file names using table (more robust)
    try
        Tout = array2table(result_all, 'VariableNames', result_header);
        Tout = addvars(Tout, result_subject, 'Before', 1, 'NewVariableNames', 'subject_name');
        Tout = addvars(Tout, result_file, 'Before', 1, 'NewVariableNames', 'csv_file');
% CSV saving removed
    catch
        % if writetable fails, keep MAT only
    end
    fprintf('Saved results: %s\n', defaultMat);
    fprintf('Saved results: %s\n', defaultCsv);
end

return; % prevent falling through into local functions when run as a script

% ===============================
% Save results (by subject, MAT only)  [moved out of process_one_csv]
% ===============================
if ~isempty(result_all) && ~isempty(result_subject)
    subjects = unique(result_subject);
    perf_by_subject = struct();

    for iS = 1:numel(subjects)
        subj = subjects{iS};
        idx  = strcmp(result_subject, subj);

        fld = matlab.lang.makeValidName(subj); % fieldname 안전화
        perf_by_subject.(fld).result_all    = result_all(idx, :);
        perf_by_subject.(fld).result_header = result_header;
        perf_by_subject.(fld).result_file   = result_file(idx);
    end

    save(fullfile(csvPath, 'Performancewithassist.mat'), 'perf_by_subject');
end



function [res_mat, success_rate] = process_one_csv(csvFullPath)


    [~, csvFile, csvExt] = fileparts(csvFullPath);
    csvFile = [csvFile csvExt];

opts = detectImportOptions(csvFullPath, 'Encoding','UTF-8');
opts.DataLines = [2 Inf];
opts.VariableNamingRule = 'preserve';
T = readtable(csvFullPath, opts);

% realtime   
reqCols = {'hc_count','s_norm_vel_HC','s_norm_T_HC','s_gait_mode'};
for k = 1:numel(reqCols)
    if ~ismember(reqCols{k}, T.Properties.VariableNames)
        error('CSV   "%s" () .', reqCols{k});
    end
end

% offline   
needOfflineCols = {'RightThighAngle','LeftThighAngle','is_moving'};
for k = 1:numel(needOfflineCols)
    if ~ismember(needOfflineCols{k}, T.Properties.VariableNames)
        error('offline    "%s" () .', needOfflineCols{k});
    end
end

hc_count     = T.hc_count;
rt_feat_vel  = T.s_norm_vel_HC;
rt_feat_T    = T.s_norm_T_HC;
rt_gait_mode = T.s_gait_mode;

% () CSV  s_gait_mode  
% 1  3 -> 2
% 2  4 -> 1
rt_gait_mode = double(rt_gait_mode); %   / 
orig_gm = rt_gait_mode;
mask13 = (orig_gm == 1) | (orig_gm == 3);
mask24 = (orig_gm == 2) | (orig_gm == 4);

rt_gait_mode(mask13) = 2;
rt_gait_mode(mask24) = 1;

angle_R_raw  = double(T.RightThighAngle);
angle_L_raw  = double(T.LeftThighAngle);
is_moving    = double(T.is_moving);

if ismember('s_hc_deg_thresh', T.Properties.VariableNames)
    s_hc_deg_thresh = double(T.s_hc_deg_thresh);
else
    s_hc_deg_thresh = [];
end

%% 2) Realtime HC   (hc_count  )
[rtFeat, rtLab, rtIdx, rtHcVal] = extract_realtime_hc_events(hc_count, rt_feat_vel, rt_feat_T, rt_gait_mode);
if isempty(rtFeat), error('hc_count    .'); end
fprintf('Realtime HC events extracted: %d points\n', size(rtFeat,1));

%% 3) Offline feature  (Refdata_process_many.m : LPF + compute_features_offline)
fs = 500;
N = numel(angle_R_raw);
t = (0:N-1)'/fs;

fc = 20;
angle_R = LPF(angle_R_raw, fs, fc);
angle_L = LPF(angle_L_raw, fs, fc);

%  (Refdata_process_many )
t_gap_default      = 400;
thres_up_default   = 10;
thres_down_default = 10;
var_crit_default   = 0;

[offFeat, off_feat_y_raw, off_idx_list, hc_mask, peaks, usedParams] = compute_offline_features_with_fallback( ...
    angle_R, angle_L, is_moving, fs, ...
    t_gap_default, thres_up_default, thres_down_default, var_crit_default, T);

fprintf('Offline computed feature points: %d points\n', size(offFeat,1));
fprintf('Offline feature params used: t_gap=%.1f, th_up=%.1f, th_dn=%.1f\n', usedParams(1), usedParams(2), usedParams(3));

if isempty(offFeat)
    error('offline feature 0. (fallback ) CSV is_moving/angle/threshold  .');
end

%% 4) Offline review figure (HC/upeak/dpeak plot + scatter) + outlier delete + manual label input
%  figure:
% - : time-series with HC/upeak/dpeak
% - : scatter of offline features (index )
% -  : (1) outlier   -> (2) label  
hFig = figure('Name','Offline Review: delete outliers then input labels', ...
    'NumberTitle','off', 'Color','w', 'Position',[80 60 1500 920]);
hTxtCount = uicontrol('Parent', hFig, 'Style','text', 'Units','pixels', 'Position',[70 895 500 20], ...
    'HorizontalAlignment','left', 'BackgroundColor','w', 'Tag','txt_offline_count', ...
    'String', sprintf('Offline detected HC: %d', size(offFeat,1)));

% ----  plot ----
ax1 = axes('Parent', hFig, 'Units','pixels', 'Position',[70 520 1360 370]);
hold(ax1,'on'); grid(ax1,'on');

plot(ax1, t, angle_R, 'r-');
plot(ax1, t, angle_L, 'b-');

if ~isempty(s_hc_deg_thresh) && numel(s_hc_deg_thresh)==numel(t)
    plot(ax1, t, s_hc_deg_thresh, 'k-');
end
if ~isempty(hc_mask) && numel(hc_mask)==numel(t)
    plot(ax1, t, hc_mask*50, 'k', 'LineWidth', 2);
end

if ~isempty(peaks.upeak_R), plot(ax1, t(peaks.upeak_R), angle_R(peaks.upeak_R), 'mo', 'MarkerSize',6, 'LineWidth',1.2); end
if ~isempty(peaks.upeak_L), plot(ax1, t(peaks.upeak_L), angle_L(peaks.upeak_L), 'go', 'MarkerSize',6, 'LineWidth',1.2); end
if ~isempty(peaks.dpeak_R), plot(ax1, t(peaks.dpeak_R), angle_R(peaks.dpeak_R), 'mx', 'MarkerSize',7, 'LineWidth',1.4); end
if ~isempty(peaks.dpeak_L), plot(ax1, t(peaks.dpeak_L), angle_L(peaks.dpeak_L), 'gx', 'MarkerSize',7, 'LineWidth',1.4); end
if ~isempty(peaks.HC_R),    plot(ax1, t(peaks.HC_R), angle_R(peaks.HC_R), 'ks', 'MarkerSize',6, 'LineWidth',1.2); end
if ~isempty(peaks.HC_L),    plot(ax1, t(peaks.HC_L), angle_L(peaks.HC_L), 'ks', 'MarkerSize',6, 'LineWidth',1.2); end

xlabel(ax1,'Time (s)');
ylabel(ax1,'Angle (deg)');
title(ax1, ['File: ' csvFile], 'Interpreter','none');

% ----  scatter ----
ax2 = axes('Parent', hFig, 'Units','pixels', 'Position',[70 245 1360 240]);
hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');

scaling_x = 86.17;
scaling_y = 0.91;
% () offline feature  scaling 
fx = offFeat(:,1) ./ scaling_x;
fy = offFeat(:,2) ./ scaling_y;
fyr = off_feat_y_raw(:); % raw swing period(sec) or similar ()
lab_tmp = nan(size(fx)); %    signature 

plot(ax2, fx, fy, 'ko', 'MarkerFaceColor',[0.85 0.85 0.85], 'MarkerSize',6);
text(ax2, fx, fy, string(1:numel(fx)), 'FontSize',9, 'Color','k', ...
    'VerticalAlignment','bottom', 'HorizontalAlignment','left');
xlabel(ax2,'offline s\_norm\_vel\_HC');
ylabel(ax2,'offline s\_norm\_T\_HC');
title(ax2,'Offline feature scatter (numbered). Delete outliers first.');

% ----  ----
uicontrol('Parent', hFig, 'Style','text', 'Units','pixels', 'Position',[70 205 1360 28], ...
    'HorizontalAlignment','left', 'BackgroundColor','w', ...
    'String',': (1)  Delete  outlier  -> (2) Label     / -> OK. Skip   .');

% ---- 4-1) outlier  +   (embedded,  ) ----
% saveFlag: 0=Cancel, 1=Save/OK, 2=Skip
[fx2, fy2, fyr2, lab2, saveFlag] = manual_deletion_tool_plot2_embedded(hFig, ax2, fx, fy, fyr, lab_tmp, ax1, t, angle_R, off_idx_list); %#ok<ASGLU>
if saveFlag == 0
    close(hFig);
    error(' outlier/label  .');
elseif saveFlag == 2
    close(hFig);
    error(' Skip .');
end

%   +  
valid = isfinite(fx2) & isfinite(fy2);
fx2 = fx2(valid);
fy2 = fy2(valid);
lab2 = lab2(valid);

off_idx_list2 = off_idx_list(valid);     %    

M2 = numel(fx2);
if M2 == 0
    close(hFig);
    error('   offline feature 0.');
end

% scatter (  )
cla(ax2); hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');

idx1 = (lab2==1);
idx2 = (lab2==2);

if any(idx1), plot(ax2, fx2(idx1), fy2(idx1), 'ro', 'MarkerFaceColor','r', 'MarkerSize',6); end
if any(idx2), plot(ax2, fx2(idx2), fy2(idx2), 'bo', 'MarkerFaceColor','b', 'MarkerSize',6); end

text(ax2, fx2, fy2, string(1:M2), 'FontSize',9, 'Color','k', ...
    'VerticalAlignment','bottom', 'HorizontalAlignment','left');
xlabel(ax2,'offline s\_norm\_vel\_HC');
ylabel(ax2,'offline s\_norm\_T\_HC');
title(ax2,'After deletion + manual labels (set in bottom panel).');

label_manual = double(lab2(:));
if numel(label_manual) ~= M2
    close(hFig);
    error('   . (%d vs %d)', numel(label_manual), M2);
end
if any(~ismember(label_manual, [1 2]))
    close(hFig);
    error(' 1  2 .');
end


close(hFig);

offFeat_final = [fx2(:), fy2(:)];
offLab_final  = label_manual(:);
offMeta_final = struct();
offMeta_final.sampleIdx = double(off_idx_list2(:));

%% 5) Realtime vs Offline (  : offMeta.sampleIdx vs rtIdx)
tol_samples = 5;
[matchRtEventIdx, dt_samples, isMatched] = match_by_time_index(offMeta_final.sampleIdx, rtIdx, tol_samples);

validM = isMatched;
offFeat_m = offFeat_final(validM,:);
offLab_m  = offLab_final(validM);
offMeta_m = struct(); offMeta_m.sampleIdx = offMeta_final.sampleIdx(validM);

rtFeat_m = rtFeat(matchRtEventIdx(validM),:);
rtLab_m  = rtLab(matchRtEventIdx(validM));
rtMeta_m = struct();
rtMeta_m.sampleIdx = rtIdx(matchRtEventIdx(validM));
rtMeta_m.hcVal     = rtHcVal(matchRtEventIdx(validM));

%% 6)  


%% (added) build per-event result rows for saving
% Compute signed distance to class boundary in the (offline) feature space
% using a simple nearest-same vs nearest-other margin proxy.
if ~isempty(offFeat_m)
    off_bd = boundary_margin_proxy(offFeat_m, offLab_m, offFeat_final, offLab_final);
    on_bd  = boundary_margin_proxy(rtFeat_m,  rtLab_m,  offFeat_final, offLab_final);
    success_rate = mean(offLab_m(:) == rtLab_m(:));
    res_mat = [ ...
        offFeat_m(:,1), offFeat_m(:,2), off_bd(:), offLab_m(:), ...
        rtFeat_m(:,1),  rtFeat_m(:,2),  on_bd(:),  rtLab_m(:), ...
        repmat(success_rate, size(offFeat_m,1), 1)];
else
    res_mat = [];
    success_rate = NaN;
end
assignin('base','result_last_csv',res_mat);
assignin('base','success_rate_last_csv',success_rate);

print_compare_summary(offFeat_m, offLab_m, rtFeat_m, rtLab_m, dt_samples(validM), offMeta_m, rtMeta_m);

% ===============================
% Final success / trial summary
% ===============================
trial = numel(offLab_m);
success = sum(offLab_m == rtLab_m);

fprintf('\n===== Final Result =====\n');
fprintf('Success / Trial = %d / %d\n', success, trial);
if trial > 0
    fprintf('Success rate = %.2f %%\n', 100 * success / trial);
else
    fprintf('Success rate = NaN %%\n');
end

%% 7)  
plot_compare_results(rtFeat, rtLab, offFeat_final, offLab_final, matchRtEventIdx, isMatched, rtIdx, offMeta_final);

end



%% ========================= Local functions =========================

function [rtFeat, rtLab, rtIdx, rtHcVal] = extract_realtime_hc_events(hc_count, norm_vel, norm_T, gait_mode)
    hc = double(hc_count);
    hc(isnan(hc)) = 0;

    d = diff(hc);
    incIdx = find(d > 0) + 1;

    incHcVal = hc(incIdx);
    [~, ia] = unique(incHcVal, 'stable');
    incIdx = incIdx(ia);
    incHcVal = incHcVal(ia);

    v = double(norm_vel(incIdx));
    t = double(norm_T(incIdx));
    y = double(gait_mode(incIdx));

    ok = ~(isnan(v) | isnan(t) | isnan(y));
    incIdx = incIdx(ok);
    incHcVal = incHcVal(ok);
    v = v(ok); t = t(ok); y = y(ok);

    rtFeat  = [v(:), t(:)];
    rtLab   = y(:);
    rtIdx   = incIdx(:);
    rtHcVal = incHcVal(:);
end

function [offFeat, feat_y_raw, idx_list, hc_mask, peaks, usedParams] = compute_offline_features_with_fallback( ...
    angle_R, angle_L, is_moving, fs, t_gap0, th_up0, th_dn0, var_crit0, T)

    usedParams = [t_gap0, th_up0, th_dn0];

    % 1)  1
    [~, hc_mask, fx, fy, fyr, idx_list, ~, HC_R, HC_L, uR, uL, dR, dL] = ...
        compute_features_offline(angle_R, angle_L, is_moving, fs, t_gap0, th_up0, th_dn0, var_crit0);

    offFeat = [fx(:), fy(:)];
    feat_y_raw = fyr(:);
    peaks = struct('HC_R',HC_R,'HC_L',HC_L,'upeak_R',uR,'upeak_L',uL,'dpeak_R',dR,'dpeak_L',dL);

    if ~isempty(offFeat)
        return;
    end

    % 2) CSV   
    t_gap_try = t_gap0; th_up_try = th_up0; th_dn_try = th_dn0;

    if ismember('s_t_gap_R_ms', T.Properties.VariableNames)
        v = double(T.s_t_gap_R_ms); v = v(isfinite(v));
        if ~isempty(v), t_gap_try = median(v); end
    end
    if ismember('s_thres_up', T.Properties.VariableNames)
        v = double(T.s_thres_up); v = v(isfinite(v));
        if ~isempty(v), th_up_try = median(v); end
    end
    if ismember('s_thres_down', T.Properties.VariableNames)
        v = double(T.s_thres_down); v = v(isfinite(v));
        if ~isempty(v), th_dn_try = median(v); end
    end

    [~, hc_mask, fx, fy, fyr, idx_list, ~, HC_R, HC_L, uR, uL, dR, dL] = ...
        compute_features_offline(angle_R, angle_L, is_moving, fs, t_gap_try, th_up_try, th_dn_try, var_crit0);

    offFeat = [fx(:), fy(:)];
    feat_y_raw = fyr(:);
    peaks = struct('HC_R',HC_R,'HC_L',HC_L,'upeak_R',uR,'upeak_L',uL,'dpeak_R',dR,'dpeak_L',dL);
    usedParams = [t_gap_try, th_up_try, th_dn_try];

    if ~isempty(offFeat)
        fprintf('Offline fallback used CSV params\n');
        return;
    end

    % 3) sweep
    t_gap_list = unique([t_gap0 200 300 400 500 600]);
    th_up_list = unique([th_up0 5 10 15 20]);
    th_dn_list = unique([th_dn0 5 10 15 20]);

    bestN = 0;
    bestParams = usedParams;

    best_fx=[]; best_fy=[]; best_fyr=[]; best_idx=[]; best_hm=[]; best_peaks=peaks;

    for tg = t_gap_list
        for tu = th_up_list
            for td = th_dn_list
                [~, hm, fx, fy, fyr, il, ~, HC_R, HC_L, uR, uL, dR, dL] = ...
                    compute_features_offline(angle_R, angle_L, is_moving, fs, tg, tu, td, var_crit0);
                n = numel(fx);
                if n > bestN
                    bestN = n;
                    bestParams = [tg, tu, td];
                    best_fx = fx; best_fy = fy; best_fyr = fyr; best_idx = il; best_hm = hm;
                    best_peaks = struct('HC_R',HC_R,'HC_L',HC_L,'upeak_R',uR,'upeak_L',uL,'dpeak_R',dR,'dpeak_L',dL);
                end
            end
        end
    end

    if bestN > 0
        offFeat = [best_fx(:), best_fy(:)];
        feat_y_raw = best_fyr(:);
        idx_list = best_idx(:);
        hc_mask = best_hm;
        peaks = best_peaks;
        usedParams = bestParams;
        fprintf('Offline fallback sweep succeeded: bestN=%d\n', bestN);
    else
        offFeat = [];
        feat_y_raw = [];
        idx_list = [];
        hc_mask = zeros(size(angle_R));
    end
end

function [matchIdx, dt_samples, isMatched] = match_by_time_index(off_sampleIdx, rtIdx, tol_samples)
    off_sampleIdx = double(off_sampleIdx(:));
    rtIdx = double(rtIdx(:));

    matchIdx = nan(size(off_sampleIdx));
    dt_samples = nan(size(off_sampleIdx));
    isMatched = false(size(off_sampleIdx));

    for i = 1:numel(off_sampleIdx)
        [dmin, j] = min(abs(rtIdx - off_sampleIdx(i)));
        matchIdx(i) = j;
        dt_samples(i) = rtIdx(j) - off_sampleIdx(i);
        isMatched(i) = (dmin <= tol_samples);
    end
end

function print_compare_summary(offFeat, offLab, rtFeat, rtLab, dt_samples, offMeta, rtMeta)
    fprintf('\n===== Compare Summary (offline manual labels vs realtime) =====\n');
    fprintf('Matched points: %d\n', size(offFeat,1));

    if isempty(offFeat)
        fprintf('  0. tol_samples    .\n');
        return;
    end

    df = offFeat - rtFeat;
    rmse_v = sqrt(mean(df(:,1).^2));
    rmse_T = sqrt(mean(df(:,2).^2));
    mae_v  = mean(abs(df(:,1)));
    mae_T  = mean(abs(df(:,2)));

    fprintf('Feature RMSE: vel=%.6f, T=%.6f\n', rmse_v, rmse_T);
    fprintf('Feature MAE : vel=%.6f, T=%.6f\n', mae_v, mae_T);

    label_pair = [offLab(:), rtLab(:)];
    label_diff = label_pair(:,1) - label_pair(:,2);
    acc = mean(label_diff == 0) * 100;
    fprintf('Label accuracy: %.2f %%\\n', acc);
    assignin('base','label_pair',label_pair);
    assignin('base','label_diff',label_diff);

    fprintf('Time alignment: mean(dt)=%.2f samples, median=%.2f, maxAbs=%.2f\n', ...
        mean(dt_samples), median(dt_samples), max(abs(dt_samples)));

    cm = zeros(2,2);
    for i = 1:numel(offLab)
        cm(offLab(i), rtLab(i)) = cm(offLab(i), rtLab(i)) + 1;
    end
    fprintf('\nConfusion matrix (rows=offline manual, cols=realtime)\n');
    fprintf('          rt=1     rt=2\n');
    fprintf('off=1   %8d  %8d\n', cm(1,1), cm(1,2));
    fprintf('off=2   %8d  %8d\n', cm(2,1), cm(2,2));

    bad = find(offLab ~= rtLab);
    if ~isempty(bad)
        fprintf('\nTop mismatches (up to 20):\n');
        fprintf('k  off_sampleIdx  rt_sampleIdx  rt_hcVal  off_vel  off_T   off_lab  rt_vel  rt_T   rt_lab  dt\n');
        for ii = 1:min(20, numel(bad))
            i = bad(ii);
            fprintf('%2d %12d %12d %8.0f %8.4f %8.4f %7d %8.4f %8.4f %7d %6.0f\n', ...
                ii, offMeta.sampleIdx(i), rtMeta.sampleIdx(i), rtMeta.hcVal(i), ...
                offFeat(i,1), offFeat(i,2), offLab(i), rtFeat(i,1), rtFeat(i,2), rtLab(i), dt_samples(i));
        end
    end
end

function signed_margin = boundary_margin_proxy(queryFeat, queryLab, refFeat, refLab)
    % signed_margin > 0 means closer to same-label reference than other-label reference
    queryFeat = double(queryFeat);
    queryLab  = double(queryLab(:));
    refFeat   = double(refFeat);
    refLab    = double(refLab(:));

    signed_margin = nan(size(queryLab));
    if isempty(queryFeat) || isempty(refFeat)
        return;
    end

    for i = 1:size(queryFeat,1)
        li = queryLab(i);
        same = (refLab == li);
        other = ~same;
        if ~any(same) || ~any(other)
            signed_margin(i) = NaN;
            continue;
        end
        ds = min(sum((refFeat(same,:)  - queryFeat(i,:)).^2, 2));
        do = min(sum((refFeat(other,:) - queryFeat(i,:)).^2, 2));
        ds = sqrt(ds);
        do = sqrt(do);
        signed_margin(i) = (do - ds) / 2;
    end
end

function plot_compare_results(rtFeatAll, rtLabAll, offFeatAll, offLabAll, matchRtEventIdx, isMatchedTime, rtIdxAll, offMetaAll)
    figure('Name','Realtime vs Offline (manual labels) - Feature space', 'Color','w'); hold on; grid on;

    idx1 = (rtLabAll == 1);
    idx2 = (rtLabAll == 2);

    if any(idx1), plot(rtFeatAll(idx1,1), rtFeatAll(idx1,2), 'r.', 'DisplayName','Realtime label=1'); end
    if any(idx2), plot(rtFeatAll(idx2,1), rtFeatAll(idx2,2), 'b.', 'DisplayName','Realtime label=2'); end

    for i = 1:size(offFeatAll,1)
        if ~isMatchedTime(i)
            plot(offFeatAll(i,1), offFeatAll(i,2), 'ks', 'MarkerSize',7, 'LineWidth',1.2, 'DisplayName','Offline unmatched');
            continue;
        end
        j = matchRtEventIdx(i);

        if offLabAll(i) == rtLabAll(j)
            plot(offFeatAll(i,1), offFeatAll(i,2), 'go', 'MarkerSize',7, 'LineWidth',1.2, 'DisplayName','Offline label match');
        else
            plot(offFeatAll(i,1), offFeatAll(i,2), 'mx', 'MarkerSize',8, 'LineWidth',1.6, 'DisplayName','Offline label mismatch');
        end

        plot([offFeatAll(i,1), rtFeatAll(j,1)], [offFeatAll(i,2), rtFeatAll(j,2)], 'k-', 'HandleVisibility','off');
    end

    xlabel('norm\_vel');
    ylabel('norm\_T');
    title('Realtime vs Offline (time-index matched)');

    ax = gca;
    ch = ax.Children;
    if ~isempty(ch)
        names = get(ch, 'DisplayName');
    if ischar(names), names = {names}; end
    keep = true(size(names));
    seen = containers.Map();
    for k = 1:numel(names)
        nm = names{k};
        if isempty(nm)
            keep(k)=false;
            continue;
        end
        if isKey(seen, nm)
            keep(k)=false;
        else
            seen(nm)=true;
        end
    end
    if any(keep)
        legend(ch(keep), names(keep), 'Location','best');
    end
    end
    figure('Name','Time Index Alignment', 'Color','w'); grid on; hold on;
    plot(offMetaAll.sampleIdx, 'k.-', 'DisplayName','offline sampleIdx (per feature)');
    plot(rtIdxAll, 'c.-', 'DisplayName','realtime hc_count increment sampleIdx');
    xlabel('event order');
    ylabel('sample index');
    title('Offline event sampleIdx vs Realtime hc sampleIdx');
    legend('Location','best');
end

% =========================================================================
% Refdata_process_many.m  embedded label UI
% =========================================================================
function label = embed_labeling_ui(hFig, M, fname)
    hPanel = uipanel('Parent', hFig, 'Position', [0 0 1 0.22], ...
        'BackgroundColor', [0.94 0.94 0.94], 'Title', 'Labeling Control Panel');

    uistack(hPanel, 'top');   % :   
    drawnow;                 % :  

    uicontrol(hPanel, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.05 0.70 0.9 0.20], 'FontSize', 11, 'FontWeight', 'bold', ...
        'String', sprintf('[%s]  HC : %d ->   %d .', fname, M, M), ...
        'BackgroundColor', [0.94 0.94 0.94]);

    uicontrol(hPanel, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.05 0.45 0.15 0.20], 'String', ' :', ...
        'HorizontalAlignment', 'right', 'FontSize', 10, 'BackgroundColor', [0.94 0.94 0.94]);

    hEdit = uicontrol(hPanel, 'Style', 'edit', 'Units', 'normalized', ...
        'Position', [0.22 0.47 0.60 0.18], 'FontSize', 10, ...
        'String', mat2str(ones(1,M)*2));

    % Preset buttons
    uicontrol(hPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.05 0.15 0.16 0.20], 'String', 'All 1', ...
        'Callback', @(~,~) setPreset(1));

    uicontrol(hPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.23 0.15 0.16 0.20], 'String', 'All 2', ...
        'Callback', @(~,~) setPreset(2));

    uicontrol(hPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.41 0.15 0.24 0.20], 'String', 'All 2 last 1', ...
        'Callback', @(~,~) setPreset(3));

    % OK / Skip
    uicontrol(hPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.68 0.15 0.14 0.20], 'String', 'OK', ...
        'Callback', @confirmLabel);

    uicontrol(hPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.84 0.15 0.12 0.20], 'String', 'Skip', ...
        'Callback', @skipFile);

    final_label = [];
    uiwait(hFig);
    label = final_label;

    function setPreset(mode)
        switch mode
            case 1
                L = ones(1,M);
            case 2
                L = ones(1,M)*2;
            case 3
                if M >= 1
                    L = ones(1,M)*2;
                    L(end) = 1;
                else
                    L = [];
                end
        end
        set(hEdit, 'String', mat2str(L));
    end

    function confirmLabel(~,~)
        txt = get(hEdit, 'String');
        L = str2num(txt); %#ok<ST2NM>
        if isempty(L) || numel(L) ~= M
            errordlg(sprintf('  %d .', M));
            return;
        end
        if any(~ismember(L, [1 2]))
            errordlg(' 1  2 .');
            return;
        end
        final_label = L(:);
        uiresume(hFig);
    end

    function skipFile(~,~)
        final_label = [];
        uiresume(hFig);
    end
end

% =========================================================================
% Refdata_process_many.m embedded deletion tool ( )
% =========================================================================
function [fx, fy, fyr, lab, saveFlag] = manual_deletion_tool_plot2_embedded(hFig, targetAx, fx, fy, fyr, lab, timeAx, t, angle_R, off_sampleIdx)
    %   :
    % - Delete: scatter   (Enter )
    % - Label :   + preset (all1 / all2 / all2 last1)
    % - Save: + 
    % - Skip:  
    % - Cancel: 
    %
    % saveFlag: 0=Cancel, 1=Save, 2=Skip

    hAx = targetAx;
    hAxTime = timeAx;
    axes(hAx);

    %   (uipanel)   ( )
    oldPanels = findall(hFig, 'Type','uipanel');
    if ~isempty(oldPanels)
        delete(oldPanels);
        drawnow;
    end

    hPanel = uipanel('Parent', hFig, 'Position', [0 0 1 0.22], 'Title', 'Outlier Deletion + Label Input');

    uicontrol(hPanel, 'Style','text', 'Units','normalized', ...
        'Position',[0.02 0.78 0.96 0.18], 'BackgroundColor',[0.94 0.94 0.94], ...
        'HorizontalAlignment','left', 'FontSize', 10, ...
        'String','Delete:   ( Enter). Label:    [2 2 1 ...] . Save .');

    %  
    uicontrol(hPanel, 'Style','text', 'Units','normalized', ...
        'Position',[0.02 0.58 0.10 0.14], 'BackgroundColor',[0.94 0.94 0.94], ...
        'HorizontalAlignment','left', 'FontSize', 10, 'String','Labels:');

    hEdit = uicontrol(hPanel, 'Style','edit', 'Units','normalized', ...
        'Position',[0.12 0.585 0.58 0.14], 'FontSize', 10, ...
        'HorizontalAlignment','left', 'String', mat2str(ones(1,sum(isfinite(fx)&isfinite(fy)))*2));

    % preset buttons
    btnAll1 = uicontrol(hPanel, 'Style','pushbutton', 'Units','normalized', ...
        'Position',[0.72 0.585 0.08 0.14], 'String','All 1', ...
        'Callback', @(~,~) setPreset(1));

    btnAll2 = uicontrol(hPanel, 'Style','pushbutton', 'Units','normalized', ...
        'Position',[0.81 0.585 0.08 0.14], 'String','All 2', ...
        'Callback', @(~,~) setPreset(2));

    btnAll2Last1 = uicontrol(hPanel, 'Style','pushbutton', 'Units','normalized', ...
        'Position',[0.90 0.585 0.08 0.14], 'String','2..2, last1', ...
        'Callback', @(~,~) setPreset(3));

    % action buttons
    btnDelete = uicontrol(hPanel, 'Style','pushbutton', 'Units','normalized', ...
        'Position',[0.05 0.12 0.18 0.32], 'String','Delete', ...
        'FontSize', 11, 'FontWeight','bold', 'BackgroundColor',[1 0.85 0.85], ...
        'Callback', @enterDeleteMode);

    btnSave = uicontrol(hPanel, 'Style','pushbutton', 'Units','normalized', ...
        'Position',[0.28 0.12 0.18 0.32], 'String','Save', ...
        'FontSize', 11, 'FontWeight','bold', 'BackgroundColor',[0.85 1 0.85], ...
        'Callback', @saveAndExit);

    btnSkip = uicontrol(hPanel, 'Style','pushbutton', 'Units','normalized', ...
        'Position',[0.51 0.12 0.18 0.32], 'String','Skip', ...
        'FontSize', 11, 'FontWeight','bold', 'BackgroundColor',[0.9 0.9 0.9], ...
        'Callback', @skipAndExit);

    btnCancel = uicontrol(hPanel, 'Style','pushbutton', 'Units','normalized', ...
        'Position',[0.74 0.12 0.18 0.32], 'String','Cancel', ...
        'FontSize', 11, 'FontWeight','bold', 'BackgroundColor',[0.9 0.9 0.9], ...
        'Callback', @cancelAndExit);

    saveFlag = 0;
    refreshScatter();
    uiwait(hFig);

    function refreshScatter()
        cla(hAx); hold(hAx,'on'); grid(hAx,'on'); box(hAx,'on');
        valid = isfinite(fx) & isfinite(fy);
        plot(hAx, fx(valid), fy(valid), 'ko', 'MarkerFaceColor',[0.85 0.85 0.85], 'MarkerSize',6);
        text(hAx, fx(valid), fy(valid), string(find(valid)), 'FontSize',9, 'Color','k', ...
            'VerticalAlignment','bottom', 'HorizontalAlignment','left');
        xlabel(hAx,'offline s\_norm\_vel\_HC');
        ylabel(hAx,'offline s\_norm\_T\_HC');
        title(hAx,'Offline feature scatter (Delete   Save)');
        refreshTimeMarkers();
        updateEditDefaultIfNeeded();
    end

    function refreshTimeMarkers()
        if isempty(hAxTime) || ~isgraphics(hAxTime)
            return;
        end
        %   : marker 
        % offline feature (off_sampleIdx)  plot ( )
        valid = isfinite(fx) & isfinite(fy);
        samp = double(off_sampleIdx(:));
        if numel(samp) ~= numel(fx)
            %     
            return;
        end
        tt = t(samp(valid));
        yy = angle_R(samp(valid));
        hOld = findobj(hAxTime, 'Tag','offline_hc_click');
        if ~isempty(hOld)
            delete(hOld);
        end
        hold(hAxTime,'on');
        scatter(hAxTime, tt, yy, 36, 'k', 'filled', 'Tag','offline_hc_click', ...
            'HitTest','on', 'PickableParts','all');
        % count  
        hTxt = findobj(hFig, 'Tag','txt_offline_count');
        if ~isempty(hTxt)
            set(hTxt, 'String', sprintf('Offline detected HC: %d', sum(valid)));
        end
    end

    function updateEditDefaultIfNeeded()
        %  valid  edit    all2 
        n = sum(isfinite(fx) & isfinite(fy));
        cur = str2num(get(hEdit,'String')); %#ok<ST2NM>
        if isempty(cur) || numel(cur) ~= n
            set(hEdit,'String', mat2str(ones(1,n)*2));
        end
    end

    function setPreset(mode)
        n = sum(isfinite(fx) & isfinite(fy));
        if n <= 0
            set(hEdit,'String','[ ]');
            return;
        end
        switch mode
            case 1
                L = ones(1,n);
            case 2
                L = ones(1,n)*2;
            case 3
                L = ones(1,n)*2;
                L(end) = 1;
        end
        set(hEdit,'String', mat2str(L));
    end

    function enterDeleteMode(~,~)
        set([btnDelete btnSave btnSkip btnCancel btnAll1 btnAll2 btnAll2Last1], 'Enable','off');
        refreshScatter();

        %  ( time plot  scatter), Enter 
        set(hFig, 'Pointer', 'crosshair');
        setappdata(hFig, 'delete_mode_active', true);

        oldWBD = get(hFig, 'WindowButtonDownFcn');
        oldKPF = get(hFig, 'KeyPressFcn');

        set(hFig, 'WindowButtonDownFcn', @onMouseDownDelete);
        set(hFig, 'KeyPressFcn', @onKeyPressDelete);

        uiwait(hFig);  % Enter  uiresume

        %  
        if isgraphics(hFig)
            set(hFig, 'WindowButtonDownFcn', oldWBD);
            set(hFig, 'KeyPressFcn', oldKPF);
            set(hFig, 'Pointer', 'arrow');
        end
        setappdata(hFig, 'delete_mode_active', false);

        set([btnDelete btnSave btnSkip btnCancel btnAll1 btnAll2 btnAll2Last1], 'Enable','on');
        updateEditDefaultIfNeeded();
    end

    function onKeyPressDelete(~, evt)
        if isempty(evt) || ~isfield(evt,'Key'), return; end
        if any(strcmpi(evt.Key, {'return','enter'}))
            uiresume(hFig);
        end
    end

    function onMouseDownDelete(~,~)
        if ~isgraphics(hFig), return; end
        if ~isappdata(hFig,'delete_mode_active') || ~getappdata(hFig,'delete_mode_active')
            return;
        end

        h = hittest(hFig);
        axClicked = ancestor(h, 'axes');
        if isempty(axClicked) || ~isgraphics(axClicked)
            return;
        end

        valid = isfinite(fx) & isfinite(fy);
        if ~any(valid)
            return;
        end

        cp = get(axClicked, 'CurrentPoint');
        mx = cp(1,1);
        my = cp(1,2);

        if axClicked == hAxTime
            %  time plot:     offline HC 
            samp = double(off_sampleIdx(:));
            if numel(samp) ~= numel(fx)
                return;
            end
            tt = t(samp(valid));
            idxValid = find(valid);
            [~, j] = min(abs(tt - mx));
            ridx = idxValid(j);
        elseif axClicked == hAx
            % scatter: (fx,fy)      
            idxValid = find(valid);
            d = sqrt((fx(idxValid)-mx).^2 + (fy(idxValid)-my).^2);
            [~, j] = min(d);
            ridx = idxValid(j);
        else
            return;
        end

        fx(ridx)  = NaN;
        fy(ridx)  = NaN;
        fyr(ridx) = NaN;
        lab(ridx) = NaN;

        refreshScatter();
    end

    function saveAndExit(~,~)
        valid = isfinite(fx) & isfinite(fy);
        n = sum(valid);
        if n <= 0
            errordlg('  . Delete    Cancel   .');
            return;
        end

        txt = get(hEdit,'String');
        L = str2num(txt); %#ok<ST2NM>
        if isempty(L) || numel(L) ~= n
            errordlg(sprintf('    (%d)  .', n));
            return;
        end
        if any(~ismember(L, [1 2]))
            errordlg(' 1  2 .');
            return;
        end

        % valid   
        lab(valid) = L(:);

        saveFlag = 1;
        delete(hPanel);
        uiresume(hFig);
    end

    function skipAndExit(~,~)
        saveFlag = 2;
        delete(hPanel);
        uiresume(hFig);
    end

    function cancelAndExit(~,~)
        saveFlag = 0;
        delete(hPanel);
        uiresume(hFig);
    end
end
% =========================================================================
% Refdata_process_many.m LPF / compute_features_offline 
% =========================================================================
function y = LPF(x, fs, fc)
    y = zeros(length(x),1);
    T = 1/fs;
    RC = 1/(2*pi*fc);
    a = T / (T + RC);
    for i = 2 : length(x)
        y(i) = (1-a) * y(i-1) + a * x(i);
    end
end

function [arb, hc_mask, feat_x, feat_y, feat_y_raw, idx_list, swing_side, ...
          hc_idx_R, hc_idx_L, up_idx_R, up_idx_L, low_idx_R, low_idx_L] = ...
    compute_features_offline(Rdeg, Ldeg, is_moving, fs, t_gap, thres_up, thres_down, var_crit)

    % Refdata_process_many.m (   )
    N  = length(Rdeg); dt = 1/fs;
    arb = zeros(N,1); hc_mask = zeros(N,1);
    feat_x = []; feat_y = []; feat_y_raw = []; idx_list = []; swing_side = [];
    hc_idx_R = []; hc_idx_L = []; up_idx_R = []; up_idx_L = []; low_idx_R = []; low_idx_L = [];

    thres_up_R = thres_up; thres_up_L = thres_up;
    thres_down_R = thres_down; thres_down_L = thres_down;
    t_gap_R = t_gap; t_gap_L = t_gap;
    R_lowpeak_val = thres_down_R; L_lowpeak_val = thres_down_L;
    Rpeakval = 50; Lpeakval = 50;

    R_time_afterupFlag = 0; L_time_afterupFlag = 0; HC_time_afterFlag = 0;
    R_swing_time = 0; L_swing_time = 0;
    R_time_upcond = false; L_time_upcond = false; HC_time_upcond = false;

    pendR.valid = false; pendR.i=[]; pendR.vel=[]; pendR.T=[]; pendR.swing_start=[];
    pendL.valid = false; pendL.i=[]; pendL.vel=[]; pendL.T=[]; pendL.swing_start=[];

    for i = 3:N
        % : Refdata_process_many.m  
        R_time_afterupFlag = R_time_afterupFlag + 1;
        L_time_afterupFlag = L_time_afterupFlag + 1;
        HC_time_afterFlag  = HC_time_afterFlag  + 1;
        R_swing_time = R_swing_time + 1;
        L_swing_time = L_swing_time + 1;

        R0 = Rdeg(i-2); R1 = Rdeg(i-1); R2 = Rdeg(i);
        L0 = Ldeg(i-2); L1 = Ldeg(i-1); L2 = Ldeg(i);

        if R_time_afterupFlag >= t_gap_R * (fs/1000), R_time_upcond = true; end
        if L_time_afterupFlag >= t_gap_L * (fs/1000), L_time_upcond = true; end
        if HC_time_afterFlag  >= min(t_gap_R, t_gap_L) * (fs/1000), HC_time_upcond  = true; end

        % HC check
        if HC_time_upcond && ((R2 - L2)*(R1 - L1) <= 0) && (R2 > thres_down_R) && (L2 > thres_down_L)
            if (R2 - R1) >= (L2 - L1) && Lpeakval > 40
                HC_time_upcond = false; HC_time_afterFlag = 0; hc_mask(i) = 1;
                pendR.valid = true; pendR.i = i; pendR.vel = (R2 - R1)/dt; pendR.T = R_swing_time;
                if ~isempty(low_idx_R), pendR.swing_start = low_idx_R(end);
                else, pendR.swing_start = max(1, i-round(1/dt)); end
                hc_idx_R(end+1,1) = i;
            elseif (L2 - L1) >= (R2 - R1) && Rpeakval > 40
                HC_time_upcond = false; HC_time_afterFlag = 0; hc_mask(i) = 1;
                pendL.valid = true; pendL.i = i; pendL.vel = (L2 - L1)/dt; pendL.T = L_swing_time;
                if ~isempty(low_idx_L), pendL.swing_start = low_idx_L(end);
                else, pendL.swing_start = max(1, i-round(1/dt)); end
                hc_idx_L(end+1,1) = i;
            end
        end

        % Upper R -> feature commit
        if ((R2 - R1)*(R1 - R0) <= var_crit) && ((R2 - R1) < 0) && (R2 >= thres_up_R) && R_time_upcond && pendR.valid
            Rpeakval = R2; R_time_upcond = false; R_time_afterupFlag = 0; up_idx_R(end+1,1) = i;
            swing_period_R = (i - pendR.swing_start) * dt;

            meas_ms = swing_period_R * 1000;
            meas_clamped = min(max(meas_ms, 150), 1200);
            updated = (1-0.2)*t_gap_R + 0.2*meas_clamped;
            if t_gap_R > 0, updated = min(max(updated, t_gap_R*0.7), t_gap_R*1.3); end
            t_gap_R = min(max(updated, 200), 800);

            if ~isempty(pendR.vel) && swing_period_R > 0 && isfinite(pendR.vel)
                feat_x(end+1,1) = (pendR.vel * swing_period_R);
                feat_y(end+1,1) = (pendR.T * dt) / swing_period_R;
                feat_y_raw(end+1,1) = swing_period_R;
                idx_list(end+1,1) = pendR.i; swing_side(end+1,1)= 1;
            end
            pendR.valid = false; arb(i) = swing_period_R;
        end

        % Upper L -> feature commit
        if ((L2 - L1)*(L1 - L0) <= var_crit) && ((L2 - L1) < 0) && (L2 >= thres_up_L) && L_time_upcond && pendL.valid
            Lpeakval = L2; L_time_upcond = false; L_time_afterupFlag = 0; up_idx_L(end+1,1) = i;
            swing_period_L = (i - pendL.swing_start) * dt;

            meas_ms = swing_period_L * 1000;
            meas_clamped = min(max(meas_ms, 150), 1200);
            updated = (1-0.2)*t_gap_L + 0.2*meas_clamped;
            if t_gap_L > 0, updated = min(max(updated, t_gap_L*0.7), t_gap_L*1.3); end
            t_gap_L = min(max(updated, 200), 800);

            if ~isempty(pendL.vel) && swing_period_L > 0 && isfinite(pendL.vel)
                feat_x(end+1,1) = (pendL.vel * swing_period_L);
                feat_y(end+1,1) = (pendL.T * dt) / swing_period_L;
                feat_y_raw(end+1,1) = swing_period_L;
                idx_list(end+1,1) = pendL.i; swing_side(end+1,1)= 2;
            end
            pendL.valid = false; arb(i) = swing_period_L;
        end

        % Lower peaks
        if (R2 > R_lowpeak_val + 10), R_lowpeak_val = thres_down_R; end
        if ((R2 - R1)*(R1 - R0) <= var_crit) && ((R2 - R1) >= 0) && (R2 <= R_lowpeak_val + 2) && (is_moving(i) == 1)
            R_lowpeak_val = R2; R_swing_time = 0; low_idx_R(end+1,1) = i;
        end
        if (L2 > L_lowpeak_val + 10), L_lowpeak_val = thres_down_L; end
        if ((L2 - L1)*(L1 - L0) <= var_crit) && ((L2 - L1) > 0) && (L2 <= L_lowpeak_val + 2) && (is_moving(i) == 1)
            L_lowpeak_val = L2; L_swing_time = 0; low_idx_L(end+1,1) = i;
        end
    end
end


% =========================================================================
% (added) Draw KNN-map-style background and boundary
% - Background: predicted label regions (1=red, 2=blue)
% - Boundary: decision boundary of 1-NN trained on offline labeled points
% =========================================================================
function plot_knn_background(ax, x, y, lab, varargin)
    % Draw KNN-map-style background using PRETRAINED reference data
    % (Copied behavior from KNN_Tswing_effect.m Figure 4)
    %
    % Note:
    % - Inputs (x,y,lab) are kept for backward compatibility but are NOT used
    %   to train the background. Background is drawn from pretrained ref dataset.
    % - Pretrained dataset (.mat) must contain: ref_final_scaled (Nx3) and best_K.
    %
    % Cache model so user selects ref file only once per MATLAB session.
    persistent knn_model_cached xg_cached yg_cached RGB_cached Z_cached xmax_cached

    xmax = 1.5;
    if ~isempty(varargin)
        xmax = varargin{1};
    end

    % If axis range changed, invalidate cached grids/images (model can stay).
    if isempty(xmax_cached) || (abs(xmax_cached - xmax) > 1e-12)
        xg_cached = []; yg_cached = []; RGB_cached = []; Z_cached = [];
        xmax_cached = xmax;
    end

    % ---- Build/load pretrained KNN model (once) ----
    if isempty(knn_model_cached)
        % Fixed pretrained reference dataset path (no selection UI)
        refDir = 'C:\Users\ChanyoungKo\OneDrive - 한국과학기술원 기계공학과\Kyle\KAIST\ExoLab\성과\Publication\Ongoing\1저자_2025_RAL_StairMode\2nd_Submit\Revision_Data\Ref_dataset';
        refFile = fullfile(refDir, 'refdataset_merged_scaled.mat');

        if ~exist(refFile, 'file')
            error('Pretrained reference dataset not found: %s', refFile);
        end

        S = load(refFile, 'ref_final_scaled', 'best_K');
        if ~isfield(S, 'ref_final_scaled') || ~isfield(S, 'best_K')
            error('Pretrained MAT file must contain ref_final_scaled and best_K: %s', refFile);
        end

        ref_final_scaled = S.ref_final_scaled;
        best_K = S.best_K;

        X_train = ref_final_scaled(:, 1:2);
        Y_train = ref_final_scaled(:, 3);

        % Match KNN_Tswing_effect.m training options
        knn_model_cached = fitcknn(X_train, Y_train, 'NumNeighbors', best_K, ...
            'DistanceWeight', 'inverse', 'Standardize', false);
    end

    % ---- Create grid + predict (cache) ----
    if isempty(xg_cached) || isempty(yg_cached) || isempty(RGB_cached) || isempty(Z_cached)
        ng = 220;
        xg = linspace(0, xmax, ng);
        yg = linspace(0, xmax, ng);
        [Xg, Yg] = meshgrid(xg, yg);

        Z = predict(knn_model_cached, [Xg(:), Yg(:)]);
        Z = reshape(Z, size(Xg));

        % Build RGB background (same style as KNN_Tswing_effect.m)
        baseRed  = [1.00 0.82 0.82];
        baseBlue = [0.82 0.90 1.00];

        RGB = zeros([size(Z), 3]);
        mask1 = (Z == 1);
        mask2 = (Z == 2);
        for c = 1:3
            tmp = zeros(size(Z));
            tmp(mask1) = baseRed(c);
            tmp(mask2) = baseBlue(c);
            RGB(:,:,c) = tmp;
        end

        xg_cached = xg;
        yg_cached = yg;
        RGB_cached = RGB;
        Z_cached = Z;
    end

    % ---- Draw ----
    hold(ax, 'on');
    hImg = imagesc(ax, xg_cached, yg_cached, RGB_cached);
    set(ax, 'YDir', 'normal');

    [Xg2, Yg2] = meshgrid(xg_cached, yg_cached);
    contour(ax, Xg2, Yg2, Z_cached, [1.5 1.5], 'k--', ...
        'LineWidth', 1.5, 'HandleVisibility', 'off');

    xlim(ax, [0 xmax]); ylim(ax, [0 xmax]);

    try
        uistack(hImg, 'bottom');
    catch
    end
end

