clear all;
close all;
clc;

% =========================================================================
% 1. 파일 선택 (Multi-Select 지원)
% =========================================================================
initialPath = 'C:\Users\ChanyoungKo\OneDrive - 한국과학기술원 기계공학과\Kyle\KAIST\ExoLab\성과\Publication\Ongoing\1저자_2025_RAL_StairMode\2nd_Submit\Revision_Data\';

fprintf('처리할 CSV 파일을 선택하세요...\n');
[files, fPath] = uigetfile({'*.csv', 'CSV Files (*.csv)'}, ...
                           'Select CSV Data', ...
                           initialPath, ...
                           'MultiSelect', 'on');

if isequal(files, 0)
    fprintf('파일 선택이 취소되었습니다.\n');
    return;
end

if ischar(files)
    fileList = {files};
else
    fileList = files;
end

fprintf('총 %d개의 파일이 선택되었습니다.\n', length(fileList));

% =========================================================================
% 2. 파일 순차 처리 루프 시작
% =========================================================================
for fIdx = 1:length(fileList)
    
    % 작업 전 초기화
    close all; 
    
    csvFileName = fileList{fIdx};
    fullCsvPath = fullfile(fPath, csvFileName);
    
    fprintf('\n=================================================================\n');
    fprintf('Processing File (%d/%d): %s\n', fIdx, length(fileList), csvFileName);
    fprintf('=================================================================\n');

    %% CSV Header Definition
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

    %% Data Load
    try
        opts = detectImportOptions(fullCsvPath, 'Encoding','UTF-8');
        opts.DataLines = [2 Inf];
        opts.VariableNamesLine = 0;
        opts.VariableNamingRule = 'preserve';

        T_table = readtable(fullCsvPath, opts);
        
        if width(T_table) == numel(CSV_HEADER)
            T_table.Properties.VariableNames = CSV_HEADER;
            RightThighAngle = T_table.RightThighAngle;
            LeftThighAngle  = T_table.LeftThighAngle;
            is_moving       = T_table.is_moving;
            s_hc_deg_thresh = T_table.s_hc_deg_thresh;
            if ismember('TswingRecording_ms', T_table.Properties.VariableNames)
                TswingRecording_ms = T_table.TswingRecording_ms;
            else
                TswingRecording_ms = [];
            end
        else
             warning('헤더 개수가 맞지 않아 인덱스로 강제 로드합니다.');
             RightThighAngle = T_table{:,8};
             LeftThighAngle  = T_table{:,7};
             is_moving       = T_table{:,25};
             s_hc_deg_thresh = T_table{:,48};
             TswingRecording_ms = [];
        end

    catch err
        fprintf('Error reading file %s: %s\n', csvFileName, err.message);
        continue; 
    end

    angle_R = RightThighAngle;
    angle_L = LeftThighAngle;
    N = length(angle_R);

    fs = 500;
    t = (0:N-1).' / fs;

    %% Processing
    t_gap      = 400;   
    thres_up   = 10;
    thres_down = 10;
    var_crit   = 0;

    fc = 20;
    angle_R = LPF(angle_R, fs, fc);
    angle_L = LPF(angle_L, fs, fc);

    % Feature Calculation
    [arb, hc_mask, feat_x, feat_y, feat_y_raw, idx_list, swing_side, ...
     HC_R, HC_L, upeak_R, upeak_L, dpeak_R, dpeak_L] = compute_features_offline( ...
         angle_R, angle_L, is_moving, fs, t_gap, thres_up, thres_down, var_crit);

    %% Figure 1 Generation (Time Series + Labeling UI)
    hFig = figure('Name', ['[File ' num2str(fIdx) '] Check & Label'], ...
        'NumberTitle', 'off', 'Position', [100 100 1000 700]);
    
    % Plot 1 (Left)
    subplot('Position', [0.06 0.35 0.42 0.58]); 
    plot(t, angle_R,'r'); hold on;
    plot(t, angle_L,'b'); hold on;
    plot(t, s_hc_deg_thresh,'k'); hold on;
    plot(t, hc_mask*50, 'k', 'LineWidth', 2); grid on;
    title(['File: ' csvFileName], 'Interpreter', 'none');
    xlabel('Time (s)'); ylabel('Angle');

    % Plot 2 (Right)
    subplot('Position', [0.55 0.35 0.42 0.58]);
    plot(t, angle_R, 'r'); hold on;
    plot(t, angle_L, 'b');
    plot(t, hc_mask*50, 'k', 'LineWidth', 2); grid on;
    xlabel('Time (s)'); ylabel('Angle (deg)');
    legend('angle\_R','angle\_L', 'HC');
    title('확인 후 하단 패널에서 라벨 입력');

    % =====================================================================
    % 1단계: 라벨 입력 UI (여기서 Skip 가능)
    % =====================================================================
    M = length(feat_x);
    fprintf('HC Count: %d\n', M);
    
    label = embed_labeling_ui(hFig, M, csvFileName);
    
    % 라벨이 비어있으면(Skip 버튼 누름 or 취소) -> 저장 단계 건너뛰고 다음 파일
    if isempty(label)
        fprintf('>>> 사용자가 파일을 건너뛰었습니다 (Skipped).\n');
        continue;
    end
    
    %% Figure 2 - Feature Scatter & Deletion (화면 갱신)
    clf(hFig); 
    
    % 1. Time Series (HC Result) - Top Left
    ax1 = subplot('Position', [0.06 0.64 0.40 0.30]);
    plot(t, angle_R,'r'); hold on;
    plot(t, angle_L,'b'); hold on;
    plot(t, hc_mask*50, 'k', 'LineWidth', 2); grid on;
    title(['File: ' csvFileName], 'Interpreter', 'none');
    xlabel('Time (s)');

    % 2. Scatter Plot (Feature Space) - Top Right
    ax2 = subplot('Position', [0.54 0.64 0.40 0.30]);
    % (내용은 manual_deletion_tool에서 그림)
    title('Feature scatter');

    % 3. Check Peaks - Bottom Left
    ax3 = subplot('Position', [0.06 0.26 0.40 0.30]);
    plot(t, angle_R, 'r'); hold on; grid on;
    plot(t, angle_L, 'b'); 
    if ~isempty(HC_R), plot(t(HC_R), angle_R(HC_R), 'ms'); end
    if ~isempty(HC_L), plot(t(HC_L), angle_L(HC_L), 'gs'); end
    title('Check Peaks');
    xlabel('Time (s)');

    % 4. T_swing - Bottom Right
    ax4 = subplot('Position', [0.54 0.26 0.40 0.30]);
    if ~isempty(TswingRecording_ms)
        plot(t, TswingRecording_ms, 'k-', 'DisplayName', 'Online');
    end
    hold on; grid on;
    plot(t, arb * 1000, 'r--', 'DisplayName', 'Offline');
    legend; title('Swing Time');
    xlabel('Time (s)'); ylabel('ms');

    % =========================================================================
    % 2단계: 이상치 제거 및 저장 결정 (여기서도 Skip 가능)
    % =========================================================================
    fprintf('데이터 편집(이상치 제거) 모드를 실행합니다.\n');

    % saveFlag 반환 추가
    [feat_x, feat_y, feat_y_raw, label, saveFlag] = ...
        manual_deletion_tool_plot2_embedded(hFig, ax2, feat_x, feat_y, feat_y_raw, label);

    if saveFlag == false
        fprintf('>>> [저장 안 함] 사용자가 저장을 취소했습니다. 다음 파일로 넘어갑니다.\n');
        continue;
    end

    %% Reference dataset save (Skip되지 않았을 때만 실행)
    nFeat = length(feat_x);
    nLab  = length(label);
    nMin  = min(nFeat, nLab);

    validIdx = isfinite(feat_x(1:nMin)) & isfinite(feat_y(1:nMin)) & isfinite(feat_y_raw(1:nMin));

    ref_raw = [feat_x(validIdx), feat_y(validIdx), label(validIdx)];
    ref_type2 = [feat_x(validIdx), feat_y_raw(validIdx), label(validIdx)];

    saveDir = 'C:\Users\ChanyoungKo\OneDrive - 한국과학기술원 기계공학과\Kyle\KAIST\ExoLab\성과\Publication\Ongoing\1저자_2025_RAL_StairMode\2nd_Submit\Revision_Data\Ref_dataset';
    if ~exist(saveDir, 'dir')
        mkdir(saveDir);
    end

    [~, baseName, ~] = fileparts(convertStringsToChars(csvFileName));
    saveFileName = [baseName '_ref.mat'];
    fullSavePath = fullfile(saveDir, saveFileName);

    save(fullSavePath, 'ref_raw', 'ref_type2');

    fprintf('저장 완료! (남은 파일: %d개)\n', length(fileList) - fIdx);
    fprintf('파일: %s\n', saveFileName);
end

fprintf('\n모든 파일 처리가 완료되었습니다.\n');


%% Function Definitions

% -------------------------------------------------------------------------
% 1. Figure 내장형 라벨 입력 도구
% -------------------------------------------------------------------------
function label = embed_labeling_ui(hFig, M, fname)
    % 하단 패널 생성 (높이 22%)
    hPanel = uipanel('Parent', hFig, 'Position', [0 0 1 0.22], ...
        'BackgroundColor', [0.94 0.94 0.94], 'Title', 'Labeling Control Panel');
    
    uicontrol(hPanel, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.05 0.70 0.9 0.20], 'FontSize', 11, 'FontWeight', 'bold', ...
        'String', sprintf('[%s] 감지된 HC 개수: %d개 -> 라벨 개수도 %d개여야 합니다.', fname, M, M), ...
        'BackgroundColor', [0.94 0.94 0.94]);
    
    uicontrol(hPanel, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.05 0.45 0.15 0.20], 'String', '라벨 입력:', ...
        'HorizontalAlignment', 'right', 'FontSize', 10, 'BackgroundColor', [0.94 0.94 0.94]);

    hEdit = uicontrol(hPanel, 'Style', 'edit', 'Units', 'normalized', ...
        'Position', [0.22 0.48 0.5 0.25], 'FontSize', 11, ...
        'String', num2str(ones(1, M)), ...
        'Callback', @checkLength); 

    btnW = 0.15; btnH = 0.25; btnY = 0.10;
    
    uicontrol(hPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.05 btnY btnW btnH], 'String', '모두 1 (STS)', ...
        'Callback', @(~,~) setPreset(1));
        
    uicontrol(hPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.21 btnY btnW btnH], 'String', '모두 2 (SOS)', ...
        'Callback', @(~,~) setPreset(2));
        
    uicontrol(hPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.37 btnY btnW btnH], 'String', '마지막=1, 나머지=2', ...
        'Callback', @(~,~) setPreset(3));

    % [추가] Skip 버튼 (데이터 이상함)
    uicontrol(hPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.75 0.10 0.22 0.35], 'String', '이 파일 건너뛰기 (Skip)', ...
        'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [1 0.6 0.6], ... % 붉은색
        'Callback', @skipFile);

    % 입력 완료 버튼
    uicontrol(hPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.75 0.50 0.22 0.40], 'String', '입력 완료 (Confirm)', ...
        'FontSize', 12, 'FontWeight', 'bold', 'BackgroundColor', [0.8 1 0.8], ...
        'Callback', @confirmAndClose);

    final_label = [];

    uiwait(hFig);

    label = final_label;

    % --- Nested Callbacks ---
    function setPreset(mode)
        switch mode
            case 1, L = ones(1, M);
            case 2, L = ones(1, M) * 2;
            case 3
                if M >= 1
                    L = ones(1, M) * 2; L(end) = 1;
                else
                    L = [];
                end
        end
        set(hEdit, 'String', mat2str(L));
    end

    function checkLength(~, ~)
    end

    function skipFile(~, ~)
        final_label = []; % 빈 값이면 Main에서 Skip 처리
        uiresume(hFig);
    end

    function confirmAndClose(~, ~)
        raw_str = get(hEdit, 'String');
        raw_clean = regexprep(raw_str, '[\[\],;]', ' ');
        vals = sscanf(raw_clean, '%f');
        
        if length(vals) ~= M
            msgbox(sprintf('라벨 길이가 맞지 않습니다.\n현재: %d개 / 필요: %d개', length(vals), M), 'Error', 'error');
            return;
        end
        if any(~ismember(vals, [1 2]))
             ans_btn = questdlg('1 또는 2가 아닌 값이 있습니다. 진행할까요?', 'Warning', 'Yes', 'No', 'No');
             if strcmp(ans_btn, 'No'), return; end
        end
        final_label = vals(:);
        uiresume(hFig); 
    end
end


% -------------------------------------------------------------------------
% 2. Scatter Plot 내장형 삭제 도구
% -------------------------------------------------------------------------
function [fx, fy, fyr, lab, saveFlag] = manual_deletion_tool_plot2_embedded(hFig, targetAx, fx, fy, fyr, lab)
    hAx = targetAx;
    axes(hAx);
    
    hPanels = findobj(hFig, 'Type', 'uipanel');
    if ~isempty(hPanels)
        delete(hPanels.Children); 
        set(hPanels, 'Title', 'Outlier Deletion Tool');
    else
        hPanels = uipanel('Parent', hFig, 'Position', [0 0 1 0.22], 'Title', 'Outlier Deletion Tool');
    end
    
    % [삭제] 버튼 (좌측)
    btnDelete = uicontrol(hPanels, 'Style', 'pushbutton', 'String', '선택하여 삭제 (Delete Points)', ...
        'Units', 'normalized', 'Position', [0.05 0.3 0.25 0.4], ...
        'Callback', @enterDeleteMode, 'FontSize', 10, 'FontWeight','bold', 'BackgroundColor',[1 0.8 0.8]);
    
    % [추가] 저장 안함/Skip 버튼 (중앙)
    uicontrol(hPanels, 'Style', 'pushbutton', 'String', '저장 안 함 (Skip)', ...
        'Units', 'normalized', 'Position', [0.35 0.3 0.20 0.4], ...
        'Callback', @skipSave, 'FontSize', 10, 'FontWeight','bold', 'BackgroundColor',[0.8 0.8 0.8]);

    % [저장] 버튼 (우측)
    btnConfirm = uicontrol(hPanels, 'Style', 'pushbutton', 'String', '저장 및 다음 파일 (Save & Next)', ...
        'Units', 'normalized', 'Position', [0.60 0.3 0.35 0.4], ...
        'Callback', @confirmSave, 'FontSize', 10, 'FontWeight','bold', 'BackgroundColor',[0.8 1 0.8]);

    should_save = true; % 기본값: 저장함

    refreshPlot();
    uiwait(hFig);

    saveFlag = should_save;

    % --- Nested Callbacks ---
    function confirmSave(~,~)
        should_save = true;
        uiresume(hFig);
    end

    function skipSave(~,~)
        % 저장 안 하고 넘기기
        choice = questdlg('정말로 저장하지 않고 건너뛰시겠습니까?', 'Skip Check', 'Yes', 'No', 'No');
        if strcmp(choice, 'Yes')
            should_save = false;
            uiresume(hFig);
        end
    end

    function enterDeleteMode(~,~)
        set(btnDelete, 'Enable', 'off', 'String', '삭제 모드 실행 중...');
        set(btnConfirm, 'Enable', 'off');
        title(hAx, '삭제할 점(Feature)을 클릭하세요. (종료하려면 Enter)');
        
        while true
            try
                axes(hAx);
                [mx, my, button] = ginput(1);
            catch
                break;
            end
            if isempty(mx) || isempty(button) || button ~= 1
                break;
            end
            
            valid_mask = isfinite(fx);
            if ~any(valid_mask), break; end
            
            curr_indices = find(valid_mask);
            data_X = fx(curr_indices);
            data_Y = fy(curr_indices);
            
            dists = sqrt((data_X - mx).^2 + (data_Y - my).^2);
            [~, locIdx] = min(dists);
            realIdx = curr_indices(locIdx);
            
            fx(realIdx) = NaN; fy(realIdx) = NaN; fyr(realIdx)= NaN; lab(realIdx)= NaN;
            fprintf('Feature Index %d 삭제됨\n', realIdx);
            
            refreshPlot();
        end
        
        title(hAx, 'Feature scatter with labels');
        set(btnDelete, 'Enable', 'on', 'String', '선택하여 삭제 (Delete Points)');
        set(btnConfirm, 'Enable', 'on');
    end

    function refreshPlot()
        axes(hAx);
        cla(hAx); hold on; grid on; box on;
        
        valid = isfinite(fx) & isfinite(fy) & isfinite(lab);
        fx_v = fx(valid);
        fy_v = fy(valid);
        lab_v = lab(valid);
        
        valid_indices = find(valid);
        
        idxSTS = (lab_v == 1);
        idxSOS = (lab_v == 2);
        
        if any(idxSTS)
            plot(fx_v(idxSTS), fy_v(idxSTS), 'ro', 'MarkerFaceColor','r', 'MarkerSize', 5, 'DisplayName','STS');
        end
        if any(idxSOS)
            plot(fx_v(idxSOS), fy_v(idxSOS), 'bo', 'MarkerFaceColor','b', 'MarkerSize', 5, 'DisplayName','SOS');
        end
        
        text(fx_v, fy_v, string(valid_indices), ...
            'VerticalAlignment', 'bottom', ... 
            'HorizontalAlignment', 'left', ...
            'FontSize', 8, 'Color', 'k');
        
        xlabel('feat_x'); ylabel('feat_y'); title(hAx, 'Feature scatter with labels');
        axis tight;
        legend('Location','best');
    end
end


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
    % (기존 로직 동일)
    N  = length(Rdeg); dt = 1/fs;
    arb = zeros(N,1); hc_mask = zeros(N,1);
    feat_x = []; feat_y = []; feat_y_raw = []; idx_list = []; swing_side = [];
    hc_idx_R = []; hc_idx_L = []; up_idx_R = []; up_idx_L = []; low_idx_R = []; low_idx_L = [];
    
    % Params
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
        R_time_afterupFlag = R_time_afterupFlag + 1;
        L_time_afterupFlag = L_time_afterupFlag + 1;
        HC_time_afterFlag  = HC_time_afterFlag  + 1;
        R_swing_time = R_swing_time + 1; L_swing_time = L_swing_time + 1;

        R0 = Rdeg(i-2); R1 = Rdeg(i-1); R2 = Rdeg(i);
        L0 = Ldeg(i-2); L1 = Ldeg(i-1); L2 = Ldeg(i);

        if R_time_afterupFlag >= t_gap_R * (fs/1000), R_time_upcond = true; end
        if L_time_afterupFlag >= t_gap_L * (fs/1000), L_time_upcond = true; end
        if HC_time_afterFlag  >= min(t_gap_R, t_gap_L) * (fs/1000), HC_time_upcond  = true; end

        % HC check
        if HC_time_upcond && ((R2 - L2)*(R1 - L1) <= 0) && (R2 > thres_down_R) && (L2 > thres_down_L)
            if (R2 - R1) >= (L2 - L1) && Lpeakval > 40
                HC_time_upcond = false; HC_time_afterFlag = 0; hc_mask(i) = 1;
                pendR.valid = true; pendR.i = i; pendR.vel= (R2 - R1)/dt; pendR.T = R_swing_time;
                if ~isempty(low_idx_R), pendR.swing_start = low_idx_R(end); else, pendR.swing_start = max(1, i-round(1/dt)); end
                hc_idx_R(end+1,1) = i;
            elseif (L2 - L1) >= (R2 - R1) && Rpeakval > 40
                HC_time_upcond = false; HC_time_afterFlag = 0; hc_mask(i) = 1;
                pendL.valid = true; pendL.i = i; pendL.vel= (L2 - L1)/dt; pendL.T = L_swing_time;
                if ~isempty(low_idx_L), pendL.swing_start = low_idx_L(end); else, pendL.swing_start = max(1, i-round(1/dt)); end
                hc_idx_L(end+1,1) = i;
            end
        end

        % Upper R
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
        
        % Upper L
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