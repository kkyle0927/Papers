
clear; close all; clc;

%% 1) yesKNN 데이터 로드 (adaptive ON)
disp('=== yesKNN 데이터 선택을 시작합니다 ===');
yesKNN = load_csv_data('Select CSV(s) for yesKNN (adaptive ON)');

%% 2) noKNN 데이터 로드 (adaptive OFF)
disp('=== noKNN 데이터 선택을 시작합니다 ===');
noKNN = load_csv_data('Select CSV(s) for noKNN (adaptive OFF)');

%% 로드 완료 메시지
disp('모든 데이터 로드가 완료되었습니다.');
fprintf('yesKNN 데이터 개수: %d\n', numel(yesKNN));
fprintf('noKNN 데이터 개수: %d\n', numel(noKNN));

%% 3) Figure 1: yesKNN (adaptive ON) — one figure per file
for iFile = 1:numel(yesKNN)
    T_yes = yesKNN{iFile};

    [leftThighVel_yes, rightThighVel_yes] = compute_thigh_velocity(T_yes);
    leftPower_yes  = leftThighVel_yes  .* T_yes.LeftHipTorque;
    rightPower_yes = rightThighVel_yes .* T_yes.RightHipTorque;

    ev_yes        = detect_gait_events(T_yes);
    hc_labels_yes = label_sos_sts(ev_yes.hc_idx, 2);

    tau_cf_R_yes = gen_counterfactual_yes2no(T_yes.s_tau_cmd_R, ...
        ev_yes.hc_idx, ev_yes.up_R_idx, hc_labels_yes, ev_yes.trig_R_idx);
    tau_cf_L_yes = gen_counterfactual_yes2no(T_yes.s_tau_cmd_L, ...
        ev_yes.hc_idx, ev_yes.up_L_idx, hc_labels_yes, ev_yes.trig_L_idx);

    figure('Name', sprintf('Adaptive ON (yesKNN) — file %d', iFile), ...
           'Color', 'w', 'Position', [100 50 950 1050]);

    ran_a = 1; ran_b = height(T_yes);

    subplot(5, 1, 1);
    hold on; grid on; box on;
    plot(T_yes.LeftThighAngle,  'b-', 'LineWidth', 1.5, 'DisplayName', 'Left Thigh');
    plot(T_yes.RightThighAngle, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Right Thigh');
    plot_gait_events(gca, ev_yes, hc_labels_yes, T_yes);
    ylabel('Angle (deg)');
    title(sprintf('Adaptive ON [%d] - Thigh Angles', iFile));
    legend('Location', 'best');
    xlim([ran_a, ran_b]);

    subplot(5, 1, 2);
    hold on; grid on; box on;
    plot(T_yes.LeftHipTorque,  'b-', 'LineWidth', 1.5, 'DisplayName', 'Left Hip Torque');
    plot(T_yes.RightHipTorque, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Right Hip Torque');
    ylabel('Torque (Nm)');
    title(sprintf('Adaptive ON [%d] - Hip Torques', iFile));
    legend('Location', 'best');
    xlim([ran_a, ran_b]);

    subplot(5, 1, 3);
    hold on; grid on; box on;
    plot(T_yes.s_tau_cmd_R, 'r-',  'LineWidth', 1.5, 'DisplayName', 'Assist R (actual)');
    plot(T_yes.s_tau_cmd_L, 'b-',  'LineWidth', 1.5, 'DisplayName', 'Assist L (actual)');
    plot(tau_cf_R_yes, 'g--', 'LineWidth', 1.5, 'DisplayName', 'R if no adaptive');
    plot(tau_cf_L_yes, 'Color', [0 0.7 0], 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'L if no adaptive');
    plot_gait_events_lines_only(gca, ev_yes, hc_labels_yes);
    ylabel('Torque (Nm)');
    title(sprintf('Adaptive ON [%d] - Assist Comparison (green = if adaptive OFF)', iFile));
    legend('Location', 'best');
    xlim([ran_a, ran_b]);

    subplot(5, 1, 4);
    hold on; grid on; box on;
    plot(leftThighVel_yes,  'b-', 'LineWidth', 1.5, 'DisplayName', 'Left Thigh Vel');
    plot(rightThighVel_yes, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Right Thigh Vel');
    yline(0, 'k--', 'LineWidth', 1);
    ylabel('Angular Velocity (rad/s)');
    title(sprintf('Adaptive ON [%d] - Thigh Angular Velocity', iFile));
    legend('Location', 'best');
    xlim([ran_a, ran_b]);

    subplot(5, 1, 5);
    hold on; grid on; box on;
    plot(leftPower_yes,  'b-', 'LineWidth', 1.5, 'DisplayName', 'Left Hip Power');
    plot(rightPower_yes, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Right Hip Power');
    yline(0, 'k--', 'LineWidth', 1);
    xlabel('Samples');
    ylabel('Power (W)');
    title(sprintf('Adaptive ON [%d] - Hip Power', iFile));
    legend('Location', 'best');
    xlim([ran_a, ran_b]);
end

%% 4) Figure 2: noKNN (adaptive OFF) — one figure per file
for iFile = 1:numel(noKNN)
    T_no = noKNN{iFile};

    [leftThighVel_no, rightThighVel_no] = compute_thigh_velocity(T_no);
    leftPower_no  = leftThighVel_no  .* T_no.LeftHipTorque;
    rightPower_no = rightThighVel_no .* T_no.RightHipTorque;

    ev_no        = detect_gait_events(T_no);
    hc_labels_no = label_sos_sts(ev_no.hc_idx, 2);

    tau_cf_R_no = gen_counterfactual_no2yes(T_no.s_tau_cmd_R, ...
        ev_no.hc_idx, hc_labels_no);
    tau_cf_L_no = gen_counterfactual_no2yes(T_no.s_tau_cmd_L, ...
        ev_no.hc_idx, hc_labels_no);

    figure('Name', sprintf('Adaptive OFF (noKNN) — file %d', iFile), ...
           'Color', 'w', 'Position', [1100 50 950 1050]);

    ran_a = 1; ran_b = height(T_no);

    subplot(5, 1, 1);
    hold on; grid on; box on;
    plot(T_no.LeftThighAngle,  'b-', 'LineWidth', 1.5, 'DisplayName', 'Left Thigh');
    plot(T_no.RightThighAngle, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Right Thigh');
    plot_gait_events(gca, ev_no, hc_labels_no, T_no);
    ylabel('Angle (deg)');
    title(sprintf('Adaptive OFF [%d] - Thigh Angles', iFile));
    legend('Location', 'best');
    xlim([ran_a, ran_b]);

    subplot(5, 1, 2);
    hold on; grid on; box on;
    plot(T_no.LeftHipTorque,  'b-', 'LineWidth', 1.5, 'DisplayName', 'Left Hip Torque');
    plot(T_no.RightHipTorque, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Right Hip Torque');
    ylabel('Torque (Nm)');
    title(sprintf('Adaptive OFF [%d] - Hip Torques', iFile));
    legend('Location', 'best');
    xlim([ran_a, ran_b]);

    subplot(5, 1, 3);
    hold on; grid on; box on;
    plot(T_no.s_tau_cmd_R, 'r-',  'LineWidth', 1.5, 'DisplayName', 'Assist R (actual)');
    plot(T_no.s_tau_cmd_L, 'b-',  'LineWidth', 1.5, 'DisplayName', 'Assist L (actual)');
    plot(tau_cf_R_no, 'g--', 'LineWidth', 1.5, 'DisplayName', 'R if adaptive ON');
    plot(tau_cf_L_no, 'Color', [0 0.7 0], 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'L if adaptive ON');
    plot_gait_events_lines_only(gca, ev_no, hc_labels_no);
    ylabel('Torque (Nm)');
    title(sprintf('Adaptive OFF [%d] - Assist Comparison (green = if adaptive ON)', iFile));
    legend('Location', 'best');
    xlim([ran_a, ran_b]);

    subplot(5, 1, 4);
    hold on; grid on; box on;
    plot(leftThighVel_no,  'b-', 'LineWidth', 1.5, 'DisplayName', 'Left Thigh Vel');
    plot(rightThighVel_no, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Right Thigh Vel');
    yline(0, 'k--', 'LineWidth', 1);
    ylabel('Angular Velocity (rad/s)');
    title(sprintf('Adaptive OFF [%d] - Thigh Angular Velocity', iFile));
    legend('Location', 'best');
    xlim([ran_a, ran_b]);

    subplot(5, 1, 5);
    hold on; grid on; box on;
    plot(leftPower_no,  'b-', 'LineWidth', 1.5, 'DisplayName', 'Left Hip Power');
    plot(rightPower_no, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Right Hip Power');
    yline(0, 'k--', 'LineWidth', 1);
    xlabel('Samples');
    ylabel('Power (W)');
    title(sprintf('Adaptive OFF [%d] - Hip Power', iFile));
    legend('Location', 'best');
    xlim([ran_a, ran_b]);
end

%% 5) Figure 3: STS Assist Analysis (yesKNN vs noKNN, Left leg, all files pooled)
if ~isempty(yesKNN) && ~isempty(noKNN)
    STS_TARGET = 20;  % STS angle target — mean STS HC angle from data (~20 deg)

    % Accumulate stats across ALL yesKNN files (left leg STS only)
    dur_yes = []; past_yes = []; ang_yes = []; work_ratio_yes = []; peaks_yes = []; ov_yes = [];
    impulse_past_yes = []; pre_ratio_yes = [];
    for iFile = 1:numel(yesKNN)
        T_f   = yesKNN{iFile};
        ev_f  = detect_gait_events(T_f);
        lbl_f = label_sos_sts(ev_f.hc_idx, 2);
        s = extract_sts_swing_stats(T_f, ev_f, lbl_f, 'L');
        p = extract_sts_peak_stats(T_f,  ev_f, lbl_f, 'L', STS_TARGET);
        dur_yes          = [dur_yes,          s.duration_ms];
        past_yes         = [past_yes,         s.time_past_20_ms];
        ang_yes          = [ang_yes,          s.angle_at_end];
        work_ratio_yes   = [work_ratio_yes,   s.unnecessary_work_ratio];
        impulse_past_yes = [impulse_past_yes, s.post_target_impulse_Nms];
        pre_ratio_yes    = [pre_ratio_yes,    s.pre_target_ratio];
        peaks_yes        = [peaks_yes,        p.peak_angle];
        ov_yes           = [ov_yes,           p.overshoot];
    end

    % Accumulate stats across ALL noKNN files (left leg STS only)
    dur_no = []; past_no = []; ang_no = []; work_ratio_no = []; peaks_no = []; ov_no = [];
    impulse_past_no = []; pre_ratio_no = [];
    for iFile = 1:numel(noKNN)
        T_f   = noKNN{iFile};
        ev_f  = detect_gait_events(T_f);
        lbl_f = label_sos_sts(ev_f.hc_idx, 2);
        s = extract_sts_swing_stats(T_f, ev_f, lbl_f, 'L');
        p = extract_sts_peak_stats(T_f,  ev_f, lbl_f, 'L', STS_TARGET);
        dur_no          = [dur_no,          s.duration_ms];
        past_no         = [past_no,         s.time_past_20_ms];
        ang_no          = [ang_no,          s.angle_at_end];
        work_ratio_no   = [work_ratio_no,   s.unnecessary_work_ratio];
        impulse_past_no = [impulse_past_no, s.post_target_impulse_Nms];
        pre_ratio_no    = [pre_ratio_no,    s.pre_target_ratio];
        peaks_no        = [peaks_no,        p.peak_angle];
        ov_no           = [ov_no,           p.overshoot];
    end

    figure('Name', 'STS Assist Analysis (Left)', 'Color', 'w', 'Position', [100 50 1200 800]);

    % [1,1] Total assist duration
    subplot(2, 3, 1);
    plot_group_stats(gca, dur_yes, dur_no, 'Duration (ms)', 'Total Assist Duration');

    % [1,2] Angle at assist termination
    subplot(2, 3, 2);
    plot_group_stats(gca, ang_yes, ang_no, 'Thigh Angle (deg)', 'Angle at Assist Termination');
    yline(STS_TARGET, 'b--', sprintf('%d° (STS target)', STS_TARGET), ...
        'LineWidth', 1.2, 'LabelHorizontalAlignment', 'left');
    yline(40, 'r--', '40° (SOS target)', 'LineWidth', 1.2, 'LabelHorizontalAlignment', 'left');

    % [1,3] Peak thigh angle
    subplot(2, 3, 3);
    plot_group_stats(gca, peaks_yes, peaks_no, 'Peak Thigh Angle (deg)', 'STS Swing Peak Angle');
    yline(STS_TARGET, 'b--', sprintf('%d° (STS target)', STS_TARGET), ...
        'LineWidth', 1.2, 'LabelHorizontalAlignment', 'left');
    yline(40, 'r--', '40° (SOS target)', 'LineWidth', 1.2, 'LabelHorizontalAlignment', 'left');

    % [2,1] Time past STS target
    subplot(2, 3, 4);
    plot_group_stats(gca, past_yes, past_no, 'Time (ms)', ...
        sprintf('Time Past %d° Until Assist End', STS_TARGET));

    % [2,2] Peak overshoot above STS target
    subplot(2, 3, 5);
    plot_group_stats(gca, ov_yes, ov_no, 'Overshoot (deg)', ...
        sprintf('Peak Overshoot above %d° (STS target)', STS_TARGET));
    yline(0, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');

    % [2,3] Unnecessary work ratio (angle > STS target) / total work
    subplot(2, 3, 6);
    plot_group_stats(gca, work_ratio_yes * 100, work_ratio_no * 100, 'Unnecessary Work (%)', ...
        sprintf('Unnecessary Work Ratio (angle > %d°)', STS_TARGET));
    yline(0, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
end

%% 6) Figure 4: Key metrics summary (2×2)
if ~isempty(yesKNN) && ~isempty(noKNN)
    figure('Name', 'STS Key Metrics (Left)', 'Color', 'w', 'Position', [200 150 1000 750]);

    % [1,1] Time past target angle until assist end
    subplot(2, 2, 1);
    plot_group_stats(gca, past_yes, past_no, 'Time (ms)', ...
        sprintf('Time Past %d° Until Assist End', STS_TARGET));

    % [1,2] Unnecessary work ratio
    subplot(2, 2, 2);
    plot_group_stats(gca, work_ratio_yes * 100, work_ratio_no * 100, 'Unnecessary Work (%)', ...
        sprintf('Unnecessary Work Ratio (angle > %d°)', STS_TARGET));
    yline(0, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');

    % [2,1] Post-target angle impulse
    subplot(2, 2, 3);
    plot_group_stats(gca, impulse_past_yes, impulse_past_no, 'Impulse (Nm·s)', ...
        sprintf('Post-%d° Assist Impulse', STS_TARGET));
    yline(0, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');

    % [2,2] Pre-target assist ratio (= "유효하게 쓰인 assist" 비율)
    subplot(2, 2, 4);
    plot_group_stats(gca, pre_ratio_yes * 100, pre_ratio_no * 100, 'Effective Assist (%)', ...
        sprintf('Assist Ratio Before %d° (Effective)', STS_TARGET));
    yline(100, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
end

%% ========================= Local Functions =========================

function data_cell = load_csv_data(dialog_title)
    [csvFiles, csvPath] = uigetfile({'*.csv','CSV Files (*.csv)'}, dialog_title, 'MultiSelect','on');
    if isequal(csvFiles, 0)
        warning('파일 선택이 취소되었습니다.');
        data_cell = {};
        return;
    end
    if ischar(csvFiles), csvFiles = {csvFiles}; end
    data_cell = cell(numel(csvFiles), 1);
    for iFile = 1:numel(csvFiles)
        csvFullPath = fullfile(csvPath, csvFiles{iFile});
        fprintf('Loading (%d/%d): %s\n', iFile, numel(csvFiles), csvFiles{iFile});
        opts = detectImportOptions(csvFullPath, 'Encoding','UTF-8');
        opts.DataLines = [2 Inf];
        opts.VariableNamingRule = 'preserve';
        data_cell{iFile} = readtable(csvFullPath, opts);
    end
end

function [leftVel, rightVel] = compute_thigh_velocity(T)
    leftAngle_rad  = deg2rad(T.LeftThighAngle);
    rightAngle_rad = deg2rad(T.RightThighAngle);
    dt = 0.002; % 500Hz
    leftVel  = gradient(leftAngle_rad, dt);
    rightVel = gradient(rightAngle_rad, dt);
end

function ev = detect_gait_events(T)
    N = height(T);

    % HC: hc_count 증가 시점
    hc = diff(T.hc_count);
    ev.hc_idx = find(hc > 0) + 1;

    % Upper peak: R/L count_upeak 증가 시점
    ev.up_R_idx = find(diff(T.R_count_upeak) > 0) + 1;
    ev.up_L_idx = find(diff(T.L_count_upeak) > 0) + 1;

    % Trigger: tau_cmd가 0→>0 전환
    tau_R = T.s_tau_cmd_R;
    tau_L = T.s_tau_cmd_L;
    ev.trig_R_idx = [];
    ev.trig_L_idx = [];
    for i = 2:N
        if tau_R(i) > 0.01 && tau_R(i-1) <= 0.01
            ev.trig_R_idx(end+1) = i;
        end
        if tau_L(i) > 0.01 && tau_L(i-1) <= 0.01
            ev.trig_L_idx(end+1) = i;
        end
    end
end

function labels = label_sos_sts(hc_idx, n_sos)
    % 앞 n_sos개는 SOS, 나머지는 STS
    n = numel(hc_idx);
    labels = cell(n, 1);
    for i = 1:n
        if i <= n_sos
            labels{i} = 'SOS';
        else
            labels{i} = 'STS';
        end
    end
end

function plot_gait_events(ax, ev, hc_labels, T)
    % HC: 검정 세로 점선 + SOS/STS 텍스트
    yl = ylim(ax);
    for i = 1:numel(ev.hc_idx)
        idx = ev.hc_idx(i);
        xline(ax, idx, 'k--', 'LineWidth', 0.8, 'HandleVisibility', 'off');
        text(ax, idx, yl(2)*0.95, hc_labels{i}, ...
            'FontSize', 8, 'FontWeight', 'bold', 'Color', 'k', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    end
    % Upper peak R: 빨간 ▼
    for i = 1:numel(ev.up_R_idx)
        idx = ev.up_R_idx(i);
        if idx <= height(T)
            plot(ax, idx, T.RightThighAngle(idx), 'rv', 'MarkerSize', 6, ...
                'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
        end
    end
    % Upper peak L: 파란 ▼
    for i = 1:numel(ev.up_L_idx)
        idx = ev.up_L_idx(i);
        if idx <= height(T)
            plot(ax, idx, T.LeftThighAngle(idx), 'bv', 'MarkerSize', 6, ...
                'MarkerFaceColor', 'b', 'HandleVisibility', 'off');
        end
    end
    % Trigger R: 빨간 ○
    for i = 1:numel(ev.trig_R_idx)
        idx = ev.trig_R_idx(i);
        if idx <= height(T)
            plot(ax, idx, T.RightThighAngle(idx), 'ro', 'MarkerSize', 5, ...
                'HandleVisibility', 'off');
        end
    end
    % Trigger L: 파란 ○
    for i = 1:numel(ev.trig_L_idx)
        idx = ev.trig_L_idx(i);
        if idx <= height(T)
            plot(ax, idx, T.LeftThighAngle(idx), 'bo', 'MarkerSize', 5, ...
                'HandleVisibility', 'off');
        end
    end
end

function plot_gait_events_lines_only(ax, ev, hc_labels)
    % HC: 검정 세로 점선 + SOS/STS 텍스트 (마커 없음, y축 안 건드림)
    yl = ylim(ax);
    for i = 1:numel(ev.hc_idx)
        idx = ev.hc_idx(i);
        xline(ax, idx, 'k--', 'LineWidth', 0.8, 'HandleVisibility', 'off');
        text(ax, idx, yl(2)*0.95, hc_labels{i}, ...
            'FontSize', 8, 'FontWeight', 'bold', 'Color', 'k', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    end
end

function tau_cf = gen_counterfactual_yes2no(tau_actual, hc_idx, up_idx, hc_labels, trig_idx)
    % Figure 1 (adaptive ON): STS에서 adaptive 안 했다면?
    % 가상 개형: HC에서 fall하지 않고 upper peak까지 tau_max 유지 → 30ms(15 samples) fall
    N = numel(tau_actual);
    tau_cf = tau_actual; % 기본은 실제값 복사
    fall_samples = 15; % 30ms / 2ms = 15 samples

    % STS인 HC 찾기
    sts_hc = [];
    for i = 1:numel(hc_idx)
        if strcmp(hc_labels{i}, 'STS')
            sts_hc(end+1) = hc_idx(i);
        end
    end

    % 각 STS HC에 대해: HC 이후 실제 tau가 fall했을 구간을 tau_max로 유지,
    % 그리고 다음 upper peak에서 fall
    for si = 1:numel(sts_hc)
        hc_i = sts_hc(si);

        % HC 시점의 tau_max (HC 직전 값)
        tau_max_val = tau_actual(max(1, hc_i - 1));
        if tau_max_val < 0.01
            continue;
        end

        % 이 swing의 trigger 시점 찾기 (HC 이전 가장 가까운 trigger)
        prev_trigs = trig_idx(trig_idx < hc_i);
        if isempty(prev_trigs)
            continue;
        end

        % 다음 upper peak 찾기
        next_ups = up_idx(up_idx > hc_i);
        if isempty(next_ups)
            up_i = min(N, hc_i + 250); % fallback
        else
            up_i = next_ups(1);
        end

        % HC ~ upper peak: tau_max 유지
        for k = hc_i:min(up_i, N)
            tau_cf(k) = tau_max_val;
        end

        % Upper peak에서 30ms fall
        for k = 1:fall_samples
            idx = up_i + k;
            if idx <= N
                tau_cf(idx) = tau_max_val * (1 - k / fall_samples);
                if tau_cf(idx) < 0, tau_cf(idx) = 0; end
            end
        end

        % Fall 이후 0
        for k = (up_i + fall_samples + 1):min(N, up_i + fall_samples + 50)
            tau_cf(k) = 0;
        end
    end
end

function tau_cf = gen_counterfactual_no2yes(tau_actual, hc_idx, hc_labels)
    % Figure 2 (adaptive OFF): STS에서 adaptive 했다면?
    % 가상 개형: HC에서 30ms(15 samples) fall 시작
    N = numel(tau_actual);
    tau_cf = tau_actual; % 기본은 실제값 복사
    fall_samples = 15; % 30ms / 2ms

    % STS인 HC 찾기
    sts_hc = [];
    for i = 1:numel(hc_idx)
        if strcmp(hc_labels{i}, 'STS')
            sts_hc(end+1) = hc_idx(i);
        end
    end

    for si = 1:numel(sts_hc)
        hc_i = sts_hc(si);

        % HC 시점의 tau 값
        tau_at_hc = tau_actual(hc_i);
        if tau_at_hc < 0.01
            continue;
        end

        % HC에서 30ms fall
        for k = 0:fall_samples
            idx = hc_i + k;
            if idx <= N
                tau_cf(idx) = tau_at_hc * (1 - k / fall_samples);
                if tau_cf(idx) < 0, tau_cf(idx) = 0; end
            end
        end

        % Fall 이후: 다음 trigger 또는 끝까지 0
        fall_end = hc_i + fall_samples + 1;
        % 다음 STS HC 또는 데이터 끝까지
        next_bound = N;
        remaining_hc = hc_idx(hc_idx > hc_i);
        if ~isempty(remaining_hc)
            next_bound = remaining_hc(1) - 1;
        end
        for k = fall_end:min(N, next_bound)
            if tau_cf(k) > 0
                tau_cf(k) = 0;
            end
        end
    end
end

function stats = extract_sts_swing_stats(T, ev, hc_labels, side)
    % STS 스윙별 보조력 overshoot 지표 추출
    % 각 trigger를 스윙 단위로 정의하고, 윈도우 내 첫 HC가 STS인 경우만 분석
    dt = 0.002;
    tau_thresh = 0.5; % Nm — 보조력 "종료" 기준

    if strcmp(side, 'R')
        tau   = T.s_tau_cmd_R;
        angle = T.RightThighAngle;
        trig_idx = ev.trig_R_idx;
    else
        tau   = T.s_tau_cmd_L;
        angle = T.LeftThighAngle;
        trig_idx = ev.trig_L_idx;
    end

    N = numel(tau);
    omega_rad = gradient(deg2rad(angle), dt);  % angular velocity (rad/s), full signal

    stats.duration_ms              = [];
    stats.time_past_20_ms          = [];
    stats.time_past_40_ms          = [];
    stats.angle_at_end             = [];
    stats.unnecessary_work_J       = [];
    stats.unnecessary_work_ratio   = [];
    stats.post_target_impulse_Nms  = [];
    stats.pre_target_ratio         = [];
    stats.traj_angle               = {};
    stats.traj_tau                 = {};

    for k = 1:numel(trig_idx)
        t_trig = trig_idx(k);
        if k < numel(trig_idx)
            t_win_end = trig_idx(k+1) - 1;
        else
            t_win_end = N;
        end

        % 이 윈도우 내 첫 번째 HC 찾기 → STS인지 확인
        hc_in_win = ev.hc_idx(ev.hc_idx > t_trig & ev.hc_idx <= t_win_end);
        if isempty(hc_in_win)
            continue;
        end
        hc_local = hc_in_win(1);
        hc_pos = find(ev.hc_idx == hc_local, 1);
        if isempty(hc_pos) || strcmp(hc_labels{hc_pos}, 'SOS')
            continue; % SOS 스윙은 skip
        end

        % Assist end: 윈도우 내 tau > tau_thresh인 마지막 sample
        tau_win = tau(t_trig:t_win_end);
        last_assist = find(tau_win > tau_thresh, 1, 'last');
        if isempty(last_assist)
            continue; % 유효한 보조력 없음
        end
        assist_end_idx = t_trig + last_assist - 1;

        % 구간 각도 배열 (trigger ~ assist end)
        angle_seg = angle(t_trig:assist_end_idx);

        % Duration (ms)
        duration_ms = (assist_end_idx - t_trig) * dt * 1000;

        % 20° crossing 이후 잔여 시간 (STS target = mean HC angle)
        cross_20 = find(angle_seg >= 20, 1, 'first');
        if ~isempty(cross_20)
            cross_20_abs = t_trig + cross_20 - 1;
            time_past_20_ms = (assist_end_idx - cross_20_abs) * dt * 1000;
        else
            time_past_20_ms = NaN;
        end

        % 40° crossing 이후 잔여 시간
        cross_40 = find(angle_seg >= 40, 1, 'first');
        if ~isempty(cross_40)
            cross_40_abs = t_trig + cross_40 - 1;
            time_past_40_ms = (assist_end_idx - cross_40_abs) * dt * 1000;
        else
            time_past_40_ms = NaN;
        end

        % Angle at assist end
        angle_at_end = angle(assist_end_idx);

        % Unnecessary mechanical work: ∫P dt for angle > 20° region (STS target)
        tau_seg   = tau(t_trig:assist_end_idx);
        omega_seg = omega_rad(t_trig:assist_end_idx);
        P_seg     = tau_seg .* omega_seg;
        mask_20   = angle_seg >= 20;
        unnecessary_work_J = sum(P_seg(mask_20)) * dt;
        total_work_J       = sum(P_seg) * dt;
        if total_work_J > 0
            unnecessary_work_ratio = unnecessary_work_J / total_work_J;
        else
            unnecessary_work_ratio = NaN;
        end

        % Post-target impulse: ∫τ dt for angle >= 20°
        post_target_impulse_Nms = sum(tau_seg(mask_20)) * dt;

        % Pre-target assist ratio: impulse before target / total impulse
        total_impulse = sum(tau_seg) * dt;
        if total_impulse > 0
            pre_target_ratio = sum(tau_seg(~mask_20)) / sum(tau_seg);
        else
            pre_target_ratio = NaN;
        end

        % 저장
        stats.duration_ms(end+1)              = duration_ms;
        stats.time_past_20_ms(end+1)          = time_past_20_ms;
        stats.time_past_40_ms(end+1)          = time_past_40_ms;
        stats.angle_at_end(end+1)             = angle_at_end;
        stats.unnecessary_work_J(end+1)       = unnecessary_work_J;
        stats.unnecessary_work_ratio(end+1)   = unnecessary_work_ratio;
        stats.post_target_impulse_Nms(end+1)  = post_target_impulse_Nms;
        stats.pre_target_ratio(end+1)         = pre_target_ratio;
        stats.traj_angle{end+1}               = angle(t_trig:assist_end_idx);
        stats.traj_tau{end+1}                 = tau(t_trig:assist_end_idx);
    end
end

function stats = extract_sts_peak_stats(T, ev, hc_labels, side, target_angle)
    % STS 스윙별 upper peak 각도와 target 대비 overshoot 추출
    if strcmp(side, 'R')
        angle    = T.RightThighAngle;
        trig_idx = ev.trig_R_idx;
        up_idx   = ev.up_R_idx;
    else
        angle    = T.LeftThighAngle;
        trig_idx = ev.trig_L_idx;
        up_idx   = ev.up_L_idx;
    end

    N = numel(angle);
    stats.peak_angle = [];
    stats.overshoot  = [];

    for k = 1:numel(trig_idx)
        t_trig = trig_idx(k);
        if k < numel(trig_idx)
            t_win_end = trig_idx(k+1) - 1;
        else
            t_win_end = N;
        end

        % HC 라벨 확인: STS인지
        hc_in_win = ev.hc_idx(ev.hc_idx > t_trig & ev.hc_idx <= t_win_end);
        if isempty(hc_in_win), continue; end
        hc_local = hc_in_win(1);
        hc_pos = find(ev.hc_idx == hc_local, 1);
        if isempty(hc_pos) || strcmp(hc_labels{hc_pos}, 'SOS'), continue; end

        % 이 윈도우 내 첫 번째 upper peak 찾기
        up_in_win = up_idx(up_idx > t_trig & up_idx <= t_win_end);
        if isempty(up_in_win), continue; end
        up_i = up_in_win(1);

        peak_ang = angle(up_i);
        stats.peak_angle(end+1) = peak_ang;
        stats.overshoot(end+1)  = peak_ang - target_angle;
    end
end

function plot_group_stats(ax, vals_yes, vals_no, ylabel_str, title_str)
    % scatter + mean ± SD 비교 (small N용)
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    jitter = 0.12;

    if ~isempty(vals_yes)
        x_yes = ones(1, numel(vals_yes)) + (rand(1, numel(vals_yes)) - 0.5) * jitter * 2;
        scatter(ax, x_yes, vals_yes, 55, 'b', 'filled', 'MarkerFaceAlpha', 0.7, ...
            'DisplayName', 'Adaptive ON');
        m = mean(vals_yes, 'omitnan');
        s = std(vals_yes, 'omitnan');
        plot(ax, [0.75, 1.25], [m, m], 'b-', 'LineWidth', 2.5, 'HandleVisibility', 'off');
        errorbar(ax, 1, m, s, 'b', 'LineWidth', 1.5, 'CapSize', 8, 'HandleVisibility', 'off');
    end

    if ~isempty(vals_no)
        x_no = 2*ones(1, numel(vals_no)) + (rand(1, numel(vals_no)) - 0.5) * jitter * 2;
        scatter(ax, x_no, vals_no, 55, 'r', 'filled', 'MarkerFaceAlpha', 0.7, ...
            'DisplayName', 'Adaptive OFF');
        m = mean(vals_no, 'omitnan');
        s = std(vals_no, 'omitnan');
        plot(ax, [1.75, 2.25], [m, m], 'r-', 'LineWidth', 2.5, 'HandleVisibility', 'off');
        errorbar(ax, 2, m, s, 'r', 'LineWidth', 1.5, 'CapSize', 8, 'HandleVisibility', 'off');
    end

    set(ax, 'XTick', [1, 2], 'XTickLabel', {'Adaptive ON', 'Adaptive OFF'});
    xlim(ax, [0.4, 2.6]);
    ylabel(ax, ylabel_str);
    title(ax, title_str);
    legend(ax, 'Location', 'best');
end
