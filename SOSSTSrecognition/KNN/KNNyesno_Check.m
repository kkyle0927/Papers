clear; close all; clc;

%% 1) yesKNN 데이터 로드
disp('=== yesKNN 데이터 선택을 시작합니다 ===');
yesKNN = load_csv_data('Select CSV(s) for yesKNN (yesKNN용 파일 선택)');

if isempty(yesKNN)
    disp('데이터가 로드되지 않았습니다.');
    return;
end
fprintf('yesKNN 데이터 개수: %d\n', numel(yesKNN));

%% 2) 첫 번째 파일 기준
T_yes = yesKNN{1};
num_samples = height(T_yes);

%% 3) 이벤트 인덱스 추출: count가 올라가는 시점
get_event_idx = @(col) find(diff([0; double(T_yes.(col))]) > 0);

L_dpeak_idx = []; R_dpeak_idx = [];
L_upeak_idx = []; R_upeak_idx = [];
hc_idx      = [];

vars = T_yes.Properties.VariableNames;
if ismember('L_count_dpeak', vars), L_dpeak_idx = get_event_idx('L_count_dpeak'); end
if ismember('R_count_dpeak', vars), R_dpeak_idx = get_event_idx('R_count_dpeak'); end
if ismember('L_count_upeak', vars), L_upeak_idx = get_event_idx('L_count_upeak'); end
if ismember('R_count_upeak', vars), R_upeak_idx = get_event_idx('R_count_upeak'); end
if ismember('hc_count',      vars), hc_idx      = get_event_idx('hc_count');      end

fprintf('L_dpeak: %d, R_dpeak: %d, L_upeak: %d, R_upeak: %d, HC: %d events\n', ...
    numel(L_dpeak_idx), numel(R_dpeak_idx), numel(L_upeak_idx), numel(R_upeak_idx), numel(hc_idx));

%% 4) HC 구간당 dpeak/upeak 하나씩으로 필터링
L_dpeak_idx = filter_one_per_hc(L_dpeak_idx, T_yes.LeftThighAngle,  hc_idx, num_samples, 'min');
L_upeak_idx = filter_one_per_hc(L_upeak_idx, T_yes.LeftThighAngle,  hc_idx, num_samples, 'max');
R_dpeak_idx = filter_one_per_hc(R_dpeak_idx, T_yes.RightThighAngle, hc_idx, num_samples, 'min');
R_upeak_idx = filter_one_per_hc(R_upeak_idx, T_yes.RightThighAngle, hc_idx, num_samples, 'max');

fprintf('After filtering — L_dpeak: %d, R_dpeak: %d, L_upeak: %d, R_upeak: %d\n', ...
    numel(L_dpeak_idx), numel(R_dpeak_idx), numel(L_upeak_idx), numel(R_upeak_idx));

%% 5) 오프라인 Swing Phase 계산 + 구간 인덱스 저장
% dpeak 이후 각도가 RISE_TH 이상 오른 시점을 swing start(0%)로 사용
RISE_TH = 3.0;  % deg

sw_ph_calc_L = zeros(num_samples, 1);
sw_ph_calc_R = zeros(num_samples, 1);

% 각 swing의 [i_start, i_end] 저장 (subplot 4 사용)
L_swing_segs = [];   % Nx2
R_swing_segs = [];

for k = 1:numel(L_dpeak_idx)
    i_dp  = L_dpeak_idx(k);
    rise  = find(T_yes.LeftThighAngle(i_dp:end) > T_yes.LeftThighAngle(i_dp) + RISE_TH, 1);
    if isempty(rise), continue; end
    i_start = i_dp + rise - 1;
    next_up = L_upeak_idx(L_upeak_idx > i_start);
    if isempty(next_up), continue; end
    i_end = next_up(1);
    span  = i_end - i_start;
    if span <= 0, continue; end
    sw_ph_calc_L(i_start:i_end) = (0:span)' / span * 100;
    L_swing_segs(end+1, :) = [i_start, i_end]; %#ok<AGROW>
end

for k = 1:numel(R_dpeak_idx)
    i_dp  = R_dpeak_idx(k);
    rise  = find(T_yes.RightThighAngle(i_dp:end) > T_yes.RightThighAngle(i_dp) + RISE_TH, 1);
    if isempty(rise), continue; end
    i_start = i_dp + rise - 1;
    next_up = R_upeak_idx(R_upeak_idx > i_start);
    if isempty(next_up), continue; end
    i_end = next_up(1);
    span  = i_end - i_start;
    if span <= 0, continue; end
    sw_ph_calc_R(i_start:i_end) = (0:span)' / span * 100;
    R_swing_segs(end+1, :) = [i_start, i_end]; %#ok<AGROW>
end

%% 6) Online swing 구간 추출 (swing_phase_RT > 0 인 연속 구간)
has_online   = ismember('swing_phase_RT_L', vars);
has_tau_cmd  = ismember('s_tau_cmd_L',      vars);
has_tau_orig = ismember('s_tau_original_L', vars);

L_online_segs = [];
R_online_segs = [];
if has_online
    L_online_segs = find_active_segs(T_yes.swing_phase_RT_L);
    R_online_segs = find_active_segs(T_yes.swing_phase_RT_R);
end

%% 7) Figure (4x1)
figure('Name', 'Gait Event & Phase Verification', 'Color', 'w', 'Position', [100 100 850 1050]);

% ── Subplot 1: Thigh Angle + Event Markers ──────────────────────────────
subplot(4, 1, 1); hold on; grid on; box on;
plot(T_yes.LeftThighAngle,  'b-', 'LineWidth', 1.0, 'DisplayName', 'Left Thigh');
plot(T_yes.RightThighAngle, 'r-', 'LineWidth', 1.0, 'DisplayName', 'Right Thigh');

if ~isempty(L_dpeak_idx)
    plot(L_dpeak_idx, T_yes.LeftThighAngle(L_dpeak_idx), ...
        'bv', 'MarkerFaceColor','b', 'MarkerSize',7, 'DisplayName','L-dpeak');
end
if ~isempty(R_dpeak_idx)
    plot(R_dpeak_idx, T_yes.RightThighAngle(R_dpeak_idx), ...
        'rv', 'MarkerFaceColor','r', 'MarkerSize',7, 'DisplayName','R-dpeak');
end
if ~isempty(L_upeak_idx)
    plot(L_upeak_idx, T_yes.LeftThighAngle(L_upeak_idx), ...
        'b^', 'MarkerFaceColor','b', 'MarkerSize',7, 'DisplayName','L-upeak');
end
if ~isempty(R_upeak_idx)
    plot(R_upeak_idx, T_yes.RightThighAngle(R_upeak_idx), ...
        'r^', 'MarkerFaceColor','r', 'MarkerSize',7, 'DisplayName','R-upeak');
end
if ~isempty(hc_idx)
    y_hc = min([T_yes.LeftThighAngle; T_yes.RightThighAngle]) - 3;
    plot(hc_idx, repmat(y_hc, size(hc_idx)), ...
        'ks', 'MarkerFaceColor','k', 'MarkerSize',5, 'DisplayName','HC');
end
ylabel('Angle (deg)');
title('Thigh Angles & Gait Events');
legend('Location', 'bestoutside');

% ── Subplot 2: Recorded Torque (Left only) ───────────────────────────────
subplot(4, 1, 2); hold on; grid on; box on;
if has_tau_cmd
    plot(T_yes.s_tau_cmd_L, 'b-',  'LineWidth', 1.2, 'DisplayName', 'Adaptive L (Nm)');
    if has_tau_orig
        plot(T_yes.s_tau_original_L, 'b--', 'LineWidth', 1.0, 'DisplayName', 'Original L (Nm)');
    end
else
    plot(T_yes.LeftHipTorque * 1.59, 'b-', 'LineWidth', 1.2, 'DisplayName', 'Hip L (Nm)');
end
ylabel('Torque (Nm)');
title('Recorded Torque - Left (solid: adaptive, dashed: original)');
legend('Location', 'best');

% ── Subplot 3: Gait Phase Comparison (Left only) ─────────────────────────
subplot(4, 1, 3); hold on; grid on; box on;
if has_online
    plot(T_yes.swing_phase_RT_L, 'b-',  'LineWidth', 1.0, 'DisplayName', 'Online Phase L');
end
plot(sw_ph_calc_L, 'c--', 'LineWidth', 1.5, 'DisplayName', 'Offline Phase L');
ylabel('Gait Phase (%)'); ylim([0 120]);
xlabel('Samples');
title('Gait Phase: Online vs Offline - Left');
legend('Location', 'best');

% ── Subplot 4: Swing별 Torque vs Phase 비교 (Left only) ──────────────────
subplot(4, 1, 4); hold on; grid on; box on;

% 오프라인 이상 프로파일 (trapezoidal 공식 직접 계산, 1개 선)
TRAP_OFFSET = 0.00; TRAP_RISE = 0.20; TRAP_PLAT = 0.20; TRAP_FALL = 0.20;
tau_max_L = 0;
if has_tau_orig
    tau_max_L = max(T_yes.s_tau_original_L);
elseif has_tau_cmd
    tau_max_L = max(T_yes.s_tau_cmd_L);
end
ph_grid   = linspace(0, 100, 500);
tau_ideal = trap_profile(ph_grid/100, tau_max_L, TRAP_OFFSET, TRAP_RISE, TRAP_PLAT, TRAP_FALL);
plot(ph_grid, tau_ideal, 'k--', 'LineWidth', 2.0, 'DisplayName', 'Ideal Profile L');

% offline phase 기준으로 실제 인가한 보조력: swing마다 다른 색
% (x축 = sw_ph_calc_L, y축 = s_tau_cmd_L, 같은 샘플 인덱스)
if has_online && has_tau_cmd && ~isempty(L_online_segs)
    n_sw   = size(L_online_segs, 1);
    colors = colormap(lines(n_sw));
    for k = 1:n_sw
        seg = L_online_segs(k,1):L_online_segs(k,2);
        plot(sw_ph_calc_L(seg), T_yes.s_tau_cmd_L(seg), ...
            '-', 'LineWidth', 1.5, 'Color', colors(k,:), ...
            'DisplayName', sprintf('Swing %d', k));
    end
end

xlim([0 120]); ylim([0 4]); xlabel('Offline Swing Phase (%)'); ylabel('Torque (Nm)');
title('Per-Swing: Actual Torque vs Offline Phase - Left  (dashed: ideal)');
legend('Location', 'best');

%% ── 로컬 함수 ────────────────────────────────────────────────────────────

% Trapezoidal torque profile (phase_frac: 0~1 입력)
function tau = trap_profile(phase_frac, tau_max, offset, rise, plat, fall)
    e_off  = offset;
    e_rise = offset + rise;
    e_plat = e_rise + plat;
    e_fall = e_plat + fall;
    tau = zeros(size(phase_frac));
    tau(phase_frac >= e_rise & phase_frac < e_plat) = tau_max;
    mask_rise = phase_frac >= e_off & phase_frac < e_rise;
    if rise > 0
        tau(mask_rise) = tau_max * (phase_frac(mask_rise) - e_off) / rise;
    end
    mask_fall = phase_frac >= e_plat & phase_frac < e_fall;
    if fall > 0
        tau(mask_fall) = tau_max * (1 - (phase_frac(mask_fall) - e_plat) / fall);
    end
end

% swing_phase > 0 인 연속 구간의 [start, end] 반환
function segs = find_active_segs(phase_col)
    active = phase_col > 0;
    d = diff([0; active; 0]);
    starts = find(d == 1);
    ends   = find(d == -1) - 1;
    segs   = [starts, ends];
end

% HC 구간당 이벤트 하나씩 선택
function filt_idx = filter_one_per_hc(ev_idx, angle, hc_idx, num_samples, mode)
    filt_idx = [];
    if isempty(ev_idx), return; end
    boundaries = [0; hc_idx(:); num_samples + 1];
    for k = 1:numel(boundaries) - 1
        seg_s = boundaries(k) + 1;
        seg_e = boundaries(k+1) - 1;
        in = ev_idx(ev_idx >= seg_s & ev_idx <= seg_e);
        if isempty(in), continue; end
        if strcmp(mode, 'min')
            [~, mi] = min(angle(in));
        else
            [~, mi] = max(angle(in));
        end
        filt_idx(end+1) = in(mi); %#ok<AGROW>
    end
    filt_idx = filt_idx(:);
end

function data_cell = load_csv_data(dialog_title)
    [csvFiles, csvPath] = uigetfile({'*.csv','CSV Files (*.csv)'}, dialog_title, 'MultiSelect','on');
    if isequal(csvFiles, 0), data_cell = {}; return; end
    if ischar(csvFiles), csvFiles = {csvFiles}; end
    data_cell = cell(numel(csvFiles), 1);
    for iFile = 1:numel(csvFiles)
        fPath = fullfile(csvPath, csvFiles{iFile});
        opts = detectImportOptions(fPath, 'Encoding','UTF-8');
        opts.DataLines = [2 Inf];
        opts.VariableNamingRule = 'preserve';
        data_cell{iFile} = readtable(fPath, opts);
    end
end
