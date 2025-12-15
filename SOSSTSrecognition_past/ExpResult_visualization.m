clear; close all; clc;

%% ===== 1) Reference dataset load =====
% ref_all_scaled: [x y label], (가능하면) scale_factor = [sx sy]
[refFile, refPath] = uigetfile({'*.mat','MAT-files (*.mat)'}, ...
    'refdataset_merged_scaled.mat 선택');
if isequal(refFile,0), error('Reference MAT 파일을 선택해야 합니다.'); end
Sref = load(fullfile(refPath, refFile));
if ~isfield(Sref,'ref_all_scaled')
    error('선택한 MAT 파일에 ref_all_scaled 변수가 없습니다.');
end
ref_all_scaled = Sref.ref_all_scaled;
if isfield(Sref,'scale_factor') && numel(Sref.scale_factor)>=2
    scale_factor = Sref.scale_factor(:).';
else
    warning('scale_factor가 없어 ref 범위를 사용합니다.');
    rx_rng = max(ref_all_scaled(:,1)) - min(ref_all_scaled(:,1)); if rx_rng<=0, rx_rng=1; end
    ry_rng = max(ref_all_scaled(:,2)) - min(ref_all_scaled(:,2)); if ry_rng<=0, ry_rng=1; end
    scale_factor = [rx_rng ry_rng];
end
sx = scale_factor(1); sy = scale_factor(2);

% 기본 파라미터
fs = 1000; fc = 60;
t_gap = 300; thres_up = 10; thres_down = 10; var_crit = 0;
k = 9;                   % KNN 이웃 수
ASK_COL_MARK = true;     % 피험자별 색/마커 UI에서 선택 (false면 자동 배정)
SHOW_HC_MARK = true;     % 라벨링용 각도 플롯에서 HC 마커 표시 여부

% 자동 팔레트/마커 (필요시 순환)
autoColors = [ ...
    0.89 0.10 0.11;  % red
    0.12 0.47 0.71;  % blue
    0.17 0.63 0.17;  % green
    0.58 0.40 0.74;  % purple
    1.00 0.50 0.00;  % orange
    0.65 0.34 0.16;  % brown
    0.10 0.10 0.10;  % dark gray
];
autoMarkers = {'o','s','^','d','p','h','v'};

% Reference로 학습 준비
rx   = ref_all_scaled(:,1);
ry   = ref_all_scaled(:,2);
rlab = ref_all_scaled(:,3);
mdl_ref = fitcknn([rx ry], rlab, 'NumNeighbors', k, 'Standardize', true);

%% ===== 2) 피험자 데이터 입력 (파일 선택) =====
SUBJECTS = {}; subjIdx = 0;
while true
    choice = questdlg('피험자를 추가하시겠습니까?', 'Subjects', ...
                      '추가','종료','추가');
    if ~strcmp(choice,'추가'), break; end
    subjIdx = subjIdx + 1;

    % 피험자 이름
    defname = sprintf('Subject_%02d', subjIdx);
    ansName = inputdlg('피험자 이름:', 'Subject name', [1 50], {defname});
    if isempty(ansName), subjIdx = subjIdx-1; break; end
    sname = strtrim(ansName{1}); if isempty(sname), sname = defname; end

    % CSV 다중 선택
    [fns, folder] = uigetfile({'*.csv','CSV files (*.csv)'; '*.*','All files'}, ...
                               sprintf('[%s] CSV 파일을 선택하세요 (여러 개 가능)', sname), ...
                               'MultiSelect','on');
    if isequal(fns,0)
        uiwait(warndlg('파일이 선택되지 않아 피험자 추가를 건너뜁니다.'));
        subjIdx = subjIdx-1; continue;
    end
    if ischar(fns), fns = {fns}; end
    fullpaths = cellfun(@(fn) fullfile(folder, fn), fns, 'UniformOutput', false);

    % 색/마커 선택 (선택적)
    if ASK_COL_MARK
        try
            cSel = uisetcolor(autoColors(mod(subjIdx-1,size(autoColors,1))+1,:), ...
                              sprintf('[%s] 색상 선택', sname));
            if length(cSel)~=3, cSel = autoColors(mod(subjIdx-1,size(autoColors,1))+1,:); end
        catch
            cSel = autoColors(mod(subjIdx-1,size(autoColors,1))+1,:);
        end
        [mkIdx, okMk] = listdlg('PromptString',sprintf('[%s] 마커 선택', sname), ...
                                'SelectionMode','single', ...
                                'ListString',autoMarkers, 'InitialValue',1);
        if okMk==0, mkSel = autoMarkers{mod(subjIdx-1,numel(autoMarkers))+1};
        else,       mkSel = autoMarkers{mkIdx};
        end
    else
        cSel = autoColors(mod(subjIdx-1,size(autoColors,1))+1,:);
        mkSel = autoMarkers{mod(subjIdx-1,numel(autoMarkers))+1};
    end

    SUBJECTS{end+1} = struct('name', sname, 'files', {fullpaths}, ...
                             'color', cSel, 'marker', mkSel); %#ok<SAGROW>
end
if isempty(SUBJECTS), error('선택된 피험자가 없습니다.'); end

%% ===== 2-1) 피처 추출 (라벨링 전, 모든 파일에 대해) =====
% 각 피험자 구조에 filesData 필드를 만들어 피처/메타를 모아둠
for si = 1:numel(SUBJECTS)
    S = SUBJECTS{si};
    filesData = struct('path', {}, 't', {}, 'Rdeg', {}, 'Ldeg', {}, ...
                       'feat_x', {}, 'feat_y', {}, 'idx_list', {}, 'M', {});
    for fi = 1:numel(S.files)
        fp = S.files{fi};
        try
            T = readtable(fp);
        catch ME
            warning('파일 읽기 실패: %s (%s)', fp, ME.message);
            continue;
        end

        % 열 이름 정렬 (질의의 스키마와 동일)
        T.Properties.VariableNames = { ...
            'loopCnt','assist_level', ...
            'thighDeg_RH','incPosDeg_RH','MotorActCurrent_RH','accX_Calib_RH','accY_Calib_RH','gyroZ_Calib_RH', ...
            'thighDeg_LH','incPosDeg_LH','MotorActCurrent_LH','accX_Calib_LH','accY_Calib_LH','gyroZ_Calib_LH', ...
            'u_RH','u_LH','Pvector_ref_RH','Pvector_ref_LH','extension_control_mode', ...
            'emg_R1','emg_L1','fsr_R1','fsr_R2','fsr_L1','fsr_L2','free_var1','free_var2'};

        % 신호/필터
        Rdeg = LPF(T.thighDeg_RH, fs, fc);
        Ldeg = LPF(T.thighDeg_LH, fs, fc);
        t    = (0:height(T)-1).' * 0.001;

        % 특징 추출
        [hc_mask, feat_x, feat_y, idx_list, ~, ~, ~, ~, ~, ~] = ...
            compute_features_RT(Rdeg, Ldeg, fs, t_gap, thres_up, thres_down, var_crit, 1, 1); %#ok<ASGLU>

        filesData(end+1) = struct( ... %#ok<SAGROW>
            'path', fp, 't', t, 'Rdeg', Rdeg, 'Ldeg', Ldeg, ...
            'feat_x', feat_x, 'feat_y', feat_y, 'idx_list', idx_list, ...
            'M', numel(feat_x));
    end
    SUBJECTS{si}.filesData = filesData;
end

%% ===== 2-2) 라벨링 방식 선택: (A) 개별 플롯 or (B) 파일명-일괄 수동 =====
for si = 1:numel(SUBJECTS)
    S = SUBJECTS{si};
    if isempty(S.filesData), warning('[%s] 파일 데이터 없음', S.name); continue; end

    modeSel = questdlg(sprintf('[%s] 라벨 입력 방식을 선택하세요', S.name), ...
        'Labeling Mode', '개별 플롯 라벨링', '파일명-일괄 수동', '개별 플롯 라벨링');

    switch modeSel
        case '파일명-일괄 수동'
            % ---- (B) 파일명-일괄 수동 입력 ----
            prompts = cell(numel(S.filesData),1);
            defIn   = cell(numel(S.filesData),1);
            for fi = 1:numel(S.filesData)
                [~, base, ext] = fileparts(S.filesData(fi).path);
                M = S.filesData(fi).M;
                prompts{fi} = sprintf('%s%s  (길이 %d)  [키워드: all1, last1]', base, ext, M);
                defIn{fi}   = ''; % 기본 공란 = 전부 1로 처리됨
            end

            ok = false;
            while ~ok
                answers = inputdlg(prompts, sprintf('[%s] 파일명-일괄 수동 라벨 입력', S.name), [1 80], defIn);
                if isempty(answers)
                    choice = questdlg('입력을 취소했습니다. 전부 1(STS)로 채울까요?', ...
                        '취소됨', '전부 1', '다시 입력', '전부 1');
                    if strcmp(choice,'전부 1')
                        for fi = 1:numel(S.filesData)
                            M = S.filesData(fi).M;
                            SUBJECTS{si}.filesData(fi).label = ones(M,1);
                        end
                        ok = true;
                    else
                        continue;
                    end
                else
                    % 각 파일별 파싱
                    anyError = false;
                    for fi = 1:numel(S.filesData)
                        M = S.filesData(fi).M;
                        raw = strtrim(answers{fi});
                        try
                            SUBJECTS{si}.filesData(fi).label = parse_label_string(raw, M);
                        catch ME
                            uiwait(errordlg(sprintf('(%d/%d) %s', fi, numel(S.filesData), ME.message), ...
                                            '라벨 파싱 오류','modal'));
                            anyError = true; break;
                        end
                    end
                    ok = ~anyError;
                end
            end

        otherwise
            % ---- (A) 개별 플롯 라벨링 ----
            for fi = 1:numel(S.filesData)
                fd = S.filesData(fi);
                t = fd.t; Rdeg = fd.Rdeg; Ldeg = fd.Ldeg; idx_list = fd.idx_list; M = fd.M;
                fLabel = figure('Name', sprintf('수동 라벨링 (%s) - %s', S.name, fd.path), 'Color','w');
                plot(t, Rdeg, 'r'); hold on; plot(t, Ldeg, 'b');
                if SHOW_HC_MARK && ~isempty(idx_list)
                    ymark = min([Rdeg;Ldeg]) + 0.05*(max([Rdeg;Ldeg])-min([Rdeg;Ldeg]));
                    plot(t(idx_list), ymark*ones(size(idx_list)), 'k|', 'LineWidth',1.2, 'DisplayName','HC events');
                end
                grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
                legend({'Rdeg','Ldeg','HC events'},'Location','best');
                title({'플롯 확인 후 라벨 벡터를 입력하세요 (1=STS, 2=SOS)', sprintf('파일: %s', fd.path)}, 'Interpreter','none');

                uiwait(msgbox( ...
                    sprintf(['HC 이벤트 개수: %d개\n라벨 벡터 길이도 %d이어야 합니다.\n' ...
                             '1=STS, 2=SOS\n옵션 키워드: all1(전부1), last1(마지막만1)'], M, M), ...
                    '안내','modal'));

                label = [];
                while true
                    answer = inputdlg({sprintf('길이 %d의 label 벡터 입력', M)}, ...
                                      sprintf('라벨 수동 입력 - %s', fd.path), [1 80], {''});
                    if isempty(answer)
                        choice2 = questdlg('입력을 취소했습니다. 전부 1(STS)로 채울까요?', ...
                                          '취소됨', '전부 1', '다시 입력', '전부 1');
                        if strcmp(choice2,'전부 1')
                            label = ones(M,1); break;
                        else
                            continue;
                        end
                    end
                    try
                        label = parse_label_string(strtrim(answer{1}), M);
                        break;
                    catch ME
                        uiwait(errordlg(ME.message, '라벨 파싱 오류','modal'));
                    end
                end
                close(fLabel);
                SUBJECTS{si}.filesData(fi).label = label;
            end
    end
end

%% ===== 2-3) 스케일링 & ALL_TEST 적재 =====
ALL_TEST = [];
for si = 1:numel(SUBJECTS)
    if ~isfield(SUBJECTS{si}, 'filesData'), continue; end
    for fi = 1:numel(SUBJECTS{si}.filesData)
        fd = SUBJECTS{si}.filesData(fi);
        if ~isfield(fd,'label'), continue; end
        tx = fd.feat_x / sx;
        ty = fd.feat_y / sy;
        M  = fd.M;
        subjCol = si * ones(M,1);
        ALL_TEST = [ALL_TEST; [tx(:), ty(:), fd.label(:), subjCol]]; %#ok<AGROW>
    end
end
if isempty(ALL_TEST)
    error('테스트 데이터가 비어 있습니다. CSV 선택/라벨링을 확인하세요.');
end

%% ===== 3) KNN Grid 계산 & 단일 Figure 시각화 =====
% 0~1 범위를 꽉 채우는 그리드
nx = 400; ny = 400;
xv = linspace(0,1,nx); yv = linspace(0,1,ny);
[Xg, Yg] = meshgrid(xv, yv);
[pred_grid, scores_grid] = predict(mdl_ref, [Xg(:) Yg(:)]);
Z = reshape(pred_grid, size(Xg));

% 배경 RGB (흰띠 제거, 클래스 단색)
baseRed=[1 0 0]; baseBlue=[0 0 1];
RGB = zeros([size(Z) 3]); mask1=(Z==1); mask2=(Z==2);
for c=1:3, RGB(:,:,c) = baseRed(c)*mask1 + baseBlue(c)*mask2; end

figure('Name', sprintf('KNN Multi-Subject (K=%d) — UI Select + Labeling', k), 'Color','w'); hold on;
% 배경을 [0 1]로 정확히 채움 + 옅은 알파
hImg = image('XData',[0 1], 'YData',[0 1], 'CData', RGB);
set(gca,'YDir','normal'); set(hImg,'AlphaData',0.20);

% 결정경계 (점수 차 0 등고선)
hCt = gobjects(1);
if size(scores_grid,2) >= 2
    Sd = reshape(scores_grid(:,1)-scores_grid(:,2), size(Xg));
    [~, hCt] = contour(Xg, Yg, Sd, [0 0], 'k--','LineWidth',1.0);
end

% Reference 점 (아주 옅게)
hRef = scatter(rx, ry, 12, [0.5 0.5 0.5], 'filled', ...
    'MarkerFaceAlpha',0.10, 'MarkerEdgeAlpha',0.10);

% 모든 test 예측/정확도
tx_all = ALL_TEST(:,1); ty_all = ALL_TEST(:,2);
tlab_all = ALL_TEST(:,3); subj_all = ALL_TEST(:,4);
pred_all = predict(mdl_ref, [tx_all ty_all]);
acc_all  = mean(pred_all == tlab_all);

% % 피험자별로 색/마커 적용 (정분류=마커, 오분류=해당 피험자 색의 X)
% hSubjOK  = gobjects(0); legOKNames  = {};
% hSubjBad = gobjects(0); legBadNames = {};
% 
% for si = 1:numel(SUBJECTS)
%     maskS = (subj_all==si);
%     if ~any(maskS), continue; end
%     ci = SUBJECTS{si}.color; mk = SUBJECTS{si}.marker;
% 
%     ok  = maskS & (pred_all==tlab_all);
%     bad = maskS & (pred_all~=tlab_all);
% 
%     if any(ok)
%         hOk = scatter(tx_all(ok), ty_all(ok), 48, ci, 'filled', ...
%                       'Marker', mk, 'MarkerEdgeColor','k','LineWidth',0.5);
%         hSubjOK(end+1)   = hOk; %#ok<AGROW>
%         legOKNames{end+1}= sprintf('%s (OK)', SUBJECTS{si}.name); %#ok<AGROW>
%     end
%     if any(bad)
%         hBad = plot(tx_all(bad), ty_all(bad), 'x', ...
%                     'MarkerSize', 9, 'LineWidth', 1.6, 'Color', ci);
%         uistack(hBad, 'top'); % X를 가장 위로
%         hSubjBad(end+1)   = hBad; %#ok<AGROW>
%         legBadNames{end+1}= sprintf('%s (X)', SUBJECTS{si}.name); %#ok<AGROW>
%     end
% end

% ==== BEGIN CUSTOM SUBJECT STYLES (replace this block) ====

% 피험자별로 색/마커 적용 (정분류=마커, 오분류=해당 피험자 색의 X)
hSubjOK  = gobjects(0); legOKNames  = {};
hSubjBad = gobjects(0); legBadNames = {};

% 요청 사양: 1번=초록 사각형, 2번=밝은주황 삼각형
% - 테두리 없음
% - 크기: 기존(48)보다 조금 큰 64
msOK = 250;      % OK / X 모두 동일 사이즈
lwX  = 1.6;     % X 라인 두께

color_subj1 = [0.18 0.70 0.23];  % 초록
marker_subj1 = 's';              % 사각형
color_subj2 = [1.00 0.60 0.20];  % 밝은 주황
marker_subj2 = '^';              % 세모

for si = 1:numel(SUBJECTS)
    maskS = (subj_all==si);
    if ~any(maskS), continue; end

    % 기본은 SUBJECTS 설정, 단 1/2번 피험자는 요구한 스타일로 override
    if     si == 1
        ci = color_subj1;
        mk = marker_subj1;
        edgeColor = 'none';   % 테두리 없음
    elseif si == 2
        ci = color_subj2;
        mk = marker_subj2;
        edgeColor = 'none';   % 테두리 없음
    else
        ci = SUBJECTS{si}.color;
        mk = SUBJECTS{si}.marker;
        edgeColor = 'k';      % 나머지는 테두리 유지 (원하면 'none'으로 변경)
    end

    ok  = maskS & (pred_all==tlab_all);
    bad = maskS & (pred_all~=tlab_all);

    % 정분류: 지정 마커/색, 테두리 옵션 반영, 사이즈 msOK
    if any(ok)
        hOk = scatter(tx_all(ok), ty_all(ok), msOK, ci, 'filled', ...
                      'Marker', mk, 'MarkerEdgeColor', edgeColor, ...
                      'LineWidth', 0.5);
        hSubjOK(end+1)    = hOk; %#ok<AGROW>
        legOKNames{end+1} = sprintf('%s (OK)', SUBJECTS{si}.name); %#ok<AGROW>
    end

    % 오분류: 같은 색의 'x' 마커, 크기도 msOK로 통일
    if any(bad)
        hBad = scatter(tx_all(bad), ty_all(bad), msOK, ...
                       'Marker', 'x', 'MarkerEdgeColor', ci, ...
                       'MarkerFaceColor', 'none', 'LineWidth', lwX);
        uistack(hBad, 'top');   % X를 가장 위로
        hSubjBad(end+1)    = hBad; %#ok<AGROW>
        legBadNames{end+1} = sprintf('%s (X)', SUBJECTS{si}.name); %#ok<AGROW>
    end
end

% ==== END CUSTOM SUBJECT STYLES ====



% 축/제목/범례
xlim([0 1]); ylim([0 1]); axis equal; grid on; box on;
% xlabel('x (scaled)'); ylabel('y (scaled)');
% title(sprintf('K=%d | All Test Acc = %.2f%%', k, 100*acc_all));

% 범례: 경계, Reference, 피험자별 OK, 피험자별 X
legH = []; legN = {};
if isgraphics(hCt),  legH(end+1)=hCt;  legN{end+1}='Decision boundary'; end %#ok<AGROW>
if isgraphics(hRef), legH(end+1)=hRef; legN{end+1}='Reference (faint)'; end %#ok<AGROW>
if ~isempty(hSubjOK)
    legH = [legH, hSubjOK];
    legN = [legN, legOKNames];
end
if ~isempty(hSubjBad)
    legH = [legH, hSubjBad];
    legN = [legN, legBadNames];
end
if ~isempty(legH), legend(legH, legN, 'Location','bestoutside'); end
hold off;
xlim([0 1]); ylim([0 1]);
set(gca,'XTickLabel',[],'YTickLabel',[]);   % <-- 눈금 숫자 제거

%% ---- Command Window Log: 피험자별 Accuracy ----
fprintf('\n==== Subject-wise Accuracy Report ====\n');
for si = 1:numel(SUBJECTS)
    maskS = (subj_all==si);
    if ~any(maskS), continue; end
    acc_si = mean(pred_all(maskS) == tlab_all(maskS));
    fprintf('%s: %.2f %%  (총 %d개 이벤트)\n', ...
        SUBJECTS{si}.name, 100*acc_si, sum(maskS));
end
fprintf('-------------------------------------\n');
fprintf('Overall Accuracy: %.2f %%  (총 %d개 이벤트)\n\n', ...
    100*acc_all, numel(tlab_all));


%% ================= Helper Functions =================
function label = parse_label_string(raw, M)
% 키워드: 'all1' => 전부 1, 'last1' => 마지막만 1(나머지 2)
% 공란('') => 전부 1
    if isempty(raw)
        label = ones(M,1); return;
    end
    if strcmpi(raw,'all1')
        label = ones(M,1); return;
    end
    if strcmpi(raw,'last1')
        if M==0, label = []; else, label = 2*ones(M,1); label(end)=1; end
        return;
    end
    raw_clean = regexprep(raw, '[\[\],;]', ' ');
    nums = sscanf(raw_clean, '%f');
    if numel(nums) ~= M
        error('라벨 길이는 %d이어야 합니다. 현재 %d개가 입력되었습니다.', M, numel(nums));
    end
    if any(~ismember(nums, [1 2]))
        error('라벨에는 1/2만 허용됩니다. (입력값: %s)', raw);
    end
    label = nums(:);
end

function [hc_mask, feat_x, feat_y, idx_list, ...
          hc_idx_R, hc_idx_L, up_idx_R, up_idx_L, low_idx_R, low_idx_L] = ...
    compute_features_RT(Rdeg, Ldeg, fs, t_gap, thres_up, thres_down, var_crit, scaling_X, scaling_Y)

    N = length(Rdeg); dt = 1/fs;
    R_time_afterupFlag = 0;  L_time_afterupFlag = 0;  HC_time_afterFlag = 0;
    R_swing_time = 0;        L_swing_time = 0;
    R_time_upcond = false;   L_time_upcond = false;   HC_time_upcond = false;
    HC_Rswingflag4upcond = 0; HC_Lswingflag4upcond = 0;

    swing_period  = 400;
    R_lowpeak_val = thres_down; L_lowpeak_val = thres_down;
    Rpeakval = 50; Lpeakval = 50;

    hc_mask = zeros(N,1); feat_x = []; feat_y = []; idx_list = [];
    hc_idx_R = []; hc_idx_L = []; up_idx_R = []; up_idx_L = []; low_idx_R= []; low_idx_L= [];

    for i = 3:N
        R_time_afterupFlag = R_time_afterupFlag + 1;
        L_time_afterupFlag = L_time_afterupFlag + 1;
        HC_time_afterFlag  = HC_time_afterFlag + 1;
        R_swing_time       = R_swing_time + 1;
        L_swing_time       = L_swing_time + 1;

        R0 = Rdeg(i-2); R1 = Rdeg(i-1); R2 = Rdeg(i);
        L0 = Ldeg(i-2); L1 = Ldeg(i-1); L2 = Ldeg(i);

        if R_time_afterupFlag >= t_gap, R_time_upcond = true; end
        if L_time_afterupFlag >= t_gap, L_time_upcond = true; end
        if HC_time_afterFlag  >= t_gap, HC_time_upcond  = true; end

        % HC
        if HC_time_upcond && ((R2 - L2)*(R1 - L1) <= 0) && (R2 > thres_down) && (L2 > thres_down)
            HC_time_upcond    = false;
            HC_time_afterFlag = 0;
            hc_mask(i) = 1;

            if (R2 - R1) >= (L2 - L1) && Lpeakval > 40
                vel_HC = (R2 - R1)/dt;  T_HC = R_swing_time; HC_Rswingflag4upcond = 1; hc_idx_R(end+1,1) = i;
            elseif (L2 - L1) >= (R2 - R1) && Rpeakval > 40
                vel_HC = (L2 - L1)/dt;  T_HC = L_swing_time; HC_Lswingflag4upcond = 1; hc_idx_L(end+1,1) = i;
            else
                continue;
            end

            norm_vel_HC = (vel_HC * (swing_period * dt)) / scaling_X;
            norm_T_HC   = (T_HC   / (swing_period * dt)) / scaling_Y;

            feat_x(end+1,1)  = norm_vel_HC;
            feat_y(end+1,1)  = norm_T_HC;
            idx_list(end+1,1)= i;
        end

        % Upper peak (R/L)
        if ((R2 - R1)*(R1 - R0) < var_crit) && ((R2 - R1) < 0) && (R2 >= thres_up) && R_time_upcond && HC_Rswingflag4upcond
            Rpeakval = R2; R_time_upcond = false; swing_period = R_swing_time; R_time_afterupFlag = 0; HC_Rswingflag4upcond = 0; up_idx_R(end+1,1) = i;
        end
        if ((L2 - L1)*(L1 - L0) < var_crit) && ((L2 - L1) < 0) && (L2 >= thres_up) && L_time_upcond && HC_Lswingflag4upcond
            Lpeakval = L2; L_time_upcond = false; swing_period = L_swing_time; L_time_afterupFlag = 0; HC_Lswingflag4upcond = 0; up_idx_L(end+1,1) = i;
        end

        % Lower peak (R/L)
        if (R2 > R_lowpeak_val), R_lowpeak_val = thres_down; end
        if ((R2 - R1)*(R1 - R0) < var_crit) && ((R2 - R1) >= 0) && (R2 <= R_lowpeak_val + 3) && (R2 - R1) - (L2 - L1) > 0
            R_lowpeak_val = R2; R_swing_time = 0; low_idx_R(end+1,1) = i;
        end
        if (L2 > L_lowpeak_val), L_lowpeak_val = thres_down; end
        if ((L2 - L1)*(L1 - L0) < var_crit) && ((L2 - L1) > 0) && (L2 <= L_lowpeak_val + 3) && (R2 - R1) - (L2 - L1) < 0
            L_lowpeak_val = L2; L_swing_time = 0; low_idx_L(end+1,1) = i;
        end
    end
end

function y = LPF(x, fs, fc)
    y = zeros(length(x),1);
    T = 1/fs; RC = 1/(2*pi*fc); a = T / (T + RC);
    for i = 2:length(x)
        y(i) = (1-a) * y(i-1) + a * x(i);
    end
end
