clear all;
close all;
clc;

%% H10 data load
% cd 'C:\Users\ChanyoungKo\OneDrive - 한국과학기술원 기계공학과\Kyle\KAIST\ExoLab\성과\Publication\Ongoing\1저자_2025_미정_Real-time Gait Mode Recognition in Stair Climbing\250814_SOSSTSdata_KAIST4stair'
cd 'C:\Users\ChanyoungKo\OneDrive - 한국과학기술원 기계공학과\Kyle\KAIST\ExoLab\성과\Publication\Ongoing\1저자_2025_미정_Real-time Gait Mode Recognition in Stair Climbing\exp data\250829_CYK_validation_exp_mixedmode_bigassist'
file_H10 = {'CYK_250829_mixmode_selfpaced_validation_bigF2.csv'};

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

%% ARB
figure;
subplot(3,1,1);
plot(data_H10.t, data_H10.thighDeg_RH,'r','LineWidth',3); hold on;
plot(data_H10.t, data_H10.thighDeg_LH,'b','LineWidth',3); hold on; grid on;
legend('R Thigh deg', 'L Thigh deg', 'Assistance trigger moment');
subplot(3,1,2);
plot(data_H10.t, data_H10.free_var1); hold on;
subplot(3,1,3);
plot(data_H10.t, data_H10.free_var2); hold on;

%% trigger / HC moment
idx_trig = find(data_H10.free_var1 >= 0);
idx_hc = find(data_H10.free_var2 >= 0);

%% Real data plot
figure;
subplot(3,1,1);
plot(data_H10.t, data_H10.thighDeg_RH,'r','LineWidth',2); hold on;
plot(data_H10.t, data_H10.thighDeg_LH,'b','LineWidth',2); hold on; grid on;
plot(data_H10.t, data_H10.free_var1*30,'k','LineWidth',1.5); hold on; grid on;
legend('R Thigh deg', 'L Thigh deg', 'Assistance trigger moment');
subplot(3,1,2);
plot(data_H10.t, data_H10.free_var2,'k','LineWidth',1.5); hold on; grid on;
legend('Recognized gait type (1-STS, 2-SOS)');
subplot(3,1,3);
plot(data_H10.t, (data_H10.free_var1+1),'k','LineWidth',1.5); hold on; grid on;
plot(data_H10.t, data_H10.MotorActCurrent_RH*1.59,'r','LineWidth',2); hold on;
plot(data_H10.t, data_H10.MotorActCurrent_LH*1.59,'b','LineWidth',2); grid on;
legend('Assistance trigger moment', 'R assist torque', 'L assist torque');

%% 논문용 figure
fontsize = 30;   % ✅ 전체 폰트 크기 지정

figure;

% ==== Subplot 1 ====
subplot(3,1,1);
plot(data_H10.t, data_H10.thighDeg_RH,'r','LineWidth',2); hold on;
plot(data_H10.t, data_H10.thighDeg_LH,'b','LineWidth',2); hold on;
xlim([0 16]); ylim([-10 60]);

plot(data_H10.t(idx_hc), data_H10.thighDeg_LH(idx_hc), 'o', ...
    'MarkerSize',12, 'LineWidth',2, ...
    'MarkerEdgeColor',[1 0.5 0], 'MarkerFaceColor','w');  

% ✅ idx_trig: 초록색 별표로 표시
for k = 1:length(idx_trig)
    xPos = data_H10.t(idx_trig(k));
    yPos = data_H10.thighDeg_LH(idx_trig(k));
    plot(xPos, yPos, 'g*', 'MarkerSize',14, 'LineWidth',2);
end
xticklabels([]); xlabel('');
% yl1 = ylabel('Thigh angle [^\circ]', 'FontSize',fontsize);

% ==== Subplot 2 ====
subplot(3,1,2);
plot(data_H10.t, data_H10.free_var2,'k','LineWidth',2); hold on;
xlim([0 16]); ylim([0 2.5]);

plot(data_H10.t(idx_hc), data_H10.free_var2(idx_hc), 'o', ...
    'MarkerSize',12, 'LineWidth',2, ...
    'MarkerEdgeColor',[1 0.5 0], 'MarkerFaceColor','w');
xticklabels([]); xlabel('');
% yl2 = ylabel('gait mode', 'FontSize',fontsize);

% ==== Subplot 3 ====
subplot(3,1,3);
% plot(data_H10.t, (data_H10.free_var1+1),'k','LineWidth',1.5); hold on;
plot(data_H10.t, data_H10.MotorActCurrent_RH*1.59,'r','LineWidth',2); hold on;
plot(data_H10.t, data_H10.MotorActCurrent_LH*1.59,'b','LineWidth',2);
xlim([0 16]); ylim([0 2.5]);

plot(data_H10.t(idx_hc), data_H10.MotorActCurrent_LH(idx_hc)*1.59, 'o', ...
    'MarkerSize',12, 'LineWidth',2, ...
    'MarkerEdgeColor',[1 0.5 0], 'MarkerFaceColor','w');

% ✅ idx_trig: 초록색 별표로 표시
for k = 1:length(idx_trig)
    xPos = data_H10.t(idx_trig(k));
    yPos = data_H10.MotorActCurrent_LH(idx_trig(k))*1.59;
    plot(xPos, yPos, 'g*', 'MarkerSize',14, 'LineWidth',2);
end
% yl3 = ylabel('motor current [A]', 'FontSize',fontsize);

% ==== 전체 글꼴 설정 ====
set(findall(gcf,'-property','FontName'),'FontName','Times New Roman');
set(findall(gcf,'-property','FontSize'),'FontSize',fontsize);

% ==== Y축 라벨 정렬 ====
pos1 = get(yl1,'Position');
pos2 = get(yl2,'Position');
pos3 = get(yl3,'Position');

commonX = min([pos1(1), pos2(1), pos3(1)]);
set(yl1,'Position',[commonX pos1(2) pos1(3)]);
set(yl2,'Position',[commonX pos2(2) pos2(3)]);
set(yl3,'Position',[commonX pos3(2) pos3(3)]);

%% 논문용 figure (모두 동일 길이 보장)
fontsize  = 30;

% ---- 기본 구간 ----
startIdx  = 1600;
reqEndIdx = 15000;

% ---- 공통 구간의 실제 끝 인덱스 산출 (모든 시리즈 중 최소 길이) ----
seriesLens = [numel(data_H10.t), ...
              numel(data_H10.thighDeg_RH), numel(data_H10.thighDeg_LH), ...
              numel(data_H10.free_var2), ...
              numel(data_H10.MotorActCurrent_RH), numel(data_H10.MotorActCurrent_LH)];

endIdx = min([reqEndIdx, seriesLens]);

% 안전장치
if startIdx >= endIdx
    error('startIdx(%d)와 endIdx(%d)가 올바르지 않습니다.', startIdx, endIdx);
end

% ---- 잘라 쓰기 (시간 0 보정 포함) ----
t_cut     = data_H10.t(startIdx:endIdx) - data_H10.t(startIdx);
thighRH   = data_H10.thighDeg_RH(startIdx:endIdx);
thighLH   = data_H10.thighDeg_LH(startIdx:endIdx);
freeVar2  = data_H10.free_var2(startIdx:endIdx);
motorRH   = data_H10.MotorActCurrent_RH(startIdx:endIdx) * 1.59;
motorLH   = data_H10.MotorActCurrent_LH(startIdx:endIdx) * 1.59;

% ---- 이벤트 인덱스도 동일 구간으로 재정의 ----
idx_hc_cut   = idx_hc(idx_hc>=startIdx & idx_hc<=endIdx) - startIdx + 1;
idx_trig_cut = idx_trig(idx_trig>=startIdx & idx_trig<=endIdx) - startIdx + 1;

figure;

% ==== Subplot 1 ====
subplot(3,1,1);
plot(t_cut, thighRH,'r','LineWidth',2); hold on;
plot(t_cut, thighLH,'b','LineWidth',2);
xlim([0 13.5]); ylim([-10 60]);

if ~isempty(idx_hc_cut)
    plot(t_cut(idx_hc_cut), thighLH(idx_hc_cut), 'o', ...
        'MarkerSize',12, 'LineWidth',2, ...
        'MarkerEdgeColor',[1 0.5 0], 'MarkerFaceColor','w');
end
if ~isempty(idx_trig_cut)
    plot(t_cut(idx_trig_cut), thighLH(idx_trig_cut), 'g*', ...
        'MarkerSize',14, 'LineWidth',2);
end
xticklabels([]); xlabel('');
% yl1 = ylabel('Thigh angle [^\circ]', 'FontSize',fontsize);

% ==== Subplot 2 ====
subplot(3,1,2);
plot(t_cut, freeVar2,'k','LineWidth',2); hold on;
xlim([0 13.5]); ylim([0 2.5]);

if ~isempty(idx_hc_cut)
    plot(t_cut(idx_hc_cut), freeVar2(idx_hc_cut), 'o', ...
        'MarkerSize',12, 'LineWidth',2, ...
        'MarkerEdgeColor',[1 0.5 0], 'MarkerFaceColor','w');
end
xticklabels([]); xlabel('');
% yl2 = ylabel('gait mode', 'FontSize',fontsize);

% ==== Subplot 3 ====
subplot(3,1,3);
plot(t_cut, motorRH,'r','LineWidth',2); hold on;
plot(t_cut, motorLH,'b','LineWidth',2);
xlim([0 13.5]); ylim([0 2.5]);

if ~isempty(idx_hc_cut)
    plot(t_cut(idx_hc_cut), motorLH(idx_hc_cut), 'o', ...
        'MarkerSize',12, 'LineWidth',2, ...
        'MarkerEdgeColor',[1 0.5 0], 'MarkerFaceColor','w');
end
if ~isempty(idx_trig_cut)
    plot(t_cut(idx_trig_cut), motorLH(idx_trig_cut), 'g*', ...
        'MarkerSize',14, 'LineWidth',2);
end
% yl3 = ylabel('motor current [A]', 'FontSize',fontsize);

% ==== 전체 글꼴 설정 ====
set(findall(gcf,'-property','FontName'),'FontName','Times New Roman');
set(findall(gcf,'-property','FontSize'),'FontSize',fontsize);

% ==== Y축 라벨 정렬 ====
pos1 = get(yl1,'Position'); pos2 = get(yl2,'Position'); pos3 = get(yl3,'Position');
commonX = min([pos1(1), pos2(1), pos3(1)]);
set(yl1,'Position',[commonX pos1(2) pos1(3)]);
set(yl2,'Position',[commonX pos2(2) pos2(3)]);
set(yl3,'Position',[commonX pos3(2) pos3(3)]);


%% Standing / moving 판단
fs = 1000; % 실제 샘플레이트로 설정
res = detectStandingMoving_LPF(data_H10);
flag = res.movingFlag;   % 0=standing, 1=moving (샘플별)

% (선택) 확인용 간단 플롯
t = (0:numel(flag)-1)/fs;
figure;
subplot(2,1,1);
plot(t, res.w_R_raw,'k'); hold on;
plot(t, res.w_R_f,'r'); hold on;

subplot(2,1,2);
plot(t, res.v_metric,'k'); hold on;

%% Simulation
figure;
subplot(5,1,1);
plot(data_H10.t, data_H10.thighDeg_RH,'r','LineWidth',3); hold on;
plot(data_H10.t, data_H10.thighDeg_LH,'b','LineWidth',3); hold on; grid on;
plot(data_H10.t, flag*80,'k','LineWidth',3);
legend('R Thigh', 'L Thigh');
subplot(5,1,2);
plot(data_H10.t, data_H10.free_var2,'k','LineWidth',3); hold on; grid on;
subplot(5,1,3);
plot(data_H10.t, data_H10.free_var1*0.1,'k','LineWidth',3); hold on; grid on;
plot(data_H10.t, data_H10.MotorActCurrent_RH,'r','LineWidth',3); hold on;
plot(data_H10.t, data_H10.MotorActCurrent_LH,'b','LineWidth',3); grid on;
legend('R Motor Current', 'L Motor Current');
subplot(5,1,4);
plot(t, res.v_metric,'r'); hold on;
plot(data_H10.t, res.movingFlag*80,'k','LineWidth',3);

%% functions

function result = detectStandingMoving_LPF(data_H10)
    % ==== 고정 파라미터 ====
    fs = 1000;
    alpha_theta = 0.0591174;   % 10 Hz (theta)
    alpha_omega = 0.1357552;   % 25 Hz (omega)

    % ==== 상태머신 파라미터 ====
    V_ON_THRESH    = 40.0;
    V_OFF_THRESH   = 15.0;
    ON_HOLD_TICKS  = 10;
    OFF_HOLD_TICKS = 50;

    % ==== 입력 ====
    theta_R = data_H10.thighDeg_RH(:);
    theta_L = data_H10.thighDeg_LH(:);
    N = min(numel(theta_R), numel(theta_L));
    theta_R = theta_R(1:N);
    theta_L = theta_L(1:N);

    % ==== 버퍼 ====
    theta_R_f = zeros(N,1); theta_L_f = zeros(N,1);
    w_R_raw   = zeros(N,1); w_L_raw   = zeros(N,1);
    w_R_f     = zeros(N,1); w_L_f     = zeros(N,1);
    v_metric  = zeros(N,1);
    movingFlag= zeros(N,1,'uint8');

    % ==== LPF 초기조건을 입력 첫 값으로 설정 (핵심 수정) ====
    y_prev_R = theta_R(1);
    y_prev_L = theta_L(1);
    theta_R_f(1) = y_prev_R;
    theta_L_f(1) = y_prev_L;

    % theta LPF
    for n = 2:N
        y_prev_R = (1 - alpha_theta)*y_prev_R + alpha_theta*theta_R(n);
        y_prev_L = (1 - alpha_theta)*y_prev_L + alpha_theta*theta_L(n);
        theta_R_f(n) = y_prev_R;
        theta_L_f(n) = y_prev_L;
    end

    % 미분 (deg/s)
    w_R_raw(1) = 0; w_L_raw(1) = 0;
    for n = 2:N
        w_R_raw(n) = fs*(theta_R_f(n) - theta_R_f(n-1));
        w_L_raw(n) = fs*(theta_L_f(n) - theta_L_f(n-1));
    end

    % omega LPF도 동일하게 첫 값으로 초기화
    w_prev_R = w_R_raw(1);  w_prev_L = w_L_raw(1);
    w_R_f(1) = w_prev_R;    w_L_f(1) = w_prev_L;
    for n = 2:N
        w_prev_R = (1 - alpha_omega)*w_prev_R + alpha_omega*w_R_raw(n);
        w_prev_L = (1 - alpha_omega)*w_prev_L + alpha_omega*w_L_raw(n);
        w_R_f(n) = w_prev_R; w_L_f(n) = w_prev_L;
    end

    % 상태머신
    GAIT_STANDING = 0; GAIT_MOVING = 1;
    s_state = GAIT_STANDING; s_on_cnt = 0; s_off_cnt = 0;

    for n = 1:N
        v = max(abs(w_R_f(n)), abs(w_L_f(n)));
        v_metric(n) = v;

        evidence_move  = (v >= V_ON_THRESH);
        evidence_stand = (v <= V_OFF_THRESH);

        if s_state == GAIT_STANDING
            if evidence_move
                s_on_cnt = s_on_cnt + 1;
                if s_on_cnt >= ON_HOLD_TICKS
                    s_state = GAIT_MOVING; s_on_cnt = 0; s_off_cnt = 0;
                end
            else
                s_on_cnt = 0;
            end
        else
            if evidence_stand
                s_off_cnt = s_off_cnt + 1;
                if s_off_cnt >= OFF_HOLD_TICKS
                    s_state = GAIT_STANDING; s_on_cnt = 0; s_off_cnt = 0;
                end
            else
                s_off_cnt = 0;
            end
        end

        movingFlag(n) = uint8(s_state == GAIT_MOVING);
    end

    % 결과
    result.theta_R_f  = theta_R_f;
    result.theta_L_f  = theta_L_f;
    result.w_R_raw    = w_R_raw;
    result.w_L_raw    = w_L_raw;
    result.w_R_f      = w_R_f;
    result.w_L_f      = w_L_f;
    result.v_metric   = v_metric;
    result.movingFlag = movingFlag;
end

% --- C의 static LPF를 그대로 옮긴 스칼라 버전 ---
function y_next = LPF_scalar(x, y_prev, alpha)
    y_next = (1 - alpha) * y_prev + alpha * x;
end
