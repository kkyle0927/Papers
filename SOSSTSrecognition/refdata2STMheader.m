clear all;
close all;
clc;

%%
addpath('C:\Users\ChanyoungKo\OneDrive - 한국과학기술원 기계공학과\바탕 화면\250829_refdataset_exp');
matFile  = 'refdataset_merged_scaled.mat';  % 새 .mat 파일 경로
outHeader= 'KNN_ref_dataset.h';
write_ref_header(matFile, outHeader);

%% function 
function write_ref_header(matFile, outHeader)
% write_ref_header(matFile, outHeader)
%   matFile   : .mat 경로 (첨부 형식 또는 기존 형식)
%   outHeader : 생성할 헤더 파일 이름(예: 'ref_dataset.h')
%
% 변경 사항:
% - 첨부 형식 지원: 'ref_all_scaled' (N×3), 선택적으로 'scale_factor'([1×2])
% - 하위 호환: 'ref_clean' (N×3)도 자동 인식
% - 3열은 라벨로 가정(1->1, 그 외->2), label==1이 먼저 오도록 정렬
%
% 출력 포맷(동일):
%   #pragma once
%   #include <stdint.h>
%   #define REF_COUNT ...
%   #define REF_TYPE1_COUNT ...
%   #define REF_TYPE2_COUNT (REF_COUNT - REF_TYPE1_COUNT)
%   static inline uint8_t ref_label(unsigned idx) { return idx < REF_TYPE1_COUNT ? 1u : 2u; }
%   static const float ref_xy[REF_COUNT][2] = { {xf, yf}, ... };

    % --- 로드 및 변수 감지 ---
    if ~exist(matFile, 'file')
        error('파일을 찾을 수 없습니다: %s', matFile);
    end

    vars = whos('-file', matFile);
    has_ref_all_scaled = any(strcmp({vars.name}, 'ref_all_scaled'));
    has_ref_clean      = any(strcmp({vars.name}, 'ref_clean'));

    if has_ref_all_scaled
        S = load(matFile, 'ref_all_scaled');
        A = S.ref_all_scaled;
    elseif has_ref_clean
        S = load(matFile, 'ref_clean');
        A = S.ref_clean;
    else
        error('mat 파일에 ref_all_scaled 또는 ref_clean 변수가 없습니다.');
    end

    if ~isnumeric(A) || size(A,2) < 3
        error('참조 데이터는 (N×3) numeric 배열이어야 합니다. (x, y, label)');
    end

    % --- x/y/label 분리 및 검증 ---
    x = A(:,1);
    y = A(:,2);
    lab_raw = A(:,3);

    if any(~isfinite(x)) || any(~isfinite(y))
        error('x/y에 유효하지 않은 값(NaN/Inf)이 있습니다.');
    end

    % --- 라벨 1/2로 정규화 & 정렬(1 먼저) ---
    lab = ones(size(lab_raw));      % 기본 1
    lab(lab_raw ~= 1) = 2;          % 1이 아니면 2로
    idx1 = find(lab == 1);
    idx2 = find(lab == 2);
    XY   = [x(idx1) y(idx1); x(idx2) y(idx2)];

    REF_COUNT       = size(XY,1);
    REF_TYPE1_COUNT = numel(idx1);
    %REF_TYPE2_COUNT = REF_COUNT - REF_TYPE1_COUNT; % 매크로에서 계산

    % --- 헤더 파일 작성 ---
    if nargin < 2 || isempty(outHeader)
        outHeader = 'ref_dataset.h';
    end
    fid = fopen(outHeader, 'w');
    if fid < 0
        error('헤더 파일을 생성할 수 없습니다: %s', outHeader);
    end
    cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>

    fprintf(fid, "#pragma once\n");
    fprintf(fid, "#include <stdint.h>\n\n");
    fprintf(fid, "#define REF_COUNT %uu\n", REF_COUNT);
    fprintf(fid, "#define REF_TYPE1_COUNT %uu\n", REF_TYPE1_COUNT);
    fprintf(fid, "#define REF_TYPE2_COUNT (REF_COUNT - REF_TYPE1_COUNT)\n\n");
    fprintf(fid, "static inline uint8_t ref_label(unsigned idx) { return idx < REF_TYPE1_COUNT ? 1u : 2u; }\n\n");
    fprintf(fid, "static const float ref_xy[REF_COUNT][2] = {\n");

    % 숫자 출력: 고정소수 9자리
    for i = 1:REF_COUNT
        fprintf(fid, "    { %.9ff, %.9ff }", XY(i,1), XY(i,2));
        if i < REF_COUNT, fprintf(fid, ",\n"); else, fprintf(fid, "\n"); end
    end
    fprintf(fid, "};\n");

    fprintf(1, '헤더 생성 완료: %s\n', outHeader);
    fprintf(1, 'REF_COUNT=%d, REF_TYPE1_COUNT=%d, REF_TYPE2_COUNT=%d\n', ...
            REF_COUNT, REF_TYPE1_COUNT, REF_COUNT-REF_TYPE1_COUNT);
end
