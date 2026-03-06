"""
HC 감지 순간을 기준으로 yesKNN vs noKNN3 토크 감쇄 패턴 비교.
핵심 질문: STS HC 이후 토크가 더 빠르게 꺾이는가? (KNN 억제 효과)
"""
import csv
import math
from collections import defaultdict

def load_csv(path):
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        rows = []
        for r in reader:
            d = {}
            for k, v in r.items():
                try:    d[k] = float(v)
                except: d[k] = v
            rows.append(d)
        return rows

mode_names = {0:'NONE', 1:'RSTS', 2:'RSOS', 3:'LSTS', 4:'LSOS'}
STS_MODES  = {1, 3}
DT_MS      = 2.0

def extract_hc_events(rows):
    events, prev = [], 0
    for i, r in enumerate(rows):
        hc = int(r['hc_count'])
        if hc != prev:
            events.append({'idx': i, 'hc_num': hc,
                           'gait_mode': int(r['s_gait_mode']),
                           't_ms': r['LoopCnt'] * 2,
                           'L_tor': r['LeftHipTorque'],
                           'R_tor': r['RightHipTorque'],
                           'conf': r['s_g_knn_conf'],
                           'norm_vel': r['s_norm_vel_HC'],
                           'norm_T':   r['s_norm_T_HC'],
                           'vel_HC':   r['s_vel_HC'],
                           'T_HC_s':   r['s_T_HC_s'],
                           'Rdeg': r['RightThighAngle'],
                           'Ldeg': r['LeftThighAngle'],})
        prev = hc
    return events

def extract_upeak_events(rows):
    events, pr, pl = [], 0, 0
    for i, r in enumerate(rows):
        ru, lu = int(r['R_count_upeak']), int(r['L_count_upeak'])
        if ru != pr:
            events.append({'idx': i, 'side': 'R', 't_ms': r['LoopCnt'] * 2})
        if lu != pl:
            events.append({'idx': i, 'side': 'L', 't_ms': r['LoopCnt'] * 2})
        pr, pl = ru, lu
    return events

def get_tor(rows, idx, side):
    return rows[idx]['RightHipTorque'] if side == 'R' else rows[idx]['LeftHipTorque']

def decay_rate(tor_start, tor_end, dt_ms):
    """단순 선형 감쇄율 [Nm/s]"""
    if dt_ms <= 0: return 0.0
    return (tor_start - tor_end) / (dt_ms / 1000.0)

def half_life_ms(rows, start_idx, side):
    """HC 시점 토크의 절반이 될 때까지 걸린 시간 (ms)"""
    t0 = get_tor(rows, start_idx, side)
    if t0 <= 0: return None
    half = t0 / 2.0
    for i in range(start_idx, len(rows)):
        if get_tor(rows, i, side) <= half:
            return (rows[i]['LoopCnt'] - rows[start_idx]['LoopCnt']) * DT_MS
    return None

# ──────────────────────────────────────────────────────────────
# 1. noKNN3 STS 이벤트 기준값 먼저 추출
# ──────────────────────────────────────────────────────────────
no_rows    = load_csv('noKNN3.csv')
no_hc_ev   = extract_hc_events(no_rows)
no_up_ev   = extract_upeak_events(no_rows)
no_sts_hc  = [e for e in no_hc_ev if e['gait_mode'] in STS_MODES]

no_ref = {}   # hc_num -> {tor_hc, decay_rate, half_life, tor_at_up, ...}
for hc in no_sts_hc:
    side   = 'R' if hc['gait_mode'] == 1 else 'L'
    t_hc   = hc['t_ms']
    tor_hc = hc['R_tor'] if side == 'R' else hc['L_tor']
    next_up = next((u for u in no_up_ev if u['side'] == side and u['t_ms'] > t_hc), None)
    if next_up is None: continue
    t_up   = next_up['t_ms']
    dt_up  = t_up - t_hc
    tor_up = get_tor(no_rows, next_up['idx'], side)
    hl     = half_life_ms(no_rows, hc['idx'], side)
    dr     = decay_rate(tor_hc, tor_up, dt_up)

    # 구간 내 토크 배열 (HC~UP, 최대 20 샘플)
    span = next_up['idx'] - hc['idx'] + 1
    step = max(1, span // 10)
    traj = [(rows_i := hc['idx'] + j,
             (no_rows[rows_i]['LoopCnt'] * 2 - t_hc),
             get_tor(no_rows, rows_i, side))
            for j in range(0, span, step)]

    no_ref[hc['hc_num']] = {
        'mode': mode_names[hc['gait_mode']], 'side': side,
        'tor_hc': tor_hc, 'tor_up': tor_up,
        'dt_up': dt_up, 'decay_rate': dr, 'half_life': hl, 'traj': traj,
        'norm_vel': hc['norm_vel'], 'norm_T': hc['norm_T'], 'conf': hc['conf'],
    }

# ──────────────────────────────────────────────────────────────
# 2. yesKNN1~7 각 파일 분석 + noKNN3 대조
# ──────────────────────────────────────────────────────────────
files = [f'yesKNN{i}.csv' for i in range(1, 8)]

print('=' * 90)
print(' HC 감지 직후 토크 감쇄율 비교: yesKNN vs noKNN3')
print(' (같은 HC#의 noKNN3 값이 없으면 N/A)')
print('=' * 90)

all_records = []   # 전체 집계용

for fname in files:
    rows   = load_csv(fname)
    hc_ev  = extract_hc_events(rows)
    up_ev  = extract_upeak_events(rows)
    sts_hc = [e for e in hc_ev if e['gait_mode'] in STS_MODES]

    print(f'\n{"─"*90}')
    print(f' {fname}')
    print(f'{"─"*90}')
    print(f'  {"HC#":>4} {"mode":>5} {"side":>4} | '
          f'{"tor@HC":>7} {"tor@UP":>7} {"Δt_UP":>7} | '
          f'{"감쇄율(Nm/s)":>12} {"반감기(ms)":>11} | '
          f'{"noKNN감쇄율":>12} {"noKNN반감기":>12} | '
          f'{"감쇄율 비":>9} {"판정":>10}')
    print('  ' + '-' * 88)

    for hc in sts_hc:
        side   = 'R' if hc['gait_mode'] == 1 else 'L'
        t_hc   = hc['t_ms']
        tor_hc = hc['R_tor'] if side == 'R' else hc['L_tor']
        next_up = next((u for u in up_ev if u['side'] == side and u['t_ms'] > t_hc), None)
        if next_up is None: continue

        t_up   = next_up['t_ms']
        dt_up  = t_up - t_hc
        tor_up = get_tor(rows, next_up['idx'], side)
        hl     = half_life_ms(rows, hc['idx'], side)
        dr     = decay_rate(tor_hc, tor_up, dt_up)

        # noKNN3 대조값
        ref = no_ref.get(hc['hc_num'])
        if ref and ref['side'] == side:
            dr_no = ref['decay_rate']
            hl_no = ref['half_life']
            ratio = dr / dr_no if dr_no > 0 else float('inf')
            # 판정: yes가 10%이상 빠르면 STS 억제 효과 있음
            verdict = '★빠름(STS억제?)' if ratio > 1.10 else ('○비슷(자연감쇄)' if ratio > 0.90 else '▽느림')
        else:
            dr_no, hl_no, ratio, verdict = None, None, None, 'N/A'

        hl_s    = f'{hl:.0f}' if hl else 'none'
        hl_no_s = f'{hl_no:.0f}' if hl_no else 'none'
        dr_no_s = f'{dr_no:.2f}' if dr_no else 'N/A'
        ratio_s = f'{ratio:.2f}' if ratio else 'N/A'

        print(f'  {hc["hc_num"]:>4} {mode_names[hc["gait_mode"]]:>5} {side:>4} | '
              f'{tor_hc:7.3f} {tor_up:7.3f} {dt_up:7.0f} | '
              f'{dr:12.2f} {hl_s:>11} | '
              f'{dr_no_s:>12} {hl_no_s:>12} | '
              f'{ratio_s:>9} {verdict:>10}')

        all_records.append({'file': fname, 'hc_num': hc['hc_num'],
                            'mode': mode_names[hc['gait_mode']], 'side': side,
                            'tor_hc': tor_hc, 'dt_up': dt_up,
                            'dr_yes': dr, 'dr_no': dr_no,
                            'hl_yes': hl, 'hl_no': hl_no,
                            'ratio': ratio, 'verdict': verdict})

# ──────────────────────────────────────────────────────────────
# 3. 전체 집계
# ──────────────────────────────────────────────────────────────
print(f'\n{"="*90}')
print(' 전체 집계 (noKNN3 대조 가능한 이벤트)')
print(f'{"="*90}')

comparable = [r for r in all_records if r['ratio'] is not None]
faster = [r for r in comparable if r['verdict'].startswith('★')]
similar = [r for r in comparable if r['verdict'].startswith('○')]
slower = [r for r in comparable if r['verdict'].startswith('▽')]

print(f'  총 비교 가능 STS HC: {len(comparable)}건')
print(f'    ★ 빠름(STS 억제 가능): {len(faster)}건  ({len(faster)/len(comparable)*100:.0f}%)')
print(f'    ○ 비슷(자연 감쇄):     {len(similar)}건  ({len(similar)/len(comparable)*100:.0f}%)')
print(f'    ▽ 느림:                {len(slower)}건  ({len(slower)/len(comparable)*100:.0f}%)')

if comparable:
    ratios = [r['ratio'] for r in comparable]
    print(f'\n  감쇄율 비(yes/no) 통계: min={min(ratios):.3f}, max={max(ratios):.3f}, '
          f'mean={sum(ratios)/len(ratios):.3f}')

# HC#별 패턴
print(f'\n  HC#별 평균 감쇄율 비:')
hc_groups = defaultdict(list)
for r in comparable:
    hc_groups[r['hc_num']].append(r['ratio'])
for hc_num in sorted(hc_groups):
    vals = hc_groups[hc_num]
    mn = sum(vals)/len(vals)
    verdict_tag = '★ 빠름' if mn > 1.10 else '○ 비슷'
    print(f'    HC#{hc_num}: n={len(vals)}, mean_ratio={mn:.3f}  {verdict_tag}')

# ──────────────────────────────────────────────────────────────
# 4. 대표 HC에 대한 토크 궤적 상세 비교 (yesKNN7 vs noKNN3 HC#1,2,3)
# ──────────────────────────────────────────────────────────────
print(f'\n{"="*90}')
print(' 대표 STS HC 토크 궤적 상세 (yesKNN7 vs noKNN3)')
print('="*90')

yes7_rows  = load_csv('yesKNN7.csv')
yes7_hc_ev = extract_hc_events(yes7_rows)
yes7_up_ev = extract_upeak_events(yes7_rows)
yes7_sts   = [e for e in yes7_hc_ev if e['gait_mode'] in STS_MODES]

for hc_y in yes7_sts:
    side  = 'R' if hc_y['gait_mode'] == 1 else 'L'
    t_hc  = hc_y['t_ms']
    next_up = next((u for u in yes7_up_ev if u['side'] == side and u['t_ms'] > t_hc), None)
    hc_n = no_ref.get(hc_y['hc_num'])
    if not next_up or not hc_n or hc_n['side'] != side:
        continue

    no_t_ms_str = '?'
    for e in no_sts_hc:
        if e['hc_num'] == hc_y['hc_num']:
            no_t_ms_str = f'{e["t_ms"]:.0f}'
            break
    print(f'\n  HC#{hc_y["hc_num"]} {mode_names[hc_y["gait_mode"]]}  '
          f'(yesKNN7 @ {t_hc:.0f}ms  vs  noKNN3 @ {no_t_ms_str}ms)')
    print(f'  {"Δt(ms)":>8} | {"yesKNN7 tor":>12} {"noKNN3 tor":>12} | {"차이":>8}')
    print(f'  {"-"*8}-+-{"-"*12}-{"-"*12}-+-{"-"*8}')

    # yesKNN7 궤적
    y_start = hc_y['idx']
    y_end   = next_up['idx']

    # noKNN3 궤적 (같은 HC#)
    no_hc_match = next((e for e in no_sts_hc if e['hc_num'] == hc_y['hc_num']), None)
    if not no_hc_match: continue
    no_up_match = next((u for u in no_up_ev if u['side'] == side and u['t_ms'] > no_hc_match['t_ms']), None)
    if not no_up_match: continue

    n_start = no_hc_match['idx']
    span_y  = y_end - y_start + 1
    span_n  = no_up_match['idx'] - n_start + 1
    span    = max(span_y, span_n)
    step    = max(1, span // 12)

    for j in range(0, span, step):
        yi   = y_start + j
        ni   = n_start + j
        dt_y = j * DT_MS
        tor_y = get_tor(yes7_rows, yi, side) if yi < len(yes7_rows) else None
        tor_n = get_tor(no_rows, ni, side)  if ni < len(no_rows)  else None
        diff  = tor_y - tor_n if (tor_y is not None and tor_n is not None) else None

        mark_y = ' <HC' if j == 0 else (' <UP' if yi >= y_end else '')
        tor_y_s = f'{tor_y:.3f}{mark_y}' if tor_y is not None else '  ---'
        tor_n_s = f'{tor_n:.3f}' if tor_n is not None else '  ---'
        diff_s  = f'{diff:+.3f}' if diff is not None else '  ---'
        print(f'  {dt_y:8.0f} | {tor_y_s:>12} {tor_n_s:>12} | {diff_s:>8}')
