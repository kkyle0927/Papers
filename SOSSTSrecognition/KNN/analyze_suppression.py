import csv

def load_csv(path):
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        rows = []
        for r in reader:
            d = {}
            for k, v in r.items():
                try:
                    d[k] = float(v)
                except:
                    d[k] = v
            rows.append(d)
        return rows

yes = load_csv('yesKNN7.csv')
no  = load_csv('noKNN3.csv')

mode_names = {0:'NONE', 1:'RSTS', 2:'RSOS', 3:'LSTS', 4:'LSOS'}
STS_MODES = {1, 3}  # RSTS, LSTS

def extract_hc_events(rows):
    """HC 이벤트 목록 반환: (row_index, hc_num, gait_mode, t_ms)"""
    events = []
    prev_hc = 0
    for i, r in enumerate(rows):
        hc = int(r['hc_count'])
        if hc != prev_hc:
            events.append({
                'idx': i,
                'hc_num': hc,
                'gait_mode': int(r['s_gait_mode']),
                't_ms': r['LoopCnt'] * 2,
                'L_tor': r['LeftHipTorque'],
                'R_tor': r['RightHipTorque'],
            })
        prev_hc = hc
    return events

def extract_upeak_events(rows):
    """Upper peak 이벤트: (row_index, side, t_ms)"""
    events = []
    prev_ru = 0
    prev_lu = 0
    for i, r in enumerate(rows):
        ru = int(r['R_count_upeak'])
        lu = int(r['L_count_upeak'])
        if ru != prev_ru:
            events.append({'idx': i, 'side': 'R', 't_ms': r['LoopCnt'] * 2,
                           'L_tor': r['LeftHipTorque'], 'R_tor': r['RightHipTorque']})
        if lu != prev_lu:
            events.append({'idx': i, 'side': 'L', 't_ms': r['LoopCnt'] * 2,
                           'L_tor': r['LeftHipTorque'], 'R_tor': r['RightHipTorque']})
        prev_ru = ru
        prev_lu = lu
    return events

def torque_at_time(rows, idx, side):
    return rows[idx]['RightHipTorque'] if side == 'R' else rows[idx]['LeftHipTorque']

def find_torque_drop_time(rows, start_idx, side, threshold=0.1):
    """start_idx 이후 torque가 threshold 이하로 내려간 첫 번째 시각(ms) 반환"""
    for i in range(start_idx, len(rows)):
        tor = torque_at_time(rows, i, side)
        if tor <= threshold:
            return rows[i]['LoopCnt'] * 2, i
    return None, None

def analyze_sts_suppression(rows, label, dt_ms=2.0):
    print(f'\n{"="*60}')
    print(f' {label}: STS 억제 원인 분석 (HC vs Upper Peak)')
    print(f'{"="*60}')

    hc_events  = extract_hc_events(rows)
    up_events  = extract_upeak_events(rows)

    sts_hc = [e for e in hc_events if e['gait_mode'] in STS_MODES]

    for hc in sts_hc:
        side = 'R' if hc['gait_mode'] == 1 else 'L'  # RSTS=1, LSTS=3
        t_hc = hc['t_ms']
        tor_at_hc = hc['R_tor'] if side == 'R' else hc['L_tor']

        # 해당 다리의 다음 upper peak 찾기 (HC 이후)
        next_up = None
        for up in up_events:
            if up['side'] == side and up['t_ms'] > t_hc:
                next_up = up
                break

        if next_up is None:
            print(f'  HC#{hc["hc_num"]} {mode_names[hc["gait_mode"]]} @ {t_hc:.0f}ms: upper peak 없음')
            continue

        t_up = next_up['t_ms']
        delta_t = t_up - t_hc
        tor_at_up_prev = torque_at_time(rows, next_up['idx'] - 1, side) if next_up['idx'] > 0 else 0
        tor_at_up = next_up['R_tor'] if side == 'R' else next_up['L_tor']

        # torque가 0.1 Nm 이하로 내려간 시각
        drop_t, drop_idx = find_torque_drop_time(rows, hc['idx'], side, threshold=0.1)

        print(f'\n  HC#{hc["hc_num"]} {mode_names[hc["gait_mode"]]} @ t={t_hc:.0f}ms  →  Upper Peak @ t={t_up:.0f}ms  (Δt={delta_t:.0f}ms)')
        print(f'    {side}Tor @ HC       : {tor_at_hc:+.3f} Nm')

        # 구간 내 torque 궤적 (HC부터 upper peak까지, 10 tick 간격)
        print(f'    {side}Tor 궤적 (HC~UP):')
        start_i = hc['idx']
        end_i   = next_up['idx']
        step = max(1, (end_i - start_i) // 8)
        for i in range(start_i, end_i + 1, step):
            t_i  = rows[i]['LoopCnt'] * 2
            tor  = torque_at_time(rows, i, side)
            mark = ' <-- HC' if i == start_i else (' <-- UP' if i == end_i else '')
            print(f'      t={t_i:.0f}ms: {tor:+.3f} Nm{mark}')

        if drop_t is not None:
            drop_relative = drop_t - t_hc
            if drop_t < t_up:
                verdict = f'★ HC 이후 {drop_relative:.0f}ms 만에 0으로 → STS 억제가 원인'
            else:
                verdict = f'  Upper Peak 이후 {drop_t - t_up:.0f}ms 후 0으로 → Upper Peak가 원인'
            print(f'    0.1Nm 이하 도달: t={drop_t:.0f}ms  (+{drop_relative:.0f}ms after HC)')
            print(f'    판정: {verdict}')
        else:
            print(f'    0.1Nm 이하 도달: 없음 (구간 내 미달)')

analyze_sts_suppression(yes, 'yesKNN7')
analyze_sts_suppression(no,  'noKNN3')

# 추가: 두 파일의 동일 HC 번호 대비 비교
print(f'\n{"="*60}')
print(' yesKNN7 vs noKNN3 : STS HC별 torque drop 타이밍 비교')
print(f'{"="*60}')

def get_sts_summary(rows):
    hc_events = extract_hc_events(rows)
    up_events = extract_upeak_events(rows)
    summary = {}
    for hc in hc_events:
        if hc['gait_mode'] not in STS_MODES:
            continue
        side = 'R' if hc['gait_mode'] == 1 else 'L'
        t_hc = hc['t_ms']
        next_up = next((u for u in up_events if u['side'] == side and u['t_ms'] > t_hc), None)
        drop_t, _ = find_torque_drop_time(rows, hc['idx'], side, threshold=0.1)
        summary[hc['hc_num']] = {
            'mode': mode_names[hc['gait_mode']],
            'side': side,
            't_hc': t_hc,
            't_up': next_up['t_ms'] if next_up else None,
            'tor_hc': hc['R_tor'] if side == 'R' else hc['L_tor'],
            'drop_t': drop_t,
        }
    return summary

yes_sum = get_sts_summary(yes)
no_sum  = get_sts_summary(no)

all_hc = sorted(set(list(yes_sum.keys()) + list(no_sum.keys())))
print(f'  {"HC":>4} {"mode":>5} | {"yesKNN7":^38} | {"noKNN3":^38}')
print(f'  {"":>4} {"":>5} | {"HC→0.1Nm(ms)":>14} {"HC→UP(ms)":>12} {"원인":>10} | {"HC→0.1Nm(ms)":>14} {"HC→UP(ms)":>12} {"원인":>10}')
print(f'  {"-"*4} {"-"*5}-+-{"-"*38}-+-{"-"*38}')
for n in all_hc:
    y = yes_sum.get(n)
    no = no_sum.get(n)
    def fmt(s):
        if s is None: return f'{"N/A":>14} {"N/A":>12} {"N/A":>10}'
        dt_drop = f"{s['drop_t'] - s['t_hc']:.0f}" if s['drop_t'] else 'none'
        dt_up   = f"{s['t_up'] - s['t_hc']:.0f}" if s['t_up'] else 'none'
        cause   = 'STS★' if (s['drop_t'] and s['t_up'] and s['drop_t'] < s['t_up']) else 'UPeak'
        return f'{dt_drop:>14} {dt_up:>12} {cause:>10}'
    mode_str = (y or no)['mode'] if (y or no) else '???'
    print(f'  {n:>4} {mode_str:>5} | {fmt(y)} | {fmt(no)}')
