import csv
from collections import defaultdict

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

mode_names = {0:'NONE', 1:'RSTS', 2:'RSOS', 3:'LSTS', 4:'LSOS'}
STS_MODES = {1, 3}

def extract_hc_events(rows):
    events = []
    prev_hc = 0
    for i, r in enumerate(rows):
        hc = int(r['hc_count'])
        if hc != prev_hc:
            events.append({
                'idx': i, 'hc_num': hc,
                'gait_mode': int(r['s_gait_mode']),
                't_ms': r['LoopCnt'] * 2,
                'L_tor': r['LeftHipTorque'],
                'R_tor': r['RightHipTorque'],
                'conf': r['s_g_knn_conf'],
                'norm_vel': r['s_norm_vel_HC'],
                'norm_T': r['s_norm_T_HC'],
            })
        prev_hc = hc
    return events

def extract_upeak_events(rows):
    events = []
    prev_ru, prev_lu = 0, 0
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

def torque_at(rows, idx, side):
    return rows[idx]['RightHipTorque'] if side == 'R' else rows[idx]['LeftHipTorque']

def find_drop_time(rows, start_idx, side, threshold=0.1):
    for i in range(start_idx, len(rows)):
        if torque_at(rows, i, side) <= threshold:
            return rows[i]['LoopCnt'] * 2, i
    return None, None

def get_sts_summary(rows):
    hc_events = extract_hc_events(rows)
    up_events = extract_upeak_events(rows)
    results = []
    for hc in hc_events:
        if hc['gait_mode'] not in STS_MODES:
            continue
        side = 'R' if hc['gait_mode'] == 1 else 'L'
        t_hc = hc['t_ms']
        tor_hc = hc['R_tor'] if side == 'R' else hc['L_tor']
        next_up = next((u for u in up_events if u['side'] == side and u['t_ms'] > t_hc), None)
        drop_t, _ = find_drop_time(rows, hc['idx'], side, threshold=0.1)
        t_up = next_up['t_ms'] if next_up else None

        if drop_t is not None and t_up is not None:
            cause = 'STS' if drop_t < t_up else 'UPeak'
        elif drop_t is not None:
            cause = 'STS(noUP)'
        else:
            cause = 'none'

        results.append({
            'hc_num': hc['hc_num'],
            'mode': mode_names[hc['gait_mode']],
            'side': side,
            't_hc': t_hc,
            'tor_hc': tor_hc,
            't_up': t_up,
            'drop_t': drop_t,
            'dt_drop': drop_t - t_hc if drop_t else None,
            'dt_up': t_up - t_hc if t_up else None,
            'cause': cause,
            'conf': hc['conf'],
            'norm_vel': hc['norm_vel'],
            'norm_T': hc['norm_T'],
        })
    return results


# ─── 파일별 기본 통계 ───────────────────────────────────────────
files = [f'yesKNN{i}.csv' for i in range(1, 8)]

print('=' * 80)
print(' 파일별 기본 통계')
print('=' * 80)
print(f'  {"파일":12} {"시간(s)":>8} {"tau":>4} {"HC":>4} {"STS":>4} {"SOS":>4} '
      f'{"conf_min":>9} {"conf_mean":>10} {"R_tor_max":>10} {"L_tor_max":>10} '
      f'{"T_swing 초→말(ms)":>20}')
print('  ' + '-' * 90)

all_sts_results = {}  # file -> list of STS HC summary dicts

for fname in files:
    rows = load_csv(fname)
    total_s = len(rows) * 2 / 1000
    tau = int(rows[0]['tau_max_setting'])
    hc_ev = extract_hc_events(rows)
    sts_n = sum(1 for e in hc_ev if e['gait_mode'] in STS_MODES)
    sos_n = sum(1 for e in hc_ev if e['gait_mode'] in {2, 4})
    confs = [e['conf'] for e in hc_ev] if hc_ev else [0]
    r_max = max(r['RightHipTorque'] for r in rows)
    l_max = max(r['LeftHipTorque'] for r in rows)
    t_sw_start = rows[0]['T_swing_ms']
    t_sw_end = rows[-1]['T_swing_ms']

    print(f'  {fname:12} {total_s:8.1f} {tau:4d} {len(hc_ev):4d} {sts_n:4d} {sos_n:4d} '
          f'{min(confs):9.3f} {sum(confs)/len(confs):10.3f} {r_max:10.3f} {l_max:10.3f} '
          f'  {t_sw_start:.0f} -> {t_sw_end:.0f}')

    all_sts_results[fname] = get_sts_summary(rows)


# ─── STS 억제 원인 판정 상세 ────────────────────────────────────
print()
print('=' * 80)
print(' STS HC별 억제 원인 판정 (yesKNN1~7 전체)')
print('  ★ = 0.1Nm 도달이 Upper Peak 이전 → STS 감지가 원인 가능')
print('  ○ = 0.1Nm 도달이 Upper Peak 이후 → Upper Peak가 원인')
print('=' * 80)
print(f'  {"파일":12} {"HC#":>4} {"mode":>5} {"tor@HC":>8} {"HC→0.1Nm":>10} '
      f'{"HC→UP":>8} {"판정":>7} {"conf":>6} {"nV":>6} {"nT":>6}')
print('  ' + '-' * 80)

total_sts = 0
cause_counter = defaultdict(int)
tau_cause = defaultdict(lambda: defaultdict(int))

for fname in files:
    tau = int(load_csv(fname)[0]['tau_max_setting'])
    results = all_sts_results[fname]
    for r in results:
        total_sts += 1
        cause_counter[r['cause']] += 1
        tau_cause[tau][r['cause']] += 1
        mark = '★' if r['cause'] == 'STS' else '○'
        dt_drop_s = f"{r['dt_drop']:.0f}" if r['dt_drop'] is not None else 'none'
        dt_up_s   = f"{r['dt_up']:.0f}"   if r['dt_up']   is not None else 'none'
        print(f'  {fname:12} {r["hc_num"]:4d} {r["mode"]:>5} {r["tor_hc"]:8.3f} '
              f'{dt_drop_s:>10} {dt_up_s:>8} {mark+" "+r["cause"]:>7} '
              f'{r["conf"]:6.3f} {r["norm_vel"]:6.3f} {r["norm_T"]:6.3f}')


# ─── 전체 집계 ───────────────────────────────────────────────────
print()
print('=' * 80)
print(' 전체 집계')
print('=' * 80)
print(f'  STS HC 이벤트 총 {total_sts}개')
for cause, cnt in sorted(cause_counter.items()):
    pct = cnt / total_sts * 100
    mark = '★' if cause == 'STS' else '○'
    print(f'    {mark} {cause:10}: {cnt:3d}건  ({pct:.0f}%)')

print()
print('  tau_max_setting별 집계:')
print(f'  {"tau":>4} {"총STS":>6} {"STS억제":>8} {"UPeak억제":>10}')
for tau in sorted(tau_cause.keys()):
    d = tau_cause[tau]
    tot = sum(d.values())
    sts_n = d.get('STS', 0)
    up_n  = d.get('UPeak', 0)
    print(f'  {tau:4d} {tot:6d} {sts_n:8d} ({sts_n/tot*100:.0f}%)   {up_n:6d} ({up_n/tot*100:.0f}%)')
