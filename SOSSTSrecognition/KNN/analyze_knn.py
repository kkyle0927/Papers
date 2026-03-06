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

yes = load_csv('yesKNN7.csv')
no  = load_csv('noKNN3.csv')

mode_names = {0:'NONE', 1:'RSTS', 2:'RSOS', 3:'LSTS', 4:'LSOS'}

def analyze_hc_events(rows, label):
    print(f'=== {label}: HC 이벤트 상세 ===')
    prev_hc = 0
    for r in rows:
        hc = int(r['hc_count'])
        if hc != prev_hc:
            t_ms = r['LoopCnt'] * 2
            gm = mode_names.get(int(r['s_gait_mode']), '?')
            conf = r['s_g_knn_conf']
            nv = r['s_norm_vel_HC']
            nt = r['s_norm_T_HC']
            tswing = r['T_swing_ms']
            vel = r['s_vel_HC']
            T_HC = r['s_T_HC_s']
            print(f'  HC#{hc:2d} @ t={t_ms:.0f}ms | mode={gm:5s} | conf={conf:.3f} | '
                  f'norm_vel={nv:.3f} norm_T={nt:.3f} | vel_HC={vel:.1f} deg/s | '
                  f'T_HC={T_HC:.3f}s | T_swing={tswing:.0f}ms')
        prev_hc = hc
    print()

def analyze_mode_transitions(rows, label):
    print(f'=== {label}: STS 감지 및 보조력 억제 확인 ===')
    prev_mode = 0
    for r in rows:
        m = int(r['s_gait_mode'])
        if m != prev_mode:
            t_ms = r['LoopCnt'] * 2
            ltor = r['LeftHipTorque']
            rtor = r['RightHipTorque']
            nm = mode_names.get(m, str(m))
            pm = mode_names.get(prev_mode, str(prev_mode))
            print(f'  t={t_ms:6.0f}ms: {pm:5s} -> {nm:5s} | L_tor={ltor:+.3f} R_tor={rtor:+.3f}')
        prev_mode = m
    print()

def analyze_torque_per_gait(rows, label):
    print(f'=== {label}: gait mode별 보조력 통계 ===')
    buckets = defaultdict(list)
    for r in rows:
        m = int(r['s_gait_mode'])
        buckets[m].append((r['LeftHipTorque'], r['RightHipTorque']))
    for m in sorted(buckets):
        vals = buckets[m]
        lv = [v[0] for v in vals]
        rv = [v[1] for v in vals]
        print(f'  {mode_names.get(m,m):5s}: n={len(vals):4d} | '
              f'L max={max(lv):.3f} mean={sum(lv)/len(lv):.3f} | '
              f'R max={max(rv):.3f} mean={sum(rv)/len(rv):.3f}')
    print()

def analyze_swing_adaptation(rows, label):
    print(f'=== {label}: Swing period 적응 과정 ===')
    prev_ts = rows[0]['T_swing_ms']
    print(f'  초기: T_swing={prev_ts:.1f}ms, SOS={rows[0]["T_swing_SOS_ms"]:.1f}ms, STS={rows[0]["T_swing_STS_ms"]:.1f}ms')
    for r in rows:
        ts = r['T_swing_ms']
        if abs(ts - prev_ts) > 1.0:
            t_ms = r['LoopCnt'] * 2
            print(f'  t={t_ms:.0f}ms: T_swing {prev_ts:.1f} -> {ts:.1f}ms '
                  f'(SOS={r["T_swing_SOS_ms"]:.1f}, STS={r["T_swing_STS_ms"]:.1f})')
            prev_ts = ts
    print(f'  최종: T_swing={rows[-1]["T_swing_ms"]:.1f}ms, '
          f'SOS={rows[-1]["T_swing_SOS_ms"]:.1f}ms, STS={rows[-1]["T_swing_STS_ms"]:.1f}ms')
    print()

def check_sts_suppression(rows, label):
    print(f'=== {label}: STS 구간 토크 억제 검증 ===')
    sts_modes = {1, 3}
    sts_torques_L = []
    sts_torques_R = []
    sos_torques_L = []
    sos_torques_R = []
    none_torques_L = []
    none_torques_R = []
    for r in rows:
        m = int(r['s_gait_mode'])
        lt = r['LeftHipTorque']
        rt = r['RightHipTorque']
        if m in sts_modes:
            sts_torques_L.append(lt)
            sts_torques_R.append(rt)
        elif m in {2, 4}:
            sos_torques_L.append(lt)
            sos_torques_R.append(rt)
        else:
            none_torques_L.append(lt)
            none_torques_R.append(rt)

    def stats(vals, name):
        if not vals:
            return f'{name}: n=0'
        return (f'{name}: n={len(vals)} | max={max(vals):.3f} '
                f'mean={sum(vals)/len(vals):.3f}')

    print(f'  STS 구간 - {stats(sts_torques_L, "L")}')
    print(f'           - {stats(sts_torques_R, "R")}')
    print(f'  SOS 구간 - {stats(sos_torques_L, "L")}')
    print(f'           - {stats(sos_torques_R, "R")}')
    print(f'  NONE     - {stats(none_torques_L, "L")}')
    print(f'           - {stats(none_torques_R, "R")}')
    print()


analyze_hc_events(yes, 'yesKNN7')
analyze_hc_events(no, 'noKNN3')

analyze_mode_transitions(yes, 'yesKNN7')
analyze_mode_transitions(no, 'noKNN3')

analyze_torque_per_gait(yes, 'yesKNN7')
analyze_torque_per_gait(no, 'noKNN3')

analyze_swing_adaptation(yes, 'yesKNN7')
analyze_swing_adaptation(no, 'noKNN3')

check_sts_suppression(yes, 'yesKNN7')
check_sts_suppression(no, 'noKNN3')
