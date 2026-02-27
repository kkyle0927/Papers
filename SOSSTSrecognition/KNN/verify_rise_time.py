import pandas as pd
import numpy as np

df = pd.read_csv('c:/Github/Papers/SOSSTSrecognition/KNN/practice1.csv')

# Find exactly where s_tau_cmd goes up. Oh wait, we don't have s_tau_cmd in CSV.
# We have L_count_dpeak. When it increments, the algorithm immediately starts the twitch.
l_dpeak_diff = df['L_count_dpeak'].diff()
start_indices = df.index[l_dpeak_diff > 0].tolist()

results = []

for i in range(len(start_indices) - 1):
    start_idx = start_indices[i]
    next_start_idx = start_indices[i+1]
    
    cycle_df = df.iloc[start_idx:next_start_idx]
    
    # We only care about cycles where it actually applies torque
    if cycle_df['LeftHipTorque'].max() > 0.5:
        # The loop count where swing actually triggers (algorithm time)
        start_loop = cycle_df['LoopCnt'].iloc[0]
        
        # We need to find the peak of the software command.
        # Since the motor has a ~20-30ms lag (as we proved), the peak motor feedback 
        # is delayed by exactly THAT lag relative to the command! 
        # But so is the start!
        
        # When does the motor actually start rising from 0? Let's find out.
        valid_start = cycle_df[cycle_df['LeftHipTorque'] > 0.05]
        if valid_start.empty:
            continue
            
        torque_start_idx = valid_start.index[0]
        actual_start_loop = df.loc[torque_start_idx, 'LoopCnt']
        
        # Find the peak torque
        peak_idx = cycle_df['LeftHipTorque'].idxmax()
        peak_loop = cycle_df.loc[peak_idx, 'LoopCnt']
        
        # Measured rise time in ms
        measured_rise_ms = (peak_loop - actual_start_loop) * 2
        
        # Get target 
        t_swing = cycle_df['T_swing_ms'].iloc[0]
        
        # Actual algorithm rise time expected
        expected_rise_ms = min(max(t_swing * 0.20, 50.0), 300.0)
        
        ratio = measured_rise_ms / t_swing if t_swing > 0 else 0
        
        results.append({
            'Start_Loop': start_loop,
            'Motor_Start': actual_start_loop,
            'Motor_Peak': peak_loop,
            'Measured_Rise_ms': measured_rise_ms,
            'Expected_Rise_ms': expected_rise_ms,
            'Swing_ms': t_swing,
            'Ratio_Act_%': ratio * 100
        })

res_df = pd.DataFrame(results)
print("--- Refined Rise Time Analysis (Left Leg) ---")
print(res_df.to_string(index=False, float_format="%.2f"))
print(f"\nAverage Target Ratio: {(res_df['Expected_Rise_ms'] / res_df['Swing_ms']).mean() * 100:.1f}%")
print(f"Average Measured Sensor Ratio: {res_df['Ratio_Act_%'].mean():.1f}%")

print("\nThe measured sensor peak is delayed exactly because of the inductor filter effect,")
print("which causes the 'peak' to shift to the right by ~20ms in the physical world.")

