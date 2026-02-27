import pandas as pd

df = pd.read_csv('c:/Github/Papers/SOSSTSrecognition/KNN/practice1.csv')

# Find where R_count_upeak increments
r_upeak_diff = df['R_count_upeak'].diff()
l_upeak_diff = df['L_count_upeak'].diff()

r_upeak_indices = df.index[r_upeak_diff > 0]
l_upeak_indices = df.index[l_upeak_diff > 0]

print("Right Upper Peak events:")
for idx in r_upeak_indices:
    loop_cnt = df.loc[idx, 'LoopCnt']
    torque_before = df.loc[idx-1, 'LeftHipTorque']
    torque_at = df.loc[idx, 'LeftHipTorque']
    torque_after = df.loc[idx+1, 'LeftHipTorque']
    print(f"LoopCnt {loop_cnt}: LeftTorque changed from {torque_before:.4f} -> {torque_at:.4f} -> {torque_after:.4f}")

print("\nLeft Upper Peak events:")
for idx in l_upeak_indices:
    loop_cnt = df.loc[idx, 'LoopCnt']
    torque_before = df.loc[idx-1, 'RightHipTorque']
    torque_at = df.loc[idx, 'RightHipTorque']
    torque_after = df.loc[idx+1, 'RightHipTorque']
    print(f"LoopCnt {loop_cnt}: RightTorque changed from {torque_before:.4f} -> {torque_at:.4f} -> {torque_after:.4f}")

print("\nLet's check if there are any discrete drops (diff < -0.5) overall:")
r_drop_indices = df.index[df['RightHipTorque'].diff() < -0.5]
l_drop_indices = df.index[df['LeftHipTorque'].diff() < -0.5]
print(f"Large drops in RightTorque at LoopCnt: {df.loc[r_drop_indices, 'LoopCnt'].tolist()}")
print(f"Large drops in LeftTorque at LoopCnt: {df.loc[l_drop_indices, 'LoopCnt'].tolist()}")
