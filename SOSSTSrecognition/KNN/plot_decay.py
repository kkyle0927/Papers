import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('c:/Github/Papers/SOSSTSrecognition/KNN/practice1.csv')

# Let's find an upper peak event where the torque drops
r_upeak_diff = df['R_count_upeak'].diff()
# Get the first right upper peak index
upeak_idx = df.index[r_upeak_diff > 0][0]

# Plot a window around it
start_idx = max(0, upeak_idx - 50)
end_idx = min(len(df), upeak_idx + 150)

subset = df.iloc[start_idx:end_idx]

plt.figure(figsize=(12, 6))
plt.plot(subset['LoopCnt'], subset['LeftHipTorque'], 'r.-', label='LeftTorque')
plt.plot(subset['LoopCnt'], subset['RightHipTorque'], 'b.-', label='RightTorque')

plt.axvline(x=df.loc[upeak_idx, 'LoopCnt'], color='g', linestyle='--', label='R_count_upeak triggers')

plt.title('Torque Drop at Upper Peak')
plt.xlabel('LoopCnt')
plt.ylabel('Torque')
plt.legend()
plt.grid(True)
plt.savefig('c:/Github/Papers/SOSSTSrecognition/KNN/torque_drop_check.png')
print("Plot saved as torque_drop_check.png")
