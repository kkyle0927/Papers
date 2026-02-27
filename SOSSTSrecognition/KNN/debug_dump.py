import pandas as pd

df = pd.read_csv('c:/Github/Papers/SOSSTSrecognition/KNN/practice1.csv')

# Find the row around 1918
print("Data around L_count_upeak (LoopCnt 1910 to 1925)")
subset = df[(df['LoopCnt'] >= 1910) & (df['LoopCnt'] <= 1925)]
print(subset[['LoopCnt', 'LeftHipTorque', 'RightHipTorque', 'L_count_upeak', 'T_swing_ms']])
