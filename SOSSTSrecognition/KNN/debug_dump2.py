import pandas as pd

df = pd.read_csv('c:/Github/Papers/SOSSTSrecognition/KNN/practice1.csv')

# Find exactly where LeftTorque drops below 0.5 around 1918 and print the range
subset = df[(df['LoopCnt'] >= 1915) & (df['LoopCnt'] <= 1935)]
print("Data around L_count_upeak (LoopCnt 1915 to 1935)")
print(subset[['LoopCnt', 'LeftHipTorque', 'L_count_upeak']])
