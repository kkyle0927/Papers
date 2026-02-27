import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('c:/Github/Papers/SOSSTSrecognition/KNN/practice1.csv')

# Zoom into a specific drop (e.g. around loop count 2000)
# Find where the first significant torque occurs, then plot range around it.
mask = df['LeftHipTorque'] > 0.5
if mask.any():
    first_peak_idx = mask.idxmax()
    start_idx = max(0, first_peak_idx - 100)
    end_idx = min(len(df), first_peak_idx + 800)
    
    plt.figure(figsize=(10, 5))
    plt.plot(df['LoopCnt'].iloc[start_idx:end_idx], df['LeftHipTorque'].iloc[start_idx:end_idx], 'r.-', label='LeftTorque')
    plt.plot(df['LoopCnt'].iloc[start_idx:end_idx], df['RightHipTorque'].iloc[start_idx:end_idx], 'b.-', label='RightTorque')
    plt.title('Zoomed In Torque Profile')
    plt.grid()
    plt.legend()
    plt.savefig('c:/Github/Papers/SOSSTSrecognition/KNN/torque_zoom.png')
    print("Saved torque_zoom.png")

