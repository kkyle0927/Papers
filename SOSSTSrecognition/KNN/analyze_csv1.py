import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load new data
df = pd.read_csv('c:/Github/Papers/SOSSTSrecognition/KNN/practice1.csv')

# Plot 1: Torque, Thigh Angle, and Triggers
plt.figure(figsize=(15, 10))

plt.subplot(3, 1, 1)
plt.plot(df['LoopCnt'], df['LeftHipTorque'], label='LeftHipTorque', color='r')
plt.plot(df['LoopCnt'], df['RightHipTorque'], label='RightHipTorque', color='b')
plt.title('Hip Torque Profile (practice1.csv)')
plt.ylabel('Torque')
plt.legend()
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(df['LoopCnt'], df['LeftThighAngle'], label='LeftThighAngle', color='r')
plt.plot(df['LoopCnt'], df['RightThighAngle'], label='RightThighAngle', color='b')
plt.title('Thigh Angles')
plt.ylabel('Angle (deg)')
plt.legend()
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(df['LoopCnt'], df['is_moving'], label='is_moving', color='k')
plt.plot(df['LoopCnt'], df['s_gait_mode'], label='s_gait_mode (STS=2/3)', color='m')
plt.title('State Variables')
plt.xlabel('LoopCnt')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig('c:/Github/Papers/SOSSTSrecognition/KNN/torque_analysis_p1_1.png')

# Plot 2: Timing variables
plt.figure(figsize=(15, 8))

plt.subplot(2, 1, 1)
plt.plot(df['LoopCnt'], df['T_swing_ms'], label='T_swing_ms (Norm ref)', color='g')
plt.title('Swing Period Reference')
plt.legend()
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(df['LoopCnt'], df['LeftHipTorque'], label='LeftTorque', color='r')
plt.plot(df['LoopCnt'], df['RightHipTorque'], label='RightTorque', color='b')
plt.title('Torque Overlay')
plt.xlabel('LoopCnt')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig('c:/Github/Papers/SOSSTSrecognition/KNN/torque_analysis_p1_2.png')

print("Plots saved as torque_analysis_p1_1.png and torque_analysis_p1_2.png")
