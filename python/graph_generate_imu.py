import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

def butter_lowpass(lowcut, fs, order=4):
    '''バターワースローパスフィルタを設計する関数
    '''
    nyq = 0.5 * fs
    low = lowcut / nyq
    b, a = signal.butter(order, low, btype='low')
    return b, a


def butter_lowpass_filter(x, lowcut, fs, order=4):
    '''データにローパスフィルタをかける関数
    '''
    b, a = butter_lowpass(lowcut, fs, order=order)
    y = signal.filtfilt(b, a, x)
    return y


# csvファイルパス (アップロードされたファイルを直接指定)
file_path_cog_body_foot_ref = '../log/AY2025/Oct/2025-10-01_18-42-27/log.csv'
file_path_body_acc_angle = '../log/AY2025/Oct/2025-10-01_18-42-27/log_imu.csv'
file_path_cog_body = '../log/AY2025/Oct/2025-10-01_18-42-27/log_cog.csv'


fig_base_acc_path = '../log/AY2025/Oct/2025-10-01_18-42-27/base_acceleration_plots.png'
fig_base_angle_path= '../log/AY2025/Oct/2025-10-01_18-42-27/base_angle_plots.png'

# csvファイルを読み込み (タブ区切りであるこsとを指定)
try:
    # sep='\t'は必須
    df_cog_ref = pd.read_csv(file_path_cog_body_foot_ref, sep='\t')
    print("CSV file read successfully.")
except FileNotFoundError:
    print(f"Error: The file '{file_path_cog_body_foot_ref}' was not found.")
    exit()

try:
    # sep='\t'は必須
    df_body = pd.read_csv(file_path_body_acc_angle, sep='\t')
    print("CSV file read successfully.")
except FileNotFoundError:
    print(f"Error: The file '{file_path_body_acc_angle}' was not found.")
    exit()

try:
    # sep='\t'は必須
    df_cog = pd.read_csv(file_path_cog_body, sep='\t')
    print("CSV file read successfully.")
except FileNotFoundError:
    print(f"Error: The file '{file_path_cog_body}' was not found.")
    exit()

# mylog<<"time\t"<<"COG_x\t"<<"COG_y\t"<<"COG_z\t"<<"Pr_x\t"<<"Pr_y\t"<<"Pr_z\t"<<"Pl_x\t"<<"Pl_y\t"<<"Pl_z\t"<<"BODY_x\t"<<"BODY_y\t"<<"BODY_z\t"<<"BODY_vx\t"<<"BODY_vy\t"<<"BODY_vz\t"<<"BODY_ax\t"<<"BODY_ay\t"<<"BODY_az\t"<<"COG_vx\t"<<"COG_vy\t"<<"COG_vz\t"<<"COG_ax\t"<<"COG_ay\t"<<"COG_az\t"<<"Rfoot_ref_x\t"<<"Rfoot_ref_y\t"<<"Rfoot_ref_z\t"<<"Lfoot_ref_x\t"<<"Lfoot_ref_y\t"<<"Lfoot_ref_z\t"<<std::endl;
t_cog_ref=df_cog_ref['time']
COG_p_ref = df_cog_ref[['COG_x', 'COG_y', 'COG_z']]
R_foot_ref = df_cog_ref[['Pr_x', 'Pr_y', 'Pr_z']]
L_foot_ref = df_cog_ref[['Pr_x', 'Pr_y', 'Pr_z']]
BODY_p_ref = df_cog_ref[['BODY_x', 'BODY_y', 'BODY_z']]
BODY_v_ref = df_cog_ref[['BODY_vx', 'BODY_vy', 'BODY_vz']]
BODY_a_ref = df_cog_ref[['BODY_ax', 'BODY_ay', 'BODY_az']]
COG_v_ref = df_cog_ref[['COG_vx', 'COG_vy', 'COG_vz']]
COG_a_ref = df_cog_ref[['COG_ax', 'COG_ay', 'COG_az']]

# mylog_imu<<"Time\t"<<"IMU_ddx\t"<<"IMU_ddy\t"<<"IMU_ddz\t"<<"IMU_yaw\t"<<"IMU_roll\t"<<"IMU_pitch\t"<<std::endl;
t_body=df_body['Time']
BODY_a_imu = df_body[['IMU_ddx', 'IMU_ddy', 'IMU_ddz']]
BODY_angle = df_body[['IMU_yaw', 'IMU_roll', 'IMU_pitch']]

# mylog_cog<<"time\t"<<"COG_x\t"<<"COG_y\t"<<"COG_z\t"<<"COG_xx\t"<<"COG_yy\t"<<"COG_zz\t"<<"p_x\t"<<"p_y\t"<<"p_z\t"<<"BODY_x\t"<<"BODY_y\t"<<"BODY_z\t"<<"BODY_vx\t"<<"BODY_vy\t"<<"BODY_vz\t"<<"BODY_ax\t"<<"BODY_ay\t"<<"BODY_az\t"<<"COG_vx\t"<<"COG_vy\t"<<"COG_vz\t"<<"COG_ax\t"<<"COG_ay\t"<<"COG_az\t"<<std::endl;
t_cog=df_cog['time']
COG_p = df_cog[['COG_x', 'COG_y', 'COG_z']]
BODY_p = df_cog[['BODY_x', 'BODY_y', 'BODY_z']]
BODY_v = df_cog[['BODY_vx', 'BODY_vy', 'BODY_vz']]
BODY_a = df_cog[['BODY_ax', 'BODY_ay', 'BODY_az']]
COG_v = df_cog[['COG_vx', 'COG_vy', 'COG_vz']]
COG_a = df_cog[['COG_ax', 'COG_ay', 'COG_az']]

data_num=t_body.shape[0]
print("number of calculations=",data_num)
g=9.8
N = data_num # サンプル数
dt = t_body.values[data_num-1]/(data_num-1) # サンプリング周期 [s]
fs = 1 / dt
fc=3#カットオフ周波数
print(dt)
#座標変換
body_a_raw=np.zeros((data_num,3))
for i in range(data_num):

    R_roll=np.array([[1,0,0],
                    [0,np.cos(BODY_angle.values[i][1]),-np.sin(BODY_angle.values[i][1])],
                    [0,np.sin(BODY_angle.values[i][1]),np.cos(BODY_angle.values[i][1])]])

    R_pitch=np.array([[np.cos(BODY_angle.values[i][2]),0,np.sin(BODY_angle.values[i][2])],
                    [0,1,0],
                    [-np.sin(BODY_angle.values[i][2]),0,np.cos(BODY_angle.values[i][2])]])

    R_yow=np.array([[1,0,0],
                    [0,1,0],
                    [0,0,1]])
    R=R_yow@R_pitch@R_roll
    acc_imu=np.array([BODY_a_imu.values[i][0],BODY_a_imu.values[i][1],BODY_a_imu.values[i][2]])
    acc_tran=R@acc_imu
    acc_tran[2]=acc_tran[2]#-g_offset
    body_a_raw[i:]=acc_tran

body_a_filtered = np.zeros((data_num, 3))

# X軸のフィルタ処理
body_a_filtered[:, 0] = butter_lowpass_filter(body_a_raw[:, 0], fc, fs, order=4)*g
# Y軸のフィルタ処理
body_a_filtered[:, 1] = butter_lowpass_filter(body_a_raw[:, 1], fc, fs, order=4)*g
# Z軸のフィルタ処理 (重力除去はここで検討)
body_a_filtered[:, 2] = butter_lowpass_filter(body_a_raw[:, 2], fc, fs, order=4)*g- g 

# print(body_a_filtered)



# ## グラフ生成-----------------------

# ベースリンク加速度
fig_base, ax_base = plt.subplots(1, 3, figsize=(18, 18))

# ここを修正: .values を追加してNumPy配列に変換
ax_base[0].plot(t_cog_ref.values, BODY_a_ref['BODY_ax'].values, label="BASE_ax_ref", linestyle="--")
ax_base[0].plot(t_body.values, body_a_filtered[:,0], label="BASE_ax_imu")
ax_base[0].plot(t_cog.values, BODY_a['BODY_ax'].values, label="BASE_ax_estimate")
# ax_base[0].plot(t_body.values, body_a_raw[:,0]*g, label="BASE_ax_raw")
# ax_base[0].plot(t_body.values, BODY_a_imu.values[:,0]*g, label="BASE_ax")
# ax_base[0, 0].set_xlim(4, 8)
ax_base[0].set_xlabel("Time [s]")
ax_base[0].set_ylabel("Acceleration [m/s^2]")
ax_base[0].set_title("BASE_ddx")
ax_base[0].legend(loc='best')
ax_base[0].grid(True)

ax_base[1].plot(t_cog_ref.values, BODY_a_ref['BODY_ay'].values, label="BASE_ay_ref", linestyle="--")
ax_base[1].plot(t_body.values, body_a_filtered[:,1], label="BASE_ay_imu")
ax_base[1].plot(t_cog.values, BODY_a['BODY_ay'].values, label="BASE_ay_estimate")
# ax_base[1].plot(t_body.values, body_a_raw[:,1]*g, label="BASE_ay_raw")
# ax_base[1].plot(t_body.values, BODY_a_imu.values[:,1]*g, label="BASE_ay")
# ax_base[0, 1].set_xlim(4, 8)
ax_base[1].set_xlabel("Time [s]")
ax_base[1].set_ylabel("Acceleration [m/s^2]")
ax_base[1].set_title("BASE_ddy")
ax_base[1].legend(loc='best')
ax_base[1].grid(True)

ax_base[2].plot(t_cog_ref.values, BODY_a_ref['BODY_az'].values, label="BASE_az_ref", linestyle="--")
ax_base[2].plot(t_body.values, body_a_filtered[:,2], label="BASE_az_imu")
ax_base[2].plot(t_cog.values, BODY_a['BODY_az'].values, label="BASE_az_estimate")
# ax_base[2].plot(t_body.values, body_a_raw[:,2]*g, label="BASE_az_raw")
# ax_base[2].plot(t_body.values, BODY_a_imu.values[:,2]*g, label="BASE_az")
# ax_base[0, 2].set_xlim(4, 8)
ax_base[2].set_xlabel("Time [s]")
ax_base[2].set_ylabel("Acceleration [m/s^2]")
ax_base[2].set_title("BASE_ddz")
ax_base[2].legend(loc='best')
ax_base[2].grid(True)

# グラフをPNGファイルとして保存
plt.tight_layout()
plt.savefig(fig_base_acc_path)
print("Base acceleration plots saved as 'base_acceleration_plots.png'")
#plt.show()

# ベースリンク加速度
fig_angle, ax_angle = plt.subplots(1, 3, figsize=(18, 18))

# ここを修正: .values を追加してNumPy配列に変換
# ax_angle[0].plot(t_cog_ref.values, BODY_a_ref['BODY_ax'].values, label="BASE_ax_ref", linestyle="--")
ax_angle[0].plot(t_body.values, BODY_angle.values[:,1]*(180/3.14), label="BASE_roll")
# ax_angle[0, 0].set_xlim(4, 8)
ax_angle[0].set_xlabel("Time [s]")
ax_angle[0].set_ylabel("Angle [deg]")
ax_angle[0].set_title("BASE_roll")
ax_angle[0].legend(loc='best')
ax_angle[0].grid(True)

# ax_angle[1].plot(t_cog_ref.values, BODY_a_ref['BODY_ay'].values, label="BASE_ay_ref", linestyle="--")
ax_angle[1].plot(t_body.values, BODY_angle.values[:,2]*(180/3.14), label="BASE_pitch")
# ax_angle[0, 1].set_xlim(4, 8)
ax_angle[1].set_xlabel("Time [s]")
ax_angle[1].set_ylabel("Angle [deg]")
ax_angle[1].set_title("BASE_pitch")
ax_angle[1].legend(loc='best')
ax_angle[1].grid(True)

# ax_angle[2].plot(t_cog_ref.values, BODY_a_ref['BODY_az'].values, label="BASE_az_ref", linestyle="--")
ax_angle[2].plot(t_body.values, BODY_angle.values[:,0]*(180/3.14), label="BASE_yaw")
# ax_angle[0, 2].set_xlim(4, 8)
ax_angle[2].set_xlabel("Time [s]")
ax_angle[2].set_ylabel("Angle [deg]")
ax_angle[2].set_title("BASE_yaw")
ax_angle[2].legend(loc='best')
ax_angle[2].grid(True)

# グラフをPNGファイルとして保存
plt.tight_layout()
plt.savefig(fig_base_angle_path)
print("Base angle plots saved as 'base_angle_plots.png'")
plt.show()
