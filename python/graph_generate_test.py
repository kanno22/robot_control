import pandas as pd
import matplotlib.pyplot as plt

# csvファイルパス (アップロードされたファイルを直接指定)
file_path_cog_body_foot_ref = '../log/AY2025/Oct/2025-10-01_14-37-02/log.csv'
file_path_joint_ref = '../log/AY2025/Oct/2025-10-01_14-37-02/log2.csv'

fig_right_path = '../log/AY2025/Oct/2025-10-01_14-37-02/right_leg_plots.png'
fig_left_path = '../log/AY2025/Oct/2025-10-01_14-37-02/left_leg_plots.png'
fig_cog_path = '../log/AY2025/Oct/2025-10-01_14-37-02/cog_plots.png'
fig_top_path='../log/AY2025/Oct/2025-10-01_14-37-02/cog_top.png'
fig_foot_path='../log/AY2025/Oct/2025-10-01_14-37-02/foot_plots.png'

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
    df_joint_ref = pd.read_csv(file_path_joint_ref, sep='\t')
    print("CSV file read successfully.")
except FileNotFoundError:
    print(f"Error: The file '{file_path_joint_ref}' was not found.")
    exit()

# mylog<<"time\t"<<"COG_x\t"<<"COG_y\t"<<"COG_z\t"<<"Pr_x\t"<<"Pr_y\t"<<"Pr_z\t"<<"Pl_x\t"<<"Pl_y\t"<<"Pl_z\t"<<"BODY_x\t"<<"BODY_y\t"<<"BODY_z\t"<<"BODY_vx\t"<<"BODY_vy\t"<<"BODY_vz\t"<<"BODY_ax\t"<<"BODY_ay\t"<<"BODY_az\t"<<"COG_vx\t"<<"COG_vy\t"<<"COG_vz\t"<<"COG_ax\t"<<"COG_ay\t"<<"COG_az\t"<<"Rfoot_ref_x\t"<<"Rfoot_ref_y\t"<<"Rfoot_ref_z\t"<<"Lfoot_ref_x\t"<<"Lfoot_ref_y\t"<<"Lfoot_ref_z\t"<<std::endl;
t_cog_ref=df_cog_ref['time']
COG_p_ref = df_cog_ref[['COG_x', 'COG_y', 'COG_z']]
R_foot_ref = df_cog_ref[['Pr_x', 'Pr_y', 'Pr_z']]
L_foot_ref = df_cog_ref[['Pl_x', 'Pl_y', 'Pl_z']]
BODY_p_ref = df_cog_ref[['BODY_x', 'BODY_y', 'BODY_z']]
BODY_v_ref = df_cog_ref[['BODY_vx', 'BODY_vy', 'BODY_vz']]
BODY_a_ref = df_cog_ref[['BODY_ax', 'BODY_ay', 'BODY_az']]
COG_v_ref = df_cog_ref[['COG_vx', 'COG_vy', 'COG_vz']]
COG_a_ref = df_cog_ref[['COG_ax', 'COG_ay', 'COG_az']]


#  mylog2<<"time\t"<<"R_C_yaw\t"<<"R_C_roll\t"<<"R_C_pitch\t"<<"R_Linear\t"<<"R_A_pitch\t"<<"R_A_roll\t"<<"L_C_yaw\t"<<"L_C_roll\t"<<"L_C_pitch\t"<<"L_Linear\t"<<"L_A_pitch\t"<<"L_A_roll\t"<<std::endl;
t_joint_ref = df_joint_ref['time']

R_hip_roll_ref = df_joint_ref['R_C_roll']
R_hip_pitch_ref = df_joint_ref['R_C_pitch']
R_knee_ref = df_joint_ref['R_Linear']
R_ankle_pitch_ref = df_joint_ref['R_A_pitch']
R_ankle_roll_ref = df_joint_ref['R_A_roll']

L_hip_roll_ref = df_joint_ref['L_C_roll']
L_hip_pitch_ref = df_joint_ref['L_C_pitch']
L_knee_ref = df_joint_ref['L_Linear']
L_ankle_pitch_ref = df_joint_ref['L_A_pitch']
L_ankle_roll_ref = df_joint_ref['L_A_roll']

## グラフ生成-----------------------
# 右脚のグラフ
fig_r, ax_r = plt.subplots(2, 2, figsize=(18, 10))

# ここを修正: .values を追加してNumPy配列に変換
ax_r[0, 0].plot(t_joint_ref.values, R_hip_roll_ref.values, label="R_hip_roll_angle_ref", linestyle="--")
# ax_r[0, 0].plot(t_joint_ref.values, R_hip_roll['R_C_roll_angle'].values, label="R_hip_roll_angle")
ax_r[0, 0].plot(t_joint_ref.values, R_hip_pitch_ref.values, label="R_hip_pitch_angle_ref", linestyle="--")
# ax_r[0, 0].plot(t_joint_ref.values, R_hip_pitch['R_C_pitch_angle'].values, label="R_hip_pitch_angle")
ax_r[0, 0].plot(t_joint_ref.values, R_ankle_pitch_ref.values, label="R_ankle_pitch_angle_ref", linestyle="--")
# ax_r[0, 0].plot(t_joint_ref.values, R_ankle_pitch['R_A_pitch_angle'].values, label="R_ankle_pitch_angle")
ax_r[0, 0].plot(t_joint_ref.values, R_ankle_roll_ref.values, label="R_ankle_roll_angle_ref", linestyle="--")
# ax_r[0, 0].plot(t_joint_ref.values, R_ankle_roll['R_A_roll_angle'].values, label="R_ankle_roll_angle")
# ax_r[0, 0].set_ylim(-100, 100)
ax_r[0, 0].set_xlabel("Time [s]")
ax_r[0, 0].set_ylabel("Angle [deg]")
ax_r[0, 0].set_title("R_Angle")
ax_r[0, 0].legend(loc='best')
ax_r[0, 0].grid(True)

ax_r[0, 1].plot(t_joint_ref.values, R_knee_ref.values, label="R_knee_length_ref", linestyle="--")
# ax_r[0, 1].plot(t_joint_ref.values, R_knee.values, label="R_knee_length")
ax_r[0, 1].set_xlabel("Time [s]")
ax_r[0, 1].set_ylabel("Length [m]")
ax_r[0, 1].set_title("R_Length")
ax_r[0, 1].legend(loc='best')
ax_r[0, 1].grid(True)

# ax_r[1, 0].plot(t_joint_ref.values, R_hip_roll['R_C_roll_current'].values, label="R_hip_roll_current")
# ax_r[1, 0].plot(t_joint_ref.values, R_hip_pitch['R_C_pitch_current'].values, label="R_hip_pitch_current")
# ax_r[1, 0].plot(t_joint_ref.values, R_ankle_pitch['R_A_pitch_current'].values, label="R_ankle_pitch_current")
# ax_r[1, 0].plot(t_joint_ref.values, R_ankle_roll['R_A_roll_current'].values, label="R_ankle_roll_current")
ax_r[1, 0].set_xlabel("Time [s]")
ax_r[1, 0].set_ylabel("Current [mA]")
ax_r[1, 0].set_title("R_Current")
ax_r[1, 0].legend(loc='best')
ax_r[1, 0].grid(True)

# ax_r[1, 1].plot(t.values, R_knee['R_knee_force'].values, label="R_knee_force")
# ax_r[1, 1].set_xlabel("Time [s]")
# ax_r[1, 1].set_ylabel("Force [N]")
# ax_r[1, 1].set_title("R_Force")
# ax_r[1, 1].legend(loc='best')
# ax_r[1, 1].grid(True)

# グラフをPNGファイルとして保存
plt.tight_layout()
plt.savefig(fig_right_path)
print("Right leg plots saved as 'right_leg_plots.png'")
#plt.show()

# 左脚のグラフ
fig_l, ax_l = plt.subplots(2, 2, figsize=(18, 10))

# ここを修正: .values を追加してNumPy配列に変換
ax_l[0, 0].plot(t_joint_ref.values, L_hip_roll_ref.values, label="L_hip_roll_angle_ref", linestyle="--")
# ax_l[0, 0].plot(t_joint_ref.values, L_hip_roll['L_C_roll_angle'].values, label="L_hip_roll_angle")
ax_l[0, 0].plot(t_joint_ref.values, L_hip_pitch_ref.values, label="L_hip_pitch_angle_ref", linestyle="--")
# ax_l[0, 0].plot(t_joint_ref.values, L_hip_pitch['L_C_pitch_angle'].values, label="L_hip_pitch_angle")
ax_l[0, 0].plot(t_joint_ref.values, L_ankle_pitch_ref.values, label="L_ankle_pitch_angle_ref", linestyle="--")
# ax_l[0, 0].plot(t_joint_ref.values, L_ankle_pitch['L_A_pitch_angle'].values, label="L_ankle_pitch_angle")
ax_l[0, 0].plot(t_joint_ref.values, L_ankle_roll_ref.values, label="L_ankle_roll_angle_ref", linestyle="--")
# ax_l[0, 0].plot(t_joint_ref.values, L_ankle_roll['L_A_roll_angle'].values, label="L_ankle_roll_angle")
# ax_l[0, 0].set_ylim(-100, 100)
ax_l[0, 0].set_xlabel("Time [s]")
ax_l[0, 0].set_ylabel("Angle [deg]")
ax_l[0, 0].set_title("L_Angle")
ax_l[0, 0].legend(loc='best')
ax_l[0, 0].grid(True)

ax_l[0, 1].plot(t_joint_ref.values, L_knee_ref.values, label="L_knee_length_ref", linestyle="--")
# ax_l[0, 1].plot(t_joint_ref.values, L_knee.values, label="L_knee_length")
ax_l[0, 1].set_xlabel("Time [s]")
ax_l[0, 1].set_ylabel("Length [m]")
ax_l[0, 1].set_title("L_Length")
ax_l[0, 1].legend(loc='best')
ax_l[0, 1].grid(True)

# ax_l[1, 0].plot(t_joint_ref.values, L_hip_roll['L_C_roll_current'].values, label="L_hip_roll_current")
# ax_l[1, 0].plot(t_joint_ref.values, L_hip_pitch['L_C_pitch_current'].values, label="L_hip_pitch_current")
# ax_l[1, 0].plot(t_joint_ref.values, L_ankle_pitch['L_A_pitch_current'].values, label="L_ankle_pitch_current")
# ax_l[1, 0].plot(t_joint_ref.values, L_ankle_roll['L_A_roll_current'].values, label="L_ankle_roll_current")
ax_l[1, 0].set_xlabel("Time [s]")
ax_l[1, 0].set_ylabel("Current [mA]")
ax_l[1, 0].set_title("L_Current")
ax_l[1, 0].legend(loc='best')
ax_l[1, 0].grid(True)

# ax_l[1, 1].plot(t.values, L_knee['L_knee_force'].values, label="L_knee_force")
# ax_l[1, 1].set_xlabel("Time [s]")
# ax_l[1, 1].set_ylabel("Force [N]")
# ax_l[1, 1].set_title("L_Force")
# ax_l[1, 1].legend(loc='best')
# ax_l[1, 1].grid(True)

# グラフをPNGファイルとして保存
plt.tight_layout()
plt.savefig(fig_left_path)
print("Left leg plots saved as 'left_leg_plots.png'")
#plt.show()

# 重心のグラフ
fig_cog, ax_cog = plt.subplots(3, 3, figsize=(18, 18))

# ここを修正: .values を追加してNumPy配列に変換
ax_cog[0, 0].plot(t_cog_ref.values, COG_p_ref['COG_x'].values, label="COG_x_ref", linestyle="--")
# ax_cog[0, 0].plot(t.values, COG_x['COG_x'].values, label="COG_x")
# ax_cog[0, 0].set_xlim(4, 8)
ax_cog[0, 0].set_xlabel("Time [s]")
ax_cog[0, 0].set_ylabel("Position [m]")
ax_cog[0, 0].set_title("COG_x")
ax_cog[0, 0].legend(loc='best')
ax_cog[0, 0].grid(True)

ax_cog[0, 1].plot(t_cog_ref.values, COG_v_ref['COG_vx'].values, label="COG_dx_ref", linestyle="--")
# ax_cog[0, 1].plot(t.values, COG_dx['COG_dx'].values, label="COG_dx")
# ax_cog[0, 1].set_xlim(4, 8)
ax_cog[0, 1].set_xlabel("Time [s]")
ax_cog[0, 1].set_ylabel("Velocity [m/s]")
ax_cog[0, 1].set_title("COG_dx")
ax_cog[0, 1].legend(loc='best')
ax_cog[0, 1].grid(True)

ax_cog[0, 2].plot(t_cog_ref.values, COG_a_ref['COG_ax'].values, label="COG_ddx_ref", linestyle="--")
# ax_cog[0, 2].plot(t.values, COG_ddx['COG_ddx'].values, label="COG_ddx")
# ax_cog[0, 2].set_xlim(4, 8)
ax_cog[0, 2].set_xlabel("Time [s]")
ax_cog[0, 2].set_ylabel("Acceleration [m/s^2]")
ax_cog[0, 2].set_title("COG_ddx")
ax_cog[0, 2].legend(loc='best')
ax_cog[0, 2].grid(True)

ax_cog[1, 0].plot(t_cog_ref.values, COG_p_ref['COG_y'].values, label="COG_y_ref", linestyle="--")
# ax_cog[1, 0].plot(t.values, COG_y['COG_y'].values, label="COG_y")
# ax_cog[1, 0].set_xlim(4, 8)
ax_cog[1, 0].set_xlabel("Time [s]")
ax_cog[1, 0].set_ylabel("Position [m]")
ax_cog[1, 0].set_title("COG_y")
ax_cog[1, 0].legend(loc='best')
ax_cog[1, 0].grid(True)

ax_cog[1, 1].plot(t_cog_ref.values, COG_v_ref['COG_vy'].values, label="COG_dy_ref", linestyle="--")
# ax_cog[1, 1].plot(t.values, COG_dy['COG_dy'].values, label="COG_dy")
# ax_cog[1, 1].set_xlim(4, 8)
ax_cog[1, 1].set_xlabel("Time [s]")
ax_cog[1, 1].set_ylabel("Velocity [m/s]")
ax_cog[1, 1].set_title("COG_dy")
ax_cog[1, 1].legend(loc='best')
ax_cog[1, 1].grid(True)

ax_cog[1, 2].plot(t_cog_ref.values, COG_a_ref['COG_ay'].values, label="COG_ddy_ref", linestyle="--")
# ax_cog[1, 2].plot(t.values, COG_ddy['COG_ddy'].values, label="COG_ddy")
# ax_cog[1, 2].set_xlim(4, 8)
ax_cog[1, 2].set_xlabel("Time [s]")
ax_cog[1, 2].set_ylabel("Acceleration [m/s^2]")
ax_cog[1, 2].set_title("COG_ddy")
ax_cog[1, 2].legend(loc='best')
ax_cog[1, 2].grid(True)

ax_cog[2, 0].plot(t_cog_ref.values, COG_p_ref['COG_z'].values, label="COG_z_ref", linestyle="--")
# ax_cog[2, 0].plot(t.values, COG_z['COG_z'].values, label="COG_z")
# ax_cog[2, 0].set_xlim(4, 8)
ax_cog[2, 0].set_xlabel("Time [s]")
ax_cog[2, 0].set_ylabel("Position [m]")
ax_cog[2, 0].set_title("COG_z")
ax_cog[2, 0].legend(loc='best')
ax_cog[2, 0].grid(True)

ax_cog[2, 1].plot(t_cog_ref.values, COG_v_ref['COG_vz'].values, label="COG_dz_ref", linestyle="--")
# ax_cog[2, 1].plot(t.values, COG_dz['COG_dz'].values, label="COG_dz")
# ax_cog[2, 1].set_xlim(4, 8)
ax_cog[2, 1].set_xlabel("Time [s]")
ax_cog[2, 1].set_ylabel("Velocity [m/s]")
ax_cog[2, 1].set_title("COG_dz")
ax_cog[2, 1].legend(loc='best')
ax_cog[2, 1].grid(True)

ax_cog[2, 2].plot(t_cog_ref.values, COG_a_ref['COG_az'].values, label="COG_ddz_ref", linestyle="--")
# ax_cog[2, 2].plot(t.values, COG_ddz['COG_ddz'].values, label="COG_ddz")
# ax_cog[2, 2].set_xlim(4, 8)
#ax_cog[0, 0].set_ylim(-100, 100)
ax_cog[2, 2].set_xlabel("Time [s]")
ax_cog[2, 2].set_ylabel("Acceleration [m/s^2]")
ax_cog[2, 2].set_title("COG_ddz")
ax_cog[2, 2].legend(loc='best')
ax_cog[2, 2].grid(True)
# グラフをPNGファイルとして保存
plt.tight_layout()
plt.savefig(fig_cog_path)
print("Cog plots saved as 'cog_plots.png'")
#plt.show()

# 俯瞰からの重心位置
fig_top, ax_top = plt.subplots(1, 1, figsize=(18, 18))
# ここを修正: .values を追加してNumPy配列に変換
ax_top.plot(COG_p_ref['COG_x'].values, COG_p_ref['COG_y'].values, label="COG_ref", linestyle="--")
# ax_top.plot(COG_x['COG_x'].values, COG_y['COG_y'].values, label="COG")
ax_top.plot(BODY_p_ref['BODY_x'].values, BODY_p_ref['BODY_y'].values, label="BASE_ref", linestyle="--")
# ax_top.plot(base_x['BASE_x'].values, base_y['BASE_y'].values, label="BASE")
# ax_top.plot(zmp_x['zmp_x_ref'].values, zmp_y['zmp_y_ref'].values, label="zmp_ref", linestyle="--")
#ax_cog[0, 0].set_ylim(-100, 100)
ax_top.set_xlabel("Position [m]")
ax_top.set_ylabel("Position [m]")
ax_top.set_title("Position")
ax_top.legend(loc='best')
ax_top.grid(True)
# グラフをPNGファイルとして保存
plt.tight_layout()
plt.savefig(fig_top_path)
print("Top plots saved as 'top_plots.png'")
# plt.show()

# 足リンクのグラフ
fig_foot, ax_foot = plt.subplots(2, 3, figsize=(18, 18))

# ここを修正: .values を追加してNumPy配列に変換
ax_foot[0, 0].plot(t_cog_ref.values, R_foot_ref['Pr_x'].values, label="R_foot_x_ref", linestyle="--")
# ax_foot[0, 0].plot(t_cog.values, COG_p['COG_x'].values, label="COG_x")
# ax_foot[0, 0].set_xlim(4, 8)
ax_foot[0, 0].set_xlabel("Time [s]")
ax_foot[0, 0].set_ylabel("Position [m]")
ax_foot[0, 0].set_title("R_foot_x")
ax_foot[0, 0].legend(loc='best')
ax_foot[0, 0].grid(True)

ax_foot[0, 1].plot(t_cog_ref.values, R_foot_ref['Pr_y'].values, label="R_foot_y_ref", linestyle="--")
# ax_foot[0, 1].plot(t_cog.values, COG_v['COG_vx'].values, label="COG_dx")
# ax_foot[0, 1].set_xlim(4, 8)
ax_foot[0, 1].set_xlabel("Time [s]")
ax_foot[0, 1].set_ylabel("Position [m]")
ax_foot[0, 1].set_title("R_foot_y")
ax_foot[0, 1].legend(loc='best')
ax_foot[0, 1].grid(True)

ax_foot[0, 2].plot(t_cog_ref.values, R_foot_ref['Pr_z'].values, label="R_foot_z_ref", linestyle="--")
# ax_foot[0, 2].plot(t_cog.values, COG_a['COG_ax'].values, label="COG_ddx")
# ax_foot[0, 2].set_xlim(4, 8)
ax_foot[0, 2].set_xlabel("Time [s]")
ax_foot[0, 2].set_ylabel("Position [m]")
ax_foot[0, 2].set_title("R_foot_z")
ax_foot[0, 2].legend(loc='best')
ax_foot[0, 2].grid(True)

ax_foot[1, 0].plot(t_cog_ref.values, L_foot_ref['Pl_x'].values, label="L_foot_x_ref", linestyle="--")
#ax_foot[1, 0].plot(t_cog.values, COG_p['COG_y'].values, label="COG_y")
# ax_foot[1, 0].set_xlim(4, 8)
ax_foot[1, 0].set_xlabel("Time [s]")
ax_foot[1, 0].set_ylabel("Position [m]")
ax_foot[1, 0].set_title("L_foot_x")
ax_foot[1, 0].legend(loc='best')
ax_foot[1, 0].grid(True)

ax_foot[1, 1].plot(t_cog_ref.values, L_foot_ref['Pl_y'].values, label="L_foot_y_ref", linestyle="--")
#ax_foot[1, 1].plot(t_cog.values, COG_v['COG_vy'].values, label="COG_dy")
# ax_foot[1, 1].set_xlim(4, 8)
ax_foot[1, 1].set_xlabel("Time [s]")
ax_foot[1, 1].set_ylabel("Position [m]")
ax_foot[1, 1].set_title("L_foot_y")
ax_foot[1, 1].legend(loc='best')
ax_foot[1, 1].grid(True)

ax_foot[1, 2].plot(t_cog_ref.values, L_foot_ref['Pl_z'].values, label="L_foot_z_ref", linestyle="--")
#ax_foot[1, 2].plot(t_cog.values, COG_a['COG_ay'].values, label="COG_ddy")
# ax_foot[1, 2].set_xlim(4, 8)
ax_foot[1, 2].set_xlabel("Time [s]")
ax_foot[1, 2].set_ylabel("Position [m]")
ax_foot[1, 2].set_title("L_foot_z")
ax_foot[1, 2].legend(loc='best')
ax_foot[1, 2].grid(True)

# グラフをPNGファイルとして保存
plt.tight_layout()
plt.savefig(fig_foot_path)
print("Foot plots saved as 'foot_plots.png'")
plt.show()
