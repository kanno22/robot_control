set terminal png 
set datafile separator "\t ,"
set grid#グリッド線　描画
  
file="./log/log.csv"
file2="./log/log2.csv"
file3="./log/log_sensor.csv"

set output "./fig/test.png"
set autoscale x
set autoscale y
set xlabel "Time [sec]"
set ylabel "position [m]"

plot file using 2 title "COG_x" w l lw 2, file using 3 title "COG_y" w l lw 2, file using 4 title "COG_z" w l lw 2

set output "./fig/COG.png"
set autoscale x
set autoscale y
set xlabel "x [m]"
set ylabel "y [m]"

plot file using 2:3 title "COG" w l lw 2,file using 5:6 title "R_ZMP" w l lw 2,file using 8:9 title "L_ZMP" w l lw 2

set output "./fig/foot.png"
set autoscale x
set autoscale y
set xlabel "x [m]"
set ylabel "z [m]"

plot file using 5:7 title "R_z" w l lw 2,file using 8:10 title "L_z" w l lw 2

set output "./fig/foot_xy.png"
set autoscale x
set autoscale y
set xlabel "Time [sec]"
set ylabel "x,y [m]"

plot file using 5 title "R_x" w l lw 2,file using 6 title "R_y" w l lw 2,file using 8 title "L_x" w l lw 2,file using 9 title "L_y" w l lw 2


set output "./fig/COG_v.png"
set autoscale x
set autoscale y
set xlabel "Time [sec]"
set ylabel "velocity [m/s]"

plot file using 11 title "COG_Vx" w l lw 2, file using 12 title "COG_Vy" w l lw 2# set output "./fig/test.png"

set output "./fig/R_ref.png"
set autoscale x
set autoscale y
set xlabel "Time [sec]"
set ylabel "ref [deg],[m]"

plot file2 using 2 title "R_C_yaw" w l lw 2, file2 using 3 title "R_C_roll" w l lw 2, file2 using 4 title "R_C_pitch" w l lw 2, file2 using 5 title "R_Linear" w l lw 2, file2 using 6 title "R_A_pitch" w l lw 2, file2 using 7 title "R_A_roll" w l lw 2

set output "./fig/L_ref.png"
set autoscale x
set autoscale y
set xlabel "Time [sec]"
set ylabel "ref [deg],[m]"

plot file2 using 8 title "L_C_yaw" w l lw 2, file2 using 9 title "L_C_roll" w l lw 2, file2 using 10 title "L_C_pitch" w l lw 2, file2 using 11 title "L_Linear" w l lw 2, file2 using 12 title "L_A_pitch" w l lw 2, file2 using 13 title "L_A_roll" w l lw 2

# set output "./fig/Cq_ref.png"
# set autoscale x
# set autoscale y
# set xlabel "Time [sec]"
# set ylabel "ref [deg]"

# plot file2 using 9 title "L_C_roll_ref" w l lw 2, file3 using 1 title "L_C_roll" w l lw 2

# set output "./fig/Aq_ref.png"
# set autoscale x
# set autoscale y
# set xlabel "Time [sec]"
# set ylabel "ref [deg]"

# plot file2 using 13 title "L_A_roll_ref" w l lw 2, file3 using 2 title "L_A_roll" w l lw 2

# set output "./fig/Linear_ref.png"
# set autoscale x
# set autoscale y
# set xlabel "Time [sec]"
# set ylabel "ref [deg]"

# plot file2 using 11 title "L_Linear_ref" w l lw 2, file3 using 3 title "L_Linear" w l lw 2

# set output "./fig/Current.png"
# set autoscale x
# set autoscale y
# set xlabel "Time [sec]"
# set ylabel "Current [mA]"

# plot file3 using 4 title "L_C_roll_c" w l lw 2, file3 using 5 title "L_A_roll_c" w l lw 2

set output "./fig/COG_BODY.png"
set autoscale x
set autoscale y
set xlabel "x [m]"
set ylabel "y [m]"

plot file using 2:3 title "COG" w l lw 2,file using 5:6 title "R_ZMP" w l lw 2,file using 8:9 title "L_ZMP" w l lw 2,file using 13:14 title "BODY" w l lw 2

set output "./fig/BODY.png"
set autoscale x
set autoscale y
set xlabel "Time [sec]"
set ylabel "position [m]"

plot file using 13 title "BODY_x" w l lw 2, file using 14 title "BODY_y" w l lw 2, file using 15 title "BODY_z" w l lw 2

set output "./fig/Right_angle.png"
set autoscale x
set autoscale y
set xlabel "Time [sec]"
set ylabel "Angle [deg]"

plot file2 using 3 title "R_C_roll_ref" w l lw 2, file2 using 4 title "R_C_pitch_ref" w l lw 2, file2 using 6 title "R_A_pitch_ref" w l lw 2, file2 using 7 title "R_A_roll_ref" w l lw 2, file3 using 1 title "R_C_roll" w l lw 1, file3 using 2 title "R_C_pitch" w l lw 1, file3 using 4 title "R_A_pitch" w l lw 1, file3 using 5 title "R_A_roll" w l lw 1

set output "./fig/Left_angle.png"
set autoscale x
set autoscale y
set xlabel "Time [sec]"
set ylabel "Angle [deg]"

plot file2 using 9 title "L_C_roll_ref" w l lw 2, file2 using 10 title "L_C_pitch_ref" w l lw 2, file2 using 12 title "L_A_pitch_ref" w l lw 2, file2 using 13 title "L_A_roll_ref" w l lw 2, file3 using 6 title "L_C_roll" w l lw 1, file3 using 7 title "L_C_pitch" w l lw 1, file3 using 9 title "L_A_pitch" w l lw 1, file3 using 10 title "L_A_roll" w l lw 1

#mylog_s<<"L_C_roll_angle\t"<<"L_C_pitch_angle\t"<<"L_linear_dis\t"<<"L_A_pitch_angle\t"<<"L_A_roll_angle\t"<<"R_C_roll_angle\t"<<"R_C_pitch_angle\t"<<"R_linear_dis\t"<<"R_A_pitch_angle\t"<<"R_A_roll_angle\t"<<"L_C_roll_current\t"<<"L_C_pitch_current\t"<<"L_A_pitch_current\t"<<"L_A_roll_current\t"<<"R_C_roll_current\t"<<"R_C_pitch_current\t"<<"R_A_pitch_current\t"<<"R_A_roll_current\t"<<endl;

set output "./fig/Right_current.png"
set autoscale x
set autoscale y
set xlabel "Time [sec]"
set ylabel "Current [mA]"

plot file3 using 11 title "R_C_roll" w l lw 2, file3 using 12 title "R_C_pitch" w l lw 2, file3 using 13 title "R_A_pitch" w l lw 2, file3 using 14 title "R_A_roll" w l lw 2

set output "./fig/Left_current.png"
set autoscale x
set autoscale y
set xlabel "Time [sec]"
set ylabel "Current [mA]"

plot file3 using 15 title "L_C_roll" w l lw 2, file3 using 16 title "L_C_pitch" w l lw 2, file3 using 17 title "L_A_pitch" w l lw 2, file3 using 18 title "L_A_roll" w l lw 2

set output "./fig/Linear.png"
set autoscale x
set autoscale y
set xlabel "Time [sec]"
set ylabel "distance [mm]"

plot file2 using 5 title "R_Linear_ref" w l lw 2, file3 using 3 title "R_Linear" w l lw 1,   file2 using 11 title "L_Linear_ref" w l lw 2, file3 using 8 title "L_Linear" w l lw 1

# set autoscale x
# set autoscale y
# set y2tics
# set y2label "Vref [V]"
# set y2range [-4:4]
# p [][] file u 1:($2) title "between BOGIE-link and MOTOR-link" w l lw 2, file u 1:($2-$3) title "between ROCKER-link and BOGIE-link" w l lw 2, file u 1:($6-127.5)/48.8 axis x1y2 title "Vref" w l lw 2

# set output "./fig/linear.png"
# set autoscale x
# set autoscale y
# set xlabel "Time [sec]"
# set ylabel "length [m]"

# plot file2 using 2 title "Rleg_l" w l lw 2, file2 using 3 title "Lleg_l" w l lw 2
