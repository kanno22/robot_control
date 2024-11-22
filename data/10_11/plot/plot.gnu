set terminal png 
set datafile separator "\t ,"
set grid#グリッド線　描画
  
file="./log/log_24.csv"
file2="./log/log_26.csv"

set output "./fig/test.png"
set xrange [15:30] 
set yrange [-3:3]
set xlabel "Time [sec]"
set ylabel "position [m]"

plot file using 1:2 title "BODY_ddx" w l lw 2, file2 using 1:2 title "BODY_ddx" w l lw 2

set output "./fig/deg.png"
set autoscale x
set autoscale y
set xlabel "Time [sec]"
set ylabel "Deg [rad]"

plot file using 1:5 title "BODY_yaw" w l lw 2, file using 1:6 title "BODY_roll" w l lw 2, file using 1:7 title "BODY_pitch" w l lw 2
