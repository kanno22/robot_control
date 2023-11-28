set terminal png 
set datafile separator "\t ,"
set grid#グリッド線　描画
  
file="./log/log.csv"

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

set output "./fig/COG_v.png"
set autoscale x
set autoscale y
set xlabel "Time [sec]"
set ylabel "velocity [m/s]"

plot file using 11 title "COG_Vx" w l lw 2, file using 12 title "COG_Vy" w l lw 2# set output "./fig/test.png"
# set autoscale x
# set autoscale y
# set y2tics
# set y2label "Vref [V]"
# set y2range [-4:4]
# p [][] file u 1:($2) title "between BOGIE-link and MOTOR-link" w l lw 2, file u 1:($2-$3) title "between ROCKER-link and BOGIE-link" w l lw 2, file u 1:($6-127.5)/48.8 axis x1y2 title "Vref" w l lw 2
