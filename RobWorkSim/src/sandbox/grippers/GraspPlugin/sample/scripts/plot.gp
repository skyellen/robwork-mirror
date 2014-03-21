reset

set terminal png size 1024,768
set terminal pngcairo dashed size 1024,768
set output 'plot.png'

unset key

set size 1,1
set origin 0,0

set style line 1 lw 1 lt 1 lc rgb '#000000'
set style line 2 lw 1 lt 2 lc rgb '#000000'
set style line 3 lw 1 lt 1 lc rgb '#cccccc'
set style line 4 lw 1 lt 1 lc rgb '#cccccc'

set multiplot layout 4,3 columnsfirst scale 1.1,0.9
set title "length"
plot './length.txt' u 1:2 w l ls 1, './length.txt' u 1:3 w l ls 2, './length.txt' u 1:4 w l ls 3, './length.txt' u 1:5 w l ls 4
set title "width"
plot './width.txt' u 1:2 w l ls 1, './width.txt' u 1:3 w l ls 2, './width.txt' u 1:4 w l ls 3, './width.txt' u 1:5 w l ls 4
set title "depth"
plot './depth.txt' u 1:2 w l ls 1, './depth.txt' u 1:3 w l ls 2, './depth.txt' u 1:4 w l ls 3, './depth.txt' u 1:5 w l ls 4
set title "chfdepth"
plot './chfdepth.txt' u 1:2 w l ls 1, './chfdepth.txt' u 1:3 w l ls 2, './chfdepth.txt' u 1:4 w l ls 3, './chfdepth.txt' u 1:5 w l ls 4
set title "chfangle"
plot './chfangle.txt' u 1:2 w l ls 1, './chfangle.txt' u 1:3 w l ls 2, './chfangle.txt' u 1:4 w l ls 3, './chfangle.txt' u 1:5 w l ls 4
set title "cutdepth"
plot './cutdepth.txt' u 1:2 w l ls 1, './cutdepth.txt' u 1:3 w l ls 2, './cutdepth.txt' u 1:4 w l ls 3, './cutdepth.txt' u 1:5 w l ls 4
set title "cutangle"
plot './cutangle.txt' u 1:2 w l ls 1, './cutangle.txt' u 1:3 w l ls 2, './cutangle.txt' u 1:4 w l ls 3, './cutangle.txt' u 1:5 w l ls 4
set title "tcpoff"
plot './tcpoff.txt' u 1:2 w l ls 1, './tcpoff.txt' u 1:3 w l ls 2, './tcpoff.txt' u 1:4 w l ls 3, './tcpoff.txt' u 1:5 w l ls 4
set title "jawdist"
plot './jawdist.txt' u 1:2 w l ls 1, './jawdist.txt' u 1:3 w l ls 2, './jawdist.txt' u 1:4 w l ls 3, './jawdist.txt' u 1:5 w l ls 4
set title "opening"
plot './opening.txt' u 1:2 w l ls 1, './opening.txt' u 1:3 w l ls 2, './opening.txt' u 1:4 w l ls 3, './opening.txt' u 1:5 w l ls 4
set title "force"
plot './force.txt' u 1:2 w l ls 1, './force.txt' u 1:3 w l ls 2, './force.txt' u 1:4 w l ls 3, './force.txt' u 1:5 w l ls 4
unset multiplot
