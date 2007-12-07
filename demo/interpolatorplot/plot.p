#
#
set terminal png

set xlabel "time"

set output "cubicx.png"
plot "cubic.dat" using 1:2 title "Cubic spline plot X"
set ylabel "x"

set output "cubicxd.png"
plot "cubic.dat" using 1:3 title "Cubic spline plot Xd"
set ylabel "xd"

set output "cubicxdd.png"
plot "cubic.dat" using 1:4 title "Cubic spline plot Xdd"
set ylabel "xdd"

set output "linex.png"
plot "line.dat" using 1:2 title "Line plot X"
set ylabel "x"

set output "linexd.png"
plot "line.dat" using 1:3 title "Line plot Xd"
set ylabel "xd"

set output "linexdd.png"
plot "line.dat" using 1:4 title "Line plot Xdd"
set ylabel "xdd"


