if (!exists("FILENAME")) FILENAME='logs/gnuplot.log'

# Uses line number (uses DATA_NUM last lines in the data file (FILENAME)) as x axis
# and all of the actual sensor values as the y axis.
# plot for [i=1:PORT_NUM] '< tail -n '.DATA_NUM.' '.FILENAME using (column(i)) title 'Sensor '.i


# SIDE_LEN = ceil(sqrt(PORT_NUM * PORT_NUM))
#
# set multiplot layout SIDE_LEN,SIDE_LEN rowsfirst

set xrange [0:DATA_NUM + 1]
# Set the size ratio to be the same in all directions
set size square
# set style data linespoints
set style line 1 lc rgb 'black' pt 3

# set term wxt 0 persist
# set yrange [-30:40]
# set title 'Yaw'
# set xlabel 'Time Tick'
# set ylabel 'Angle (deg)'
# plot '< tail -n'.DATA_NUM.' '.FILENAME using (column(1)) title 'Yaw' with linespoints pt 7
#
# set term wxt 1 persist
# set title 'Pitch'
# set xlabel 'Time Tick'
# set ylabel 'Angle (deg)'
# set yrange [-60:60]
# plot '< tail -n'.DATA_NUM.' '.FILENAME using (column(2)) title 'Pitch' with linespoints pt 7
#
# set term wxt 2 persist
# set title 'Hypotenuse'
# set xlabel 'Time Tick'
# set ylabel 'Distance (in)'
# set yrange [0:200]
# plot '< tail -n'.DATA_NUM.' '.FILENAME using (column(3)) title 'Hypotenuse' with linespoints pt 7
#
set term wxt 3 persist
set title 'Angular Displacement'
set xlabel 'Time Tick'
set ylabel 'Angle (deg)'
set yrange [-80:80]
plot '< tail -n'.DATA_NUM.' '.FILENAME using 0:3 title 'Angular Displacement' with linespoints pt 7, \
     '' using 0:3:(sprintf("%.1f",$3)) with labels nopoint offset char 0,1 font '6' notitle, \
     # '' using 0:3:(sprintf("%.1f",$3)) every 2::1 with labels nopoint offset char 0,-2 font '6' notitle

# set title 'XMOS Real Time Data'
# set xlabel 'Time Tick'
# set ylabel 'Sensor Value'

# Set the size ratio to be the same in all directions
# set size square
# set style data linespoints

#pause 1 # Pause 1 second
reread
