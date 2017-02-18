if (!exists("FILENAME")) FILENAME='logs/gnuplot.log'

# Uses line number (uses DATA_NUM last lines in the data file (FILENAME)) as x axis
# and all of the actual sensor values as the y axis.
plot for [i=1:PORT_NUM] '< tail -n '.DATA_NUM.' '.FILENAME using (column(i)) title 'Sensor '.i

# Set the x and y ranges with known max and min values
set xrange [0:DATA_NUM + 1]
set yrange [0:MAX_SENSOR_VALUE]

set title 'XMOS Real Time Data'
set xlabel 'Time Tick'
set ylabel 'Sensor Value'

# Set the size ratio to be the same in all directions
set size square
set style data linespoints

#pause 1 # Pause 1 second
reread
