if (!exists("FILENAME")) FILENAME='logs/gnuplot.log'

# Uses line number (uses DATA_NUM last lines in the data file (FILENAME)) as x axis
# and all of the actual sensor values as the y axis.
# plot for [i=1:PORT_NUM] '< tail -n '.DATA_NUM.' '.FILENAME using (column(i)) title 'Sensor '.i

if (DIR != '') {
    system ("rm -rf images/gnuplot/".DIR)
    system ("mkdir images/gnuplot/".DIR)
}

time_tick = 0
while (1) {
    set xrange [0:DATA_NUM + 1]
    # Set the size ratio to be the same in all directions
    set size square
    # set style data linespoints
    set style line 1 lc rgb 'black' pt 3

    # Automatically update the last data point and replot
    if (DIR != '') {
        set term png size 1500,1500
        set grid
        set output sprintf('images/gnuplot/%s/vision_data%05.0f.png', DIR, time_tick)
        time_tick = time_tick + 1;
    }
    set title 'Angular Displacement'
    set xlabel 'Time Tick'
    set ylabel 'Angle (deg)'
    set yrange [-80:80]
    plot '< tail -n'.DATA_NUM.' '.FILENAME using 0:3 title 'Euclidean Distance' with linespoints pt 7, \
         '' using 0:3:(sprintf("%.2f",$3)) with labels nopoint offset char 0,1 font '6' notitle, \
         '< tail -n'.DATA_NUM.' '.FILENAME using 0:4 title 'Theta' with linespoints pt 7, \
         '' using 0:4:(sprintf("%.2f",$4)) with labels nopoint offset char 0,1 font '6' notitle
    # plot '< tail -n'.DATA_NUM.' '.FILENAME using 0:2 title 'Phi' with linespoints pt 7, \
    #      '' using 0:2:(sprintf("%.2f",$2)) with labels nopoint offset char 0,1 font '6' notitle, \
         # '< tail -n'.DATA_NUM.' '.FILENAME using 0:3 title 'Beta' with linespoints pt 7, \
         # '' using 0:3:(sprintf("%.2f",$3)) with labels nopoint offset char 0,1 font '6' notitle, \

    set term wxt 0 persist size 1500,1500
    set output
    replot
}
