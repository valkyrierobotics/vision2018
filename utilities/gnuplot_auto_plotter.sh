#! /bin/bash
# Gnuplot utility to watch and auto update a plot of a specified formatted data file.
# Relies on gnuplot being installed.
# Written by Min Hoo Lee on September 18, 2016

# Default values
filename=logs/gnuplot.log
dir=''
number=5000
channels=8
max=3000

echo ""

usage()
{
    echo "Usage: $0 [-f filename] [-d dir] [-n number] [-c channels] [-m max]";
    echo ""
    echo "    -f filename :   Name of the text file to plot data from";
    echo "    -d dir      :   Name of the directory to save graph to";
    echo "    -c channels :   Number of channels (separate sensors) to plot";
    echo "    -n x_max    :   Maximum value of the x axis (number of data points)";
    echo "    -m y_max    :   Maximum value of the y axis (sensor values)";
    echo ""
}

check_file_exist()
{
    if [ ! -f "$1" ]; then
        echo "File '$1' does not exist"
        exit 1
    fi
}

parse_opts()
{
    local opt

    while getopts "f:d:n:c:m:h" opt; do
        case $opt in
            f)
                filename=$OPTARG
                ;;
            d)
                dir=$OPTARG
                ;;
            n)
                number=$OPTARG
                ;;
            c)  channels=$OPTARG
                ;;
            m)
                max=$OPTARG
                ;;
            h)
                usage
                exit 0
                ;;
            \?)
                echo "Invalid option: -$OPTARG" >&2
                exit 1
                ;;
            :)
                echo "-$OPTARG requires an argument." >&2
                exit 1
                ;;
        esac
    done

    return $OPTIND
}

parse_opts "$@"

echo "Using the following options:"
echo "Log file to read from: $filename"
if [[ $dir != '' ]]; then
    echo "Saving graph to images/gnuplot/$dir"
fi
echo "X axis (number of time ticks): [0:$number]"
echo "Y axis (maximum sensor value): [0:$max]"
echo "Number of channels (sensors): $channels"
echo ""

echo "Kill existing instances of gnuplot with \`killall gnuplot\`"
echo "Starting gnuplot autoplotter instance"
echo ""

# Temporary solution for creating separate graphs for each column in the data file for vision data
if [[ $filename == "logs/processed_data.log" ]]; then
    gnuplot -e "FILENAME='$filename'; DATA_NUM='$number'; PORT_NUM='$channels'; MAX_SENSOR_VALUE='$max'; DIR='$dir'" utilities/gnuplot_vision_data.plt
else
    gnuplot -e "FILENAME='$filename'; DATA_NUM='$number'; PORT_NUM='$channels'; MAX_SENSOR_VALUE='$max'; DIR='$dir'" utilities/gnuplot_fps.plt
fi
