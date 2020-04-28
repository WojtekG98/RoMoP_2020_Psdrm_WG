#!/usr/bin/gnuplot -persist
set title "Robot's path" font ",14" textcolor rgbcolor "royalblue"
set xrange [0:500]
set yrange [0:500]
set xtics
set ytics
set object 1 circle front at 250,250 size 100 fillcolor rgb "black" lw 1
plot "path.txt" using 1:2 with linespoints
set size square
while (1) {
    replot
    pause 1
}
