set terminal png
set output "@PNGFILE"
set title "@TITLE"
set xlabel "Time (ms)"
set ylabel "Degree"
set grid xtics mxtics ytics mytics
plot "@DATAFILE" using 1:6 w lp ps 0 ti ""