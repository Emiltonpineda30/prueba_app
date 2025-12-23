# Set the output format and file name
set terminal pngcairo size 800,600 enhanced font 'Verdana,10'
set output 'network_performance.png'

# Set titles and labels
set title "Network Performance with Fuzzy Logic Controller"
set xlabel "Time (s)"
set ylabel "Latency (ms)"
set y2label "Adjusted Data Rate (Mbps)"

# Enable the second Y axis
set y2tics

# Set the range for the axes (optional, but good for clarity)
set xrange [2:21]
set yrange [0:*]
set y2range [0:*]

# Add a grid for easier reading
set grid

# Plot the data
# Column 1: Time (s)
# Column 2: Latency (ms)
# Column 4: New Rate (bps)
plot 'results.dat' using 1:2 with linespoints title 'Latency' axis x1y1, \
     'results.dat' using 1:($4/1000000) with linespoints title 'Data Rate' axis x1y2
