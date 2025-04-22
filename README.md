# Betweenness centrality estimator

## Build
~~~
mkdir build
cmake ..
cmake --build .
~~~

## Use
Sample graph datasets are included in ```./datasets```.
~~~
Usage: ./betweenness-centrality <dataset_filename> <command> <...parameters>
Commands: exact-bc, shortest-paths, estimate-bc

Parameters:
- shortest-path: <num_samples> (default is 100)
- estimate-bc: <num_centroids> (default is based on the graph diameter)
~~~
Functionality implemented to date:
1. Reading a graph and finding its centroids.
2. Estimating shortest paths using centroids, outputting some statistics.
3. Using Brandes algorithm to get exact betwenness centrality.