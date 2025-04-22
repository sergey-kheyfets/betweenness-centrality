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
Commands: exact-bc, shortest-paths
~~~
Functionality implemented to date:
1. Reading a graph and finding its centroids.
2. Estimating shortest paths using centroids, outputting some statistics.
3. Using Brandes algorithm to get exact betwenness centrality.