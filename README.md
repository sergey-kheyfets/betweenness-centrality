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
./betweenness-centrality ../datasets/congress-network/edgelist 
~~~
Functionality implemented to date:
1. Reading a graph and finding its centroids.
2. Estimating shortest paths using centroids, outputting some statistics.
