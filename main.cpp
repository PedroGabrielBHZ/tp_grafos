#include <iostream>
#include "DirectedWeightedGraph.h"

int main()
{
    // Create a graph with 5 vertices
    DirectedWeightedGraph graph(5);
    graph.addEdge(0, 1, 3);
    graph.printGraph();
    return 0;
}
