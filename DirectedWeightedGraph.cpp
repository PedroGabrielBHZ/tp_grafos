#include <iostream>
#include "DirectedWeightedGraph.h"

DirectedWeightedGraph::DirectedWeightedGraph(int numVertices) : numVertices(numVertices)
{
    adjacencyMatrix.resize(numVertices, std::vector<int>(numVertices, 0));
}

void DirectedWeightedGraph::addEdge(int source, int destination, int weight)
{
    if (source >= 0 && source < numVertices && destination >= 0 && destination < numVertices)
    {
        adjacencyMatrix[source][destination] = weight;
    }
}

void DirectedWeightedGraph::removeEdge(int source, int destination)
{
    if (source >= 0 && source < numVertices && destination >= 0 && destination < numVertices)
    {
        adjacencyMatrix[source][destination] = 0;
    }
}

void DirectedWeightedGraph::printGraph()
{
    for (int i = 0; i < numVertices; i++)
    {
        for (int j = 0; j < numVertices; j++)
        {
            std::cout << adjacencyMatrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
}