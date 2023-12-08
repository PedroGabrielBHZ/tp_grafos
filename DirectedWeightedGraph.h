#include <vector>

class DirectedWeightedGraph
{
private:
    int numVertices;
    std::vector<std::vector<int>> adjacencyMatrix;

public:
    DirectedWeightedGraph(int numVertices);
    void addEdge(int source, int destination, int weight);
    void removeEdge(int source, int destination);
    void printGraph();
};