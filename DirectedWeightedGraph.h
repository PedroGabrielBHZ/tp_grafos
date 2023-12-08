#include <iostream>
#include <tuple>
#include <vector>
#include <algorithm>

class DirectedWeightedGraph
{
private:
    int numVertices;
    std::vector<std::vector<std::pair<int, int>>> adjacencyList;
    std::vector<int> getCycleUtil(int v, std::vector<bool> &visited, std::vector<int> &path);

public:
    DirectedWeightedGraph(int numVertices);
    void addEdge(int source, int destination, int weight);
    void removeEdge(int source, int destination);
    void printGraph();
    int getEdgeWeight(int source, int destination);
    int getNumVertices();

    std::vector<int> getCycleNodes();
    DirectedWeightedGraph contractCycle(std::vector<int> cycle);
    std::vector<std::pair<int, int>> getEdges();
    std::vector<std::tuple<int, int, int>> getNodeIncomingEdges(int node);
    std::vector<std::pair<int, int>> getCycleEdges(std::vector<int> cycle);
    void updateCheapestIncomingEdges(int root);
};