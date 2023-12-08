#include <vector>
#include "DirectedWeightedGraph.h"

class Edmonds
{
private:
    int root;
    DirectedWeightedGraph graph;

public:
    Edmonds(int root, DirectedWeightedGraph graph);
    DirectedWeightedGraph findMinimumSpanningArborescence();
};