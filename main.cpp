#include "Edmonds.h"

DirectedWeightedGraph initializeDemoGraph()
{
    // Create a graph with 8 vertices
    DirectedWeightedGraph graph(8);

    // Add edges (source, destination, weight)
    // Leaving root node
    graph.addEdge(0, 1, 5);
    graph.addEdge(0, 7, 11);

    // Leaving node 1
    graph.addEdge(1, 2, 3);
    graph.addEdge(1, 5, 13);

    // Leaving node 2
    graph.addEdge(2, 3, 12);
    graph.addEdge(2, 5, 9);

    // Leaving node 3
    graph.addEdge(3, 4, 1);

    // Leaving node 4
    graph.addEdge(4, 2, 4);

    // Leaving node 5
    graph.addEdge(5, 4, 8);
    graph.addEdge(5, 6, 7);

    // Leaving node 6
    graph.addEdge(6, 1, 2);
    graph.addEdge(6, 7, 10);

    // Leaving node 7
    graph.addEdge(7, 1, 6);

    return graph;
}

DirectedWeightedGraph initializeDAG()
{
    DirectedWeightedGraph graph(3);

    // Add edges (source, destination, weight)
    // Leaving root node
    graph.addEdge(0, 1, 1);
    graph.addEdge(0, 2, 1);

    return graph;
}

int main()
{
    DirectedWeightedGraph graph = initializeDemoGraph();

    // Print the graph
    std::cout << "Graph: " << std::endl;
    graph.printGraph();

    // Get the minimum incoming edges graph
    graph.updateCheapestIncomingEdges(0);

    // Print the minimum incoming edges graph
    std::cout << "Minimum incoming edges graph: " << std::endl;
    graph.printGraph();

    // Find a 0-cost cycle
    std::vector<int> cycle = graph.getCycleNodes();

    // Print the cycle
    std::cout << "Cycle: ";
    for (long unsigned int i = 0; i < cycle.size(); i++)
    {
        std::cout << cycle[i] << " ";
    }
    std::cout << std::endl;

    // Contract the cycle
    DirectedWeightedGraph contractedGraph = graph.contractCycle(cycle);

    // Print the contracted graph
    std::cout << "Contracted graph: " << std::endl;
    contractedGraph.printGraph();

    // Get the minimum incoming edges graph
    contractedGraph.updateCheapestIncomingEdges(0);

    // Print the minimum incoming edges graph
    std::cout << "Minimum incoming edges graph: " << std::endl;
    contractedGraph.printGraph();

    // Find a 0-cost cycle
    std::vector<int> cycle2 = contractedGraph.getCycleNodes();

    // Print the cycle
    std::cout << "Cycle: ";
    for (long unsigned int i = 0; i < cycle2.size(); i++)
    {
        std::cout << cycle2[i] << " ";
    }
    std::cout << std::endl;

    // Contract the cycle
    DirectedWeightedGraph contractedGraph2 = contractedGraph.contractCycle(cycle2);

    // Print the contracted graph
    std::cout << "Contracted graph: " << std::endl;
    contractedGraph2.printGraph();

    return 0;
}
