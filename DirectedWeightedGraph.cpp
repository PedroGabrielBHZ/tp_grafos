#include "DirectedWeightedGraph.h"

// Constructor
DirectedWeightedGraph::DirectedWeightedGraph(int numVertices) : numVertices(numVertices)
{
    adjacencyList.resize(numVertices);
}

/**
 * Returns a vector of edges that are incoming to the given node
 * @param node The node to get incoming edges for
 * @return A vector of edges that are incoming to the given node
 * @pre The node must be a valid node in the graph
 */
std::vector<std::tuple<int, int, int>> DirectedWeightedGraph::getNodeIncomingEdges(int node)
{
    std::vector<std::tuple<int, int, int>> incomingEdges;
    for (int i = 0; i < numVertices; i++)
    {
        for (const auto &edge : adjacencyList[i])
        {
            if (edge.first == node)
            {
                incomingEdges.push_back(std::make_tuple(i, edge.first, edge.second));
            }
        }
    }
    return incomingEdges;
}

/**
 * Adds an edge to the graph
 * @param source The source node of the edge
 * @param destination The destination node of the edge
 * @param weight The weight of the edge
 * @pre The source and destination nodes must be valid nodes in the graph
 */
void DirectedWeightedGraph::addEdge(int source, int destination, int weight)
{
    if (source >= 0 && source < numVertices && destination >= 0 && destination < numVertices)
    {
        adjacencyList[source].push_back({destination, weight});
    }
}

/**
 * Returns a vector of edges in the graph
 * @return A vector of edges in the graph
 */
std::vector<std::pair<int, int>> DirectedWeightedGraph::getEdges()
{
    std::vector<std::pair<int, int>> edges;
    for (int i = 0; i < numVertices; i++)
    {
        for (const auto &edge : adjacencyList[i])
        {
            edges.push_back({i, edge.first});
        }
    }
    return edges;
}

/**
 * Returns the number of vertices in the graph
 * @return The number of vertices in the graph
 */
int DirectedWeightedGraph::getNumVertices()
{
    return numVertices;
}

/**
 * Returns the weight of the edge between the given nodes
 * @param source The source node of the edge
 * @param destination The destination node of the edge
 * @return The weight of the edge between the given nodes
 * @pre The source and destination nodes must be valid nodes in the graph
 */
int DirectedWeightedGraph::getEdgeWeight(int source, int destination)
{
    if (source >= 0 && source < numVertices && destination >= 0 && destination < numVertices)
    {
        for (const auto &edge : adjacencyList[source])
        {
            if (edge.first == destination)
            {
                return edge.second;
            }
        }
    }
    return -1;
}

/**
 * Removes an edge from the graph
 * @param source The source node of the edge
 * @param destination The destination node of the edge
 * @pre The source and destination nodes must be valid nodes in the graph
 */
void DirectedWeightedGraph::removeEdge(int source, int destination)
{
    if (source >= 0 && source < numVertices && destination >= 0 && destination < numVertices)
    {
        for (long unsigned int i = 0; i < adjacencyList[source].size(); i++)
        {
            if (adjacencyList[source][i].first == destination)
            {
                adjacencyList[source].erase(adjacencyList[source].begin() + i);
                break;
            }
        }
    }
}

/**
 * Returns a vector of nodes that form a cycle in the graph
 * @param v The current node
 * @param visited A vector of booleans representing whether a node has been visited
 * @param path A vector of nodes that form a path from the root node to the current node
 * @return A vector of nodes that form a cycle in the graph
 */
std::vector<int> DirectedWeightedGraph::getCycleUtil(int v, std::vector<bool> &visited, std::vector<int> &path)
{
    visited[v] = true;
    path.push_back(v);

    for (const auto &edge : adjacencyList[v])
    {
        // Skip edges with a weight not equal to 0
        if (edge.second != 0)
            continue;

        if (!visited[edge.first])
        {
            std::vector<int> cycle = getCycleUtil(edge.first, visited, path);
            if (!cycle.empty())
                return cycle;
        }
        else
        {
            // If this node is in the current path, we have found a cycle.
            auto it = std::find(path.begin(), path.end(), edge.first);
            if (it != path.end())
                return std::vector<int>(it, path.end());
        }
    }

    path.pop_back();
    return std::vector<int>();
}

/**
 * Returns a vector of nodes that form a cycle in the graph
 * @return A vector of nodes that form a cycle in the graph
 */
std::vector<int> DirectedWeightedGraph::getCycleNodes()
{
    std::vector<bool> visited(numVertices, false);
    std::vector<int> path;

    for (int i = 0; i < numVertices; i++)
    {
        std::vector<int> cycle = getCycleUtil(i, visited, path);
        if (!cycle.empty())
            return cycle;
    }

    // No cycle found, return empty vector
    return std::vector<int>();
}

/**
 * Returns a vector of edges that form a cycle in the graph
 * @param cycle A vector of nodes that form a cycle in the graph
 * @return A vector of edges that form a cycle in the graph
 */
std::vector<std::pair<int, int>> DirectedWeightedGraph::getCycleEdges(std::vector<int> cycle)
{
    std::vector<std::pair<int, int>> cycleEdges;
    for (long unsigned int i = 0; i < cycle.size() - 1; i++)
    {
        for (const auto &edge : adjacencyList[cycle[i]])
        {
            if (edge.first == cycle[i + 1])
            {
                cycleEdges.push_back({cycle[i], cycle[i + 1]});
                break;
            }
        }
    }
    // Add an edge from the last node in the cycle to the first node in the cycle
    cycleEdges.push_back({cycle[cycle.size() - 1], cycle[0]});
    return cycleEdges;
}

/**
 * Create a graph of cheapest incoming edges to each node in the given graph, excluding the given root node
 * @param root The root node of the graph
 * @return A graph of cheapest incoming edges to each node in the given graph, excluding the given root node
 */
void DirectedWeightedGraph::updateCheapestIncomingEdges(int root)
{
    // For each node in the graph, find the cheapest incoming edge
    for (int i = 0; i < numVertices; i++)
    {
        // Skip the root node
        if (i == root)
            continue;

        // Get all incoming edges to this node
        std::vector<std::tuple<int, int, int>> incomingEdges = getNodeIncomingEdges(i);

        // If no incoming edges, skip
        if (incomingEdges.empty())
            continue;

        // Find the cheapest incoming edge
        std::tuple<int, int, int> cheapestIncomingEdge = incomingEdges[0];
        for (const auto &edge : incomingEdges)
        {
            if (std::get<2>(edge) < std::get<2>(cheapestIncomingEdge))
            {
                cheapestIncomingEdge = edge;
            }
        }

        // Subtract the weight of the cheapest incoming edge from all incoming edges to this node in the adjacency list
        for (const auto &edge : incomingEdges)
        {
            // Subtract the weight of the cheapest incoming edge from this edge
            removeEdge(std::get<0>(edge), std::get<1>(edge));
            addEdge(std::get<0>(edge), std::get<1>(edge), std::get<2>(edge) - std::get<2>(cheapestIncomingEdge));
        }
    }
}

/**
 * Contract the given cycle into a single node, as described in the paper by Edmonds.
 * @param cycle A vector of nodes that form a cycle in the graph
 * @return The contracted graph
 */
DirectedWeightedGraph DirectedWeightedGraph::contractCycle(std::vector<int> cycle)
{
    // Create a new graph with one extra node which will be the contracted cycle
    DirectedWeightedGraph contractedGraph(numVertices + 1);

    // Get the edges in the cycle
    std::vector<std::pair<int, int>> cycleEdges = getCycleEdges(cycle);

    // Get all edges that are incoming to the cycle
    std::vector<std::tuple<int, int, int>> incomingEdgesToCycle;
    for (const auto &node : cycle)
    {
        // Get every edge that is incoming to this node
        std::vector<std::tuple<int, int, int>> incomingEdges = getNodeIncomingEdges(node);

        // Remove the edges that have both nodes in the cycle
        incomingEdges.erase(std::remove_if(incomingEdges.begin(), incomingEdges.end(), [&cycle](const std::tuple<int, int, int> &incomingEdge)
                                           { return std::find(cycle.begin(), cycle.end(), std::get<0>(incomingEdge)) != cycle.end() &&
                                                    std::find(cycle.begin(), cycle.end(), std::get<1>(incomingEdge)) != cycle.end(); }),
                            incomingEdges.end());

        // Add the remaining edges incoming to this node to the incoming edges to the cycle
        incomingEdgesToCycle.insert(incomingEdgesToCycle.end(), incomingEdges.begin(), incomingEdges.end());
    }

    // Get all edges that are outgoing from the cycle
    std::vector<std::tuple<int, int, int>> outgoingEdgesFromCycle;
    for (const auto &edge : cycleEdges)
    {
        for (const auto &outgoingEdge : adjacencyList[edge.second])
        {
            // If the outgoing edge is not in the cycle, add it to the outgoing edges from the cycle
            if (std::find(cycle.begin(), cycle.end(), outgoingEdge.first) == cycle.end())
            {
                outgoingEdgesFromCycle.push_back(std::make_tuple(edge.second, outgoingEdge.first, outgoingEdge.second));
            }
        }
    }

    // Set the incoming edges to the cycle as incoming to the new node
    for (const auto &edge : incomingEdgesToCycle)
    {
        contractedGraph.addEdge(std::get<0>(edge), numVertices, std::get<2>(edge));
    }

    // Set the outgoing edges from the cycle as outgoing from the new node
    for (const auto &edge : outgoingEdgesFromCycle)
    {
        contractedGraph.addEdge(numVertices, std::get<1>(edge), std::get<2>(edge));
    }

    // Add all edges that are not in the cycle to the contracted graph
    for (int i = 0; i < numVertices; i++)
    {
        for (const auto &edge : adjacencyList[i])
        {
            // If one of the nodes in the edge is in the cycle, don't add it to the contracted graph
            if (std::find(cycle.begin(), cycle.end(), i) == cycle.end() &&
                std::find(cycle.begin(), cycle.end(), edge.first) == cycle.end())
            {
                contractedGraph.addEdge(i, edge.first, edge.second);
            }
        }
    }

    return contractedGraph;
}

/**
 * Prints the graph to the console
 */
void DirectedWeightedGraph::printGraph()
{
    for (int i = 0; i < numVertices; i++)
    {
        std::cout << "Vertex " << i << ": ";
        for (const auto &edge : adjacencyList[i])
        {
            std::cout << "(" << edge.first << ", " << edge.second << ") ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}