#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <cctype> // For toupper()

using namespace std;

// Define a structure to represent an edge in the graph
struct Edge {
    int to, weight;
};

// Define the graph as an adjacency list
vector<vector<Edge>> graph;

// Dijkstra's algorithm to find the shortest path
pair<int, vector<int>> dijkstra(int start, int end) {
    int n = graph.size();
    vector<int> dist(n, numeric_limits<int>::max()); // Distance to each node
    vector<int> prev(n, -1); // Previous node in the optimal path
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq; // Min-heap priority queue

    // Initialize the start node
    dist[start] = 0;
    pq.push({0, start});

    while (!pq.empty()) {
        int currentDist = pq.top().first;
        int currentNode = pq.top().second;
        pq.pop();

        // If the current node is the target node, stop
        if (currentNode == end) break;

        // If a shorter path to currentNode has been found
        if (currentDist > dist[currentNode]) continue;

        // Visit each neighbor of the current node
        for (const Edge& edge : graph[currentNode]) {
            int neighbor = edge.to;
            int weight = edge.weight;

            int newDist = currentDist + weight;
            if (newDist < dist[neighbor]) {
                dist[neighbor] = newDist;
                prev[neighbor] = currentNode;
                pq.push({newDist, neighbor});
            }
        }
    }

    // Reconstruct the shortest path
    vector<int> path;
    for (int at = end; at != -1; at = prev[at]) {
        path.push_back(at);
    }
    reverse(path.begin(), path.end());

    return {dist[end], path};
}

// Helper function to convert node names to indices
int getNodeIndex(char node) {
    return toupper(node) - 'A'; // Ensure the character is uppercase
}

int main() {
    // Initialize the graph with 5 nodes
    graph.resize(5);

    // Add edges to the graph
    // A -> B (10), A -> E (3)
    graph[getNodeIndex('A')].push_back({getNodeIndex('B'), 10});
    graph[getNodeIndex('A')].push_back({getNodeIndex('E'), 3});
    
    // B -> C (2), B -> E (4)
    graph[getNodeIndex('B')].push_back({getNodeIndex('C'), 2});
    graph[getNodeIndex('B')].push_back({getNodeIndex('E'), 4});
    
    // C -> D (9), C -> E (8)
    graph[getNodeIndex('C')].push_back({getNodeIndex('D'), 9});
    graph[getNodeIndex('C')].push_back({getNodeIndex('E'), 8});
    
    // D -> C (7), D -> E (2)
    graph[getNodeIndex('D')].push_back({getNodeIndex('C'), 7});
    graph[getNodeIndex('D')].push_back({getNodeIndex('E'), 2});
    
    // E -> B (1), E -> C (8)
    graph[getNodeIndex('E')].push_back({getNodeIndex('B'), 1});
    graph[getNodeIndex('E')].push_back({getNodeIndex('C'), 8});

    // Take user input for the start and end nodes
    char startNode, endNode;
    cout << "Enter the start node (A-E): ";
    cin >> startNode;
    cout << "Enter the end node (A-E): ";
    cin >> endNode;

    // Convert to uppercase to handle case sensitivity
    startNode = toupper(startNode);
    endNode = toupper(endNode);

    // Get the indices of the start and end nodes
    int start = getNodeIndex(startNode);
    int end = getNodeIndex(endNode);

    // Run Dijkstra's algorithm
    pair<int, vector<int>> result = dijkstra(start, end);
    int shortestDistance = result.first;
    vector<int> path = result.second;

    // Output the result
    if (shortestDistance == numeric_limits<int>::max()) {
        cout << "No path from " << startNode << " to " << endNode << endl;
    } else {
        cout << "Shortest path cost from " << startNode << " to " << endNode << ": " << shortestDistance << endl;
        cout << "Path: ";
        for (int node : path) {
            cout << char(node + 'A') << " ";
        }
        cout << endl;
    }

    return 0;
}
