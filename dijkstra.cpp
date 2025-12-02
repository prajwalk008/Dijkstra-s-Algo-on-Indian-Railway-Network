#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <algorithm>
#include <map>
#include <unordered_map>
#include <set>
#include <string>
#include <fstream>
#include <iomanip>
#include <sstream>

using namespace std;

// Structure to represent an edge with string destination and weight
struct Edge {
    string destination;
    int weight;
    
    Edge(const string& dest, int w) : destination(dest), weight(w) {}
};

// Structure to hold Dijkstra results (distances and previous vertices for path reconstruction)
struct DijkstraResult {
    map<string, int> distances;
    map<string, string> previous;  // previous[vertex] = previous vertex in shortest path
};

// Classic Dijkstra's Algorithm with string-based vertices
DijkstraResult dijkstra(const map<string, vector<Edge>>& graph, const string& source) {
    DijkstraResult result;
    map<string, int>& distances = result.distances;
    map<string, string>& previous = result.previous;
    set<string> visited;
    
    // Initialize all distances to INFINITY
    for (const auto& pair : graph) {
        distances[pair.first] = INT_MAX;
    }
    
    // Priority queue: (distance, vertex)
    // Using greater<pair<int,string>> to make it a min-heap
    priority_queue<pair<int, string>, vector<pair<int, string>>, greater<pair<int, string>>> pq;
    
    // Initialize source vertex
    distances[source] = 0;
    previous[source] = "";  // Source has no previous vertex
    pq.push({0, source});
    
    while (!pq.empty()) {
        // Extract vertex with minimum distance
        string u = pq.top().second;
        int dist_u = pq.top().first;
        pq.pop();
        
        // Skip if already processed
        if (visited.find(u) != visited.end()) {
            continue;
        }
        
        visited.insert(u);
        
        // Relax all edges from u
        // Check if vertex exists in graph
        auto it = graph.find(u);
        if (it != graph.end()) {
            for (const Edge& edge : it->second) {
                string v = edge.destination;
                int weight = edge.weight;
                
                // Initialize distance for destination if not already present
                if (distances.find(v) == distances.end()) {
                    distances[v] = INT_MAX;
                }
                
                if (visited.find(v) == visited.end() && distances[u] != INT_MAX && 
                    distances[u] + weight < distances[v]) {
                    distances[v] = distances[u] + weight;
                    previous[v] = u;  // Record previous vertex for path reconstruction
                    pq.push({distances[v], v});
                }
            }
        }
    }
    
    return result;
}

// Function to reconstruct and print the shortest path from source to destination
void printShortestPath(const DijkstraResult& result, const string& source, const string& destination) {
    const map<string, int>& distances = result.distances;
    const map<string, string>& previous = result.previous;
    
    // Check if destination exists in distances map
    if (distances.find(destination) == distances.end()) {
        cout << "Error: Destination vertex \"" << destination << "\" not found in graph." << endl;
        return;
    }
    
    // Check if path exists
    if (distances.at(destination) == INT_MAX) {
        cout << "No path exists from \"" << source << "\" to \"" << destination << "\"." << endl;
        return;
    }
    
    // Reconstruct path
    vector<string> path;
    string current = destination;
    
    while (current != "") {
        path.push_back(current);
        auto it = previous.find(current);
        if (it != previous.end() && it->second != "") {
            current = it->second;
        } else {
            break;
        }
    }
    
    // Reverse path to get source -> destination order
    reverse(path.begin(), path.end());
    
    // Print path
    cout << "Shortest path from \"" << source << "\" to \"" << destination << "\":" << endl;
    cout << "Path: ";
    for (size_t i = 0; i < path.size(); i++) {
        cout << path[i];
        if (i < path.size() - 1) {
            cout << " -> ";
        }
    }
    cout << endl;
    cout << "Distance: " << distances.at(destination) << endl;
}

// Structure to store a path with its distance
struct PathInfo {
    vector<string> path;
    int distance;
    
    PathInfo(const vector<string>& p, int d) : path(p), distance(d) {}
};

// DFS function to find all paths from source to destination within threshold
void findAllPaths(const map<string, vector<Edge>>& graph, const string& current, 
                  const string& destination, vector<string>& currentPath, 
                  int currentDistance, int threshold, set<vector<string>>& foundPaths,
                  map<vector<string>, int>& pathDistances) {
    // Add current vertex to path
    currentPath.push_back(current);
    
    // If reached destination
    if (current == destination) {
        foundPaths.insert(currentPath);
        pathDistances[currentPath] = currentDistance;
        currentPath.pop_back();
        return;
    }
    
    // If exceeded threshold, prune this branch
    if (currentDistance > threshold) {
        currentPath.pop_back();
        return;
    }
    
    // Explore neighbors
    auto it = graph.find(current);
    if (it != graph.end()) {
        for (const Edge& edge : it->second) {
            string next = edge.destination;
            
            // Avoid cycles (don't visit same vertex twice in a path)
            bool alreadyVisited = false;
            for (const string& v : currentPath) {
                if (v == next) {
                    alreadyVisited = true;
                    break;
                }
            }
            
            if (!alreadyVisited) {
                findAllPaths(graph, next, destination, currentPath, 
                           currentDistance + edge.weight, threshold, 
                           foundPaths, pathDistances);
            }
        }
    }
    
    // Backtrack
    currentPath.pop_back();
}

// Function to find and print alternative paths
void findAlternativePaths(const map<string, vector<Edge>>& graph, 
                         const string& source, const string& destination, 
                         int shortestDistance) {
    if (shortestDistance == INT_MAX) {
        cout << "No shortest path exists, cannot find alternatives." << endl;
        return;
    }
    
    // Threshold: 100% longer means 200% of shortest distance
    int threshold = shortestDistance * 2;
    
    set<vector<string>> foundPaths;
    map<vector<string>, int> pathDistances;
    vector<string> currentPath;
    
    cout << "\n=== Alternative Paths (within 100% of shortest path) ===" << endl;
    cout << "Shortest path distance: " << shortestDistance << endl;
    cout << "Maximum allowed distance: " << threshold << endl;
    cout << endl;
    
    // Find all paths
    findAllPaths(graph, source, destination, currentPath, 0, threshold, foundPaths, pathDistances);
    
    if (foundPaths.empty()) {
        cout << "No alternative paths found within the threshold." << endl;
        return;
    }
    
    // Sort paths by distance
    vector<pair<vector<string>, int>> sortedPaths;
    for (const auto& path : foundPaths) {
        sortedPaths.push_back({path, pathDistances[path]});
    }
    sort(sortedPaths.begin(), sortedPaths.end(), 
         [](const pair<vector<string>, int>& a, const pair<vector<string>, int>& b) {
             return a.second < b.second;
         });
    
    // Print up to 10 shortest paths (or all if fewer than 10)
    size_t maxPathsToShow = min<size_t>(10, sortedPaths.size());
    int pathNum = 1;
    for (size_t idx = 0; idx < maxPathsToShow; idx++) {
        const auto& pathInfo = sortedPaths[idx];
        const vector<string>& path = pathInfo.first;
        int distance = pathInfo.second;
        
        // Calculate percentage increase
        double percentageIncrease = ((double)(distance - shortestDistance) / shortestDistance) * 100.0;
        
        cout << "Path #" << pathNum << " (";
        if (distance == shortestDistance) {
            cout << "SHORTEST";
        } else {
            cout << "+" << fixed << setprecision(1) << percentageIncrease << "% longer";
        }
        cout << ", Distance: " << distance << "):" << endl;
        cout << "  ";
        for (size_t i = 0; i < path.size(); i++) {
            cout << path[i];
            if (i < path.size() - 1) {
                cout << " -> ";
            }
        }
        cout << endl << endl;
        pathNum++;
    }
    
    cout << "Displayed " << maxPathsToShow << " path(s)";
    if (sortedPaths.size() > maxPathsToShow) {
        cout << " out of " << sortedPaths.size() << " paths within the threshold";
    }
    cout << "." << endl;
}

// Function to print graph in a readable text format
void printGraph(const map<string, vector<Edge>>& graph) {
    cout << "\n=== Graph Visualization (Text Format) ===" << endl;
    for (const auto& pair : graph) {
        cout << "\n" << pair.first << " -> ";
        for (size_t i = 0; i < pair.second.size(); i++) {
            cout << pair.second[i].destination << "(" << pair.second[i].weight << ")";
            if (i < pair.second.size() - 1) {
                cout << ", ";
            }
        }
    }
    cout << "\n" << endl;
}

// Function to generate Graphviz DOT file for visualization
void generateDotFile(const map<string, vector<Edge>>& graph, const string& filename = "graph.dot", 
                     const string& layout = "LR", const string& engine = "dot") {
    ofstream file(filename);
    if (!file.is_open()) {
        cerr << "Error: Could not create file " << filename << endl;
        return;
    }
    
    // Find min and max weights for scaling
    int minWeight = INT_MAX;
    int maxWeight = 0;
    for (const auto& pair : graph) {
        for (const Edge& edge : pair.second) {
            if (edge.weight < minWeight) minWeight = edge.weight;
            if (edge.weight > maxWeight) maxWeight = edge.weight;
        }
    }
    
    // Calculate scaling factor: map weights to edge lengths (1.0 to 5.0 inches)
    double scaleFactor = 1.0;
    if (maxWeight > minWeight) {
        scaleFactor = 4.0 / (maxWeight - minWeight);  // Scale to 1-5 inch range
    }
    
    file << "digraph G {\n";
    
    // For engines that support edge lengths (neato, fdp), use len attribute
    if (engine == "neato" || engine == "fdp") {
        file << "    layout=" << engine << ";\n";
        file << "    overlap=false;\n";
        file << "    splines=true;\n";
        file << "    node [shape=circle, style=filled, fillcolor=lightblue];\n";
        file << "    edge [fontsize=10];\n\n";
        
        // Write all edges with len attribute proportional to weight
        for (const auto& pair : graph) {
            for (const Edge& edge : pair.second) {
                // Calculate edge length: minimum 1.0, scaled by weight
                double edgeLen = 1.0 + (edge.weight - minWeight) * scaleFactor;
                file << "    \"" << pair.first << "\" -> \"" << edge.destination 
                     << "\" [label=\"" << edge.weight << "\", len=" << edgeLen << "];\n";
            }
        }
    } else {
        // For dot engine, use rankdir but note that edge lengths won't be proportional
        file << "    rankdir=" << layout << ";\n";
        file << "    node [shape=circle, style=filled, fillcolor=lightblue];\n";
        file << "    edge [fontsize=10];\n\n";
        
        // Write all edges
        for (const auto& pair : graph) {
            for (const Edge& edge : pair.second) {
                file << "    \"" << pair.first << "\" -> \"" << edge.destination 
                     << "\" [label=\"" << edge.weight << "\"];\n";
            }
        }
    }
    
    file << "}\n";
    file.close();
    cout << "Graphviz DOT file generated: " << filename << endl;
    if (engine == "neato" || engine == "fdp") {
        cout << "Edge lengths are proportional to weights (min=" << minWeight 
             << ", max=" << maxWeight << ")" << endl;
        cout << "To visualize, run: " << engine << " -Tpng " << filename << " -o graph.png" << endl;
    } else {
        cout << "Note: For proportional edge lengths, use 'neato' or 'fdp' engine" << endl;
        cout << "Layout: " << layout << " (LR=Left-Right, TB=Top-Bottom, BT=Bottom-Top, RL=Right-Left)" << endl;
        cout << "To visualize, run: " << engine << " -Tpng " << filename << " -o graph.png" << endl;
    }
    cout << "Available engines: dot (hierarchical), neato (spring, supports edge lengths), fdp (force-directed, supports edge lengths), circo (circular)" << endl;
}

// Function to print graph statistics
void printGraphStats(const map<string, vector<Edge>>& graph) {
    int totalVertices = 0;
    int totalEdges = 0;
    set<string> allVertices;
    
    // Count vertices and edges
    for (const auto& pair : graph) {
        allVertices.insert(pair.first);
        totalEdges += pair.second.size();
        for (const Edge& edge : pair.second) {
            allVertices.insert(edge.destination);
        }
    }
    totalVertices = allVertices.size();
    
    cout << "\n=== Graph Statistics ===" << endl;
    cout << "Total Vertices: " << totalVertices << endl;
    cout << "Total Edges: " << totalEdges << endl;
    cout << "Vertices: ";
    for (const string& v : allVertices) {
        cout << v << " ";
    }
    cout << "\n" << endl;
}

// Function to load graph from file
// File format: source destination weight (one edge per line)
// Lines starting with # are treated as comments
bool loadGraphFromFile(map<string, vector<Edge>>& graph, const string& filename = "graph_data.txt") {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error: Could not open file \"" << filename << "\"" << endl;
        cerr << "Please make sure the file exists in the same directory as the executable." << endl;
        return false;
    }
    
    string line;
    int lineNumber = 0;
    int edgesLoaded = 0;
    
    while (getline(file, line)) {
        lineNumber++;
        
        // Trim whitespace
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);
        
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        // Parse line: source destination weight
        istringstream iss(line);
        string source, destination;
        int weight;
        
        if (iss >> source >> destination >> weight) {
            // Validate weight is positive
            if (weight <= 0) {
                cerr << "Warning: Line " << lineNumber << " has invalid weight (" << weight 
                     << "). Skipping." << endl;
                continue;
            }
            
            graph[source].push_back(Edge(destination, weight));
            edgesLoaded++;
        } else {
            cerr << "Warning: Line " << lineNumber << " has invalid format. Skipping." << endl;
            cerr << "  Expected format: source destination weight" << endl;
            cerr << "  Got: " << line << endl;
        }
    }
    
    file.close();
    
    if (edgesLoaded == 0) {
        cerr << "Error: No valid edges found in file." << endl;
        return false;
    }
    
    cout << "Successfully loaded " << edgesLoaded << " edges from \"" << filename << "\"" << endl;
    return true;
}

int main() {
    // Graph representation: map from string (vertex name) to list of edges
    map<string, vector<Edge>> graph;
    
    // Load graph from file
    if (!loadGraphFromFile(graph)) {
        cerr << "Failed to load graph data. Exiting." << endl;
        return 1;
    }
    
    // Menu for visualization
    int choice;
    cout << "\n=== Graph Visualization Menu ===" << endl;
    cout << "1. View graph statistics" << endl;
    cout << "2. View graph in text format" << endl;
    cout << "3. Generate Graphviz DOT file" << endl;
    cout << "4. Skip visualization and find path" << endl;
    cout << "Enter your choice (1-4): ";
    cin >> choice;
    
    switch(choice) {
        case 1:
            printGraphStats(graph);
            break;
        case 2:
            printGraph(graph);
            break;
        case 3:
            {
                string layout, engine;
                cout << "Choose layout direction:" << endl;
                cout << "  LR = Left to Right (default)" << endl;
                cout << "  TB = Top to Bottom" << endl;
                cout << "  BT = Bottom to Top" << endl;
                cout << "  RL = Right to Left" << endl;
                cout << "Enter layout (LR/TB/BT/RL) [default: LR]: ";
                cin >> layout;
                if (layout.empty()) layout = "LR";
                
                cout << "Choose layout engine:" << endl;
                cout << "  neato = Spring model (RECOMMENDED - edge lengths proportional to weights)" << endl;
                cout << "  fdp = Force-directed (edge lengths proportional to weights)" << endl;
                cout << "  dot = Hierarchical (edge lengths NOT proportional to weights)" << endl;
                cout << "  circo = Circular layout (edge lengths NOT proportional to weights)" << endl;
                cout << "Enter engine (neato/fdp/dot/circo) [default: neato]: ";
                cin >> engine;
                if (engine.empty()) engine = "neato";
                
                generateDotFile(graph, "graph.dot", layout, engine);
            }
            break;
        case 4:
            break;
        default:
            cout << "Invalid choice. Skipping visualization." << endl;
    }
    
    // Get source and destination from user
    string source, destination;
    cout << "Enter source vertex: ";
    cin >> source;
    cout << "Enter destination vertex: ";
    cin >> destination;
    
    // Run Dijkstra's algorithm
    DijkstraResult result = dijkstra(graph, source);
    
    // Print shortest path and distance
    printShortestPath(result, source, destination);
    
    // Get shortest distance for alternative paths
    int shortestDistance = INT_MAX;
    if (result.distances.find(destination) != result.distances.end()) {
        shortestDistance = result.distances.at(destination);
    }
    
    // Find and print alternative paths
    if (shortestDistance != INT_MAX) {
        findAlternativePaths(graph, source, destination, shortestDistance);
    }
    
    return 0;
}

