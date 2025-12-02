# Train Route Path Finder - Dijkstra's Algorithm Implementation

A comprehensive C++ implementation of Dijkstra's shortest path algorithm for finding optimal train routes between stations. This project features graph visualization, alternative path finding, and detailed route analysis.

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Compilation](#compilation)
- [Usage](#usage)
- [Graph Structure](#graph-structure)
- [Features in Detail](#features-in-detail)
- [Examples](#examples)
- [Troubleshooting](#troubleshooting)

## 🎯 Overview

This project implements Dijkstra's algorithm to find the shortest path between train stations in a railway network. The graph is represented using string-based vertex names (station codes) and weighted edges (distances between stations). The program provides:

- Shortest path calculation between any two stations
- Alternative route suggestions (within 100% of shortest path)
- Graph visualization capabilities
- Detailed path analysis with distance and percentage comparisons

## ✨ Features

1. **Shortest Path Finding**: Classic Dijkstra's algorithm implementation
2. **Alternative Path Discovery**: Finds up to 10 alternative routes within 100% of shortest path
3. **Graph Visualization**: 
   - Text-based graph representation
   - Graph statistics
   - Graphviz DOT file generation for visual graphs
4. **Edge Length Proportional Visualization**: Edge lengths in visualizations are proportional to actual weights
5. **Percentage Comparison**: Shows how much longer alternative paths are compared to shortest path
6. **String-based Vertices**: Uses station codes (e.g., "NGP", "BD") instead of numeric IDs

## 🔧 Prerequisites

### Required Software

1. **C++ Compiler** (C++17 or later)
   - **Windows**: MinGW-w64, MSVC (Visual Studio), or Clang
   - **Linux**: GCC (`sudo apt-get install g++`)
   - **macOS**: Xcode Command Line Tools (`xcode-select --install`)

2. **Graphviz** (for graph visualization)
   - **Windows**: Download from [Graphviz Official Website](https://graphviz.org/download/) or use winget:
     ```powershell
     winget install --id Graphviz.Graphviz -e
     ```
   - **Linux**: `sudo apt-get install graphviz`
   - **macOS**: `brew install graphviz`

### Verify Installation

**Check C++ Compiler:**
```bash
g++ --version
```

**Check Graphviz:**
```bash
dot -V
# If not found, use full path:
# Windows: "C:\Program Files\Graphviz\bin\dot.exe" -V
```

## 📦 Installation

### Step 1: Clone or Download the Project

```bash
# If using git
git clone <repository-url>
cd "train project cpp"

# Or simply download and extract the project folder
```

### Step 2: Install Graphviz (if not already installed)

**Windows (using winget):**
```powershell
winget install --id Graphviz.Graphviz -e
```

**Windows (Manual):**
1. Download Graphviz from https://graphviz.org/download/
2. Run the installer
3. Add `C:\Program Files\Graphviz\bin` to your system PATH (see [Adding Graphviz to PATH](#adding-graphviz-to-path))

**Linux:**
```bash
sudo apt-get update
sudo apt-get install graphviz
```

**macOS:**
```bash
brew install graphviz
```

### Step 3: Add Graphviz to PATH (Windows)

**Temporary (current session only):**
```powershell
$env:Path += ";C:\Program Files\Graphviz\bin"
```

**Permanent (recommended):**
1. Open System Properties → Advanced → Environment Variables
2. Under "User variables", select `Path` → Edit
3. Click "New" and add: `C:\Program Files\Graphviz\bin`
4. Click OK on all dialogs
5. **Restart PowerShell/Terminal**

**Or use PowerShell (run as Administrator):**
```powershell
[Environment]::SetEnvironmentVariable(
    "Path",
    $env:Path + ";C:\Program Files\Graphviz\bin",
    "User"
)
```

## 🔨 Compilation

### Basic Compilation

```bash
g++ -std=c++17 dijkstra.cpp -o dijkstra.exe
```

### Compilation with Optimization

```bash
g++ -std=c++17 -O2 -Wall -Wextra dijkstra.cpp -o dijkstra.exe
```

### Platform-Specific Notes

**Windows:**
- Output file: `dijkstra.exe`
- Run with: `.\dijkstra.exe`

**Linux/macOS:**
- Output file: `dijkstra` (no extension)
- Run with: `./dijkstra`

## 🚀 Usage

### Basic Usage

1. **Compile the program:**
   ```bash
   g++ -std=c++17 dijkstra.cpp -o dijkstra.exe
   ```

2. **Run the program:**
   ```bash
   .\dijkstra.exe
   ```

3. **Follow the prompts:**
   - Choose visualization option (1-4)
   - Enter source station code (e.g., `NGP`)
   - Enter destination station code (e.g., `BD`)

### Example Session

```
=== Graph Visualization Menu ===
1. View graph statistics
2. View graph in text format
3. Generate Graphviz DOT file
4. Skip visualization and find path
Enter your choice (1-4): 4

Enter source vertex: NGP
Enter destination vertex: BD

Shortest path from "NGP" to "BD":
Path: NGP -> NRKR -> BD
Distance: 224

=== Alternative Paths (within 100% of shortest path) ===
Shortest path distance: 224
Maximum allowed distance: 448

Path #1 (SHORTEST, Distance: 224):
  NGP -> NRKR -> BD

Path #2 (+12.5% longer, Distance: 252):
  NGP -> SEGM -> WR -> BD

Path #3 (+25.0% longer, Distance: 280):
  NGP -> CWA -> AMLA -> NRKR -> BD

Total alternative paths found: 3
```

## 📊 Graph Structure

### How the Graph is Represented

The graph uses a **map-based adjacency list** structure:

```cpp
map<string, vector<Edge>> graph;
```

- **Key**: Station code (string) - e.g., `"NGP"`, `"BD"`
- **Value**: Vector of edges from that station
- **Edge**: Contains destination station and weight (distance)

### Adding Stations and Routes

Edit the `main()` function in `dijkstra.cpp`:

```cpp
// Add a route from station A to station B with distance X
graph["StationA"].push_back(Edge("StationB", X));

// Example: Route from Nagpur to SEGM with distance 76
graph["NGP"].push_back(Edge("SEGM", 76));
```

**Important Notes:**
- Stations are case-sensitive
- Distances should be positive integers
- Bidirectional routes require two entries:
  ```cpp
  graph["A"].push_back(Edge("B", 10));
  graph["B"].push_back(Edge("A", 10));
  ```

## 🎨 Features in Detail

### 1. Shortest Path Finding

**Algorithm**: Classic Dijkstra's algorithm using priority queue (min-heap)

**How it works:**
- Starts from source vertex
- Explores neighbors in order of shortest distance
- Maintains distances and previous vertex for path reconstruction
- Returns shortest distance and path to destination

**Output:**
- Complete path sequence
- Total distance

### 2. Alternative Path Discovery

**Feature**: Finds multiple paths between source and destination

**Constraints:**
- Paths must be ≤ 200% of shortest path distance (not more than 100% longer)
- Maximum of 10 paths shown (whichever is less)
- Uses DFS (Depth-First Search) with pruning

**Output:**
- Up to 10 alternative paths
- Each path shows:
  - Path sequence
  - Distance
  - Percentage increase compared to shortest path

**Example Output:**
```
Path #1 (SHORTEST, Distance: 224):
  NGP -> NRKR -> BD

Path #2 (+12.5% longer, Distance: 252):
  NGP -> SEGM -> WR -> BD
```

### 3. Graph Visualization

#### Option 1: Graph Statistics

Shows:
- Total number of vertices (stations)
- Total number of edges (routes)
- List of all station codes

#### Option 2: Text Format Visualization

Displays the graph in readable text format:
```
NGP -> SEGM(76), NRKR(86), CWA(150), TMR(80)
SEGM -> WR(2), BPQ(132), NGP(76)
...
```

#### Option 3: Graphviz DOT File Generation

Generates a `.dot` file that can be visualized as an image.

**Layout Options:**
- **LR**: Left to Right (default)
- **TB**: Top to Bottom
- **BT**: Bottom to Top
- **RL**: Right to Left

**Layout Engines:**
- **neato**: Spring model (RECOMMENDED - edge lengths proportional to weights)
- **fdp**: Force-directed (edge lengths proportional to weights)
- **dot**: Hierarchical (edge lengths NOT proportional)
- **circo**: Circular layout (edge lengths NOT proportional)

**To Generate Image:**

```bash
# Using neato (recommended for proportional edge lengths)
neato -Tpng graph.dot -o graph.png

# Using fdp
fdp -Tpng graph.dot -o graph.png

# Using dot (hierarchical)
dot -Tpng graph.dot -o graph.png

# Other formats
neato -Tsvg graph.dot -o graph.svg  # SVG format
neato -Tpdf graph.dot -o graph.pdf  # PDF format
```

**Note**: If Graphviz is not in PATH, use full path:
```powershell
&"C:\Program Files\Graphviz\bin\neato.exe" -Tpng graph.dot -o graph.png
```

### 4. Edge Length Proportional Visualization

When using `neato` or `fdp` engines:
- Edge lengths in the visualization are **proportional to actual weights**
- Longer routes appear as longer edges in the graph
- Automatically scales weights to 1.0-5.0 inch range

**Example:**
- Route with weight 100 → shorter edge
- Route with weight 300 → longer edge

## 📝 Examples

### Example 1: Finding Shortest Path

**Input:**
```
Source: NGP
Destination: BD
```

**Output:**
```
Shortest path from "NGP" to "BD":
Path: NGP -> NRKR -> BD
Distance: 224
```

### Example 2: Finding Alternative Paths

**Input:**
```
Source: NGP
Destination: BD
```

**Output:**
```
Shortest path from "NGP" to "BD":
Path: NGP -> NRKR -> BD
Distance: 224

=== Alternative Paths (within 100% of shortest path) ===
Shortest path distance: 224
Maximum allowed distance: 448

Path #1 (SHORTEST, Distance: 224):
  NGP -> NRKR -> BD

Path #2 (+12.5% longer, Distance: 252):
  NGP -> SEGM -> WR -> BD

Path #3 (+25.0% longer, Distance: 280):
  NGP -> CWA -> AMLA -> NRKR -> BD

Total alternative paths found: 3
```

### Example 3: Generating Graph Visualization

1. Run program and choose option 3
2. Select layout: `LR` (or press Enter for default)
3. Select engine: `neato` (or press Enter for default)
4. Generate PNG:
   ```bash
   neato -Tpng graph.dot -o graph.png
   ```
5. Open `graph.png` to view the graph

## 🐛 Troubleshooting

### Issue: "g++: command not found"

**Solution:**
- Install MinGW-w64 or use Visual Studio
- Or use `cl` (MSVC compiler) if Visual Studio is installed

### Issue: "dot: command not found" or "neato: command not found"

**Solution:**
1. Verify Graphviz is installed:
   ```powershell
   Test-Path "C:\Program Files\Graphviz\bin\dot.exe"
   ```
2. Add to PATH (see [Adding Graphviz to PATH](#adding-graphviz-to-path))
3. Or use full path:
   ```powershell
   &"C:\Program Files\Graphviz\bin\neato.exe" -Tpng graph.dot -o graph.png
   ```

### Issue: "No path exists"

**Possible Causes:**
- Station codes don't match (case-sensitive)
- No route exists between stations
- Station not in graph

**Solution:**
- Check station codes are correct
- Verify stations exist in graph
- Use graph statistics (option 1) to see all stations

### Issue: Program runs slowly for large graphs

**Explanation:**
- Alternative path finding uses DFS which can be slow for large graphs
- Consider reducing the threshold or limiting path count

### Issue: Too many alternative paths shown

**Solution:**
- The program automatically limits to 10 paths
- Paths beyond 200% of shortest distance are filtered out

## 📁 Project Structure

```
train project cpp/
│
├── dijkstra.cpp          # Main source file with all functionality
├── graph.dot             # Generated Graphviz DOT file (after running)
├── graph.png             # Generated graph image (after visualization)
└── README.md             # This file
```

## 🔍 Code Structure

### Key Components

1. **Edge Structure**: Represents a route with destination and weight
2. **DijkstraResult**: Stores distances and previous vertices
3. **dijkstra()**: Main algorithm implementation
4. **printShortestPath()**: Reconstructs and displays shortest path
5. **findAllPaths()**: DFS-based alternative path finder
6. **findAlternativePaths()**: Filters and displays alternative paths
7. **printGraph()**: Text visualization
8. **printGraphStats()**: Graph statistics
9. **generateDotFile()**: Graphviz DOT file generation

## 📚 Algorithm Details

### Dijkstra's Algorithm

- **Time Complexity**: O((V + E) log V) where V = vertices, E = edges
- **Space Complexity**: O(V)
- **Data Structures**: Priority queue, maps, sets
- **Assumptions**: Non-negative edge weights

### Alternative Path Finding

- **Algorithm**: DFS with pruning
- **Time Complexity**: O(V!) worst case (but pruned by threshold)
- **Optimization**: Early termination when threshold exceeded

## 🤝 Contributing

To add new features or modify the graph:

1. Edit `dijkstra.cpp`
2. Modify the graph in `main()` function
3. Recompile: `g++ -std=c++17 dijkstra.cpp -o dijkstra.exe`
4. Test your changes

## 📄 License

This project is provided as-is for educational and personal use.

## 🙏 Acknowledgments

- Dijkstra's algorithm by Edsger W. Dijkstra
- Graphviz for graph visualization capabilities

---

**Happy Path Finding! 🚂**
