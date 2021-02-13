# a-star

### Some Design Decisions
The graph structure was implemented using boost graph library. Nodes and Edges are user defined structures that are
stored in a std::vector. The graph was configured for bidirectional architecture.

The open nodes in the a-star algorithm are selected from a priority queue. Priority queue was constructed using 
**std::priority_queue**. It was configured to use a **std::deque** as its base container.

The code was written to allow a desired level of configurability. The configuration can be controlled by passing the
following arguments when executing from terminal:  
The following arguments at respective positions are required:  
Position 1: Start node id as listed in the Nodes file. Example: 9  
Position 2: Goal node id as listed in the Nodes file. Example: 5  
  
Additionally, the following arguments can be specified: (Optional)  
Position 3: relative/absolute path to the Nodes file. Example: nodes.txt  
Position 4: relative/absolute path to the Edges file. Example: edges.txt  
Position 5: relative/absolute path to the Path file. Example: output_path.txt  
Position 6: relative/absolute path to the Graph file. Example: search_tree.txt  
Position 7: Weight assigned to heuristic. Example: 4.5  
Position 8: Verbosity. Choose between 0 and 1  
example: `./a_star.exe 9 5 nodes.txt edges.txt output_path.txt search_tree.txt 4.5 1`

Also, the program can be compiled to use a heuristic from a list of heuristics defined in file [search.h](./search.h).  
Some examples of heuristics are: eu_dist_heuristic, man_dist_heuristic, zero_heuristic.

### Run Instructions
**For Ubuntu:**  
```shell script
# open this directory
cd <path to this directory>

# view the CMakeLists.txt file of the sub-project
cat ./CMakeLists.txt

# make sure proper directories are included in the CMakeLists.txt file.
<use code editor of your preference to edit CMakeLists.txt>

# create a build directory
mkdir build

# open the build directory
cd build

# cmake the project in the build directory
cmake .. 

# make the cmake project
make -j4

# execute the generated executible file
./<executible file>
```

**For Windows:** I would recommend using an IDE that uses CMake for build process. Example: CLion by Jetbrains.

