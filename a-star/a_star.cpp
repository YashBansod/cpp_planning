/*!
 * @brief
 *
 * @file
 *
 * @ingroup     a-star
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include <iostream>
#include <string>

#include "graph_def.h"
#include "utils.h"
#include "search.h"

/*------------------------------------------        Main Function           ------------------------------------------*/
int main(int argc, char **argv) {
    // Terminal input handling
    if (argc < 3) {
        a_star::print_help_text();
        return -1;
    }
    const int start_node = std::stoi(argv[1]) - 1, goal_node = std::stoi(argv[2]) - 1;
    std::string node_fp = "nodes.txt", edge_fp = "edges.txt", path_fp = "output_path.txt", graph_fp = "search_tree.txt";
    float weight = 1;
    bool verbose = false;
    if (argc > 3) node_fp = argv[3];
    if (argc > 4) edge_fp = argv[4];
    if (argc > 5) path_fp = argv[5];
    if (argc > 6) graph_fp = argv[6];
    if (argc > 7) weight = std::stof(argv[7]);
    if (argc > 8) verbose = (bool) std::stoi(argv[8]);

    // Graph Creation
    a_star::Graph g;
    a_star::read_nodes(node_fp, g, verbose);
    a_star::read_edges(edge_fp, g, verbose);

    // Weighted A-Star Search
    bool search_status = a_star::search(start_node, goal_node, g, a_star::eu_dist_heuristic, weight, verbose);

    // Output Presentation
    if (search_status and verbose) {
        std::cout << "The search was successful." << std::endl <<
                  "Start Node: " << g[start_node] << std::endl << "Goal Node: " << g[goal_node] << std::endl
                  << std::endl;
    } else if (verbose) {
        std::cout << "The search was unsuccessful." << std::endl <<
                  "Start Node: " << g[start_node] << std::endl << "Goal Node: " << g[goal_node] << std::endl
                  << std::endl;
    }

    // Output Saving
    a_star::write_path(path_fp, goal_node, g, verbose);
    a_star::write_graph(graph_fp, g, verbose);

    return 0;
}
