/*!
 * @brief
 *
 * @file
 *
 * @ingroup     a-star
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include <iostream>
#include <chrono>

#include "graph_def.h"
#include "utils.h"
#include "search.h"

/*------------------------------------------        Main Function           ------------------------------------------*/
int main(int argc, char **argv) {
    // Terminal input handling
    a_star::Config c;
    bool stat = a_star::parse_args(argc, argv, c);
    if(not stat) return -1;

    // Graph Creation
    a_star::Graph g;
    a_star::read_nodes(c.node_fp, g, c.verbose);
    a_star::read_edges(c.edge_fp, g, c.verbose);

    // Weighted A-Star Search
    auto t1 = std::chrono::high_resolution_clock::now();
    bool search_status = a_star::search_1(c.start_id, c.goal_id, g, a_star::eu_dist_heuristic, c.weight, c.verbose);
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();


    // Output Presentation
    if (search_status and c.verbose) {
        std::cout << "The search was successful. It took " << duration << " microseconds." << std::endl <<
                  "Start Node: " << g[c.start_id] << std::endl << "Goal Node: " << g[c.goal_id] << std::endl
                  << std::endl;
    } else if (c.verbose) {
        std::cout << "The search was unsuccessful. It took " << duration << " microseconds." << std::endl <<
                  "Start Node: " << g[c.start_id] << std::endl << "Goal Node: " << g[c.goal_id] << std::endl
                  << std::endl;
    }

    // Output Saving
    a_star::write_path(c.path_fp, c.goal_id, g, c.verbose);
    a_star::write_graph(c.search_fp, g, c.verbose);

    return 0;
}
