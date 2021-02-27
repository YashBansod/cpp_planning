/*!
 * @brief
 *
 * @file
 *
 * @ingroup     rrt_2d
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include <iostream>
#include <chrono>

#include "src/graph_def.h"
#include "src/search.h"
#include "src/utils.h"

/*------------------------------------------        Main Function           ------------------------------------------*/
int main(int argc, char **argv) {
    // Terminal input handling
    rrt::Config c;
    bool stat = rrt::parse_args(argc, argv, c);
    if(not stat) return -1;

    rrt::Workspace w_space(rrt::Point2D(-50, -50), rrt::Point2D(50, 50), c.goal.center, c.goal_bias);

    rrt::ObstacleVec obs_vec;
    rrt::read_obstacles(c.obs_fp, obs_vec, c.verbose);

    rrt::Graph g;

    auto t1 = std::chrono::high_resolution_clock::now();
    auto [g_id, search_status] = search_1(c.start, c.goal, g, w_space,obs_vec, c.eps, c.iter_lim, c.verbose);
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

    // Output Presentation
    if (search_status and c.verbose) {
        std::cout << "The search was successful. It took " << duration << " microseconds." << std::endl <<
                "Start: " << c.start << std::endl << "Goal: " << c.goal << std::endl << std::endl;
    } else if (c.verbose) {
        std::cout << "The search was unsuccessful. It took " << duration << " microseconds." << std::endl <<
                "Start: " << c.start << std::endl << "Goal: " << c.goal << std::endl << std::endl;
    }

    // Output Saving
    rrt::write_path(c.path_fp, g_id, g, c.verbose);
    rrt::write_graph(c.search_fp, g, c.verbose);

    return 0;
}
