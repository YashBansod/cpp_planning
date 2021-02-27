/*!
 * @brief
 *
 * @file
 *
 * @ingroup     rrt_template_exp
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include <iostream>
#include <string>

#include "src/graph_def.h"
#include "src/search.h"
#include "src/utils.h"

/*------------------------------------------        Main Function           ------------------------------------------*/
int main(int argc, char **argv) {
    // Terminal input handling
    rrt::Config<rrt::Point2D> c;
    bool stat = rrt::parse_args(argc, argv, c);
    if(not stat) return -1;

    rrt::Workspace<rrt::Point2D> w_space(rrt::Point2D(-50, -50), rrt::Point2D(50, 50));
    std::cout << std::is_same_v<decltype(w_space), decltype(rrt::Workspace<rrt::Point2D>())>  << std::endl;

    rrt::ObstacleVec<rrt::CircleObstacle<rrt::Point2D>> obs_vec;
    rrt::read_obstacles_2d(c.obs_fp, obs_vec, c.verbose);

    rrt::Graph<rrt::Node<rrt::Point2D>, rrt::Edge> g;
//    bool search_status = rrt_template_exp::search_1(c.start, c.goal, c.radius, c.goal_bias, w_space,
//                                       obs_vec, g, rrt_template_exp::traj_func_1<rrt_template_exp::Point2D>, c.eps, c.iter_lim, c.verbose);



    return 0;
}
