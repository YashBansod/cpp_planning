/*!
 * @brief
 *
 * @file
 *
 * @ingroup     rrt_template_exp
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include <iostream>

#include "boost/graph/adjacency_list.hpp"
#include "graph_def.h"

/*------------------------------------------            Definitions         ------------------------------------------*/
#ifndef RRT_SEARCH_H
#define RRT_SEARCH_H
namespace rrt {

    template<typename T>
    inline Node<T> traj_func_1(const double eps, const ObstacleVec <CircleObstacle<T>> &obs_vec) {
        return Node<T>();
    }

    template<typename T>
    bool
    search_1(const T &start, const T &goal, const double radius, const double bais, const Workspace<T> &w_space,
             const ObstacleVec <CircleObstacle<T>> &obs_vec, Graph <Node<T>, Edge> &g,
             const std::function<Node<T>(const double, const ObstacleVec <CircleObstacle<T>> &)> &t_func,
             const double eps = 1, const int iter_lim = 1e8, const int verbose = 0) {

        auto s_id = boost::add_vertex(g);
        g[s_id].node_id = s_id;
        g[s_id].node = start;
        g[s_id].g_cost = 0;

        auto g_id = boost::add_vertex(g);
        g[g_id].node_id = g_id;
        g[g_id].node = goal;

        for(int _iter = 0; (g[g_id].p_node_id == -1 and _iter < iter_lim); ++_iter){


            auto n_id = boost::add_vertex(g);
            g[n_id] = traj_func_1(eps, obs_vec);
            double goal_dist = goal.eu_dist(g[n_id].node);

        }

        std::cout << g[s_id] << std::endl;
        std::cout << g[g_id] << std::endl;

        std::cout << t_func(start, goal, radius, eps, obs_vec) << std::endl;
        return false;
    }

}
#endif //RRT_SEARCH_H


