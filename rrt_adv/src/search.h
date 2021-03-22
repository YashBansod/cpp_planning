/*!
 * @brief
 *
 * @file
 *
 * @ingroup     rrt_adv
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include <iostream>

#include "boost/graph/adjacency_list.hpp"
#include "graph_def.h"

/*------------------------------------------            Definitions         ------------------------------------------*/
#ifndef RRT_ADV_SEARCH_H
#define RRT_ADV_SEARCH_H
namespace rrt {

    inline bool collision_check(Point2D a, Point2D b, const CircleObstacle &c) {
        a.x -= c.center.x;
        a.y -= c.center.y;
        b.x -= c.center.x;
        b.y -= c.center.y;
        const double _a = std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2);
        const double _b = 2 * (a.x * (b.x - a.x) + a.y * (b.y - a.y));
        const double _c = std::pow(a.x, 2) + std::pow(a.y, 2) - std::pow(c.r, 2);
        const double discriminant = std::pow(_b, 2) - 4 * _a * _c;
        if (discriminant <= 0) return false;
        const double sqrt_disc = std::sqrt(discriminant);
        const double t1 = (-_b + sqrt_disc) / (2 * _a);
        const double t2 = (-_b - sqrt_disc) / (2 * _a);
        if ((0 < t1 && t1 < 1) || (0 < t2 && t2 < 1)) return true;
        return false;
    }

    std::pair<int, bool>
    search(const Point2D &start, const GoalZone &goal, Graph &g, Workspace &w_space, const ObstacleVec &obs_vec,
           const std::function<bool(Point2D, Point2D, const CircleObstacle&)> &collision_func,
           const double eps = 1, const int iter_lim = 1e8, const int verbose = 0) {

        auto s_id = boost::add_vertex(g);
        g[s_id].node_id = s_id;
        g[s_id].node = start;
        g[s_id].g_cost = 0;
        if (verbose > 1) std::cout << "Added Start Node: " << g[s_id] << std::endl;

        int g_id = -1;
        bool found_goal = false;
        // If goal is completely covered by an obstacle then return failure.
        for(auto& x: obs_vec){
            if(x.center.eu_dist(goal.center) <= std::max(x.r, goal.r) - std::min(x.r, goal.r)) return {g_id, found_goal};
            if(x.center.eu_dist(start) <= x.r) return {g_id, found_goal};
        }


        for(int _iter = 0; (not found_goal and _iter < iter_lim); ++_iter){
            Point2D rand_pt = w_space.sample();
            auto[v, end_v] = boost::vertices(g);
            double min_dist = rand_pt.eu_dist(g[*v].node);
            int min_ind = *v++;
            for (; v != end_v; ++v) {
                double dist = rand_pt.eu_dist(g[*v].node);
                if (dist < min_dist) {
                    min_dist = dist;
                    min_ind = *v;
                }
            }
            if (min_dist > eps) {
                double ratio = eps / min_dist;
                rand_pt.x = (1 - ratio) * g[min_ind].node.x + ratio * rand_pt.x;
                rand_pt.y = (1 - ratio) * g[min_ind].node.y + ratio * rand_pt.y;
                min_dist = eps;
            }
            bool collision = false;
            for(auto& obs : obs_vec){
                collision = collision_func(g[min_ind].node, rand_pt, obs);
                if(collision) break;
            }
            if(not collision) {
                auto n_id = boost::add_vertex(g);
                g[n_id].node_id = n_id;
                g[n_id].node = rand_pt;
                g[n_id].p_node_id = min_ind;
                g[n_id].g_cost = g[min_ind].g_cost + min_dist;

                auto e = boost::add_edge(min_ind, n_id, g).first;
                g[e].source = min_ind;
                g[e].target = n_id;
                g[e].cost = min_dist;

                if (verbose > 2) std::cout << "Added Node: " << g[n_id] << std::endl;

                double goal_dist = rand_pt.eu_dist(goal.center);
                if(goal_dist < goal.r){
                    g_id = n_id;
                    found_goal = true;
                    if (verbose > 1) std::cout << "Found Node: " << g[n_id] << "in GoalZone: " << goal << std::endl;
                }
            }
        }

        return {g_id, found_goal};
    }
}
#endif //RRT_ADV_SEARCH_H


