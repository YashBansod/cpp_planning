/*!
 * @brief
 *
 * @file
 *
 * @ingroup     a-star
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include <iostream>
#include <functional>
#include <queue>
#include "graph_def.h"
#include "utils.h"
#include "boost/graph/adjacency_list.hpp"

/*------------------------------------------            Definitions         ------------------------------------------*/
#ifndef A_STAR_SEARCH_H
#define A_STAR_SEARCH_H
namespace a_star {
    inline double eu_dist_heuristic(Node &node_1, Node &node_2) {
        return std::sqrt(std::pow(node_1.x - node_2.x, 2) + std::pow(node_1.y - node_2.y, 2));
    }

    inline double man_dist_heuristic(Node &node_1, Node &node_2) {
        return std::abs(node_1.x - node_2.x) + std::abs(node_1.y - node_2.y);
    }

    inline double zero_heuristic(Node &node_1, Node &node_2) {
        return 0;
    }

    bool search(int start_node_id, int goal_node_id, Graph &g, const std::function<double(Node & , Node & )> &heuristic,
                const double weight = 1, const int verbose = 0) {

        auto gt_compare = [weight](const Node *l, const Node *r) {
            return (l->g_cost + weight * l->h_cost) > (r->g_cost + weight * r->h_cost);};
        std::priority_queue<Node *, std::vector<Node *>, decltype(gt_compare)> open_nodes(gt_compare);

        g[start_node_id].g_cost = 0;
        g[start_node_id].h_cost = heuristic(g[start_node_id], g[goal_node_id]);
        g[start_node_id].status = open;
        open_nodes.emplace(&g[start_node_id]);
        int curr_node_id;

        while (!open_nodes.empty()) {

            curr_node_id = open_nodes.top()->node_id;
            open_nodes.pop();
            g[curr_node_id].status = closed;
            if (curr_node_id == goal_node_id) return true;

            for (auto[e_iter, e_end] = boost::out_edges(curr_node_id, g); e_iter != e_end; ++e_iter) {
                Graph::vertex_descriptor dst_v = boost::target(*e_iter, g);
                if (g[dst_v].status == undef) {
                    g[dst_v].status = open;
                    g[dst_v].h_cost = heuristic(g[dst_v], g[goal_node_id]);
                    g[dst_v].g_cost = g[curr_node_id].g_cost + g[*e_iter].cost;
                    g[dst_v].p_node_id = curr_node_id;
                    open_nodes.emplace(&g[dst_v]);
                    if (verbose > 1) std::cout << "Opened: " << g[dst_v] << std::endl;
                } else if (g[dst_v].status == open) {
                    double new_cost = g[curr_node_id].g_cost + g[*e_iter].cost;
                    if (new_cost < g[dst_v].g_cost - 1e-15) {
                        g[dst_v].g_cost = new_cost;
                        g[dst_v].p_node_id = curr_node_id;
                        if (verbose > 1) std::cout << "Modified: " << g[dst_v] << std::endl;
                    }
                } else {double new_cost = g[curr_node_id].g_cost + g[*e_iter].cost;
                    if (new_cost < g[dst_v].g_cost - 1e-15) {
                        g[dst_v].status = open;
                        g[dst_v].g_cost = new_cost;
                        g[dst_v].p_node_id = curr_node_id;
                        open_nodes.emplace(&g[dst_v]);
                        if (verbose > 1) std::cout << "Reopened: " << g[dst_v] << std::endl;
                    }
                }
            }
        }
        return false;
    }
}
#endif //A_STAR_SEARCH_H
