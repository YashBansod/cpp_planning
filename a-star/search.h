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
#include <set>
#include <vector>
#include <queue>
#include <list>
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

    // Open Nodes List uses std::set. Element needs to be removed before modification and then re-inserted.
    bool search_1(int s_node_id, int g_node_id, Graph &g, const std::function<double(Node & , Node & )> &h_func,
                  const double weight = 1, const int verbose = 0) {

        auto lt_compare = [](const Node *l, const Node *r) {return l->f_cost < r->f_cost;};
        std::multiset<Node *, decltype(lt_compare)> open_nodes(lt_compare);

        g[s_node_id].g_cost = 0;
        g[s_node_id].h_cost = weight * h_func(g[s_node_id], g[g_node_id]);
        g[s_node_id].f_cost = g[s_node_id].g_cost + g[s_node_id].h_cost;
        g[s_node_id].status = open;
        open_nodes.emplace(&g[s_node_id]);
        int curr_node_id;

        while (!open_nodes.empty()) {

            curr_node_id = open_nodes.extract(open_nodes.begin()).value()->node_id;

            g[curr_node_id].status = closed;
            if (curr_node_id == g_node_id) return true;

            for (auto[e_iter, e_end] = boost::out_edges(curr_node_id, g); e_iter != e_end; ++e_iter) {
                Graph::vertex_descriptor dst_v = boost::target(*e_iter, g);
                if (g[dst_v].status == undef) {
                    g[dst_v].status = open;
                    g[dst_v].h_cost = weight * h_func(g[dst_v], g[g_node_id]);
                    g[dst_v].g_cost = g[curr_node_id].g_cost + g[*e_iter].cost;
                    g[dst_v].f_cost = g[dst_v].g_cost + g[dst_v].h_cost;
                    g[dst_v].p_node_id = curr_node_id;
                    open_nodes.emplace(&g[dst_v]);
                    if (verbose > 1) std::cout << "Opened: " << g[dst_v] << std::endl;
                } else if (g[dst_v].status == open) {
                    double new_cost = g[curr_node_id].g_cost + g[*e_iter].cost;
                    if (new_cost < g[dst_v].g_cost - 1e-15) {
                        auto node = open_nodes.extract(&g[dst_v]).value();
                        node->g_cost = new_cost;
                        node->f_cost = node->g_cost + node->h_cost;
                        node->p_node_id = curr_node_id;
                        open_nodes.emplace(node);
                        if (verbose > 1) std::cout << "Modified: " << g[dst_v] << std::endl;
                    }
                } else {double new_cost = g[curr_node_id].g_cost + g[*e_iter].cost;
                    if (new_cost < g[dst_v].g_cost - 1e-15) {
                        g[dst_v].status = open;
                        g[dst_v].g_cost = new_cost;
                        g[dst_v].f_cost = g[dst_v].g_cost + g[dst_v].h_cost;
                        g[dst_v].p_node_id = curr_node_id;
                        open_nodes.emplace(&g[dst_v]);
                        if (verbose > 1) std::cout << "Reopened: " << g[dst_v] << std::endl;
                    }
                }
            }
        }
        return false;
    }

    // Open Nodes List uses std::priority_queue. Heap needs to be remade after modification of an element.
    bool search_2(int s_node_id, int g_node_id, Graph &g, const std::function<double(Node & , Node & )> &h_func,
                  const double weight = 1, const int verbose = 0) {

        auto gt_compare = [](const Node *l, const Node *r) {return l->f_cost > r->f_cost;};
        std::priority_queue<Node *, std::vector<Node *>, decltype(gt_compare)> open_nodes(gt_compare);

        g[s_node_id].g_cost = 0;
        g[s_node_id].h_cost = weight * h_func(g[s_node_id], g[g_node_id]);
        g[s_node_id].f_cost = g[s_node_id].g_cost + g[s_node_id].h_cost;
        g[s_node_id].status = open;
        open_nodes.emplace(&g[s_node_id]);
        int curr_node_id;

        while (!open_nodes.empty()) {
            curr_node_id = open_nodes.top()->node_id;
            open_nodes.pop();

            g[curr_node_id].status = closed;
            if (curr_node_id == g_node_id) return true;

            for (auto[e_iter, e_end] = boost::out_edges(curr_node_id, g); e_iter != e_end; ++e_iter) {
                Graph::vertex_descriptor dst_v = boost::target(*e_iter, g);
                if (g[dst_v].status == undef) {
                    g[dst_v].status = open;
                    g[dst_v].h_cost = weight * h_func(g[dst_v], g[g_node_id]);
                    g[dst_v].g_cost = g[curr_node_id].g_cost + g[*e_iter].cost;
                    g[dst_v].f_cost = g[dst_v].g_cost + g[dst_v].h_cost;
                    g[dst_v].p_node_id = curr_node_id;
                    open_nodes.emplace(&g[dst_v]);
                    if (verbose > 1) std::cout << "Opened: " << g[dst_v] << std::endl;
                } else if (g[dst_v].status == open) {
                    double new_cost = g[curr_node_id].g_cost + g[*e_iter].cost;
                    if (new_cost < g[dst_v].g_cost - 1e-15) {
                        g[dst_v].g_cost = new_cost;
                        g[dst_v].f_cost = g[dst_v].g_cost + g[dst_v].h_cost;
                        g[dst_v].p_node_id = curr_node_id;
                        std::make_heap(const_cast<Node**>(&open_nodes.top()),
                                       const_cast<Node**>(&open_nodes.top()) + open_nodes.size(), gt_compare);
                        if (verbose > 1) std::cout << "Modified: " << g[dst_v] << std::endl;
                    }
                } else {double new_cost = g[curr_node_id].g_cost + g[*e_iter].cost;
                    if (new_cost < g[dst_v].g_cost - 1e-15) {
                        g[dst_v].status = open;
                        g[dst_v].g_cost = new_cost;
                        g[dst_v].f_cost = g[dst_v].g_cost + g[dst_v].h_cost;
                        g[dst_v].p_node_id = curr_node_id;
                        open_nodes.emplace(&g[dst_v]);
                        if (verbose > 1) std::cout << "Reopened: " << g[dst_v] << std::endl;
                    }
                }
            }
        }
        return false;
    }

    // Open Nodes List uses std::list. Insertion sort is used to maintain order.
    bool search_3(int s_node_id, int g_node_id, Graph &g, const std::function<double(Node & , Node & )> &h_func,
                  const double weight = 1, const int verbose = 0) {

        std::list<Node*> open_nodes;

        g[s_node_id].g_cost = 0;
        g[s_node_id].h_cost = weight * h_func(g[s_node_id], g[g_node_id]);
        g[s_node_id].f_cost = g[s_node_id].g_cost + g[s_node_id].h_cost;
        g[s_node_id].status = open;
        open_nodes.emplace_front(&g[s_node_id]);
        int curr_node_id;


        while (!open_nodes.empty()) {
            curr_node_id = open_nodes.front()->node_id;
            open_nodes.pop_front();

            g[curr_node_id].status = closed;
            if (curr_node_id == g_node_id) return true;

            for (auto[e_iter, e_end] = boost::out_edges(curr_node_id, g); e_iter != e_end; ++e_iter) {
                Graph::vertex_descriptor dst_v = boost::target(*e_iter, g);
                if (g[dst_v].status == undef) {
                    g[dst_v].status = open;
                    g[dst_v].h_cost = weight * h_func(g[dst_v], g[g_node_id]);
                    g[dst_v].g_cost = g[curr_node_id].g_cost + g[*e_iter].cost;
                    g[dst_v].f_cost = g[dst_v].g_cost + g[dst_v].h_cost;
                    g[dst_v].p_node_id = curr_node_id;
                    auto it = std::find_if(open_nodes.begin(), open_nodes.end(), [&](const Node* i){
                        return i->f_cost > g[dst_v].f_cost;});
                    open_nodes.insert(it, &g[dst_v]);
                    if (verbose > 1) std::cout << "Opened: " << g[dst_v] << std::endl;
                } else if (g[dst_v].status == open) {
                    double new_cost = g[curr_node_id].g_cost + g[*e_iter].cost;
                    if (new_cost < g[dst_v].g_cost - 1e-15) {
                        auto it = std::find(open_nodes.begin(), open_nodes.end(), &g[dst_v]);
                        open_nodes.erase(it);
                        g[dst_v].g_cost = new_cost;
                        g[dst_v].f_cost = g[dst_v].g_cost + g[dst_v].h_cost;
                        g[dst_v].p_node_id = curr_node_id;
                        auto it_2 = std::find_if(open_nodes.begin(), open_nodes.end(), [&](const Node* i){
                            return i->f_cost > g[dst_v].f_cost;});
                        open_nodes.insert(it_2, &g[dst_v]);
                        if (verbose > 1) std::cout << "Modified: " << g[dst_v] << std::endl;
                    }
                } else {double new_cost = g[curr_node_id].g_cost + g[*e_iter].cost;
                    if (new_cost < g[dst_v].g_cost - 1e-15) {
                        g[dst_v].status = open;
                        g[dst_v].g_cost = new_cost;
                        g[dst_v].f_cost = g[dst_v].g_cost + g[dst_v].h_cost;
                        g[dst_v].p_node_id = curr_node_id;
                        auto it = std::find_if(open_nodes.begin(), open_nodes.end(), [&](const Node* i){
                            return i->f_cost > g[dst_v].f_cost;});
                        open_nodes.insert(it, &g[dst_v]);
                        if (verbose > 1) std::cout << "Reopened: " << g[dst_v] << std::endl;
                    }
                }
            }
        }


        return false;
    }
}
#endif //A_STAR_SEARCH_H
