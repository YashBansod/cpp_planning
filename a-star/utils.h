/*!
 * @brief
 *
 * @file
 *
 * @ingroup     a-star
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>

#include "boost/graph/adjacency_list.hpp"
#include "graph_def.h"

/*------------------------------------------            Definitions         ------------------------------------------*/
#ifndef A_STAR_UTILITIES_H
#define A_STAR_UTILITIES_H

namespace a_star {
    void read_nodes(const std::string &file_name, Graph &g, const bool verbose = false) {
        std::ifstream node_file(file_name);
        if (node_file.is_open()) {
            if (verbose) std::cout << "Successfully opened the nodes file: " << file_name << std::endl;
            std::string line, temp;
            int num_nodes = 0, id = 0;
            float x = 0, y = 0;
            node_file >> num_nodes;
            if (verbose) std::cout << "The nodes files has " << num_nodes << " nodes." << std::endl;
            while (!node_file.eof()) {
                node_file >> id >> temp >> x >> temp >> y;
                if (node_file.good()) {
                    if (verbose) std::cout << id << ", " << x << ", " << y << std::endl;
                    id--;
                    boost::add_vertex(g);
                    g[id].node_id = id;
                    g[id].x = x;
                    g[id].y = y;
                }
            }
            node_file.close();
            if (verbose) std::cout << boost::num_vertices(g) << " nodes were added to the graph.\n" << std::endl;
        } else if (verbose) std::cout << "Could not read the file \"" << file_name << "\".\n" << std::endl;
    }

    void read_edges(const std::string &file_name, Graph &g, const bool verbose = false) {
        std::ifstream edge_file(file_name);
        if (edge_file.is_open()) {
            if (verbose) std::cout << "Successfully opened the edges file: " << file_name << std::endl;
            std::string line, temp;
            int num_edges = 0, id_1 = 0, id_2 = 0;
            float cost = 0;
            edge_file >> num_edges;
            if (verbose) std::cout << "The edges file has " << num_edges << " edges." << std::endl;
            while (!edge_file.eof()) {
                edge_file >> id_1 >> temp >> id_2 >> temp >> cost;
                if (edge_file.good()) {
                    if (verbose) std::cout << "(" << id_1 << ", " << id_2 << "): " << cost << std::endl;
                    id_1--;
                    id_2--;
                    auto[e_dtr, exists] = boost::edge(id_1, id_2, g);
                    if (exists) g[e_dtr].cost = std::fmin(g[e_dtr].cost, cost);
                    else {
                        auto e = boost::add_edge(id_1, id_2, g).first;
                        g[e].source = id_1;
                        g[e].target = id_2;
                        g[e].cost = cost;
                    }
                }
            }
            edge_file.close();
            if (verbose) std::cout << boost::num_edges(g) << " unique edges were added to the graph.\n" << std::endl;
        } else if (verbose) std::cout << "Could not read the file \"" << file_name << "\".\n" << std::endl;
    }

    void write_path(const std::string &file_name, const int goal_node_id, const Graph &g, const bool verbose = false) {
        std::ofstream path_file(file_name);
        if (path_file.is_open()) {
            if (verbose) std::cout << "Successfully opened the path file: " << file_name << std::endl;
            int c_node = goal_node_id;
            while (c_node != -1) {
                path_file << g[c_node].node_id + 1 << ", " << g[c_node].x << ", " << g[c_node].y << std::endl;
                c_node = g[c_node].p_node_id;
            }
            path_file.close();
            if (verbose) std::cout << "Successfully wrote path file: \"" << file_name << "\".\n" << std::endl;
        } else if (verbose) std::cout << "Could not open the file \"" << file_name << "\".\n" << std::endl;
    }

    void write_graph(const std::string &file_name, const Graph &g, const bool verbose = false) {
        std::ofstream graph_file(file_name);
        if (graph_file.is_open()) {
            if (verbose) std::cout << "Successfully opened the graph file: " << file_name << std::endl;
            int p;
            for (auto[v, end_v] = boost::vertices(g); v != end_v; ++v) {
                p = g[*v].p_node_id;
                if (p == -1) continue;
                graph_file << g[*v].node_id + 1 << ", " << g[*v].x << ", " << g[*v].y << ", " <<
                           p + 1 << ", " << g[p].x << ", " << g[p].y << std::endl;
            }
            graph_file.close();
            if (verbose) std::cout << "Successfully wrote graph file: \"" << file_name << "\".\n" << std::endl;
        } else if (verbose) std::cout << "Could not open the file \"" << file_name << "\".\n" << std::endl;
    }

    void print_help_text() {
        std::cerr << "Required arguments missing. Terminating the program." << std::endl;
        std::cerr << "The following arguments at respective positions are required: " << std::endl;
        std::cerr << "Position 1: Start node id as listed in the Nodes file. Example: 9" << std::endl;
        std::cerr << "Position 2: Goal node id as listed in the Nodes file. Example: 5\n" << std::endl;

        std::cerr << "Additionally, the following arguments can be specified: (Optional)" << std::endl;
        std::cerr << "Position 3: relative/absolute path to the Nodes file. Example: nodes.txt" << std::endl;
        std::cerr << "Position 4: relative/absolute path to the Edges file. Example: edges.txt" << std::endl;
        std::cerr << "Position 5: relative/absolute path to the Path file. Example: output_path.txt" << std::endl;
        std::cerr << "Position 6: relative/absolute path to the Graph file. Example: search_tree.txt" << std::endl;
        std::cerr << "Position 7: Weight assigned to heuristic. Example: 4.5" << std::endl;
        std::cerr << "Position 8: Verbosity. Choose between 0 and 1" << std::endl;
    }
}

#endif //A_STAR_UTILITIES_H
