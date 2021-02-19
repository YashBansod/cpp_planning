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
#include "boost/program_options.hpp"
#include "graph_def.h"

namespace po = boost::program_options;

/*------------------------------------------            Definitions         ------------------------------------------*/
#ifndef A_STAR_UTILITIES_H
#define A_STAR_UTILITIES_H

namespace a_star {

    struct Config {
        int start_id, goal_id, verbose;
        double weight;
        std::string node_fp, edge_fp, path_fp, search_fp;
    };

    inline std::ostream &operator<<(std::ostream &o_str, const Config &c) {
        o_str << "start_id: " << c.start_id + 1 << ", goal_id: " << c.goal_id + 1 <<
                ", weight: " << c.weight << ", verbose: " << c.verbose <<
                "\nnode_fp: " << c.node_fp << "\nedge_fp: " << c.edge_fp <<
                "\npath_fp: " << c.path_fp << "\nsearch_fp: " << c.search_fp << " ";
        return o_str;
    }

    bool parse_args(int argc, char **argv, Config& c){
        try {
            po::options_description desc("Allowed options", 120, 60);
            desc.add_options()
                    ("help,h", "Print the help message\n")
                    ("start_id,s", po::value<int>(&c.start_id)->default_value(1),
                     "Start node id as listed in the Nodes file")
                    ("goal_id,g", po::value<int>(&c.goal_id)->default_value(1),
                     "Goal node id as listed in the Nodes file")
                    ("weight,w", po::value<double>(&c.weight)->default_value(1.0),
                     "Weight assigned to heuristic")
                    ("verbose,v", po::value<int>(&c.verbose)->implicit_value(1)->default_value(0),
                     "Verbosity. Choose between 0, 1, 2 and 3")
                    ("node_fp,n", po::value<std::string>(&c.node_fp)->default_value("nodes.txt"),
                     "Relative/absolute path to the Nodes file")
                    ("edge_fp,e", po::value<std::string>(&c.edge_fp)->default_value("edges.txt"),
                     "Relative/absolute path to the Edges file")
                    ("path_fp,p", po::value<std::string>(&c.path_fp)->default_value("path_output.txt"),
                     "Relative/absolute path to the Path output file")
                    ("search_fp,t", po::value<std::string>(&c.search_fp)->default_value("search_output.txt"),
                     "Relative/absolute path to the Search output file")
                    ;
            po::variables_map vm;
            po::store(po::parse_command_line(argc, argv, desc), vm);
            po::notify(vm);
            c.start_id--;
            c.goal_id--;
            if(c.verbose) std::cout << c << std::endl << std::endl;

            if (vm.count("help")) {
                std::cout << desc << std::endl;
                return false;
            }
        }
        catch(std::exception& e) {
            std::cerr << "error: " << e.what() << "\n";
            return false;
        }
        catch(...) {
            std::cerr << "Exception of unknown type!\n";
            return false;
        }
        return true;
    }

    void read_nodes(const std::string &file_name, Graph &g, const int verbose = 0) {
        std::ifstream node_file(file_name);
        if (node_file.is_open()) {
            if (verbose) std::cout << "Successfully opened the nodes file: " << file_name << std::endl;
            std::string line, temp;
            int num_nodes = 0, id = 0;
            double x = 0, y = 0;
            node_file >> num_nodes;
            if (verbose) std::cout << "The nodes files has " << num_nodes << " nodes." << std::endl;
            while (!node_file.eof()) {
                node_file >> id >> temp >> x >> temp >> y;
                if (node_file.good()) {
                    if (verbose > 2) std::cout << id << ", " << x << ", " << y << std::endl;
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

    void read_edges(const std::string &file_name, Graph &g, const int verbose = 0) {
        std::ifstream edge_file(file_name);
        if (edge_file.is_open()) {
            if (verbose) std::cout << "Successfully opened the edges file: " << file_name << std::endl;
            std::string line, temp;
            int num_edges = 0, id_1 = 0, id_2 = 0;
            double cost = 0;
            edge_file >> num_edges;
            if (verbose) std::cout << "The edges file has " << num_edges << " edges." << std::endl;
            while (!edge_file.eof()) {
                edge_file >> id_1 >> temp >> id_2 >> temp >> cost;
                if (edge_file.good()) {
                    if (verbose > 2) std::cout << "(" << id_1 << ", " << id_2 << "): " << cost << std::endl;
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

    void write_path(const std::string &file_name, const int goal_node_id, const Graph &g, const int verbose = 0) {
        std::ofstream path_file(file_name);
        if (path_file.is_open()) {
            if (verbose) std::cout << "Successfully opened the path file: " << file_name << std::endl;
            int c_node = goal_node_id;
            std::string wayp;
            while (c_node != -1) {
                wayp = std::to_string(g[c_node].node_id + 1) + ", " +
                        std::to_string(g[c_node].x) + ", " + std::to_string(g[c_node].y);
                if (verbose > 2) std::cout << wayp << std::endl;
                    path_file << wayp << std::endl;
                c_node = g[c_node].p_node_id;
            }
            path_file.close();
            if (verbose) std::cout << "Successfully wrote path file: \"" << file_name << "\".\n" << std::endl;
        } else if (verbose) std::cout << "Could not open the file \"" << file_name << "\".\n" << std::endl;
    }

    void write_graph(const std::string &file_name, const Graph &g, const int verbose = 0) {
        std::ofstream graph_file(file_name);
        if (graph_file.is_open()) {
            if (verbose) std::cout << "Successfully opened the graph file: " << file_name << std::endl;
            int p;
            std::string entry;
            for (auto[v, end_v] = boost::vertices(g); v != end_v; ++v) {
                p = g[*v].p_node_id;
                if (p == -1) continue;
                entry = std::to_string(g[*v].node_id + 1) + ", " + std::to_string(g[*v].x) + ", " +
                        std::to_string(g[*v].y) + ", " + std::to_string(p + 1) + ", " +
                        std::to_string(g[p].x) + ", " + std::to_string(g[p].y);
                if (verbose > 2) std::cout << entry << std::endl;
                graph_file << entry << std::endl;
            }
            graph_file.close();
            if (verbose) std::cout << "Successfully wrote graph file: \"" << file_name << "\".\n" << std::endl;
        } else if (verbose) std::cout << "Could not open the file \"" << file_name << "\".\n" << std::endl;
    }
}

#endif //A_STAR_UTILITIES_H
