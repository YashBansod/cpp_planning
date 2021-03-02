/*!
 * @brief
 *
 * @file
 *
 * @ingroup     rrt_2d
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include <iostream>
#include <string>
#include <fstream>

#include "boost/program_options.hpp"
#include "graph_def.h"

namespace po = boost::program_options;

/*------------------------------------------            Definitions         ------------------------------------------*/
#ifndef RRT_2D_UTILS_H
#define RRT_2D_UTILS_H
namespace rrt {
    struct Config {
        int verbose = 0, iter_lim = 1e8, r_seed = -1;
        Point2D start;
        GoalZone goal;
        double eps = 1, goal_bias = 0.0;
        std::string obs_fp, path_fp, search_fp;
    };

    inline std::ostream &operator<<(std::ostream &o_str, const Config &c) {
        o_str << "start: " << c.start << ", goal: " << c.goal << ", bias: " << c.goal_bias << ", seed: " << c.r_seed <<
                ", epsilon: " << c.eps <<", verbose: " << c.verbose << ", iter_lim: " << c.iter_lim <<
                "\nobs_fp: " << c.obs_fp << "\npath_fp: " << c.path_fp << "\nsearch_fp: " << c.search_fp << " ";
        return o_str;
    }

    bool parse_args(int argc, char **argv, Config& c){
        try {
            po::options_description desc("Allowed options", 120, 60);

            desc.add_options()
                    ("help,h", "Print the help message\n")
                    ("start,s", po::value<Point2D>(&c.start)->default_value({0, 0}),
                     "Start node coordinate as (x, y)")
                    ("goal,g", po::value<GoalZone>(&c.goal)->default_value({40, 40, 1}),
                     "Goal node coordinate as (x, y, r)")
                    ("bias,b", po::value<double>(&c.goal_bias)->default_value(0),
                     "The goal bias. Probability in range [0, 1]")
                    ("epsilon,e", po::value<double>(&c.eps)->default_value(1.0),
                     "The maximum distance covered by a new trajectory during exploration.")
                    ("iter_lim,i", po::value<int>(&c.iter_lim)->default_value(1e8),
                     "The maximum number of exploration iterations allowed.")
                    ("rand_seed,r", po::value<int>(&c.r_seed)->default_value(-1),
                     "The seed for the random number generator. -1 would mean non-deterministically random.")
                    ("verbose,v", po::value<int>(&c.verbose)->implicit_value(1)->default_value(0),
                     "Verbosity. Choose between 0, 1, 2 and 3")
                    ("obs_fp,o", po::value<std::string>(&c.obs_fp)->default_value("obstacle.txt"),
                     "Relative/absolute path to the obstacle file")
                    ("path_fp,p", po::value<std::string>(&c.path_fp)->default_value("path_output.txt"),
                     "Relative/absolute path to the Path output file")
                    ("search_fp,t", po::value<std::string>(&c.search_fp)->default_value("search_output.txt"),
                     "Relative/absolute path to the Search output file")
                    ;

            po::variables_map vm;
            po::store(po::parse_command_line(argc, argv, desc), vm);
            po::notify(vm);

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

    void read_obstacles(const std::string &f_name, ObstacleVec &v, const int verbose = 0) {
        std::ifstream obs_file(f_name);
        if (obs_file.is_open()) {
            if (verbose) std::cout << "Successfully opened the obstacle file: " << f_name << std::endl;
            std::string line, temp;
            int num_obs = 0;
            int init_sz = v.size();
            double x = 0, y = 0, r = 0;
            CircleObstacle circle_obs;

            obs_file >> num_obs;
            if (verbose) std::cout << "The obstacle file has " << num_obs << " obstacles." << std::endl;
            while (!obs_file.eof()) {
                obs_file >> x >> temp >> y >> temp >> r;
                if (obs_file.good()) {
                    circle_obs = {x, y, r};
                    v.emplace_back(circle_obs);
                }
            }
            obs_file.close();
            if (verbose > 2) std::cout << v << std::endl;
            if (verbose) std::cout << v.size() - init_sz << " obstacles were added to obstacle vector.\n" << std::endl;
        } else if (verbose) std::cout << "Could not read the file \"" << f_name << "\".\n" << std::endl;
    }

    void write_path(const std::string &file_name, const int goal_node_id, const Graph &g, const int verbose = 0) {
        std::ofstream path_file(file_name);
        if (path_file.is_open()) {
            if (verbose) std::cout << "Successfully opened the path file: " << file_name << std::endl;
            int c_node = goal_node_id;
            std::string wayp;
            while (c_node != -1) {
                wayp = std::to_string(g[c_node].node_id + 1) + ", " +
                       std::to_string(g[c_node].node.x) + ", " + std::to_string(g[c_node].node.y);
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
                entry = std::to_string(g[*v].node_id + 1) + ", " + std::to_string(g[*v].node.x) + ", " +
                        std::to_string(g[*v].node.y) + ", " + std::to_string(p + 1) + ", " +
                        std::to_string(g[p].node.x) + ", " + std::to_string(g[p].node.y);
                if (verbose > 2) std::cout << entry << std::endl;
                graph_file << entry << std::endl;
            }
            graph_file.close();
            if (verbose) std::cout << "Successfully wrote graph file: \"" << file_name << "\".\n" << std::endl;
        } else if (verbose) std::cout << "Could not open the file \"" << file_name << "\".\n" << std::endl;
    }
}
#endif //RRT_2D_UTILS_H
