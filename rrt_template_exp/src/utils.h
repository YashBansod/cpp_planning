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
#include <fstream>

#include "boost/program_options.hpp"
#include "graph_def.h"

namespace po = boost::program_options;

/*------------------------------------------            Definitions         ------------------------------------------*/
#ifndef RRT_UTILS_H
#define RRT_UTILS_H
namespace rrt {
    template<typename T>
    struct Config {
        int verbose = 0, iter_lim = 1e8;
        T start, goal;
        double eps = 1, radius = 1, goal_bias = 0.0;
        std::string obs_fp, path_fp, search_fp;
    };

    template<typename T>
    inline std::ostream &operator<<(std::ostream &o_str, const Config<T> &c) {
        o_str << "start: " << c.start << ", goal: " << c.goal << ", radius: " << c.radius << ", bias: " << c.goal_bias
            << ", epsilon: " << c.eps <<", verbose: " << c.verbose << ", iter_lim: " << c.iter_lim <<
              "\nobs_fp: " << c.obs_fp << "\npath_fp: " << c.path_fp << "\nsearch_fp: " << c.search_fp << " ";
        return o_str;
    }

    bool parse_args(int argc, char **argv, Config<Point2D>& c){
        try {
            po::options_description desc("Allowed options", 120, 60);

            desc.add_options()
                    ("help,h", "Print the help message\n")
                    ("start,s", po::value<Point2D>(&c.start)->default_value({0, 0}),
                     "Start node coordinate as (x, y)")
                    ("goal,g", po::value<Point2D>(&c.goal)->default_value({40, 40}),
                     "Goal node coordinate as (x, y)")
                    ("radius,r", po::value<double>(&c.radius)->default_value(1.0),
                     "Trajectories ending in region covered by goal radius are accepted.")
                    ("bias,b", po::value<double>(&c.goal_bias)->default_value(0),
                     "The goal bias. Probability in range [0, 1]")
                    ("epsilon,e", po::value<double>(&c.eps)->default_value(1.0),
                     "The maximum distance covered by a new trajectory during exploration.")
                    ("iter_lim,i", po::value<int>(&c.iter_lim)->default_value(1e8),
                     "The maximum number of exploration iterations allowed.")
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

    void read_obstacles_2d(const std::string &f_name, ObstacleVec<CircleObstacle<Point2D>> &v, const int verbose = 0) {
        std::ifstream obs_file(f_name);
        if (obs_file.is_open()) {
            if (verbose) std::cout << "Successfully opened the obstacle file: " << f_name << std::endl;
            std::string line, temp;
            int num_obs = 0;
            int init_sz = v.size();
            double x = 0, y = 0, r = 0;
            CircleObstacle<Point2D> circle_obs;

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
}
#endif //RRT_UTILS_H
