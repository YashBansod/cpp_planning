/*!
 * @brief
 *
 * @file
 *
 * @ingroup     rrt_adv
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include <iostream>
#include <string>

#include "graph_def.h"
#include "utils.h"

/*------------------------------------------        Main Function           ------------------------------------------*/
int main(int argc, char **argv) {

    std::string ip_path, op_path;
    int verbose;
    try {
        po::options_description desc("Allowed options", 120, 60);

        desc.add_options()
                ("help,h", "Print the help message\n")
                ("verbose,v", po::value<int>(&verbose)->implicit_value(1)->default_value(0),
                 "Verbosity. Choose between 0, 1, 2 and 3")
                ("ip_path,i", po::value<std::string>(&ip_path)->default_value("path_output.txt"),
                 "Relative/absolute path to the input path file")
                ("op_path,o", po::value<std::string>(&op_path)->default_value("traj_output.txt"),
                 "Relative/absolute path to the input path file");

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if(verbose) std::cout << "v:" << verbose << ", i: " << ip_path << ", o: " << op_path << std::endl << std::endl;

        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }
    }
    catch(std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return -1;
    }
    catch(...) {
        std::cerr << "Exception of unknown type!\n";
        return -2;
    }

    rrt::PathVec p_vec;
    rrt::read_path(ip_path, p_vec, verbose);

    const double max_v = 5, max_a = 2;
    const double max_w = M_PI / 2, max_g = M_PI / 2;
    const double t1_thresh = max_v / max_a;
    const double t2_thresh = max_w / max_g;
    const double s_thresh = max_a * t1_thresh * t1_thresh;
    const double th_thresh = max_g * t2_thresh * t2_thresh;

    rrt::TrajVec t_vec;
    double t_sum = 0;

    for(int i = 1; i < p_vec.size(); ++i){
        double s = p_vec[i].p.eu_dist(p_vec[i - 1].p);
        double th = rrt::rad0_2pi(p_vec[i].o - p_vec[i-1].o);
        double _x = p_vec[i - 1].p.x, _y = p_vec[i - 1].p.y, _th = p_vec[i - 1].o;
        double _th2 = p_vec[i].o;
        // angular acceleration till half of th covered, then decelerate till full th is covered.
        if(th <= th_thresh){
            double dt = std::sqrt(th/max_g);
            t_vec.emplace_back(rrt::TrajPoint(t_sum, rrt::RoboState(_x, _y, _th, 0, 0), rrt::ControlInput(0, max_g)));
            t_sum += dt;
            _th = rrt::rad0_2pi(_th + th / 2);
            t_vec.emplace_back(rrt::TrajPoint(t_sum, rrt::RoboState(_x, _y, _th, 0, max_g * dt), rrt::ControlInput(0, -max_g)));
            t_sum += dt;
        }
        // Accelerate to max angular velocity, then maintain for some time, then decelerate till angular velocity is 0.
        else{
            double dt1 = std::sqrt(th_thresh/max_g);
            double dt2 = (th - th_thresh) / max_w;
            t_vec.emplace_back(rrt::TrajPoint(t_sum, rrt::RoboState(_x, _y, _th, 0, 0), rrt::ControlInput(0, max_g)));
            t_sum += dt1;
            _th = rrt::rad0_2pi(_th + th_thresh / 2);
            t_vec.emplace_back(rrt::TrajPoint(t_sum, rrt::RoboState(_x, _y, _th, 0, max_w), rrt::ControlInput(0, 0)));
            t_sum += dt2;
            _th = rrt::rad0_2pi(_th + dt2 * max_w);
            t_vec.emplace_back(rrt::TrajPoint(t_sum, rrt::RoboState(_x, _y, _th, 0, max_w), rrt::ControlInput(0, -max_g)));
            t_sum += dt1;
        }

        // linear acceleration till half of s covered, then decelerate till full s is covered.
        if(s <= s_thresh){
            double dt = std::sqrt(s/max_a);
            t_vec.emplace_back(rrt::TrajPoint(t_sum, rrt::RoboState(_x, _y, _th2, 0, 0), rrt::ControlInput(max_a, 0)));
            t_sum += dt;
            _x = 0.5 * p_vec[i- 1].p.x + 0.5 * p_vec[i].p.x;
            _y = 0.5 * p_vec[i- 1].p.y + 0.5 * p_vec[i].p.y;
            t_vec.emplace_back(rrt::TrajPoint(t_sum, rrt::RoboState(_x, _y, _th2, max_a * dt, 0), rrt::ControlInput(-max_a, 0)));
            t_sum += dt;
        }
        // Accelerate to max linear velocity, then maintain for some time, then decelerate till linear velocity is 0.
        else{
            double dt1 = std::sqrt(s_thresh/max_a);
            double dt2 = (s - s_thresh) / max_v;
            double ratio = 0.5 * s_thresh/s;
            t_vec.emplace_back(rrt::TrajPoint(t_sum, rrt::RoboState(_x, _y, _th2, 0, 0), rrt::ControlInput(max_a, 0)));
            t_sum += dt1;
            _x = (1-ratio) * p_vec[i- 1].p.x + ratio * p_vec[i].p.x;
            _y = (1-ratio) * p_vec[i- 1].p.y + ratio * p_vec[i].p.y;
            t_vec.emplace_back(rrt::TrajPoint(t_sum, rrt::RoboState(_x, _y, _th2, max_v, 0), rrt::ControlInput(0, 0)));
            t_sum += dt2;
            ratio = (s - 0.5 * s_thresh)/s;
            _x = (1-ratio) * p_vec[i- 1].p.x + ratio * p_vec[i].p.x;
            _y = (1-ratio) * p_vec[i- 1].p.y + ratio * p_vec[i].p.y;
            t_vec.emplace_back(rrt::TrajPoint(t_sum, rrt::RoboState(_x, _y, _th2, max_v, 0), rrt::ControlInput(-max_a, 0)));
            t_sum += dt1;
        }
    }
    if(not p_vec.empty())
        t_vec.emplace_back(rrt::TrajPoint(t_sum, rrt::RoboState(p_vec.back(), 0, 0), rrt::ControlInput(0, 0)));

    rrt::write_trajectory(op_path, t_vec, verbose);
    return 0;
}
