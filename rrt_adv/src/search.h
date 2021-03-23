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

    inline bool collision_check_1(const Point2D &pa, const Point2D &pb, const CircleObstacle &c) {
        Point2D a = pa;
        Point2D b = pb;
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

    inline bool collision_check_2(const Point2D& a, const Point2D& b, const CircleObstacle &c) {
        const double ab_dist = a.eu_dist(b);
        const double dist_thresh = 0.5;
        if(a.eu_dist(c.center) < c.r) return true;
        if(b.eu_dist(c.center) < c.r) return true;
        if(ab_dist <= dist_thresh) return false;
        double ratio = dist_thresh / ab_dist;
        double const old_r = ratio;
        Point2D test;
        while(ratio < 1) {
            test.x = (1 - ratio) * a.x + ratio * b.x;
            test.y = (1 - ratio) * a.y + ratio * b.y;
            if (test.eu_dist(c.center) < c.r) return true;
            ratio += old_r;
        }
        return false;
    }

    inline bool collision_check_3(const Point2D& a, const Point2D& b, const CircleObstacle &c) {
        if(b.eu_dist(c.center) < c.r) return true;
        return false;
    }

//    inline RoboState forward_sim(const RoboState& r, const double accel, const double gamma, const double dt) {
//        const double min_c = -50, max_c = 50;
//        const double min_o = 0, max_o = 2 * M_Pi;
//        const double min_v = -5, max_v = 5;
//        const double min_w = -M_PI / 2, max_w = M_PI / 2;
//        RoboState new_r;
//        new_r.v = std::clamp(r.v + accel * dt, min_v, max_v);
//        new_r.w = std::clamp(r.w + gamma * dt, min_w, max_w);
//        new_r.o =
//    }

    inline RoboGeometry RotateRobot(RoboGeometry rob, const double theta){
        for (auto &r: rob){
            double x = r.x * std::cos(theta) - r.y * std::sin(theta);
            double y = r.x * std::sin(theta) + r.y * std::cos(theta);
            r.x = x;
            r.y = y;
        }
        return rob;
    }

    inline bool GeomInWorkspace(const Point2D& offset, const RoboGeometry& rob, const Workspace& w){
        for (auto &r: rob){
            double x = r.x + offset.x, y = r.y + offset.y;
            if((x < w.p1.x) or (x > w.p2.x)) return false;
            if((y < w.p1.y) or (y > w.p2.y)) return false;
        }
        return true;
    }

    std::pair<int, bool>
    search(const Point2_1D &start, const GoalZone &goal, Graph &g,
           Workspace &w_space, const ObstacleVec &obs_vec, const RoboGeometry &robot,
           const std::function<bool(const Point2D&, const Point2D&, const CircleObstacle&)> &collision_func,
           const double eps = 1, const int iter_lim = 1e8, const int verbose = 0) {

        auto s_id = boost::add_vertex(g);
        g[s_id].node_id = s_id;
        g[s_id].node = start;
        g[s_id].g_cost = 0;
        if (verbose > 1) std::cout << "Added Start Node: " << g[s_id] << std::endl;

        int g_id = -1;
        bool found_goal = false;

        RoboGeometry _start_geom = RotateRobot(robot, g[s_id].node.o);
        if (not GeomInWorkspace(g[s_id].node.p, _start_geom, w_space)) return {g_id, found_goal};

        // If goal is completely covered by an obstacle then return failure.
        for(auto& obs: obs_vec){
            if(obs.center.eu_dist(goal.center) <= std::max(obs.r, goal.r) - std::min(obs.r, goal.r))
                return {g_id, found_goal};
            for(auto& pt : _start_geom){
                bool collision = collision_check_3(g[s_id].node.p, g[s_id].node.p + pt, obs);
                if(collision) return {g_id, found_goal};
            }
        }

        for(int _iter = 0; (not found_goal and _iter < iter_lim); ++_iter){
            Point2D rand_pt = w_space.sample();
            auto[v, end_v] = boost::vertices(g);
            double min_dist = rand_pt.eu_dist(g[*v].node.p);
            int min_ind = *v++;
            for (; v != end_v; ++v) {
                double dist = rand_pt.eu_dist(g[*v].node.p);
                if (dist < min_dist) {
                    min_dist = dist;
                    min_ind = *v;
                }
            }
            if (min_dist > eps) {
                double ratio = eps / min_dist;
                rand_pt.x = (1 - ratio) * g[min_ind].node.p.x + ratio * rand_pt.x;
                rand_pt.y = (1 - ratio) * g[min_ind].node.p.y + ratio * rand_pt.y;
                min_dist = eps;
            }
            bool collision = false;
            double _theta = rad0_2pi(std::atan2(rand_pt.y - g[min_ind].node.p.y, rand_pt.x - g[min_ind].node.p.x));
            double d_theta = rad0_2pi(_theta - g[min_ind].node.o);

            RoboGeometry new_geom = RotateRobot(robot, _theta);
            if (not GeomInWorkspace(g[min_ind].node.p, new_geom, w_space)) continue;
            if (not GeomInWorkspace(rand_pt, new_geom, w_space)) continue;

            double c_theta = M_PI / 8;
            while(c_theta < d_theta){
                RoboGeometry _geom = RotateRobot(robot, rad0_2pi(c_theta + g[min_ind].node.o));
                if (not GeomInWorkspace(g[min_ind].node.p, _geom, w_space)){collision = true; break;}
                for(auto& obs : obs_vec){
                    if(collision) break;
                    for(auto& pt : _geom){
                        collision = collision_check_3(g[min_ind].node.p, g[min_ind].node.p + pt, obs);
                        if(collision) break;
                    }
                }
                if(collision) break;
                c_theta += M_PI / 8;
            }

            if(collision) continue;
            for(auto& obs : obs_vec){
                for(auto& pt : new_geom){
                    collision = collision_check_3(g[min_ind].node.p, g[min_ind].node.p + pt, obs);
                    if(collision) break;
                }
                if(collision) break;
            }

            if(collision) continue;
            for(auto& obs : obs_vec){
                for(auto& pt : new_geom) {
                    collision = collision_func(g[min_ind].node.p + pt, rand_pt + pt, obs);
                    if (collision) break;
                }
                if(collision) break;
            }

            if(not collision) {
                auto n_id = boost::add_vertex(g);
                g[n_id].node_id = n_id;
                g[n_id].node.p = rand_pt;
                g[n_id].node.o = _theta;
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


