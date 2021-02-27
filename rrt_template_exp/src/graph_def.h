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
#include <vector>
#include <random>
#include "boost/graph/adjacency_list.hpp"

/*------------------------------------------            Definitions         ------------------------------------------*/
#ifndef RRT_GRAPH_DEF_H
#define RRT_GRAPH_DEF_H
namespace rrt {

    struct Point2D {
        double x, y;
        Point2D() : x(0), y(0) {}
        Point2D(double _x, double _y) : x(_x), y(_y) {}
        double eu_dist(const Point2D& p2){return std::sqrt(std::pow(p2.x - x, 2) + std::pow(p2.y - y, 2));}
    };

    inline std::ostream &operator<<(std::ostream &o_str, const Point2D &p) {
        o_str << "x: " << p.x << ", y: " << p.y << " ";
        return o_str;
    }

    inline std::istream &operator>>(std::istream &i_str, Point2D &p) {
        char temp;
        i_str >> temp >>  p.x >> temp >> p.y >> temp;
        return i_str;
    }

    struct Point3D {
        double x, y, z;
        Point3D() : x(0), y(0), z(0){}
        Point3D(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
        double eu_dist(const Point3D& p2){
            return std::sqrt(std::pow(p2.x - x, 2) + std::pow(p2.y - y, 2) + std::pow(p2.z - z, 2));
        }
    };

    inline std::ostream &operator<<(std::ostream &o_str, const Point3D &p) {
        o_str << "x: " << p.x << ", y: " << p.y << ", z: " << p.z << " ";
        return o_str;
    }

    inline std::istream &operator>>(std::istream &i_str, Point3D &p) {
        char temp;
        i_str >> temp >>  p.x >> temp >> p.y >> temp >> p.z >> temp;
        return i_str;
    }

    template<typename T>
    struct Workspace {
        T p1, p2;
        Workspace() : p1(), p2(){}
        Workspace(const Point2D& _p1, const Point2D& _p2) : p1(_p1), p2(_p2) {}
    };

    class RandUniformPoint2D{
    public:
        RandUniformPoint2D(Workspace<Point2D>& w){
            std::random_device rd;
            gen = std::mt19937(rd());
            dist_x = std::uniform_real_distribution<>(w.p1.x, w.p2.x);
            dist_y = std::uniform_real_distribution<>(w.p1.y, w.p2.y);
        }
        Point2D sample(){return Point2D(dist_x(gen), dist_y(gen));}
    private:
        std::mt19937 gen;
        std::uniform_real_distribution<> dist_x;
        std::uniform_real_distribution<> dist_y;
    };

    class RandUniformPoint3D{
    public:
        RandUniformPoint3D(Workspace<Point3D>& w){
            std::random_device rd;
            gen = std::mt19937(rd());
            dist_x = std::uniform_real_distribution<>(w.p1.x, w.p2.x);
            dist_y = std::uniform_real_distribution<>(w.p1.y, w.p2.y);
            dist_z = std::uniform_real_distribution<>(w.p1.z, w.p2.z);
        }
        Point3D sample(){return Point3D(dist_x(gen), dist_y(gen), dist_z(gen));}
    private:
        std::mt19937 gen;
        std::uniform_real_distribution<> dist_x;
        std::uniform_real_distribution<> dist_y;
        std::uniform_real_distribution<> dist_z;
    };

    template <typename T>
    inline std::ostream &operator<<(std::ostream &o_str, const Workspace<T> &w) {
        o_str << "lim point 1: " << w.p1 << ", lim point 2: " << w.p2 << " ";
        return o_str;
    }


    template <typename T>
    struct Node {
        int node_id = -1;
        T node;
        int p_node_id = -1;
        double g_cost = -1;
    };

    template <typename T>
    inline std::ostream &operator<<(std::ostream &o_str, const Node<T> &n) {
        o_str << "node_id: " << n.node_id << ", " << n.node <<
              ", p_node_id: " << n.p_node_id << ", g_cost: " << n.g_cost << " ";
        return o_str;
    }

    struct Edge {
        int source = -1, target = -1;
        double cost = -1;
    };

    inline std::ostream &operator<<(std::ostream &o_str, const Edge &e) {
        o_str << "source: " << e.source << ", target: " << e.target << ", cost: " << e.cost << " ";
        return o_str;
    }

    template<typename T1, typename T2>
    using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, T1, T2>;

    template <typename T>
    struct CircleObstacle {
        T center;
        double r;
        CircleObstacle() : r(0){};
        CircleObstacle(double _x, double _y, double _r) : center({_x, _y}), r(_r){}
        CircleObstacle(double _x, double _y, double _z, double _r) : center({_x, _y, _z}), r(_r){}
    };

    template <typename T>
    inline std::ostream &operator<<(std::ostream &o_str, const CircleObstacle<T> &c) {
        o_str << c.center << ", r: " << c.r << " ";
        return o_str;
    }

    template <typename T>
    using ObstacleVec = std::vector<T>;

    template <typename T>
    inline std::ostream &operator<<(std::ostream &o_str, const ObstacleVec<T> &v) {
        for(auto& x: v) o_str << x << std::endl;
        return o_str;
    }
}

#endif //RRT_GRAPH_DEF_H
