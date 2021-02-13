/*!
 * @brief
 *
 * @file
 *
 * @ingroup     a-star
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include <iostream>
#include <string>
#include "boost/graph/adjacency_list.hpp"

/*------------------------------------------            Definitions         ------------------------------------------*/
#ifndef A_STAR_GRAPH_DEF_H
#define A_STAR_GRAPH_DEF_H
namespace a_star{

    enum node_stat {undef, open, closed};
    const char* labels[] = {"undef", "opened", "closed"};

    struct Node{
        int node_id = -1;
        float x = 0, y = 0;
        int p_node_id = -1;
        node_stat status = undef;
        float g_cost = -1, h_cost = 0;
    };

    inline std::ostream& operator<<(std::ostream& o_str, const Node& n){
        o_str << "node_id: "<< n.node_id + 1 << ", x: " << n.x << ", y: " << n.y << ", p_node_id: " << n.p_node_id + 1
                << ", status: " << labels[n.status] << ", g_cost: " << n.g_cost << ", h_cost: " << n.h_cost << " ";
        return o_str;
    }
    /*
    inline bool operator < (const Node& n1, const Node& n2){
        return n1.g_cost + n1.h_cost < n2.g_cost + n2.h_cost;
    }
    inline bool operator > (const Node& n1, const Node& n2){
        return n1.g_cost + n1.h_cost > n2.g_cost + n2.h_cost;
    }
    inline bool operator <= (const Node& n1, const Node& n2){
        return n1.g_cost + n1.h_cost <= n2.g_cost + n2.h_cost;
    }
    inline bool operator >= (const Node& n1, const Node& n2){
        return n1.g_cost + n1.h_cost >= n2.g_cost + n2.h_cost;
    }
    inline bool operator == (const Node& n1, const Node& n2){
        return n1.g_cost + n1.h_cost == n2.g_cost + n2.h_cost;
    }
    inline bool operator != (const Node& n1, const Node& n2){
        return n1.g_cost + n1.h_cost != n2.g_cost + n2.h_cost;
    }
    */
    struct Edge{
        int source = -1, target=-1;
        float cost = -1;
    };

    inline std::ostream& operator<<(std::ostream& o_str, const Edge& e){
        o_str << "source: " << e.source + 1 << ", target: " << e.target + 1 << ", cost: " << e.cost << " ";
        return o_str;
    }
    /*
    bool operator < (const Edge& e1, const Edge& e2){
        return e1.cost < e2.cost;
    }
    bool operator > (const Edge& e1, const Edge& e2){
        return e1.cost > e2.cost;
    }
    bool operator <= (const Edge& e1, const Edge& e2){
        return e1.cost <= e2.cost;
    }
    bool operator >= (const Edge& e1, const Edge& e2){
        return e1.cost >= e2.cost;
    }
    bool operator == (const Edge& e1, const Edge& e2){
        return e1.cost == e2.cost;
    }
    bool operator != (const Edge& e1, const Edge& e2){
        return e1.cost != e2.cost;
    }
    */

    typedef boost::adjacency_list<boost::vecS,  boost::vecS, boost::bidirectionalS, Node, Edge> Graph;

}
#endif //A_STAR_GRAPH_DEF_H
