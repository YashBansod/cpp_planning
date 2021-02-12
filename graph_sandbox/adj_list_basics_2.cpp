#include <iostream>
#include <vector>
#include <cmath>
#include "boost/graph/adjacency_list.hpp"

struct Node{
    int index;
    double x, y;
    int status;
};

struct Edge{
    double cost;
};

int main() {
    typedef boost::adjacency_list<boost::listS,  boost::listS, boost::bidirectionalS, Node, Edge> Graph;
    int num_vertices = 10;
    std::vector<double> x_cord = {2, 3, 4, 5, 6, 4, 3, 4, 1, 9};
    std::vector<double> y_cord = {4, 5, 6, 4, 3, 4, 1, 9, 6, 7};
    Graph g(num_vertices);

    auto x_cord_iter = x_cord.begin();
    auto y_cord_iter = y_cord.begin();
    int index = 0;

    std::cout << std::endl << "\t**** VERTEX DATA ****" << std::endl;
    for(auto [v, end_v] = boost::vertices(g); v !=end_v; ++v){
        if(x_cord_iter == x_cord.end() or y_cord_iter == y_cord.end()) break;
        g[*v].index = index++;
        g[*v].x = *x_cord_iter++;
        g[*v].y = *y_cord_iter++;
        g[*v].status = 0;
    }
    auto [v, end_v] = boost::vertices(g);
    for(; v !=end_v; --v){
        auto v_0 = v;
        auto v_1 = ++v;
        if(v == end_v) break;
        boost::add_edge(*v_0, *v_1, g);
        boost::add_edge(*v_1, *v_0, g);
        auto v_2 = ++v;
        if(v == end_v) break;
        boost::add_edge(*v_0, *v_2, g);
        boost::add_edge(*v_2, *v_0, g);
    }

    for(auto [v, end_v] = boost::vertices(g); v !=end_v; ++v){
        std::cout << "vertex: " << *v << ", with index: "<< g[*v].index << ", x_cord: " << g[*v].x <<
            ", y_cord: " << g[*v].y << ", status: " <<  g[*v].status << ", o_degree: " << boost::out_degree(*v, g) <<
            ", i_degree: " << boost::in_degree(*v, g) << std::endl;
    }

    std::cout << std::endl << "\t**** EDGE DATA ****" << std::endl;

    for(auto [e, end_e] = boost::edges(g); e != end_e; ++e){
        Graph::vertex_descriptor src_v = boost::source(*e, g);
        Graph::vertex_descriptor dst_v = boost::target(*e, g);
        g[*e].cost = std::sqrt(std::pow(g[dst_v].x - g[src_v].x, 2) + std::pow(g[dst_v].y - g[src_v].y, 2));
        std::cout << "edge: " << *e << ", from index: " << g[src_v].index << ", to index: "
            << g[dst_v].index << ", cost: " << g[*e].cost << std::endl;
    }

    std::cout << std::endl << "\t**** OTHER GRAPH DATA ****" << std::endl;
    std::cout << "Graph has " << boost::num_vertices(g) << " vertices" << std::endl;
    std::cout << "Graph has " << boost::num_edges(g) << " edges" << std::endl;

    return 0;
}
