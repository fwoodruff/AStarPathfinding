
#ifndef graph_hpp
#define graph_hpp

#include <list>
#include <vector>
#include <queue>
#include <valarray>
#include <numeric>
#include <algorithm>
#include <array>
#include <limits>

// connects numbered graph vertices
class Edge {
public:
    int other;
    double weight;
};

// defines a graph with weighted edges between numbered vertices
// paths between vertices can be queried
class Graph {
private:
    std::vector<std::list<Edge>> edge_lists;
    std::vector<std::valarray<double>> vertices;
    
    [[nodiscard]] double heuristic(int vertex_a, int vertex_b) const noexcept;
    void accommodate(int n_vertices);
    void improve_path_helper(Edge next, double weight, int path_idx, std::vector<int> detour, std::vector<int>& path, int depth) const;
    [[nodiscard]] static std::vector<int> reconstruct_path(const std::vector<int>& came_from, int origin, int destination);
    double weight_accumulator(const double lhs, const int& rhs) const;
public:
    double path_length(std::vector<int> path) const;
    void add_edge(int vertex_a, int vertex_b, double weight);
    void position_vertex(int vertex, double x, double y, double z = 0);
    [[nodiscard]] std::vector<int> a_star_pathfinder(int vertex_origin, int vertex_destination) const;
    [[nodiscard]] std::vector<int> make_shortcuts(std::vector<int> path, int depth) const;
};

#endif /* graph_hpp */
