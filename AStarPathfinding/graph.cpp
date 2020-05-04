
#include "graph.hpp"

const static double INF = std::numeric_limits<double>::infinity();
const static double max_rounding_error_factor = 1 + std::numeric_limits<double>::epsilon() * 2;

// used in std::accumulate to determine total path weight
double Graph::weight_accumulator(const double lhs, const int& rhs) const {
    const auto edges = edge_lists[rhs];
    auto edge_it = find_if(edges.begin(), edges.end(), [&](Edge edge){ return edge.other == *(&rhs + 1); });
    if(edge_it == edges.end()) throw std::invalid_argument("vertices are not connected\n");
    return lhs + edge_it->weight;
};

// ensures that the graph can hold a vertex with id x internally
void Graph::accommodate(int x) {
    if(x >= vertices.size()) {
        edge_lists.resize(x + 1);
        vertices.resize(x + 1);
    }
}

// adds a weighted connection between two graph vertices
void Graph::add_edge(int vertex_a, int vertex_b, double weight) {
    assert(weight > 0 and vertex_a != vertex_b and vertex_a >= 0 and vertex_b >= 0);
    accommodate(std::max(vertex_a, vertex_b));
    edge_lists[vertex_a].push_back({vertex_b, weight});
    edge_lists[vertex_b].push_back({vertex_a, weight});
}

// positions a vertex in space for use in the heuristic
void Graph::position_vertex(int vertex, double x, double y, double z) {
    assert(vertex >= 0);
    
    accommodate(vertex);
    vertices[vertex] = {x, y, z};
}

// determines the absolute Euclidean distance between two vertices
double Graph::heuristic(int vertex_a, int vertex_b) const noexcept {
    assert(vertex_a < vertices.size() and vertex_b < vertices.size());
    
    if(vertex_a == vertex_b) return 0;
    const auto displacement = vertices[vertex_a] - vertices[vertex_b];
    return sqrt((displacement * displacement).sum());
}

// constructs the path from origin to destination using information found in the pathfinding search
std::vector<int> Graph::reconstruct_path(const std::vector<int>& came_from, const int origin, int const destination) {
    std::vector<int> path;
    for(int i = destination; origin != i; i = came_from[i])
        path.push_back(i);
    path.push_back(origin);
    std::reverse(path.begin(), path.end());
    return path;
}

// uses A*-pathfinding to find a path between origin and destination vertices
std::vector<int> Graph::a_star_pathfinder(const int origin, const int destination) const {
    assert(std::max(origin, destination) < vertices.size());
    
    std::vector<double> origin_distance(vertices.size(), INF);
    std::vector<double> destination_distance(vertices.size(), INF);
    std::vector<int> came_from(vertices.size());
    std::vector<bool> priority_queue_has(vertices.size(), false);
    
    origin_distance[origin] = 0;
    destination_distance[origin] = heuristic(origin, destination);
    auto compare_heuristic = [&](int vertex_a, int vertex_b) { return destination_distance[vertex_a] > destination_distance[vertex_b]; };
    std::priority_queue<int, std::vector<int>, decltype(compare_heuristic)> priority_queue(compare_heuristic);
    priority_queue.push(origin);
    priority_queue_has[origin] = true;
    while(!priority_queue.empty()) {
        const int vertex = priority_queue.top();
        priority_queue.pop();
        priority_queue_has[vertex] = false;
        if(vertex == destination) return reconstruct_path(came_from, origin, destination);
        for(const auto& edge : edge_lists[vertex]) {
            double tentative = origin_distance[vertex] + edge.weight;
            if (tentative < origin_distance[edge.other]) {
                came_from[edge.other] = vertex;
                origin_distance[edge.other] = tentative;
                destination_distance[edge.other] = origin_distance[edge.other] + heuristic(edge.other, destination);
                if(!priority_queue_has[edge.other]) {
                    priority_queue.push(edge.other);
                    priority_queue_has[edge.other] = true;
                }
            }
        }
    }
    throw std::invalid_argument("vertices are not connected\n");
}

// performs short detours from the path tree-recursively
// when a detour intersect the path, determines if the detour is a shortcut or a more scenic route and amends the path
// no effect when heuristic is admissible
void Graph::improve_path_helper(Edge next, double weight, int path_idx, std::vector<int> detour, std::vector<int>& path, int depth) const {
    if(find(detour.begin(), detour.end(), next.other) != detour.end()) return;
    detour.push_back(next.other);
    const auto junction = find(path.begin(), path.end(), detour.back());
    if(junction == path.end()) {
        if(depth < 1) return;
        for(const auto& edge : edge_lists[detour.back()])
            improve_path_helper(edge, weight + next.weight, path_idx, detour, path, depth - 1);
    } else if(junction > path.begin() + path_idx) {
        const double path_weight = accumulate(path.begin() + path_idx, junction, 0.0,
                                              [&](const double lhs, const int& rhs) {return this->weight_accumulator(lhs,rhs);});
        const double detour_weight = weight + next.weight;
        if(path_weight > detour_weight * max_rounding_error_factor) {
            path.erase(path.begin() + path_idx, junction);
            path.insert(path.begin() + path_idx, detour.begin(), detour.end() - 1);
        }
    }
}

// performs all shortcuts
std::vector<int> Graph::make_shortcuts(std::vector<int> path, int depth) const {
    assert(depth > 1 and depth < vertices.size());
    if(any_of(path.begin(),path.end(), [&]
              (int vertex_id) { return vertex_id >= vertices.size(); })) {
        throw std::invalid_argument("path contains vertex not in graph\n");
    }
    
    std::vector<int> copy;
    do {
        copy = path;
        for (int i = 0; i < path.size(); i++)
            for(const auto& edge : edge_lists[path[i]])
                improve_path_helper(edge, 0, i, { path[i] }, path, depth - 1);
    } while (copy != path);
    return path;
}

// returns sum of weights between vertices along a path
double Graph::path_length(std::vector<int> path) const {
    return accumulate(path.begin(), path.end()-1, 0.0, [&](const double lhs, const int& rhs) { return weight_accumulator(lhs,rhs); });
}
