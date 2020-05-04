
#include "graph.hpp"
#include <iostream>
#include <vector>
#include <chrono>

using namespace std::chrono;

int main(int argc, char *argv[]){
    Graph graph;
    
    // defines a two dimensional grid with impassable squares
    constexpr int grid_size = 10;
    std::array<std::array<bool,grid_size>,grid_size> grid {{
        {{0,0,0,0,0,1,0,0,0,0}},
        {{0,1,0,0,1,0,0,0,0,0}},
        {{0,0,0,1,0,0,1,0,0,0}},
        {{0,1,1,1,1,0,1,0,0,0}},
        {{0,1,0,1,0,0,0,0,0,0}},
        {{0,1,0,0,0,0,1,1,1,0}},
        {{0,1,0,0,0,0,1,1,0,0}},
        {{1,1,0,0,0,0,0,0,0,1}},
        {{1,0,0,0,0,0,0,0,0,0}},
        {{0,0,0,0,0,0,0,0,1,0}}
    }};
    
    // connects nearby vertices in the grid
    for(int i=0;i<grid_size;i++) {
        for(int j=0;j<grid_size;j++) {
            if(grid[i][j]==0) {
                graph.position_vertex(grid_size*i+j,i,j);
                if(i<grid_size-1 and grid[i+1][j]==0) graph.add_edge(grid_size*i+j,grid_size*(i+1)+j,1);
                if(i<grid_size-1 and j<grid_size-1 and grid[i+1][j+1]==0) graph.add_edge(grid_size*i+j,(grid_size*(i+1))+j+1,sqrt(2));
                if(j<grid_size-1 and grid[i][j+1]==0) graph.add_edge(grid_size*i+j,grid_size*i+j+1,1);
                if(i<grid_size-1 and j>0 and grid[i+1][j-1]==0) graph.add_edge(grid_size*i+j,(grid_size*(i+1))+j-1,sqrt(2));
            }
        }
    }
    
    // finds a path between the nodes on the opposite diagonal and improves on this with paths up to 3 edges
    try {
        auto timer_start = high_resolution_clock::now();
        auto path = graph.a_star_pathfinder(0, 99);
        auto timer_mid = high_resolution_clock::now();
        auto improved_path = graph.make_shortcuts(path, 3);
        auto timer_stop = high_resolution_clock::now();
        
        auto duration_pathfinder = duration_cast<microseconds>(timer_mid - timer_start);
        auto duration_path_improver = duration_cast<microseconds>(timer_stop - timer_mid);
        
        auto a_star_path_length = graph.path_length(path);
        auto improved_path_length = graph.path_length(improved_path);
        
        std::cout << "A* path:\n";
        for(const auto& vertex : path) std::cout << vertex << ", ";
        std::cout << "\n" << duration_pathfinder.count() << " microseconds\n";
        std::cout << "total path weight: " << a_star_path_length << "\n\n";
        
        std::cout << "Improved path:\n";
        for(const auto& vertex : improved_path) std::cout << vertex << ", ";
        std::cout << "\n" << duration_path_improver.count() << " microseconds\n";
        std::cout << "total path weight: " << improved_path_length << "\n\n";
        
    } catch(std::logic_error e) {
        std::cout << e.what();
    }
    
    return 0;
}
