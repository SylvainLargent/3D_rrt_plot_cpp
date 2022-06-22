#include <iostream>
#include <cmath>
#include "Point3.hpp"
#include "Segment.hpp"
#include "Env.hpp"
#include "RRT.hpp"
#include "Node.hpp"
#include <fstream>



int main(int argc, char ** argv){
    Env environment = Env(-3, 3, -1, 8, 0, 3, 0.25, 0.25, 0.099);
//Rectangles obstacles (Coordonnées du coin le plus proche de l'origine, épaisseur suivant les 3 directions x,y,z)
    vector<vector<double>> list_rectangles =
        {
            {-3, 3, 0, 2.70, 2, 3},
            {0.7, 3, 0, 2.3, 2, 3}
        };
//Obstacles
    // vector<vector<double>> list_rectangles =
    //     {
    //         {-3, 4, 0, 6, 1, 2},
    //         {-3, 1, 1, 6, 1, 3}
    //     };
    //Point de départ et destination
    Point3 s_start = Point3(0,0,1.25);
    Point3 s_goal  = Point3(0,5.5,1.25);

    //Number of iterations and step length at each step
    int iter_max = atoi(argv[1]);
    double step_len = atof(argv[2]);
    double goal_sample_rate = 0.1; //Pour l'instant inutil car mal compris
    
//Adding rectangles to the scene
    for(int i = 0; i < list_rectangles.size(); ++i){
        environment.add_rectangle(list_rectangles[i]);
    }

RRT rrt_algo = RRT(environment, s_start, s_goal, step_len, goal_sample_rate,iter_max);


//Giving out, arena or scene necessary info !
    ofstream arena_boundaries;
    arena_boundaries.open("boundaries.txt");
    arena_boundaries << environment.get_x_min() << std::endl;
    arena_boundaries << environment.get_x_max() << std::endl;
    arena_boundaries << environment.get_y_min() << std::endl;
    arena_boundaries << environment.get_y_max() << std::endl;
    arena_boundaries << environment.get_z_min() << std::endl;
    arena_boundaries << environment.get_z_max() << std::endl;
    arena_boundaries.close();

    ofstream start_destination;
    start_destination.open("start_destination.txt");
    start_destination << (*rrt_algo.ps_start) << std::endl;
    start_destination << (*rrt_algo.ps_goal) << std::endl;   
    start_destination.close();  
    


//Plotting rectangle obstacles thanks to their vertices
    vector<vector<Node*>> all_boundaries = rrt_algo.env.get_obstacles_vertex();  
    ofstream all_boundaries_stream;
    all_boundaries_stream.open("obstacles.txt");
    if(!all_boundaries.empty()){
        for(int i = 0; i < all_boundaries.size(); ++i){
            for(int j = 0; j < all_boundaries[i].size();++j){
                all_boundaries_stream << (*(all_boundaries[i][j])) << std::endl;
                delete(all_boundaries[i][j]);
            }
        }
    }
    all_boundaries_stream.close();

//Plotting path trajectory
    vector<Node*> path = rrt_algo.planning();  
    ofstream path_stream;
    path_stream.open("trajectory.txt");
    if(!path.empty()){
        for(int i = 0; i < path.size(); ++i){
            path_stream << (*path[i]) << std::endl;
        }
        path_stream << rrt_algo.iter_goal << std::endl;  
    }
    path_stream.close();
    if(!path.empty()){
        std::cout << "Path length : " << get_path_length(path) << std::endl;
    }


//Plotting tree 
    vector<Node*> tree = rrt_algo.tree;  
    ofstream tree_stream;
    tree_stream.open("tree.txt");
    if(!tree.empty()){
        for(int i = 0; i < tree.size(); ++i){
            tree_stream << (*tree[i]) << std::endl;
        }
    }
    tree_stream.close();


    return 0;
}
