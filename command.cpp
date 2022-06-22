#include <iostream>
#include <cmath>
#include "Point3.hpp"
#include "Segment.hpp"
#include "Env.hpp"
#include "RRT.hpp"
#include "Node.hpp"
#include <fstream>

vector<vector<double>> determine_cmd(vector<Node*> path, double speed_norm){
    double dist;
    Node* parent;
    Node* child;
    Point3 dir;
    double vx, vy, vz;
    vector<vector<double>> list_of_velocities;
    for(int i = path.size()-2; i > -1; --i){
            child = path[i];
            parent = path[i+1];
            dist = distance(parent,child);
            dir  = unit_vector((child->p-parent->p));
            vx = dir[0]*speed_norm;
            vy = dir[1]*speed_norm;
            vz = dir[2]*speed_norm;
            vector<double> velocity;
            velocity.push_back(vx);
            velocity.push_back(vy);
            velocity.push_back(vz);
            list_of_velocities.push_back(velocity);
    }
    return list_of_velocities;
}

vector<double> determine_time(vector<Node*> path, double speed_norm){
    double dist;
    Node* parent;
    Node* child;
    Point3 dir;
    double time;
    vector<double> pduration_list;
    for(int i = 1; i < path.size(); ++i){
            child = path[i];
            parent = path[i-1];
            dist = distance(parent,child);
            dir  = unit_vector((child->p-parent->p));
            time = dist/speed_norm;
            pduration_list.push_back(time);
    }
    return pduration_list;
}



int main(int argc, char ** argv){
    double speed = 0.1; //
    Env environment = Env(-2, 2, -1, 8, 0, 1.5, 0.25, 0.25, 0.099);
//Rectangles obstacles (Coordonnées du coin le plus proche de l'origine, épaisseur suivant les 3 directions x,y,z)
    // Couloir étroit
    // vector<vector<double>> list_rectangles =
    //     {
    //         {-1, 3, -1, 4.5, 2, 10.5},
    //         {4.5, 3, -1, 5.25, 2, 10.5}
    //     };


    //Simple rectangle
     vector<vector<double>> list_rectangles =
        {
            {-3, 3, 0, 6, 1.5, 0.5}
        };
    //Point de départ et destination
    Point3 s_start = Point3(0,0,0.1);
    Point3 s_goal  = Point3(0,6,0.1);

    //Number of iterations and step length at each step
    int iter_max = atoi(argv[1]);
    double step_len = atof(argv[2]);
    double goal_sample_rate = 0; //Pour l'instant inutil car mal compris
    
//Adding rectangles to the scene
    for(int i = 0; i < list_rectangles.size(); ++i){
        environment.add_rectangle(list_rectangles[i]);
    }

    RRT rrt_algo = RRT(environment, s_start, s_goal, step_len, goal_sample_rate,iter_max);

    vector<Node*> path = rrt_algo.planning();  
//Commands !
    vector<double>  pduration_list = determine_time(path, speed);
    vector<vector<double>> list_of_velocities = determine_cmd(path, speed);

//Speed data
    ofstream velocity_stream;
    velocity_stream.open("velocity.txt");
    if(!list_of_velocities.empty()){
        for(int i = 0; i < list_of_velocities.size(); ++i){
            for(int j = 0; j < list_of_velocities[i].size(); ++j){ 
            velocity_stream << list_of_velocities[i][j] << ( (j==2)? "" : ", " );
        }
        velocity_stream << std::endl;  
        }
    }
    velocity_stream.close();


    for(int i = 0; i < pduration_list.size(); ++i){
        std::cout << pduration_list[i] << std::endl;
    }


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
    ofstream path_stream;
    path_stream.open("trajectory.txt");
    if(!path.empty()){
        for(int i = 0; i < path.size(); ++i){
            path_stream << (*path[i]) << std::endl;
        }
        path_stream << rrt_algo.iter_goal << std::endl;  
    }
    path_stream.close();


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
