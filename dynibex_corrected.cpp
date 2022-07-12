#include <iostream>
#include <cmath>
#include "Point3.hpp"
#include "Segment.hpp"
#include "Env.hpp"
#include "RRT_STAR.hpp"
#include "Node.hpp"
#include <fstream>
#include "ibex.h"
#include <cstring>  
#include <string>

using namespace ibex;
using namespace std;

int main(int argc, char ** argv){
    //Voliere de l'U2IS
    Env environment = Env(-1.5, 1.1, -1.4, 1.8, 0.0, 2, 0.25, 0.25, 0.25);

//Defining the obstacles of the scene
    vector<vector<double>> list_rectangles =
        {   
            {-1.0, -1.25, -0.1, 0.25, 0.7, 0.6}, // Boite proche du mur
            {-1.1, 0.71, -0.1, 0.4, 0.6, 0.5},  //Boite opérateur
            // {-0.6, 0.7, -0.1, 1.25, 1, 3},    //Opérateur
            // {-1.65, -0.2, -0.1, 1.2, 0.3, 3}      //Obstacle virtuel
            {-1.15, -0.4, -0.1, 1.15, 0.8, 3}      //Obstacle virtuel
        };

    //Adding rectangles to the scene
    for(int i = 0; i < list_rectangles.size(); ++i){
        environment.add_rectangle(list_rectangles[i]);
    }

    //Obstacles for dynibex
    vector<vector<double>>rectangles_intervals = environment.get_rectangles_boundaries(); //Inflated obstacles boundaries
    
    vector<IntervalVector> obstacles_itv;
    for(int k = 0; k < rectangles_intervals.size(); ++k){
        double boundaries_itv[3][2] = {{rectangles_intervals[k][0],rectangles_intervals[k][1]},
                                        {rectangles_intervals[k][2],rectangles_intervals[k][3]},
                                            {rectangles_intervals[k][4],rectangles_intervals[k][5]}};
        IntervalVector temp = IntervalVector(3, boundaries_itv);
        obstacles_itv.push_back(temp);
    }

    //Exporting dynibex simulation
    ofstream export_3d;
    export_3d.open("export_3d.txt");

    //Exporting trajectory realised
    ofstream trajectory_boxes;
    trajectory_boxes.open("trajectory_boxes.txt");

    //Opening the command with all the necessary information
    ifstream command;
    command.open("csv_files/data1.txt");

    string line;

    vector<vector<double>> list_of_information;

    if (command.is_open())
    {
        while ( getline (command,line) )
        {
            vector<double> temp;
            string s ="";
            for (auto x : line)
            {
                if (x == ','){
                    temp.push_back(std::stod(s));
                    s = "";
                }
                else {
                    s = s+x;
                }
            }
            temp.push_back(std::stod(s));
            list_of_information.push_back(temp);
        }
        //cout << list_of_information << endl;
        command.close();
    }

    bool first_loop = true;
    const int n= 3; 
    Variable y(n);
    Affine2Vector state(n);

    IntervalVector yinit(n);

    for(int index = 0; index < list_of_information.size()-2; ++index){
        if(index%1 == 0){
            //defaut sur la mesure Optitrack
            double deltax = 0.1;
            double deltay = 0.1;
            double deltaz = 0.1;

            if(first_loop == true){
                yinit[0] = Interval(list_of_information[index][3] - deltax, list_of_information[index][3] + deltax);
                yinit[1] = Interval(list_of_information[index][4] - deltay, list_of_information[index][4] + deltay);
                yinit[2] = Interval(list_of_information[index][5] - deltaz, list_of_information[index][5] + deltaz);
                first_loop = false;
            }

            //Paramètres 
            Interval Kp(1.8);
            Interval Ki(3.2);

            double x_goal = list_of_information[index+1][3];
            double y_goal = list_of_information[index+1][4];
            double z_goal = list_of_information[index+1][5];

            Interval xGoal(x_goal);
            Interval yGoal(y_goal);
            Interval zGoal(z_goal);
            IntervalVector destination_itvs = IntervalVector(6);
            destination_itvs[0] = xGoal + Interval(-0.01, 0.01);
            destination_itvs[1] = yGoal + Interval(-0.01, 0.01);
            destination_itvs[2] = zGoal + Interval(-0.01, 0.01);

            //commande uX,uY,uZ
            Interval uX = Interval(list_of_information[index][0]);
            Interval uY = Interval(list_of_information[index][1]);
            Interval uZ = Interval(list_of_information[index][2]);

            // cout << "Avant la simu : "<< yinit << endl;
            double alphaX = 1;
            double alphaY = 0.22;
            double alphaZ = 0.2;
            Function ydot = Function(y,Return( 
                                uX*alphaX,
                                uY*alphaY,
                                uZ*alphaZ)
                            );

            ivp_ode problem = ivp_ode(ydot,list_of_information[index][10],yinit); //Modèle dynamique, temps de départ, yinit

            simulation simu = simulation(&problem,list_of_information[index+1][10],HEUN,1e-1);

            simu.run_simulation();
            yinit = simu.get_last();

            // cout << "Après la simu : "<< yinit << endl;


    //Verification no obstacles touched, and destination reached
            bool no_collision = false;
            for(int k = 0; k < obstacles_itv.size(); ++k){
                if(simu.has_crossed_b(obstacles_itv[k]) == NO){
                    no_collision = true;
                }
            }
            
            // std::cout << simu.get_last() << std::endl;
            // std::cout << x_goal << ", " << y_goal << ", " << z_goal <<std::endl;
            // std::cout << "Objectives done accordingly ? " << !no_collision  << std::endl;
            // std::cout << "Step Destination reached ? " << (simu.finished_in(destination_itvs) == false)  << std::endl;
            

    //Write CSV for the 3D boxes for the plot
            // std::list<solution_g>::iterator iterator_list;
            // for(iterator_list=simu.list_solution_g.begin();iterator_list!=simu.list_solution_g.end();iterator_list++)
            // {
            // export_3d << iterator_list->box_jnh->operator[](0) <<
            // " ; " << iterator_list->box_jnh->operator[](1) <<
            // " ; " << iterator_list->box_jnh->operator[](2) << std::endl;
            // }
            export_3d << yinit[0] <<
            " ; " << yinit[1] <<
            " ; " << yinit[2] << std::endl;

    //Write CSV for the 3D boxes representinf the trajectory
            trajectory_boxes<< destination_itvs[0] <<
            " ; " << destination_itvs[1] <<
            " ; " << destination_itvs[2] << std::endl;
        }//end else
    }//end while


    export_3d.close(); 

    return 0;

}
