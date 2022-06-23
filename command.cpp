#include "command.hpp"

//ADEQUATE SPEED AND COMMAND
    double speed = 0.4 * (5/2); //
    vector<double>  pduration_list = determine_time(path, speed);
    vector<vector<double>> list_of_velocities = determine_cmd(path, speed);

    ros::NodeHandle nh1;
    ros::NodeHandle nh2;
    ros::NodeHandle nh_move;

    std_msgs::Empty msg1;
    std_msgs::Empty msg2;

    geometry_msgs::Twist var;

    ros::Publisher take_off_pub;
    take_off_pub = nh1.advertise<std_msgs::Empty>("/tello/takeoff", 100);
    ros::Publisher move_pub ;
    move_pub = nh_move.advertise<geometry_msgs::Twist>("/tello/cmd_vel", 1000);
    ros::Publisher land_pub;
    land_pub = nh2.advertise<std_msgs::Empty>("/tello/land", 100);

//TAKE OFF
    sleep(2);
    ROS_INFO("Before takeoff");
    take_off_pub.publish(msg1);
    ROS_INFO("tookoff");
    sleep(1);

//COMMAND
    double time;
    //0.5 correspond à 0.2 m/s
    //ros::Duration(0.5).sleep(); //sleep for floats
    ROS_INFO("engagin movement");
    for(int i = 0; i < pduration_list.size(); ++i){
        time = pduration_list[i];
        ROS_INFO("duration : %f", time);
        var.linear.x = list_of_velocities[i][0];
        ROS_INFO("vx : %f",var.linear.x);
        var.linear.y = list_of_velocities[i][1];
        ROS_INFO("vy : %f",var.linear.y);
        var.linear.z = list_of_velocities[i][2];
        ROS_INFO("vz : %f",var.linear.z);
        move_pub.publish(var);
        ros::Duration(time).sleep();
        // sleep(time);
    }

    var.linear.x = 0;
    var.linear.y = 0;
    var.linear.z = 0;
    move_pub.publish(var);
    sleep(1);

//LANDING
    land_pub.publish(msg2);
    ROS_INFO("Landed");



/home/student/Desktop/Sylvain/3D_rrt_plot.cpp/boundaries.txt