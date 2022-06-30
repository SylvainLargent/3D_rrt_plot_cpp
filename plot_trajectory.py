from ast import Num
from ctypes import pointer
#from turtle import color
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from numpy import double, true_divide
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import pandas as pd
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots(subplot_kw={'projection': '3d'})



import os

def main():
    trajectory_b  = True
    plot_position_animation = True
    
    if(trajectory_b):
        animation             = False
        plot_tree             = False
        animation_trajectory  = False
        plot_trajectory_bool  = True    
    else :
        animation             = False
        plot_tree             = True   
        animation_trajectory  = False
        plot_trajectory_bool  = False


    ##Plotted
    trajectory = []
    tree = []
    obstacles = []
    position = []

    if(plot_trajectory_bool):
        with open("csv_files/trajectory.txt", "r") as file:
            for line in file:
                grade_data = line.strip().split(',')
                vector = []
                for i in range(len(grade_data)):
                    vector.append(double(grade_data[i]))
                trajectory.append(vector)
        Number_of_iteration = trajectory.pop()   #Vector containing an integer, number of iteration till goal
        print(Number_of_iteration)

    if(plot_tree == True):
        with open("csv_files/tree.txt", "r") as file:
            for line in file:
                grade_data = line.strip().split(',')
                vector = []
                for i in range(len(grade_data)):
                    vector.append(double(grade_data[i]))
                tree.append(vector)

    with open("csv_files/obstacles.txt", "r") as file:
        for line in file:
            grade_data = line.strip().split(',')
            vector = []
            for i in range(len(grade_data)):
                vector.append(double(grade_data[i]))
            obstacles.append(vector)

    boundaries_info = []
    with open("csv_files/boundaries.txt", "r") as file:
        for line in file:
            grade_data = line
            boundaries_info.append(double(grade_data))

    start_destination = []
    with open("csv_files/start_destination.txt", "r") as file:
        for line in file:
            grade_data = line.strip().split(',')
            vectorPOS = []
            for i in range(len(grade_data)):
                start_destination.append(double(grade_data[i]))

    #Boundaries info
    #Plot starting and goal points
    ax.set_xlim3d(boundaries_info[0], boundaries_info[1])
    ax.set_ylim3d(boundaries_info[2], boundaries_info[3])
    ax.set_zlim3d(boundaries_info[4], boundaries_info[5])

    ax.scatter(start_destination[0],start_destination[1],start_destination[2], color = 'r',s = 200)
    ax.scatter(start_destination[3],start_destination[4],start_destination[5], color = 'r', s = 200)

    #Ploting obstacles
    Z=[]
    for i in range(len(obstacles)):
        Z.append([obstacles[i][0], obstacles[i][1], obstacles[i][2]]) #On lui donne les coordonnees de chaque sommet
        if(len(Z)==8):                                                #Avec la position des 8 sommets il dessinne le polygone
            verts = [[Z[0],Z[1],Z[2],Z[3]],
                    [Z[4],Z[5],Z[6],Z[7]],
                    [Z[0],Z[1],Z[5],Z[4]],
                    [Z[2],Z[3],Z[7],Z[6]],
                    [Z[1],Z[2],Z[6],Z[5]],
                    [Z[4],Z[7],Z[3],Z[0]]]
            ax.add_collection3d(Poly3DCollection(verts, facecolors='cyan', linewidths=1, edgecolors='g', alpha=.20))
            Z = []

    #Plotting the tree
    if(plot_tree == True):
        for i in range(1,len(tree),2):
            xt =[tree[i][0], tree[i-1][0]]
            yt =[tree[i][1], tree[i-1][1]]
            zt =[tree[i][2], tree[i-1][2]] 
            ax.plot(xt, yt, zt, label='tree', color = 'b')
            if(animation):
                plt.pause(0.000000000000000001)

    #Plotting the trajectory
    if(plot_trajectory_bool):
        xs = []
        ys = []
        zs = []
        for i in range(0,len(trajectory)):
            xs.append(trajectory[i][0])
            ys.append(trajectory[i][1])
            zs.append(trajectory[i][2])
            ax.plot(xs, ys, zs, label='parametric curve %i' %Number_of_iteration[0], color = 'r')
            if(animation_trajectory):
                    plt.pause(0.000000000000000001)

    
   

    if(plot_position_animation):
        def animate(i):
            line = pd.read_csv('csv_files/position.txt', sep=";|,", engine="python", skipfooter=1, header = None, usecols=[0,1,2], names=['x', 'y','z'])
            x = line["x"]
            y = line["y"]
            z = line["z"]
            ax.plot3D(x, y, z, 'green')
        ani = FuncAnimation(plt.gcf(), animate, interval=500)

    if(not plot_position_animation):
        with open("csv_files/position.txt", "r") as file:
            for line in file:
                grade_data = line.strip().split(',')
                vector = []
                for i in range(len(grade_data)):
                    vector.append(double(grade_data[i]))
                position.append(vector)

        ##Plotting the position
            xs = []
            ys = []
            zs = []
            for i in range(0,len(position)):
                xs.append(position[i][0])
                ys.append(position[i][1])
                zs.append(position[i][2])
                ax.plot(xs, ys, zs, label='parametric curve %i' %Number_of_iteration[0], color = 'g')
    
    plt.savefig('PLOT.png')
    plt.show()  


if __name__ == '__main__':
    main()
  