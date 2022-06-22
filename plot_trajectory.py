from ast import Num
from ctypes import pointer
from turtle import color
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from numpy import double
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

fig, ax = plt.subplots(subplot_kw={'projection': '3d'})



import os

def main():
    animation             = False
    only_the_trajectory   = False
    animation_trajectory  = False
    plot_trajectory_bool  = True

    ##Plotted
    trajectory = []
    tree = []
    obstacles = []
    if(plot_trajectory_bool):
        with open("trajectory.txt", "r") as file:
            for line in file:
                grade_data = line.strip().split(',')
                vector = []
                for i in range(len(grade_data)):
                    vector.append(double(grade_data[i]))
                trajectory.append(vector)
        Number_of_iteration = trajectory.pop()   #Vecteur contenant un entier
        print(Number_of_iteration)

    if(only_the_trajectory == False):
        with open("tree.txt", "r") as file:
            for line in file:
                grade_data = line.strip().split(',')
                vector = []
                for i in range(len(grade_data)):
                    vector.append(double(grade_data[i]))
                tree.append(vector)

    with open("obstacles.txt", "r") as file:
        for line in file:
            grade_data = line.strip().split(',')
            vector = []
            for i in range(len(grade_data)):
                vector.append(double(grade_data[i]))
            obstacles.append(vector)

    boundaries_info = []
    with open("boundaries.txt", "r") as file:
        for line in file:
            grade_data = line
            boundaries_info.append(double(grade_data))

    start_destination = []
    with open("start_destination.txt", "r") as file:
        for line in file:
            grade_data = line.strip().split(',')
            vectorPOS = []
            for i in range(len(grade_data)):
                start_destination.append(double(grade_data[i]))

    #Definition des limites de la scene donnee par l'environment
    #Et plot point de depart et d'arrivee
    ax.set_xlim3d(boundaries_info[0], boundaries_info[1])
    ax.set_ylim3d(boundaries_info[2], boundaries_info[3])
    ax.set_zlim3d(boundaries_info[4], boundaries_info[5])

    ax.scatter(start_destination[0],start_destination[1],start_destination[2], color = 'r',s = 200)
    ax.scatter(start_destination[3],start_destination[4],start_destination[5], color = 'r', s = 200)

    #Plot des obstacles
    Z=[]
    for i in range(len(obstacles)):
        Z.append([obstacles[i][0], obstacles[i][1], obstacles[i][2]])
        if(len(Z)==8):
            verts = [[Z[0],Z[1],Z[2],Z[3]],
                    [Z[4],Z[5],Z[6],Z[7]],
                    [Z[0],Z[1],Z[5],Z[4]],
                    [Z[2],Z[3],Z[7],Z[6]],
                    [Z[1],Z[2],Z[6],Z[5]],
                    [Z[4],Z[7],Z[3],Z[0]]]
            ax.add_collection3d(Poly3DCollection(verts, facecolors='cyan', linewidths=1, edgecolors='g', alpha=.20))
            Z = []


    #Plot de l'arbre
    if(only_the_trajectory == False):
        for i in range(1,len(tree),2):
            xt =[tree[i][0], tree[i-1][0]]
            yt =[tree[i][1], tree[i-1][1]]
            zt =[tree[i][2], tree[i-1][2]] 
            ax.plot(xt, yt, zt, label='tree', color = 'b')
            if(animation):
                plt.pause(0.000000000000000001)


    #Plot de la trajectoire
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

    plt.savefig('PLOT.png')
    plt.show()  


if __name__ == '__main__':
    main()
  