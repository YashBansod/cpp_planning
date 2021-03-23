# !/usr/bin/env python
"""
File Description: File used plotting the search outputs
"""

# ******************************************    Libraries to be imported    ****************************************** #
import csv
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.collections import LineCollection


# ******************************************        Main Program Start      ****************************************** #
def main():
    # file_index = 1
    # start = (30, -35, 90)
    # goal = (0, 0, 5)

    # file_index = 2
    # start = (40, -40, 0)
    # goal = (30, 40, 5)

    # file_index = 3
    # start = (30, 40, 270)
    # goal = (-50, -30, 5)

    # file_index = 4
    # start = (-50, -30, 90)
    # goal = (30, -35, 5)

    file_index = 5
    start = (-30, -35, 45)
    goal = (-35, 30, 5)

    lw = [0.5, 1, 2, 3]
    ms = [5, 8, 10, 12]
    sz = [15, 13]

    obstacles_fp = "../inputs/obstacles.txt"
    path_fp = "../outputs/path_output_%d.txt" % file_index
    search_fp = "../outputs/search_output_%d.txt" % file_index
    traj_fp = "../outputs/traj_output_%d.txt" % file_index

    obstacles = []
    with open(obstacles_fp) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        first_line = False
        for row in csv_reader:
            if not first_line:
                first_line = True
            else:
                obstacles.append((float(row[0]), float(row[1]), float(row[2])))

    path = []
    with open(path_fp) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            path.append((float(row[1]), float(row[2])))
    path = np.array(path)

    traj = []
    with open(traj_fp) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            traj.append((float(row[1]), float(row[2])))
    traj = np.array(traj)

    search = []
    with open(search_fp) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            search.append((float(row[1]), float(row[2]), float(row[3]), float(row[5]), float(row[6]), float(row[7])))
    search = np.array(search)

    plt.style.use('bmh')

    s_edges = []
    for edge in search:
        s_edges.append([(edge[0:2]), (edge[3:5])])

    lc2 = LineCollection(s_edges, colors='m', linewidths=lw[1], linestyles='--')

    fig, ax = plt.subplots()
    fig.set_size_inches(sz[0], sz[1])
    for obs in obstacles:
        ax.add_patch(plt.Circle((obs[0], obs[1]), obs[2], color='black'))
    ax.add_patch(plt.Circle((goal[0], goal[1]), goal[2], color='red', alpha=0.5))

    if search.shape[0] > 0:
        plt.plot(search[:, 0], search[:, 1], 'p', c='m', ms=ms[1], label='Search Tree')
        plt.plot(search[:, 3], search[:, 4], '1', c='m', ms=ms[1])
        ax.add_collection(lc2)

    if path.shape[0] > 0:
        plt.plot(path[:, 0], path[:, 1], '-p', c='green', lw=lw[3], ms=ms[2], label='Solution Path')
        plt.plot(traj[:, 0], traj[:, 1], '-x', c='orange', lw=lw[1], ms=ms[1], label='Solution Trajectory')

    plt.plot(start[0], start[1], 'D', c='blue', ms=ms[2], label="Start Node")
    plt.plot(goal[0], goal[1], 'X', c='red', ms=ms[2], label="Goal Node")
    ax.add_patch(plt.Rectangle((-50, -50), 100, 100, edgecolor='grey', fill=False, linewidth=2, label='Workspace'))
    plt.legend(loc='lower center', ncol=3, bbox_to_anchor=(0.5, 1))
    plt.tight_layout()
    plt.savefig("../outputs/sol_%d.png" % file_index, format='png')
    plt.show()


# ******************************************        Main Program End        ****************************************** #
# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    try:
        main()
        print('\nFile executed successfully!\n')
    except KeyboardInterrupt:
        print('\nProcess interrupted by user. Bye!')

"""
Author(s): Yash Bansod
Repository: https://github.com/YashBansod/cpp_robotics
Organization: University of Maryland at College Park
"""
