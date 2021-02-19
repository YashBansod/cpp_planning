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
    file_index = 1
    start_index = 9
    goal_index = 5

    lw = [0.5, 1, 2]
    ms = [5, 8, 10]

    nodes_fp = "../inputs/nodes_%d.txt" % file_index
    edges_fp = "../inputs/edges_with_costs_%d.txt" % file_index

    path_fp = "../outputs/path_output_%d.txt" % file_index
    search_fp = "../outputs/search_output_%d.txt" % file_index

    nodes = []
    with open(nodes_fp) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        first_line = False
        for row in csv_reader:
            if not first_line:
                first_line = True
            else:
                nodes.append((int(row[0]), float(row[1]), float(row[2])))
    nodes = np.array(nodes)

    edges = []
    with open(edges_fp) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        first_line = False
        for row in csv_reader:
            if not first_line:
                first_line = True
            else:
                edges.append((int(row[0]), int(row[1]), float(row[2])))
    edges = np.array(edges, dtype=np.int)

    path = []
    with open(path_fp) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            path.append((int(row[0]), float(row[1]), float(row[2])))
    path = np.array(path)

    search = []
    with open(search_fp) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            search.append((int(row[0]), float(row[1]), float(row[2]), int(row[3]), float(row[4]), float(row[5])))
    search = np.array(search)

    plt.style.use('bmh')

    g_edges = []
    for edge in edges:
        g_edges.append([(nodes[edge[0] - 1, 1:3]), (nodes[edge[1] - 1, 1:3])])

    s_edges = []
    for edge in search:
        s_edges.append([(edge[1:3]), (edge[4:6])])

    lc1 = LineCollection(g_edges, colors='gray', linewidths=lw[0], linestyles=":")
    lc2 = LineCollection(s_edges, colors='m', linewidths=lw[1], linestyles='--')

    fig, ax = plt.subplots()
    fig.set_size_inches(10, 9)

    plt.plot(nodes[edges[:, 0] - 1, 1], nodes[edges[:, 0] - 1, 2], 'o', c='black', ms=ms[0], label='Graph')
    plt.plot(nodes[edges[:, 1] - 1, 1], nodes[edges[:, 1] - 1, 2], 'o', c='black', ms=ms[0])
    ax.add_collection(lc1)

    plt.plot(search[:, 1], search[:, 2], 'p', c='m', ms=ms[1], label='Search Tree')
    plt.plot(search[:, 1], search[:, 2], 'p', c='m', ms=ms[1])
    ax.add_collection(lc2)

    plt.plot(path[:, 1], path[:, 2], '-p', c='green', lw=lw[2], ms=ms[1], label='Solution Path')

    plt.plot(nodes[start_index - 1, 1], nodes[start_index - 1, 2], 'D', c='blue', ms=ms[2], label="Start Node")
    plt.plot(nodes[goal_index - 1, 1], nodes[goal_index - 1, 2], 'X', c='red', ms=ms[2], label="Goal Node")

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