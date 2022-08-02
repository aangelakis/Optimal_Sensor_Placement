# Algorithms written and developed by Alexandros Angelakis, University of Crete, ICS-FORTH, angelakis@ics.forth.gr
# supervised by Yannis Pantazis, IACM-FORTH, pantazis@iacm.forth.gr


import utilities as utl
import geopandas as gpd
import matplotlib.pyplot as plt
import networkx as nx
import math


def main():
    # Create the random tree
    G = nx.random_tree(n=60, seed=0)
    print(G)
    tree = nx.convert_node_labels_to_integers(G)
    pos = nx.nx_agraph.graphviz_layout(tree, prog="neato", args="")

    # Initialize the tree for our algorithms
    utl.initialize_graph(tree)
    utl.find_length(tree, pos, 0)

    # This is the range of the sensors we are going to run the algorithms with.
    # Change it if you want to run them with different range.
    sensor_range = 200

    # Insert new nodes and edges if it is needed
    utl.add_new_nodes_edges_int_graph(tree, pos, sensor_range)

    # The starting node to run the BFS algorithm with.
    # Change it if you want to run the algorithm with a different starting node.
    start = 0

    # Initializations after the addition of new nodes/edges
    utl.initialize_graph(tree)
    utl.find_length(tree, pos, 0)

    # Plot the random tree
    utl.print_graph_with_sensors(tree, start, pos, title='a random tree')

    # Run the coverage algorithm on our graph.
    sensors, sensor_coverage = utl.sensors_coverage_problem(tree, start, sensor_range)

    print('number of sensors after coverage algorithm: ', len(sensors))
    utl.print_graph_with_sensors(tree, start, pos, title='graph with sensors, with the coverage algorithm')

    # Run the leakage algorithm on our graph
    utl.sensors_leakage_problem(tree, sensor_coverage, sensor_range)

    print('number of sensors after leakage algorithm: ', len(sensor_coverage))
    utl.print_graph_with_sensors(tree, start, pos, title='graph with sensors, with the leakage algorithm')


if __name__ == '__main__':
    main()
    plt.show()

