from utilities import * 
import matplotlib.pyplot as plt
import networkx as nx


def main():
    # Create the random tree
    G = nx.random_tree(n=60, seed=0)
    print(G)
    tree = nx.convert_node_labels_to_integers(G)
    pos = nx.nx_agraph.graphviz_layout(tree, prog="neato", args="")

    # Initialize the tree for our algorithms
    initialize_graph(tree)
    find_length(tree, pos, 0)

    # This is the range of the sensors we are going to run the algorithms with.
    # Change it if you want to run them with different range.
    sensor_range = 200

    # Insert new nodes and edges if it is needed
    add_new_nodes_edges_int_graph(tree, pos, sensor_range)

    # The starting node to run the BFS algorithm with.
    # Change it if you want to run the algorithm with a different starting node.
    start = 0

    # Initializations after the addition of new nodes/edges
    initialize_graph(tree)
    find_length(tree, pos, 0)

    # Plot the random tree
    print_graph_with_sensors(tree, start, pos, title='a random tree')

    # Run the coverage algorithm on our graph.
    sensors, sensor_coverage = sensors_coverage_problem(tree, start, sensor_range)

    print('number of sensors after coverage algorithm: ', len(sensors))
    print_graph_with_sensors(tree, start, pos, title='graph with sensors, with the coverage algorithm')

    # Run the leakage algorithm on our graph
    sensors_leakage_problem(tree, sensor_coverage, sensor_range)

    print('number of sensors after leakage algorithm: ', len(sensor_coverage))
    print_graph_with_sensors(tree, start, pos, title='graph with sensors, with the leakage algorithm')


if __name__ == '__main__':
    main()
    plt.show()

