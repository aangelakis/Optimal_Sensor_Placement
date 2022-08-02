# Algorithms written and developed by Alexandros Angelakis, University of Crete, ICS-FORTH, angelakis@ics.forth.gr
# supervised by Yannis Pantazis, IACM-FORTH, pantazis@iacm.forth.gr


import utilities as utl
import geopandas as gpd
import matplotlib.pyplot as plt
import networkx as nx
import math


def main():
    test_grid_graph()


def test_grid_graph():
    # Create the grid graph and print it
    g = create_grid_graph(max_x=500, max_y=250, x_step=25, y_step=25)
    g1 = nx.convert_node_labels_to_integers(g)
    g2 = nx.convert_node_labels_to_integers(g)
    g_positions = utl.find_node_positions(g)

    print(g2)

    # This is the range of the sensors we are going to run the algorithms with.
    # Change it if you want to run them with different range.
    sensor_range = 73

    # Insert new nodes and edges if it is needed
    utl.add_new_nodes_edges_int_graph(g, g_positions, sensor_range)
    utl.add_new_nodes_edges_graph(g2, g_positions, sensor_range)

    # The starting node to run the BFS algorithm with.
    # Change it if you want to run the algorithm with a different starting node.
    start = 0

    # Initializations after the addition of new nodes/edges
    g2_pos = utl.find_node_positions(g)
    utl.initialize_graph_without_new(g2)
    utl.find_length(g2, g2_pos, start)

    # Run the coverage algorithm on our graph.
    sensors, sensor_coverage = utl.sensors_coverage_problem(g2, start, sensor_range)

    print('number of sensors after coverage algorithm: ', len(sensors))
    utl.print_graph_with_sensors_grid(g2, start, 0, title='graph with sensors, with the coverage algorithm')

    # Run the leakage algorithm on our graph.
    utl.sensors_leakage_problem(g2, sensor_coverage, sensor_range)

    print('number of sensors after leakage algorithm: ', len(sensor_coverage))
    utl.print_graph_with_sensors_grid(g2, start, 0, title='graph with sensors, with the leakage algorithm')


# Function that creates a grid graph with size max_x,max_y and nodes every x_step for the x axis and y_step for the y axis.
def create_grid_graph(max_x, max_y, x_step, y_step):
    x = 0
    y = 0
    g = nx.Graph()
    g.add_node((x, y), pos=(x, y), sensor='no', new='no')
    x += x_step
    while x < max_x and y < max_y:
        g.add_node((x, y), pos=(x, y), sensor='no', new='no')

        if y > 0 and x == 0:
            g.add_edge((x, y - y_step), (x, y), covered='no', possible_leakage='no', length=y_step)
        elif y > 0:
            g.add_edge((x, y - y_step), (x, y), covered='no', possible_leakage='no', length=y_step)
            g.add_edge((x - x_step, y), (x, y), covered='no', possible_leakage='no', length=x_step)
        else:
            g.add_edge((x - x_step, y), (x, y), covered='no', possible_leakage='no', length=x_step)
        x += x_step

        if (x >= max_x):
            y += y_step
            x = 0

    g1 = nx.convert_node_labels_to_integers(g)

    pos = nx.get_node_attributes(g1, 'pos')
    lengths = nx.get_edge_attributes(g1, 'length')
    plt.figure(figsize=(10, 10))
    nx.draw(g1, pos, with_labels=True, font_size=10)
    nx.draw_networkx_edge_labels(g, pos, edge_labels=lengths, font_size=7)
    return g


if __name__ == '__main__':
    main()
    plt.show()