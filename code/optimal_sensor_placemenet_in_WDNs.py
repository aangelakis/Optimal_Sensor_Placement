# Algorithms written and developed by Alexandros Angelakis, University of Crete, ICS-FORTH, angelakis@ics.forth.gr
# supervised by Yannis Pantazis, IACM-FORTH, pantazis@iacm.forth.gr


import utilities as utl
import geopandas as gpd
import matplotlib.pyplot as plt
import networkx as nx
import math


def main():
    # We read a shape file and store it in the variable data
    # This is the WDN of Mohos, a village in Hersonissos, Crete, Greece
    # You can change the path and run the algorithms in your own topologies.
    data = gpd.read_file('network/diktio.shp')
    data['points'] = data.apply(lambda x: [y for y in x['geometry'].coords], axis=1)

    # Create the graph from the shape file
    graph = gdf_to_nx(data)
    print(graph)
    graph_pos = {n: [n[0], n[1]] for n in list(graph.nodes)}
    lengths = nx.get_edge_attributes(graph, 'length')

    # For easier debugging and visibility, we make the graph with int node labels
    G = nx.convert_node_labels_to_integers(graph, label_attribute='id')
    G_pos = utl.find_node_positions(graph)
    G_lengths = nx.get_edge_attributes(G, 'length')

    # This is the range of the sensors we are going to run the algorithms with.
    # Change it if you want to run them with different range.
    sensor_range = 200

    # Insert new nodes and edges if it is needed
    utl.add_new_nodes_edges_int_graph(G, G_pos, sensor_range)
    utl.add_new_nodes_edges_graph(graph, graph_pos, sensor_range)

    # The starting node to run the BFS algorithm with.
    # Change it if you want to run the algorithm with a different starting node.
    start = 0

    # Initializations after the addition of new nodes/edges
    G_pos = utl.find_node_positions(graph)
    utl.initialize_graph_without_new(G)
    utl.find_length(G, G_pos, start)

    # Run the coverage algorithm on our graph.
    sensors, sensor_coverage = utl.sensors_coverage_problem(G, start, sensor_range)
    print('number of sensors after the coverage problem:', len(sensors))

    # Plot the graph after the coverage algorithm (the red nodes are the sensors).
    utl.print_graph_with_sensors(G, start, G_pos, 'graph with sensors, with the coverage algorithm')

    # Run the leakage algorithm on our graph.
    utl.sensors_leakage_problem(G, sensor_coverage, sensor_range)
    print('number of sensors after the leakage problem:', len(sensor_coverage))

    utl.print_graph_with_sensors(G, start, G_pos, 'graph with sensors, with the leakage algorithm')


# Function that generates a graph from GeoDataFrame of Linestrings
def gdf_to_nx(gdf_network):
    net = nx.Graph()
    net.graph['crs'] = gdf_network.crs
    fields = list(gdf_network.columns)
    for index, row in enumerate(gdf_network.iterrows()):
        linestring = gdf_network.points[index]
        for point in range(len(linestring)):
            if point == len(linestring) - 1:
                break
            start = linestring[point]
            end = linestring[point + 1]
            if not net.has_node(start):
                net.add_node(start, sensor='no', new='no')
            if not net.has_node(end):
                net.add_node(end, sensor='no', new='no')
            weight = round(math.dist(start, end), 3)
            net.add_edge(start, end, length=weight, covered='no',possible_leakage='no')
    return net


if __name__ == '__main__':
    main()
    plt.show()