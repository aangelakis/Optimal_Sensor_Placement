from utilities import *
import geopandas as gpd
import matplotlib.pyplot as plt
import networkx as nx


def main():
    # We read a shape file and store it in the variable data
    # This is the WDN of Mohos, a village in Hersonissos, Crete, Greece
    # You can change the path and run the algorithms in your own topologies.
    
    data = gpd.read_file('network/diktio.shp')
    # data = gpd.read_file('limnes/limnes.shp')
    # data = gpd.read_file('GRC_wat/GRC_water_lines_dcw.shp')
    data['points'] = data['geometry'].apply(extract_coords)

    # Create the graph from the shape file
    graph = gdf_to_nx(data)

    components = list(nx.connected_components(graph))
    subgraphs = [graph.subgraph(c).copy() for c in components]
    print('number of subgraphs:', len(subgraphs))
    
    for i, subgraph in enumerate(subgraphs):
        print(f"\n--- Subgraph {i+1} with {subgraph.number_of_nodes()} nodes and {subgraph.number_of_edges()} edges ---")
        
        if len(subgraph.nodes) < 10:
            print('subgraph is too small, skipping')
            continue
        
        graph_pos = {n: [n[0], n[1]] for n in list(subgraph.nodes)}
        
        # For easier debugging and visibility, we make the graph with int node labels
        G = nx.convert_node_labels_to_integers(subgraph, label_attribute='id')
        G_pos = find_node_positions(subgraph)
        G_lengths = nx.get_edge_attributes(G, 'length')
        
        print_graph_with_sensors(G, -1, G_pos, 'graph')
        
        sensor_range = 200
        # Insert new nodes and edges if it is needed
        add_new_nodes_edges_int_graph(G, G_pos, sensor_range)
        add_new_nodes_edges_graph(subgraph, graph_pos, sensor_range)
        
        # The starting node to run the BFS algorithm with.
        # Change it if you want to run the algorithm with a different starting node.
        start = 0
        
        # Initializations after the addition of new nodes/edges
        G_pos = find_node_positions(subgraph)
        initialize_graph_without_new(G)
        find_length(G, G_pos, start)
        
        # Run the coverage algorithm on our graph.
        sensors, sensor_coverage = sensors_coverage_problem(G, start, sensor_range)
        print('number of sensors after the coverage problem:', len(sensors))

        # Plot the graph after the coverage algorithm (the red nodes are the sensors).
        print_graph_with_sensors(G, start, G_pos, 'graph with sensors, with the coverage algorithm')

        # Run the leakage algorithm on our graph.
        sensors_leakage_problem(G, sensor_coverage, sensor_range)
        print('number of sensors after the leakage problem:', len(sensor_coverage))

        print_graph_with_sensors(G, start, G_pos, 'graph with sensors, with the leakage algorithm')


if __name__ == '__main__':
    main()
    plt.show()