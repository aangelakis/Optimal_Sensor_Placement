import matplotlib.pyplot as plt
import networkx as nx
import math
from tqdm import tqdm
from collections import deque


def gdf_to_nx(gdf_network):
    """Convert a GeoDataFrame of Linestrings and MultiLineStrings into a NetworkX graph.

    Args:
        gdf_network (GeoDataFrame): The GeoDataFrame of the network

    Returns:
        NetworkX Graph: The NetworkX Graph of the network
    """
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
                # Set the default attributes for the node
                net.add_node(start, sensor='no', new='no')
            if not net.has_node(end):
                # Set the default attributes for the node
                net.add_node(end, sensor='no', new='no')
            weight = round(math.dist(start, end), 3)
            if weight < 1:
                # Convert meters to mm if the weight is less than 1 m
                weight = weight * 1000
            # print('weight:', weight)
            # Add the edge to the graph with the weight attribute
            net.add_edge(start, end, length=weight, covered='no', possible_leakage='no')
    return net


def extract_coords(geom):
    """
    Extracts the coordinates from a GeoPandas GeoSeries object.

    Parameters
    ----------
    geom : GeoSeries
        The GeoSeries object containing the geometry of the network

    Returns
    -------
    list
        A list of coordinates (tuples of (x, y))
    """
    if geom.geom_type == 'LineString':
        # If the geometry is a single LineString, just return its coordinates
        return list(geom.coords)
    elif geom.geom_type == 'MultiLineString':
        # If the geometry is a MultiLineString, flatten the list of coordinates
        # from all the LineStrings
        return [pt for line in geom.geoms for pt in line.coords]
    elif geom.geom_type == 'Polygon':
        # Get the boundary of the polygon (outer + holes)
        boundary = geom.boundary
        if boundary.geom_type == 'LineString':
            return list(boundary.coords)
        elif boundary.geom_type == 'MultiLineString':
            return [pt for line in boundary.geoms for pt in line.coords]
    elif geom.geom_type == 'MultiPolygon':
        coords = []
        for polygon in geom.geoms:
            boundary = polygon.boundary
            if boundary.geom_type == 'LineString':
                coords.extend(boundary.coords)
            elif boundary.geom_type == 'MultiLineString':
                for line in boundary.geoms:
                    coords.extend(line.coords)
        return coords
    else:
        # Unsupported geometry
        return []


def find_length(G, pos, start):
    """
    Finds the length of all the edges in a graph and stores it in the 'length' attribute of the edge.

    Parameters
    ----------
    G : NetworkX Graph
        The graph to find the length of the edges in
    pos : dict
        A dictionary with the positions of the nodes in the graph
    start : tuple
        The starting node of the graph

    Returns
    -------
    None
    """
    explored = []
    queue = [start]

    while queue:
        node = queue.pop(0)
        while node in queue:
            queue.remove(node)

        if node not in explored:
            explored.append(node)
            neighbours = (list(G.adj.items())[node])[1]

        for next in neighbours:
            if (next not in explored):
                weight = round(math.dist(pos[node], pos[next]), 3)
                if weight < 1:
                    # Convert meters to mm if the weight is less than 1 m
                    weight = weight * 1000

                # Calculate the length of the edge
                G.edges[(node, next)]['length'] = weight
                # Add the next node to the queue
                queue.append(next)


def find_node_positions(graph):
    """
    Finds the positions of all the nodes in a graph and stores them in a dictionary.

    Parameters
    ----------
    graph : NetworkX Graph
        The graph to find the positions of the nodes in

    Returns
    -------
    dict
        A dictionary with the positions of the nodes in the graph
    """
    G_positions = {}
    # Iterate over all the nodes in the graph
    for index, n in enumerate(list(graph.nodes)):
        # Store the position of each node in the dictionary
        G_positions[index] = [n[0], n[1]]
    return G_positions


def add_new_nodes_edges_graph(graph, pos, sensor_range):
    """
    Adds new nodes and edges to the graph, if an edge is longer than the sensor range.

    Parameters
    ----------
    graph : NetworkX Graph
        The graph to add new nodes and edges to
    pos : dict
        The positions of the nodes in the graph
    sensor_range : float
        The range of the sensors

    Returns
    -------
    None
    """
    new_nodes = []
    new_nodes_pos = []
    new_edges = []
    for edge in graph.edges:
        distance = graph.edges[edge]['length']
        (node1, node2) = edge
        old_node1 = node1
        if distance > sensor_range:
            # Calculate how many new nodes and edges are needed
            cuts = math.ceil(distance / sensor_range)
            new_edges_num = cuts
            new_nodes_num = cuts - 1

            # Calculate the positions of the new nodes
            (node1_x, node1_y) = pos[node1]
            (node2_x, node2_y) = pos[node2]

            for i in range(new_nodes_num):
                new_y = (abs(node2_y - node1_y) / cuts) + min(node1_y, node2_y)
                new_x = (abs(node2_x - node1_x) / cuts) + min(node1_x, node2_x)

                # Add the new node to the list of new nodes
                new_nodes.append((new_x, new_y))
                # Add the new node's position to the list of new node positions
                new_nodes_pos.append((new_x, new_y))
                # Add the edge between the new node and the old node to the list of new edges
                new_edges.append((node1, (new_x, new_y)))

                # Update the node1's position to the position of the new node
                node1_y = new_y
                node1_x = new_x
                node1 = (new_x, new_y)
            # Add the edge between the new node and the node2 to the list of new edges
            new_edges.append((node1, node2))
            # Remove the old edge between the old node1 and node2
            graph.remove_edge(old_node1, node2)

    # Add the new nodes and their positions to the graph
    for i in range(len(new_nodes)):
        graph.add_node(new_nodes[i])
        pos[new_nodes[i]] = new_nodes_pos[i]
        graph.nodes[new_nodes[i]]['new'] = 'yes'

    # Add the new edges to the graph
    for i in range(len(new_edges)):
        (x, y) = new_edges[i]
        graph.add_edge(x, y)


def add_new_nodes_edges_int_graph(graph, pos, sensor_range):
    """
    Add new nodes and edges to the graph with int node labels, if it is needed,
    if there is an edge that is greater than the sensor range.

    Parameters
    ----------
    graph : NetworkX Graph
        The graph to add new nodes and edges to
    pos : dict
        The positions of the nodes in the graph
    sensor_range : float
        The range of the sensors

    Returns
    -------
    None
    """
    new_nodes = []
    new_nodes_pos = []
    new_edges = []
    number_of_nodes = graph.number_of_nodes()

    # Iterate over all the edges in the graph
    for edge in graph.edges:
        distance = graph.edges[edge]['length']
        (node1, node2) = edge
        old_node1 = node1

        # If the distance of an edge is greater than the sensor range
        if distance > sensor_range:
            # Calculate how many new nodes and edges are needed
            cuts = math.ceil(distance / sensor_range)
            new_edges_num = cuts
            new_nodes_num = cuts - 1

            # Calculate the positions of the new nodes
            (node1_x, node1_y) = pos[node1]
            (node2_x, node2_y) = pos[node2]

            # Iterate over the number of new nodes needed
            for i in range(new_nodes_num):
                new_y = (abs(node2_y - node1_y) / cuts) + min(node1_y, node2_y)
                new_x = (abs(node2_x - node1_x) / cuts) + min(node1_x, node2_x)

                # Add the new node to the list of new nodes
                new_nodes.append(number_of_nodes)
                # Add the new node's position to the list of new node positions
                new_nodes_pos.append((new_x, new_y))
                # Add the edge between the new node and the old node to the list of new edges
                new_edges.append((node1, number_of_nodes))

                # Update the node1's position to the position of the new node
                node1_y = new_y
                node1_x = new_x
                node1 = number_of_nodes
                number_of_nodes += 1
            # Add the edge between the new node and the node2 to the list of new edges
            new_edges.append((node1, node2))
            # Remove the old edge between the old node1 and node2
            graph.remove_edge(old_node1, node2)

    # Add the new nodes and their positions to the graph
    for i in range(len(new_nodes)):
        graph.add_node(new_nodes[i])
        pos[new_nodes[i]] = new_nodes_pos[i]
        graph.nodes[new_nodes[i]]['new'] = 'yes'

    # Add the new edges to the graph
    for i in range(len(new_edges)):
        (x, y) = new_edges[i]
        graph.add_edge(x, y)


def initialize_graph_without_new(graph):
    """
    Initialize the graph's edges attributes without the 'new' attribute.
    This function is used when the graph is created from a GeoDataFrame and the 'new' attribute is not needed.

    Parameters
    ----------
    graph : NetworkX Graph
        The graph to initialize the edges attributes of

    Returns
    -------
    None
    """
    nx.set_edge_attributes(graph, 0, 'length')
    nx.set_edge_attributes(graph, 'no', 'possible_leakage')
    nx.set_edge_attributes(graph, 'no', 'covered')
    nx.set_node_attributes(graph, 'no', 'sensor')


def initialize_graph(graph):
    """
    Initialize the graph's edges and nodes attributes.

    Parameters
    ----------
    graph : NetworkX Graph
        The graph to initialize the edges and nodes attributes of

    Returns
    -------
    None
    """
    # Initialize the edges attributes
    nx.set_edge_attributes(graph, 0, 'length')
    nx.set_edge_attributes(graph, 'no', 'possible_leakage')
    nx.set_edge_attributes(graph, 'no', 'covered')
    # Initialize the nodes attributes
    nx.set_node_attributes(graph, 'no', 'sensor')
    nx.set_node_attributes(graph, 'no', 'new')


def sensors_coverage_problem(graph, start, sensor_range):
    explored = set()
    queue = deque([start])
    lengths = {}
    sensor_coverage = {}
    sensors = []
    pbar = tqdm(total=graph.number_of_nodes())

    while queue:
        node = queue.popleft()

        if node in explored:
            continue
        explored.add(node)

        neighbours = graph.adj[node]
        tmp = {}
        node_is_sensor = False

        for nxt in neighbours:
            if nxt in explored:
                continue

            edge_length = graph.edges[node, nxt]['length']
            total_length = edge_length

            # Check previous paths for cumulative length
            for prev_node, path_lengths in lengths.items():
                prev_edge = (prev_node, node)
                if prev_edge in path_lengths:
                    total_length += path_lengths[prev_edge]
                    break  # Only need one path

            tmp[(node, nxt)] = total_length

            # Make starting node a sensor
            if not lengths and node not in sensors:
                graph.nodes[node]['sensor'] = 'yes'
                sensors.append(node)
                node_is_sensor = True

            if total_length > sensor_range and graph.edges[node, nxt]['covered'] == 'no':
                if node not in sensors:
                    graph.nodes[node]['sensor'] = 'yes'
                    sensors.append(node)
                    node_is_sensor = True

            queue.append(nxt)

        if node_is_sensor:
            sensor_coverage[node] = find_covered_edges_from_sensor(graph, node, sensor_range)

        lengths[node] = tmp
        pbar.update(1)

    return sensors, sensor_coverage


def find_covered_edges_from_sensor(graph, sensor, sensor_range):
    """
    Finds the edges that can be covered from a sensor in a graph.

    Parameters
    ----------
    graph : NetworkX Graph
        The graph to find the covered edges in
    sensor : int
        The sensor node
    sensor_range : float
        The range of the sensor

    Returns
    -------
    node_coverage : list
        The list of edges that can be covered from the sensor
    """
    explored = set()
    queue = deque([sensor])
    lengths = {}
    node_coverage = []

    while queue:
        node = queue.popleft()

        if node in explored:
            continue
        explored.add(node)

        neighbours = graph.adj[node]
        tmp = {}

        for nxt in neighbours:
            if nxt in explored:
                continue

            edge_length = graph.edges[node, nxt]['length']
            total_length = edge_length
            found = False

            for prev_node, path_lengths in lengths.items():
                prev_edge = (prev_node, node)
                if prev_edge in path_lengths:
                    total_length += path_lengths[prev_edge]
                    found = True
                    break  # Only need one valid previous path

            tmp[(node, nxt)] = total_length

            if total_length <= sensor_range:
                node_coverage.append((node, nxt))
                graph.edges[node, nxt]['covered'] = 'yes'

            queue.append(nxt)

        lengths[node] = tmp

    return node_coverage


def sensors_leakage_problem(G, sensor_coverage, sensor_range):
    """
    Finds the leakage in a graph by finding the possible leakage edges and placing sensors to cover them.

    Parameters
    ----------
    G : NetworkX Graph
        The graph to find the leakage in
    sensor_coverage : dict
        A dictionary with the nodes as keys and the edges that can be covered from them as values
    sensor_range : float
        The range of the sensors

    Returns
    -------
    new_sensors : list
        A list of the new sensors that were placed to cover the leakage
    """
    # Find the possible leakage edges
    find_leakage(G, sensor_coverage, sensor_range)

    new_sensors = []
    iters = 0
    
    # Place a sensor on each leaf of the graph
    for (x, y) in G.edges:
        if G.edges[(x, y)]['possible_leakage'] == 'yes':
            neighbours_x = (list(G.adj.items())[x])[1]
            neighbours_y = (list(G.adj.items())[y])[1]
            if len(neighbours_x) == 1:
                # If the node is a leaf, place a sensor on it
                G.nodes[x]['sensor'] = 'yes'
                new_sensors.append(x)
                # Find the edges that can be covered from the new sensor
                sensor_coverage[x] = find_covered_edges_from_sensor(G, x, sensor_range)
                # Find the possible leakage edges from the new sensor
                find_leakage_sensor(G, x, sensor_coverage, sensor_range)
            elif len(neighbours_y) == 1:
                # If the node is a leaf, place a sensor on it
                G.nodes[y]['sensor'] = 'yes'
                new_sensors.append(y)
                # Find the edges that can be covered from the new sensor
                sensor_coverage[y] = find_covered_edges_from_sensor(G, y, sensor_range)
                # Find the possible leakage edges from the new sensor
                find_leakage_sensor(G, y, sensor_coverage, sensor_range)

    # Place a sensor to cover all the possible leakage edges
    for (x, y) in G.edges:
        if G.edges[(x, y)]['possible_leakage'] == 'yes':
            # Place a sensor on the edge
            G.nodes[y]['sensor'] = 'yes'
            new_sensors.append(y)
            # Find the edges that can be covered from the new sensor
            sensor_coverage[y] = find_covered_edges_from_sensor(G, y, sensor_range)
            # Find the possible leakage edges from the new sensor
            find_leakage_sensor(G, y, sensor_coverage, sensor_range)

    # Find the edges that are not explored because of the shortest path algorithm and fix them
    find_shortest_path_errors(G, sensor_coverage, sensor_range, new_sensors)


def find_shortest_path_errors(G, sensor_coverage, sensor_range, new_sensors):
    """
    Identifies and fixes errors in edge coverage caused by the shortest path algorithm.
    Temporary sensors are placed on nodes to evaluate if path coverage can be improved.

    Parameters
    ----------
    G : NetworkX Graph
        The graph to analyze for shortest path errors.
    sensor_coverage : dict
        Dictionary mapping nodes to edges they can cover.
    sensor_range : float
        The maximum range of the sensors.
    new_sensors : list
        List to append new sensors added during this process.

    Returns
    -------
    None
    """

    # Find edges still marked as possible leakage
    remaining_possible_leakage = [
        (x, y) for (x, y) in G.edges if G.edges[(x, y)]['possible_leakage'] == 'yes'
    ]
    
    xs = [x for (x, y) in remaining_possible_leakage]
    ys = [y for (x, y) in remaining_possible_leakage]

    # Identify nodes involved in more than one possible leakage edge
    nodes = list(set(x for x in xs if xs.count(x) > 1) | set(y for y in ys if ys.count(y) > 1))

    # Temporarily make nodes sensors, compute paths, and check if coverage can be improved
    paths = {}
    paths_length = []
    for node in nodes:
        G.nodes[node]['sensor'] = 'yes'
        new_sensors.append(node)
        sensor_coverage[node] = find_covered_edges_from_sensor(G, node, sensor_range)

        # Find destinations for paths
        destinations = list(set(
            x if G.nodes[x]['sensor'] == 'yes' else y
            for (x, y) in sensor_coverage[node]
            if x != node and y != node
        ))

        # Compute shortest paths from node to each destination
        tmp_paths = []
        for dest in destinations:
            for path in nx.all_shortest_paths(G, node, dest, weight='length'):
                tmp_length = sum(G.edges[path[i], path[i + 1]]['length'] for i in range(len(path) - 1))
                paths_length.append(tmp_length)
                tmp_paths.append(path)
        paths[node] = tmp_paths

    # Check if any two paths can be concatenated within sensor range
    for node, node_paths in paths.items():
        for path1 in node_paths:
            for path2 in node_paths:
                if path1 == path2 or path2[1] == path1[1]:
                    continue
                length1 = sum(G.edges[path1[i], path1[i + 1]]['length'] for i in range(len(path1) - 1))
                length2 = sum(G.edges[path2[i], path2[i + 1]]['length'] for i in range(len(path2) - 1))
                if length1 + length2 <= sensor_range:
                    G.edges[(node, path1[1])]['possible_leakage'] = 'no'
                    G.edges[(node, path2[1])]['possible_leakage'] = 'no'

        # Revert node to non-sensor
        G.nodes[node]['sensor'] = 'no'


def find_leakage_sensor(graph, sensor, sensor_coverage, sensor_range):
    """
    Finds the possible leakage edges this sensor can cover.

    Parameters
    ----------
    graph : NetworkX Graph
        The graph to find the possible leakage edges in
    sensor : int
        The sensor node
    sensor_coverage : dict
        The dictionary of the sensors and their coverage
    sensor_range : float
        The range of the sensors

    Returns
    -------
    None
    """
    paths = []
    # Find the possible leakage edges
    xs = []
    ys = []
    destinations = []
    for (x, y) in sensor_coverage[sensor]:
        xs.append(x)
        ys.append(y)
    for y in set(ys):
        if y not in set(xs):
            destinations.append(y)
    for dest in destinations:
        for path in nx.all_shortest_paths(graph, sensor, dest, weight='length'):
            paths.append(path)
            index_of_sensor = -1
            no_sensors = 0
            for node in path:
                index_of_sensor += 1
                if node != sensor and graph.nodes[node]['sensor'] == 'yes':
                    break
                elif node != sensor and graph.nodes[node]['sensor'] == 'no':
                    no_sensors += 1

            if no_sensors != (len(path) - 1):
                for i in range(0, index_of_sensor):
                    if graph.edges[path[i], path[i + 1]]['possible_leakage'] == 'yes':
                        graph.edges[path[i], path[i + 1]]['possible_leakage'] = 'no'


def find_leakage(graph, sensor_coverage, sensor_range):
    """
    Finds all the possible leakage edges in the graph.

    Parameters
    ----------
    graph : NetworkX Graph
        The graph to find the possible leakage edges in
    sensor_coverage : dict
        The dictionary of the sensors and their coverage
    sensor_range : float
        The range of the sensors

    Returns
    -------
    None
    """
    paths = []
    # Find the possible leakage edges
    for sensor in sensor_coverage:
        xs = []
        ys = []
        destinations = []
        for (x, y) in sensor_coverage[sensor]:
            xs.append(x)
            ys.append(y)
        for y in set(ys):
            if y not in set(xs):
                destinations.append(y)
        for dest in destinations:
            for path in nx.all_shortest_paths(graph, sensor, dest, weight='length'):
                prev = path[-2]
                node = path[-1]
                paths.append(path)
                neighbours = (list(graph.adj.items())[node])[1]

                # If the node has only one neighbour, mark it as possible leakage
                if len(neighbours) == 1:
                    graph.edges[(prev, node)]['possible_leakage'] = 'yes'
                # If the node and its previous node are not sensors, mark it as possible leakage
                elif graph.nodes[node]['sensor'] == 'no' and graph.nodes[prev]['sensor'] == 'no':
                    graph.edges[(prev, node)]['possible_leakage'] = 'yes'

    # Remove the invalid possible leakage edges
    for (x, y) in graph.edges:
        if graph.edges[(x, y)]['possible_leakage'] == 'yes':
            for path in paths:
                if x in path and y in path and (path.index(y) - path.index(x) == 1) and graph.nodes[path[-1]][
                    'sensor'] == 'yes':
                    graph.edges[(x, y)]['possible_leakage'] = 'no'


def print_graph_with_sensors_grid(graph, start, pos, title=''):
    """
    Prints a graph with sensors and new nodes using a grid layout.

    Parameters
    ----------
    graph : NetworkX Graph
        The graph to print
    start : int
        The starting node of the graph
    pos : dict
        A dictionary with the positions of the nodes in the graph
    title : str
        The title of the graph

    Returns
    -------
    None
    """
    # Get the positions of the nodes
    pos = nx.get_node_attributes(graph, 'pos')
    node_color_map = []
    edge_color_map = []

    # Set the colors of the nodes
    for i in range(graph.number_of_nodes()):
        if i == start and graph.nodes[i]['sensor'] == 'yes':
            # Starting node and sensor
            node_color_map.append('cyan')
        elif i == start:
            # Starting node
            node_color_map.append('green')
        elif graph.nodes[i]['sensor'] == 'yes':
            # Sensor
            node_color_map.append('red')
        elif graph.nodes[i]['new'] == 'yes':
            # New node
            node_color_map.append('yellow')
        else:
            # Normal node
            node_color_map.append('blue')

    # Set the colors of the edges
    for edge in graph.edges:
        if graph.edges[edge]['possible_leakage'] == 'yes':
            # Possible leakage
            edge_color_map.append('green')
        else:
            # Not possible leakage
            edge_color_map.append('red')

    # Get the lengths of the edges
    lengths = nx.get_edge_attributes(graph, 'length')

    # Plot the graph
    plt.figure(figsize=(25, 25))
    plt.title(title)
    nx.draw(graph, pos, with_labels=True, font_size=10, node_color=node_color_map, edge_color=edge_color_map)
    nx.draw_networkx_edge_labels(graph, pos, edge_labels=lengths, font_size=7)
    plt.show()
    

def print_graph_with_sensors(graph, start, pos, title=''):
    """
    Prints a graph with sensors and new nodes using a spring layout.

    Parameters
    ----------
    graph : NetworkX Graph
        The graph to print
    start : int
        The starting node of the graph
    pos : dict
        A dictionary with the positions of the nodes in the graph
    title : str
        The title of the graph

    Returns
    -------
    None
    """
    # Get the positions of the nodes
    # pos = nx.get_node_attributes(graph, 'pos')

    # Set the colors of the nodes
    node_color_map = []
    for i in range(graph.number_of_nodes()):
        if i == start and graph.nodes[i]['sensor'] == 'yes':
            # Starting node and sensor
            node_color_map.append('cyan')
        elif i == start:
            # Starting node
            node_color_map.append('green')
        elif graph.nodes[i]['sensor'] == 'yes':
            # Sensor
            node_color_map.append('red')
        elif graph.nodes[i]['new'] == 'yes':
            # New node
            node_color_map.append('yellow')
        else:
            # Normal node
            node_color_map.append('blue')

    # Set the colors of the edges
    edge_color_map = []
    for edge in graph.edges:
        if graph.edges[edge]['possible_leakage'] == 'yes':
            # Possible leakage
            edge_color_map.append('green')
        else:
            # Not possible leakage
            edge_color_map.append('red')

    # Get the lengths of the edges
    lengths = nx.get_edge_attributes(graph, 'length')

    # Plot the graph
    plt.figure(figsize=(25, 25))
    plt.title(title)
    nx.draw(graph, pos, with_labels=True, font_size=10, node_color=node_color_map, edge_color=edge_color_map)
    nx.draw_networkx_edge_labels(graph, pos, edge_labels=lengths, font_size=7)
    plt.show()
