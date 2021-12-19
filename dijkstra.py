import sys


class Graph(object):
    def __init__(self, nodes, init_graph):
        self.nodes = nodes
        self.graph = self.construct_graph(nodes, init_graph)

    def construct_graph(self, nodes, init_graph):
        '''
        This method makes sure that the graph is symmetrical. In other words, if there's a path from
        node A to B with a value V, there needs to be a path from node B to node A with a value V.
        '''
        graph = {}
        for node in nodes:
            graph[node] = {}

        graph.update(init_graph)

        for node, edges in graph.items():
            for adjacent_node, value in edges.items():
                if graph[adjacent_node].get(node, False) == False:
                    graph[adjacent_node][node] = value

        return graph

    def get_nodes(self):
        "Returns the nodes of the graph."
        return self.nodes

    def get_outgoing_edges(self, node):
        "Returns the neighbors of a node."
        connections = []
        for out_node in self.nodes:
            if self.graph[node].get(out_node, False) != False:
                connections.append(out_node)
        return connections

    def value(self, node1, node2):
        "Returns the value of an edge between two nodes."
        return self.graph[node1][node2]

# The Dijkstra algorithm:
# "graph" is a dictionary of nodes and their connections
# "start_node" is the node we want to start from
def dijkstra_algorithm(graph, start_node):
    # Initialize a list of unvisited nodes
    unvisited_nodes = list(graph.get_nodes())

    # Create two dicts:
    # "shortest_path" will hold the shortest path to each node
    # "previous_nodes" stores the trajectory of the best known path to each node
    shortest_path = {}
    previous_nodes = {}

    # Use "max_value" to initialize the "infinity" value of the unvisited nodes
    max_value = sys.maxsize

    for node in unvisited_nodes:
        shortest_path[node] = max_value

    # Initialize the starting node value with zero
    shortest_path[start_node] = 0

    # While there are still unvisited nodes in the graph
    while unvisited_nodes:
        # Find the node with the lowest value
        current_min_node  = None
        for node in unvisited_nodes:
            if current_min_node is None:
                current_min_node = node
            elif shortest_path[node] < shortest_path[current_min_node]:
                current_min_node = node

        # Once node is found, visit all of its unvisited neighbors.
        # If a new path is better than the current best path,
        # update the shortest_path and previous_nodes dictionaries.
        neighbors = graph.get_outgoing_edges(current_min_node)
        for neighbor in neighbors:
            tentative_value = (
                shortest_path[current_min_node] + graph.value(current_min_node, neighbor)
            )
            if tentative_value < shortest_path[neighbor]:
                shortest_path[neighbor] = tentative_value
                # Also update the best path to the current node
                previous_nodes[neighbor] = current_min_node

        # Remove the current node from unvisited list after visiting all of its neighbors
        unvisited_nodes.remove(current_min_node)

    # Return the two dictionaries
    return previous_nodes, shortest_path


# Helper function to print results and accepts arguments:
# "previous_nodes" is a dictionary of nodes and their best known path to the start node
# "shortest_path" - holds the shortest path to each node
# "start_node" - the starting node (origin)
# "target_node" - the target node (destination)
def print_result(previous_nodes, shortest_path, start_node, target_node):
    path = []
    node = target_node

    while node != start_node:
        path.append(node)
        node = previous_nodes[node]

    # Add the starting node to the path
    path.append(start_node)

    print("Thes best path with a value of {}.".format(shortest_path[target_node]))
    print(" -> ".join(reversed(path)))


# Main function
nodes = ["Reykjavik", "Oslo", "Moscow", "London", "Rome", "Berlin", "Belgrade", "Athens"]

init_graph = {}
for node in nodes:
    init_graph[node] = {}

init_graph["Reykjavik"]["Oslo"] = 5
init_graph["Reykjavik"]["London"] = 4
init_graph["Oslo"]["Berlin"] = 1
init_graph["Oslo"]["Moscow"] = 3
init_graph["Moscow"]["Belgrade"] = 5
init_graph["Moscow"]["Athens"] = 4
init_graph["Athens"]["Belgrade"] = 1
init_graph["Rome"]["Berlin"] = 2
init_graph["Rome"]["Athens"] = 2

# Construct an object of the Graph class using the above values
graph = Graph(nodes, init_graph)

# Pass the constructed graph and the starting node to the Dijkstra algorithm
previous_nodes, shortest_path = dijkstra_algorithm(graph, "Reykjavik")

# Print the results
print_result(previous_nodes, shortest_path, "Reykjavik", "Belgrade")
