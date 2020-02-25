import networkx as nx
import math

def write_pushed_nodes(output_file, variable_name, pushed_nodes, path_variable, path):
    with open(output_file, 'w') as file:
        file.write('var '+variable_name+'=[\n')
        for i in range(len(pushed_nodes)):
            node = pushed_nodes[i]
            
            line = '['+str(nx.get_node_attributes(G, 'attr_dict')[node]['lat'])+','+str(nx.get_node_attributes(G, 'attr_dict')[node]['lon'])+']'
            if i != len(pushed_nodes) - 1:
                line += ',\n'
            else:
                line += '];\n'
            file.write(line)
        file.write('var ' + path_variable + '=[\n')
        for i in range(len(path)):
            node = path[i]
            line = '['+str(nx.get_node_attributes(G, 'attr_dict')[node]['lat'])+','+str(nx.get_node_attributes(G, 'attr_dict')[node]['lon'])+']'
            if i != len(path) - 1:
                line += ',\n'
            else:
                line += '];\n'
            file.write(line)  
            
# load the map around Lewis
min_lat = 41.545409
max_lat = 41.647551
min_lon = -88.174645
max_lon = -87.974142

MG = ox.graph_from_bbox(max_lat, min_lat, max_lon, min_lon, simplify=False)
MG_projected = ox.project_graph(MG)
G = nx.Graph()
for u,v,data in MG_projected.edges(data=True):
    if G.has_edge(u,v):
        if G.get_edge_data(u,v)['length'] != data['length']:
            print('extra edge not the same length',G.get_edge_data(u,v)['length'],data['length'])
    else:
        if not G.has_node(u):
            G.add_node(u, attr_dict = MG_projected.nodes[u])
        if not G.has_node(v):
            G.add_node(v, attr_dict = MG_projected.nodes[v])
        G.add_edge(u, v, osmid=data['osmid'], highway=data['highway'], length=data['length'])

# find the shortest path from an east location to Lewis
source=[41.590134, -88.115686] # A location eastern to Lewis
destination = [41.605080, -88.080890] # A location in Lewis
min_source_dist = 999999
min_destination_dist = 999999
source_node = None
destination_node = None
for node in G.nodes():
    lat = nx.get_node_attributes(G, 'attr_dict')[node]['lat']
    lon = nx.get_node_attributes(G, 'attr_dict')[node]['lon']
    source_dist = math.sqrt((source[0]-lat)*(source[0]-lat) + (source[1]-lon)*(source[1]-lon))
    if source_dist < min_source_dist:
        source_node = node
        min_source_dist = source_dist
    destination_dist = math.sqrt((destination[0]-lat)*(destination[0]-lat) + (destination[1]-lon)*(destination[1]-lon))
    if destination_dist < min_destination_dist:
        destination_node = node
        min_destination_dist = destination_dist

path, pushed_nodes, dijkstra_dist = astar_path(G, source_node, destination_node, weight='length')
write_pushed_nodes('east_dijkstra_nodes.js', 'east_dijkstra_nodes', pushed_nodes, 'east_dijkstra_path', path)
path, pushed_nodes, astar_dist = astar_path(G, source_node, destination_node, heuristic=heuristic, weight='length')
write_pushed_nodes('east_astar_nodes.js', 'east_astar_nodes', pushed_nodes, 'east_astar_path', path)

# find the shortest path from a west location to Lewis
source = [41.597274, -88.048803] # A location western to Lewis
destination = [41.605080, -88.080890] # A location in Lewis
min_source_dist = 999999
min_destination_dist = 999999
source_node = None
destination_node = None
for node in G.nodes():
    lat = nx.get_node_attributes(G, 'attr_dict')[node]['lat']
    lon = nx.get_node_attributes(G, 'attr_dict')[node]['lon']
    source_dist = math.sqrt((source[0]-lat)*(source[0]-lat) + (source[1]-lon)*(source[1]-lon))
    if source_dist < min_source_dist:
        source_node = node
        min_source_dist = source_dist
    destination_dist = math.sqrt((destination[0]-lat)*(destination[0]-lat) + (destination[1]-lon)*(destination[1]-lon))
    if destination_dist < min_destination_dist:
        destination_node = node
        min_destination_dist = destination_dist

path, pushed_nodes, dijkstra_dist = astar_path(G, source_node, destination_node, weight='length')
write_pushed_nodes('west_dijkstra_nodes.js', 'west_dijkstra_nodes', pushed_nodes, 'west_dijkstra_path', path)
path, pushed_nodes, astar_dist = astar_path(G, source_node, destination_node, heuristic=heuristic, weight='length')
write_pushed_nodes('west_astar_nodes.js', 'west_astar_nodes', pushed_nodes, 'west_astar_path', path)
