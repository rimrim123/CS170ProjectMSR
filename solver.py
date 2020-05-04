'''Solver for 170 Project'''
'''README - cd into project directory and run python3 solver.py, then
run python3 prepare_submission.py outputs/ submission.json to get the json file
of the outputs
'''

import networkx as nx
from parse import read_input_file, write_output_file
from utils import is_valid_network, average_pairwise_distance, average_pairwise_distance_fast
import sys
import os
from networkx.algorithms.approximation import min_weighted_dominating_set
from networkx.algorithms.traversal.breadth_first_search import bfs_tree
from networkx.algorithms.components import connected_components



def solve(G):
	"""
	Args:
		G: networkx.Graph

	Returns:
		T: networkx.Graph
	"""


	def helper2(G):
		T = nx.minimum_spanning_tree(G)
		curr_lowest = average_pairwise_distance(T)
		curr_lowest_tree = T

		S = min_weighted_dominating_set(T)

		newG = nx.subgraph(T, S)

		ncc = nx.number_connected_components(newG)
		ccs = list(connected_components(newG))


		for i in range (len(ccs) - 1):
			curr_node = ccs[i].pop()
			ccs[i].add(curr_node)
			next_node = ccs[i+1].pop()
			ccs[i+1].add(next_node)
			path = nx.dijkstra_path(G, curr_node, next_node)


			for n in path:
				if (n not in list(newG.nodes)):
					S.add(n)

			newG = nx.subgraph(G, S)
			newT = nx.minimum_spanning_tree(newG)
			if (is_valid_network(G, newT)):
				apd = average_pairwise_distance(newT)
				if (apd < curr_lowest):
					curr_lowest = apd
					curr_lowest_tree = newT

		return curr_lowest_tree

	def remove_attribute(G,tnode,attr):
		G.node[tnode].pop(attr,None)

	def helper(G, start):

		visited2 = [] # List to keep track of visited nodes.
		queue2 = []     #Initialize a queue

		def bfs_set_weights(visited, graph, node):
			visited2.append(node)
			queue2.append(node)

			node_weights = {}
			node_weights[node] = 1

			while queue2:
				s = queue2.pop(0)
				for neighbour in G.neighbors(s):
					if neighbour not in visited2:
						visited2.append(neighbour)
						queue2.append(neighbour)
						node_weight = G.get_edge_data(neighbour,s)['weight']
						node_weights[neighbour] = node_weight
			return node_weights

		node_weights_dict = bfs_set_weights(visited2, G, start)
		nx.set_node_attributes(G, node_weights_dict, 'node_weight')
		D = min_weighted_dominating_set(G, 'node_weight')
		for node2 in list(G.nodes):
			remove_attribute(G, node2, 'node_weight')

		visited = [] # List to keep track of visited nodes.
		queue = []     #Initialize a queue
		def bfs(visited, graph, node):
			visited.append(node)
			queue.append(node)
			level_tracker = {}
			level_tracker[node] = 0

			levels = {}
			levels[0] = [[node], [node, True]]





			while queue:
				s = queue.pop(0)

				for neighbour in G.neighbors(s):
					if neighbour not in visited:
						visited.append(neighbour)
						queue.append(neighbour)
						level_count = level_tracker.get(s) + 1
						level_tracker[neighbour] = level_count
						if neighbour in D:
							dom_set = True
						else:
							dom_set = False
						if level_count in levels.keys():
							if dom_set == True:
								levels[level_count].append([neighbour, dom_set])
								levels[level_count][0].append(neighbour)
							else:
								levels[level_count].append([neighbour, dom_set])
						else:
							if dom_set == True:
								levels[level_count] = [[neighbour], [neighbour, dom_set]]
							else:
								levels[level_count] = [[], [neighbour, dom_set]]


			return levels

		bfs_levels = bfs(visited, G, start)
		#print(bfs_levels)
		#print(list(G.edges))
		#print('edge data ', G.get_edge_data(0,1,default=0)['weight'])


		leaf_level = max(bfs_levels.keys())

		level = leaf_level
		d_set_levels = []
		while level >= 0:
			if bfs_levels[level][0]:
				d_set = bfs_levels[level][0]

				d_set_levels.append(d_set)
			level = level - 1

		newG = nx.Graph()
		for i in range(len(d_set_levels)-1):
			d_level = d_set_levels[i]
			level_above = d_set_levels[i+1]
			for d_node in d_level:
				for possible_parent in level_above:
					if nx.has_path(G, d_node, possible_parent):
						parent = possible_parent
						path = nx.dijkstra_path(G, d_node, parent, 'weight')
						path_sub = G.subgraph(path).copy()
						newG.update(path_sub)
		#print(list(newG.edges))

		T = nx.minimum_spanning_tree(newG)
		if list(newG.edges) == []:
			T.add_node(d_set_levels[0][0])
			return T
		return T

	#Running through all start nodes

	all_nodes = list(G.nodes)
	minT = nx.minimum_spanning_tree(G)
	sumin_tree = helper2(G)
	curr_lowest_tree = sumin_tree
	curr_lowest = average_pairwise_distance_fast(sumin_tree)
	for start in all_nodes:
		helper_graph = helper(G, start)
		if (is_valid_network(G, helper_graph)):
			apd = average_pairwise_distance_fast(helper_graph)
			if (apd < curr_lowest):
				curr_lowest = apd
				curr_lowest_tree = helper_graph
	#print("MST", average_pairwise_distance_fast(minT))
	#print("currLowest", curr_lowest)
	return curr_lowest_tree














'''
	if bfs_levels[leaf_level][0] == True:
		d_set = []
		for v in bfs_levels[leaf_level][1:]:
			if v[1] = True:
				d_set.append(v[0])
		for d in d_set:
			print('dd')
'''


	#return nx.minimum_spanning_tree(G)







 #Here's an example of how to run your solver.

 #Usage: python3 solver.py test.in

if __name__ == '__main__':
	output_dir = "outputs"
	input_dir = "inputs"
	for input_path in os.listdir(input_dir):
		graph_name = input_path.split(".")[0]
		G = read_input_file(f"{input_dir}/{input_path}")
		T = solve(G)
		assert is_valid_network(G, T)
		# print(input_path, "- ", ": {}".format(average_pairwise_distance(T)))
		write_output_file(T, f"{output_dir}/{graph_name}.out")
'''
	assert len(sys.argv) == 2
	path = sys.argv[1]
	G = read_input_file(path)
	T = solve(G)
	assert is_valid_network(G, T)
	print("Average  pairwise distance: {}".format(average_pairwise_distance(T)))
	write_output_file(T, 'out/test.out')
'''
