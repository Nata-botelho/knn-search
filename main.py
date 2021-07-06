from collections import defaultdict
from random import SystemRandom
from sklearn.neighbors import NearestNeighbors
from sklearn.neighbors import kneighbors_graph
import matplotlib.pyplot as plt
import networkx as nx
from pyvis.network import Network

class Utils:
  def generateArray(minV, maxV, k):
    coordinatesArray = []
    random = SystemRandom()
    for _ in range(k):
      coordinatesArray.append((random.randint(minV, maxV), random.randint(minV, maxV)))
    return coordinatesArray

  def printArray(_array):
    for i in _array:
      print(i)


  

# BUSCA POR PROFUNDIDADE (DFS) #
class DFS:
  
  def DFS(graph, start, goal):
    stack, path = [start], []

    while stack:
      node = stack.pop()
      if node == goal:
        break
      if node in path:
        continue
      path.append(node)
      print("i: ",node, graph[node].indices)
      for neighbor in graph[node].indices:
        stack.append(neighbor)
    
    print(path)
    return path

# BUSCA POR LARGURA (BFS) #
class BFS:

  def BFS(graph, s, final): 
    #marca todos os vértices como não visitados.
    visited = [False] * 500       

    #cria uma fila vazia para o BFS 
    queue = []  

    #pega o nó de origem, marca como visitado e insere ele na fila
    queue.append(s) 
    visited[s] = True 
    
    #enquanto a fila não for vazia
    while queue:  
      #retira o último vértice inserido na fila e imprime
      s = queue.pop(0) 
      #print(s, " ")  
      if s == final :
        print("===== Achou=====")
        queue.append(s)
        print(queue)
        return
      for i in graph[s].indices:
        if visited[i] == False: 
          queue.append(i) 
          visited[i] = True
          print(queue)
  
  # BUSCA BEST FIRST #
class BestFirst:
    def BestFirst(graph, start, goal):
        
        return 


  # A & A* #
class aStar:
  def aStrar(graph, start, goal):
    closed = []
    open = []
    path = {}
    path[start] = start
    open.append(start)

    while open:
      current_node = open.pop(0)
      closed.append(current_node)

      if current_node == goal:
        reconst_path = []
        while current_node != start:
          reconst_path.append(current_node)
          current_node = path[current_node]
        reconst_path.append(start)
        reconst_path.reverse()
        return reconst_path

      for neighbor in graph[current_node].indices:
        if neighbor not in open and neighbor not in closed:
          open.add(neighbor)
          path[neighbor] = current_node
        else:
          if neighbor in closed:
            closed.remove(neighbor)
            open.add(neighbor)

      open.remove(current_node)
      closed.add(current_node)
    return None


######### VARIAVEIS #############
coordMinValue = 1
coordMaxValue = 501
nodeQuantity = 500
edgeQuantity = 3
start = 0
goal = 499

######### CRIAR ARRAY DE NOS ####
print("=======================================")
coordinatesArray = Utils.generateArray(coordMinValue, coordMaxValue, nodeQuantity)
posDic = {}
i=0
for node in coordinatesArray:
  posDic[i] = coordinatesArray[i]
  i+=1

#Utils.printArray(coordinatesArray)

######### MONTAGEM DO GRAFO #####
grp = kneighbors_graph(coordinatesArray, edgeQuantity, mode='distance', p=2)
print(coordinatesArray[0], coordinatesArray[499])
print("---------------------------------------")
final_path = DFS.DFS(grp, start, goal)
#BFS.BFS(grp,start, goal)


# PRINT GRAPH
G = nx.Graph()
G.add_nodes_from(posDic.keys())

color_map = []
for node in G:
  if node in final_path:
    color_map.append('red')
  else:
    color_map.append('cyan')

for i in range(nodeQuantity):
  for j in range(edgeQuantity):
    G.add_edge(i, grp[i].indices[j])
nx.draw(G, posDic, node_color=color_map, with_labels=True)
plt.show()
