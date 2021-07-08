from collections import Counter, defaultdict
from random import SystemRandom
from time import struct_time
from networkx import linalg
from sklearn.neighbors import NearestNeighbors
from sklearn.neighbors import kneighbors_graph
import matplotlib.pyplot as plt
import networkx as nx
from pyvis.network import Network
from math import dist
from operator import itemgetter
import time

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

  def checkPath(path):
    if(final_path):
      return True
    else:
      print("BFS: Caminho não encontrado")
      return False

  def drawGraph(graph, posDict, path, nq, eq):
    # PRINT GRAPH
    G = nx.Graph()
    G.add_nodes_from(posDict.keys())

    color_map = []
    for node in G:
      if node in path:
        color_map.append('red')
      else:
        color_map.append('blue')

    for i in range(nq):
      for j in range(eq):
        G.add_edge(i, graph[i].indices[j])

    nx.draw(G, posDic, node_color=color_map, with_labels=False, node_size=50)
    plt.show()

  def calcDistance(n1, n2):
    return dist(n1, n2)
  

# BUSCA POR PROFUNDIDADE (DFS) #
class DFS:

  def DFS(graph, start, goal):
    stack, path = [start], []
    goal_flag = False

    while stack:
      node = stack.pop()

      if node == goal:
        print("===== Achou =====")
        goal_flag = True
        path.append(node)
        break
      if node in path:
        continue
      path.append(node)
      #print("i: ",node, graph[node].indices)
      for neighbor in graph[node].indices:
        stack.append(neighbor)

    #print(path)
    if(goal_flag):
      print("DFS - Nº de nos visitado: ", len(path))
      return path

# BUSCA POR LARGURA (BFS) #
class BFS:

  def BFS(graph, s, final, size): 
    #marca todos os vértices como não visitados.
    visited = [False] * size   

    #cria uma fila vazia para o BFS 
    queue = [] 
    path = []
  
    #pega o nó de origem, marca como visitado e insere ele na fila
    queue.append(s)
    visited[s] = True 
    
    #enquanto a fila não for vazia
    while queue:  
      #retira o último vértice inserido na fila e imprime
      s = queue.pop(0) 
      #print(s, " ")  
      if s == final :
        print("===== Achou =====")
        queue.append(s)
        #print(queue)
        for i in range(size):
          if(visited[i] == True):
            path.append(i)
        print("BFS - Nº de nos visitado: ", len(path))
        return path

      for i in graph[s].indices:
        if visited[i] == False: 
          queue.append(i) 
          visited[i] = True
          #print(queue)
  
  # BUSCA BEST FIRST #
class BestFirst:
    def BestFirst(graph,nodeList,start,goal ):

      open_list = [{
        'index': 0,
        'h': 0,
        'path': [start]
      }]
      closed_list = []

      while len(open_list) > 0:
        current_node = open_list.pop(0)
        closed_list.append(current_node['index'])
        # Found the goal
        if current_node['index'] == goal:
          print("===== Achou =====")
          print("A* - Nº de nos visitado: ", len(current_node['path']))
          return current_node['path']

        for neighbor in graph[current_node['index']].indices:
          if neighbor not in closed_list:
            nodeDict = {
              'index': neighbor,
              'h': dist(nodeList[current_node['index']],nodeList[neighbor]),
              'path': current_node['path'] + [neighbor]
            }
            open_list.append(nodeDict)
            open_list = sorted(open_list, key=itemgetter('h'))

      return None

  # A & A* #
class aCommon:
  def aCommon(graph, start, goal):

    open_list = [{
      'index': 0,
      'path': [start]
    }]
    closed_list = []

    visit = []
    visit.append(start)

    while len(open_list) > 0:
      current_node = open_list.pop(0)
      closed_list.append(current_node['index'])

      x = current_node['index']
      visit.append(x)
      # Found the goal
      if current_node['index'] == goal:
        print("===== Achou =====")
        #print("A - Nº de nos visitado: ", len(current_node['path']))
        #return current_node['path']
        print("A - Nº de nos visitado: ", len(visit))
        return visit


      for neighbor in graph[current_node['index']].indices:
        if neighbor not in closed_list:
          nodeDict = {
            'index': neighbor,
            'path': current_node['path']+[neighbor]
          }
          open_list.append(nodeDict)

    return None

class aStar:
  def aStrar(graph, nodePosList, start, goal):
    
    nodeDistList = []
    for i in range(len(nodePosList)):
      nodeDistList.append(Utils.calcDistance(nodePosList[i], nodePosList[goal]))

    open_list = [{
      'index': 0,
      'g': 0,
      'h': nodeDistList[start],
      'f': 0,
      'path': [start]
    }]
    closed_list = []

    visit = []
    visit.append(start)

    while len(open_list) > 0:
      current_node = open_list.pop(0)
      closed_list.append(current_node['index'])
      
      x = current_node['index']
      visit.append(x)

      # Found the goal
      if current_node['index'] == goal:
        print("===== Achou =====")
        #print("A* - Nº de nos visitado: ", len(current_node['path']))
        #return current_node['path']
        print("A - Nº de nos visitado: ", len(visit))
        return visit


      for neighbor in graph[current_node['index']].indices:
        if neighbor not in closed_list:
          nodeDict = {
            'index': neighbor,
            'g': current_node['g'] + dist(nodePosList[current_node['index']], nodePosList[neighbor]),
            'h': nodeDistList[neighbor],
            'f': 0,
            'path': current_node['path']+[neighbor]
          }
          nodeDict['f'] = nodeDict['g']+nodeDict['h']
          open_list.append(nodeDict)
          open_list = sorted(open_list, key=itemgetter('f'))
    
    return None


######### VARIAVEIS #############
coordMinValue = 1
coordMaxValue = 501
nodeQuantity = 500
edgeQuantity = 5
start = 0
goal = 499

######### CRIAR ARRAY DE NOS ####
print("=======================================")
coordinatesArray = Utils.generateArray(coordMinValue, coordMaxValue, nodeQuantity)
#Utils.printArray(coordinatesArray)

posDic = {}
i=0
for node in coordinatesArray:
  posDic[i] = coordinatesArray[i]
  i+=1

######### MONTAGEM DO GRAFO #####
grp = kneighbors_graph(coordinatesArray, edgeQuantity, mode='distance', p=2)
print(coordinatesArray[0], coordinatesArray[499])
print("---------------------------------------")
final_path = False



start_time = time.time()
#final_path = DFS.DFS(grp, start, goal)
end_time = time.time()
print("DFS: ", end_time - start_time)
if(Utils.checkPath(final_path)): Utils.drawGraph(grp, posDic, final_path, nodeQuantity, edgeQuantity)

start_time = time.time()
#final_path = BFS.BFS(grp, start, goal, nodeQuantity)
end_time = time.time()
print("BFS: ", end_time - start_time)
if(Utils.checkPath(final_path)): Utils.drawGraph(grp, posDic, final_path, nodeQuantity, edgeQuantity)

start_time = time.time()
#final_path = aStar.aStrar(grp, coordinatesArray, start, goal)
end_time = time.time()
print("A*: ", end_time - start_time)
if(Utils.checkPath(final_path)): Utils.drawGraph(grp, posDic, final_path, nodeQuantity, edgeQuantity)

start_time = time.time()
#final_path = aCommon.aCommon(grp, start, goal)
end_time = time.time()
print("A: ", end_time - start_time)
if(Utils.checkPath(final_path)): Utils.drawGraph(grp, posDic, final_path, nodeQuantity, edgeQuantity)

final_path = BestFirst.BestFirst(grp,coordinatesArray,start,goal)
if(Utils.checkPath(final_path)): Utils.drawGraph(grp, posDic, final_path, nodeQuantity, edgeQuantity)

