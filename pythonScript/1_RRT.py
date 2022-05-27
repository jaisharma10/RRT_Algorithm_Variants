# =============================================================
#                       RRT algorithm
# =============================================================

"""
Authors: Jai Sharma and Divyansh Agrawal
Assignment: Project 5 - RRT
Task: implement RRT algorithm on 2d Map for Point Robot
"""
# import Libraries

import math
import random
import sys
import time
import pygame

# =============================================================
#                       RRT class
# =============================================================

class RRT:
    
    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []   # Path to x
            self.path_y = []   # Path to y
            self.parent = None
            
        def __repr__(self):
            return f'X: {self.x},Y: {self.y}'

    # initialize the object's states
    def __init__(self,start,goal,rand_area,expand_dis=0.1,path_resolution=0.05, goal_sample_rate = 1, max_iter=8000):
        self.start = self.Node(start[0], start[1])   # start node [x,y]
        self.end = self.Node(goal[0], goal[1])       # goal node [x,y]
        self.min_rand = rand_area[0]                 # sampling area has a range [min, max]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis                # step size, set to 0.1 
        self.path_resolution = path_resolution      # maybe the step size, set to 0.05
        self.goal_sample_rate = goal_sample_rate    # set to 1
        self.max_iter = max_iter                    # number of iterations to run, set to 3000
        self.node_list = []                         # saves node information
    
    # path planning function
    def algorithm(self, animation=True):
        
        mf = 100                # magnifying factor
        pygame.init()
        screen = pygame.display.set_mode((10*mf, 10*mf))
        colour1 = (0,0,255)
        startCol = (255,0,0)
        goalCol = (0,255,0)
        counter = 0
        path = None

        while(True):
            screen.fill((25,25,25))
            obstacleCol = (255,255,255)
            pygame.draw.circle(screen, obstacleCol, (100*2.5, 100*2.5), 100*1)
            pygame.draw.circle(screen, obstacleCol, (100*7.5, 100*2.5), 100*1)
            pygame.draw.circle(screen, obstacleCol, (100*7.5, 100*7.5), 100*1)
            pygame.draw.circle(screen, obstacleCol, (100*2.5, 100*7.5), 100*1)
            pygame.draw.circle(screen, obstacleCol, (100*5, 100*5), 100*1)
            pygame.draw.circle(screen, (0,255,0), (mf*self.end.x, mf*10-(mf*self.end.y)), 14)
            pygame.draw.circle(screen, (0,0,255), (mf*self.start.x, mf*10-(mf*self.start.y)), 14)
        
            # Goal threshold
            pygame.draw.circle(screen, goalCol, (mf*self.end.x, mf*10-(mf*self.end.y)), 20)
        
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            if counter == 0:

                self.node_list = [self.start] # add start node to nodeList
                pygame.draw.circle(screen, startCol, (mf*self.start.x, mf*10-(mf*self.start.y)), 5)
            
                for i in range(self.max_iter):
                            
                    rnd_node = self.getRandom()                                    # get random node
                    nearest_ind = self.getNodeIndex(self.node_list, rnd_node)      # get nearest index
                    nearest_node = self.node_list[nearest_ind]                           # get node associated with nearest index
                    new_node = self.steer(nearest_node, rnd_node, self.expand_dis)       # get new node
                
                    # check if the edge enters obstacle space
                    if self.obstacleCheck(new_node):
                        self.node_list.append(new_node)   # record newNode if valid

                        # Show Vertices and Edges for each newNode
                        pygame.draw.circle(screen, (0,255,255), (mf*new_node.x, mf*10 - mf*new_node.y), 2)
                        pygame.draw.line(screen, (255,255,255), (mf*nearest_node.x, mf*10 - mf*nearest_node.y), (mf*new_node.x, mf*10 - mf*new_node.y))

                    # if new node is close enough to goal
                    if self.dist2goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                        print('New Node Reached!')
                        final_node = self.steer(self.node_list[-1], self.end, self.expand_dis) # to node is the end node here
                        if self.obstacleCheck(final_node): # check if final node is in obstacle space
                            path = self.getPath(len(self.node_list) - 1) # add path nodes to a list called Path
                    
                    pygame.display.update()
                    
                if path is None:
                    print("Cannot find path")
                else:
                    for point in path:
                        pygame.draw.circle(screen, (255,0,0), (mf*point[0], 10*mf-mf*point[1]), 5)
                    print("Path found using RRT Algorithm")
                    
                pygame.display.update()
            
            counter +=1
            # print(counter)
            # print(len(path))
            
    # function to steer from nearNode to randNode  --> get New Node
    def steer(self, fromN, toN, extend_length = float("inf") ):
            
        new_node = self.Node(fromN.x, fromN.y)
        d, theta = self.distAng(new_node, toN) # straight line, (distance and angle)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        extend_length = min(d, extend_length)
        
        n_expand = math.floor(extend_length / self.path_resolution) # round down

        for _ in range(n_expand):
            # update node values
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.distAng(new_node, toN)
        
        # if randNode is within step size of node
        if d <= self.path_resolution:  
            new_node.path_x.append(toN.x)
            new_node.path_y.append(toN.y)
            new_node.x = toN.x
            new_node.y = toN.y

        new_node.parent = fromN

        return(new_node)
    
    # backtrack from goal node to node unless node with no parent found
    def getPath(self, goalInd):
        backTrack = [[self.end.x, self.end.y]]
        node = self.node_list[goalInd]
        while node.parent is not None:
            backTrack.append([node.x, node.y])
            node = node.parent 
        backTrack.append([node.x, node.y])
        return(backTrack)   
    
    # gives eucilidean distance to goal
    def dist2goal(self, x, y):
        distance = math.hypot(x - self.end.x, y - self.end.y)
        return(distance)
            
    # gives a random node from map 
    def getRandom(self):
        if random.randint(0, 100) > self.goal_sample_rate: # random integer
            xRand, yRand = random.uniform(0, 10), random.uniform(0, 10) # random float numbers between 0 and 10
            randNode = self.Node(xRand, yRand)    # build random node
        else:  # goal point sampling
            randNode = self.Node(self.end.x, self.end.y)
        return(randNode)
     
    # gives distance and angle to next node, to build edge    
    @ staticmethod   
    def distAng(fromN, toN):
        # print(fromN)
        # print(toN)
        distance = math.hypot(toN.x - fromN.x, toN.y - fromN.y)
        angle = math.atan2(toN.y - fromN.y,toN.x - fromN.x)      
        return(distance, angle)
    
    @ staticmethod
    # get index of Nearest Node
    def getNodeIndex(nodeList, randNode):
        distanceList =  [(node.x - randNode.x)**2 + (node.y - randNode.y)**2
                         for node in nodeList]
        minIndex = distanceList.index(min(distanceList))
        return(minIndex)
    
    @ staticmethod
    def obstacleCheck(node):
        if node is None:
            return False
        
        x = node.x
        y = node.y
        
        if node.path_x:
            # Boundary condition
            if (x < 0) or (x > 10) or (y < 0) or (y > 10): 
                
                return False
            
            # Obstacle 1 (Circle Up)
            
            elif (x-2.5)**2 + (y-2.5)**2 - (1)**2 <= 0: 
                # print('Circle Up!')

                return False
            
              # Obstacle 4 (Circle Down)
            elif (x-2.5)**2 + (y-7.5)**2 - (1)**2 <= 0:
                # print('Circle Down!')
  
                return False
            
            elif (x-7.5)**2 + (y-2.5)**2 - (1)**2 <= 0:
                # print('Circle Down!')
  
                return False

            elif (x-7.5)**2 + (y-7.5)**2 - (1)**2 <= 0:
                # print('Circle Down!')
  
                return False

            elif (x-5)**2 + (y-5)**2 - (1)**2 <= 0:
                # print('Circle Down!')
  
                return False
            
            for (x, y) in zip(node.path_x, node.path_y):
                
                # Boundary condition
                if (x < 0) or (x > 10) or (y < 0) or (y > 10): 
                    return False
                
                elif (x-2.5)**2 + (y-2.5)**2 - (1)**2 <= 0: 
                # print('Circle Up!')

                    return False
            
              # Obstacle 4 (Circle Down)
                elif (x-2.5)**2 + (y-7.5)**2 - (1)**2 <= 0:
                    # print('Circle Down!')
    
                    return False
                
                elif (x-7.5)**2 + (y-2.5)**2 - (1)**2 <= 0:
                    # print('Circle Down!')
    
                    return False

                elif (x-7.5)**2 + (y-7.5)**2 - (1)**2 <= 0:
                    # print('Circle Down!')
    
                    return False

                elif (x-5)**2 + (y-5)**2 - (1)**2 <= 0:
                    # print('Circle Down!')
    
                    return False
                
                else:
                    # Node in Freespace
                    return True 
        
        return True       
# =============================================================
#                         Main
# =============================================================

def main():
    rrt = RRT(start = [1,1], goal = [9,9], rand_area=[0, 1000])
    rrt.algorithm(animation = True)

if __name__ == '__main__':
    main()  

    