#!usr/bin/env python
import numpy as np
from time import sleep
import copy

from queue import PriorityQueue
import heapq
import math

def ScalerDist(pos,target):
       
    x1 = pos[0]
    y1 = pos[1]
    x2 = target[0]
    y2 = target[1]
    disTo = (x2-x1,y2-y1)

    return np.sqrt(disTo[0]**2 + disTo[1]**2)

def AngleBetween(Rover,target):
    yaw = Rover.yaw
    forward = (np.cos(np.deg2rad(yaw)),np.sin(np.deg2rad(yaw)))
    position = Rover.pos
    
    x1 = position[0]
    y1 = position[1]
    x2 = target[0]
    y2 = target[1]
    disTo = (x2-x1,y2-y1)
    direcTo = disTo/np.sqrt(disTo[0]**2+disTo[1]**2)
    dp = np.dot(forward,direcTo)
    angle = np.arccos(dp)*180/np.pi
    
    SinofZfrom2VecCross = np.sin(np.cross((forward[0],forward[1],0),(direcTo[0],direcTo[1],0)))[2]
    if (SinofZfrom2VecCross >0):
        angle += 180
    
    return angle

def heading(position,target):
    x1 = position[0]
    y1 = position[1]
    x2 = target[0]
    y2 = target[1]
    disTo = (x2-x1,y2-y1)
    direcTo = disTo/np.sqrt(disTo[0]**2+disTo[1]**2)
    return direcTo
               
def steer(Rover):
    a = AngleBetween(Rover,(199,199))
   
    if a > 180:
        Rover.steer = np.clip(a, 0, 15)
    else: Rover.steer = np.clip(a, 0, 15)*-1
  
    

    return  Rover.worldmap[ int(round(posInfront[1])) , int(round(posInfront[0])),0 ] > 1       

def furthestNav(graph,goal):
    if isNav(goal,graph):
        return goal

    min = 300
    closestNavPix = (0,0)
    for i in range( graph.shape[0]):
        for j in range( graph.shape[1]):
            if graph[i,j][2]>graph[i,j][0]:
                d = ScalerDist((j,i),goal)
                if d < min:
                    min = d
                    closestNavPix = (j,i)
                    if min<10:
                        break
    

    return closestNavPix

def isNav(pos,graph):
   
    return graph[pos[1],pos[0]][2]>1

def GetCost(value,goal,start):
    d = ScalerDist(goal,value)
    c = ScalerDist(start,value)
    if d - c <0:
        cost = c
    else: 
        cost = d
   
    return d,(d+c)


class PixNode():
    def __init__(self,value , parent = 0 ,start = (0,0),goal = (0,0),parentcost = 0):
        
        self.children = []
        self.parent = parent 
        self.dist = 0
        self.cost = 0
        self.start = start
        self.parentcost = parentcost
        self.value  = value
        if parent:
            self.path = parent.path[:]
            self.path.append(self.value)
           
            self.goal = parent.goal
            
        else:
            self.path =[self.value]
           
            self.goal = goal
    def __lt__(self, other):
       return self.cost > other.cost

    def ___le__(self, other):
        return self.cost >= other.cost

    def __eq__(self, other):
        return self.cost == other

    def __ne__(self, other):
        return self.cost != other

    def __gt__(self, other):
        return self.cost < other.cost

    def __ge__(self, other):
        return self.cost <= other.cost
        
        self.dist,self.cost = GetCost(self.value,self.goal,self.value)

        
    def MakeChildren(self,graph):
             #for this application I wont be near
             # picture boundry so I wont need to  make bounds checks
     
        upleft = graph[self.value[0]-1,self.value[1]-1,:]
        upcent = graph[self.value[0]-1,self.value[1],:]
        upright = graph[self.value[0]-1,self.value[1]+1,:]
        left = graph[self.value[0],self.value[1]-1,:]
        right = graph[self.value[0],self.value[1]+1,:]
        downleft = graph[self.value[0]+1,self.value[1]-1,:]
        downcent = graph[self.value[0]+1,self.value[1],:]
        downright = graph[self.value[0]+1,self.value[1]+1,:]
        
        if upleft[2]:
            child1 = PixNode((self.value[0]-1,self.value[1]+1),self,self.start,self.goal,self.cost)
            #print(str(child1.value)+str(1))
            self.children.append(child1)
        if upcent[2]:
            child2 = PixNode((self.value[0],self.value[1]+1),self,self.start,self.goal,self.cost)
            #print(str(child2.value)+str(2))
            self.children.append(child2)
        if upright[2]:
            child3 = PixNode((self.value[0]+1,self.value[1]+1),self,self.start,self.goal,self.cost)
            #print(str(child3.value)+str(3))
            self.children.append(child3)
        if left[2]:
            child4 = PixNode((self.value[0]-1,self.value[1]),self,self.start,self.goal,self.cost)
            #print(str(child4.value)+str(4))
            self.children.append(child4)
        if right[2]:
            child5 = PixNode((self.value[0]+1,self.value[1]),self,self.start,self.goal,self.cost)
            #print(str(child5.value)+str(5))
            self.children.append(child5)
        if downleft[2]:
            child6 = PixNode((self.value[0]-1,self.value[1]-1),self,self.start,self.goal,self.cost)
            #print(str(child6.value)+str(6))
            self.children.append(child6)
        if downcent[2]:
            child7 = PixNode((self.value[0],self.value[1]-1),self,self.start,self.goal,self.cost)
            #print(str(child7.value)+ str(7))
            self.children.append(child7)
        if downright[2]:
            child8 = PixNode((self.value[0]+1,self.value[1]-1),self,self.start,self.goal,self.cost)
            #print(str(child8.value) + str(8))
            self.children.append(child8)
        
      
        
        return
            
def AStarSolve(startPos, goalPos, graph):
    
    print('new path')
    #localGoal = None
    #if not isNav(goalPos,graph):
    #    localGoal = furthestNav(graph[:],goalPos)
    #else : localGoal = goalPos
    localGoal = goalPos

    path = []     
    visitedQueue = []
    heap = PriorityQueue()
    startNode = PixNode(startPos,0,startPos,goalPos,0)
    heap.put(startNode)
    count = 0
    print(heap.qsize())
    while(not path and heap.qsize()):
       
        print(len(visitedQueue))
        closestChild = heap.get()
        
        closestChild.MakeChildren(graph[:])
        visitedQueue.append(closestChild.value)
        
       
        for kid in closestChild.children:
            if  kid.value not in visitedQueue:
               
                if kid.value == localGoal:
                    path = kid.path
                    break
                heap.put(kid)
   
    
    visitedQueue.clear()
    return path
    #path can return None


            









hacktimer = 0
stuck = False
stuckyaw = 0.0
stuckpos = (0,0)
# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
   
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    global hacktimer,stuck,stuckyaw,stuckpos
    
    if Rover.nav_angles is not None :
       # print(Rover.mode)
        
        # Check for Rover.mode status
        
        #graph = Rover.worldmap
        #print(graph[int(math.floor(Rover.pos[1])),int(math.ceil(Rover.pos[0])),:])

      
        if(Rover.firstRun):
            sleep(.3)
            Rover.firstRun = False
            
            Rover.toStack.push((141,104))
            Rover.toStack.push((128,110))
            Rover.toStack.push((115,99))
            Rover.fromStack.push((99,85))
     
      
        if Rover.mode == 'finished':
            
            Rover.steer = 5
            
           # Rover.mode = 'newheading'
        if Rover.mode == 'startup':
            if(Rover.vel >0):
                pass
               
        if Rover.mode == 'test':
            
           
            if Rover.rock_dists.any():
               rAng = np.mean(Rover.rock_angles)
               yawR = np.radians(Rover.yaw)
               rdist = np.min(Rover.rock_dists)*.1
               rovhead = (np.cos(yawR),np.sin(yawR))
               rockhead = (np.cos(rAng),np.sin(rAng))
              
               print((Rover.pos[0]+(rockhead[0]*rdist) ,Rover.pos[1]+(rockhead[1]*rdist))) 
                  # print(rdist) 
                
            
               
        if Rover.mode == 'newheading':
              
            #hacktimer = 0
            Rover.steer = 0 
            Rover.throttle = -5
            if Rover.vel <-1.5:
                Rover.mode = 'forward'
                stuck = False
            hacktimer+= 1
            if hacktimer >40:
                Rover.mode = 'forward'
                stuck = False
                hacktimer = 0
            
        if Rover.mode == 'seeking': 
            # Check the extent of navigable terrain
            if AngleBetween(Rover,Rover.navPath[0])>0 and AngleBetween(Rover,Rover.navPath[0])<270:
                Rover.mode = 'newheading'
               
            if ScalerDist(Rover.pos,Rover.navPath[0])>1 :  
                print( Rover.pos,Rover.navPath[0])
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = .1
                    Rover.brake = 0
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                steer(Rover)
                #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
                #print(Rover.navPath[0])
            if ScalerDist(Rover.pos,Rover.navPath[0])<1:
                    print('popped')
                    #Rover.throttle = 0
                    Rover.navPath.pop(0)
                    if(Rover.navPath == []):
                 
                       Rover.mode = 'stop'
                    
                    ##if Rover.toStack.size()>1:
                    ##   Rover.fromStack.push(Rover.toStack.pop(0))
                    #else: Rover.toStack.pop()
                    #if Rover.toStack.isEmpty():
                    #    Rover.mode = 'finished'
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
        elif Rover.mode == 'forward': 
                # Check the extent of navigable terrain
                
                #if np.max(Rover.rock_dists)<5:
                      #    Rover.mode = 'startup'
                    #Rover.targets.append(rock)
                    #Rover.rocksCollected.append((rock[0],rock[1]))
                    #print((rock[0],rock[1]))
                        
                if len(Rover.nav_angles) >= Rover.stop_forward and not stuck:  
                    
                    # If mode is forward, navigable terrain looks good 
                    # and velocity is below max, then throttle 
                    if Rover.vel < Rover.max_vel:
                        # Set throttle value to throttle setting
                        Rover.throttle = Rover.throttle_set
                       
                    
                    else: # Else coast
                        Rover.throttle = 0
                        hacktimer = 0   
                    if Rover.vel < .01 and Rover.vel > -.01:
                       
                      hacktimer+= 1
                      if hacktimer >20:
                        Rover.mode = 'newheading'
                        stuck = True
                        hacktimer = 0
                       
                    if(len(Rover.rock_angles)>1 and np.mean(Rover.rock_dists)<150):
                       Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
                       if np.mean(Rover.rock_dists)<35:
                           Rover.throttle = .1
                    else: Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    if Rover.rock_dists.any():
                        if  np.amin(Rover.rock_dists)<10:
                            Rover.mode = 'stop'
                    #if Rover.obs_dists.any():
                    #    if math.fabs(np.mean(Rover.obs_angles)- np.mean(Rover.nav_angles))<3:
                    #       Rover.steer = 5
                  
                 
                    Rover.brake = 0
                    # Set steering to average angle clipped to the range +/- 15
                   
                # If there's a lack of navigable terrain pixels then go to 'stop' mode
                elif len(Rover.nav_angles) < Rover.stop_forward:
                        # Set mode to "stop" and hit the brakes!
                        Rover.throttle = 0
                        # Set brake to stored brake value
                        Rover.brake = Rover.brake_set
                        Rover.steer = 0
                        Rover.mode = 'stop'
                
    
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
   
        

    if Rover.near_sample and Rover.vel < .1 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover