# Copyright 2012 paren8esis

# This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.

#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.

#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.

#!/usr/bin/python
# Filename: ai1.py

import sys
import random
import math

class Robot:

	def __init__(self, x, y, cellsNum):
		''' Class constructor. '''
		self.currPosX = x
		self.currPosY = y
		self.cellsNum = cellsNum

	def moveUp(self, numSq):
		''' The robot moves up numSq squares. '''
		self.currPosX -= numSq

	def moveDown(self, numSq):
		''' The robot moves down numSq squares. '''
		self.currPosX += numSq

	def moveRight(self, numSq):
		''' The robot moves right numSq squares. '''
		self.currPosY += numSq

	def moveLeft(self, numSq):
		''' The robot moves left numSq squares. '''
		self.currPosY -= numSq


class DumbRobot(Robot):

	def __init__(self, x2, y2, roomMap):
		''' Class constructor. '''
		Robot.__init__(self, x2, y2, 1)
		self.roomMap = roomMap
		self.visitedNodes = [(x2,y2)]

	def moveRandomly(self):
		''' The robot moves randomly in space. 
		It checks where it can move to and if it's been there before, and then randomly picks a move.
		This way it doesn't get stuck in corners or around the same spot.
		'''
		possibleMoves = []
		if (self.currPosX - 1 >= 0) and (roomMap[self.currPosX - 1][self.currPosY] != 1):
			if ((self.currPosX-1, self.currPosY) in self.visitedNodes):
				possibleMoves.append('self.moveUp(1)')
			else:
				for i in range(4):
					possibleMoves.append('self.moveUp(1)')
		if (self.currPosX + 1 <= len(roomMap)-1) and (roomMap[self.currPosX + 1][self.currPosY] != 1):
			if ((self.currPosX+1, self.currPosY) in self.visitedNodes):
				possibleMoves.append('self.moveDown(1)')
			else:
				for i in range(4):
					possibleMoves.append('self.moveDown(1)')
		if (self.currPosY + 1 <= len(roomMap[0])-1) and (roomMap[self.currPosX][self.currPosY + 1] != 1):
			if ((self.currPosX, self.currPosY+1) in self.visitedNodes):
				possibleMoves.append('self.moveRight(1)')
			else:
				for i in range(4):
					possibleMoves.append('self.moveRight(1)')
		if (self.currPosY - 1 >= 0) and (roomMap[self.currPosX][self.currPosY - 1] != 1):
			if ((self.currPosX, self.currPosY-1) in self.visitedNodes):
					possibleMoves.append('self.moveLeft(1)')
			else:
				for i in range(4):
						possibleMoves.append('self.moveLeft(1)')

		a = random.sample(possibleMoves, 1)
		exec(a[0])
		if ((self.currPosX, self.currPosY) not in self.visitedNodes):
			self.visitedNodes.append((self.currPosX, self.currPosY))
		
		print("Robot 2 moved to: ({0}, {1})".format(self.currPosX, self.currPosY))

class SmartRobot(Robot):

	def __init__(self, x1, y1, x2, y2, roomMap):
		''' Class constructor. '''
		Robot.__init__(self, x1, y1, 3)
		self.roomMap = roomMap

	def computeManhattanDist(self, curr, goal):
		''' Computes the manhattan distance from given node to goal. '''
		manhDst = abs(curr[0] - goal[0]) + abs(curr[1] - goal[1])
		return manhDst

	def computeEuclideanSquared(self, curr, goal):
		''' Computes the squared euclidean distance from given node to goal. '''
		euclDst = int(math.pow(curr[0] - goal[0], 2) + math.pow(curr[1] - goal[1], 2))
		return euclDst	

	def findPossibleMoves(self, x, y, s, goal):
		''' Finds all possible moves from current position and
		adds node to state space accordingly.
		Note: If node is already in state space, this method checks whether its g cost (cost from start) is
		higher than the new node's. If it is, it replaces it with the new node.
		'''
		possibleMoves = []
		if (x - 1 >= 0) and (roomMap[x - 1][y] != 1):
			manDst = self.computeManhattanDist((x-1, y), goal)
			if ((x-1, y) not in list(s.s.keys())) or (((x-1, y) in list(s.s.keys())) and (s.s[(x-1, y)][2] > s.s[(x,y)][1] + 1 + manDst)):
				s.addNode((x-1, y), (x, y))
				s.s[(x-1, y)][2] = 1 + manDst
				possibleMoves.append((x-1, y))
		if (x + 1 <= len(roomMap)-1) and (roomMap[x + 1][y] != 1):
			manDst = self.computeManhattanDist((x+1, y), goal)
			if ((x+1, y) not in list(s.s.keys())) or (((x+1, y) in list(s.s.keys())) and (s.s[(x+1, y)][2] > s.s[(x,y)][1] + 1 + manDst)):
				s.addNode((x+1, y), (x, y))
				s.s[(x+1, y)][2] = 1 + manDst
				possibleMoves.append((x+1, y))
		if (y + 1 <= len(roomMap[0])-1) and (roomMap[x][y + 1] != 1):
			manDst = self.computeManhattanDist((x, y+1), goal)
			if ((x, y+1) not in list(s.s.keys())) or (((x, y+1) in list(s.s.keys())) and (s.s[(x, y+1)][2] > s.s[(x,y)][1] + 1 + manDst)):
				s.addNode((x, y+1), (x, y))
				s.s[(x, y+1)][2] = 1 + manDst
				possibleMoves.append((x, y+1))
		if (y - 1 >= 0) and (roomMap[x][y - 1] != 1):
			manDst = self.computeManhattanDist((x, y-1), goal)
			if ((x, y-1) not in list(s.s.keys())) or (((x, y-1) in list(s.s.keys())) and (s.s[(x, y-1)][2] > s.s[(x,y)][1] + 1 + manDst)):
				s.addNode((x, y-1), (x, y))
				s.s[(x, y-1)][2] = 1 + manDst
				possibleMoves.append((x, y-1))

		return possibleMoves

	def addToOpenSet(self, openSet, newNodes, s):
		''' Adds the new nodes to openSet in sorted order, 
		according to the estimated cost to goal.
		'''
		for newNode in newNodes:
			added = False
			for node in openSet:
				if (s[node][2] > s[newNode][2]):
					openSet.insert(openSet.index(node), newNode)
					added = True
					break
			if (not added):
				openSet.append(newNode)

	def a_star(self, goalX, goalY):
		''' A* algorithm. 
		Computes next best move from current position to given goal.
		'''	
		goal = (goalX, goalY)
		stateSpace = StateSpace(self.currPosX, self.currPosY)

		# Expand from current position
		possibleMoves = self.findPossibleMoves(self.currPosX, self.currPosY, stateSpace, goal)

		closedSet = [(self.currPosX, self.currPosY)]   # List of nodes already evaluated
		openSet = [] 
		self.addToOpenSet(openSet, possibleMoves, stateSpace.s)   # List of nodes yet to be evaluated
		cameFrom = {}

		while (len(openSet) != 0):
			current = openSet.pop(0)
			if (current == goal):
				break

			closedSet.append(current)

			children = self.findPossibleMoves(current[0], current[1], stateSpace, goal)
			for child in children:
				tentative_g_score = stateSpace.s[current][1] + 1
				try:
					closedSet.index(child)
					if (tentative_g_score >= stateSpace.s[child][1]):
						continue
				except ValueError:
					pass

				inOpen = False
				try:
					openSet.index(child)
					inOpen = True
				except ValueError:
					pass

				if (not inOpen):
					cameFrom[child] = current
					stateSpace.s[child][1] = tentative_g_score
					stateSpace.s[child][2] = tentative_g_score + self.computeManhattanDist(child, goal)
					self.addToOpenSet(openSet, [child], stateSpace.s)
				else:
					if (tentative_g_score < stateSpace.s[child][1]):
						cameFrom[child] = current
						stateSpace.s[child][1] = tentative_g_score
						stateSpace.s[child][2] = tentative_g_score + self.computeManhattanDist(child, goal)

		while (current in list(cameFrom.keys())):
			current = cameFrom[current]
		
		self.currPosX = current[0]
		self.currPosY = current[1]

		print('Robot 1 moved to: {0}'.format(current))
	
class StateSpace:
	''' Class of state space. 
	s is the data structure that holds the tree.
	It is a dictionary with elements of the form:
	KEY : Node coordinates 
	VALUE : [ 0 -> parent,
	  1 -> cost from start
	  2 -> estimated cost to goal,
	  3 -> isLeaf,
	  4 -> list of children
	]
	'''

	def __init__(self, rootX, rootY):
		''' Class constructor. '''
		self.s = {}
		self.s[(rootX, rootY)] = [None, 0, 0, True, []]

	def addNode(self, coord, parent):
		''' Adds a node to the state space. '''
		if (coord not in list(self.s.keys())):
			self.s[parent][3] = False
			self.s[parent][4].append(coord)		

			# Compute cost from start
			g = 1
			nextPar = self.s[parent][0]
			while (nextPar != None):
				g += 1
				nextPar = self.s[nextPar][0]

			self.s[coord] = [parent, g, 0, True, []]

### Main ###

if (len(sys.argv) < 2):
	print("No file entered.")

try:
	f = open(sys.argv[1], 'r')   # Open for reading

	numline = 1	
	roomMap = []
	while True:
		line = f.readline()

		if (len(line) == 0):
		# Zero length indicates EOF
			break
		
		if (numline == 1):   # Space dimensions
			dim = line.split(' ')
			y = int(dim[0])
			x = int(dim[1])
		elif (numline == 2):   # Initial position
			initialPos = line.split(' ')
			x1 = int(initialPos[0]) - 1
			y1 = int(initialPos[1]) - 1
		elif (numline == 3):   # Final position
			finalPos = line.split(' ')
			x2 = int(finalPos[0]) - 1
			y2 = int(finalPos[1]) - 1
		else:   # Space details		
			oneLine = []
			for char in line:
				if (char == '\n'):
					break
				elif (char == 'X'):
					oneLine.append(1)
				else:
					oneLine.append(0)
			roomMap.append(oneLine)

		numline += 1
	f.close()

	robot1 = SmartRobot(x1, y1, x2, y2, roomMap)
	robot2 = DumbRobot(x2, y2, roomMap)

	print("Robot 1 initial pos: ({0},{1})".format(robot1.currPosX, robot1.currPosY))
	print("Robot 2 initial pos: ({0},{1})".format(robot2.currPosX, robot2.currPosY))
	print('\n')

	while (True):
		# Until the two robots meet
		robot2.moveRandomly()
		if ((robot1.currPosX == robot2.currPosX) and (robot1.currPosY == robot2.currPosY)):
			break
		robot1.a_star(robot2.currPosX, robot2.currPosY)
		if ((robot1.currPosX == robot2.currPosX) and (robot1.currPosY == robot2.currPosY)):
			break
		robot1.a_star(robot2.currPosX, robot2.currPosY)
		if ((robot1.currPosX == robot2.currPosX) and (robot1.currPosY == robot2.currPosY)):
			break
		robot1.a_star(robot2.currPosX, robot2.currPosY)
		if ((robot1.currPosX == robot2.currPosX) and (robot1.currPosY == robot2.currPosY)):
			break

except IOError as ioe:
	print(ioe)	
