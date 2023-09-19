'''
*****************************************************************************************
*
*        		     ===============================================
*           		       Pharma Bot (PB) Theme (eYRC 2022-23)
*        		     ===============================================
*
*  This script contains all the past implemented functions of Pharma Bot (PB) Theme 
*  (eYRC 2022-23).
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			PB_theme_functions.py
# Functions:		
# 					[ Comma separated list of functions in this file ]

####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the three available ##
## modules for this task (numpy, opencv)                    ##
##############################################################
import socket
import time
import os, sys
from zmqRemoteApi import RemoteAPIClient
import traceback
import zmq
import numpy as np
import cv2
from cv2 import aruco
from pyzbar.pyzbar import decode
import json
import math
import heapq
##############################################################

################# ADD UTILITY FUNCTIONS HERE #################
def signal_coordintes(signal):
    coordinates = []
    c_z = 0.15588

    if signal[0] <= "C":
        temp = ord("C") - ord(signal[0]) 
        c_x = -0.178 - (temp)*0.356
    else:
        temp = ord(signal[0]) - ord("D")
        c_x = 0.178 + (temp)*0.356

    if int(signal[1]) <= 3:
        c_y = 0.178 + (3-int(signal[1]))*0.356
    else:
        c_y = -0.178 - (int(signal[1])-4)*0.356
    
    coordinates.append(c_x)
    coordinates.append(c_y)
    coordinates.append(c_z)

    return coordinates

def horiz_baricade_coordinates(barricade):
    coordinates = []

    if barricade[0] < "C":
        temp = ord("C") - ord(barricade[0]) 
        c_x = -(temp)*0.356
    else:
        temp = ord(barricade[0]) - ord("D")
        c_x = (temp+1)*0.356

    if int(barricade[1]) <= 3:
        c_y = 0.178 + (3-int(barricade[1]))*0.356
    else:
        c_y = -0.178 - (int(barricade[1])-4)*0.356

    coordinates.append(c_x)
    coordinates.append(c_y)

    return coordinates

def vert_baricade_coordinates(barricade):
    coordinates = []

    if barricade[0] <= "C":
        temp = ord("C") - ord(barricade[0]) 
        c_x = -0.178 - (temp)*0.356
    else:
        temp = ord(barricade[0]) - ord("D")
        c_x = 0.178 + (temp)*0.356

    if int(barricade[1]) <= 3:
        c_y = (3-int(barricade[1]))*0.356
    else:
        c_y = -(int(barricade[1])-4+1)*0.356

    coordinates.append(c_x)
    coordinates.append(c_y)

    return coordinates

def moves_transform(list):
	ans = []

	for i in list:
		if i == "STRAIGHT":
			ans.append("STRAIGHT")

		elif i == "LEFT":
			ans.append("LEFT")
			ans.append("STRAIGHT")

		elif i == "RIGHT":
			ans.append("RIGHT")
			ans.append("STRAIGHT")

		elif i == "TURN_180":
			ans.append("LEFT")
			ans.append("LEFT")
			ans.append("STRAIGHT")
		else:
			ans.append(i)
	
	return ans

def orientation(instructions, initial_orientation):
	# Define a dictionary to map orientation indices to their respective (x,y) directions
	orientations = {0: (0, 1), 1: (1, 0), 2: (0, -1), 3: (-1, 0)}

	# Initialize the current orientation and position of the robot
	current_orientation = initial_orientation
	current_position = [0, 0]

	moves_list = instructions.copy()

	if "TURN_180" in moves_list:
		moves_list[0] = "LEFT"
		moves_list.insert(0, "LEFT")
	else:
		None
    
	# Loop through the instructions and update the position and orientation accordingly
	for instruction in moves_list:
		if instruction == 'STRAIGHT':
			direction = orientations[current_orientation]
			current_position[0] += direction[0]
			current_position[1] += direction[1]
		elif instruction == 'LEFT':
			current_orientation = (current_orientation - 1) % 4
		elif instruction == 'RIGHT':
			current_orientation = (current_orientation + 1) % 4

	# Return the final orientation of the robot
	return current_orientation



"""Adding functions written in Task 3a Here"""
############## FUNCTIONS HERE  ###############################
def estimated_cost(node, goalNode):
	cost = 0

	n = ord(node[0]) - 64
	g = ord(goalNode[0]) - 64

	cost =  abs(int(goalNode[1]) - int(node[1])) + abs((g - n))
	return cost

def lowest_cost(list, goalNode):
	index = 0
	lowestCost = estimated_cost(list[0], goalNode)
	for i in range(len(list)):
		currentCost = estimated_cost(list[i], goalNode)
		if currentCost < lowestCost:
			lowestCost = currentCost
			index = i

	return list[index]			

def allInClose(list, close):
	ans = 1
	for i in list:
		if i in close:
			ans *= 1
		else:
			ans*= 0
	
	if ans == 1:
		return True
	else:
		return False

def direction(node1, node2, node3):
	
	if node1[0] == node2[0] == node3[0]:
		return "STRAIGHT"

	elif node1[0]==node2[0]:
		if node3[0]>node1[0]:
			if node2[1]<node1[1]:
				return 'RIGHT'
			else:
				return 'LEFT'
		else:
			if node2[1]<node1[1]:
				return 'LEFT'
			else:
				return 'RIGHT'
	elif node1[1] == node2[1] == node3[1]:
		return "STRAIGHT"
	else:
		if node3[1]>node1[1]:
			if node2[0]>node1[0]:
				return 'RIGHT'
			else:
				return 'LEFT'
		else:
			if node2[0]>node1[0]:
				return 'LEFT'
			else:
				return 'RIGHT'

def path_planning(graph, start, goal):
    """
    A* algorithm for path planning

    Args:
        graph (dict): Dictionary representing the graph/map
        start (str): Starting node in the format of 'A1', 'B2', etc.
        goal (str): Goal node in the format of 'A1', 'B2', etc.

    Returns:
        path (list): List of nodes representing the shortest path from start to goal
    """
    # Heuristic function (Euclidean distance)
    def heuristic(a, b):
        (x1, y1) = node_to_xy(a)
        (x2, y2) = node_to_xy(b)
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    # Convert node string to x, y coordinates
    def node_to_xy(node):
        x = ord(node[0]) - 65
        y = int(node[1:]) - 1
        return (x, y)

    # Convert x, y coordinates to node string
    def xy_to_node(x, y):
        node = chr(x + 65) + str(y + 1)
        return node

    # Initialize variables
    frontier = []
    visited = {}
    came_from = {}
    g_score = {node: float('inf') for node in graph.keys()}
    f_score = {node: float('inf') for node in graph.keys()}

    # Start node
    start_node = start
    g_score[start_node] = 0
    f_score[start_node] = heuristic(start_node, goal)
    heapq.heappush(frontier, (f_score[start_node], start_node))

    # A* algorithm
    while frontier:
        current_node = heapq.heappop(frontier)[1]

        if current_node == goal:
            # Reconstruct path
            path = []
            node = goal
            while node != start:
                path.append(node)
                node = came_from[node]
            path.append(start)
            path.reverse()
            return path

        visited[current_node] = True

        for neighbor, cost in graph[current_node].items():
            if neighbor in visited:
                continue

            tentative_g_score = g_score[current_node] + cost

            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current_node
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                if neighbor not in [node[1] for node in frontier]:
                    heapq.heappush(frontier, (f_score[neighbor], neighbor))

    return None

def a_star(graph, start, end):

	"""
	Purpose:
	---
	This function takes the graph(dict), start and end node for planning the shortest path

	** Note: You can use any path planning algorithm for this but need to produce the path in the form of 
	list given below **

	Input Arguments:
	---
	`graph` :	{ dictionary }
			dict of all connecting path
	`start` :	str
			name of start node
	`end` :		str
			name of end node


	Returns:
	---
	`backtrace_path` : [ list of nodes ]
			list of nodes, produced using path planning algorithm

		eg.: ['C6', 'C5', 'B5', 'B4', 'B3']
	
	Example call:
	---
	arena_parameters = detect_arena_parameters(maze_image)
	"""    

	backtrace_path=[]

	##############	ADD YOUR CODE HERE	##############
	# backtrace_path.append(start)
	tempList = []
	openNode = []
	closedNode = []
	openNode.append(start)

	while True:
		current = lowest_cost(openNode, end)
		tempList.append(current)
		# print(current)
		openNode.remove(current)
		closedNode.append(current)
		if current == end:
			break
		neighbours =graph.get(current)
		keysNeighbour = list(neighbours.keys())
		for key in keysNeighbour:
			if key in closedNode  or key in openNode:
				continue
			else:
				openNode.append(key)
		# print(openNode)

		for i in openNode:
			if i == end:
				continue
			if (len(list(graph.get(i).keys())) == 0) or (i in closedNode) or allInClose(list(graph.get(i).keys()), closedNode):
				openNode.remove(i)
				closedNode.append(i)
				continue
	li = tempList
	index=[0,1]
	for i in range(len(li)):
		a=li[i]
		list1=list(graph.get(str(a)).keys())
		# print(list1)
		for j in range(i+2,len(li)):
			if li[j] in list1 and i+2!=j:
				index=[i,j]
				# print(index)
	li=li[0:index[0]+1]+li[index[1]:]
	return li	

def paths_to_moves(paths, traffic_signal, orientation):

	"""
	Purpose:
	---
	This function takes the list of all nodes produces from the path planning algorithm
	and connecting both start and end nodes

	Input Arguments:
	---
	`paths` :	[ list of all nodes ]
			list of all nodes connecting both start and end nodes (SHORTEST PATH)
	`traffic_signal` : [ list of all traffic signals ]
			list of all traffic signals
	---
	`moves` : [ list of moves from start to end nodes ]
			list containing moves for the bot to move from start to end

			Eg. : ['UP', 'LEFT', 'UP', 'UP', 'RIGHT', 'DOWN']
	
	Example call:
	---
	moves = paths_to_moves(paths, traffic_signal)
	"""    
	
	list_moves=[]

	##############	ADD YOUR CODE HERE	##############
	# print("Paths: ", paths)

	for i in range(len(paths)-1):
		if i == 0:
			if paths[i][1] != paths[(i+1)][1]: ## Both node are vertical
				if orientation == 0:
					if paths[i+1][1] < paths[i][1]:
						list_moves.append("STRAIGHT")
					else:
						list_moves.append("TURN_180")  
				elif orientation == 1:
					if paths[i+1][1] < paths[i][1]:
						list_moves.append("LEFT")
					else:
						list_moves.append("RIGHT")
				elif orientation == 2:
					if paths[i+1][1] < paths[i][1]:
						list_moves.append("TURN_180")
					else:
						list_moves.append("STRAIGHT")
				elif orientation == 3:
					if paths[i+1][1] < paths[i][1]:
						list_moves.append("RIGHT")
					else:
						list_moves.append("LEFT")

			else: ## Both node are horizontal to each other
				if orientation == 0:
					if paths[i+1][0] < paths[i][0]:
						list_moves.append("LEFT")
					else:
						list_moves.append("RIGHT")  
				elif orientation == 1:
					if paths[i+1][0] < paths[i][0]:
						list_moves.append("TURN_180")
					else:
						list_moves.append("STRIAGHT")
				elif orientation == 2:
					if paths[i+1][0] < paths[i][0]:
						list_moves.append("RIGHT")
					else:
						list_moves.append("LEFT")
				elif orientation == 3:
					if paths[i+1][0] < paths[i][0]:
						list_moves.append("STRAIGHT")
					else:
						list_moves.append("TURN_180")

			# print("list_moves: ", list_moves)
		elif paths[i] in traffic_signal:
			list_moves.append("WAIT_5")
			dir = direction(paths[i], paths[i+1], paths[i-1])
			list_moves.append(dir)
		else:
			dir = direction(paths[i], paths[i+1], paths[i-1])
			# print(dir)
			list_moves.append(dir)

	# #checking for orientation of robot
	# if orientation == 1:
	# 	list_moves = list_moves
	# elif orientation == 
	##################################################
	
	return list_moves

##############################################################


################## ADD SOCKET COMMUNICATION ##################
####################### FUNCTIONS HERE #######################
"""
Add functions written in Task 3D for setting up a Socket
Communication Server in this section
"""

def setup_server(host, port):

	"""
	Purpose:
	---
	This function creates a new socket server and then binds it 
	to a host and port specified by user.

	Input Arguments:
	---
	`host` :	[ string ]
			host name or ip address for the server

	`port` : [ string ]
			integer value specifying port name
	Returns:

	`server` : [ socket object ]
	---

	
	Example call:
	---
	server = setupServer(host, port)
	""" 

	server = None

	server = socket.socket()
	server.bind((socket.gethostbyname(host), port))

	return server

def setup_connection(server):
	"""
	Purpose:
	---
	This function listens for an incoming socket client and
	accepts the connection request

	Input Arguments:
	---
	`server` :	[ socket object ]
			socket object created by setupServer() function
	Returns:
	---
	`server` : [ socket object ]
	
	Example call:
	---
	connection = setupConnection(server)
	"""
	connection = None
	address = None

	server.listen()
	connection, address = server.accept()


	return connection, address


def listen_client(connection, q):
    """Function to listen to client"""
    while True:
        try:
            # receive data from client
            data = connection.recv(1024).decode()
            # put the data in the queue
            q.put(data)
        except socket.error as e:
            # handle socket error
            # ...
            break
    
    
def receive_message_via_socket(connection):
	"""
	Purpose:
	---
	This function listens for a message from the specified
	socket connection and returns the message when received.

	Input Arguments:
	---
	`connection` :	[ connection object ]
			connection object created by setupConnection() function
	Returns:
	---
	`message` : [ string ]
			message received through socket communication
	
	Example call:
	---
	message = receive_message_via_socket(connection)
	"""

	message = None

	message = connection.recv(1024).decode()

	return message

def send_message_via_socket(connection, message):
	"""
	Purpose:
	---
	This function sends a message over the specified socket connection

	Input Arguments:
	---
	`connection` :	[ connection object ]
			connection object created by setupConnection() function

	`message` : [ string ]
			message sent through socket communication

	Returns:
	---
	None
	
	Example call:
	---
	send_message_via_socket(connection, message)
	"""
	if type(message) == str:
		connection.send(bytes(message, 'utf-8'))
	elif type(message) == list:
		# print("Message before: ", message)
		message = moves_transform(message)
		# print("Message sent: ", message)
		message = "\x00".join(message).encode('utf-8')
		connection.send(message)

##############################################################
##############################################################

######################### ADD TASK 2B ########################
####################### FUNCTIONS HERE #######################
"""
Add functions written in Task 2B for reading QR code from
CoppeliaSim arena in this section
"""

def read_qr_code(sim):
	"""
	Purpose:
	---
	This function detects the QR code present in the CoppeliaSim vision sensor's 
	field of view and returns the message encoded into it.

	Input Arguments:
	---
	`sim`    :   [ object ]
		ZeroMQ RemoteAPI object

	Returns:
	---
	`qr_message`   :    [ string ]
		QR message retrieved from reading QR code

	Example call:
	---
	control_logic(sim)
	"""
	qr_message = None

	image = cv2.imwrite(sim)
	img=cv2.imread(image)
	qr=decode(img)
	qr_message = qr.decode('utf-8')

	return qr_message

##############################################################
##############################################################

############### ADD ARENA PARAMETER DETECTION ################
####################### FUNCTIONS HERE #######################
"""
Add functions written in Task 1A and 3A for detecting arena parameters
from configuration image in this section
"""

def detect_all_nodes(image):
	"""
	Purpose:
	---
	This function takes the image as an argument and returns a list of
	nodes in which traffic signals, start_node and end_node are present in the image

	Input Arguments:
	---
	`maze_image` :	[ numpy array ]
			numpy array of image returned by cv2 library
	Returns:
	---
	`traffic_signals, start_node, end_node` : [ list ], str, str
			list containing nodes in which traffic signals are present, start and end node too
	
	Example call:
	---
	traffic_signals, start_node, end_node = detect_all_nodes(maze_image)
	"""    
	traffic_signals = []
	start_node = ""
	end_node = ""
	temp1 = '94'
	temp2 = '94'
	alp = ['A', 'B', 'C', 'D', 'E', 'F']
	for i in range(6):
		for j in range(6):
			temp3 = int(str(i)+temp1)
			temp4 = int(str(j)+temp2)
			(b, g, r) = image[temp3, temp4]
			if r == 255:
				traffic_signals.append(alp[j] + str(i+1))	
			if g==255 and b==0 and r==0:
				start_node+=(alp[j]+str(i+1))
			if b==189 and r==105 and g==43 :
				end_node+=(alp[j]+str(i+1))
				
	traffic_signals.sort()
	##################################################

	return traffic_signals, start_node, end_node


def detect_horizontal_roads_under_construction(maze_image):
	
	"""
	Purpose:
	---
	This function takes the image as an argument and returns a list
	containing the missing horizontal links

	Input Arguments:
	---
	`maze_image` :	[ numpy array ]
			numpy array of image returned by cv2 library
	Returns:
	---
	`horizontal_roads_under_construction` : [ list ]
			list containing missing horizontal links
	
	Example call:
	---
	horizontal_roads_under_construction = detect_horizontal_roads_under_construction(maze_image)
	"""    
	horizontal_roads_under_construction = []

	##############	ADD YOUR CODE HERE	##############
	temp1 = 144
	temp2 = 94
	alp = ['A', 'B', 'C', 'D', 'E', 'F']

	for i in range(6):
		temp3=i*100+temp2
		for j in range(5):
			temp4=j*100+temp1
			(b, g, r) = maze_image[temp3, temp4]
			if b==255 and g==255 and r==255:
				horizontal_roads_under_construction.append(alp[int(str(temp4)[0])-1]+str(int(str(temp3)[0])+1)+'-'+alp[int(str(temp4)[0])]+str(int(str(temp3)[0])+1))
	
	horizontal_roads_under_construction.sort()
	##################################################
	
	return horizontal_roads_under_construction

def detect_vertical_roads_under_construction(maze_image):

	"""
	Purpose:
	---
	This function takes the image as an argument and returns a list
	containing the missing vertical links

	Input Arguments:
	---
	`maze_image` :	[ numpy array ]
			numpy array of image returned by cv2 library
	Returns:
	---
	`vertical_roads_under_construction` : [ list ]
			list containing missing vertical links
	
	Example call:
	---
	vertical_roads_under_construction = detect_vertical_roads_under_construction(maze_image)
	"""    
	vertical_roads_under_construction = []

	##############	ADD YOUR CODE HERE	##############
	temp1 = 145
	temp2 = 100
	alp = ['A', 'B', 'C', 'D', 'E', 'F']

	for i in range(5):
		for j in range(6):
			temp3 = i*100 + temp1 -1
			temp4 = j*100 + temp2 -1
			# print(type(maze_image))
			# print(maze_image[temp3, temp4])
			(b, g, r) = maze_image[temp3, temp4]
			if b==255 and g==255 and r==255:
				# print(temp4, temp3)
				# print(int(str(temp4+1)[0])-1)
				vertical_roads_under_construction.append(alp[int(str(temp4+1)[0])-1]+str(int(str(temp3)[0]))+'-'+alp[int(str(temp4+1)[0])-1]+str(int(str(temp3)[0])+1))
	
	vertical_roads_under_construction.sort()
	##################################################
	
	return vertical_roads_under_construction

def detect_medicine_packages(maze_image):

	"""
	Purpose:
	---
	This function takes the image as an argument and returns a nested list of
	details of the medicine packages placed in different shops

	** Please note that the shop packages should be sorted in the ASCENDING order of shop numbers 
	   as well as in the alphabetical order of colors.
	   For example, the list should first have the packages of shop_1 listed. 
	   For the shop_1 packages, the packages should be sorted in the alphabetical order of color ie Green, Orange, Pink and Skyblue.

	Input Arguments:
	---
	`maze_image` :	[ numpy array ]
			numpy array of image returned by cv2 library
	Returns:
	---
	`medicine_packages` : [ list ]
			nested list containing details of the medicine packages present.
			Each element of this list will contain 
			- Shop number as Shop_n
			- Color of the package as a string
			- Shape of the package as a string
			- Centroid co-ordinates of the package
	Example call:
	---
	medicine_packages = detect_medicine_packages(maze_image)
	"""    
	medicine_packages = []

	##############	ADD YOUR CODE HERE	##############
	temp1 = 106
	temp2 = 194
	temp3 = 130
	temp4 = 130
	for i in range(6):
		temp5 = temp3 + i*100
		temp6 = temp4
		
		# Shape detection or Object detection starts here

		temp_img = maze_image[temp1:temp2, temp1+i*100:temp2+i*100]
		imgGry = cv2.cvtColor(temp_img, cv2.COLOR_BGR2GRAY)

		ret , thrash = cv2.threshold(imgGry, 240 , 255, cv2.CHAIN_APPROX_NONE)
		contours , hierarchy = cv2.findContours(thrash, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

		shape_list = []
		for contour in contours:
			approx = cv2.approxPolyDP(contour, 0.01* cv2.arcLength(contour, True), True)
			x = approx.ravel()[0]
			y = approx.ravel()[1] - 5
			if len(approx) == 3:
				text = 'Triangle'
				shape_list.append(text)
			elif len(approx) == 4 :
				x, y , w, h = cv2.boundingRect(approx)
				aspectRatio = float(w)/h
				if aspectRatio >= 0.95 and aspectRatio < 1.05 :
					text = 'Square'
					shape_list.append(text)
				else:
					text = 'Rectangle'
					shape_list.append(text)
			elif len(approx) == 5 :
				text = 'Pentagon'
				shape_list.append(text)
			elif len(approx) == 10 :
				text = 'Star'
				shape_list.append(text)
			else:
				text = 'Circle'
				shape_list.append(text)

			# Shape detection or Object Detection ends here

		for j in range(2):
			for k in range(2):
				temp_list = []
				(b, g, r) = maze_image[temp6+j*40, temp5+k*40]
				if (b, g, r) == (255, 255, 255):
					None
				elif (b, g, r) == (0, 255, 0):
					shop = 'Shop_'+ str(i+1)
					color = 'Green'
					shape = shape_list[0]
					if shape == "Triangle":
						coordinates = [temp5+k*40, temp6+j*40-1]
					else: 
						coordinates = [temp5+k*40, temp6+j*40]
					temp_list.extend((shop, color, shape, coordinates))  
					medicine_packages.append(temp_list) 
				elif (b, g, r) == (180, 0, 255):
					shop = 'Shop_'+ str(i+1)
					color = 'Pink'
					shape = shape_list[0]
					if shape == "Triangle":
						coordinates = [temp5+k*40, temp6+j*40-1]
					else: 
						coordinates = [temp5+k*40, temp6+j*40]
					temp_list.extend((shop, color, shape, coordinates))
					medicine_packages.append(temp_list)
				elif (b, g, r) == (255, 255, 0):
					shop = 'Shop_'+ str(i+1)
					color = 'Skyblue'
					shape = shape_list[0]
					if shape == "Triangle":
						coordinates = [temp5+k*40, temp6+j*40-1]
					else: 
						coordinates = [temp5+k*40, temp6+j*40]
					temp_list.extend((shop, color, shape, coordinates))
					medicine_packages.append(temp_list)
				elif (b, g, r) == (0, 127, 255):
					shop = 'Shop_'+ str(i+1)
					color = 'Orange'
					shape = shape_list[0]
					if shape == "Triangle":
						coordinates = [temp5+k*40, temp6+j*40-1]
					else: 
						coordinates = [temp5+k*40, temp6+j*40]
					temp_list.extend((shop, color, shape, coordinates))
					medicine_packages.append(temp_list)

	medicine_packages.sort()
	##################################################

	return medicine_packages

def detect_all_nodes(image):
	"""
	Purpose:
	---
	This function takes the image as an argument and returns a list of
	nodes in which traffic signals, start_node and end_node are present in the image

	Input Arguments:
	---
	`maze_image` :	[ numpy array ]
			numpy array of image returned by cv2 library
	Returns:
	---
	`traffic_signals, start_node, end_node` : [ list ], str, str
			list containing nodes in which traffic signals are present, start and end node too
	
	Example call:
	---
	traffic_signals, start_node, end_node = detect_all_nodes(maze_image)
	"""    
	traffic_signals = []
	start_node = ""
	end_node = ""
	temp1 = '94'
	temp2 = '94'
	alp = ['A', 'B', 'C', 'D', 'E', 'F']
	for i in range(6):
		for j in range(6):
			temp3 = int(str(i)+temp1)
			temp4 = int(str(j)+temp2)
			(b, g, r) = image[temp3, temp4]
			if r == 255:
				traffic_signals.append(alp[j] + str(i+1))	
			if g==255 and b==0 and r==0:
				start_node+=(alp[j]+str(i+1))
			if b==189 and r==105 and g==43 :
				end_node+=(alp[j]+str(i+1))
				
	traffic_signals.sort()
	##################################################

	return traffic_signals, start_node, end_node

def detect_traffic_signals(maze_image):

	"""
	Purpose:
	---
	This function takes the image as an argument and returns a list of
	nodes in which traffic signals are present in the image

	Input Arguments:
	---
	`maze_image` :	[ numpy array ]
			numpy array of image returned by cv2 library
	Returns:
	---
	`traffic_signals` : [ list ]
			list containing nodes in which traffic signals are present
	
	Example call:
	---
	traffic_signals = detect_traffic_signals(maze_image)
	"""    
	traffic_signals = []

	##############	ADD YOUR CODE HERE	##############
	# maze_image = cv2.imread(maze_image)
	# print(maze_image)
	temp1 = '94'
	temp2 = '94'
	alp = ['A', 'B', 'C', 'D', 'E', 'F']

	for i in range(6):
		for j in range(6):
			temp3 = int(str(i)+temp1)
			temp4 = int(str(j)+temp2)
			(b, g, r) = maze_image[temp3, temp4]
			if r == 255:
				if i+1 == 7:
					pass
				else:
					traffic_signals.append(alp[j] + str(i+1))	
	
	traffic_signals.sort()
	#######################################_###########
	
	return traffic_signals

def detect_paths_to_graph(image):
    """
	Purpose:
	---
	This function takes the image as an argument and returns a dictionary of the
	connect path from a node to other nodes and will be used for path planning

	HINT: Check for the road besides the nodes for connectivity 

	Input Arguments:
	---
	`maze_image` :	[ numpy array ]
			numpy array of image returned by cv2 library
	Returns:
	---
	`paths` : { dictionary }
			Every node's connection to other node and set it's value as edge value 
			Eg. : { "D3":{"C3":1, "E3":1, "D2":1, "D4":1}, 
					"D5":{"C5":1, "D2":1, "D6":1 }  }

			Why edge value 1? -->> since every road is equal

	Example call:
	---
	paths = detect_paths_to_graph(maze_image)
	"""    

    
	##############	ADD YOUR CODE HERE	##############
    sub_path={}
    temp1 = '00'
    temp2 = '00'
    alp = ['','A', 'B', 'C', 'D', 'E', 'F']
    paths={}
    for i in range(1,7):
        for j in range(1,7):
            temp3 = int(str(i)+temp1)
            temp4 = int(str(j)+temp2)
            (b,g,r)= image[temp3,temp4+50]
            #print(b,g,r)
            if b==0 and r==0 and g==0 and alp[j]!='F':
                sub_path[alp[j+1]+str(i)]=1
    
            (b,g,r)=image[temp3+50,temp4]
            #print(b,g,r)
            if b==0 and r==0 and g==0 and i!=6:
                sub_path[alp[j]+str(i+1)]=1

            (b,g,r)= image[temp3,temp4-50]
            #print(b,g,r)
            if b==0 and r==0 and g==0 and alp[j]!='A':
                sub_path[alp[j-1]+str(i)]=1
            (b,g,r)=image[temp3-50,temp4]
            #print(b,g,r)
            if b==0 and r==0 and g==0 and i!=1:
                sub_path[alp[j]+str(i-1)]=1
            aks = (alp[j]+str(i))
            #print(aks)
            #sub_path.sort()
            #print(sub_path)
            paths[aks]=sub_path
            sub_path={}
	##################################################
    return paths

def detect_arena_parameters(maze_image):

	"""
	Purpose:
	---
	This function takes the image as an argument and returns a dictionary
	containing the details of the different arena parameters in that image

	The arena parameters are of four categories:
	i) traffic_signals : list of nodes having a traffic signal
	ii) horizontal_roads_under_construction : list of missing horizontal links
	iii) vertical_roads_under_construction : list of missing vertical links
	iv) medicine_packages : list containing details of medicine packages

	These four categories constitute the four keys of the dictionary

	Input Arguments:
	---
	`maze_image` :	[ numpy array ]
			numpy array of image returned by cv2 library
	Returns:
	---
	`arena_parameters` : { dictionary }
			dictionary containing details of the arena parameters
	
	Example call:
	---
	arena_parameters = detect_arena_parameters(maze_image)
	"""    
	arena_parameters = {}

	##############	ADD YOUR CODE HERE	##############
	traffic_signals, start_node, end_node = detect_all_nodes(maze_image)
	paths = detect_paths_to_graph(maze_image)
	arena_parameters = {'traffic_signals': detect_traffic_signals(maze_image), 'horizontal_roads_under_construction': detect_horizontal_roads_under_construction(maze_image), 'vertical_roads_under_construction': detect_vertical_roads_under_construction(maze_image), 'medicine_packages': detect_medicine_packages(maze_image), 'start_node': start_node, 'end_node':end_node, "paths": paths}
	
	##################################################
	
	return arena_parameters

##############################################################
##############################################################
################### SHOWING ROBOT ON COPPELIASIM SCENE########
"""
Adding functions written in Task 3C for emulating the robot in
coppeliasim scene
"""
def detect_ArUco_details(image):

    """
    Purpose:
    ---
    This function takes the image as an argument and returns a dictionary such
    that the id of the ArUco marker is the key and a list of details of the marker
    is the value for each item in the dictionary. The list of details include the following
    parameters as the items in the given order
        [center co-ordinates, angle from the vertical, list of corner co-ordinates] 
    This order should be strictly maintained in the output

    Input Arguments:
    ---
    `image` :	[ numpy array ]
            numpy array of image returned by cv2 library
    Returns:
    ---
    `ArUco_details_dict` : { dictionary }
            dictionary containing the details regarding the ArUco marker
    
    Example call:
    ---
    ArUco_details_dict = detect_ArUco_details(image)
    """    
    ArUco_details_dict = {} #should be sorted in ascending order of ids
    ArUco_corners = {}
    
    ##############	ADD YOUR CODE HERE	##############

    centres=[0,0]
    li=[0,0]
    dic={}
    # print("Aruco detect function called")

    ARUCO_PARAMETERS = aruco.DetectorParameters_create()
    ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_250) 
    image=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPonts = aruco.detectMarkers(image, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
    # print(corners)
    # print(ids)
    for i in range (len(ids)):
        for j in range (2):
            centres[j]=((corners[i][0,0,j]+corners[i][0,2,j]) + (corners[i][0,1,j]+corners[i][0,3,j]))//4

        m=(corners[i][0][0][1]-corners[i][0][3][1])/(corners[i][0][0][0]-corners[i][0][3][0])
        a=corners[i][0][0][0]
        b=corners[i][0][3][0]
        c=corners[i][0][0][1]
        d=corners[i][0][3][1]


        if a>=b and c>=d:
            angle=-90-math.degrees(math.atan(m))
        
        if a<=b and c<=d:
            angle=90-math.degrees(math.atan(m))

        if a<=b and c>=d:
            angle=90-math.degrees(math.atan(m))

        if a>=b and c<=d:
            angle=-90-math.degrees(math.atan(m))


        #for k in range (2):
        centres = list(map(int, centres))
        li[0]=centres[:]
        li[1]=int(angle)
            
        dic[int(ids[i][0])]=li[:]
 
        ArUco_corners[int(ids[i][0])]=corners[i][0].tolist()
        ArUco_details_dict=dic
   
    ##################################################
    
    return ArUco_details_dict, ArUco_corners 

def perspective_transform(image):

    """
    Purpose:
    ---
    This function takes the image as an argument and returns the image after 
    applying perspective transform on it. Using this function, you should
    crop out the arena from the full frame you are receiving from the 
    overhead camera feed.

    HINT:
    Use the ArUco markers placed on four corner points of the arena in order
    to crop out the required portion of the image.

    Input Arguments:
    ---
    `image` :	[ numpy array ]
            numpy array of image returned by cv2 library 

    Returns:
    ---
    `warped_image` : [ numpy array ]
            return cropped arena image as a numpy array
    
    Example call:
    ---
    warped_image = perspective_transform(image)
    """   
    warped_image = [] 
#################################  ADD YOUR CODE HERE  ###############################
    ArUco_details_dict, ArUco_corners = detect_ArUco_details(image) #detecting the Aruco markers using the task2b function
    # print("ArUco details: ", ArUco_details_dict)

    x_firstCorner = int(ArUco_corners.get(3)[2][0])
    x_secondCorner = int(ArUco_corners.get(4)[3][0])
    y_firstCorner = int(ArUco_corners.get(3)[2][1])
    y_fourthCorner = int(ArUco_corners.get(2)[1][1])

    # print("Corners: ",x_firstCorner, x_secondCorner, y_firstCorner, y_fourthCorner)

    warped_image = image[y_firstCorner:y_fourthCorner, x_firstCorner:x_secondCorner] #cropping the given image
    # cv2.imshow("cropped image", warped_image)
    return warped_image

######################################################################################

    # return warped_image

def transform_values(image):

    """
    Purpose:
    ---
    This function takes the image as an argument and returns the 
    position and orientation of the ArUco marker (with id 5), in the 
    CoppeliaSim scene.

    Input Arguments:
    ---
    `image` :	[ numpy array ]
            numpy array of image returned by camera

    Returns:
    ---
    `scene_parameters` : [ list ]
            a list containing the position and orientation of ArUco 5
            scene_parameters = [c_x, c_y, c_angle] where
            c_x is the transformed x co-ordinate [float]
            c_y is the transformed y co-ordinate [float]
            c_angle is the transformed angle [angle]
    
    HINT:
        Initially the image should be cropped using perspective transform 
        and then values of ArUco (5) should be transformed to CoppeliaSim
        scale.
    
    Example call:
    ---
    scene_parameters = transform_values(image)
    """   
    scene_parameters = []
#################################  ADD YOUR CODE HERE  ###############################
    # ArUco_details_dict, ArUco_corners = detect_ArUco_details(image)
    # c_x = ArUco_details_dict.get(5)[0][0]
    # c_y = ArUco_details_dict.get(5)[0][1]
    # c_angle = ArUco_details_dict.get(5)[1]

    ArUco_details_dict, ArUco_corners = detect_ArUco_details(image)
    # print(ArUco_details_dict)
    list = ArUco_details_dict.get(5)
    # print("Aruco 5 angle: ", list[1])

    if list[0][0] <206:
        c_x = -0.9550 + float((0.9550/205)*list[0][0])
    else:
        c_x = float((0.9550/205)*(list[0][0]-205))
    
    if list[0][1] <206:
        c_y = 0.9550 - float((0.9550/205)*list[0][1])
    else:
        c_y = -(float((0.9550/205)*(list[0][1]-205)))
    
    

    # scale=float((0.9550*2)/511)
    # c_x= float((float(list[0][0])*scale)-0.9550+0.2000)
    # c_y= float((float(list[0][1])*scale*(-1))+0.9550)

    scale_angle = 0.01745
    c_angle= list[1]*scale_angle
    # print("Tranformed angle: ", c_angle)

    scene_parameters.append(c_x)
    scene_parameters.append(c_y)
    scene_parameters.append(c_angle)
    # print(c_x, c_y, c_angle)

######################################################################################

    return scene_parameters


def set_values(sim, scene_parameters):
	"""
	Purpose:
	---
	This function takes the scene_parameters, i.e. the transformed values for
	position and orientation of the ArUco marker, and sets the position and 
	orientation in the CoppeliaSim scene.

	Input Arguments:
	---
	`scene_parameters` :	[ list ]
			list of co-ordinates and orientation obtained from transform_values()
			function

	Returns:
	---
	None

	HINT:
		Refer Regular API References of CoppeliaSim to find out functions that can
		set the position and orientation of an object.

	Example call:
	---
	set_values(scene_parameters)
	"""   
	aruco_handle = sim.getObject('/alphabot')
#################################  ADD YOUR CODE HERE  ###############################
	arena_handle = sim.getObject('/Arena')
	# print(scene_parameters)
	#setting postion

	oldPostion= sim.getObjectPosition(aruco_handle, arena_handle) # we will use this to get the c_z value to be used in the set value funtion
	# print(oldPostion)
	newPostion = scene_parameters # Note the values given by this funtion are c_x, c_y and c_angle
	# print(scene_parameters)
	newPostionCoordinates = [newPostion[0], newPostion[1], oldPostion[2]]
	sim.setObjectPosition(aruco_handle, arena_handle, newPostionCoordinates)

	#setting orientation

	orientation = sim.getObjectOrientation(aruco_handle, arena_handle)
	orientation = [-1.57, 0, -1.57]
	# print("hello")
	# print(orientation)
	orientation[1] =  - scene_parameters[2]
	sim.setObjectOrientation(aruco_handle, arena_handle, orientation)
	orientation = sim.getObjectOrientation(aruco_handle, arena_handle)
	# print("Orientation in coppeliaSim: ",orientation)
######################################################################################

	return None


####################### ADD ARENA SETUP ######################
####################### FUNCTIONS HERE #######################
"""
Add functions written in Task 4A for setting up the CoppeliaSim
Arena according to the configuration image in this section
"""
def place_packages(medicine_package_details, sim, all_models):
    """
	Purpose:
	---
	This function takes details (colour, shape and shop) of the packages present in 
    the arena (using "detect_arena_parameters" function from task_1a.py) and places
    them on the virtual arena. The packages should be inserted only into the 
    designated areas in each shop as mentioned in the Task document.

    Functions from Regular API References should be used to set the position of the 
    packages.

	Input Arguments:
	---
	`medicine_package_details` :	[ list ]
                                nested list containing details of the medicine packages present.
                                Each element of this list will contain 
                                - Shop number as Shop_n
                                - Color of the package as a string
                                - Shape of the package as a string
                                - Centroid co-ordinates of the package			

    `sim` : [ object ]
            ZeroMQ RemoteAPI object

    `all_models` : [ list ]
            list containing handles of all the models imported into the scene
	Returns:

    `all_models` : [ list ]
            list containing handles of all the models imported into the scene
	
	Example call:
	---
	all_models = place_packages(medicine_package_details, sim, all_models)
	"""
    models_directory = os.getcwd()
    packages_models_directory = os.path.join(models_directory, "package_models")
    arena = sim.getObject('/Arena')    
####################### ADD YOUR CODE HERE #########################
    # print(medicine_package_details)
    shape_dict = {"circle": "cylinder", "triangle": "cone", "square":"cube"}
    shop_flags = [0, 0, 0, 0, 0, 0]
    shop_x_coordinates = [-0.84, -0.47, -0.125, 0.235, 0.588, 0.948 ]

    for x in medicine_package_details:
        shop_number = int(x[0][-1])
        color = x[1]
        shape = x[2].lower()
        position = x[3]

        package_directory_location = os.path.join(packages_models_directory, color + "_" + shape_dict.get(shape) + ".ttm")

        model_handle = sim.loadModel(package_directory_location)
        sim.setObjectParent(model_handle, arena)
        sim.setObjectAlias(model_handle ,color + "_" + shape_dict.get(shape))

        coordinates_model = sim.getObjectPosition(model_handle, arena)
        coordinates_model[1] = 0.66
        coordinates_model[0] = shop_x_coordinates[shop_number-1] + 0.078*shop_flags[shop_number-1]
        shop_flags[shop_number-1]+=1
        sim.setObjectPosition(model_handle, arena, coordinates_model)

        all_models.append(model_handle)
####################################################################
    return all_models

def node_coordinates(node):
	"""Takes the node and return the corrdinates of it in the coppeliasim coordinates"""
	coordinates = []
	c_z = 0.1

	if node[0] <= "C":
		temp = ord("C") - ord(node[0]) 
		c_x = -0.23 - (temp)*0.356
	else:
		temp = ord(node[0]) - ord("D")
		c_x = 0.12 + (temp)*0.356

	if int(node[1]) <= 3:
		c_y = 0.23 + (3-int(node[1]))*0.356
	else:
		c_y = -0.12 - (int(node[1])-4)*0.356

	coordinates.append(c_x)
	coordinates.append(c_y)
	coordinates.append(c_z)

	return coordinates


def place_traffic_signals(traffic_signals, sim, all_models):
	"""
	Purpose:
	---
	This function takes position of the traffic signals present in 
	the arena (using "detect_arena_parameters" function from task_1a.py) and places
	them on the virtual arena. The signal should be inserted at a particular node.

	Functions from Regular API References should be used to set the position of the 
	signals.

	Input Arguments:
	---
	`traffic_signals` : [ list ]
			list containing nodes in which traffic signals are present

	`sim` : [ object ]
			ZeroMQ RemoteAPI object

	`all_models` : [ list ]
			list containing handles of all the models imported into the scene
	Returns:

	`all_models` : [ list ]
			list containing handles of all the models imported into the scene
	None

	Example call:
	---
	all_models = place_traffic_signals(traffic_signals, sim, all_models)
	"""
	models_directory = os.getcwd()
	traffic_sig_model = os.path.join(models_directory, "signals", "traffic_signal.ttm" )
	arena = sim.getObject('/Arena')
    
    
####################### ADD YOUR CODE HERE #########################
	for traffic_signal in traffic_signals:
		model_coordinates = signal_coordintes(traffic_signal)
		model_handle = sim.loadModel(traffic_sig_model)
		sim.setObjectParent(model_handle, arena)
		sim.setObjectAlias(model_handle ,"Signal_" + traffic_signal)
		sim.setObjectPosition(model_handle, arena, model_coordinates)
		all_models.append(model_handle)


####################################################################
	return all_models

def place_start_end_nodes(start_node, end_node, sim, all_models):
	"""
	Purpose:
	---
	This function takes position of start and end nodes present in 
	the arena and places them on the virtual arena. 
	The models should be inserted at a particular node.

	Functions from Regular API References should be used to set the position of the 
	start and end nodes.

	Input Arguments:
	---
	`start_node` : [ string ]
	`end_node` : [ string ]
					

	`sim` : [ object ]
			ZeroMQ RemoteAPI object

	`all_models` : [ list ]
			list containing handles of all the models imported into the scene
	Returns:

	`all_models` : [ list ]
			list containing handles of all the models imported into the scene
	---
	None

	Example call:
	---
	all_models = place_start_end_nodes(start_node, end_node, sim, all_models)
	"""
	models_directory = os.getcwd()
	start_node_model = os.path.join(models_directory, "signals", "start_node.ttm" )
	end_node_model = os.path.join(models_directory, "signals", "end_node.ttm" )
	arena = sim.getObject('/Arena')   

	start_node_coordintes = signal_coordintes(start_node)
	end_node_coordintes = signal_coordintes(end_node)

	start_model_handle = sim.loadModel(start_node_model)
	end_model_handle = sim.loadModel(end_node_model)

	sim.setObjectPosition(start_model_handle, arena, start_node_coordintes)
	all_models.append(start_model_handle)

	sim.setObjectPosition(end_model_handle, arena, end_node_coordintes)
	all_models.append(end_model_handle)

	sim.setObjectParent(start_model_handle, arena)
	sim.setObjectParent(end_model_handle, arena)

	sim.setObjectAlias(start_model_handle ,"Start_Node")
	sim.setObjectAlias(end_model_handle, "End_Node")

	return all_models

def place_horizontal_barricade(horizontal_roads_under_construction, sim, all_models):
	"""
	Purpose:
	---
	This function takes the list of missing horizontal roads present in 
	the arena (using "detect_arena_parameters" function from task_1a.py) and places
	horizontal barricades on virtual arena. The barricade should be inserted 
	between two nodes as shown in Task document.

	Functions from Regular API References should be used to set the position of the 
	horizontal barricades.

	Input Arguments:
	---
	`horizontal_roads_under_construction` : [ list ]
			list containing missing horizontal links		

	`sim` : [ object ]
			ZeroMQ RemoteAPI object

	`all_models` : [ list ]
			list containing handles of all the models imported into the scene
	Returns:

	`all_models` : [ list ]
			list containing handles of all the models imported into the scene
	---
	None

	Example call:
	---
	all_models = place_horizontal_barricade(horizontal_roads_under_construction, sim, all_models)
	"""
	models_directory = os.getcwd()
	horiz_barricade_model = os.path.join(models_directory, "barricades", "horizontal_barricade.ttm" )
	arena = sim.getObject('/Arena')  
	for baricade in horizontal_roads_under_construction:
		model_coordinates = horiz_baricade_coordinates(baricade)

		model_handle = sim.loadModel(horiz_barricade_model)
		sim.setObjectParent(model_handle, arena)
		sim.setObjectAlias(model_handle ,"Horizontal_missing_road_"+baricade[0:2]+"_"+baricade[3:5])

		model_coordinates.append(sim.getObjectPosition(model_handle, arena)[2])
		sim.setObjectPosition(model_handle, arena, model_coordinates)

		all_models.append(model_handle)

	return all_models


def place_vertical_barricade(vertical_roads_under_construction, sim, all_models):
	"""
	Purpose:
	---
	This function takes the list of missing vertical roads present in 
	the arena (using "detect_arena_parameters" function from task_1a.py) and places
	vertical barricades on virtual arena. The barricade should be inserted 
	between two nodes as shown in Task document.

	Functions from Regular API References should be used to set the position of the 
	vertical barricades.

	Input Arguments:
	---
	`vertical_roads_under_construction` : [ list ]
			list containing missing vertical links		

	`sim` : [ object ]
			ZeroMQ RemoteAPI object

	`all_models` : [ list ]
			list containing handles of all the models imported into the scene
	Returns:

	`all_models` : [ list ]
			list containing handles of all the models imported into the scene
	---
	None

	Example call:
	---
	all_models = place_vertical_barricade(vertical_roads_under_construction, sim, all_models)
	"""
	models_directory = os.getcwd()
	vert_barricade_model = os.path.join(models_directory, "barricades", "vertical_barricade.ttm" )
	arena = sim.getObject('/Arena')

	for baricade in vertical_roads_under_construction:
		model_coordinates = vert_baricade_coordinates(baricade)

		model_handle = sim.loadModel(vert_barricade_model)
		sim.setObjectParent(model_handle, arena)
		sim.setObjectAlias(model_handle ,"Vertical_missing_road_"+baricade[0:2]+"_"+baricade[3:5])

		model_coordinates.append(sim.getObjectPosition(model_handle, arena)[2])
		sim.setObjectPosition(model_handle, arena, model_coordinates)

		all_models.append(model_handle)

	return all_models

##############################################################
##############################################################