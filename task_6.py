'''
*****************************************************************************************
*
*        		===============================================
*           		Pharma Bot (PB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script is to implement Task 6 of Pharma Bot (PB) Theme (eYRC 2022-23).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ PB_3282 ]
# Filename:			task_6.py
# Functions:		
# 					[ get_key, action_shop, action_node, task_6_implementation ]
# Theme: Pharma Bot
# Global Variables: [shop_already_reached, delivery_address, shops_have_packages, end_node, orientation ]

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
from pyzbar.pyzbar import decode
import json
import random
import threading
import queue
##############################################################

global shop_already_reached, delivery_address, shops_have_packages, end_node, orientation
shop_already_reached = []
# function to return key for any value
def get_key(dict, val):
    """Takes the dictionary and the value and return the key"""
    for key, value in dict.items():
        if val == value:
            return key

def action_shop(sim, shop_node, arena_parameters):
	"""It will work if robot has reached a shop node, 
		Input : sim, shop_node, arena_parameters
		output: delivery_address
	
	"""
	global shop_already_reached, delivery_address, shops_have_packages, end_node, orientation

	## Shop nodes dictionary
	shop_nodes_dict = {'Shop_1':'B2', 'Shop_2':'C2', 'Shop_3':'D2', 'Shop_4':'E2', 'Shop_5':'F2', }
	shop_nodes = list(shop_nodes_dict.values())

	# package_details = pb_theme.read_qr_code("qr_drop_5.png")
	# package_details = {"Orange_cone": "E3", "Skyblue_cone": "F3"}
	# delivery_address = package_details
	# print(package_details)

	## Check if shop has already been reached or not, if yes then taking the last package in the delivey_address
	if shop_node in shop_already_reached:
		delivery_address = {get_key(delivery_address, delivery_address[list(delivery_address.keys())[-1]]):delivery_address[list(delivery_address.keys())[-1]]}

	if len(delivery_address) > 3:
		delivery_address.popitem() ## If there are more than 3 packages in the shop then remove the last package
		shops_have_packages[get_key(shop_nodes_dict, shop_node)] -=3 ## Decresing the shops_have_package value by 3
	else:
		del shops_have_packages[get_key(shop_nodes_dict, shop_node)]

	## Palcing the packges on the robot
	package_names = list(delivery_address.keys())
	package_handle = [i.split("_") for i in package_names]
	

	colors = [i[0] for i in package_handle]

	# print("Led colors: ",colors)

	## Sending the colors of packages
	pb_theme.send_message_via_socket(connection_2, colors)

	x =0
	## Placing the packages on alphabot in coppeliasim
	for i in range(len(delivery_address)):
		package_names[i] = "/" + package_names[i]
		model_handle = sim.getObject(package_names[i])
		parent_handel = sim.getObject("/platform")
		sim.setObjectParent(model_handle, parent_handel)
		position = [x, 0, 0.08]
		sim.setObjectPosition(model_handle, parent_handel,position)
		not_dynamic = sim.modelproperty_not_dynamic
		sim.setModelProperty(model_handle, not_dynamic)
		x+=0.05

	## Path plan to first package delivery address
	back_path=pb_theme.path_planning(arena_parameters["paths"], shop_node, list(delivery_address.values())[0])
	moves=pb_theme.paths_to_moves(back_path, arena_parameters["traffic_signals"], orientation)
	# print("Old orientation: ", orientation)
	orientation = pb_theme.orientation(moves, orientation)
	# print("New orientation: ", orientation)
	moves.append(list(delivery_address.values())[0])

	# print("original moves:", moves)

	## Sending the required move to Raspberry Pi client for the motion of the robot
	pb_theme.send_message_via_socket(connection_2, moves)
	# print("Moves: ", moves)

	## Putting in shop_already_reached
	shop_already_reached.append(shop_node)

	return delivery_address

def action_node(sim, node_reached, arena_parameters ):
	"""It will work if robot has reached a node other than shop node,
	input: sim, node_reached, arena_parameters
	output: delivery address
	"""
	global shop_already_reached, delivery_address, shops_have_packages, end_node, orientation

	## Shop nodes dictionary
	shop_nodes_dict = {'Shop_1':'B2', 'Shop_2':'C2', 'Shop_3':'D2', 'Shop_4':'E2', 'Shop_5':'F2', }
	shop_nodes = list(shop_nodes_dict.values())

	## Droppin the package delivered in the coppeliasim scene
	package_handle = "/" + get_key(delivery_address, node_reached)
	model_handle = sim.getObject(package_handle)
	alphabot_handel = sim.getObject("/alphabot")
	arena_handle = sim.getObject("/Arena")
	sim.setObjectParent(model_handle, arena_handle)
	coordinates = pb_theme.node_coordinates(node_reached)
	sim.setObjectPosition(model_handle, arena_handle, coordinates)
	not_dynamic = sim.modelproperty_not_dynamic
	sim.setModelProperty(model_handle, not_dynamic)

	## Taking the color of the package and sending it to robot to turn off the led of that color
	color = get_key(delivery_address, node_reached)
	index = color.index("_")
	color = color[0:index] 
	pb_theme.send_message_via_socket(connection_2, color)

	## Deleting the deliverd package from the delivery _address
	del delivery_address[get_key(delivery_address, node_reached)]

	## Path planning to next node
	if len(delivery_address) != 0: ## Means robot still have some package left to deliver before going to next shop
		back_path=pb_theme.path_planning(arena_parameters["paths"], node_reached, list(delivery_address.values())[0])
		moves=pb_theme.paths_to_moves(back_path, arena_parameters["traffic_signals"], orientation)
		# print("Old orientation: ", orientation)
		orientation = pb_theme.orientation(moves, orientation)
		# print("New orientation: ", orientation)
		moves.append(list(delivery_address.values())[0])
	elif len(delivery_address)==0 and len(shops_have_packages) == 0:
		back_path=pb_theme.path_planning(arena_parameters["paths"], node_reached, end_node)
		moves=pb_theme.paths_to_moves(back_path, arena_parameters["traffic_signals"], orientation)
		# print("Old orientation: ", orientation)
		orientation = pb_theme.orientation(moves, orientation)
		# print("New orientation: ", orientation)
		moves.append(end_node)
	elif len(delivery_address) == 0:
		## Path plan to next shop
		back_path=pb_theme.path_planning(arena_parameters["paths"], node_reached, shop_nodes_dict[list(shops_have_packages.keys())[0]])
		moves=pb_theme.paths_to_moves(back_path, arena_parameters["traffic_signals"], orientation)
		# print("Old orientation: ", orientation)
		orientation = pb_theme.orientation(moves, orientation)
		# print("New orientation: ", orientation)
		moves.append(shop_nodes_dict[list(shops_have_packages.keys())[0]])

	## Sending the required move to Raspberry Pi client for the motion of the robot
	pb_theme.send_message_via_socket(connection_2, moves)

	return delivery_address



## Import PB_theme_functions code
try:
	pb_theme = __import__('PB_theme_functions')

except ImportError:
	print('\n[ERROR] PB_theme_functions.py file is not present in the current directory.')
	print('Your current directory is: ', os.getcwd())
	print('Make sure PB_theme_functions.py is present in this current directory.\n')
	sys.exit()
	
except Exception as e:
	print('Your PB_theme_functions.py throwed an Exception, kindly debug your code!\n')
	traceback.print_exc(file=sys.stdout)
	sys.exit()

def task_6_implementation(sim, arena_parameters):
	"""
	Purpose:
	---
	This function contains the implementation logic for task 6 

	Input Arguments:
	---
    `sim` : [ object ]
            ZeroMQ RemoteAPI object

	You are free to define additional input arguments for this function.

	Returns:
	---
	You are free to define output parameters for this function.
	
	Example call:
	---
	task_5_implementation(sim)
	"""

	##################	ADD YOUR CODE HERE	##################
	global shop_already_reached, delivery_address, shops_have_packages, end_node, orientation

	## Taking required data from the arena_parameters
	requirement = arena_parameters.copy()
	del requirement['paths']
	del requirement['traffic_signals']
	del requirement['horizontal_roads_under_construction']
	del requirement['vertical_roads_under_construction']
	# print(requirement)

	## Shop nodes dictionary
	shop_nodes_dict = {'Shop_1':'B2', 'Shop_2':'C2', 'Shop_3':'D2', 'Shop_4':'E2', 'Shop_5':'F2', }
	shop_nodes = list(shop_nodes_dict.values())

	## Nodes for the start, end and next checkpoint
	start_node = requirement['start_node']
	next_node = None
	end_node = requirement['end_node']

	## Finding which shops have packages and how many packages
	shops_have_packages = {}

	for i in range(len(requirement['medicine_packages'])):
		shop_name = requirement['medicine_packages'][i][0]

		if shop_name in shops_have_packages:
			shops_have_packages[shop_name] += 1
		else:
			shops_have_packages[shop_name] = 1
	
	# print(shops_have_packages)
	orientation = 0

	## Path plan to first shop in the shop_have_packges
	next_node = shop_nodes_dict[list(shops_have_packages.keys())[0]]
	# print(arena_parameters["paths"])
	back_path=pb_theme.path_planning(arena_parameters["paths"], start_node, next_node)
	# print(back_path)
	moves=pb_theme.paths_to_moves(back_path, arena_parameters["traffic_signals"], orientation)
	orientation = pb_theme.orientation(moves, orientation)
	# print("orientation: ", orientation)
	moves.append(next_node)
	# print(moves)
	

	## Sending the required move to Raspberry Pi client for the motion of the robot
	pb_theme.send_message_via_socket(connection_2, moves)

	## create a queue for passing data from client to main thread
	q = queue.Queue()
	
	t = threading.Thread(target=pb_theme.listen_client, args=(connection_2, q))
	t.start()
	count = 0
	while True:
    # Read a frame from the video
		ret, frame = capture.read()

		# If the frame was successfully read
		if ret:
			# Display the frame
			# cv2.imshow('Video', frame)


			## Cropping the frame and emulating in the coppeliasim
			try:
				croped = pb_theme.perspective_transform(frame)
				cv2.imshow('Video', croped)
				scene_parameters = pb_theme.transform_values(croped)
				pb_theme.set_values(sim, scene_parameters)
				ArUco_details_dict, ArUco_corners = pb_theme.detect_ArUco_details(frame)
				# print(ArUco_details_dict.get(5)[0])
			except:
				None

			## check if there is data in the queue
			if not q.empty():
				message = q.get()
				# message = pb_theme.receive_message_via_socket(connection_2)
				# print("Message recived:", message)
			
			else:
				message = ''

			## Check if the message have REACHED in it or not
			if "REACHED" in  message:
				# print(message)
				node_reached = message.split("_")[1] # Node which robot has reaached

				## check if message have reached a shop_node, last_node or any another node
				if node_reached in shop_nodes:
					## Feeding the qr codes values
					if count == 0:
						delivery_address = {"Pink_cylinder": "F5", "Green_cylinder":"C4"} 	
						count += 1
					elif count ==1:
						delivery_address = {"Pink_cube": "C3", "Orange_cube": "B5", "Skyblue_cube": "A6"}
						count += 1
					
					delivery_address = action_shop(sim, node_reached, arena_parameters)

				elif node_reached == requirement['end_node']:
					pb_theme.send_message_via_socket(connection_2, "DONE")
					break

				else:
					delivery_address = action_node(sim, node_reached, arena_parameters)
			else:
				None

		# Wait for 1 millisecond for a key event
		key = cv2.waitKey(30)

		# If the 'q' key is pressed, exit the loop
		if key == ord('q'):
			break
	
	## Releasing capture frame and delting all windows
	capture.release()
	cv2.destroyAllWindows()	

	##########################################################

if __name__ == "__main__":
	
	host = '192.168.224.12'
	port = 5050

	global capture
	capture = cv2.VideoCapture(0)


	## Set up new socket server
	try:
		server = pb_theme.setup_server(host, port)
		print("Socket Server successfully created")

		# print(type(server))

	except socket.error as error:
		print("Error in setting up server")
		print(error)
		sys.exit()


	## Set up new connection with a socket client (PB_task3d_socket.exe)
	try:
		print("\nPlease run PB_socket.exe program to connect to PB_socket client")
		connection_1, address_1 = pb_theme.setup_connection(server)
		print("Connected to: " + address_1[0] + ":" + str(address_1[1]))

	except KeyboardInterrupt:
		sys.exit()


	# ## Set up new connection with a socket client (socket_client_rgb.py)
	try:
		print("\nPlease connect to Raspberry pi client")
		connection_2, address_2 = pb_theme.setup_connection(server)
		print("Connected to: " + address_2[0] + ":" + str(address_2[1]))

	except KeyboardInterrupt:
		sys.exit()

	## Send setup message to PB_socket
	pb_theme.send_message_via_socket(connection_1, "SETUP")

	message = pb_theme.receive_message_via_socket(connection_1)
	## Loop infinitely until SETUP_DONE message is received
	while True:
		if message == "SETUP_DONE":
			break
		else:
			print("Cannot proceed further until SETUP command is received")
			message = pb_theme.receive_message_via_socket(connection_1)


	try:
		
		# obtain required arena parameters
		image_filename = os.path.join(os.getcwd(), "config_image.png")
		config_img = cv2.imread(image_filename)
		detected_arena_parameters = pb_theme.detect_arena_parameters(config_img)			
		medicine_package_details = detected_arena_parameters["medicine_packages"]
		traffic_signals = detected_arena_parameters['traffic_signals']
		start_node = detected_arena_parameters['start_node']
		end_node = detected_arena_parameters['end_node']
		horizontal_roads_under_construction = detected_arena_parameters['horizontal_roads_under_construction']
		vertical_roads_under_construction = detected_arena_parameters['vertical_roads_under_construction']

		# print(detected_arena_parameters)
		# print("Medicine Packages: ", medicine_package_details)
		# print("Traffic Signals: ", traffic_signals)
		# print("Start Node: ", start_node)
		# print("End Node: ", end_node)
		# print("Horizontal Roads under Construction: ", horizontal_roads_under_construction)
		# print("Vertical Roads under Construction: ", vertical_roads_under_construction)
		# print("\n\n")

	except Exception as e:
		print('Your task_1a.py throwed an Exception, kindly debug your code!\n')
		traceback.print_exc(file=sys.stdout)
		sys.exit()

	try:

		## Connect to CoppeliaSim arena
		coppelia_client = RemoteAPIClient()
		sim = coppelia_client.getObject('sim')

		## Define all models
		all_models = []

		## Setting up coppeliasim scene
		print("[1] Setting up the scene in CoppeliaSim")
		all_models = pb_theme.place_packages(medicine_package_details, sim, all_models)
		all_models = pb_theme.place_traffic_signals(traffic_signals, sim, all_models)
		all_models = pb_theme.place_horizontal_barricade(horizontal_roads_under_construction, sim, all_models)
		all_models = pb_theme.place_vertical_barricade(vertical_roads_under_construction, sim, all_models)
		all_models = pb_theme.place_start_end_nodes(start_node, end_node, sim, all_models)
		print("[2] Completed setting up the scene in CoppeliaSim")
		print("[3] Checking arena configuration in CoppeliaSim")

	except Exception as e:
		print('Your task_4a.py throwed an Exception, kindly debug your code!\n')
		traceback.print_exc(file=sys.stdout)
		sys.exit()

	pb_theme.send_message_via_socket(connection_1, "CHECK_ARENA")

	## Check if arena setup is ok or not
	message = pb_theme.receive_message_via_socket(connection_1)
	while True:
		# message = pb_theme.receive_message_via_socket(connection_1)

		if message == "ARENA_SETUP_OK":
			print("[4] Arena was properly setup in CoppeliaSim")
			break
		elif message == "ARENA_SETUP_NOT_OK":
			print("[4] Arena was not properly setup in CoppeliaSim")
			connection_1.close()
			# connection_2.close()
			server.close()
			sys.exit()
		else:
			pass

	## Send Start Simulation Command to PB_Socket
	pb_theme.send_message_via_socket(connection_1, "SIMULATION_START")
	
	## Check if simulation started correctly
	message = pb_theme.receive_message_via_socket(connection_1)
	while True:
		# message = pb_theme.receive_message_via_socket(connection_1)

		if message == "SIMULATION_STARTED_CORRECTLY":
			print("[5] Simulation was started in CoppeliaSim")
			break

		if message == "SIMULATION_NOT_STARTED_CORRECTLY":
			print("[5] Simulation was not started in CoppeliaSim")
			sys.exit()

	# send_message_via_socket(connection_2, "START")

	task_6_implementation(sim, detected_arena_parameters)


	## Send Stop Simulation Command to PB_Socket
	pb_theme.send_message_via_socket(connection_1, "SIMULATION_STOP")

	## Check if simulation started correctly
	message = pb_theme.receive_message_via_socket(connection_1)
	while True:
		# message = pb_theme.receive_message_via_socket(connection_1)

		if message == "SIMULATION_STOPPED_CORRECTLY":
			print("[6] Simulation was stopped in CoppeliaSim")
			break

		if message == "SIMULATION_NOT_STOPPED_CORRECTLY":
			print("[6] Simulation was not stopped in CoppeliaSim")
			sys.exit()