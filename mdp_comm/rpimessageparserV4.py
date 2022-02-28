# -*- coding: utf-8 -*-
"""rpiMessageParser.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1CMhq_vBd1h5QRsakDWc7wH_kmEjeGYSg

Rpi to android W,A,S,D
Android to Rpi f,b,e,q


Rpi to android
TARGET, <obstacleNo>, <ImgID>

ROBOT, <coordsx>, <coordsy> up/down/left/right


Android to rpi
starting (x,y,0/1/2/3)

Obstacle (x,y). (ObstacleNO) Facing
"""

# Android sends to rpi the robots starting location and where it is facing
# RPI must be able to understand the information that android is sending over
def androidToRpi(message):

  first_word = message.split()[0]

  if first_word == "starting":
    x_coord, y_coord, robot_facing_direction = startPos("".join(message.split()[1:]))
    return first_word, x_coord, y_coord, robot_facing_direction
  elif first_word == "Obstacle":
    x_coord, y_coord, obstacle_number, obstacle_image_direction = obstaclePos("".join(message.split()[1:]))
    return first_word, x_coord, y_coord, obstacle_number, obstacle_image_direction
  elif first_word == "moving":
    movement = robotMovement("".join(message.split()[1:]))
    return first_word, movement
  elif first_word == "discover":
    return (first_word,)
  elif first_word == "fastest":
    return (first_word,)
# When android places down the robot on the virtual grid map, a start message gets sent to RPI
# This will be in the form of a string "starting (x,y,0/1/2/3)"
# 0 means the robot is facing up, 
# 1 means the robot is facing right,
# 2 means the robot is facing down,
# 3 means the robot is facing left
def startPos(startPositionInfo):
  message_without_brackets = startPositionInfo[1:len(startPositionInfo)-1]
  message_array = message_without_brackets.split(",")

  x_coord = message_array[0]

  y_coord = message_array[1]
  
  robot_facing_direction = message_array[2]
  
  # MAYBE SEND RPI THE VARIABLES HERE INSTEAD OF RETURNING 
  return x_coord, y_coord, robot_facing_direction


# When android places down an obstacle on the virtual grid map, a message gets sent to RPI
# This will be in the form of a string "Obstacle (x,y). (ObstacleNO) Facing"
# Obstacle denotes the start of an obsacle string
# (x,y) are the x and y coordinates,
# ObstacleNO is the number of the obstacle we are putting down (),
# Facing can be up/down/left/right. This represents the direction the image is facing
def obstaclePos(obstaclePositionInfo):
  message_without_brackets = obstaclePositionInfo
  message_array = message_without_brackets.split(")")

  xy_coords = message_array[0].split(",")
  x_coord = xy_coords[0][1:]
  y_coord = xy_coords[1]

  obstacle_number = message_array[1][2]

  obstacle_image_direction = message_array[2]

  # MAYBE SEND RPI THE VARIABLES HERE INSTEAD OF RETURNING
  return x_coord, y_coord, obstacle_number, obstacle_image_direction

def robotMovement(robotMovementInfo):

  # ADD WRAPPER
  blahblah = robotMovementInfo
  return blahblah

# # Test cases to check the functions that parse data that comes from android
# a, b, c, d = androidToRpi("Obstacle (5,1). (3) up")
# print(a, b, c, d)

# a, b, c= androidToRpi("starting (x,y,0)")
# print(a, b, c)

# tup = androidToRpi("starting (x,y,0)")
# tup
tup = androidToRpi("moving f")
tup

# Code that sends from RPI to android
# RPI must be able to pack the data so that android would be able to understand what to do with it

# Code that sends imageID that was detected to android
# TARGET, <obstacleNo>, <ImgID>     must be sent to android so as to activate the function to change the image face when image id has been successfully detected
# TARGET, <2>, <11>                 would mean that we have detected obstacle 2 to have contained the image with image ID of 11
def foundImgId(obstacleNo, imgIdDetected):
  image_id = str (imgIdDetected)
  obstacle_number = str (obstacleNo)

  command_to_change_image = "TARGET, " + "<" + obstacle_number + ">, <" + image_id + ">"
  
  print(command_to_change_image)

foundImgId(3, 21)

# Rpi to android W,A,S,D
# Android to Rpi f,b,e,q

# This sends instructions via strings to RPI
def rpiToAndroidMovement(algorithmPathArray):
  
  # When path array == go up, than we send W
  # When path array == go left, than we send A ect...

# direction is the way robot is facing
  direction = 0                                # direction 0/1/2/3 corresponds to N/E/S/W
  movement_array = []   
  for sub_path in algorithmPathArray:
    for i in range(len(sub_path) - 1):
      initial_pos = sub_path[i]
      new_pos = sub_path[i+1]

      initial_pos = str(initial_pos)
      initial_pos = initial_pos.lstrip("(").rstrip(")")
      initial_pos = initial_pos.split(",")
      initial_pos_x = int( initial_pos[0].strip() )
      initial_pos_y = int( initial_pos[1].strip() )

      new_pos = str(new_pos)
      new_pos = new_pos.lstrip("(").rstrip(")")
      new_pos = new_pos.split(",")
      new_pos_x = int( new_pos[0].strip() )
      new_pos_y = int( new_pos[1].strip() )

      
      if (direction == 0):
        if (initial_pos_x - new_pos_x == 0 and initial_pos_y - new_pos_y == -1):
          movement_array.append("W")
          print("RPI to ANDROID", "W", str(direction) )
        elif (initial_pos_x - new_pos_x == 0 and initial_pos_y - new_pos_y == 1):
          movement_array.append("S")
          print("RPI to ANDROID", "S", str(direction) )
        elif (initial_pos_x - new_pos_x == 1 and initial_pos_y - new_pos_y == -1):
          direction = 3
          movement_array.append("A")
          print("RPI to ANDROID", "A", str(direction) )
        elif (initial_pos_x - new_pos_x == -1 and initial_pos_y - new_pos_y == -1):
          direction = 1
          movement_array.append("D")
          print("RPI to ANDROID", "D", str(direction) )

      elif (direction == 1):
        if (initial_pos_x - new_pos_x == -1 and initial_pos_y - new_pos_y == 0):
          movement_array.append("W")
          print("RPI to ANDROID", "W", str(direction) )
        elif (initial_pos_x - new_pos_x == 1 and initial_pos_y - new_pos_y == 0):
          movement_array.append("S")
          print("RPI to ANDROID", "S", str(direction) )
        elif (initial_pos_x - new_pos_x == -1 and initial_pos_y - new_pos_y == -1):
          direction = 0
          movement_array.append("A")
          print("RPI to ANDROID", "A", str(direction) )
        elif (initial_pos_x - new_pos_x == -1 and initial_pos_y - new_pos_y == 1):
          direction = 2
          movement_array.append("D")
          print("RPI to ANDROID", "D", str(direction) )

      elif (direction == 2):
        if (initial_pos_x - new_pos_x == 0 and initial_pos_y - new_pos_y == 1):
          movement_array.append("W")
          print("RPI to ANDROID", "W", str(direction) )
        elif (initial_pos_x - new_pos_x == 0 and initial_pos_y - new_pos_y == -1):
          movement_array.append("S")
          print("RPI to ANDROID", "S", str(direction) )
        elif (initial_pos_x - new_pos_x == -1 and initial_pos_y - new_pos_y == 1):
          direction = 1
          movement_array.append("A")
          print("RPI to ANDROID", "A", str(direction) )
        elif (initial_pos_x - new_pos_x == 1 and initial_pos_y - new_pos_y == 1):
          direction = 3
          movement_array.append("D")
          print("RPI to ANDROID", "D", str(direction) )

      elif (direction == 3):
        if (initial_pos_x - new_pos_x == 1 and initial_pos_y - new_pos_y == 0):
          movement_array.append("W")
          print("RPI to ANDROID", "W", str(direction) )
        elif (initial_pos_x - new_pos_x == -1 and initial_pos_y - new_pos_y == 0):
          movement_array.append("S")
          print("RPI to ANDROID", "S", str(direction) )
        elif (initial_pos_x - new_pos_x == 1 and initial_pos_y - new_pos_y == 1):
          movement_array.append("A")
          direction = 2
          print("RPI to ANDROID", "A", str(direction) )
        elif (initial_pos_x - new_pos_x == 1 and initial_pos_y - new_pos_y == -1):
          direction = 0
          movement_array.append("D")
          print("RPI to ANDROID", "D", str(direction) )

  return movement_array

rpiToAndroidMovement([[(10,10), (10,11), (10,12), (11, 13), (12,13)]])
