from cmath import atan, tan
import sys
import time
import random
import math
import numpy as np
import cv2 # type: ignore
sys.path.append('C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python\src') 
from coppeliasim_zmqremoteapi_client import RemoteAPIClient # type: ignore

# This code connects to CoppeliaSim
client = RemoteAPIClient()
sim = client.getObject('sim')
print('Connected to CoppeliaSim')
sim.stopSimulation()
# This code starts the simulation
sim.startSimulation()
print("Simulation started.")

# Get Object Handle is basically the handle of the robot we want to control
handle = sim.getObject('/NiryoOne') # Robot name is NiryoOne
print(f"Robot handle: {handle}")

# Getting the base
Robot_base = sim.getObject('/NiryoOne/visible')
print(f"Base handle: {Robot_base}")

# Getting the Gripper
Robot_gripper = sim.getObject('/NiryoOne/NiryoLGripper')
print(f"Gripper handle: {Robot_gripper}")

# Getting Vision Sensor
Vision_sensor = sim.getObject('/NiryoOne/Vision_sensor')
print(f"Vision Sensor handle: {Vision_sensor}")

# Getting Proximity Sensor
Proximity_sensor = sim.getObject('/NiryoOne/NiryoLGripper/Proximity_sensor')
print(f"Proximity Sensor handle: {Proximity_sensor}")

# Getting the joints
Robot_joints = [ 
    sim.getObject('/NiryoOne/Joint'),
    sim.getObject('/NiryoOne/Link/Joint'),
    sim.getObject('/NiryoOne/Link/Joint/Link/Joint'),
    sim.getObject('/NiryoOne/Link/Joint/Link/Joint/Link/Joint'),
    sim.getObject('/NiryoOne/Link/Joint/Link/Joint/Link/Joint/Link/Joint'),
    sim.getObject('/NiryoOne/Link/Joint/Link/Joint/Link/Joint/Link/Joint/Link/Joint'),
]

# Configuration function for the robot Arm
def move_to_config(joints, maxVel, maxAccel, maxJerk, target_config):
    params = {
        'joints': joints,
        'targetPos': target_config,
        'maxVel': maxVel,
        'maxAccel': maxAccel,
        'maxJerk': maxJerk
    }
    sim.moveToConfig(params)

# Converts Degrees to Radian
def deg_to_radian(x):
    return x * 3.14159 / 180

# getting ditance between two points
def calculate_distance(pos1, pos2):
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[0])**2)

# This is for randomly placing bricks in the robot's workspace and dummies (needed for IK)!
def place_bricks(base, arm_position, max_radius, placed_bricks):

    # Getting Base position
    base_position = sim.getObjectPosition(base, -1)

    # Getting size of brick (length x width x height)
    brick_size = [0.04, 0.04, 0.04]

    min_distance = 0.35
    max_attempts = 100

    # Two different locations for the regions for the bricks to be spawned in
    region1 = {'x': [max_radius * 0.6,max_radius * 0.9], 'y': [-0.1, 0.1]}
    region2 = {'x': [max_radius * 0.4,max_radius * 0.8], 'y': [-0.1, 0.1]}

    selected_region = random.choice([region1, region2])

    attempts = 0

    while attempts < max_attempts:
        attempts += 1

        # Getting location of brick
        angle = random.uniform(0, 2 * math.pi)  # Random angle for radial distribution
        radius = random.uniform(max_radius * 0.5, max_radius * 0.8)

        x_offset = radius * math.cos(angle)
        y_offset = radius * math.sin(angle)

        x_offset += random.uniform(selected_region['x'][0], selected_region['x'][1])
        y_offset += random.uniform(selected_region['y'][0], selected_region['y'][1])
        z_offset = brick_size[2] / 2 + 0.2

        #brick_position = [base_position[0] + x_offset, base_position[1] + y_offset, base_position[2] + z_offset]
        brick_position = [0.33147, 0.4317, 0.04]

        valid_position = True
        for existing_brick in placed_bricks:
            existing_pos = sim.getObjectPosition(existing_brick, -1)
            if calculate_distance(brick_position, existing_pos) < min_distance:
                valid_position = False
                break

        if valid_position:

            #brick_orientation = [roll, pitch, yaw]
            brick_orientation = [0,0,0]

            # Create Brick
            brick = sim.createPrimitiveShape(sim.primitiveshape_cuboid, brick_size, 0)
            sim.setObjectPosition(brick, -1, brick_position)
            sim.setObjectOrientation(brick, -1, brick_orientation)

            # Colours for Brick
            colour = [random.random(), random.random(), random.random()]
            sim.setShapeColor(brick, None, sim.colorcomponent_ambient, colour)

            # Create dummy
            dummy = sim.createDummy(0.02, colour)
            sim.setObjectParent(dummy, brick, True)
            sim.setObjectPosition(dummy, brick, [0, 0, brick_size[2]/4])

            # Brick Settings
            floor = sim.getObject('/Floor')

            sim.setObjectInt32Param(brick, sim.shapeintparam_respondable, 1)
            sim.setObjectInt32Param(floor, sim.shapeintparam_respondable, 1)

            # Making brick dynamic
            sim.setObjectInt32Param(brick, sim.shapeintparam_static, 0)
            sim.setObjectFloatParam(brick, sim.shapefloatparam_mass, 0.1)

            # Making brick and Floor collidable :)
            sim.setObjectSpecialProperty(brick, sim.objectspecialproperty_collidable)
            sim.setObjectSpecialProperty(floor, sim.objectspecialproperty_collidable)

            # Slowed down the bricks
            sim.setArrayParam(sim.arrayparam_gravity, [0, 0, -2.5])

            return brick, dummy
    return None



def close_gripper():
    sim.setJointTargetPosition(Robot_gripper, 1)

def open_gripper():
    left_joint = sim.getObject('/NiryoOne/NiryoLGripper/leftJoint1')
    right_joint = sim.getObject('/NiryoOne/NiryoLGripper/rightJoint1')

    sim.setJointTargetPosition(left_joint, -1.39)
    sim.setJointTargetPosition(right_joint, +1.39)
    sim.wait(2.0)

def close_gripper():
    left_joint = sim.getObject('/NiryoOne/NiryoLGripper/leftJoint1')
    right_joint = sim.getObject('/NiryoOne/NiryoLGripper/rightJoint1')

    sim.setJointTargetPosition(left_joint, 0)
    sim.setJointTargetPosition(right_joint, 0)
    sim.wait(2.0)

def smooth_move(robot_tip, target_pos, target_ori, waypoints=[], duration=1.0):
    waypoints.append(target_pos)
    
    # Move smoothly through each waypoint
    for waypoint in waypoints:
        sim.setObjectPosition(robot_tip, -1, waypoint)
        sim.setObjectOrientation(robot_tip, -1, target_ori)
        sim.wait(duration)

# Moving the robot
def move_to_position(robot_tip, target_position, target_orientation):

    # Open the gripper
    open_gripper()

    # check for the orientation here!
    sim.setObjectOrientation(robot_tip, -1, target_orientation)
    sim.setObjectPosition(robot_tip, -1, target_position)
    sim.wait(5.0)

    # Close the gripper
    close_gripper()


# Moving the robot
def move_to_plane(robot_tip, target_position, target_orientation):

    waypoints = [
        [target_position[0], target_position[1], target_position[2] ],  # Slightly above the target
        target_position
    ]
    smooth_move(robot_tip, target_position, target_orientation, waypoints, duration=1.0)
    sim.wait(0.2)

    # Open the gripper
    open_gripper()

def path_movement(robot_tip, target_position, target_orientation):

    waypoints = [
        [target_position[0], target_position[1], target_position[2] ],  # Slightly above the target
        target_position
    ]
    smooth_move(robot_tip, target_position, target_orientation, waypoints, duration=1.0)
    sim.wait(0.2)


# Moving the robot
def move_to_default(robot_tip, target_position, target_orientation):

    waypoints = [
        [target_position[0], target_position[1], target_position[2] + 0.046],
        target_position
    ]
    smooth_move(robot_tip, target_position, target_orientation, waypoints, duration = 1.0)
    

# This checks whether the gripper is close or not
def is_gripper_close(gripper_position, block_position, threshold=0.05):

    distance = np.linalg.norm(np.array(gripper_position) - np.array(block_position))
    return distance <= threshold

# Detecting the blocks from the Vision sensor
def scanning_for_blocks(brick, dummy,target_position,path):

    for angle in range(0, 360, 360):

        target_config = [deg_to_radian(angle), 0,0,0,0,0]

        # Robot moves
        move_to_config(Robot_joints, maxVel, maxAccel, maxJerk, target_config)

        # Setting the robot tip
        previous_tip = sim.getObject('/Dummy')
        

        # Sensor Time! (Ignored for now)
        # vision_data = sim.getVisionSensorImg(Vision_sensor, 0)
        # image = Image_retriver(vision_data, 256, 256)
        # blocks = detect_blocks(image)

        # for block in blocks:
        #     block_position = getBlock_Position(block)
        #     if block_position:
        #         detected_blocks.append(block_position)

        # Pick up the Block
        
        #gripper_position = sim.getObjectPosition(Robot_gripper, -1)
        brick_position = sim.getObjectPosition(dummy, -1)
        brick_orientation = sim.getObjectOrientation(dummy, -1)
        brick_matrix = sim.getObjectMatrix(brick, -1)
        
        print("Detected block to Pick UP OVER HEREREER")
        brick_position = [brick_position[0], brick_position[1], brick_position[2]+0.045]
        # = [brick_orientation[0], brick_orientation[1], brick_orientation[2]]
        sim.setObjectPose(previous_tip, -1, sim.getObjectPose(dummy, -1))
        # getting the block
        move_to_position(previous_tip, brick_position, brick_orientation)
                    

        sim.setObjectParent(brick, Robot_gripper, True)

        move_to_config(Robot_joints, maxVel, maxAccel, maxJerk, target_config)

        # Change the target place to the plane
        target_dummy = sim.getObject('/TargetDummy')
        sim.setObjectPosition(target_dummy, -1, target_position)
        sim.setObjectPose(previous_tip, -1, sim.getObjectPose(target_dummy, -1))
        brick_position = sim.getObjectPosition(target_dummy, -1)
        brick_orientation = sim.getObjectOrientation(target_dummy, -1)

        # for i in range(len(path)):
        #     print("Path: ",path[i])
        #     path_movement(previous_tip, path[i], [brick_orientation[0],brick_orientation[1],brick_orientation[2]+deg_to_radian(90)])

        # Move to the plane
        move_to_plane(previous_tip, brick_position, [brick_orientation[0],brick_orientation[1],brick_orientation[2]+deg_to_radian(90)])
        # Increase the height of the target dummy to stack them - WRITE CODE HERE!!!
        print("brick Position: ",brick_position)
        sim.setObjectPosition(target_dummy, -1, [brick_position[0], brick_position[1], brick_position[2]])
        
        move_to_config(Robot_joints, maxVel, maxAccel, maxJerk, target_config)
        # Log progress
        print("Block successfully picked and returned to default position!")
                
# Setting up the RML Vectors (Rapid Motion Action) (velocity, acceleration, jerk)

# vel is Velocity:
# -> increasing it makes robot move faster across all the joints
# -> decreasing it makes robot move slower across all the joints
vel = 80

# accel is Acceleration
# -> increasing it makes robot reach max vel quickly
# -> decreasing it makes robot reach max vel slowly
accel = 100

# jerk is Jerk
# -> increasing it makes robot movement jerky
# -> decreasing it makes robot movement smooth
jerk = 10

def assign_values(map, i, j, value, max_h, max_w, memo):
    # Base case: Out of bounds, already visited, or cell not empty
    if i < 0 or j < 0 or i >= max_h or j >= max_w or (i, j) in memo or map[i][j] != [0, 0]:
        return
    
    # Memoize the cell
    memo.add((i, j))
    
    # Assign the value to the map
    map[i][j] = value
    
    # Recur for neighboring cells
    assign_values(map, i + 1, j, [value[0] + 0.01, value[1]], max_h, max_w, memo)
    assign_values(map, i - 1, j, [value[0] - 0.01, value[1]], max_h, max_w, memo)
    assign_values(map, i, j + 1, [value[0], value[1] + 0.01], max_h, max_w, memo)
    assign_values(map, i, j - 1, [value[0], value[1] - 0.01], max_h, max_w, memo)


# def create_map(start_pos, end_pos, workspace_bounds, resolution): 
#     # Retrieve positions
#     base_position = sim.getObjectPosition(Robot_base, -1)
#     obstacle = sim.getObject('/Obstacle')
#     obstacle_position = sim.getObjectPosition(obstacle, -1)
    
#     # Initialize the grid map
#     x_min, x_max, y_min, y_max = workspace_bounds
#     grid_width = int((x_max - x_min) / resolution)
#     grid_height = int((y_max - y_min) / resolution)
#     grid_map = [[[0, 0] for _ in range(grid_width)] for _ in range(grid_height)]

#     print("Grid map size:", grid_height, grid_width)

#     # Calculate grid indices for start position
#     start_i = int((start_pos[0] - x_min) / resolution)
#     start_j = int((start_pos[1] - y_min) / resolution)
    
#     # Initialize memoization set
#     memo = set()
#     assign_values(grid_map, start_i, start_j, [start_pos[0], start_pos[1]], grid_height, grid_width, memo)
    
#     # Mark other key positions
#     end_i = int((end_pos[0] - x_min) / resolution)
#     end_j = int((end_pos[1] - y_min) / resolution)
#     grid_map[end_i][end_j] = [end_pos[0], end_pos[1]]

#     obstacle_i = int((obstacle_position[0] - x_min) / resolution)
#     obstacle_j = int((obstacle_position[1] - y_min) / resolution)
#     grid_map[obstacle_i][obstacle_j] = [obstacle_position[0], obstacle_position[1]]

#     base_i = int((base_position[0] - x_min) / resolution)
#     base_j = int((base_position[1] - y_min) / resolution)
#     grid_map[base_i][base_j] = [base_position[0], base_position[1]]

#     return grid_map,[start_i, start_j],[end_i, end_j]


# def visualize_map(grid_map):
#     # Simple text-based visualization
#     for row in grid_map:
#         print("".join(["." if cell == 0 else str(cell) for cell in row]))

def A_Star_and_RRT(start_pos, end_p, obstacle_position):
    path = []
    i, j = start_pos[0], start_pos[1]
    end_pos = [end_p[0], end_p[1]]
    moves = [[0.01, 0.01], [-0.01, -0.01], [0.01, -0.01], [-0.01, 0.01], [0, 0.01], [0.01, 0], [0, -0.01], [-0.01, 0]]
    iter_count = 0
    robot_base_pos=sim.getObjectPosition(Robot_base, -1)
    cost=1

   

    while calculate_distance([i, j], end_pos) > 0.01:
        iter_count += 1
        

        best_move = None
        min_distance = float('inf')

        # Check all possible moves
        for mv in moves:
            ti = i + mv[0]
            tj = j + mv[1]
            
            
            
            disT = calculate_distance([ti, tj], end_pos)
            
            if disT < min_distance:
                min_distance = disT
                best_move = [ti, tj]

        # If no valid move is found (corner case)
        if best_move is None:
            print("No valid moves found. Algorithm terminated.")
            break

        if calculate_distance([best_move[0], best_move[1]], obstacle_position) < 0.05:
            path.pop()
            path.pop()
            temp=path.pop()
            t1=calculate_distance([temp[0], temp[1]+0.2], robot_base_pos)
            t2=calculate_distance([temp[0], temp[1]-0.2], robot_base_pos)
            cost-=2
            continue
            

        # Update current position and add to path
        i, j = best_move[0], best_move[1]
        path.append([i, j])
        cost+=1

    return path



        


maxVel = [deg_to_radian(vel)] * 6
maxAccel = [deg_to_radian(accel)] * 6
maxJerk = [deg_to_radian(jerk)] * 6


def robot_default_position():
    default_position = [0.33147, 0.4317, 0.09, 0, 0, 0]
    move_to_config(Robot_joints, maxVel, maxAccel, maxJerk, default_position)

# This is the main loop for the code
def main_loop():
    try:
                
        # get link size
        link1_size = 0.15
        link2_size = 0.15
        
        max_radius = 0.25
        arm_position = sim.getObjectPosition(Robot_gripper, -1)
        
        obstacle= sim.getObject('/Obstacle')
        obstacle_position = sim.getObjectPosition(obstacle, -1)
        base_position = sim.getObjectPosition(Robot_base, -1)
        
        col=2
        bricks = []
        dummies = []
        max_bricks = 16
        
        target_position = [-0.225, 0.025, 0.002-0.04]
        workspace_bounds = (-0.5, 0.5, -0.5, 0.5)  # (x_min, x_max, y_min, y_max)
        resolution = 0.05  # Grid cell size
        x_min, x_max, y_min, y_max = workspace_bounds
        grid_width = int((x_max - x_min) / resolution)
        grid_height = int((y_max - y_min) / resolution)
        start_pos=[0.33147, 0.4317, 0.04]
        end_pos=[-0.10, 0.025, 0.08]

        previous_tip = sim.getObject('/Dummy')
        robot_default_position()
        # Map,startP,endP=create_map([0.33147, 0.4317, 0.04], [-0.20, 0.025, 0.08],workspace_bounds,resolution) #create Map
        path=A_Star_and_RRT(start_pos,end_pos,obstacle_position) #find path
        print("Path length",len(path))
        print("Path",path)
        for i in range(len(path)):
            path[i]=[path[i][0],path[i][1],0.35]

        newPath=[]
        for i in range(len(path)):
            if i>len(path)-len(path)/5:
                break

            if i%15==0:
                newPath.append(path[i])
        path = newPath
        
        
        
        
        for i in range(max_bricks):
            new_brick, new_dummy = place_bricks(Robot_base, arm_position, max_radius, bricks)
            bricks.append(new_brick)
            dummies.append(new_dummy)
            robot_default_position()
            time.sleep(0.2)
            scanning_for_blocks(bricks[i], dummies[i], target_position,path)
            
            if col>0:
                col-=1
                target_position[0]+=0.0401
            else:
                col=2
                target_position[2]+=0.041
                target_position[0]=-0.225


            
            
    # To unexpectedly stop in case of infinite loop or error        
    except KeyboardInterrupt:
        print("Exiting loop.")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        print("Stopping simulation.")
        sim.stopSimulation()


# Calling the main function



main_loop()
#Robot_saying_HI()

# Stop simulation
sim.stopSimulation()
print("Simulation stopped.")
