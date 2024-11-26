import pybullet as p
import time
import math
import numpy as np
import random

############################################### Environment Setup ####################################################
p.connect(p.GUI)

p.resetSimulation()

p.setGravity(0, 0, -10)
useRealTimeSim = 0

p.setRealTimeSimulation(useRealTimeSim)  # either this

# load plane
track = p.loadURDF("data/plane/plane.urdf")
# load car
car = p.loadURDF("f10_racecar/racecar_differential.urdf", [0, 0, 0])

# load obstacles, in this projects, we used six cube as obstacles
def random_obstacles():
    np.random.seed()
    xy_position = [0, 0]
    xy_position_float = np.random.rand(2)
    x_poistion_range = np.random.randint(1, 10)
    y_poistion_range = np.random.randint(-10, -1)

    xy_position[0] = xy_position_float[0] + x_poistion_range
    xy_position[1] = xy_position_float[1] + y_poistion_range

    np.asarray(xy_position)
    position = np.append(xy_position, 0.2)
    return position

cube_1_position = random_obstacles()
cube_1 = p.loadURDF('data/cube/marble_cube.urdf',cube_1_position)

cube_2_position = random_obstacles()
cube_2 = p.loadURDF('data/cylinder/cylinder.urdf',cube_2_position)

cube_3_position = random_obstacles()
cube_3 = p.loadURDF('data/pyramid/pyramid.urdf',cube_3_position)

cube_4_position = random_obstacles()
cube_4 = p.loadURDF('data/cube/marble_cube.urdf',cube_4_position)

cube_5_position = random_obstacles()
cube_5 = p.loadURDF('data/cylinder/cylinder.urdf',cube_5_position)

cube_6_position = random_obstacles()
cube_6 = p.loadURDF('data/pyramid/pyramid.urdf',cube_6_position)

for wheel in range(p.getNumJoints(car)):
    # print("joint[", wheel, "]=", p.getJointInfo(car, wheel))
    p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
    p.getJointInfo(car, wheel)

wheels = [8, 15]
print("----------------")

# p.setJointMotorControl2(car,10,p.VELOCITY_CONTROL,targetVelocity=1,force=10)
c = p.createConstraint(car, 9, car, 11, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=1, maxForce=10000)

c = p.createConstraint(car, 10, car, 13, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 9, car, 13, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 16, car, 18, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=1, maxForce=10000)

c = p.createConstraint(car, 16, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 17, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 1, car, 18, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)
c = p.createConstraint(car, 3, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)

steering = [0, 2]

hokuyo_joint = 4

#
# targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -50, 50, 0)
# maxForceSlider = p.addUserDebugParameter("maxForce", 0, 50, 20)
# steeringSlider = p.addUserDebugParameter("steering", -1, 1, 0)

replaceLines = True
# numRays = 100
# numRays = 100
numRays = 50
rayFrom = []
rayTo = []
rayIds = []
rayHitColor = [1, 0, 0]
rayMissColor = [0, 1, 0]
rayLen = 8
rayStartLen = 0.25
for i in range(numRays):
    rayFrom.append([rayStartLen * math.sin(-0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays),
                    rayStartLen * math.cos(-0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays), 0])
    rayTo.append([rayLen * math.sin(-0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays),
                  rayLen * math.cos(-0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays), 0])
    if (replaceLines):
        rayIds.append(p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, parentObjectUniqueId=car,
                                         parentLinkIndex=hokuyo_joint))
    else:
        rayIds.append(-1)

frame = 0
lineId = p.addUserDebugLine([0, 0, 0], [0, 0, 1], [1, 0, 0])
lineId2 = p.addUserDebugLine([0, 0, 0], [0, 0, 1], [1, 0, 0])
lineId3 = p.addUserDebugLine([0, 0, 0], [0, 0, 1], [1, 0, 0])
print("lineId=", lineId)
lastTime = time.time()
lastControlTime = time.time()
lastLidarTime = time.time()

frame = 0

############################################ implement your function here ##########################################################

def control_car(carPos, turn_angle):

    # carPos -> position of the car
    # turn_angle -> turn angle of the car, return from function navigate_car
    	
   final_goal_pos = [9, -9]
   for wheel in wheels:
       p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=75, force=75)
   for steer in steering:
       #print("steeringAngle", np.degrees(turn_angle))
       p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=turn_angle)
    

def navigate_car(carPos, carOrn, sensor_readings):
    
    for i in range(numRays):
        hitObjectUid = sensor_readings[i][0]
        print("I:", i, "Hit Object id:", hitObjectUid)
        hitFraction = sensor_readings[i][2]
        print("I:", i, "Hit Fraction:", hitFraction)
        hitPosition = sensor_readings[i][3]
        print("I:", i, "Hit Position:", hitPosition)
        
        
    # carPos, carOrn -> position and orientation of the car
    # sensor_readings -> Parameter sensor_readings (line 188), returned from function rayTestBatch. function rayTestBatch is specified in pybullet quick guide
    # The length of each ray is 8
    # return turn angle
   
    final_goal_pos = [9, -9]
    # convert the quaternion to Euler 
    #(-1, -1, 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    carEuler = p.getEulerFromQuaternion(carOrn)
    carYaw = carEuler[2]
    
    AB=(carPos[0]-sensor_readings[3][0], carPos[1]-sensor_readings[3][1],carPos[2]-sensor_readings[3][2])
    AC=(carPos[0]-sensor_readings[4][0], carPos[1]-sensor_readings[4][1],carPos[2]-sensor_readings[4][2])
    
    
    
    ss=0
    s1=0
    s2=0
    for i in range(3):
        ss+=AB[i]*AC[i]
        s1+=AB[i]**2
        s2+=AC[i]**2
    
    s1sr = math.sqrt(s1)
    s2sr = math.sqrt(s2)
    
    theta = math.asin(math.radians(ss/(s1sr*s2sr)))
    return carYaw-math.degrees(theta//2)
        
#####################################################################################################################################
frame = 0
while (True):

    nowTime = time.time()
    # render Camera at 10Hertz
    if (nowTime - lastTime > .1):
        lastTime = nowTime

    nowControlTime = time.time()

    nowLidarTime = time.time()
    # lidar at 20Hz

    if (nowLidarTime - lastLidarTime > .03):

        #print("Lidar!") ray length is 8
        numThreads = 0
        sensor_readings = p.rayTestBatch(rayFrom, rayTo, numThreads, parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
        final_goal_pos = [9, -9]
        target_ori = [0, 0]
        # current car position
        carPos, carOrn = p.getBasePositionAndOrientation(car)
        maxForce = 20
        targetVelocity = 0
        steeringAngle = 0

        for i in range(numRays):
            hitObjectUid = sensor_readings[i][0]
            hitFraction = sensor_readings[i][2]
            hitPosition = sensor_readings[i][3]
            if (hitFraction == 1.):
                p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, replaceItemUniqueId=rayIds[i],
                                   parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
            else:
                localHitTo = [rayFrom[i][0] + hitFraction * (rayTo[i][0] - rayFrom[i][0]),
                              rayFrom[i][1] + hitFraction * (rayTo[i][1] - rayFrom[i][1]),
                              rayFrom[i][2] + hitFraction * (rayTo[i][2] - rayFrom[i][2])]
                # print(localHitTo)
                p.addUserDebugLine(rayFrom[i], localHitTo, rayHitColor, replaceItemUniqueId=rayIds[i],
                                   parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
        lastLidarTime = nowLidarTime
              
        ################################ Implemented functions ######################################      
        turn_angle = navigate_car(carPos,carOrn,sensor_readings)
        
        control_car(carPos,turn_angle)
        #############################################################################################
        
        if (useRealTimeSim == 0):
            frame += 1
            p.stepSimulation()
        lastControlTime = nowControlTime





