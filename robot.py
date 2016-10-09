
import vrep
from math import radians
from random import randint
from time import sleep
from collections import namedtuple
from sys import exit
from functools import partial


# Move = namedtuple("Move", ["Wrist", "Elbow", "Shoulder"])
# nbrMove = 7
# Individu = [ Move * nbrMove ]
clientID = None
handles_robots = []
initial_position = None

def init_simulator(nbr_robots=3):
    global clientID, initial_position, handles_robots
    clientID = init_vrep()
    opmode = vrep.simx_opmode_oneshot_wait
    vrep.simxStopSimulation(clientID, opmode)
    handles = getRobotObject(clientID)
    wristHandle, elbowHandle, shoulderHandle, robotHandle = handles
    handles_robots.append(handles)
    initial_position = get_position(clientID, robotHandle)
    initial_position[0] = 0.0
    gprint("initial_position")
    for i in range(1, nbr_robots):
        handles = getRobotObject(clientID, n=i)
        handles_robots.append(handles)
    for n, handles in enumerate(handles_robots):
        wristHandle, elbowHandle, shoulderHandle, robotHandle = handles
        # vrep.simxSetObjectPosition(clientID, robotHandle, -1, [0.0, 0.25 * (n) - 2.5, 0.075], opmode)
        vrep.simxSetObjectPosition(clientID, robotHandle, -1, [0.0, 0.25 * (n) - 1.0, 0.075], opmode)
        bprint("n")
    input("configure camera and press Enter")

def get_position(clientID, handler, opmode=vrep.simx_opmode_oneshot_wait):
    # stop_simulation()
    ret, pos = vrep.simxGetObjectPosition(clientID, handler, -1, opmode)
    return pos

def init_vrep():
    print("init_vrep")
    # Close eventual old connections
    vrep.simxFinish(-1)
    # Connect to V-REP remote server
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    assert clientID != -1, "connection to vrep failed (be sure vrep is started first)"
    return clientID


def getRobotObject(clientID, n=0):
    global handles_robots, initial_position
    # Communication operating mode with the remote API : wait for its answer before continuing (blocking mode)
    # http://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm
    opmode = vrep.simx_opmode_oneshot_wait
    vrep_get = partial(vrep.simxGetObjectHandle,
                       clientID=clientID, operationMode=opmode)

    # Try to retrieve motors and robot handlers
    # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxGetObjectHandle
    index = "" if n == 0 else "#{}".format(n - 1)
    print("get 2W1A" + index)
    ret_test, robotHandle = vrep_get(objectName="2W1A" + index)
    if ret_test != 0 and handles_robots:
        vrep.simxCopyPasteObjects(clientID, [handles_robots[0][-1]], opmode)

    ret1, wristHandle = vrep_get(objectName="WristMotor" + index)
    ret2, elbowHandle = vrep_get(objectName="ElbowMotor" + index)
    ret3, shoulderHandle = vrep_get(objectName="ShoulderMotor" + index)
    ret4, robotHandle = vrep_get(objectName="2W1A" + index)
    bprint("ret1 ret2 ret3 ret4")
    if ret1 == ret2 == ret3 == ret4 == 0:
        if initial_position:
            gprint("initial_position")
        #     print("set position")
        #     robotPos = list(initial_position)
        #     robotPos[1] += 0.5 * (n + 1)

        return wristHandle, elbowHandle, shoulderHandle, robotHandle
    raise("vrep_get error (be sure the chicken robot is there) 2W1A0{}".format(index))

def stop_simulation(clientID=clientID, opmode=vrep.simx_opmode_oneshot_wait):
    vrep.simxStopSimulation(clientID, opmode)
    print("--simulation stopped--")
    sleep(1)

def start_simulation(clientID=clientID, opmode=vrep.simx_opmode_oneshot_wait):
    vrep.simxStartSimulation(clientID, opmode)
    print("--simulation started--")

# def move_motor_angle(clientID, motor, angle, opmode=vrep.simx_opmode_oneshot_wait):
def move_motor_angle(clientID, motor, angle, opmode=vrep.simx_opmode_oneshot):
    vrep.simxSetJointTargetPosition(clientID, motor, radians(angle), opmode)

# def manathan_distance(a, b):
    #     return sum([ abs(ia - ib) for ia, ib in zip(a, b) ])

def x_manathan_distance(a, b):
    # return b[0] - a[0] # the max of this is to go forward
    return a[0] - b[0] # the max of this is to go backward

def eval_population_split(population):
    global handles_robots
    print("eval_population_split({}) nbr_robots = {}".format(len(population), len(handles_robots)))
    distances = []
    for i in range(0, len(population), len(handles_robots)):
        distances += eval_population(population[i:i + len(handles_robots)])
    print(len(distances), distances)
    return distances

def eval_population(population):
    global clientID, handles_robots, initial_position
    # wristHandle, elbowHandle, shoulderHandle, robotHandle = handles_robots[1]
    print("ipos = {}".format(initial_position))
    print("eval_population({})".format(len(population)))
    # return [ (sum(ind),) for ind in population ]
    start_simulation(clientID)
    # replay twice so we generate a cyclic pattern
    # 2 * 6 genes for how to start
    for i in range(0, 12, 2):
        for (wrist, elbow, shoulder, robot), individual in zip(handles_robots, population):
            move_motor_angle(clientID, wrist, individual[i + 0])
            move_motor_angle(clientID, elbow, individual[i + 1])
            move_motor_angle(clientID, shoulder, 150)
            sleep(0.3)

    # 2 * 8 genes for cylic move
    for _ in range(3):
        for i in range(12, len(population[0]), 2):
            for (wrist, elbow, shoulder, robot), individual in zip(handles_robots, population):
                move_motor_angle(clientID, wrist, individual[i + 0])
                move_motor_angle(clientID, elbow, individual[i + 1])
            sleep(0.3)

    distances = []
    for (wrist, elbow, shoulder, robot), individual in zip(handles_robots, population):
        end_position = get_position(clientID, robot)
        distance = x_manathan_distance(initial_position, end_position)
        distances.append((distance * 10 + 100.0,))
        bprint("end_position distance")

    stop_simulation(clientID)
    return distances

# def eval_robot(individual):
#     global clientID, handles_robots, initial_position
#     wristHandle, elbowHandle, shoulderHandle, robotHandle = handles_robots[1]
#     print("ipos = {}".format(initial_position))
#     bprint("individual")
#     start_simulation(clientID)
#     for i in range(0, len(individual), 3):
#         move_motor_angle(clientID, wristHandle, individual[i + 0])
#         move_motor_angle(clientID, elbowHandle, individual[i + 1])
#         move_motor_angle(clientID, shoulderHandle, individual[i + 2])
#         sleep(0.4)
#
#     end_position = get_position(clientID, robotHandle)
#     distance = x_manathan_distance(initial_position, end_position)
#     bprint("end_position distance")
#     stop_simulation(clientID)
#     return distance

from inspect import currentframe
def bprint(fmt):
    fmt = ', '.join([ "{0} = {{{0}}}".format(word)
                     for word in fmt.split(' ')])
    print(fmt.format_map(currentframe().f_back.f_locals))

def gprint(fmt):
    fmt = ', '.join([ "{0} = {{{0}}}".format(word)
                     for word in fmt.split(' ')])
    print(fmt.format_map(globals()))

if __name__ == "__main__":
    main()
