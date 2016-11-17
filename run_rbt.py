import config
from base import get_robot_object, start_simulation, stop_simulation, get_position, \
  init_deap_lib, load_population, Gene, move_motor_angle, put_value_on_position
import vrep
import time

def main(opmode, clientID, motors, roboth, pos_init, toolbox, savedrobot=None):
  # Get initial Popluation from File or from a new population
  pop = savedrobot
  for individual in pop:
    start_simulation(clientID)
    for angle in individual:
      move_motor_angle(clientID, angle.motor, angle)
      time.sleep(1)
    stop_simulation(clientID)
    dist = put_value_on_position(pos_init, get_position(clientID, roboth))
    print("Distance Made by this individual %s" % dist)

def init_simulation(opmode = vrep.simx_opmode_oneshot_wait):
  # Close eventual old connections
  vrep.simxFinish(-1)
  # Connect to V-REP remote server              # Args could be set to False if real remote
  clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

  pos_inits =  []
  
  if clientID == -1:
    raise Exception('Not Connected to remote API server')

  stop_simulation(clientID, opmode)
  handlers = get_robot_object(clientID)
  pos_inits = get_position(clientID, handlers[-1])
  return clientID, handlers, pos_inits

    
def preload_main(savedrobot):
  opmode = vrep.simx_opmode_oneshot_wait
  clientID, motors, pos_init = init_simulation()
  stop_simulation(clientID, opmode)
#  pos_init = get_position(clientID, motors[-1], opmode=opmode)
  toolbox = init_deap_lib(clientID, motors, pos_init)
  savedrobot = load_population(toolbox, savedrobot=savedrobot)
  print("Robot To Play %s" % savedrobot) 
  return main(opmode, clientID, motors[:-1], motors[-1], pos_init, toolbox, savedrobot=savedrobot)
  
if __name__ == '__main__':
  import sys
  savedrobot=config.filename_to_save_best
  if len(sys.argv) >= 2:
    savedrobot = sys.argv[1]
  preload_main(savedrobot=savedrobot)

