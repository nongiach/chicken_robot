import config

def main(opmode, clientID, motors, roboth, pos_init, toolbox, savedrobot=None):
  # Get initial Popluation from File or from a new population
  pop = savedrobot
  for individual in pop:
    start_simulation(clientID)
    for angle in individual:
      move_motor_angle(clientID, angle.motor, angle)
      time.sleep(1)
    stop_simulation(clientID)
    dist = put_value_on_position(pos_init, new_pos)
    print("Distance Made by this individual %s" % dist)

  
def preload_main(savedrobot=None):
  opmode = vrep.simx_opmode_oneshot_wait
  clientID, motors, roboth = init_simulation()
  stop_simulation(clientID, opmode)
  pos_init = get_position(clientID, roboth, opmode=opmode)
  toolbox = init_deap_lib(clientID, roboth, motors, pos_init)
  savedrobot = load_population(savedrobot=savedrobot)
  return main(opmode, clientID, motors, roboth, pos_init, toolbox, savedrobot=savedrobot)
  
if __name__ == '__main__':
  import sys
  savedrobot=config.file_save
  if len(sys.argv) >= 2:
    savedrobot = sys.argv[1]
  preload_main(savedrobot=savedrobot)

