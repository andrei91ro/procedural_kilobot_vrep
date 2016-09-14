import logging
import colorlog # colors log output
from vrep_bridge import vrep_bridge # for getState, setState
#from lulu_pcol_sim import sim
import sys # for argv, stdout
#from copy import deepcopy # for deepcopy (value not reference as = does for objects)
from enum import IntEnum # for enumerations (enum from C)
#import random # for stochastic chosing of actions
import time # for time.perf_counter()

class MoveState(IntEnum):

    """Enumeration of possible programmed movement states"""

    st1_stop_led_off     = 1,

    st2_straight_green   = 2,

    st3_left_red         = 3,
    st4_left_red         = 4,

    st5_straight_green   = 5,

    st6_right_blue       = 6,
    st7_right_blue       = 7,

    st8_straight_green   = 8,
    st9_straight_green   = 9,

    st10_stop_white      = 10

# end class States

timer_interval = 2

class Kilobot():

    """Class used to store the state of a Kilobot robot for use in a controller that used Pcolonies."""

    def __init__(self, uid):
        self.uid = uid # the unique id of the robot
        self.responseTimer = 0 # used to store the time at sensor data receive and compute the time needed by the controller to give a response (T_response - responseTimer)
        self.raw_input_state = {"uid": uid, "light": [], "distances":{}} # dictionary of raw sensor values
        self.output_state = {
                "motion" : vrep_bridge.Motion.stop, # motion (vrep_bridge.Motion) motion type
                "led_rgb" : [0, 0, 0] # light [r, g, b] with values between 0-2
                } # dictionary of output states
        self.distances = {} # dictionary of the most recent distance measurements = {robot_uid: distance}
        self.distances_prev = {} # dictionary of previous distance measurements = {robot_uid: distance}
        self.light = -1 # current light intensity
        self.light_prev = -1 # previous light intensity
        self.timexp_moveTimer = time.perf_counter() + timer_interval  # timer.perf_timer() time in the future for the next action
        self.moveState = MoveState.st1_stop_led_off # the current move state of the programmed movement algorithm
    # end __init__()

    def procInputModule(self, paramLightThreshold = 20, paramDistanceThreshold = 55):
        """Process raw_state info received from sensors and trigger the corresponding event

        :paramLightThreshold: light threshold value used to classify a light sensor reading as low or high
        :paramDistanceThreshold: distance threshold value used to classify a distance from a robot as small or big"""

        for uid, d in self.raw_input_state["distances"].items():
            # if I already have a distance from this robot
            if (uid in self.distances):
                # update previous distances
                self.distances_prev[uid] = self.distances[uid]
                # store the new distance
                self.distances[uid] = d
            # if this is the first time I receive a measurement from this robot
            else:
                self.distances[uid] = self.distances_prev[uid] = d

        # if this is not the first light intensity measurement
        if (self.light != -1):
            self.light_prev = self.light
            self.light = self.raw_input_state["light"]
        # this is the first time I receive a light intensity measurement
        else:
            self.light = self.light_prev = self.raw_input_state["light"]

        ## if no robot was closer than the threshold
        #if (dist_big):
            ##self.colony.agents['msg_distance'].obj["B_all"] = 1 # distance big
            #self.fsm.current_event = EventType.all_neighbors_distant # distance big
        #else:
            ##self.colony.agents['msg_distance'].obj["S_all"] = 1 # distance small (at least one robot is close)
            #self.fsm.current_event = EventType.neighbor_close # distance small (at least one robot is close)

    # end procInputModule()

# end class Kilobot

##########################################################################
#   MAIN
formatter = colorlog.ColoredFormatter(
        "%(log_color)s%(levelname)-8s %(message)s %(reset)s",
        datefmt=None,
        reset=True,
        log_colors={
                'DEBUG':    'cyan',
                'INFO':     'green',
                'WARNING':  'yellow',
                'ERROR':    'red',
                'CRITICAL': 'red,bg_white',
        },
        secondary_log_colors={},
        style='%'
)
if ('--debug' in sys.argv):
    colorlog.basicConfig(stream = sys.stdout, level = logging.DEBUG)
else:
    colorlog.basicConfig(stream = sys.stdout, level = logging.INFO) # default log level

if ('--defaultOutput' in sys.argv):
    defaultOutput = True
else:
    defaultOutput = False

stream = colorlog.root.handlers[0]
stream.setFormatter(formatter);

# make link with v-rep
bridge = vrep_bridge.VrepBridge()

#if (type(pObj) == sim.Pcolony):
    #robot = Kilobot(0, pObj)
#else:
# array of Kilobot objects
robots = []
# used to determine how many robots have been set up up so far with this colony name
# so that the first one gets the original colony and the others get a clone
#config.nrConfiguredRobotsWithColony = {colonyName: 0 for colonyName in config.nrAsignedRobotsPerColony.keys()}

# spawn n-1 robots because 1 is already in the scene and is copied
#bridge.spawnRobots(nr = config.nrRobots - 1, spawnType = config.spawnType)

## contruct the array of robots each controlled by its own FSM
#for i in range(config.nrRobots):
for i in range(1):
    robots.append(Kilobot(0))
    #robots[-1].fsm.handles = [robots[-1].handle_default, # EventType.no_event
    #robots[-1].handle_neighbor_close, # EventType.neighbor_close
    #robots[-1].handle_all_neighbors_distant] # EventType.all_neighbors_distant

#end for clones

#print("\n Robot - Pcolony association table:")
#print("robot_id    colony_name\n")
#for i in range(config.nrRobots):
    #print("robot_%d    %s" % (i, config.robotColony[i]))
#print("\n")

# open a csv file for response time logging for each robot
csvFile = open("responseTimer.csv", mode = 'w')

simStepNr = 0
# the next distances reinitialization will take place after config.clearDistancesStepNr steps from now
#nextClearStepNr = simStepNr + config.clearDistancesStepNr
while (True):
    #print("\n")

    for robot in robots:
        # request current sensor data of the robot from V-REP only if an input request was made
        #if (robot.wasInputRequestMade()):
        #robot.raw_input_state = bridge.getState(robot.uid)
        if (time.perf_counter() >= robot.timexp_moveTimer and robot.timexp_moveTimer != 0):
            # store the current time after receving new sensor data
            # from Python DOCS: Return the value (in fractional seconds) of a performance counter, i.e. a clock with the highest available resolution to measure a short duration. It does include time elapsed during sleep and is system-wide. The reference point of the returned value is undefined, so that only the difference between the results of consecutive calls is valid.
            robot.responseTimer = time.perf_counter()

            # increment the state counter
            robot.moveState += 1

            logging.warning("robot.moveState = %d" % robot.moveState)

            if (robot.moveState == MoveState.st2_straight_green or
                    robot.moveState == MoveState.st5_straight_green or
                    robot.moveState == MoveState.st8_straight_green or
                    robot.moveState == MoveState.st9_straight_green):

                robot.output_state["motion"] = vrep_bridge.Motion.forward
                robot.output_state["led_rgb"] = vrep_bridge.Led_rgb.green

            elif (robot.moveState == MoveState.st3_left_red or
                    robot.moveState == MoveState.st4_left_red):

                robot.output_state["motion"] = vrep_bridge.Motion.left
                robot.output_state["led_rgb"] = vrep_bridge.Led_rgb.red

            elif (robot.moveState == MoveState.st6_right_blue or
                    robot.moveState == MoveState.st7_right_blue):

                robot.output_state["motion"] = vrep_bridge.Motion.right
                robot.output_state["led_rgb"] = vrep_bridge.Led_rgb.blue

            elif (robot.moveState == MoveState.st10_stop_white):
                robot.output_state["motion"] = vrep_bridge.Motion.stop
                robot.output_state["led_rgb"] = vrep_bridge.Led_rgb.white

            # schedule a new action
            robot.timexp_moveTimer = time.perf_counter() + timer_interval
        #robot.procInputModule()

    # process output module for all robots
    for robot in robots:
        #robot.procOutputModule(defaultOutput)

        # set the output state of the robot only if output command objects are present in the output modules
        if (robot.responseTimer != 0):
            robot.responseTimer = time.perf_counter() - robot.responseTimer
            # for uid = 3, nr = 5
            # print("0, " * uid + "%d, " % 2 + "0, " * (nr - uid - 1))
            # 0, 0, 0, 2, 0,
            #csvFile.write("0, " * (robot.uid) + "%f, " % robot.responseTimer + "0," * (config.nrRobots - robot.uid - 1) + "\n")
            csvFile.write("%f,\n" % robot.responseTimer)
            robot.responseTimer = 0

            bridge.setState(robot.uid, robot.output_state["motion"], robot.output_state["led_rgb"])

    simStepNr += 1
    if (robot.moveState >= MoveState.st10_stop_white):
        break
# end while

# close the connection with v-rep
bridge.close()
