# -*- coding: utf-8 -*-
# Author: Adam Rojik

##### Imports
import mujoco_py as mjp
import csv
import time, datetime
import numpy as np
import getopt, sys, os

##### Setup
try:
   opts, args = getopt.getopt(sys.argv[1:], "hvm:s:e:",["help", "viewer", "model=", "simstop=", "simep="])
except getopt.GetoptError as e:
   print(e)
   sys.exit(2)

##### Default settings
modelPath = "model/mjmodel.xml"
output = {
   'path':"output/"+time.strftime("%y-%m-%d_%H-%M-%S")+"-"
}
loadViewer = False
simulationStop = 300
simulationEpisodes = 1

for opt,arg in opts:
   if opt == '-m' or opt == '--model':
      modelPath = arg
      print("Using custom model path.")
   elif opt == '-s' or opt == '--simstop':
      simulationStop = int(arg)
      print("Using "+str(simulationStop)+" steps per episode.")
   elif opt == '-e' or opt == '--simep':
      simulationEpisodes = int(arg)
      print("Using "+str(simulationEpisodes)+" episodes per simulation.")
   elif opt == '-v' or opt == '--viewer':
      print("Will load viewer.")
      loadViewer = True
   elif opt == '-h' or opt == '--help':
      print("Arguments:")
      print("--help -h (this)")
      print("--viewer -v (activate MjViwer, disabled by default)")
      print("--model -m "+modelPath+" (set custom model path)")
      print("--simstop -s "+str(simulationStop)+" (steps per episode)")
      print("--simep -e "+str(simulationEpisodes)+" (episodes per simulation)")
      sys.exit(2)


def moveJoints(sim, ctrl, steps = 0):
   """Sets actuator goals to ctrl in simulation. Possible to also add X steps.
   
   Keyword arguments:
   sim -- MjSim: Simulation from MuJoCo.
   ctrl -- dict: {actuator:value,...} actuator names and their goal values.
   steps -- int(0): calls sim.step() `steps` times"""
   for actuator in ctrl:
      sim.data.ctrl[sim.model.actuator_name2id(actuator.replace("joint","actuator"))] = ctrl[actuator]

   for i in range(steps):
      sim.step()
   sim.forward()

def getJointDataFromActuators(sim, actuators, data="joint"):
   """Loads joint values for given actuators. Joints are expected have same name as `actuator`s, but replacing it with `data`.
   
   Keyword arguments:
   sim -- MjSim: Simulation from MuJoCo.
   actuators -- list: actuator names.
   data -- string(joint): actuator data-name.
   :return -- dict: {actuator:value,...}.
   """
   jointData = {}
   for actuator in actuators:
      dataName = actuator.replace("actuator", data)
      jointData[actuator] = sim.data.get_sensor(dataName)
   return jointData

def moveJointsEndSame(sim, ctrl, precision=8, iterationsLimit=float("inf")):
   """Finds joint with minimal velocity, predicts no. iterations and slows all other joints accordingly.
   May oscilate on colision as velocity is calculated from next simulation step. Tweakable with `precision`.

   WIP, does not work correctly.
   
   Keyword arguments:
   sim -- MjSim: Simulation from MuJoCo.
   ctrl -- dict: {actuator:value,...} actuator names and their goal values.
   precision -- int(8): threshold for smooth movement.
   iterationsLimit -- float(inf): set maximal division of delta.
   """
   actuators = list(ctrl.keys())
   jointPositions = getJointDataFromActuators(sim, actuators)
   jointVelocity = getJointDataFromActuators(sim, actuators, "jointvel")
   jointDistance = {}
   jointTime = {}

   newCtrl = dict(ctrl)
   time = 0
   targetActuator = ""

   for actuator in actuators:
      jointDistance[actuator] = ctrl[actuator] - jointPositions[actuator]

      if abs(jointVelocity[actuator]) < 10**(-precision) or abs(jointDistance[actuator]) < 10**(-precision):
         del ctrl[actuator]
         continue

      jointTime[actuator] = jointDistance[actuator]/jointVelocity[actuator]

      if abs(jointTime[actuator]) > time:
         time = abs(jointTime[actuator])
         targetActuator = actuator
   
   if targetActuator != "":
      del ctrl[targetActuator]
   
   for actuator in ctrl:
      delta = jointDistance[actuator]/time
      newCtrl[actuator] = jointPositions[actuator] + delta
   print("newCtrl", newCtrl)

   moveJoints(sim, newCtrl)



### Main code
np.random.seed(0) # for tests - always same results
totalSimulationTime = 0

# Load model
xml = ""
with open(modelPath, "r") as input:
   newPath = os.path.realpath(input.name).split("/")
   newPath[-1] = ""
   xml = input.read().replace('file="', 'file="'+"/".join(newPath))

# Load all output files
with open(output['path']+"simulation.csv", "w+") as output['simulation']:
   output['simulationCSV'] = csv.writer(output['simulation'])
   with open(output['path']+"dataset.csv", "w+") as output['dataset']:
      output['datasetCSV'] = csv.writer(output['dataset'])

      # Setup model
      model = mjp.load_model_from_xml(xml)
      sim = mjp.MjSim(model)
      
      objectInitPosition = np.array(sim.data.get_joint_qpos("SIM"))
      sim.step()
      timestep = sim.data.time
      sim.reset()
      
      touchedTimes = 0
      
      # Episodes
      for episode in range(simulationEpisodes):
         startTime = time.time()

         # Generate random position for object
         c = 0.005
         position = np.round((objectInitPosition[:3] + np.random.uniform(low=-c, high=c, size=3)), 4)
         
         # Random uniform quaternion (http://planning.cs.uiuc.edu/node198.html)
         S = np.random.uniform(low=0.0001, high=0.9999, size=3)
         objectQpos = np.concatenate([position, [np.sqrt(1-S[0])*np.sin(2*np.pi*S[1]), np.sqrt(1-S[0])*np.cos(2*np.pi*S[1]), np.sqrt(S[0])*np.sin(2*np.pi*S[2]), np.sqrt(S[0])*np.cos(2*np.pi*S[2])]])
         sim.data.set_joint_qpos("SIM", objectQpos)
         sim.forward()
         
         if episode == 0:
            sensorNames = [sim.model.sensor_names[i] for i in range(len(sim.model.sensor_names)) for j in range(sim.model.sensor_dim[i])]
            output['datasetCSV'].writerow(sensorNames)
      
         # Setup robot joints
         force = 0
         initPos = {"iiwa_joint_2": 1.3, "iiwa_joint_4": -1.57, "iiwa_joint_6": -1.35, "bh_j12_joint":1.1, "bh_j13_joint":0, "bh_j22_joint":1.1, "bh_j23_joint":0, "bh_j32_joint":1.1, "bh_j33_joint":0}
         for jointName in initPos:
            sim.data.set_joint_qpos(jointName, initPos[jointName])
            sim.data.ctrl[sim.model.actuator_name2id(jointName.replace("joint", "actuator"))] = initPos[jointName]
         sim.forward()

         if loadViewer:
            if episode == 0:
               viewer = mjp.MjViewer(sim)
            viewer.render()

         force = 1
         goalPos = {"bh_j12_joint":force, "bh_j22_joint":force, "bh_j32_joint":force}
         moveJoints(sim, goalPos)

         # Steps
         simulationSteps = 0

         isLogging = False
         hasTouched = False
         while simulationSteps < simulationStop:
            # very slow, will need optimalization (from 0.3 real-time to 1.25)
#            moveJointsEndSame(sim, {"iiwa_actuator_2":1.2, "iiwa_actuator_4":-1.4, "iiwa_actuator_6":-1.}, iterationsLimit=8)
#            moveJointsEndSame(sim, {"iiwa_actuator_2":0, "iiwa_actuator_4":0, "iiwa_actuator_6":0}, iterationsLimit=8)

            
            if not hasTouched:
               for i in range(len(sim.model.sensor_type)):
                  if sim.model.sensor_type[i] == 0 and abs(sim.data.sensordata[i]) > 10*sim.model.sensor_noise[i]:
                     simulationSteps = int(simulationStop - 2./timestep) - 1
                     #print("Activation on ", sim.model.sensor_names[i], simulationSteps)
                     hasTouched = True
            else:
               if simulationSteps == simulationStop - 1:
                  isLogging = True

            
            # Data logging
            if isLogging:
               output['datasetCSV'].writerow(sim.data.sensordata)
               output['simulationCSV'].writerow(sim.data.qpos)
 
            if loadViewer:
               viewer.render()
            sim.step()
            simulationSteps += 1

         touchedTimes += hasTouched
         simulationTime = time.time() - startTime
         totalSimulationTime += simulationTime
         secondsLeft = (simulationEpisodes-episode-1)/(episode+1)*totalSimulationTime
         if episode == 0:
            print("Simulating "+str(round(sim.data.time,2))+" seconds per episode. First episode took " + str(round(totalSimulationTime,2)) + " seconds.")
         if episode + 1 != simulationEpisodes:
           print("Ep."+str(episode)+", "+str(touchedTimes)+" touches. Estimating "+str(round(secondsLeft,2)) + " seconds left, at "+(datetime.datetime.now()+datetime.timedelta(seconds=secondsLeft)).strftime("%Y-%m-%d %H:%M:%S")+".")
         else:
            print("Simulation done, took "+str(round(totalSimulationTime, 2))+" seconds.")

# Keep rendering after simulation ended.
while loadViewer:
   viewer.render()