import os
import pybullet
import pybullet_data

pybullet.connect(pybullet.GUI)
pybullet.resetSimulation()
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

plane = pybullet.loadURDF("plane.urdf")
# this may take a while...
os.system("git clone https://github.com/ros-industrial/kuka_experimental.git")

robot = pybullet.loadURDF("kuka_experimental/kuka_kr210_support/urdf/kr210l150.urdf")
position, orientation = pybullet.getBasePositionAndOrientation(robot)


pybullet.getNumJoints(robot)


joint_index = 2
joint_info = pybullet.getJointInfo(robot, joint_index)
name, joint_type, lower_limit, upper_limit = joint_info[1], joint_info[2], joint_info[8], joint_info[9]

joint_positions = [j[0] for j in pybullet.getJointStates(robot, range(6))]
print(joint_positions)

world_position, world_orientation = pybullet.getLinkState(robot, 2)[:2]




pybullet.resetSimulation()
plane = pybullet.loadURDF("plane.urdf")
robot = pybullet.loadURDF("kuka_experimental/kuka_kr210_support/urdf/kr210l150.urdf",
                          [0, 0, 0], useFixedBase=1)  # use a fixed base!
pybullet.setGravity(0, 0, -9.81)
pybullet.setTimeStep(0.0001)
pybullet.setRealTimeSimulation(0)


pybullet.setJointMotorControlArray(robot, range(6), pybullet.POSITION_CONTROL,targetPositions=[0.1] * 6)
for _ in range(10000000):
    
    joint_positions = [j[0] for j in pybullet.getJointStates(robot, range(6))]
    print(joint_positions)
    pybullet.stepSimulation()
    
    
pybullet.disconnect()