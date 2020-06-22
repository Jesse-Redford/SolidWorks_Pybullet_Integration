import pybullet
import pybullet_data

pybullet.connect(pybullet.GUI)
pybullet.resetSimulation()
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

def get_joint_info(robot):
    print('The system has', pybullet.getNumJoints(robot), 'joints')
    num_joints = pybullet.getNumJoints(robot)
    for i in range(num_joints):
        joint_info = pybullet.getJointInfo(robot, i)
        print('Joint number',i)
        print('-------------------------------------')
        print('Joint Index:',joint_info[0])
        print('Joint Name:',joint_info[1])
        print('Joint misc:',joint_info[2:])
        print('-------------------------------------')
    return

def create_joint_velocity_controller(joint_index=0,lower_limit=-10,upper_limit=10,inital_velosity=0):
    joint_info = pybullet.getJointInfo(robot, joint_index) # get name of joint, to create on screen label
    joint_parameters = pybullet.addUserDebugParameter(paramName=str(joint_info[1])+'VC', rangeMin=lower_limit, rangeMax =upper_limit, startValue=inital_velosity)
    # pass the returned array to activate_position_contoller in the main loop of your script
    return [ joint_index,joint_parameters]
    
def activate_velocity_controller(joint_parameters):
    joint_index = joint_parameters[0]
    velosity = joint_parameters[1]
    user_velocity = pybullet.readUserDebugParameter(velosity)
    pybullet.setJointMotorControl2(robot, joint_index, pybullet.VELOCITY_CONTROL,targetVelocity= user_velocity)
    joint_info = pybullet.getJointState(robot,joint_index)
    joint_position = joint_info[0] 
    joint_velosity = joint_info[1]
    return joint_position,joint_velosity
    
    
# Load robot URDF file created in solidworks
robot = pybullet.loadURDF(r'C:\Users\Jesse\Desktop\PyFluid\Assem_test8\urdf\Assem_test8.urdf',[0,0,0],useFixedBase=1)

# get joint info about robot
get_joint_info(robot)

# Create Position controller for a particular joint
Joint1_VC = create_joint_velocity_controller(joint_index =0,lower_limit=-3.14,upper_limit=3.14,inital_velosity=0)

# Set up simulation parameters
pybullet.setGravity(0, 0, -9.81)
pybullet.setTimeStep(0.0001)

# Start simulation activate Joint1 for on screen control and get position and velosity readings
while True:
    joint1_position,joint1_velocity = activate_velocity_controller(Joint1_VC)
    print(joint1_position,joint1_velocity )
    pybullet.stepSimulation()
pybullet.disconnect()



