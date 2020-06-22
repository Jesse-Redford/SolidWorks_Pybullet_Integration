# Installing URDF exporter for Solidworks, Windows 10
Assuming you already have solidworks verison 2016 or greater already installed, click on the link below
https://github.com/ros/solidworks_urdf_exporter/releases/tag/1.5.1

Download the Source Code (zip) file
https://github.com/ros/solidworks_urdf_exporter/archive/1.5.1.zip

Once downloaded unzip the folder and navigate to 
--> solidworks_urdf_exporter-1.5.1
 --> INSTALL
  --> Output
  
In the Output folder you should see the file sw2urdfSetup.exe, double click to install it.

After installing open solidworks and create a new partfile, click the file tab and scroll down, you should see an option (Export as URDF).
If you do not see this option, you will need to try and reinstall the source code.


# Installing PyBullet 
pybullet is a simple python interface to the physics engine "Bullet", this allows you to import robots into a simulator and controll them. pybullet can load kinematic descriptions of robots or other objects from URDF files.

- pip install pybullet


# Run test scripts
python bullet_test.py - reference https://alexanderfabisch.github.io/pybullet.html 
python test_position_controller.py
python test_velocity_controller.py


# Example of exporting URDF file from solidworks, how I created (Assem_test8 folder)
![Exporting URDF file]https://github.com/Jesse-Redford/Deep_Robot_Development/blob/master/Test_Files/creating_URDF_file.gif?raw=true

# python test_position_controller.py
![python test_position_controller.py]https://github.com/Jesse-Redford/Deep_Robot_Development/blob/master/Test_Files/test_position_controller.gif?raw=true

# python test_velocity_controller.py
![python test_velocity_controller.py]https://github.com/Jesse-Redford/Deep_Robot_Development/blob/master/Test_Files/test_velocity_controller.gif?raw=true




# Modeling your own robot in Solidworks



# Exporting your Solidworks Assembly as a URDF file.
Once you have created your robot parts and joined them in an assemblie file, we need to set up the parameters for the URDF file.




# Importing and interperating the URDF file in python

    import os
    import pybullet
    import pybullet_data

    pybullet.connect(pybullet.GUI)
    pybullet.resetSimulation()
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Load the URDF file of your robot.
    robot = pybullet.loadURDF(r'C:\Users\Jesse\Desktop\PyFluid\Assem_test8\urdf\Assem_test8.urdf',[0,0,0],useFixedBase=1)

If the file loads successfully, we can move on to the next step of checking that our robot was configured properly. Define the function below in your python scripy. This will print out the number of joints the system has and details related to each of them. Test function and read what gets printed out to the terminal.

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
      
    # get joint info about robot
    get_joint_info(robot)
      
The names of each joint should match the names that were given in Section "Exporting your Solidworks Assembly as a URDF file"

# Creating On Screen Joint Position/Velosity Controllers 

Lets first create a position controller for one of the systems joints. The inputs to the function will be the joint_index of the joint we wish to create the controller for, followed by an upper/lower position limit in radians, and an inital position we want the joint to start in when we run the simulation.


    def create_joint_position_controller(joint_index=0,lower_limit=-3.14,upper_limit=3.14,inital_position=0):
       # get name of joint, to create on screen label
       joint_info = pybullet.getJointInfo(robot, joint_index)
       #define joint paramters for controller 
       joint_parameters = pybullet.addUserDebugParameter(paramName=str(joint_info[1])+'PC', rangeMin=lower_limit, rangeMax =upper_limit,          startValue=inital_position)
       # return array containing joint index and paramters
       # pass the returned array to activate_position_contoller in the main loop of your script
       return [ joint_index,joint_parameters]
       
       
Next we will create a velosity controller for the same joint, The inputs to the function will be the joint_index of the joint we wish to create the controller for, followed by an upper/lower velosity limit in radians/sec, and an inital velocity we want the joint to have a the start of the simulation.
       
    def create_joint_velocity_controller(joint_index=0,lower_limit=-10,upper_limit=10,inital_velosity=0):
       # get name of joint, to create on screen label
       joint_info = pybullet.getJointInfo(robot, joint_index)
        #define joint paramters for controller 
       joint_parameters = pybullet.addUserDebugParameter(paramName=str(joint_info[1])+'VC', rangeMin=lower_limit, rangeMax =upper_limit,        startValue=inital_velosity)
       # return array containing joint index and paramters
        # pass this to activate_position_contoller
       return [ joint_index,joint_parameters]



Now create a two variables to store the returned array from the functions above, I use "PC" to indicate Position Controll and "VC" to indicate a velosity controller.

    Joint1_PC = create_joint_position_controller(joint_index =0,lower_limit=-3.14,upper_limit=3.14,inital_position=0)
    
    Joint1_VC = create_joint_velocity_controller(joint_index =0,lower_limit=-10,upper_limit=10,inital_velosity=0)
    
    
Next we will create functions which will active our controllers while the simulation is running. The functions will accept the variable defined above ie: Joint1_PC and Joint1_VC , and create a sidebar slider on the pybullet GUI which we can use to controll the robots joint. The function will then return the joints current position and velocity. 

    def activate_position_controller(joint_parameters):
        joint_index = joint_parameters[0]
       angle = joint_parameters[1]
       user_angle = pybullet.readUserDebugParameter(angle)
       pybullet.setJointMotorControl2(robot, joint_index, pybullet.POSITION_CONTROL,targetPosition= user_angle)
       joint_info = pybullet.getJointState(robot,joint_index)
       joint_position = joint_info[0] 
       joint_velosity = joint_info[1]
       return joint_position,joint_velosity


    def activate_velocity_controller(joint_parameters):
       joint_index = joint_parameters[0]
       velosity = joint_parameters[1]
       user_velocity = pybullet.readUserDebugParameter(velosity)
       pybullet.setJointMotorControl2(robot, joint_index, pybullet.VELOCITY_CONTROL,targetVelocity= user_velocity)
       joint_info = pybullet.getJointState(robot,joint_index)
       joint_position = joint_info[0] 
       joint_velosity = joint_info[1]
       return joint_position,joint_velosity

Now lets test the postion/velocity controllers, by starting a simulation and testing each of them.
    
     pybullet.setGravity(0, 0, -9.81) # define x,y,z gravity constants
     pybullet.setTimeStep(0.0001)
    
     while True:
    
      """ Note you can only activate either a postion or velosity controller, cannot use both simutaneously"""
    
     #activate Joint1 for on screen control and get position and velosity readings
    
     joint1_position,joint1_velosity = activate_position_controller(Joint1_PC)
     
     # comment out the line above and uncomment the below to test the velocity_controller
     # joint1_position,joint1_velocity = activate_velocity_controller(Joint1_VC)
    
     print(joint1_position,joint1_velocity)
    
     pybullet.stepSimulation()
    
    pybullet.disconnect()


    
    
