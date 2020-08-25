# Solidworks Integration with Pybullet Tutorial
A development guide for building custom robot assemblies in Solidworks, converting them to URDF, importing to pybullet environment, and setting up position and speed controllers. To get started I recommend downloading the Test_Files folder. This includes a simple solidworks model and the urdf I will be referencing for this tutorial. It is also assumed that you have solidoworks and python installed on your machine. 

### Topics
- creating robot assemblies in Solidworks
- converting solidworks assembly to URDF
- importing URDF into pybullet enviroment
- Setting up GUI and configuring pybullet parameters
- retriving info about the URDF model
- creating position and speed controlers for robot joints

### Installing PyBullet 
- pybullet is a simple python interface to the physics engine "Bullet", this allows you to import robots into a simulator and controll them. pybullet can load kinematic descriptions of robots or other objects from URDF files.

      - pip install pybullet

### Installing URDF exporter for Solidworks, Windows 10

  1. Assuming you already have solidworks verison 2016 or greater already installed, click on the link below
    - https://github.com/ros/solidworks_urdf_exporter/releases/tag/1.5.1

  2. Download the Source Code (zip) file
    - https://github.com/ros/solidworks_urdf_exporter/archive/1.5.1.zip

  3. Once downloaded unzip the folder and navigate to  --> solidworks_urdf_exporter-1.5.1 --> INSTALL --> Output
  
  4. In the Output folder you should see the file sw2urdfSetup.exe, double click to install it.

  5. After installing open solidworks and create a new partfile, click the file tab and scroll down, you should see an option (Export as URDF). If you do not see this option, you will need to try and reinstall the source code.
  


### Modeling your own robot in Solidworks
- Assuming you have some basic knowledge of creating parts and assemblies in solidworks, a simple work flow for creating your robot should look something like the steps listed below. If you need help learning how to create solidworks model, additional information can be found here. 

     1. Create seperate part files for each major component of your robot.
     2. Create an assembly file.
     3. Import you part files into the assembly.
     4. Define refference geometries for each part (ie joint axises and planes)
     5. Construct your robot assembly by assign mates to your geometery references for each part .
 


### Exporting your Solidworks Assembly as a URDF file.
- Once you have created your robot parts and joined them in an assemblie file, we need to set up the parameters for the URDF file. Below is an example of what this processes looks like. how I created (Assem_test8 folder)

  1. Select urdf exporter
  2. Define main link 
  3. Create tree list of joint and assign names
  4. Verify information on the urdf exporter (joint type, position, ect..)
  5. Create URDF model
  6. Locate your model directory 

<p align="center">
<img src="https://github.com/Jesse-Redford/Deep_Robot_Development/blob/master/Test_Files/creating_URDF_file.gif" width="250" height="250"> 
</p>

### Importing URDF file into PyBullet Enviroment
- To check if your URDF model was created correctly, locate the model in the folder created by the URDF exporter
- Next create a new python script and paste in the code below.
    
      import os
      import pybullet
      import pybullet_data

      pybullet.connect(pybullet.GUI)
      pybullet.resetSimulation()
      pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
      pybullet.setGravity(0, 0, -9.81) # define x,y,z gravity constants
      pybullet.setTimeStep(0.0001)
      
      # Load the URDF file of your robot.
      robot = pybullet.loadURDF(r'C:\Users\Jesse\Desktop\PyFluid\Assem_test8\urdf\Assem_test8.urdf',[0,0,0],useFixedBase=1)
    

### Getting information about your URDF model
- If the file loads successfully, we can move on to the next step of checking that our robot was configured properly. Define the function below in your python script. This will print out the number of joints the system has and details related to each of them. Test function and read what gets printed out to the terminal.

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
        
- Now to get info about the joints of your URDF model simply pass the loaded robot to the function. The names of each joint should match the names that were given in Section "Exporting your Solidworks Assembly as a URDF file"
 
      get_joint_info(robot)
      
#### Creating Positional Joint Controllers 
- Lets first create a position controller for one of the systems joints. The inputs to the function will be the joint_index of the joint we wish to create the controller for, followed by an upper/lower position limit in radians, and an inital position we want the joint to start in when we run the simulation.

       def create_joint_position_controller(joint_index=0,lower_limit=-3.14,upper_limit=3.14,inital_position=0):
           # get name of joint, to create on screen label
           joint_info = pybullet.getJointInfo(robot, joint_index)
           #define joint paramters for controller 
           joint_parameters = pybullet.addUserDebugParameter(paramName=str(joint_info[1])+'PC', rangeMin=lower_limit, rangeMax =upper_limit,          startValue=inital_position)
           # return array containing joint index and paramters
           # pass the returned array to activate_position_contoller in the main loop of your script
           return [ joint_index,joint_parameters]
       
#### Creating Velocity Joint Controllers        
- Next we will create a velosity controller for the same joint, The inputs to the function will be the joint_index of the joint we wish to create the controller for, followed by an upper/lower velosity limit in radians/sec, and an inital velocity we want the joint to have a the start of the simulation.
       
      def create_joint_velocity_controller(joint_index=0,lower_limit=-10,upper_limit=10,inital_velosity=0):
        # get name of joint, to create on screen label
         joint_info = pybullet.getJointInfo(robot, joint_index)
          #define joint paramters for controller 
         joint_parameters = pybullet.addUserDebugParameter(paramName=str(joint_info[1])+'VC', rangeMin=lower_limit, rangeMax =upper_limit,        startValue=inital_velosity)
        # return array containing joint index and paramters
         # pass this to activate_position_contoller
        return [ joint_index,joint_parameters]

- Now create two variables to store the returned array from the functions above, I use "PC" to indicate Position Controll and "VC" to indicate a velosity controller.

      Joint1_PC = create_joint_position_controller(joint_index =0,lower_limit=-3.14,upper_limit=3.14,inital_position=0)
    
      Joint1_VC = create_joint_velocity_controller(joint_index =0,lower_limit=-10,upper_limit=10,inital_velosity=0)
    
### Activating your Joint Controllers as a GUI gooey
- Next lets create functions which will active the controllers we just made while the simulation is running. The functions will accept the variable defined above ie: Joint1_PC and Joint1_VC, and create a sidebar slider on the pybullet GUI which we can use to control the robots joint. The function will return the joints current position and velocity. 

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

- Now lets test the postion/velocity controllers, by starting a simulation and testing each of them.
    
      while True:
    
      """ Note you can only activate either a postion or velosity controller, cannot use both simutaneously"""
    
       #activate Joint1 for on screen control and get position and velosity readings
    
       joint1_position,joint1_velosity = activate_position_controller(Joint1_PC)
     
       # comment out the line above and uncomment the below to test the velocity_controller
       # joint1_position,joint1_velocity = activate_velocity_controller(Joint1_VC)
    
       print(joint1_position,joint1_velocity)
    
      pybullet.stepSimulation()
    
      pybullet.disconnect()
    
    

### Wrapping Up
- If you followed along correctly, you should have something like the examples shown below. 
      - python bullet_test.py - reference https://alexanderfabisch.github.io/pybullet.html 
      - python test_position_controller.py
      - python test_velocity_controller.py

<p align="center">
<img src="https://github.com/Jesse-Redford/Deep_Robot_Development/blob/master/Test_Files/test_position_controller.gif" width="250" height="250"> 
<img src="https://github.com/Jesse-Redford/Deep_Robot_Development/blob/master/Test_Files/test_velocity_controller.gif" width="250" height="250"> 
</p>



#### Endless Posibilites
- Example of a real robot I designed in solidworks moving around in the pybullet enviroment. 

<p align="center">
<img src="https://github.com/Jesse-Redford/Deep_Robot_Development/blob/master/Snakebot.gif" width="450" height="450"> 
</p>







