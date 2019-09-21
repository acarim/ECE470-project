# ECE470-project

By the end of the semester, we plan to implement a robot capable of driving a car autonomously, both interacting with a predefined set of obstacles and localizing itself in a specific map. These obstacles can be fixed, representing walls signs, etc. or mobile, in order to explore the interaction with pedestrians and other vehicles. The predefined map will represent a set of pathways and the robot will be assigned the task of determining its position using a particle filter whose sensor data is still to determine. Once the starting position is defined, the desired destination will be specified and the robot will have to compute the shortest path and travel it safely.


# GETTING STARTED 

The repository contains all the files needed to run the simulation. The "circuit.ttt" is the current version of the VREP scene and the "update2" python file contains the code to run the simulation. The rest of the files are needed for the remote API access required to let python communicate with Vrep to run the simulation. 

To test, download or clone all files in the repository to your computer and run on the required software.

# PREREQUISITES AND RUNNING TESTS

To run the files, you will need a current version of VREP and access to python. All the files should be saved in the same directory which ideally should be different from the locaation of the VREP location. then follow the steps below;
  1. Open the update2 python code. 
  2. Open the circuit scene on vrep.
  3. Make sure the port no in the Client ID on python is the same as that on VREP. 
  4. Press play on the simulation on VREP
  5. Run the python script.
  
 A lonk to a YouTube vidoe showing the current simulation can be seen here
 
 Make sure the port no in the Client ID is the same as that on VREP. 
 
 # AUTHORS
 
 Jacob Roing
 Alvaro
 Abdulmalik Carim
 Taofik 
 Jason Sach
