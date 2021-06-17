# robotics_lab

## Final Project
### Question - 1 :-
*By extending on the model done during assignment one, on this version we have added the gripper by following the specification of the recently question description.
  
### Question - 2 :-
*On this part we have extended on top of the previous controller(which is done during assignment 1) and added the following functionalities
-> creating a topic to control the gripper
-> defining two methods which will assist for grabbing and releasing.
-> important service message files which will be used for IK and FK.
-> creating a method to perpare the starting state of the robot.
 
### Question - 3 :-
*On this part we have created and implemented the two IK and FK services and also created a python ros package called 'Transformation Calculator' which is used to assist in the mathematical calculation for the two services

### Final (ALL-IN-ONE):-
*This is where the complete final project is found it contains all of the above and their integration of those services on the controllers.


## Assignment 1
### Question - 1 :-
*Contain the fist question independent of the others, it contains two files VectorTransformServer.py, VectorTransformClient.py 
*VectorTransformServer uses service(with a service message called Transform.srv) to accept the vector, rotation angles and translation distance... then it uses numpy to transform the vector and send it back to the caller(which is VectorTransformClient.py).

### Question - 2 :-
*it contains the arm_description folder which has the model.sdf file edited per the assignment document specification.

### Question - 3 :-
*it contains the arm_ws folder, the update is contained inside the arm_gazebo folder which uses ros topic to publish angle states using a message object called JointAngleState.msg.

### Question - 4 :-
*it contains the arm_ws folder, which also contain all of the answers(starting from question 2-4). the final update is contained inside the arm_gazebo folder which uses ros topic to subscribe to a topic called 'UpdateJointAngles' and update the joints angle using a member class, while also publishing the joints angle unde the 'JointAngles' topic. 
*we have also created another ros node using python called ChangeJointAngles.py(found under the scripts folder), which accepts the four angles from users and publish it to the 'UpdateJointAngles' topic.
