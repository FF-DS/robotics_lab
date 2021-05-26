# robotics_lab

## Question - 1 :-
*Contain the fist question independent of the others, it contains two files VectorTransformServer.py, VectorTransformClient.py 
*VectorTransformServer uses service(with a service message called Transform.srv) to accept the vector, rotation angles and translation distance... then it uses numpy to transform the vector and send it back to the caller(which is VectorTransformClient.py).

## Question - 2 :-
*it contains the arm_description folder which has the model.sdf file edited per the assignment document specification.

## Question - 3 :-
*it contains the arm_ws folder, the update is contained inside the arm_gazebo folder which uses ros topic to publish angle states using a message object called JointAngleState.msg.

## Question - 4 :-
*it contains the arm_ws folder, which also contain all of the answers(starting from question 2-4). the final update is contained inside the arm_gazebo folder which uses ros topic to subscribe to a topic called 'UpdateJointAngles' and update the joints angle using a member class, while also publishing the joints angle unde the 'JointAngles' topic. 
*we have also created another ros node using python called ChangeJointAngles.py(found under the scripts folder), which accepts the four angles from users and publish it to the 'UpdateJointAngles' topic.
