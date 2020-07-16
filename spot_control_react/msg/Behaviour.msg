# Behaviour message used higher level controllers to instruct reactive controller what the rover should be doing both
# now and in the future
# type: required; indicates the behaviour type. Supported behaviour types are described in an enum and include: MOVE, WONDER,
# EXPLORE, etc.
uint8 type
# movement: optional; describes how the rover should move if the behaviour type requires the rover to move in a particular
# direction or speed
geometry_msgs/TwistStamped movement
# duration: optional; is the number of seconds that the rover should perform the behaviour
uint8 duration
# source: required; indicates the origin of the behaviour message. Supported sources include: REMOTE, FEAT, etc. This is used for
# prioritzing the execution of multiple behaviours
uint8 source 
# goal_pose: optional; indicates the pose the rover should attempt to match
geometry_msgs/PoseStamped goal_pose
