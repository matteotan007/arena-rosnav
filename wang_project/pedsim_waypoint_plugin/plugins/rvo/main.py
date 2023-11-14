from pedsim_waypoint_plugin.pedsim_waypoint_generator import OutputData, PedsimWaypointGenerator, InputData, WaypointPluginName, WaypointPlugin
import pedsim_msgs.msg
from collections import defaultdict
import numpy as np

@PedsimWaypointGenerator.register(WaypointPluginName.RVO)
class Plugin_rvo(WaypointPlugin):

    def __init__(self):
        ...

    def callback(self, data) -> OutputData:
        return [pedsim_msgs.msg.AgentFeedback(unforce=True) for agent in data.agents]
        
        
        
        
# class Plugin_RVO(WaypointPlugin):

#     def __init__(self):
#         ...
# 	def calculate_target(self,robot:pedsim_msgs.msg.RobotState):
# 		target_x = 10.0
# 		target_y = 10.0
# 		vector_x = target_x - robot.pose.position.x
# 		vector_y = target_y - robot.pose.position.y
		
# 		length = np.sqrt(vector_x ** 2 + vector_y ** 2)
# 		normalized_vector_x = vector_x / length
# 		normalized_vector_y = vector_y / length

# 		return normalized_vector_x, normalized_vector_y

#     def callback(self, data) -> OutputData:
        
#         def robot_to_waypoint(robot:pedsim_msgs.msg.RobotState) -> pedsim_msgs.msg.AgentFeedback:
#             target_vector = self.calculate_target(robot)
#             feedback = pedsim_msgs.msg.AgentFeedback()
            
#             feedback.id = robot.id 
#             feedback.force.x = target_vector[0]
#             feedback.force.y = target_vector[1]
#             return feedback
    
#         return [pedsim_msgs.msg.AgentFeedback(unforce=True) for agent in data.agents]
