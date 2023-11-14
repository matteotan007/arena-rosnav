from pedsim_waypoint_plugin.pedsim_waypoint_generator import OutputData, PedsimWaypointGenerator, InputData, WaypointPluginName, WaypointPlugin
import pedsim_msgs.msg
import rvo2
import numpy as np

@PedsimWaypointGenerator.register(WaypointPluginName.RVO2)
class Plugin_Rvo2(WaypointPlugin):
    def __init__(self):
        self.simulator = rvo2.PyRVOSimulator(1/60., 1.5, 5, 1.5, 2, 0.4, 2)  # Adjusted parameters
        self.agent_id_map = {}
        self.diameter = 4.0 

    def callback(self, data) -> OutputData:
        
        def datapoint_to_vec(agent: pedsim_msgs.msg.AgentState) -> pedsim_msgs.msg.AgentFeedback:

            angle = agent.direction
            
            #velocity = np.linalg.norm(np.array([agent.twist.linear.x, agent.twist.linear.y]))
            velocity = self.diameter / 2
            feedback = pedsim_msgs.msg.AgentFeedback()
	     
            feedback.id = agent.id
            angle = np.pi * self.simulator.getGlobalTime() / (2*np.pi)  # Change from 0 to pi over approx. one time period
            feedback.force.x = np.cos(angle) * velocity
            feedback.force.y = np.sin(angle) * velocity

            # If agent not in simulator, add it and process any obstacles
            if agent.id not in self.agent_id_map.keys():
                self.agent_id_map[agent.id] = self.simulator.addAgent((agent.pose.position.x, agent.pose.position.y), 1.5, 5, 1.5, 2, 0.4, 2, (feedback.force.x, feedback.force.y))
                self.simulator.processObstacles()
            
            # Set the calculated force as the preferred velocity of the agent in the simulator
            self.simulator.setAgentPrefVelocity(self.agent_id_map[agent.id], (feedback.force.x, feedback.force.y))

            return feedback

        # Perform simulation step
        self.simulator.doStep()
        
        return [datapoint_to_vec(agent) for agent in data.agents]
    
    



