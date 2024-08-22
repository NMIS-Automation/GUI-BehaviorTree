# behavior_nodes.py
import rospy
import py_trees
from rpl_msgs.msg import ActionCommand, SensorData

class ActionNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, command):
        super(ActionNode, self).__init__(name)
        self.command = ActionCommand()
        self.command.command = command

    def update(self):
        rospy.loginfo(f"Executing command: {self.command.command}")
        return py_trees.common.Status.SUCCESS

class ConditionNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, sensor_data):
        super(ConditionNode, self).__init__(name)
        self.sensor_data = SensorData()
        self.sensor_data.sensor_name = sensor_data

    def update(self):
        rospy.loginfo(f"Checking sensor data: {self.sensor_data.sensor_name}")
        if self.check_sensor_data():
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def check_sensor_data(self):
        # Placeholder for actual sensor data check
        return True
