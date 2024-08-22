import rospy



class topics:
    def __init__(self):
        self.topic_dict = {}

    def refresh(self):
        topics = rospy.get_published_topics()
        if len(topics)==0:
            return None
        self.topic_dict = {}
        for t in topics:
            if t[1] in list(self.topic_dict.keys()):
                self.topic_dict[t[1]].append(t[0])
            else:
                self.topic_dict[t[1]] = [t[0]]
        return len(topics)
    
    def get_sensor_topics(self):
        if 'rpl_msgs/SensorData' in list(self.topic_dict.keys()):
            return self.topic_dict['rpl_msgs/SensorData']
        return None

    def get_image_topics(self):
        if 'sensor_msgs/Image' in list(self.topic_dict.keys()):
            return self.topic_dict['sensor_msgs/Image']
        return None

    def get_q_topic(self,name):
        if name in list(self.topic_dict.keys()):
            return self.topic_dict[name]
        return None
    




