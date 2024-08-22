import rospy
from plot_widget import *
from rpl_msgs.msg import SensorData




class MultipleCanvas:
    def __init__(self,name,num_vars,context):
        self.t0 = rospy.Time.now().to_sec()
        self.num_vars = num_vars
        self.widget_list = []
        for i in range(num_vars):
            self.widget_list.append(MplCanvas(parent = context))
        self.data_sub = rospy.Subscriber(name,SensorData,self.data_cb)
        

    def data_cb(self,msg):
        ynew = msg.data.data
        xnew = rospy.Time.now().to_sec() - self.t0
        for i in range(self.num_vars):
            self.widget_list.update_plot(ynew[i],xnew = xnew)

