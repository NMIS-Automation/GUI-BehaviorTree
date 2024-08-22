import rospy
from rpl_msgs.msg import SensorData
import random
import matplotlib
matplotlib.use('Qt5Agg')

from PyQt5 import QtCore, QtWidgets

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure



class MplCanvas(FigureCanvas):
    def __init__(self, name = None,n_data = None, fast = False, parent=None, width=5, height=4, dpi=100):
        fig = Figure()#figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)
        self.ndata = n_data
        if n_data is not None:
            self.ydata = [0 for _ in range(n_data)]
            self.xdata = range(n_data)
        else:
            self.ydata = []
            self.xdata = []

        self.fast = fast
        if name is not None:
            # if we haven't specified a subscription topic, we are handling this externally as we may need to update 6 plots for each of the individual joint torques, for example
            self.data_sub = rospy.Subscriber(name, SensorData, self.data_callback)

    def data_callback(self,msg):
        new_y = msg.data.data[0]
        new_x = None
        if msg.data.layout[0].size==2:
            new_x = msg.data.data[1]            
        if self.fast:
            self.update_plot_fast(new_y,newx=new_x)
            return
        self.update_plot(new_y, newx=new_x)            

    def update_plot(self, data,newx = None):
        # requires clearing canvas and then updating;
        self.ydata.append(data)
        if newx is not None:       
            self.xdata.append(newx)
        if self.ndata is not None:
            self.ydata = self.ydata[1:]            
            if newx is not None:
                self.xdata = self.xdata[1:]
        self.canvas.axes.cla()
        self.canvas.axes.plot(self.xdata, self.ydata, 'r')
        self.canvas.draw()

    def update_plot_fast(self,data,newx = None):
        self.ydata = self.ydata[1:].append(data)
        if newx is not None:
            self.xdata = self.xdata[1:].append(newx)
        if self._plot_ref is None:
            plot_refs = self.canvas.axes.plot(self.xdata, self.ydata, 'r')
            self._plot_ref = plot_refs[0]
        else:
            self._plot_ref.set_ydata(self.ydata)
            if newx is not None:
                self._plot_ref.set_xdata(self.xdata)
        self.canvas.draw()


