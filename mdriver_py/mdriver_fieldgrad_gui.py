#!/usr/bin/env python3

"""
This script provides a basic GUI for setting fields and gradients in an eMNS using the mdriver hardware.
"""

# importing various libraries
import sys
from PyQt5.QtWidgets import QDialog, QApplication, QPushButton, QVBoxLayout, QSlider, QSpinBox, QLabel, QHBoxLayout
from PyQt5.QtCore import *
from PyQt5.QtCore import Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.pyplot as plt
from magnetic_system import magnetic_system, compute_bfi_matrix
from std_msgs.msg import Float32MultiArray
import numpy as np
from math import sin,cos,pi
from numpy.linalg import pinv
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('QtAgg')
import rclpy
from rclpy.node import Node
  
class ROS2Thread(QThread):
    """this thread class is used to run the event loop of the ROS2 node
    """
    def __init__(self, node):
        super().__init__()
        self._node = node
        self._running = True

    def run(self):
        """this thread runs the event loop of the ROS2 node
        """
        #loops & processes ROS2 events
        rclpy.spin(self._node)

    def stop(self):
        self._running = False
        self._node.destroy_node()
        rclpy.shutdown()
        self.wait()

class FieldForceGUI(QDialog):
      
    def button_pressed_mpl(self,event):
        B=2*self.maxB*(event.xdata-self.N/2)/self.N
        F=2*self.maxF*(event.ydata-self.N/2)/self.N
        self.Bmag=B
        self.Fmag=F
        self.xmarker=event.xdata
        self.ymarker=event.ydata
        self.update_currents(B,F)

    # constructor
    def __init__(self, magnetic_system, parent=None):
        super(FieldForceGUI, self).__init__(parent)
        rclpy.init()
        self.setWindowTitle("Field Explorer")
        self.resize(500,1000)
        self.node=Node("field_explorer_node")

        self.system=magnetic_system

        #register a ros publisher to publish desired currents
        self.currents_pub=self.node.create_publisher(Float32MultiArray,"/mdriver/des_currents_reg",10)
        self.des_currents_msg=Float32MultiArray()
        self.des_currents_msg.data=[0,0,0,0,0,0]

        # a figure instance to plot on
        self.figure = plt.figure()
        self.figure2 = plt.figure()
        
        self.figure.canvas.mpl_connect("button_press_event",self.button_pressed_mpl)
  
        # this is the Canvas Widget that
        # displays the 'figure'it takes the
        # 'figure' instance as a parameter to __init__
        self.canvas = FigureCanvas(self.figure)
        self.canvas2 = FigureCanvas(self.figure2)
        
        #parameters related to field force plots
        self.N=75
        self.maxB=50e-3
        self.maxF=1
        
        #spin boxes for setting inclination and azimuth
        self.spinbox_azimuth=QSpinBox(self)
        self.spinbox_azimuth.setSingleStep(5)
        self.spinbox_azimuth.setMaximum(360)
        self.spinbox_azimuth.setMinimum(-360)
        self.spinbox_azimuth.setValue(90)
        self.spinbox_inclination=QSpinBox(self)
        self.spinbox_inclination.setSingleStep(5)
        self.spinbox_inclination.setMaximum(180)
        self.spinbox_inclination.setMinimum(-180)
        self.spinbox_inclination.setValue(90)
        self.spinbox_azimuth_f=QSpinBox(self)
        self.spinbox_azimuth_f.setSingleStep(5)
        self.spinbox_azimuth_f.setMaximum(360)
        self.spinbox_azimuth_f.setMinimum(-360)
        self.spinbox_azimuth_f.setValue(90)
        self.spinbox_inclination_f=QSpinBox(self)
        self.spinbox_inclination_f.setSingleStep(5)
        self.spinbox_inclination_f.setMaximum(180)
        self.spinbox_inclination_f.setMinimum(-180)
        self.spinbox_inclination_f.setValue(90)

        #red marker
        self.Bmag=0
        self.Fmag=0
        self.xmarker=self.N/2
        self.ymarker=self.N/2
        self.Bdir=np.array([0,0,1])
        self.Fdir=np.array([0,0,1])
        
        #button for sending currents
        self.sendcurrents_button=QPushButton("Send Currents")
        self.sendcurrents_button.clicked.connect(self.send_currents)
        
        #label for displaying currents
        self.currents_label=QLabel("[%.2f , %.2f , %.2f , %.2f , %.2f , %.2f]"%(0,0,0,0,0,0))
        self.currents_label.setAlignment(Qt.AlignCenter)
        self.field_force_label=QLabel("Force: %.2f ; Field: %.2f"%(0,0))
        self.field_force_label.setAlignment(Qt.AlignCenter)
        
        #data for potting coordinate systems
        self.quiverxs=[0.5,0,0]
        self.quiverys=[0,0.5,0]
        self.quiverzs=[0,0,0.5]
        
        #azimuth and elevation for 3d displays
        self.view_elevation=30
        self.view_azimuth=-60
          
        # adding action to the button
        #self.button.clicked.connect(self.plot)
        self.spinbox_azimuth.valueChanged.connect(self.azimuth_changed)
        self.spinbox_inclination.valueChanged.connect(self.inclination_changed)
        self.spinbox_azimuth_f.valueChanged.connect(self.azimuth_changed_f)
        self.spinbox_inclination_f.valueChanged.connect(self.inclination_changed_f)
  
        # creating a Vertical Box layout
        layout = QVBoxLayout()
          
        # adding canvas to the layout
        layout.addWidget(self.canvas)
        layout.addWidget(self.canvas2)
        
        def pack_into_hlayout(msg,spbox):
            layout=QHBoxLayout()
            label=QLabel(msg)
            layout.addWidget(label)
            layout.addWidget(spbox)
            return layout
          
        # adding push button to the layout
        layout.addLayout(pack_into_hlayout("B-Azimuth",self.spinbox_azimuth))
        layout.addLayout(pack_into_hlayout("B-Inclination",self.spinbox_inclination))
        layout.addLayout(pack_into_hlayout("F-Azimuth",self.spinbox_azimuth_f))
        layout.addLayout(pack_into_hlayout("F-Inclination",self.spinbox_inclination_f))
        layout.addWidget(self.field_force_label)
        layout.addWidget(self.currents_label)
        layout.addWidget(self.sendcurrents_button)
          
        # setting layout to the main window
        self.setLayout(layout)
        self.update_angled()

        #run the event loop of the ROS2 node in a separate thread
        self.ros2_thread=ROS2Thread(self.node)
        self.ros2_thread.start()
        
    def update_currents(self,B,F):
        if B>=0:
            A=self.system.getA((0,0,0),self.Bdir)
        else:
            A=self.system.getA((0,0,0),-self.Bdir)
        Bvec=self.Bdir*B
        Fvec=self.Fdir*F
        currents=pinv(A).dot(np.append(Bvec,Fvec))
        self.currents=currents
        self.currents_label.setText("[%.2f , %.2f , %.2f , %.2f , %.2f , %.2f]"%tuple(currents))
        self.field_force_label.setText("Force: %.2fmT/mm ; Field: %.2fmT"%(F,1e3*B))
        self.redraw_workspace()
        
    def send_currents(self,event):
        for i in range(0,len(self.currents)):
            if self.currents[i]>10:
                self.currents[i]=10
            elif self.currents[i]<-10:
                self.currents[i]=-10
        self.des_currents_msg.data=list(self.currents)
        self.currents_pub.publish(self.des_currents_msg)
        
    def update_angled(self):
        self.phi_b=self.spinbox_azimuth.value()/180*pi
        self.theta_b=self.spinbox_inclination.value()/180*pi
        self.phi_f=self.spinbox_azimuth_f.value()/180*pi
        self.theta_f=self.spinbox_inclination_f.value()/180*pi
        self.Bdir=np.array([sin(self.theta_b)*cos(self.phi_b),sin(self.theta_b)*sin(self.phi_b),cos(self.theta_b)])
        self.Fdir=np.array([sin(self.theta_f)*cos(self.phi_f),sin(self.theta_f)*sin(self.phi_f),cos(self.theta_f)])
        self.update_currents(self.Bmag,self.Fmag)
        
    def azimuth_changed(self,event):
        self.update_angled()
        self.redraw_workspace()
        self.draw_forcedir_fielddir()
        
    def inclination_changed(self,event):
        self.update_angled()
        self.redraw_workspace()
        self.draw_forcedir_fielddir()
        
    def azimuth_changed_f(self,event):
        self.update_angled()
        self.redraw_workspace()
        self.draw_forcedir_fielddir()
        
    def inclination_changed_f(self,event):
        self.update_angled()
        self.redraw_workspace()
        self.draw_forcedir_fielddir()
  
    def redraw_workspace(self):
        """
        this function generates / plot the magnetic workspace plot
        """
        self.figure.clear()

        #compute magnetic workspace image and plot it
        Bs=np.linspace(-self.maxB,self.maxB,self.N+1)
        Fs=np.linspace(-self.maxF,self.maxF,self.N+1)
        imaxs=compute_bfi_matrix(self.system,Bs,Fs,self.Bdir,self.Fdir)
        axim=self.figure.gca().imshow(imaxs,vmin=0,vmax=12,cmap="jet",origin="lower",interpolation="bilinear")

        #draw the x marker
        self.figure.gca().scatter(self.xmarker,self.ymarker,color=(0,0,0,0.5),marker="X")

        #draw magnetic field and gradient ticks
        xticks=np.array([0,self.N/4,self.N/2,3*self.N/4,self.N])
        ax=self.figure.gca()
        ax.set_xticks(xticks)
        ax.set_xticklabels(["%.2f"%(1e3*(xtick/(len(Bs)-1)*100e-3-50e-3)) for xtick in xticks])
        yticks=xticks
        ax.set_yticks(yticks)
        ax.set_yticklabels(["%.2f"%((ytick/(len(Bs)-1)*2-1)) for ytick in yticks])

        #set title and colorbar
        ax.set_xlabel("Flux Density B [mT]")
        ax.set_ylabel("Force F [mT/mm/Am²]")
        #ax.set_title("$||\mathbf{i}||_{\infty}(\mathbf{B},\mathbf{F})$ [A]")
        self.figure.colorbar(axim,ax=self.figure.gca())
        self.figure.tight_layout()
        self.canvas.draw()
        
    def draw_forcedir_fielddir(self):
        try:
            self.view_elevation=self.ax3d.elev
            self.view_azimuth=self.ax3d.azim
        except:
            pass
        self.figure2.clear()
        self.ax3d=self.figure2.add_subplot(111,projection="3d")
        self.ax3d.view_init(self.view_elevation,self.view_azimuth)
        self.ax3d.quiver([0,0,0],[0,0,0],[0,0,0],self.quiverxs,self.quiverys,self.quiverzs,color="black")
        self.ax3d.quiver(0,0,0,self.Bdir[0],self.Bdir[1],self.Bdir[2],color="blue")
        self.ax3d.quiver(0,0,0,self.Fdir[0],self.Fdir[1],self.Fdir[2],color="red")
        self.ax3d.set_xlim(-1,1)
        self.ax3d.set_ylim(-1,1)
        self.ax3d.set_zlim(-1,1)
        self.ax3d.set_xlabel("x")
        self.ax3d.set_ylabel("y")
        self.ax3d.set_zlabel("z")
        
        self.ax3d.text(self.Bdir[0],self.Bdir[1],self.Bdir[2],"Field",color="blue")
        self.ax3d.text(self.Fdir[0],self.Fdir[1],self.Fdir[2],"Force",color="red")
        self.canvas2.draw()
  
# driver code
if __name__ == '__main__':
    # creating apyqt5 application
    app = QApplication(sys.argv)

    import os
    calibration_file=os.getenv("DATABASEPATH")+"/tnb_emns/calibration_data/mininavion_steelcore_steelyoke_14cm/calibration.pkl"
    system=magnetic_system(calibration_file,"pickle",5,5,5,6)
    main = FieldForceGUI(system)
    # showing the window
    main.show()
    main.redraw_workspace()
    main.draw_forcedir_fielddir()
    # loop
    sys.exit(app.exec_())