import sys
from PyQt4 import QtGui, QtCore
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Circle
from matplotlib import animation
from time import sleep

class Window(QtGui.QDialog): #or QtGui.QWidget ???

    def __init__(self):
        super(Window, self).__init__()
        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111)  # create an axis
        self.ax.hold(False)  # discards the old graph
        self.ax.set_aspect('equal', 'box')
        self.circle = Circle((0,0), 1.0, animated=True)
        self.ax.add_artist(self.circle)
        self.ax.set_xlim([0, 10])
        self.ax.set_ylim([-2, 2])
        self.button = QtGui.QPushButton('Animate')
        self.button.clicked.connect(self.animate)

        # set the layout
        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.canvas)
        layout.addWidget(self.button)
        self.setLayout(layout)
        self.canvas.draw()
        self.ax_background = self.canvas.copy_from_bbox(self.ax.bbox)

    def animate(self):
        self.animate_loop(0)

    def animate_loop(self,i):
        for i in range(10):
            self.canvas.restore_region(self.ax_background)
            self.circle.center=(i,0)
            self.ax.draw_artist(self.circle)
            self.canvas.blit(self.ax.bbox)
            self.canvas.flush_events()
            sleep(0.1)

def main():

    app = QtGui.QApplication(sys.argv)
    ex = Window()
    ex.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main() 
    
    

    def CallGNSS_1(self, data):  # GNSS data
        self.GNSS_1 = GNSS_Raw_Array()
        self.GNSS_1 = data
        self.satecirclRad = 1.0
        for index_1 in range(len(self.GNSS_1.GNSS_Raws)):  # index all the satellite information in one epoch
            self.azim_.append( (self.GNSS_1.GNSS_Raws[index_1].elevation*(-0.55556)+50.0)* np.cos(-1*(self.GNSS_1.GNSS_Raws[index_1].azimuth-90.0)*3.14159/180.0))
            self.elev_.append( (self.GNSS_1.GNSS_Raws[index_1].elevation*(-0.55556)+50.0)* np.sin(-1*(self.GNSS_1.GNSS_Raws[index_1].azimuth-90.0)*3.14159/180.0))
            self.satIdx.append(self.GNSS_1.GNSS_Raws[index_1].prn_satellites_index)
        self.drawAll()

    def drawAll(self):
        self.drawBase()
        for sate_index in range(len(self.azim_)):  # draw the satellite into the Skyplot (template: circle+G30)
            self.axes_2.add_artist(Circle((self.azim_[sate_index], self.elev_[sate_index]), self.satecirclRad, color='r'))  # self.circle = Circle((0, 0), 50)
            self.axes_2.text(self.azim_[sate_index], self.elev_[sate_index], str(int(self.satIdx[sate_index])), fontdict={'size': self.fontSize, 'color': self.GsatColor})  # draw the 'E'
        self.axes_2.plot([self.bouSide1X, self.bouSide2X], [self.bouSide1Y, self.bouSide2Y], linewidth='1', color='fuchsia')  # draw a line from (0,0) to (50,0)
        self.canvas.draw()
        self.cleanS()

    def cleanS(self):
        self.axes_2.clear()
        del self.bouSide1X[:]
        del self.bouSide1Y[:]
        del self.bouSide2X[:]
        del self.bouSide2Y[:]
        del self.azim_[:]
        del self.elev_[:]
        del self.satIdx[:]
        # del self.GNSS_1.GNSS_Raws[:]
        # del self.posArr[:]