# An example of plotting using matplotlib for path visualization

from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

data = np.loadtxt('path.txt')
fig = plt.figure(1)
ax = fig.gca( )

ax.scatter([data[0,0],data[len(data)-1,0]],[data[0,1],data[len(data)-1,1]],c=['r','k'],edgecolor = 'face',marker='o',s = 40)
ax.plot(data[:,0],data[:,1],'.-',c = 'c')



circle1=plt.Circle((0.2,0.2),0.1,color='grey')
plt.gcf().gca().add_artist(circle1)
circle2=plt.Circle((0.8,0.8),0.1,color='grey')
plt.gcf().gca().add_artist(circle2)
circle3=plt.Circle((0.5,0.5),0.25,color='grey')
plt.gcf().gca().add_artist(circle3)


ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('path')



plt.show()







