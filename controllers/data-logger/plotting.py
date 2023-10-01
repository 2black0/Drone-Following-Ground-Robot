import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file into a Pandas DataFrame
data_drone = pd.read_csv('logger_drone.csv')
data_mobile = pd.read_csv('logger_mobile.csv')

counter = data_drone['counter'].add(-948)

xpos_drone = data_drone['xPosition']
ypos_drone = data_drone['yPosition']
altitude_drone = data_drone['altitude'].add(2.99)

xpos_mobile = data_mobile['xPosition']
ypos_mobile = data_mobile['yPosition']
altitude_mobile = data_mobile['altitude'].add(3.08)

fig = plt.figure()


#ax = fig.add_subplot(2, 1, 1)
ax = fig.subplots()
#ax.plot(counter, xpos_drone, counter, xpos_mobile)
#ax.plot(counter, ypos_drone, counter, ypos_mobile)
ax.plot(counter, altitude_drone, counter, altitude_mobile)
ax.set_title('Altitude Position of Drone and Mobile Robot')
ax.set_xlabel('time')
ax.set_ylabel('Position')
ax.grid(True)

'''
#ax = fig.add_subplot(2, 1, 2, projection='3d')
ax = fig.add_subplot(projection='3d')
ax.plot3D(ypos_drone, xpos_drone, altitude_drone, 'red')
ax.plot3D(ypos_mobile, xpos_mobile, altitude_mobile, 'blue')
ax.set_title('3D Trajectory of Drone and Mobile Robot')
ax.set_xlabel('x Position')
ax.set_ylabel('y Position')
ax.set_zlabel('altitude')'''

ax.legend(['Drone', 'Mobile Robot'])

fig.savefig('alti_pos.png', dpi=300)
plt.show()