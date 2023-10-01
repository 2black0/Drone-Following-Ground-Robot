import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file into a Pandas DataFrame
data_drone = pd.read_csv('logger_drone.csv')
data_mobile = pd.read_csv('logger_mobile.csv')

xpos_drone = data_drone['xpos']
ypos_drone = data_drone['ypos']
altitude_drone = data_drone['altitude']

xpos_mobile = data_mobile['xPosition']
ypos_mobile = data_mobile['yPosition']

fig = plt.figure()
ax = plt.axes(projection='3d')

ax.plot3D(xpos_drone, ypos_drone, altitude_drone, 'red')
plt.show()