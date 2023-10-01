import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file into a Pandas DataFrame
data = pd.read_csv('logger2.csv')

time = data['counter']
xpos = data['xpos']
ypos = data['ypos']
altitude = data['altitude']

fig = plt.figure()
ax = plt.axes(projection='3d')

ax.plot3D(xpos, ypos, altitude, 'red')
plt.show()