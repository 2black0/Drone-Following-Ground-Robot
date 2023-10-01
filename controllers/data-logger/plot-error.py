import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

data_drone = pd.read_csv('logger_drone.csv')
data_mobile = pd.read_csv('logger_mobile.csv')

error_x_positions = data_mobile['xPosition'] - data_drone['xPosition']
error_y_positions = data_mobile['yPosition'] - data_drone['yPosition']
error_alti_positions = data_drone['altitude'] - data_mobile['altitude']

fig = plt.figure()
'''
ax = fig.add_subplot(1, 2, 1)

mean_x_error = np.mean(error_x_positions)
median_x_error = np.median(error_x_positions)
std_x_error = np.std(error_x_positions)

ax.plot(data_mobile['counter'], error_x_positions, 'g-', label='Error in X Position')
ax.set_title(f'Mean Error: {mean_x_error:.2f} | Median Error: {median_x_error:.2f} | Std. Dev. of Error: {std_x_error:.2f}')
ax.set_xlabel('Time')
ax.set_ylabel('Error in X Position')
ax.grid(True)

ax = fig.add_subplot(1, 2, 2)

mean_y_error = np.mean(error_y_positions)
median_y_error = np.median(error_y_positions)
std_y_error = np.std(error_y_positions)

ax.plot(data_mobile['counter'], error_y_positions, 'g-', label='Error in X Position')
ax.set_title(f'Mean Error: {mean_y_error:.2f} | Median Error: {median_y_error:.2f} | Std. Dev. of Error: {std_y_error:.2f}')
ax.set_xlabel('Time')
ax.set_ylabel('Error in Y Position')
ax.grid(True)
'''
ax = fig.add_subplot()

print(error_alti_positions.min())

mean_alti_error = np.mean(error_alti_positions)
median_alti_error = np.median(error_alti_positions)
std_alti_error = np.std(error_alti_positions)

ax.plot(data_mobile['counter'], error_alti_positions, 'g-', label='Error in Altitude Position')
ax.set_title(f'Mean Error: {mean_alti_error:.2f} | Median Error: {median_alti_error:.2f} | Std. Dev. of Error: {std_alti_error:.2f}')
ax.set_xlabel('Time')
ax.set_ylabel('Error in Altitude Position')
ax.grid(True)

#fig.savefig('error.png', dpi=300)

plt.show()