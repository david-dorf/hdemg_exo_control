import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV data
data = pd.read_csv('smooth_cst_trap_output.csv')

# Convert time from nanoseconds to seconds
data['%time'] = data['%time'] / 1e9 - min(data['%time'] / 1e9)

# Convert DataFrame columns to NumPy arrays
time_array = data['%time'].to_numpy()
cst_array = data['field.data.data'].to_numpy()

# Plot the data
plt.plot(time_array, cst_array)
plt.title('CST Data')
plt.xlabel('Time (s)')
plt.ylabel('CST Value')
plt.show()
