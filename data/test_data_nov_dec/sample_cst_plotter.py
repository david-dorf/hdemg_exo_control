import matplotlib.pyplot as plt
import csv

"""
Makes a plot of the CST data from the sample_trap_cst.csv file. This version of the CST was done by
hand rather than the algorithmic version in the CST node. This is to compare the two versions of the
CST.
"""

# Read the CSV data
with open('sample_trap_cst.csv', 'r') as f:
    reader = csv.reader(f)
    data = [float(row[0]) for row in reader]
    time = [time_point/1000 for time_point in range(len(data))]


# Plot the data
plt.plot(time, data)
plt.title('CST Data')
plt.xlabel('Time [S]')
plt.ylabel('CST Value')
plt.show()
