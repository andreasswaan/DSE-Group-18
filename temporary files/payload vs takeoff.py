import matplotlib.pyplot as plt
import numpy as np

# Data
Payload_weights = [5, 3, 3, 1, 1.5, 5, 4]
takeoff_weights = [11.5, 10, 16, 1.8, 3.5, 18.5,6.7]

# Fit a line of best fit
coefficients = np.polyfit(Payload_weights, takeoff_weights, 1)  # Linear fit
line_of_best_fit = np.poly1d(coefficients)
print(coefficients)
# Generate x values for the line
x = np.linspace(min(Payload_weights), max(Payload_weights), 100)
y = line_of_best_fit(x)

# Calculate R²
predicted = line_of_best_fit(Payload_weights)
ss_total = np.sum((takeoff_weights - np.mean(takeoff_weights))**2)
ss_residual = np.sum((takeoff_weights - predicted)**2)
r_squared = 1 - (ss_residual / ss_total)

# Plot the data
plt.scatter(Payload_weights, takeoff_weights, color='blue', label='Data Points')
plt.scatter(Payload_weights[-1], takeoff_weights[-1], color='green', label='Data Points')
plt.plot(x, y, color='red', label='Line of Best Fit')

# Add R² value to the plot
plt.text(0.05, 0.95, f"$R^2 = {r_squared:.2f}$", transform=plt.gca().transAxes, fontsize=12, verticalalignment='top')

# Add labels and legend
plt.xlabel('Payload Weight (kg)')
plt.ylabel('Takeoff Weights (kg)')
plt.legend(loc='lower right')  # Move the legend to the bottom-right corner
# Show the plot
plt.show()