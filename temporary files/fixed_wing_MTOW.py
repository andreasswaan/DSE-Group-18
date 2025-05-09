import numpy as np
import pandas as pd
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Sample dataset: Replace with your actual values
data = {
    'Range_km':   [20, 15, 20, 8, 30, 50, 40, 80, 15, 5],
    'Payload_kg': [0.7, 3, 1.2, 1, 4, 9, 2, 2.5, 1, 1],
    'MTOW_kg':    [2, 17, 11, 5, 11, 35, 10, 27.5, 5, 2.7]
}

df = pd.DataFrame(data)

# Independent variables
X = df[['Payload_kg', 'Range_km']]
y = df['MTOW_kg']

# Train linear regression model
model = LinearRegression()
model.fit(X, y)
print("Model R^2 score:", model.score(X, y))

# Predict MTOW for mission values
print("Predicted MTOW for mission values:", model.predict([[3, 12]]))

# Coefficients
print("Intercept:", model.intercept_)
print("Coefficients:", model.coef_)

# Predict on training data
df['Predicted_MTOW'] = model.predict(X)

# Create meshgrid for plane
payload_range = np.linspace(df['Payload_kg'].min(), df['Payload_kg'].max(), 10)
range_range = np.linspace(df['Range_km'].min(), df['Range_km'].max(), 10)
Payload_mesh, Range_mesh = np.meshgrid(payload_range, range_range)

# Compute corresponding MTOW values from the regression plane
MTOW_mesh = (
    model.intercept_
    + model.coef_[0] * Payload_mesh
    + model.coef_[1] * Range_mesh
)

# 3D Plot
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Actual and predicted data points
ax.scatter(df['Payload_kg'], df['Range_km'], df['MTOW_kg'], color='blue', label='Actual MTOW')
ax.scatter(df['Payload_kg'], df['Range_km'], df['Predicted_MTOW'], color='red', label='Predicted MTOW', marker='^')

# Regression plane
ax.plot_surface(Payload_mesh, Range_mesh, MTOW_mesh, color='green', alpha=0.3, label='Regression Plane')

# Labels
ax.set_xlabel('Payload (kg)')
ax.set_ylabel('Range (km)')
ax.set_zlabel('MTOW (kg)')
ax.set_title('3D Linear Regression Plane for Drone MTOW')
ax.legend()
plt.tight_layout()
plt.show()