import numpy as np
import pandas as pd
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt

# Read CSV file
df = pd.read_csv('rent_data.csv')

# Independent variables
X = df[["distance", "m2"]]
y = df[["rent_total_monthly"]]


# Train linear regression model
model = LinearRegression()
model.fit(X, y)
print("Model R^2 score:", model.score(X, y))

# Coefficients
print("Intercept:", model.intercept_)
print("Coefficients:", model.coef_)

# Create 3D plot
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Scatter plot of actual data
ax.scatter(df["distance"], df["m2"], df["rent_total_monthly"], color='blue', label="Actual data")

# Create a meshgrid for the plane
distance_range = np.linspace(df["distance"].min(), df["distance"].max(), 20)
m2_range = np.linspace(df["m2"].min(), df["m2"].max(), 20)
distance_grid, m2_grid = np.meshgrid(distance_range, m2_range)

# Predict rent using the regression model
rent_pred = model.intercept_[0] + model.coef_[0][0] * distance_grid + model.coef_[0][1] * m2_grid

# Plot the regression plane
ax.plot_surface(distance_grid, m2_grid, rent_pred, alpha=0.5, color='red')


# Create meshgrid for plane
distance_range = np.linspace(df['distance'].min(), df['distance'].max(), 10)
m2_range = np.linspace(df['m2'].min(), df['m2'].max(), 10)
Distance_mesh, M2_mesh = np.meshgrid(distance_range, m2_range)

# Labels
ax.set_xlabel('Distance from city centre (m)')
ax.set_ylabel('Area (m2)')
ax.set_zlabel('Total monthly rent (EUR)')
ax.set_title('3D Linear Regression Plane for Depot Rent Costs')
ax.legend()
plt.tight_layout()
plt.show()
