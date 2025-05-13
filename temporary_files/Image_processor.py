import rasterio
import matplotlib.pyplot as plt

# Open the GeoTIFF
with rasterio.open('/Users/feliceverbeek/Documents/GitHub/DSE-Group-18/Images/R_37EN1.tif') as dataset:
    height_data = dataset.read(1)  # Read the first band (elevation)
    profile = dataset.profile       # Metadata including CRS and transform

# Plot the height map
plt.imshow(height_data, cmap='terrain')
plt.colorbar(label='Elevation (m)')
plt.title('Elevation Data from GeoTIFF')
plt.axis('off')
plt.show()