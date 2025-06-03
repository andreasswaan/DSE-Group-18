import rasterio
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


# === Load DSM ===
DSM_PATH = "/Users/feliceverbeek/Documents/GitHub/DSE-Group-18/Images/DSM_Delft.tif"
with rasterio.open(DSM_PATH) as src:
    dsm = src.read(1)
    transform = src.transform
    width = src.width
    height = src.height
    x_min, y_max = transform[2], transform[5]
    pixel_width, pixel_height = transform[0], -transform[4]
    x_max = x_min + width * pixel_width
    y_min = y_max - height * pixel_height
    extent = [x_min, x_max, y_min, y_max]

# === Clean DSM ===
dsm = np.where(dsm > 1000, np.nan, dsm)

# === Find high points ≥ 30m ===
threshold = 50
rows, cols = np.where(dsm >= threshold)
xs, ys = rasterio.transform.xy(transform, rows, cols)
coords = list(zip(xs, ys))

# === Print some high coordinates ===
print(f"Total points with height ≥ {threshold}m:", len(coords))
for i, coord in enumerate(coords[:10]):
    df = pd.DataFrame(coords, columns=["x_rd", "y_rd"])
    df.to_csv("high_dsm_points.csv", index=False)
    print("✅ Saved high DSM points to high_dsm_points.csv")

# === Plot DSM + High Points ===
plt.figure(figsize=(10, 6))
plt.imshow(dsm, cmap="terrain", extent=extent)
plt.scatter(xs, ys, c="red", s=1, label=f"≥ {threshold}m")
plt.colorbar(label="Elevation (m)")
plt.title("DSM with Points ≥ 30m in Red")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.legend()
plt.tight_layout()
plt.show()
