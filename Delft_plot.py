# Delft_plot.py

import rasterio
import numpy as np
import matplotlib.pyplot as plt

def load_dsm(path): 
    with rasterio.open(path) as src:
        dsm = src.read(1)
        transform = src.transform
        meta = src.meta
        width = src.width
        height = src.height
        crs = src.crs
    return dsm, transform, meta, width, height, crs

def plot_dsm(dsm, transform, highlight=True, ax=None):
    from rasterio.transform import xy

    if ax is None:
        fig, ax = plt.subplots(figsize=(10, 10))

    dsm = np.where(dsm > 1e5, np.nan, dsm)
    ax.imshow(dsm, cmap='terrain', vmin=0, vmax=70)

    if highlight:
        mask = dsm > 50
        rows, cols = np.where(mask)
        x_coords, y_coords = xy(transform, rows, cols)
        ax.scatter(x_coords, y_coords, s=5, c='red', label='>50 m')

    return ax
