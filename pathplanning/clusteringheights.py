import pandas as pd
import matplotlib.pyplot as plt

import pandas as pd
from sklearn.cluster import DBSCAN

def recluster_points(input_csv, output_csv, eps=5.0, min_samples=1):
    """
    Herclustert bestaande punten met een nieuwe DBSCAN radius (eps).
    
    Parameters:
        input_csv (str): Pad naar CSV met 'x_rd' en 'y_rd'.
        output_csv (str): Pad om de nieuwe clusters op te slaan.
        eps (float): Radius (in meters) voor clustering.
        min_samples (int): Minimaal aantal punten per cluster.
    """
    # Laad gegevens
    df = pd.read_csv(input_csv)
    
    # DBSCAN clustering
    X = df[['x_rd', 'y_rd']].values
    dbscan = DBSCAN(eps=eps, min_samples=min_samples)
    df['cluster'] = dbscan.fit_predict(X)
    
    # Bereken nieuwe clustercentra
    clustered = df.groupby('cluster')[['x_rd', 'y_rd']].mean().reset_index()
    
    # Sla op
    clustered.to_csv(output_csv, index=False)
    print(f"Herclustering voltooid. Nieuw bestand: {output_csv}")
    print(f"Aantal clusters: {len(clustered)}")

# Voorbeeldgebruik:
# recluster_points('cluster_centroids.csv', 'reclustered_centroids_5m.csv', eps=5.0)

# Load cluster centroids
df = pd.read_csv('/Users/feliceverbeek/Documents/GitHub/DSE-Group-18/cluster_centroids.csv')

# Plot clusters
plt.figure(figsize=(10, 8))
plt.scatter(df['x_rd'], df['y_rd'], s=10)
plt.xlabel('x_rd')
plt.ylabel('y_rd')
plt.title('Cluster Centroids (2m Accuracy)')
plt.grid(True)
plt.axis('equal')
plt.show()
