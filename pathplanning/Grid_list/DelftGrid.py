import geopandas as gpd
import pandas as pd
from copy import deepcopy
import matplotlib.pyplot as plt
import json
from pyproj import Transformer
import overpy
from shapely.ops import transform
from shapely.geometry import Point
from matplotlib.patches import Patch
import rasterio
import numpy as np 
from scipy.spatial import distance
# ------ FILES ------

# Data including geometries
# https://www.cbs.nl/nl-nl/dossier/nederland-regionaal/geografische-data/wijk-en-buurtkaart-2024

# Netherlands building height data:
# https://service.pdok.nl/rws/ahn/atom/dsm_05m.xml

api = overpy.Overpass()

query = """
area["name"="Delft"]->.searchArea;
(
  // Explicitly tagged with cuisine=pizza
  node["amenity"~"restaurant|fast_food"]["cuisine"~"pizza"](area.searchArea);

  // Cuisine mentions Italian (often includes pizza)
  node["amenity"~"restaurant|fast_food"]["cuisine"~"italian", i](area.searchArea);

  // Name contains "pizza" or "pizzeria"
  node["amenity"~"restaurant|fast_food"]["name"~"pizza|pizzeria", i](area.searchArea);

  // Description mentions pizza
  node["amenity"~"restaurant|fast_food"]["description"~"pizza", i](area.searchArea);
);
out center;
"""

def get_pizza_restaurants(city_name: str) -> pd.DataFrame:
    """
    Query the Overpass API for pizza restaurants in a given city and return a DataFrame.

    :param city_name: Name of the city to search in.
    :return: DataFrame of pizza restaurants with name, lat, lon, and tags.
    """

    api = overpy.Overpass()
    query = f"""
    area["name"="{city_name}"]->.searchArea;
    (
      node["amenity"~"restaurant|fast_food"]["cuisine"~"pizza"](area.searchArea);
      node["amenity"~"restaurant|fast_food"]["cuisine"~"italian", i](area.searchArea);
      node["amenity"~"restaurant|fast_food"]["name"~"pizza|pizzeria", i](area.searchArea);
      node["amenity"~"restaurant|fast_food"]["description"~"pizza", i](area.searchArea);
    );
    out center;
    """
    result = api.query(query)
    records = []
    for node in result.nodes:
        rec = {
            "name": node.tags.get("name", "Unnamed"),
            "lat": float(node.lat),
            "lon": float(node.lon),
            **node.tags
        }
        records.append(rec)
    restaurant_df = pd.DataFrame(records)
    
    # Add lat and lon WGS84 (EPSG:4326) to (EPSG:28992)
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:28992", always_xy=True)
    restaurant_df[["rd_x", "rd_y"]] = restaurant_df.apply(
        lambda row: pd.Series(transformer.transform(row["lon"], row["lat"])), axis=1
    )
    
    return restaurant_df

def get_no_fly_zones(city_name: str, amenity_list=['hospital', 'school', 'kindergarten', 'cemetery', 'fire_station', 'police', 'prison']) -> pd.DataFrame:
    """
    Query the Overpass API for no-fly zone amenities in a given city and return a DataFrame of nodes.
    """
    
    api = overpy.Overpass()
    
    # Build Overpass query for all amenities in the list
    amenity_query = "\n".join(
        [f'  node["amenity"="{amenity}"](area.searchArea);' for amenity in amenity_list]
    )
    query = f"""
    area["name"="{city_name}"]->.searchArea;
    (
    {amenity_query}
    );
    out center;
    """
    
    result = api.query(query)
    records = []
    for node in result.nodes:
        rec = {
            "name": node.tags.get("name", "Unnamed"),
            "amenity": node.tags.get("amenity", ""),
            "lat": float(node.lat),
            "lon": float(node.lon),
            **node.tags
        }
        records.append(rec)
        
    no_fly_zone_df = pd.DataFrame(records)
        
    # Add lat and lon WGS84 (EPSG:4326) to (EPSG:28992)
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:28992", always_xy=True)
    no_fly_zone_df[["rd_x", "rd_y"]] = no_fly_zone_df.apply(
        lambda row: pd.Series(transformer.transform(row["lon"], row["lat"])), axis=1
    )
    return no_fly_zone_df

def get_dutch_city_names() -> list:
    """
    Overpass API query to list all city names in the Netherlands.
    Returns a list of city names.
    """
    query = """
    area["name"="Nederland"]["boundary"="administrative"]->.nl;
    (
      node["place"="city"](area.nl);
      relation["place"="city"](area.nl);
    );
    out tags;
    """
    result = api.query(query)
    city_names = []

    # Collect names from nodes
    for node in result.nodes:
        name = node.tags.get("name")
        if name:
            city_names.append(name)
    # Collect names from relations
    for rel in result.relations:
        name = rel.tags.get("name")
        if name:
            city_names.append(name)
    return city_names

# Print all city names
#for name in get_dutch_city_names():
    #print(name)
    
#print('\nTotal cities found:', len(get_dutch_city_names()))

def get_population_density(city_name: str) -> pd.DataFrame:
    """
    Get population density data for a specific city.

    :param city_name: Name of the city to get population density for.
    :return: DataFrame with population density information.
    """
    # Load the GeoDataFrame with population density data
    gdf = gpd.read_file("pathplanning/wijkenbuurten_2024_v1.gpkg", layer="buurten")

    # Filter for the specified city and drop all other rows
    gdf_city = gdf[gdf['gemeentenaam'] == city_name].copy()
    gdf_city['bevolkingsdichtheid_inwoners_per_km2'] = gdf_city['bevolkingsdichtheid_inwoners_per_km2'].clip(lower=0)
    gdf_city['aantal_inwoners'] = gdf_city['aantal_inwoners'].clip(lower=0)
    gdf_city = gdf_city.reset_index(drop=True)

    # Convert geometry from RD New (EPSG:28992) to WGS84 (EPSG:4326)
    transformer = Transformer.from_crs("EPSG:28992", "EPSG:4326", always_xy=True)
    gdf_city['geometry_wgs84'] = gdf_city['geometry'].apply(lambda geom: transform(transformer.transform, geom))

    # Only keep relevant columns
    gdf_reduced = gdf_city[['buurtcode', 'geometry', 'geometry_wgs84', 'aantal_inwoners', 'bevolkingsdichtheid_inwoners_per_km2']]
    return gdf_reduced


def plot_city(city_name: str, geometry_type: str = 'geometry', height_data=None):
    """
    Plot the city boundaries and population density, pizza restaurants, and height locations.
    
    :param city_name: Name of the city to plot.
    :param geometry_type: Geometry type to use ('geometry' or 'WGS84').
    :param height_data: Optional DataFrame containing height location data.
    """
    
    # Extract data for the specified city
    restaurants = get_pizza_restaurants(city_name)
    pop_dens = get_population_density(city_name)
    no_fly_zones = get_no_fly_zones(city_name)
    
    # Plot population density
    fig, ax = plt.subplots(figsize=(10, 10))
    pop_dens = pop_dens.set_geometry(geometry_type)
    pop_dens.plot(
        column='bevolkingsdichtheid_inwoners_per_km2',
        ax=ax,
        legend=True,
        cmap='YlOrRd',
        edgecolor='black'
    )
    ax.set_title(f'Map of {city_name}')
    
    # Plot restaurants
    x_name = 'rd_x' if geometry_type == 'geometry' else 'lon'
    y_name = 'rd_y' if geometry_type == 'geometry' else 'lat'
    if not restaurants.empty:
        ax.scatter(
            restaurants[x_name],
            restaurants[y_name],
            color='green',
            s=30,
            edgecolor='black',
            label='Pizza Restaurants',
            zorder=11
        )
        
    # Plot no-fly zones as constant-size circles
    if not no_fly_zones.empty and geometry_type == 'geometry':
        no_fly_zone_circles = gpd.GeoDataFrame(
            geometry=[
                Point(xy).buffer(50)  # meters radius of no-fly zone
                for xy in zip(no_fly_zones['rd_x'], no_fly_zones['rd_y'])
            ],
            crs="EPSG:28992",
        )
        # Plot and add a proxy artist for legend
        no_fly_zone_circles.plot(
            ax=ax,
            color='purple',
            alpha=0.7,
            edgecolor='black',
            zorder=10
        )
    elif not no_fly_zones.empty:
        # For WGS84, fallback to scatter (cannot buffer in degrees meaningfully)
        ax.scatter(
            no_fly_zones[x_name],
            no_fly_zones[y_name],
            color='purple',
            s=150,
            alpha=0.7,
            edgecolor='black',
            label='No-Fly Zones',
            marker='o',
            zorder=10
        )

    # Plot height locations if provided
    if height_data is not None and not height_data.empty:
        try:
            height_x = 'x_rd' if geometry_type == 'geometry' else 'lon'
            height_y = 'y_rd' if geometry_type == 'geometry' else 'lat'
            ax.scatter(
                height_data[height_x],
                height_data[height_y],
                color='red',
                s=50,
                edgecolor='black',
                label='Height Locations',
                zorder=12
            )
        except KeyError as e:
            print(f"Error: Missing column in height data - {e}")

    # Plot centroid of the city
    centroid = pop_dens.unary_union.centroid
    ax.plot(centroid.x, centroid.y, 'bo', markersize=10, label='City Center', zorder=13)
    ax.set_xlim(pop_dens.total_bounds[[0, 2]])
    ax.set_ylim(pop_dens.total_bounds[[1, 3]])
    
    ax.set_xlabel('X from datum [m]')
    ax.set_ylabel('Y from datum [m]')
    ax.legend()
    plt.show()

import numpy as np
from scipy.spatial import distance

def create_navigation_grid(city_name: str, resolution: float = 10.0) -> dict:
    """
    Create a navigation grid for A* pathfinding algorithm.
    
    :param city_name: Name of the city to create grid for
    :param resolution: Grid resolution in meters
    :return: Dictionary containing grid information for pathfinding
    """
    # Get all the necessary data
    pop_dens = get_population_density(city_name)
    restaurants = get_pizza_restaurants(city_name)
    no_fly_zones = get_no_fly_zones(city_name)
    
    # Get the bounds of the city in RD coordinates
    bounds = pop_dens.total_bounds  # [min_x, min_y, max_x, max_y]
    
    # Calculate grid dimensions
    width = int(np.ceil((bounds[2] - bounds[0]) / resolution))
    height = int(np.ceil((bounds[3] - bounds[1]) / resolution))
    
    # Create empty grids
    obstacle_grid = np.zeros((height, width), dtype=bool)  # True where obstacles exist
    height_grid = np.zeros((height, width), dtype=float)   # Height values
    population_grid = np.zeros((height, width), dtype=float)  # Population density
    restaurant_grid = np.zeros((height, width), dtype=bool)  # Pizza restaurant locations
    
    # Create coordinate grids
    x_coords = np.linspace(bounds[0], bounds[2], width)
    y_coords = np.linspace(bounds[1], bounds[3], height)
    
    # Helper function to convert real coordinates to grid indices
    def coord_to_index(x, y):
        x_idx = np.argmin(np.abs(x_coords - x))
        y_idx = np.argmin(np.abs(y_coords - y))
        return x_idx, y_idx
    
    # Mark no-fly zones as obstacles (50m radius)
    for _, zone in no_fly_zones.iterrows():
        x, y = zone['rd_x'], zone['rd_y']
        x_idx, y_idx = coord_to_index(x, y)
        
        # Create a circular obstacle with 50m radius (5 grid cells at 10m resolution)
        radius_cells = int(50 / resolution)
        for i in range(max(0, y_idx-radius_cells), min(height, y_idx+radius_cells+1)):
            for j in range(max(0, x_idx-radius_cells), min(width, x_idx+radius_cells+1)):
                if distance.euclidean((x_coords[j], y_coords[i]), (x, y)) <= 50:
                    obstacle_grid[i, j] = True
    
    # Mark pizza restaurants
    for _, restaurant in restaurants.iterrows():
        x, y = restaurant['rd_x'], restaurant['rd_y']
        x_idx, y_idx = coord_to_index(x, y)
        restaurant_grid[y_idx, x_idx] = True
    
    # Add height data if available
    try:
        height_data = pd.read_csv('high_dsm_points.csv')
        for _, point in height_data.iterrows():
            x, y = point['x_rd'], point['y_rd']
            x_idx, y_idx = coord_to_index(x, y)
            height_grid[y_idx, x_idx] = point['height']
    except Exception as e:
        print(f"Could not load height data: {e}")
    
    # Add population density data
    for _, neighborhood in pop_dens.iterrows():
        # Get polygon bounds
        minx, miny, maxx, maxy = neighborhood['geometry'].bounds
        
        # Convert to grid indices
        minx_idx, miny_idx = coord_to_index(minx, miny)
        maxx_idx, maxy_idx = coord_to_index(maxx, maxy)
        
        # Fill the grid area with population density
        density = neighborhood['bevolkingsdichtheid_inwoners_per_km2']
        population_grid[miny_idx:maxy_idx+1, minx_idx:maxx_idx+1] = density
    
    return {
        'obstacle_grid': obstacle_grid,
        'height_grid': height_grid,
        'population_grid': population_grid,
        'restaurant_grid': restaurant_grid,
        'x_coords': x_coords,
        'y_coords': y_coords,
        'resolution': resolution,
        'bounds': bounds
    }

def create_weighted_grid(city_name: str, resolution: float = 10.0) -> dict:
    """
    Create a weighted grid that maintains exact spatial representation of Delft.
    Each cell gets weights based on population density, height, and contains markers for restaurants.
    
    :param city_name: Name of the city (should be "Delft")
    :param resolution: Grid resolution in meters (default 10m)
    :return: Dictionary containing all grid information
    """
    # Get the base data
    pop_dens = get_population_density(city_name)
    restaurants = get_pizza_restaurants(city_name)
    no_fly_zones = get_no_fly_zones(city_name)
    
    # Get the actual bounds of Delft in RD coordinates
    bounds = pop_dens.total_bounds  # [min_x, min_y, max_x, max_y]
    
    # Create grid dimensions that maintain exact spatial proportions
    width = int(np.ceil((bounds[2] - bounds[0]) / resolution))
    height = int(np.ceil((bounds[3] - bounds[1]) / resolution))
    
    # Create empty grids
    weight_grid = np.zeros((height, width), dtype=float)  # Combined weight grid
    obstacle_grid = np.zeros((height, width), dtype=bool)  # No-fly zones
    restaurant_grid = np.zeros((height, width), dtype=bool)  # Pizza locations
    
    # Create coordinate mapping
    x_coords = np.linspace(bounds[0], bounds[2], width)
    y_coords = np.linspace(bounds[1], bounds[3], height)
    
    # Helper function to convert real coordinates to grid indices
    def coord_to_index(x, y):
        x_idx = np.argmin(np.abs(x_coords - x))
        y_idx = np.argmin(np.abs(y_coords - y))
        return x_idx, y_idx
    
    # Process population density (primary weight factor)
    for _, neighborhood in pop_dens.iterrows():
        poly = neighborhood['geometry']
        density = neighborhood['bevolkingsdichtheid_inwoners_per_km2']
        norm_density = density / pop_dens['bevolkingsdichtheid_inwoners_per_km2'].max()
        # Get bounding box in grid indices
        minx, miny, maxx, maxy = poly.bounds
        minx_idx, miny_idx = coord_to_index(minx, miny)
        maxx_idx, maxy_idx = coord_to_index(maxx, maxy)
        # For each grid cell in the bounding box, check if its center is inside the polygon
        for y_idx in range(miny_idx, maxy_idx + 1):
            for x_idx in range(minx_idx, maxx_idx + 1):
                if 0 <= y_idx < height and 0 <= x_idx < width:
                    cell_center = Point(x_coords[x_idx], y_coords[y_idx])
                    if poly.contains(cell_center):
                        weight_grid[y_idx, x_idx] = norm_density

    # Add height data if available (secondary weight factor)
    # Add height data from clustered high points (assumed to be >50m)
    try:
        height_data = pd.read_csv('cluster_centroids.csv')
        for _, point in height_data.iterrows():
            x, y = point['x_rd'], point['y_rd']
            x_idx, y_idx = coord_to_index(x, y)
            if 0 <= y_idx < height and 0 <= x_idx < width:
                radius_cells = int(20 / resolution)  # 20m square half-size
                for i in range(max(0, y_idx - radius_cells), min(height, y_idx + radius_cells + 1)):
                    for j in range(max(0, x_idx - radius_cells), min(width, x_idx + radius_cells + 1)):
                        obstacle_grid[i, j] = True # Mark as tall building (obstacle)
                        weight_grid[i, j] = np.inf # make heigh bulding impassable
                        
    except Exception as e:
        print(f"Height data loading warning: {e}")

    # Mark no-fly zones (absolute obstacles)
    for _, zone in no_fly_zones.iterrows():
        x, y = zone['rd_x'], zone['rd_y']
        x_idx, y_idx = coord_to_index(x, y)
        radius_cells = int(50 / resolution)  # 50m radius
        for i in range(max(0, y_idx-radius_cells), min(height, y_idx+radius_cells+1)):
            for j in range(max(0, x_idx-radius_cells), min(width, x_idx+radius_cells+1)):
                if distance.euclidean((x_coords[j], y_coords[i]), (x, y)) <= 50:
                    obstacle_grid[i, j] = True
                    weight_grid[i, j] = np.inf  # Make completely impassable
    
    # Mark pizza restaurants (special points of interest)
    for _, restaurant in restaurants.iterrows():
        x, y = restaurant['rd_x'], restaurant['rd_y']
        x_idx, y_idx = coord_to_index(x, y)
        if 0 <= y_idx < height and 0 <= x_idx < width:
            restaurant_grid[y_idx, x_idx] = True
    
    return {
        'weight_grid': weight_grid,
        'obstacle_grid': obstacle_grid,
        'restaurant_grid': restaurant_grid,
        'x_coords': x_coords,
        'y_coords': y_coords,
        'bounds': bounds,
        'resolution': resolution,
        'raw_restaurants_df': restaurants,
        'raw_no_fly_zones_df': no_fly_zones
    }


def plot_weighted_grid(grid_data):
    """Visualize the weighted grid with all components"""
    fig, ax = plt.subplots(figsize=(12, 12))
    
    # Create base plot with weights (population density + height)
    weights = grid_data['weight_grid'].copy()
    masked_weights = np.ma.masked_where(weights == np.inf, weights)
  # For visualization
    
    # Create custom colormap that shows different aspects
    cmap = plt.cm.viridis.copy()
    cmap.set_over('red', 1.0)  # Obstacles in red
    
    finite_weights = masked_weights[~masked_weights.mask]
    vmax = np.nanmax(finite_weights)
    
    im = ax.imshow(
        #weights,
        masked_weights,
        cmap='YlOrRd', 
        origin='lower',
        extent=[grid_data['bounds'][0], grid_data['bounds'][2],
                grid_data['bounds'][1], grid_data['bounds'][3]],
        vmin=0.0,
        vmax=vmax)
    
        # — No-Fly Zones: oranje cirkels met 50m radius (op ware afstand) —
    try:
        # Probeer de originele DataFrame met no-fly zones uit grid_data te halen:
        no_fly_zones_df = grid_data.get("raw_no_fly_zones_df", None)
        if no_fly_zones_df is None:
            from copy_dutch_city import get_no_fly_zones
            no_fly_zones_df = get_no_fly_zones("Delft")
    except ImportError:
        no_fly_zones_df = None
    
    if no_fly_zones_df is not None and not no_fly_zones_df.empty:
        for _, zone in no_fly_zones_df.iterrows():
            cx, cy = zone['rd_x'], zone['rd_y']
            circle = plt.Circle(
                (cx, cy),
                50.0,              # 50 meter radius
                facecolor='lightblue',
                edgecolor='none',
                alpha=1.0,         # volledig ondoorzichtig
                zorder=5
            )
            ax.add_patch(circle)
    
    # — Pizza Restaurants: groene stippen —
    try:
        restaurants_df = grid_data.get("raw_restaurants_df", None)
        if restaurants_df is None:
            from copy_dutch_city import get_pizza_restaurants
            restaurants_df = get_pizza_restaurants("Delft")
    except ImportError:
        restaurants_df = None

    if restaurants_df is not None and not restaurants_df.empty:
       ax.scatter(
            restaurants_df['rd_x'],
            restaurants_df['rd_y'],
            color='green',
            s=50,
            edgecolor='black',
            linewidth=0.5,
            label='Pizza Restaurants',
            zorder=7
        )
    
    height_df = pd.read_csv("cluster_centroids.csv")
    # Helper om X,Y → gridindex om te rekenen (rij/kolom):
    x_coords = grid_data['x_coords']
    y_coords = grid_data['y_coords']

    square_halfsize = 10.0  # 10m half‐width = 20m totaal
    for _, point in height_df.iterrows():
        hx, hy = point['x_rd'], point['y_rd']
        # Bepaal linker‐onderhoek (x0,y0) van vierkant:
        x0 = hx - square_halfsize
        y0 = hy - square_halfsize
        rect = plt.Rectangle(
            (x0, y0),
            2 * square_halfsize,
            2 * square_halfsize,
            facecolor='lightblue',
            edgecolor='none',
            alpha=1.0,   # volledig ondoorzichtig
            zorder=6
        )
        ax.add_patch(rect)

    # Add labels and title
    ax.set_xlabel('X from datum [m]')
    ax.set_ylabel('Y from datum [m]')
    ax.set_title(f'Weighted Navigation Grid for Delft\n(Resolution: {grid_data["resolution"]}m)')
    
    # Add colorbar
    cbar = plt.colorbar(im, ax=ax, label='Navigation Weight')
    
    # — Aangepaste legenda: basislaag (laag gewicht=geel) + overlays —
    legend_elements = [
        Patch(facecolor='yellow',  edgecolor='none', label='Laag gewicht (0)'),
        Patch(facecolor='lightblue',  edgecolor='none', label='No-Fly Zones'),
        Patch(facecolor='lightblue', edgecolor='none', label='Height (>50m)'),
        Patch(facecolor='green',   edgecolor='black', label='Pizza Restaurants')
    ]
    ax.legend(handles=legend_elements, loc='upper right')

    plt.tight_layout()
    plt.show()

# Example usage
city = "Delft"
grid_data = create_weighted_grid(city, resolution=10.0)
no_fly_zone = get_no_fly_zones('Delft')  # 10m resolution
copy_dutch_city = ('Delft')
plot_weighted_grid(grid_data)

# --- EXPORT GRID TO DISK ---
output_path = "pathplanning/data/delft_grid_data.npz"
np.savez_compressed(output_path,
    weight_grid=grid_data['weight_grid'],
    obstacle_grid=grid_data['obstacle_grid'],
    restaurant_grid=grid_data['restaurant_grid'],
    x_coords=grid_data['x_coords'],
    y_coords=grid_data['y_coords'],
    bounds=grid_data['bounds'],
    resolution=grid_data['resolution']
)
