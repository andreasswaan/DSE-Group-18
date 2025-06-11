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
import shapely
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

def get_city_border_polygon_ring(city_name: str, buffer_dist: float = 200) -> gpd.GeoDataFrame:
    """
    Returns a GeoDataFrame with a single polygon that is a "border ring" around the city.
    """
    gdf = gpd.read_file("pathplanning/wijkenbuurten_2024_v1.gpkg", layer="buurten")
    gdf_city = gdf[gdf['gemeentenaam'] == city_name].copy()
    city_polygon = gdf_city.union_all()
    outer = city_polygon.buffer(buffer_dist)
    border_ring = outer.difference(city_polygon)
    border_gdf = gpd.GeoDataFrame({'geometry': [border_ring]}, crs=gdf_city.crs)
    return border_gdf


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


def cell_circle_overlap(cell_x, cell_y, cell_size, circle_x, circle_y, circle_radius):
    # Find the closest point on the cell to the circle center
    closest_x = np.clip(circle_x, cell_x, cell_x + cell_size)
    closest_y = np.clip(circle_y, cell_y, cell_y + cell_size)
    dist = np.hypot(closest_x - circle_x, closest_y - circle_y)
    return dist <= circle_radius

def cell_square_overlap(cell_x, cell_y, cell_size, square_cx, square_cy, square_halfsize):
    # Find the closest point on the cell to the square center
    # Square bounds
    sq_x0 = square_cx - square_halfsize
    sq_x1 = square_cx + square_halfsize
    sq_y0 = square_cy - square_halfsize
    sq_y1 = square_cy + square_halfsize
    # Cell bounds
    cell_x0 = cell_x
    cell_x1 = cell_x + cell_size
    cell_y0 = cell_y
    cell_y1 = cell_y + cell_size
    # Check for rectangle intersection
    overlap_x = not (cell_x1 < sq_x0 or cell_x0 > sq_x1)
    overlap_y = not (cell_y1 < sq_y0 or cell_y0 > sq_y1)
    return overlap_x and overlap_y



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
                square_halfsize = 10.0  # 10m half-width = 20m total
                radius_cells = int(np.ceil(square_halfsize / resolution))
                for i in range(max(0, y_idx - radius_cells), min(height, y_idx + radius_cells + 1)):
                    for j in range(max(0, x_idx - radius_cells), min(width, x_idx + radius_cells + 1)):
                        # Get cell bounds (lower left corner)
                        cell_x = x_coords[j] - resolution / 2
                        cell_y = y_coords[i] - resolution / 2
                        if cell_square_overlap(cell_x, cell_y, resolution, x, y, square_halfsize):
                            obstacle_grid[i, j] = True  # Mark as tall building (obstacle)
                            weight_grid[i, j] = np.inf  # make tall building impassable
    except Exception as e:
        print(f"Height data loading warning: {e}")

    # Mark no-fly zones (absolute obstacles)
    for _, zone in no_fly_zones.iterrows():
        x, y = zone['rd_x'], zone['rd_y']
        x_idx, y_idx = coord_to_index(x, y)
        radius_cells = int(np.ceil(50 / resolution))  # 50m radius, ceil to cover all
        for i in range(max(0, y_idx - radius_cells), min(height, y_idx + radius_cells + 1)):
            for j in range(max(0, x_idx - radius_cells), min(width, x_idx + radius_cells + 1)):
                # Get cell bounds (lower left corner)
                cell_x = x_coords[j] - resolution / 2
                cell_y = y_coords[i] - resolution / 2
                if cell_circle_overlap(cell_x, cell_y, resolution, x, y, 50):
                    obstacle_grid[i, j] = True
                    weight_grid[i, j] = np.inf  # Make completely impassable
    
    # --- Add city border as a no-fly zone ---
    try:
        border_gdf = get_city_border_polygon_ring(city_name)  # adjust buffer as needed
        border_poly = border_gdf.geometry.unary_union  # Get the (Multi)Polygon
        for i in range(height):
            for j in range(width):
                # Get cell bounds (center)
                cell_x = x_coords[j]
                cell_y = y_coords[i]
                # Create a shapely box for the cell
                cell_poly = shapely.geometry.box(
                    cell_x - resolution/2, cell_y - resolution/2,
                    cell_x + resolution/2, cell_y + resolution/2
                )
                # If the cell intersects the border polygon, mark as obstacle
                if border_poly.intersects(cell_poly):
                    obstacle_grid[i, j] = True
                    weight_grid[i, j] = np.inf
    except Exception as e:
        print(f"Warning: Could not add city border as no-fly zone: {e}")

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
    }, x_coords, y_coords


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

    no_fly_zones_df = grid_data.get("raw_no_fly_zones_df", None)

    
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

    restaurants_df = grid_data.get("raw_restaurants_df", None)


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
print("Creating weighted grid for Delft...")
# city = "Delft"
# grid_data = create_weighted_grid(city, resolution=100.0)
# no_fly_zone = get_no_fly_zones('Delft')  # 10m resolution
# copy_dutch_city = ('Delft')
# print('Plotting weighted grid...')
# plot_weighted_grid(grid_data)

# --- EXPORT GRID TO DISK ---
# output_path = "pathplanning/data/delft_grid_data_test.npz"
# np.savez_compressed(output_path,
#     weight_grid=grid_data['weight_grid'],
#     obstacle_grid=grid_data['obstacle_grid'],
#     restaurant_grid=grid_data['restaurant_grid'],
#     x_coords=grid_data['x_coords'],
#     y_coords=grid_data['y_coords'],
#     bounds=grid_data['bounds'],
#     resolution=grid_data['resolution']
# )

# for res in [5, 10, 20, 30, 40, 50, 60, 80, 90, 100]:
#     print(f"Creating weighted grid for Delft with resolution {res}m...")
#     grid_data = create_weighted_grid(city, resolution=res)
#     print('Done')

    
#     # Save each grid to a separate file
#     output_path = f"pathplanning/data/delft_grid_data_{res}_test.npz"
#     np.savez_compressed(output_path,
#         weight_grid=grid_data['weight_grid'],
#         obstacle_grid=grid_data['obstacle_grid'],
#         restaurant_grid=grid_data['restaurant_grid'],
#         x_coords=grid_data['x_coords'],
#         y_coords=grid_data['y_coords'],
#         bounds=grid_data['bounds'],
#         resolution=grid_data['resolution']
#     )

import numpy as np
import json


def create_city_grid(grid_data, restaurants_df, x_coords, y_coords):
    """
    Create the city grid data structure for use with save_city_grid (no tall buildings, silent zones = weight grid).
    - city_name: str
    - grid_data: dict with at least 'weight_grid' (population density)
    - restaurants_df: DataFrame with columns ['x', 'y', 'name']
    Returns: (population_grid, restaurant_dict, silent_zone_dict)
    """

    def coord_to_index(x, y):
        x_idx = np.argmin(np.abs(x_coords - x))
        y_idx = np.argmin(np.abs(y_coords - y))
        return x_idx, y_idx

    # Restaurants
    restaurant_dict = []
    for i, row in restaurants_df.iterrows():
        x_idx, y_idx = coord_to_index(row['rd_x'], row['rd_y'])
        restaurant_dict.append({
            'xpos': int(x_idx),
            'ypos': int(y_idx),
            'restaurant_id': i,
            'name': row.get('name', f"Restaurant_{i}"),
            'mean_nr_orders': row.get('mean_nr_orders', 400),  # Default or from df
            'mean_order_size': row.get('mean_order_size', {'small': 0, 'medium': 3, 'large': 0})
        })

    # Silent zones: every cell in the grid is a silent zone with value from obstacle_grid
    silent_zone_dict = []
    obstacle_grid = grid_data['obstacle_grid']
    height, width = obstacle_grid.shape
    for y in range(height):
        for x in range(width):
            silent_zone_dict.append({
                'xpos': x,
                'ypos': y,
                'value': float(obstacle_grid[y, x])
            })
            
    population_dict = []
    population_grid = grid_data['weight_grid']
    height, width = population_grid.shape
    for y in range(height):
        for x in range(width):
            population_dict.append({
                'xpos': x,
                'ypos': y,
                'value': float(population_grid[y, x] if population_grid[y, x] != np.inf else 0.0)  # Use 0 for impassable cells
            })

    return population_dict, restaurant_dict, silent_zone_dict

def save_city_grid(city_name, population_grid, restaurant_dict, silent_zone_dict, filename):
    city_dict = {
        'city_name': city_name,
        'population': population_grid,
        'restaurants': restaurant_dict,
        'silent_zones': silent_zone_dict
    }
    with open(filename, 'w') as f:
        json.dump(city_dict, f)
        

# Usage example:
city_name = "Delft"
grid_data, x_bounds, y_bounds = create_weighted_grid(city_name, resolution=10.0)
restaurants_df = get_pizza_restaurants(city_name)

population_dict, restaurant_dict, silent_zone_dict = create_city_grid(
    grid_data, restaurants_df, x_bounds, y_bounds
)

print('done')
print(population_dict[:5]) 
print(restaurant_dict[:5])
print(silent_zone_dict[:5])  

# Save to JSON file (no tall buildings, empty depots)
# save_city_grid(
#     city_name,
#     population_dict,
#     restaurant_dict,
#     silent_zone_dict,
#     "delft_city_grid_70_PP.json"
# )


