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
    gdf = gpd.read_file("/Users/feliceverbeek/Documents/GitHub/DSE-Group-18/Images/wijkenbuurten_2024_v1.gpkg", layer="buurten")

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

# Example usage
city = "Delft"  # INPUT: Change this to any Dutch city name

# Load height data from CSV
height_data = pd.read_csv('/Users/feliceverbeek/Documents/GitHub/DSE-Group-18/high_dsm_points.csv')  # Ensure this file exists and has the correct structure

plot_city(city, geometry_type='geometry', height_data=height_data)