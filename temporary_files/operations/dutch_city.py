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



def get_city_border_polygon_ring(city_name: str, buffer_dist: float = 500) -> gpd.GeoDataFrame:
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


def plot_city(city_name: str, geometry_type: str = 'geometry'):
    """
    Plot the city boundaries and population density, and pizza restaurants as points.
    Also plots the city border crust as a ring around the city.
    """
    # Extract data for the specified city
    restaurants = get_pizza_restaurants(city_name)
    pop_dens = get_population_density(city_name)
    no_fly_zones = get_no_fly_zones(city_name)
    print('getting city border...')
    city_border_gdf = get_city_border_polygon_ring(city_name, buffer_dist=100)
    min_x, min_y = pop_dens.total_bounds[0], pop_dens.total_bounds[1]

    # Shift all data to (0,0) origin
    pop_dens, restaurants, no_fly_zones = change_coordinates(pop_dens, restaurants, no_fly_zones)
    # Shift border ring to match the shifted city
    city_border_gdf = city_border_gdf.copy()
    print(city_border_gdf)
    city_border_gdf['geometry'] = city_border_gdf['geometry'].translate(-min_x, -min_y)
    print(city_border_gdf)
    

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

    # Plot city border as a black polygon outline
    city_border_gdf.plot(ax=ax, facecolor='black', edgecolor='black', linewidth=2, zorder=50)


    # Plot restaurants
    x_name = 'x' if geometry_type == 'geometry' else 'lon'
    y_name = 'y' if geometry_type == 'geometry' else 'lat'
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
                Point(xy).buffer(100)  # meters radius of no-fly zone
                for xy in zip(no_fly_zones[x_name], no_fly_zones[y_name])
            ],
            crs="EPSG:28992",
        )
        no_fly_zone_circles.plot(
            ax=ax,
            color='purple',
            alpha=0.7,
            edgecolor='black',
            zorder=10
        )
    elif not no_fly_zones.empty:
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

    # Plot centroid of the city
    centroid = pop_dens.unary_union.centroid
    ax.plot(centroid.x, centroid.y, 'bo', markersize=10, label='City Centroid', zorder=12)
    # ax.set_xlim(pop_dens.total_bounds[[0, 2]])
    # ax.set_ylim(pop_dens.total_bounds[[1, 3]])

    ax.set_xlabel('X from datum [m]')
    ax.set_ylabel('Y from datum [m]')
    ax.legend()
    plt.show()
    
    
def change_coordinates(population_df, restaurant_df, no_fly_zone_df):
    """
    Shift all geometries and coordinates so the minimum x/y of the city population polygons is at (0,0).
    """
    
    # Get the minimum x and y from the population polygons' bounds
    min_x, min_y = population_df.total_bounds[0], population_df.total_bounds[1]

    # Shift the geometry of the population polygons
    population_df = population_df.copy()
    population_df['geometry'] = population_df['geometry'].translate(-min_x, -min_y)

    # Shift restaurant and no-fly zone coordinates (only if RD columns exist)
    restaurant_df = restaurant_df.copy()
    if 'rd_x' in restaurant_df.columns and 'rd_y' in restaurant_df.columns:
        restaurant_df['x'] = restaurant_df['rd_x'] - min_x
        restaurant_df['y'] = restaurant_df['rd_y'] - min_y

    no_fly_zone_df = no_fly_zone_df.copy()
    if 'rd_x' in no_fly_zone_df.columns and 'rd_y' in no_fly_zone_df.columns:
        no_fly_zone_df['x'] = no_fly_zone_df['rd_x'] - min_x
        no_fly_zone_df['y'] = no_fly_zone_df['rd_y'] - min_y

    return population_df, restaurant_df, no_fly_zone_df



def create_city_data(city_name: str, change_coords: bool = False) -> tuple:
    """
    Create a DataFrame with city data including population density, pizza restaurants, and no-fly zones.
    
    :param city_name: Name of the city to create data for.
    :return: Tuple of DataFrames (population density, pizza restaurants, no-fly zones).
    """
    pop_dens = get_population_density(city_name)
    restaurants = get_pizza_restaurants(city_name)
    no_fly_zones = get_no_fly_zones(city_name)
    
    if change_coords:
        pop_dens, restaurants, no_fly_zones = change_coordinates(pop_dens, restaurants, no_fly_zones)
    
    return pop_dens, restaurants, no_fly_zones
    

if __name__ == "__main__":

    city = "Delft"  # INPUT: Change this to any Dutch city name


    restaurants = get_pizza_restaurants(city)
    print(f"Pizza restaurants in {city}:")
    print(restaurants)


    pop_dens = get_population_density(city)
    print(f"\nPopulation density in {city}:")
    print(pop_dens['geometry'].head())


    no_fly_zones = get_no_fly_zones(city)
    print(f"\nNo-fly zones in {city}:")
    print(no_fly_zones)

    pop_dens, restaurants, no_fly_zones = change_coordinates(pop_dens, restaurants, no_fly_zones)
    print(f"\nPopulation density in {city}:")
    print(pop_dens['geometry'].head())


    plot_city(city, geometry_type='geometry')
    #plot_city(city, geometry_type='geometry_wgs84')





