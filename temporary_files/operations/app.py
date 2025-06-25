from flask import Flask, render_template, request, jsonify, send_file
import json
import numpy as np
import uuid

app = Flask(__name__)

def load_city_data_from_json(filename):
    """
    Loads city_name, population, restaurants, and silent_zones from a JSON city grid file.
    Returns: city_name, population (as np.array), restaurants (list), silent_zones (list)
    """
    with open(filename, 'r') as f:
        city_dict = json.load(f)
    city_name = city_dict['city_name']
    population = np.array(city_dict['population'])
    restaurants = city_dict['restaurants']
    silent_zones = city_dict['silent_zones']
    return city_name, population, restaurants, silent_zones

city_name, population_dict, restaurant_dict, silent_zones_dict = \
load_city_data_from_json("temporary_files/operations/delft_city_grid_10_test.json")

restaurants= [restaurant['name'] for restaurant in restaurant_dict]

orders = []

@app.route('/')
def index():
    return render_template('index.html', restaurants=restaurants)

@app.route('/submit_order', methods=['POST'])
def submit_order():
    data = request.json
    data['initials'] = data.get('initials', str(uuid.uuid4())[:8])  # Generate initials if not provided
    orders.append(data)
    print("Received order:", data)
    return jsonify({'status': 'success'})

@app.route('/get_orders')
def get_orders():
    return jsonify(orders)

if __name__ == '__main__':
    app.run(debug=True)