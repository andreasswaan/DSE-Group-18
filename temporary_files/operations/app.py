from flask import Flask, render_template, request, jsonify, send_file, url_for
import json
import numpy as np
import uuid
import qrcode
import io

app = Flask(__name__)


def load_city_data_from_json(filename):
    """
    Loads city_name, population, restaurants, and silent_zones from a JSON city grid file.
    Returns: city_name, population (as np.array), restaurants (list), silent_zones (list)
    """
    with open(filename, "r") as f:
        city_dict = json.load(f)
    city_name = city_dict["city_name"]
    population = np.array(city_dict["population"])
    restaurants = city_dict["restaurants"]
    silent_zones = city_dict["silent_zones"]
    return city_name, population, restaurants, silent_zones


city_name, population_dict, restaurant_dict, silent_zones_dict = (
    load_city_data_from_json("temporary_files/operations/delft_city_grid_10_test.json")
)

restaurants = [restaurant["name"] for restaurant in restaurant_dict]
print(restaurants)
orders = []


@app.route("/")
def index():
    return render_template("index.html", restaurants=restaurants)


@app.route("/submit_order", methods=["POST"])
def submit_order():
    data = request.json
    data["initials"] = data.get(
        "initials", str(uuid.uuid4())[:8]
    )  # Generate initials if not provided
    orders.append(data)
    print("Received order:", data)
    return jsonify({"status": "success"})


@app.route("/get_orders")
def get_orders():
    return jsonify(orders)


@app.route("/delft_boundary")
def delft_boundary():
    return send_file("delft_boundary.geojson", mimetype="application/json")


@app.route("/qr")
def qr():
    # Use the local network address or your public URL if sharing with others
    url = "http://192.168.56.1:5000/"
    img = qrcode.make(url)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    buf.seek(0)
    return send_file(buf, mimetype="image/png")


if __name__ == "__main__":
    app.run(host="192.168.56.1", debug=True)
