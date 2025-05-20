import pandas as pd
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures


def get_data_from_database():
    """
    Function to read data from the database and return it as a dictionary.
    """

    # Read the data from the Excel file
    # file_path = "src/initial_sizing/database/pablo database.xlsx"
    file_path = "src/initial_sizing/database/eVTOL.xlsx"

    dataframe = pd.read_excel(file_path)

    # Display the dataframe
    return dataframe


def apply_db_filters(db, max_payload, max_range):
    """
    Function to filter data from the database and return it as a dictionary.
    """
    # Read the data from the Excel file
    # file_path = "temporary files\preliminary sizing\Database.xlsx"
    # dataframe = pd.read_excel(file_path)

    # Display the dataframe
    db["Payload [kg]"] = pd.to_numeric(db["Payload [kg]"], errors="coerce")
    db["Range [km]"] = pd.to_numeric(db["Range [km]"], errors="coerce")

    # db_filtered = db[
    #     (db["Payload [kg]"].fillna(max_payload + 1) <= max_payload)
    #     & (db["Range [km]"].fillna(max_range + 1) <= max_range)
    # ]
    db_filtered = db[(db["Payload [kg]"].fillna(max_payload + 1) <= max_payload)]
    return db_filtered


def get_regression_plane(db, max_payload, max_range):
    """
    Function to get the regression plane from the database.

    Args:
        db (dataframe): start dataframe.
        max_payload (float): maximum payload for filter.
        max_range (float): maximum range for filter.

    Returns:
        tuple: Coefficients of the regression plane and intercept.
    """
    # Read the data from the Excel file
    # file_path = "temporary files\preliminary sizing\Database.xlsx"
    # dataframe = pd.read_excel(file_path)

    # Display the dataframe
    db = db.dropna(subset=["Payload [kg]", "Range [km]", "MTOW [kg]"])
    db_filtered = apply_db_filters(db, max_payload, max_range)
    # X = db_filtered[["Payload [kg]", "Range [km]"]]
    X = db_filtered[["Payload [kg]"]]

    y = db_filtered["MTOW [kg]"]
    model = LinearRegression()
    poly = PolynomialFeatures(degree=2, include_bias=True)
    X_poly = poly.fit_transform(X)
    model.fit(X, y)
    # To get the coefficients for the polynomial regression:
    coefs = model.coef_
    intercept = model.intercept_

    return model.coef_, model.intercept_


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    db = get_data_from_database()
    db = apply_db_filters(db, 4.5, 0)
    db = db.dropna(subset=["Payload [kg]", "Range [km]", "MTOW [kg]"])
    [A], C = get_regression_plane(db, 4.5, 0)

    plt.scatter(db["Payload [kg]"], db["MTOW [kg]"])
    # Regression line
    x_vals = db["Payload [kg]"].sort_values()
    y_vals = A * x_vals + C
    plt.plot(x_vals, y_vals, color="red", label="Regression line")
    plt.xlabel("Payload [kg]")
    plt.ylabel("MTOW [kg]")
    plt.title("Payload vs MTOW")
    plt.legend()
    plt.show()

    print(db)
