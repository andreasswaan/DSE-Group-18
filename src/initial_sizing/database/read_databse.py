import pandas as pd
from sklearn.linear_model import LinearRegression


def get_data_from_database():
    """
    Function to read data from the database and return it as a dictionary.
    """

    # Read the data from the Excel file
    file_path = "src/initial_sizing/database/Database.xlsx"
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

    db_filtered = db[
        (db["Payload [kg]"].fillna(max_payload + 1) <= max_payload)
        & (db["Range [km]"].fillna(max_range + 1) <= max_range)
    ]
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
    X = db_filtered[["Payload [kg]", "Range [km]"]]
    y = db_filtered["MTOW [kg]"]
    model = LinearRegression()
    model.fit(X, y)
    return model.coef_, model.intercept_
