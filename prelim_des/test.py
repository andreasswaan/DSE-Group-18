from airfoils import Airfoil

foil = Airfoil.NACA4("6412")
foil._normalise_data_points()
foil._normalise_data_points()

foil.plot()
