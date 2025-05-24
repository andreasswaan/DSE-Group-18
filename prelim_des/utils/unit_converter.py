import datetime
import numpy as np

class ImperialConverter:
    def velocity_kts_mps(velkts):
        velmps = velkts * 0.51444444
        return velmps

    def mass_lbsh_kgs(masslbsh):
        masskgs = (masslbsh/3600) * 0.45359237
        return masskgs

    def mass_lbs_kg(masslbs):
        masskg = masslbs * 0.45359237
        return masskg
    
    def mass_kg_lbs(masskg):
        masslbs = masskg / 0.45359237
        return masslbs

    def alt_ft_m(hft):
        hm = hft * 0.3048
        return hm

    def temp_Cel_K(tempC):
        tempK = tempC + 273.15
        return tempK

    def len_in_m(lenin):
        lenm = lenin * 0.0254
        return lenm

    def moment_lbsin_kgm(momentlbsin):
        momentkgm = momentlbsin * 0.0254 * 0.45359237
        return momentkgm
    
    def len_ft_m(lenft):
        lenm = lenft * 0.3048
        return lenm
    
    def len_m_ft(lenm):
        lenft = lenm / 0.3048
        return lenft
    
    def area_m2_ft2(area_m2):
        area_ft2 = area_m2 / 0.09290304
        return area_ft2


class TimeConverter:
    def time_to_sec(time_obj):
        """
        Convert datetime.time object/HH:MM:SS to seconds since midnight
        """
        if isinstance(time_obj, str):
            elem = np.array(time_obj.split(':'),dtype='int')
            time_obj = datetime.time(*elem)
        return time_obj.hour * 3600 + time_obj.minute * 60 + time_obj.second

    def sec_to_time(sec):
        """
        Convert seconds since midnight to datetime.time object
        """
        hours = int(sec // 3600)
        minutes = int((sec % 3600) // 60)
        seconds = int(sec % 60)
        return datetime.time(hours, minutes, seconds)