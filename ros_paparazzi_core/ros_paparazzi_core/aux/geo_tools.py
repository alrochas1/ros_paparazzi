# Auxiliar functions for transform GPS Coordinates

from pyproj import Proj, Transformer, CRS    # Library for the coordenates calc

def ltp_to_wgs84(origin_lat, origin_lon, x, y):
    source_proj = f"+proj=ortho +lat_0={origin_lat} +lon_0={origin_lon}"
    transformer = Transformer.from_proj(Proj(source_proj), Proj("EPSG:4326"), always_xy=True)

    lon, lat = transformer.transform(x, y)
    return lat, lon


def wgs84_to_ltp(origin_lat, origin_lon, lat, lon):
    source_proj = f"+proj=ortho +lat_0={origin_lat} +lon_0={origin_lon}"
    transformer = Transformer.from_proj(Proj("EPSG:4326"), Proj(source_proj), always_xy=True)
    
    x, y = transformer.transform(lon, lat)
    return x, y


def wgs84_to_epsg(lat, lon):
    crs = CRS.from_epsg(3857)
    transformer = Transformer.from_crs(crs.geodetic_crs, crs)

    return transformer.transform(lat, lon)


# REVISAR
def ecef_to_ltp(origin_lat, origin_lon, x_ecef, y_ecef, z_ecef):

    ecef_proj = Proj(proj="geocent", ellps="WGS84", datum="WGS84")
    ltp_proj = Proj(proj="tmerc", lat_0=origin_lat, lon_0=origin_lon, ellps="WGS84", datum="WGS84")
    
    transformer = Transformer.from_proj(ecef_proj, ltp_proj)
    
    x_ltp, y_ltp, z_ltp = transformer.transform(x_ecef, y_ecef, z_ecef, radians=False)
    
    return x_ltp, y_ltp
