# Auxiliar functions for transform GPS Coordinates

from pyproj import Proj, Transformer    # Library for the coordenates calc

def ltp_to_wgs84(origin_lat, origin_lon, x, y):
    source_proj = f"+proj=ortho +lat_0={origin_lat} +lon_0={origin_lon}"
    transformer = Transformer.from_proj(Proj(source_proj), Proj("EPSG:4326"), always_xy=True)

    lon, lat = transformer.transform(x, y)
    return lat, lon