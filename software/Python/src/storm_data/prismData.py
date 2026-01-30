import rasterio
from rasterio.mask import mask
import geopandas as gpd


def download_prism_bulk(start_year, end_year, variable='ppt', temporal='monthly'):
    """
    Download PRISM data via their bulk download
    Requires registration at https://prism.oregonstate.edu/
    """
    # You'll need to use their bulk download service or FTP
    # Example using their HTTP service:

    base_url = "https://prism.oregonstate.edu/fetchData.php"

    for year in range(start_year, end_year + 1):
        for month in range(1, 13):
            params = {
                'type': 'monthly',
                'variable': variable,
                'year': year,
                'month': month,
                'format': 'bil'
            }
            # This is simplified - actual PRISM download requires authentication
            print(f"Would download PRISM {variable} for {year}-{month:02d}")