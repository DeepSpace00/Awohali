from pathlib import Path
from urllib.request import urlretrieve
import tarfile

import pandas as pd


def download_ghcn_daily(output_dir='data/ghcn'):
    """Download GHCN-Daily dataset"""
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    base_url = "https://www.ncei.noaa.gov/data/global-historical-climatology-network-daily/access/"

    # Download station list
    urlretrieve(
        "https://www.ncei.noaa.gov/pub/data/ghcn/daily/ghcnd-stations.txt",
        f"{output_dir}/ghcnd-stations.txt"
    )

    # Read stations and filter for Appalachian region
    stations = pd.read_fwf(
        f"{output_dir}/ghcnd-stations.txt",
        colspecs=[(0, 11), (12, 20), (21, 30), (31, 37), (38, 40), (41, 71)],
        names=['ID', 'LATITUDE', 'LONGITUDE', 'ELEVATION', 'STATE', 'NAME']
    )

    # Filter for Appalachian states
    app_states = ['NC', 'TN', 'VA', 'WV', 'KY', 'SC', 'GA', 'AL']
    app_stations = stations[stations['STATE'].isin(app_states)]

    # Download individual station files
    for station_id in app_stations['ID']:
        try:
            url = f"{base_url}{station_id}.csv"
            output_file = f"{output_dir}/{station_id}.csv"
            print(f"Downloading {station_id}...")
            urlretrieve(url, output_file)
        except Exception as e:
            print(f"Error with {station_id}: {e}")


def process_ghcn_precipitation(data_dir='data/ghcn'):
    """Process GHCN data for precipitation extremes"""
    all_data = []

    for csv_file in Path(data_dir).glob('*.csv'):
        df = pd.read_csv(csv_file, names=['ID', 'DATE', 'ELEMENT', 'VALUE', 'MFLAG', 'QFLAG', 'SFLAG', 'TIME'])

        # Filter for precipitation only
        precip = df[df['ELEMENT'] == 'PRCP'].copy()
        precip['VALUE'] = precip['VALUE'] / 10  # Convert to mm
        precip['DATE'] = pd.to_datetime(precip['DATE'])

        all_data.append(precip)

    return pd.concat(all_data, ignore_index=True)