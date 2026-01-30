import requests
import pandas as pd
from pathlib import Path
import gzip
import shutil


def download_storm_events(start_year, end_year, output_dir='data/storm_events'):
    """Download NCEI Storm Events data"""
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    base_url = "https://www.ncei.noaa.gov/pub/data/swdi/stormevents/csvfiles/"

    for year in range(start_year, end_year + 1):
        # Details file (main dataset)
        details_pattern = f"StormEvents_details-ftp_v1.0_d{year}_c*.csv.gz"

        # Get the actual filename (changes with updates)
        response = requests.get(base_url)
        files = [line for line in response.text.split() if f'd{year}_c' in line and 'details' in line]

        if files:
            filename = files[0].split('"')[1]  # Parse HTML for filename
            file_url = base_url + filename
            local_file = f"{output_dir}/{filename}"

            print(f"Downloading {filename}...")
            r = requests.get(file_url, stream=True)
            with open(local_file, 'wb') as f:
                f.write(r.content)

            # Decompress
            with gzip.open(local_file, 'rb') as f_in:
                with open(local_file.replace('.gz', ''), 'wb') as f_out:
                    shutil.copyfileobj(f_in, f_out)


def filter_appalachian_floods(data_dir='data/storm_events'):
    """Filter for flood events in Appalachian counties"""
    # Appalachian county FIPS codes (you'll need to compile this list)
    appalachian_states = ['NC', 'TN', 'VA', 'WV', 'KY', 'SC', 'GA', 'AL']

    all_floods = []
    for csv_file in Path(data_dir).glob('StormEvents_details*.csv'):
        df = pd.read_csv(csv_file, low_memory=False)

        # Filter for floods in Appalachian states
        floods = df[
            (df['EVENT_TYPE'].isin(['Flash Flood', 'Flood'])) &
            (df['STATE'].isin(appalachian_states))
            ]
        all_floods.append(floods)

    return pd.concat(all_floods, ignore_index=True)