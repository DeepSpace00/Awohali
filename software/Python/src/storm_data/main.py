#!/usr/bin/env python3
"""
Master script to download and plot Appalachian flood/precipitation data
"""

import argparse
from datetime import datetime
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.gridspec import GridSpec
import seaborn as sns
from scipy import stats
import warnings
import requests
import gzip
import shutil
from urllib.request import urlretrieve
from dataretrieval import nwis

warnings.filterwarnings('ignore')

# Set plotting style
sns.set_style('whitegrid')
plt.rcParams['figure.figsize'] = (14, 10)
plt.rcParams['font.size'] = 10


# ============================================================================
# DATA DOWNLOAD FUNCTIONS
# ============================================================================

def download_storm_events(start_year, end_year, output_dir='data/storm_events'):
    """Download NCEI Storm Events data"""
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    base_url = "https://www.ncei.noaa.gov/pub/data/swdi/stormevents/csvfiles/"

    for year in range(start_year, end_year + 1):
        print(f"Downloading Storm Events for {year}...")

        try:
            response = requests.get(base_url, timeout=30)
            response.raise_for_status()

            import re
            pattern = f'StormEvents_details-ftp_v1.0_d{year}_c\\d+\\.csv\\.gz'
            matches = re.findall(pattern, response.text)

            if matches:
                filename = matches[0]
                file_url = base_url + filename
                local_file = f"{output_dir}/{filename}"

                print(f"  Downloading {filename}...")
                r = requests.get(file_url, stream=True, timeout=60)
                r.raise_for_status()

                with open(local_file, 'wb') as f:
                    for chunk in r.iter_content(chunk_size=8192):
                        f.write(chunk)

                print(f"  Decompressing...")
                with gzip.open(local_file, 'rb') as f_in:
                    with open(local_file.replace('.gz', ''), 'wb') as f_out:
                        shutil.copyfileobj(f_in, f_out)

                Path(local_file).unlink()
                print(f"  ✓ Completed {year}")
            else:
                print(f"  ⚠ No data file found for {year}")

        except Exception as e:
            print(f"  ✗ Error downloading {year}: {e}")


def filter_appalachian_floods(data_dir='data/storm_events', output_file=None):
    """Filter for flood events in Appalachian states"""
    appalachian_states = [
        'NORTH CAROLINA', 'TENNESSEE', 'VIRGINIA', 'WEST VIRGINIA',
        'KENTUCKY', 'SOUTH CAROLINA', 'GEORGIA', 'ALABAMA'
    ]

    all_floods = []
    csv_files = list(Path(data_dir).glob('StormEvents_details*.csv'))

    if not csv_files:
        print(f"Warning: No CSV files found in {data_dir}")
        return pd.DataFrame()

    print(f"Processing {len(csv_files)} files...")

    for idx, csv_file in enumerate(csv_files):
        try:
            df = pd.read_csv(csv_file, low_memory=False)

            if 'EVENT_TYPE' not in df.columns or 'STATE' not in df.columns:
                continue

            floods = df[
                (df['EVENT_TYPE'].isin(['Flash Flood', 'Flood'])) &
                (df['STATE'].isin(appalachian_states))
                ].copy()

            if len(floods) > 0:
                # Construct BEGIN_DATE from year/month/day columns
                if 'BEGIN_YEARMONTH' in floods.columns and 'BEGIN_DAY' in floods.columns:
                    floods['BEGIN_DATE'] = pd.to_datetime(
                        floods['BEGIN_YEARMONTH'].astype(str) + floods['BEGIN_DAY'].astype(str).str.zfill(2),
                        format='%Y%m%d',
                        errors='coerce'
                    )

                all_floods.append(floods)
                if idx < 5 or len(floods) > 100:
                    print(f"  ✓ Found {len(floods)} flood events in {csv_file.name}")

        except Exception as e:
            print(f"  Error processing {csv_file.name}: {e}")

    if all_floods:
        result = pd.concat(all_floods, ignore_index=True)
        print(f"\n{'=' * 60}")
        print(f"✓ Total flood events found: {len(result)}")
        print(f"  Flash Floods: {len(result[result['EVENT_TYPE'] == 'Flash Flood'])}")
        print(f"  Floods: {len(result[result['EVENT_TYPE'] == 'Flood'])}")
        print(f"\n  By State:")
        for state, count in result['STATE'].value_counts().items():
            print(f"    {state}: {count}")
        print('=' * 60)

        if output_file:
            result.to_csv(output_file, index=False)
            print(f"\nSaved to: {output_file}")

        return result
    else:
        print("\n✗ No flood events found in any files!")
        return pd.DataFrame()


def get_known_appalachian_sites():
    """Return known USGS stream gages in Appalachian region"""
    known_sites = {
        '02138500': 'Catawba River near Marion, NC',
        '02111000': 'Yadkin River at Wilkesboro, NC',
        '03439000': 'French Broad River at Asheville, NC',
        '03500000': 'Little Tennessee River at Needmore, NC',
        '03465500': 'French Broad River at Newport, TN',
        '03528000': 'Clinch River at Speers Ferry, VA',
        '03497300': 'Little River near Townsend, TN',
        '02055000': 'Roanoke River near Roanoke, VA',
        '03171000': 'Little River at Graysonton, VA',
        '03524000': 'Clinch River at Cleveland, VA',
        '03182500': 'Greenbrier River at Alderson, WV',
        '03066000': 'Tygart Valley River at Philippi, WV',
        '03069500': 'Cheat River near Parsons, WV',
        '03281000': 'Kentucky River at Lock 2 at Lockport, KY',
        '03404500': 'Cumberland River near Harlan, KY',
        '02162500': 'Broad River near Carlisle, SC',
        '02334885': 'Chattahoochee River at Helen, GA',
        '02398000': 'Coosa River at Rome, GA',
    }

    sites_df = pd.DataFrame([
        {'site_no': site, 'station_nm': name}
        for site, name in known_sites.items()
    ])

    return sites_df


def find_appalachian_gages(state_codes=None, bbox=None):
    """Find active stream gages in Appalachian region"""

    if state_codes is None:
        state_codes = ['NC', 'TN', 'VA', 'WV', 'KY', 'SC', 'GA', 'AL']

    sites = []

    for state in state_codes:
        print(f"  Finding gages in {state}...")
        try:
            result = nwis.what_sites(
                stateCd=state,
                parameterCd='00060',
                siteStatus='active'
            )

            # what_sites returns a tuple (dataframe, metadata)
            if isinstance(result, tuple):
                site_info = result[0]
            else:
                site_info = result

            if site_info is not None and not site_info.empty:
                sites.append(site_info)
                print(f"    Found {len(site_info)} gages")
        except Exception as e:
            print(f"    Error: {e}")

    if sites:
        all_sites = pd.concat(sites, ignore_index=True)
        print(f"\n  Total gages found: {len(all_sites)}")

        if bbox and 'dec_long_va' in all_sites.columns and 'dec_lat_va' in all_sites.columns:
            original_count = len(all_sites)
            all_sites = all_sites[
                (all_sites['dec_long_va'] >= bbox[0]) &
                (all_sites['dec_lat_va'] >= bbox[1]) &
                (all_sites['dec_long_va'] <= bbox[2]) &
                (all_sites['dec_lat_va'] <= bbox[3])
                ]
            print(f"  After bbox filter: {len(all_sites)} gages")

        return all_sites
    else:
        print("\n  API query failed, using known site list as fallback...")
        return get_known_appalachian_sites()


def download_usgs_daily_discharge_api(site_numbers, start_date='2000-01-01', end_date=None):
    """Download daily discharge using USGS Water Data API (OGC)"""

    if end_date is None:
        end_date = pd.Timestamp.now().strftime('%Y-%m-%d')

    all_discharge = []

    print(f"\n  Downloading daily discharge using USGS NWIS...")
    print(f"  Date range: {start_date} to {end_date}")

    # Limit to reasonable number of sites
    site_numbers = site_numbers[:15]
    print(f"  Limited to {len(site_numbers)} sites")

    for idx, site in enumerate(site_numbers):
        print(f"    Site {idx + 1}/{len(site_numbers)}: {site}...", end='')

        try:
            # USGS NWIS web service
            base_url = f"https://waterdata.usgs.gov/nwis/dv"

            params = {
                'cb_00060': 'on',
                'format': 'rdb',
                'site_no': site,
                'begin_date': start_date,
                'end_date': end_date,
                'referred_module': 'sw'
            }

            response = requests.get(base_url, params=params, timeout=60)
            response.raise_for_status()

            # Parse RDB format
            lines = response.text.split('\n')

            # Find header line
            header_line = None
            for i, line in enumerate(lines):
                if line.startswith('agency_cd'):
                    header_line = i
                    break

            if header_line is None:
                print(f" ✗ No header found")
                if idx == 0:
                    print(f"\n    First 10 lines of response:")
                    for line in lines[:10]:
                        print(f"    {line}")
                continue

            # Read data starting from header
            from io import StringIO
            data_text = '\n'.join(lines[header_line:])
            df = pd.read_csv(StringIO(data_text), sep='\t', comment='#', skiprows=[1])

            # Debug first site
            if idx == 0:
                print(f"\n    DEBUG - Columns found: {df.columns.tolist()}")
                print(f"    First few rows:")
                print(df.head())
                print()

            # Find discharge column - various possible patterns
            discharge_col = None
            for col in df.columns:
                # Look for columns with 00060 (discharge parameter code)
                if '00060' in str(col):
                    discharge_col = col
                    break

            if discharge_col and len(df) > 0:
                print(f" using column: {discharge_col}...", end='')

                # Create clean dataframe
                clean_df = pd.DataFrame({
                    'date': pd.to_datetime(df['datetime']),
                    'discharge': pd.to_numeric(df[discharge_col], errors='coerce'),
                    'site_no': site
                })

                clean_df = clean_df.set_index('date')
                clean_df = clean_df.dropna(subset=['discharge'])

                if not clean_df.empty:
                    all_discharge.append(clean_df)
                    print(f" ✓ {len(clean_df)} records")
                else:
                    print(f" ✗ No valid data after cleaning")
            else:
                print(f" ✗ No discharge column (found: {df.columns.tolist()[:5]}...)")

        except Exception as e:
            print(f" ✗ Error: {str(e)[:80]}")
            if idx == 0:
                import traceback
                traceback.print_exc()

    if all_discharge:
        result = pd.concat(all_discharge)
        print(f"\n  ✓ Downloaded {len(result):,} daily discharge records")
        print(f"    Sites: {result['site_no'].nunique()}")
        print(f"    Date range: {result.index.min()} to {result.index.max()}")
        return result
    else:
        print(f"\n  ✗ No daily discharge data could be downloaded")
        return pd.DataFrame()


def calculate_annual_peaks(daily_df):
    """Calculate annual peak flows from daily discharge data"""

    if daily_df.empty:
        return pd.DataFrame()

    print("\n  Calculating annual peak flows from daily data...")

    # Make sure index is datetime
    if not isinstance(daily_df.index, pd.DatetimeIndex):
        daily_df.index = pd.to_datetime(daily_df.index)

    # Add year column
    daily_df['YEAR'] = daily_df.index.year

    # The discharge column should be 'discharge'
    if 'discharge' not in daily_df.columns:
        print("    ✗ Could not find discharge column")
        return pd.DataFrame()

    print(f"    Using discharge column")

    # Calculate annual peaks for each site
    annual_peaks = []

    for site in daily_df['site_no'].unique():
        site_data = daily_df[daily_df['site_no'] == site].copy()

        # Get annual maximum discharge
        annual_max = site_data.groupby('YEAR').agg({
            'discharge': 'max',
        }).reset_index()

        annual_max['site_no'] = site
        annual_max = annual_max.rename(columns={'discharge': 'peak_va'})

        # Remove any years with NaN
        annual_max = annual_max.dropna(subset=['peak_va'])

        annual_peaks.append(annual_max)

    if annual_peaks:
        result = pd.concat(annual_peaks, ignore_index=True)
        print(f"    ✓ Calculated peaks for {result['site_no'].nunique()} sites")
        print(f"    Year range: {result['YEAR'].min()} to {result['YEAR'].max()}")
        print(f"    Total records: {len(result)}")
        return result
    else:
        return pd.DataFrame()


def download_daily_discharge(site_numbers, start_date='1996-01-01', end_date=None):
    """Download daily discharge values"""

    if end_date is None:
        end_date = pd.Timestamp.now().strftime('%Y-%m-%d')

    all_discharge = []

    print(f"\n  Attempting to download daily discharge for {len(site_numbers)} sites...")

    # Limit to fewer sites for daily data
    site_numbers = site_numbers[:10]
    print(f"  Limited to {len(site_numbers)} sites for daily discharge")

    for idx, site in enumerate(site_numbers):
        print(f"    Downloading site {idx + 1}/{len(site_numbers)}: {site}...")
        try:
            result = nwis.get_record(
                sites=site,
                service='dv',
                start=start_date,
                end=end_date,
                parameterCd='00060'
            )

            # Handle tuple return
            if isinstance(result, tuple):
                df = result[0]
            else:
                df = result

            if df is not None and not df.empty:
                df['site_no'] = site
                all_discharge.append(df)
                print(f"      ✓ Got {len(df)} records")
        except Exception as e:
            print(f"      ✗ Error: {e}")

    if all_discharge:
        result = pd.concat(all_discharge, ignore_index=True)
        print(f"\n  ✓ Downloaded {len(result)} daily discharge records")
        return result
    else:
        print(f"\n  ✗ No daily discharge data could be downloaded")
        return pd.DataFrame()


def download_ghcn_daily(output_dir='data/ghcn'):
    """Download GHCN-Daily dataset"""
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    print("Downloading GHCN-Daily station list...")
    try:
        urlretrieve(
            "https://www.ncei.noaa.gov/pub/data/ghcn/daily/ghcnd-stations.txt",
            f"{output_dir}/ghcnd-stations.txt"
        )
    except Exception as e:
        print(f"Error downloading station list: {e}")
        return

    stations = pd.read_fwf(
        f"{output_dir}/ghcnd-stations.txt",
        colspecs=[(0, 11), (12, 20), (21, 30), (31, 37), (38, 40), (41, 71)],
        names=['ID', 'LATITUDE', 'LONGITUDE', 'ELEVATION', 'STATE', 'NAME']
    )

    app_states = ['NC', 'TN', 'VA', 'WV', 'KY', 'SC', 'GA', 'AL']
    app_stations = stations[stations['STATE'].isin(app_states)]

    app_bbox = [-84.5, 33.5, -79.0, 37.5]
    app_stations = app_stations[
        (app_stations['LONGITUDE'] >= app_bbox[0]) &
        (app_stations['LATITUDE'] >= app_bbox[1]) &
        (app_stations['LONGITUDE'] <= app_bbox[2]) &
        (app_stations['LATITUDE'] <= app_bbox[3])
        ]

    print(f"Found {len(app_stations)} stations in Appalachian region")
    print("Downloading station data (this may take a while)...")

    base_url = "https://www.ncei.noaa.gov/data/global-historical-climatology-network-daily/access/"

    downloaded = 0
    for idx, station_id in enumerate(app_stations['ID']):
        try:
            url = f"{base_url}{station_id}.csv"
            output_file = f"{output_dir}/{station_id}.csv"

            if Path(output_file).exists():
                continue

            urlretrieve(url, output_file)
            downloaded += 1

            if (idx + 1) % 10 == 0:
                print(f"  Downloaded {idx + 1}/{len(app_stations)} stations...")

        except Exception as e:
            if idx < 5:
                print(f"  Skipped {station_id}: {e}")

    print(f"Downloaded {downloaded} new station files")


def process_ghcn_precipitation(data_dir='data/ghcn'):
    """Process GHCN data for precipitation extremes"""
    all_data = []

    csv_files = list(Path(data_dir).glob('*.csv'))
    csv_files = [f for f in csv_files if 'stations' not in f.name]

    print(f"Processing {len(csv_files)} GHCN station files...")

    for idx, csv_file in enumerate(csv_files):
        try:
            df = pd.read_csv(csv_file, low_memory=False)

            if idx == 0:
                print(f"  Columns in first file: {df.columns.tolist()}")

            if 'PRCP' not in df.columns:
                continue

            precip = df[['STATION', 'DATE', 'PRCP']].copy()
            precip = precip.dropna(subset=['PRCP'])

            if precip.empty:
                continue

            precip = precip.rename(columns={'STATION': 'ID'})
            precip['DATE'] = pd.to_datetime(precip['DATE'], errors='coerce')

            if precip['PRCP'].max() > 1000:
                precip['VALUE'] = precip['PRCP'] / 10
            else:
                precip['VALUE'] = precip['PRCP']

            precip['ELEMENT'] = 'PRCP'
            precip['QFLAG'] = None
            precip = precip.dropna(subset=['DATE'])

            if not precip.empty:
                all_data.append(precip[['ID', 'DATE', 'ELEMENT', 'VALUE', 'QFLAG']])

            if (idx + 1) % 500 == 0:
                print(f"  Processed {idx + 1}/{len(csv_files)} files... ({len(all_data)} files with data)")

        except Exception as e:
            if idx < 10:
                print(f"  Error processing {csv_file.name}: {e}")
            continue

    if all_data:
        result = pd.concat(all_data, ignore_index=True)
        print(f"\n✓ Total precipitation records: {len(result):,}")
        print(f"  Date range: {result['DATE'].min()} to {result['DATE'].max()}")
        print(f"  Number of stations: {result['ID'].nunique()}")
        print(f"  Mean daily precip: {result['VALUE'].mean():.2f} mm")
        return result
    else:
        print("\n✗ No precipitation data could be processed")
        return pd.DataFrame()


# ============================================================================
# PLOTTING FUNCTIONS
# ============================================================================

def plot_flood_trends(floods_df, output_dir='plots'):
    """Create comprehensive flood trend visualizations"""
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    floods_df['BEGIN_DATE'] = pd.to_datetime(floods_df['BEGIN_DATE'])
    floods_df['YEAR'] = floods_df['BEGIN_DATE'].dt.year

    fig = plt.figure(figsize=(16, 12))
    gs = GridSpec(3, 2, figure=fig, hspace=0.3, wspace=0.3)

    # Panel 1: Annual flood frequency
    ax1 = fig.add_subplot(gs[0, :])
    annual_counts = floods_df.groupby(['YEAR', 'EVENT_TYPE']).size().unstack(fill_value=0)

    ax1.plot(annual_counts.index, annual_counts.get('Flash Flood', 0),
             marker='o', linewidth=2, label='Flash Floods', color='#e74c3c')
    ax1.plot(annual_counts.index, annual_counts.get('Flood', 0),
             marker='s', linewidth=2, label='Floods', color='#3498db')

    for event_type, color in [('Flash Flood', '#e74c3c'), ('Flood', '#3498db')]:
        if event_type in annual_counts.columns:
            x = annual_counts.index.values
            y = annual_counts[event_type].values
            z = np.polyfit(x, y, 1)
            p = np.poly1d(z)
            ax1.plot(x, p(x), "--", alpha=0.5, color=color, linewidth=1.5)

            slope, intercept, r_value, p_value, std_err = stats.linregress(x, y)
            significance = "**" if p_value < 0.01 else "*" if p_value < 0.05 else ""
            print(f"{event_type} trend: {slope:.2f} events/year (p={p_value:.4f}) {significance}")

    ax1.set_xlabel('Year', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Number of Events', fontsize=12, fontweight='bold')
    ax1.set_title('Annual Flood Event Frequency in Appalachia', fontsize=14, fontweight='bold')
    ax1.legend(loc='upper left', fontsize=11)
    ax1.grid(True, alpha=0.3)

    # Panel 2: Economic damages
    ax2 = fig.add_subplot(gs[1, 0])

    def parse_damage(value):
        if pd.isna(value) or value == '' or value == 0:
            return 0
        try:
            return float(value)
        except (ValueError, TypeError):
            if isinstance(value, str):
                value = value.upper().strip()
                multipliers = {'K': 1e3, 'M': 1e6, 'B': 1e9}
                for suffix, mult in multipliers.items():
                    if suffix in value:
                        try:
                            return float(value.replace(suffix, '')) * mult
                        except ValueError:
                            return 0
            return 0

    floods_df['DAMAGE_PROPERTY_NUM'] = floods_df['DAMAGE_PROPERTY'].apply(parse_damage)
    floods_df['DAMAGE_CROPS_NUM'] = floods_df['DAMAGE_CROPS'].apply(parse_damage)
    floods_df['TOTAL_DAMAGE'] = floods_df['DAMAGE_PROPERTY_NUM'] + floods_df['DAMAGE_CROPS_NUM']

    annual_damage = floods_df.groupby('YEAR')['TOTAL_DAMAGE'].sum() / 1e6

    if annual_damage.sum() > 0:
        ax2.bar(annual_damage.index, annual_damage.values, color='#e67e22', alpha=0.7, edgecolor='black')
        ax2.set_xlabel('Year', fontsize=12, fontweight='bold')
        ax2.set_ylabel('Total Damage (Million $)', fontsize=12, fontweight='bold')
        ax2.set_title('Annual Flood Damage', fontsize=13, fontweight='bold')
        ax2.grid(True, alpha=0.3, axis='y')
    else:
        ax2.text(0.5, 0.5, 'Damage data not available',
                 ha='center', va='center', transform=ax2.transAxes, fontsize=12)
        ax2.set_title('Annual Flood Damage', fontsize=13, fontweight='bold')

    # Panel 3: Casualties
    ax3 = fig.add_subplot(gs[1, 1])

    for col in ['DEATHS_DIRECT', 'DEATHS_INDIRECT', 'INJURIES_DIRECT', 'INJURIES_INDIRECT']:
        if col in floods_df.columns:
            floods_df[col] = pd.to_numeric(floods_df[col], errors='coerce').fillna(0)

    casualties = floods_df.groupby('YEAR')[['DEATHS_DIRECT', 'DEATHS_INDIRECT',
                                            'INJURIES_DIRECT', 'INJURIES_INDIRECT']].sum()
    casualties['TOTAL_DEATHS'] = casualties['DEATHS_DIRECT'] + casualties['DEATHS_INDIRECT']
    casualties['TOTAL_INJURIES'] = casualties['INJURIES_DIRECT'] + casualties['INJURIES_INDIRECT']

    ax3.bar(casualties.index - 0.2, casualties['TOTAL_DEATHS'],
            width=0.4, label='Deaths', color='#c0392b', alpha=0.8)
    ax3.bar(casualties.index + 0.2, casualties['TOTAL_INJURIES'],
            width=0.4, label='Injuries', color='#f39c12', alpha=0.8)
    ax3.set_xlabel('Year', fontsize=12, fontweight='bold')
    ax3.set_ylabel('Count', fontsize=12, fontweight='bold')
    ax3.set_title('Flood-Related Casualties', fontsize=13, fontweight='bold')
    ax3.legend(fontsize=11)
    ax3.grid(True, alpha=0.3, axis='y')

    # Panel 4: Seasonal distribution
    ax4 = fig.add_subplot(gs[2, 0])

    floods_df['MONTH'] = floods_df['BEGIN_DATE'].dt.month
    monthly_counts = floods_df.groupby('MONTH').size()
    month_names = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun',
                   'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']

    colors = plt.cm.RdYlBu_r(np.linspace(0.2, 0.8, 12))
    ax4.bar(monthly_counts.index, monthly_counts.values, color=colors, edgecolor='black')
    ax4.set_xticks(range(1, 13))
    ax4.set_xticklabels(month_names, rotation=45)
    ax4.set_xlabel('Month', fontsize=12, fontweight='bold')
    ax4.set_ylabel('Total Events (All Years)', fontsize=12, fontweight='bold')
    ax4.set_title('Seasonal Distribution of Flood Events', fontsize=13, fontweight='bold')
    ax4.grid(True, alpha=0.3, axis='y')

    # Panel 5: State distribution
    ax5 = fig.add_subplot(gs[2, 1])

    state_counts = floods_df['STATE'].value_counts().head(8)
    ax5.barh(state_counts.index, state_counts.values, color='#16a085', edgecolor='black')
    ax5.set_xlabel('Number of Events', fontsize=12, fontweight='bold')
    ax5.set_ylabel('State', fontsize=12, fontweight='bold')
    ax5.set_title('Flood Events by State', fontsize=13, fontweight='bold')
    ax5.grid(True, alpha=0.3, axis='x')

    plt.savefig(f'{output_dir}/flood_trends_comprehensive.png', dpi=300, bbox_inches='tight')
    print(f"Saved: {output_dir}/flood_trends_comprehensive.png")
    plt.close()


def plot_precipitation_trends(precip_df, output_dir='plots'):
    """Plot precipitation trends from GHCN-Daily data"""
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    precip_clean = precip_df[precip_df['QFLAG'].isna()].copy()
    precip_clean['YEAR'] = precip_clean['DATE'].dt.year
    precip_clean['VALUE_in'] = precip_clean['VALUE'] / 25.4

    fig = plt.figure(figsize=(16, 10))
    gs = GridSpec(2, 2, figure=fig, hspace=0.3, wspace=0.3)

    # Panel 1: Annual total precipitation
    ax1 = fig.add_subplot(gs[0, :])

    annual_precip = precip_clean.groupby(['YEAR', 'ID'])['VALUE_in'].sum().groupby('YEAR').mean()

    ax1.plot(annual_precip.index, annual_precip.values, marker='o',
             linewidth=2, markersize=6, color='#2980b9')

    x = annual_precip.index.values
    y = annual_precip.values
    z = np.polyfit(x, y, 1)
    p = np.poly1d(z)
    ax1.plot(x, p(x), "r--", alpha=0.8, linewidth=2, label='Trend')

    slope, intercept, r_value, p_value, std_err = stats.linregress(x, y)
    ax1.text(0.02, 0.98, f'Trend: {slope:.2f} in/year\np-value: {p_value:.4f}',
             transform=ax1.transAxes, fontsize=11, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    ax1.set_xlabel('Year', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Mean Annual Precipitation (inches)', fontsize=12, fontweight='bold')
    ax1.set_title('Annual Precipitation Trends in Appalachia', fontsize=14, fontweight='bold')
    ax1.legend(fontsize=11)
    ax1.grid(True, alpha=0.3)

    # Panel 2: Extreme precipitation days
    ax2 = fig.add_subplot(gs[1, 0])

    extreme_days = precip_clean[precip_clean['VALUE_in'] > 2.0].groupby(['YEAR', 'ID']).size()
    annual_extreme = extreme_days.groupby('YEAR').mean()

    ax2.bar(annual_extreme.index, annual_extreme.values, color='#e74c3c', alpha=0.7, edgecolor='black')

    x = annual_extreme.index.values
    y = annual_extreme.values
    z = np.polyfit(x, y, 1)
    p = np.poly1d(z)
    ax2.plot(x, p(x), "k--", alpha=0.8, linewidth=2, label='Trend')

    ax2.set_xlabel('Year', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Mean Days per Station', fontsize=12, fontweight='bold')
    ax2.set_title('Heavy Precipitation Days (>2 inches/day)', fontsize=13, fontweight='bold')
    ax2.legend(fontsize=11)
    ax2.grid(True, alpha=0.3, axis='y')

    # Panel 3: Maximum 24-hour precipitation
    ax3 = fig.add_subplot(gs[1, 1])

    annual_max_precip = precip_clean.groupby(['YEAR', 'ID'])['VALUE_in'].max().groupby('YEAR').mean()

    ax3.scatter(annual_max_precip.index, annual_max_precip.values,
                alpha=0.6, s=80, color='#8e44ad')

    x = annual_max_precip.index.values
    y = annual_max_precip.values
    z = np.polyfit(x, y, 1)
    p = np.poly1d(z)
    ax3.plot(x, p(x), "r--", alpha=0.8, linewidth=2, label='Trend')

    ax3.set_xlabel('Year', fontsize=12, fontweight='bold')
    ax3.set_ylabel('Maximum Daily Precipitation (inches)', fontsize=12, fontweight='bold')
    ax3.set_title('Annual Maximum 24-hour Precipitation', fontsize=13, fontweight='bold')
    ax3.legend(fontsize=11)
    ax3.grid(True, alpha=0.3)

    plt.savefig(f'{output_dir}/precipitation_trends.png', dpi=300, bbox_inches='tight')
    print(f"Saved: {output_dir}/precipitation_trends.png")
    plt.close()


def plot_streamflow_trends(peaks_df, output_dir='plots'):
    """Plot USGS streamflow trends from annual peaks"""
    if peaks_df.empty:
        print("  Skipping streamflow plots - no data available")
        return

    Path(output_dir).mkdir(parents=True, exist_ok=True)

    print(f"  Plotting streamflow trends...")
    print(f"    Available sites: {peaks_df['site_no'].nunique()}")

    fig, axes = plt.subplots(2, 2, figsize=(16, 12))

    sites = peaks_df['site_no'].unique()[:4]
    plot_count = 0

    for idx, site in enumerate(sites):
        if idx >= 4:
            break

        ax = axes[idx // 2, idx % 2]
        site_peaks = peaks_df[peaks_df['site_no'] == site].copy()

        # Remove any NaN values
        site_peaks = site_peaks.dropna(subset=['YEAR', 'peak_va'])

        if len(site_peaks) < 3:
            ax.text(0.5, 0.5, f'Site {site}\nInsufficient data ({len(site_peaks)} years)',
                    ha='center', va='center', transform=ax.transAxes, fontsize=12)
            continue

        # Plot
        ax.scatter(site_peaks['YEAR'], site_peaks['peak_va'], alpha=0.6, s=50, color='#3498db')

        x = site_peaks['YEAR'].values
        y = site_peaks['peak_va'].values

        z = np.polyfit(x, y, 1)
        p = np.poly1d(z)
        ax.plot(x, p(x), "r--", alpha=0.8, linewidth=2, label='Trend')

        slope, intercept, r_value, p_value, std_err = stats.linregress(x, y)

        ax.set_xlabel('Year', fontsize=11, fontweight='bold')
        ax.set_ylabel('Annual Peak Discharge (cfs)', fontsize=11, fontweight='bold')
        ax.set_title(f'USGS Site {site}\nTrend: {slope:.1f} cfs/year (p={p_value:.3f}, n={len(x)})',
                     fontsize=12, fontweight='bold')
        ax.legend()
        ax.grid(True, alpha=0.3)
        plot_count += 1

        print(f"    ✓ Plotted site {site}: {len(site_peaks)} years, trend={slope:.1f} cfs/yr")

    # Hide unused subplots
    for idx in range(plot_count, 4):
        axes[idx // 2, idx % 2].axis('off')

    if plot_count > 0:
        plt.tight_layout()
        plt.savefig(f'{output_dir}/streamflow_trends.png', dpi=300, bbox_inches='tight')
        print(f"  ✓ Saved: {output_dir}/streamflow_trends.png ({plot_count} sites)")
    else:
        print("  ✗ No streamflow plots could be generated")
        plt.close()

    plt.close()


def plot_combined_analysis(floods_df, precip_df, peaks_df, output_dir='plots'):
    """Create combined analysis figure"""
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    floods_df['YEAR'] = pd.to_datetime(floods_df['BEGIN_DATE']).dt.year
    annual_floods = floods_df.groupby('YEAR').size()

    precip_clean = precip_df[precip_df['QFLAG'].isna()].copy()
    precip_clean['YEAR'] = precip_clean['DATE'].dt.year
    precip_clean['VALUE_in'] = precip_clean['VALUE'] / 25.4

    annual_precip = precip_clean.groupby(['YEAR', 'ID'])['VALUE_in'].sum().groupby('YEAR').mean()
    extreme_days = precip_clean[precip_clean['VALUE_in'] > 2.0].groupby(['YEAR', 'ID']).size().groupby('YEAR').mean()

    fig = plt.figure(figsize=(16, 10))
    gs = GridSpec(2, 2, figure=fig, hspace=0.3, wspace=0.3)

    # Floods vs Precipitation
    ax1 = fig.add_subplot(gs[0, 0])
    merged = pd.DataFrame({'floods': annual_floods, 'precip': annual_precip}).dropna()

    ax1.scatter(merged['precip'], merged['floods'], alpha=0.6, s=100, color='#3498db')

    if len(merged) > 2:
        corr, p_val = stats.pearsonr(merged['precip'], merged['floods'])
        z = np.polyfit(merged['precip'], merged['floods'], 1)
        p = np.poly1d(z)
        x_line = np.linspace(merged['precip'].min(), merged['precip'].max(), 100)
        ax1.plot(x_line, p(x_line), "r--", alpha=0.8, linewidth=2)

        ax1.text(0.05, 0.95, f'r = {corr:.3f}\np = {p_val:.4f}',
                 transform=ax1.transAxes, fontsize=11, verticalalignment='top',
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    ax1.set_xlabel('Annual Precipitation (inches)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Number of Flood Events', fontsize=12, fontweight='bold')
    ax1.set_title('Flood Events vs Annual Precipitation', fontsize=13, fontweight='bold')
    ax1.grid(True, alpha=0.3)

    # Floods vs Extreme Days
    ax2 = fig.add_subplot(gs[0, 1])
    merged2 = pd.DataFrame({'floods': annual_floods, 'extreme': extreme_days}).dropna()

    ax2.scatter(merged2['extreme'], merged2['floods'], alpha=0.6, s=100, color='#e74c3c')

    if len(merged2) > 2:
        corr2, p_val2 = stats.pearsonr(merged2['extreme'], merged2['floods'])
        z2 = np.polyfit(merged2['extreme'], merged2['floods'], 1)
        p2 = np.poly1d(z2)
        x_line2 = np.linspace(merged2['extreme'].min(), merged2['extreme'].max(), 100)
        ax2.plot(x_line2, p2(x_line2), "r--", alpha=0.8, linewidth=2)

        ax2.text(0.05, 0.95, f'r = {corr2:.3f}\np = {p_val2:.4f}',
                 transform=ax2.transAxes, fontsize=11, verticalalignment='top',
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    ax2.set_xlabel('Heavy Precip Days (>2 in/day)', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Number of Flood Events', fontsize=12, fontweight='bold')
    ax2.set_title('Flood Events vs Heavy Precipitation Days', fontsize=13, fontweight='bold')
    ax2.grid(True, alpha=0.3)

    # Normalized trends
    ax3 = fig.add_subplot(gs[1, :])

    years = sorted(set(annual_floods.index) & set(annual_precip.index) & set(extreme_days.index))

    def normalize(series):
        return (series - series.min()) / (series.max() - series.min())

    floods_norm = normalize(annual_floods.loc[years])
    precip_norm = normalize(annual_precip.loc[years])
    extreme_norm = normalize(extreme_days.loc[years])

    ax3.plot(years, floods_norm, marker='o', linewidth=2, label='Flood Events', color='#2c3e50')
    ax3.plot(years, precip_norm, marker='s', linewidth=2, label='Annual Precipitation', color='#3498db')
    ax3.plot(years, extreme_norm, marker='^', linewidth=2, label='Heavy Precip Days', color='#e74c3c')

    ax3.set_xlabel('Year', fontsize=12, fontweight='bold')
    ax3.set_ylabel('Normalized Value (0-1)', fontsize=12, fontweight='bold')
    ax3.set_title('Normalized Trends: Floods, Precipitation, and Extreme Events',
                  fontsize=14, fontweight='bold')
    ax3.legend(loc='upper left', fontsize=11)
    ax3.grid(True, alpha=0.3)

    plt.savefig(f'{output_dir}/combined_analysis.png', dpi=300, bbox_inches='tight')
    print(f"Saved: {output_dir}/combined_analysis.png")
    plt.close()


def create_summary_report(floods_df, precip_df, peaks_df, output_dir='plots'):
    """Create text summary report"""
    report = []
    report.append("=" * 80)
    report.append("APPALACHIAN FLOODING AND PRECIPITATION TRENDS - SUMMARY REPORT")
    report.append("=" * 80)
    report.append("")

    floods_df['YEAR'] = pd.to_datetime(floods_df['BEGIN_DATE']).dt.year
    annual_floods = floods_df.groupby('YEAR').size()

    report.append("FLOOD EVENT STATISTICS:")
    report.append(f"  Total flood events: {len(floods_df)}")
    report.append(f"  Flash floods: {len(floods_df[floods_df['EVENT_TYPE'] == 'Flash Flood'])}")
    report.append(f"  Regular floods: {len(floods_df[floods_df['EVENT_TYPE'] == 'Flood'])}")
    report.append(f"  Time period: {floods_df['YEAR'].min()} - {floods_df['YEAR'].max()}")
    report.append(f"  Mean events per year: {annual_floods.mean():.1f}")

    x = annual_floods.index.values
    y = annual_floods.values
    slope, intercept, r_value, p_value, std_err = stats.linregress(x, y)

    report.append(f"\n  Flood trend: {slope:.2f} events/year")
    report.append(f"  R-squared: {r_value ** 2:.3f}")
    report.append(f"  P-value: {p_value:.4f}")
    if p_value < 0.05:
        report.append(f"  ** STATISTICALLY SIGNIFICANT TREND **")

    precip_clean = precip_df[precip_df['QFLAG'].isna()].copy()
    precip_clean['YEAR'] = precip_clean['DATE'].dt.year
    precip_clean['VALUE_in'] = precip_clean['VALUE'] / 25.4

    annual_precip = precip_clean.groupby(['YEAR', 'ID'])['VALUE_in'].sum().groupby('YEAR').mean()

    report.append("\n" + "=" * 80)
    report.append("PRECIPITATION STATISTICS:")
    report.append(f"  Mean annual precipitation: {annual_precip.mean():.1f} inches")
    report.append(f"  Time period: {precip_clean['YEAR'].min()} - {precip_clean['YEAR'].max()}")

    x_p = annual_precip.index.values
    y_p = annual_precip.values
    slope_p, intercept_p, r_value_p, p_value_p, std_err_p = stats.linregress(x_p, y_p)

    report.append(f"\n  Precipitation trend: {slope_p:.3f} inches/year")
    report.append(f"  R-squared: {r_value_p ** 2:.3f}")
    report.append(f"  P-value: {p_value_p:.4f}")

    extreme_days = precip_clean[precip_clean['VALUE_in'] > 2.0].groupby(['YEAR', 'ID']).size().groupby('YEAR').mean()
    report.append(f"\n  Mean heavy precip days (>2 in): {extreme_days.mean():.1f} per year")

    if not peaks_df.empty:
        report.append("\n" + "=" * 80)
        report.append("STREAMFLOW STATISTICS:")
        report.append(f"  Number of stream gages: {peaks_df['site_no'].nunique()}")
        report.append(f"  Total peak flow records: {len(peaks_df)}")

    report.append("\n" + "=" * 80)
    report.append("Report generated: " + datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    report.append("=" * 80)

    with open(f'{output_dir}/summary_report.txt', 'w') as f:
        f.write('\n'.join(report))

    print('\n'.join(report))


# ============================================================================
# MAIN FUNCTION
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description='Download and plot Appalachian flood/precip data')
    parser.add_argument('--start-year', type=int, default=1996)
    parser.add_argument('--end-year', type=int, default=datetime.now().year)
    parser.add_argument('--output-dir', default='data')
    parser.add_argument('--plot-dir', default='plots')
    parser.add_argument('--skip-download', action='store_true',
                        help='Skip download and only plot existing data')
    args = parser.parse_args()

    Path(args.output_dir).mkdir(parents=True, exist_ok=True)
    Path(args.plot_dir).mkdir(parents=True, exist_ok=True)

    floods_file = f"{args.output_dir}/appalachian_floods.csv"
    precip_file = f"{args.output_dir}/ghcn_precipitation.csv"

    if args.skip_download:
        if not Path(floods_file).exists():
            print(f"ERROR: {floods_file} not found!")
            print("You need to run without --skip-download first to download the data.")
            return
        if not Path(precip_file).exists():
            print(f"ERROR: {precip_file} not found!")
            print("You need to run without --skip-download first to download the data.")
            return

    if not args.skip_download:
        print("=" * 80)
        print("DOWNLOADING DATA...")
        print("=" * 80)

        print("\n1. Downloading Storm Events Database...")
        download_storm_events(args.start_year, args.end_year)

        print("\n2. Filtering for Appalachian floods...")
        floods_df = filter_appalachian_floods(output_file=floods_file)

        if floods_df.empty:
            print("ERROR: No flood data was downloaded. Exiting.")
            return

        print("\n3. Finding USGS stream gages...")
        app_bbox = [-84.5, 33.5, -79.0, 37.5]
        gages = find_appalachian_gages(
            state_codes=['NC', 'TN', 'VA', 'WV', 'SC', 'GA'],
            bbox=app_bbox
        )

        if not gages.empty:
            gages.to_csv(f"{args.output_dir}/appalachian_gages.csv", index=False)

            print("\n4. Downloading USGS daily discharge (OGC API)...")
            site_list = gages['site_no'].tolist()
            daily = download_usgs_daily_discharge_api(site_list, start_date="2000-01-01")

            if not daily.empty:
                daily.to_csv(f"{args.output_dir}/usgs_daily_discharge.csv")

                print("\n5. Calculating annual peaks from daily data...")
                peaks = calculate_annual_peaks(daily)

                if not peaks.empty:
                    peaks.to_csv(f"{args.output_dir}/usgs_annual_peaks.csv", index=False)
        else:
            print("Warning: No USGS gages found")

        print("\n6. Downloading GHCN-Daily precipitation data...")
        download_ghcn_daily()

        print("\n7. Processing GHCN precipitation records...")
        precip = process_ghcn_precipitation()

        if not precip.empty:
            precip.to_csv(precip_file, index=False)
        else:
            print("ERROR: No precipitation data was processed. Exiting.")
            return

        print("\n" + "=" * 80)
        print("DATA DOWNLOAD COMPLETE")
        print("=" * 80)

    # Load data for plotting
    print("\n" + "=" * 80)
    print("LOADING DATA FOR PLOTTING...")
    print("=" * 80)

    try:
        floods_df = pd.read_csv(floods_file)
        print(f"Loaded {len(floods_df)} flood events")

        if 'BEGIN_DATE' not in floods_df.columns:
            print("  Constructing BEGIN_DATE from year/month/day columns...")
            floods_df['BEGIN_DATE'] = pd.to_datetime(
                floods_df['BEGIN_YEARMONTH'].astype(str) +
                floods_df['BEGIN_DAY'].astype(str).str.zfill(2),
                format='%Y%m%d',
                errors='coerce'
            )
        else:
            floods_df['BEGIN_DATE'] = pd.to_datetime(floods_df['BEGIN_DATE'])

    except FileNotFoundError:
        print(f"ERROR: {floods_file} not found!")
        return

    try:
        precip_df = pd.read_csv(precip_file, parse_dates=['DATE'])
        print(f"Loaded {len(precip_df)} precipitation records")
    except FileNotFoundError:
        print(f"ERROR: {precip_file} not found!")
        return

    try:
        peaks_df = pd.read_csv(f"{args.output_dir}/usgs_annual_peaks.csv")
        print(f"✓ Loaded {len(peaks_df)} annual peak records from {peaks_df['site_no'].nunique()} sites")
    except FileNotFoundError:
        print("Note: USGS data not found, skipping streamflow plots")
        peaks_df = pd.DataFrame()

    # Generate plots
    print("\n" + "=" * 80)
    print("GENERATING PLOTS...")
    print("=" * 80)

    print("\n1. Plotting flood trends...")
    plot_flood_trends(floods_df, args.plot_dir)

    print("\n2. Plotting precipitation trends...")
    plot_precipitation_trends(precip_df, args.plot_dir)

    if not peaks_df.empty:
        print("\n3. Plotting streamflow trends...")
        plot_streamflow_trends(peaks_df, args.plot_dir)

    print("\n4. Creating combined analysis...")
    plot_combined_analysis(floods_df, precip_df, peaks_df, args.plot_dir)

    print("\n5. Generating summary report...")
    create_summary_report(floods_df, precip_df, peaks_df, args.plot_dir)

    print("\n" + "=" * 80)
    print("ANALYSIS COMPLETE!")
    print(f"Plots saved to: {args.plot_dir}/")
    print("=" * 80)


if __name__ == '__main__':
    main()