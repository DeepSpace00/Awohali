import pandas as pd
from dataretrieval import waterdata


def download_usgs_peaks(site_numbers, start_date='1950-01-01'):
    """Download peak streamflow data from USGS using waterdata"""
    all_peaks = []

    for site in site_numbers:
        print(f"Downloading site {site}...")
        try:
            # Use waterdata.get_pmcodes for peak flow data
            df = waterdata.get_pmcodes(
                sites=site,
                parameterCd='00060',  # Discharge
                start=start_date
            )
            if not df.empty:
                df['site_no'] = site
                all_peaks.append(df)
        except Exception as e:
            print(f"Error downloading {site}: {e}")

    if all_peaks:
        return pd.concat(all_peaks, ignore_index=True)
    else:
        return pd.DataFrame()


def find_appalachian_gages(state_codes=['NC', 'TN', 'VA', 'WV'], bbox=None):
    """Find active stream gages in Appalachian region using waterdata"""
    sites = []

    for state in state_codes:
        print(f"Finding gages in {state}...")
        try:
            # Get site information for discharge stations with peak flow data
            site_info = waterdata.get_info(
                stateCd=state,
                parameterCd='00060',  # Discharge
                siteStatus='active',
                hasDataTypeCd='pk'  # Peak flow data
            )

            if not site_info.empty:
                sites.append(site_info)
        except Exception as e:
            print(f"Error finding gages in {state}: {e}")

    if sites:
        all_sites = pd.concat(sites, ignore_index=True)

        # Optional: filter by bounding box for Appalachian region
        if bbox:  # bbox format: [min_lon, min_lat, max_lon, max_lat]
            all_sites = all_sites[
                (all_sites['dec_long_va'] >= bbox[0]) &
                (all_sites['dec_lat_va'] >= bbox[1]) &
                (all_sites['dec_long_va'] <= bbox[2]) &
                (all_sites['dec_lat_va'] <= bbox[3])
                ]

        return all_sites
    else:
        return pd.DataFrame()


def download_daily_discharge(site_numbers, start_date='1950-01-01', end_date=None):
    """Download daily discharge values using waterdata"""
    if end_date is None:
        end_date = pd.Timestamp.now().strftime('%Y-%m-%d')

    all_discharge = []

    for site in site_numbers:
        print(f"Downloading daily discharge for {site}...")
        try:
            df = waterdata.get_dv(
                sites=site,
                parameterCd='00060',  # Discharge
                start=start_date,
                end=end_date
            )

            if not df.empty:
                df['site_no'] = site
                all_discharge.append(df)
        except Exception as e:
            print(f"Error downloading {site}: {e}")

    if all_discharge:
        return pd.concat(all_discharge, ignore_index=True)
    else:
        return pd.DataFrame()


def get_flood_statistics(site_numbers, start_date='1950-01-01'):
    """
    Get flood-related statistics for sites using waterdata
    Includes peak flows and flood stage information
    """
    results = []

    for site in site_numbers:
        print(f"Getting flood statistics for {site}...")
        try:
            # Get peak flow data
            peaks = waterdata.get_pmcodes(
                sites=site,
                parameterCd='00060',
                start=start_date
            )

            if not peaks.empty:
                # Get site information including flood stages
                site_info = waterdata.get_info(sites=site)

                result = {
                    'site_no': site,
                    'peak_count': len(peaks),
                    'max_peak': peaks['peak_va'].max() if 'peak_va' in peaks.columns else None,
                    'mean_peak': peaks['peak_va'].mean() if 'peak_va' in peaks.columns else None,
                    'site_name': site_info['station_nm'].iloc[0] if not site_info.empty else None,
                    'drainage_area': site_info['drain_area_va'].iloc[
                        0] if not site_info.empty and 'drain_area_va' in site_info.columns else None
                }
                results.append(result)
        except Exception as e:
            print(f"Error processing {site}: {e}")

    return pd.DataFrame(results)


def download_instantaneous_data(site_numbers, start_date, end_date=None, period_days=7):
    """
    Download instantaneous (15-min) discharge data for flash flood analysis
    Useful for identifying rapid rises in streamflow
    """
    if end_date is None:
        end_date = pd.Timestamp.now().strftime('%Y-%m-%d')

    all_data = []

    for site in site_numbers:
        print(f"Downloading instantaneous data for {site}...")
        try:
            df = waterdata.get_iv(
                sites=site,
                parameterCd='00060',  # Discharge
                start=start_date,
                end=end_date
            )

            if not df.empty:
                df['site_no'] = site
                all_data.append(df)
        except Exception as e:
            print(f"Error downloading {site}: {e}")

    if all_data:
        return pd.concat(all_data, ignore_index=True)
    else:
        return pd.DataFrame()


# Example usage with Appalachian bounding box
def get_southern_appalachian_data():
    """
    Complete workflow for Southern Appalachian region
    """
    # Bounding box for Southern Appalachia (approximate)
    # [min_lon, min_lat, max_lon, max_lat]
    app_bbox = [-84.5, 33.5, -79.0, 37.5]  # Covers NC, TN, SC, GA mountains

    print("Finding gages in Southern Appalachia...")
    gages = find_appalachian_gages(
        state_codes=['NC', 'TN', 'SC', 'GA', 'VA'],
        bbox=app_bbox
    )

    print(f"Found {len(gages)} active gages")

    # Filter for gages with substantial drainage area (less urban influence)
    if 'drain_area_va' in gages.columns:
        gages = gages[gages['drain_area_va'] > 10]  # > 10 sq mi
        print(f"Filtered to {len(gages)} gages with drainage area > 10 sq mi")

    # Download peak flow data
    site_list = gages['site_no'].tolist()[:50]  # Limit to 50 sites for testing

    print("\nDownloading peak flow data...")
    peaks = download_usgs_peaks(site_list, start_date='1980-01-01')

    print("\nDownloading daily discharge data...")
    daily = download_daily_discharge(site_list, start_date='1980-01-01')

    print("\nCalculating flood statistics...")
    stats = get_flood_statistics(site_list, start_date='1980-01-01')

    return {
        'gages': gages,
        'peaks': peaks,
        'daily': daily,
        'statistics': stats
    }