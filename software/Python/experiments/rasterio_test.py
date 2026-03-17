import rasterio

with rasterio.open('/home/deepspace/Downloads/Monongalia_County_2025.sid') as src:
    data = src.read()
    profile = src.profile
    profile.update(driver='GTiff')

    with rasterio.open('/home/deepspace/Downloads/Monongalia_County_2025.tif', 'w', **profile) as dst:
        dst.write(data)