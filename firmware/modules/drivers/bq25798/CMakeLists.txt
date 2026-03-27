zephyr_library()
zephyr_library_sources(
    src/bq25798.c
    src/bq25798_zephyr.c
    platform/bq25798_platform_zephyr.c
)
zephyr_include_directories(include)