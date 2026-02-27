# Firmware

## Initialization

- ```cd example-application```
- ```west init -l```
- ```west update```


## Initialization (new)

- ```west init```
- ```west update```
- ```west zephyr-export```
- ```west packages pip --install```
- edit ```.west/config``` path to be ```applications```

make sure to install dfu-utils and fix udev rules



Copyright Madison Gleydura 2025.

This source contains firmware distributed under the [MIT](LICENSE) License.
