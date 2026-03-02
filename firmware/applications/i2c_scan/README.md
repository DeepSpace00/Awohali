# Zephyr I2C Scan

Info from: https://blog.golioth.io/how-to-use-zephyr-shell-for-interactive-prototyping-with-i2c-sensors/

## Activate Shell

```bash
minicom -D /dev/ACM0 --color=on
```

## Show I2C Commands

```bash
i2c -h
```

## I2C Scan

```bash
i2c scan <device>
```