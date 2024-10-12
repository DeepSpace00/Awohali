.. _firmware_setup:

Firmware Setup

- Initialize venv: python3 -m venv .venv. then source .venv/bin/activate. then pip install -e scripts/.

- Open the venv

- cd firmware 

- west build -b arduino_portenta_h7/stm32h747xx/m7 zephyr/samples/hello_world/

- west flash