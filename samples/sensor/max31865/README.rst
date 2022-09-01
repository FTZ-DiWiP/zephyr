.. _max31865:

MAX31865 Temperature Sensor
###########################################

Overview
********

This is a sample application to read an external MAX31865
RTD-to-Digital Converter.

Requirements
************

- MAX31865 wired to your board SPI bus
- PT100 connected to MAX31865

References
**********

 - MAX31865: https://datasheets.maximintegrated.com/en/ds/MAX31865.pdf

Building and Running
********************

This sample can be built with any board that supports SPI. A sample overlay is
provided for the Particle Xenon board.

Build the MAX31865 sample application like this:

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/max31865
   :board: particle_xenon
   :goals: build
   :compact:

Build the MAX31865 sample application which uses Segger RTT:

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/max31865
   :board: particle_xenon
   :gen-args: -DOVERLAY_CONFIG=overlay-rtt.conf
   :goals: build
   :compact:

Sample Output
=============

The application will read and print sensor temperature every second.

.. code-block:: console

   TEMP: 25.269148
   TEMP: 25.269148
   TEMP: 25.269148

   <repeats endlessly every second>
