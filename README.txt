Kernel driver sfxx
===================

Supported chips:
  * Sensirion SF04
    Prefix: 'sfxx'
    Addresses scanned: none
    Datasheet: http://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/DiffPressure/Sensirion_Differential_Pressure_SDP6x0series_Datasheet_V.1.9.pdf

Author:
  David Frey <david.frey@sensirion.com>

Description
-----------

This driver implements support for the Sensirion flow and differential-
pressure sensors that use the SF04 chip

The device communicates with the I2C protocol. Sensors can have any I2C adress,
depending on their configuration. See
Documentation/i2c/instantiating-devices for methods to instantiate the device.

There are no further configuration options.

sysfs-Interface
---------------

measured_value - value, either flow or differential pressure
scale_factor   - the scale factor used by the sensor
offset         - the offset used by the sensor
