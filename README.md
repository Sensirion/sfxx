Kernel driver sfxx
===================

Supported chips:
  * Sensirion SF04
    Prefix: 'sfxx'
    Addresses scanned: none
    Datasheet: http://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/DiffPressure/Sensirion_Differential_Pressure_SDP6x0series_Datasheet_V.1.9.pdf
  * Sensirion SF05
    Prefix: 'sfxx'
    Addresses scanned: none
    Datasheet: http://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/GasFlow/Sensirion_Gas_Flow_SFM3000_Datasheet_V2.pdf

Author:
  David Frey <david.frey@sensirion.com>

Description
-----------

This driver implements support for the Sensirion flow and differential-
pressure sensors that use the SF04 and SF05 chips.

The device communicates with the I2C protocol. Sensors can have any I2C adress,
depending on their configuration. See
Documentation/i2c/instantiating-devices for methods to instantiate the device.

The following device names are supported by the probe function:
sfxx:
      The driver will try to detect the sensor chip and will self configure
      according to the settings of the sensor.
sf04, sdp6xx:
      The driver will expect the SF04 sensor chip and will self configure
      according to the settings of the sensor.
sf05, sfm3500:
      The driver will expect the SF05 sensor chip and will self configure
      according to the settings of the sensor.

sysfs-Interface
---------------

measured_value - value, either flow or differential pressure
scale_factor   - the scale factor used by the sensor
offset         - the offset used by the sensor
unit           - the flow or pressure unit of the measured value