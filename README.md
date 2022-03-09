# KellerModbus
A library to use an Arduino as a master to control and communicate via modbus with the pressure and water level sensors produced by [In-stru](https://in-situ.com/us/).

This library depends on the [EnviroDIY/SensorModbusMaster](https://github.com/EnviroDIY/SensorModbusMaster) library or the forker version https://github.com/neilh10/InsituModbus
The In-situ [EnviroDIY/ModularSensor](https://github.com/EnviroDIY/ModularSensors) library functions for depend on this library.


## <a name="functions"></a>Functions

This library implements a small subset of functions from those described in the [In-Situ Modbus
Communication Protocol
Version 5.10l](https://in-situ.com/pub/media/support/documents/Modbus_Manual.pdf). These are:
* `getSerialNumber`
* `getValues`, for presssure and temperature

The In-Situ instrument must be changed to 9600 baud from the default 19200 
The function code 
0x03 Read Holding Registers 2.3.1.4  all registers are Holding Registers
0x11 (17) Report Device(Slave) Id   2.3.1.8  
The device register maps are in sections 
Exception Codes - Appendix A
Device Ids - Appendix B ~ LT500=1
Sensor Ids - Appendix C ~ 
    1 "Temperature"
    2 "5 PSI"

Parameter Ids  - Appendix D

Unit Ids - Appendix E
0  None
1-3 Temperature
1 Celsious


33-38 Length
35 - m
37 - inches
38 - feet

Holding Registers (1 based, in packets 0-37, 37-44,45-52,53-61)
 1-38 Sensor Common Registers
38-45 Parameter 1 Registers
46-53 Parameter 2 Registers
54-62 Parameter 3 Registers
See Sect 6 Probe Common Registers (9000) Std across In-Situ

This library also calculates water depth via the following function:
* `calcWaterDepthM`, based on water pressure and temperature-dependency on water density, following equation 6 from [JonesHarris1992-NIST-DensityWater.pdf](https://github.com/EnviroDIY/KellerModbus/blob/master/doc/JonesHarris1992-NIST-DensityWater.pdf).

## <a name="license"></a>License
Software sketches and code are released under the BSD 3-Clause License -- See [LICENSE.md](https://github.com/EnviroDIY/ModularSensors/blob/master/LICENSE.md) file for details.

Documentation is licensed as [Creative Commons Attribution-ShareAlike 4.0](https://creativecommons.org/licenses/by-sa/4.0/) (CC-BY-SA) copyright.

## <a name="acknowledgments"></a>Acknowledgments
[EnviroDIY](http://envirodiy.org/)™ is presented by the Stroud Water Research Center, with contributions from a community of enthusiasts sharing do-it-yourself ideas for environmental science and monitoring.

[Neil Hancock](https://github.com/neiilh10) is the primary developer of this library.   
This library is based on 
[Anthony Aufdenakmpe's KellerModbus](https://github.com/EnviroDIY/KellerModbus)


T