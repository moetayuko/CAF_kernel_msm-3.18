# ST Gas Gauge Linux Drivers
STC311x Fuel Gauge / Gas Gauge / Battery monitoring


This repository contain the Linux drivers for STC3117 battery monitoring IC.


---------------------------------
Linux driver standard behavior:
---------------------------------

- Probe function is called by Android kernel start-up
- Probe function uses the predefined battery parameters declared in platform data file
- All software internal compensation and features are configurable from the platform data structure
- Work function is called automatically by the internal driver scheduler
	¤ The scheduler delay can be updated accordingly to the application needs
	¤ Typical delay value 5 to 30 seconds
- The battery information are reported in the standard power_supply structure

