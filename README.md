# BatteryCharger
# Muntiple chemistry Battery Charger
- Ardenio mega based
# Features
- Multiple chemistry support
- Battery temperature monitoring
- Battery voltage monitoring
- Battery current monitoring
- Battery capacity monitoring
- Battery state of charge monitoring
- Battery health monitoring
- LCD display for real-time monitoring
- USB interface for data logging
- LED indicators for charging status
- Configurable charging profiles for different battery types
- Overcharge protection
- Overcurrent protection
- Short circuit protection
- Reverse polarity protection dependant on hardware design
- Configurable charging current and voltage
- Configurable charging time
- Configurable charging temperature limits
- Configurable charging voltage limits
- Configurable charging current limits
- Configurable charging capacity limits
- Configurable charging state of charge limits
- Configurable charging health limits
- Configurable charging profiles for different battery types
- Configurable charging parameters for different battery types
- Configurable charging parameters for different battery chemistries
- Arduino Mega based but requires additional hardware to switch power to control voltage and current
- Hardware design is not included in this repository, only the software at the moment
- The software is designed to work with a specific hardware design, so it may not work with other hardware designs without modification
- It is intended that the hardware design will be multiple P channel MOSFETs. It is a true high side switch to the battery. A prototype design is available on request



# Usage : Connect the battery to the charger
- Connect the charger to a power source
- Select the battery chemistry from the LCD menu
- Select size of the battery (Amp hours)
- Ensure the battery is connected correctly and turn on the charger
- The charger will start charging the battery when start button is pressed

