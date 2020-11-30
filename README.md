# EmonTH - Wireless Temperature and Humidity Monitoring Node

A repo forked from https://github.com/openenergymonitor/emonTH.

Adds real time power data to be transmitted over RF.

This repo is for emonTH V1.x.

Part of the openenergymonitor.org project.

Builds on JeeLabs, Adafruit and Miles Burton

## Libraries Needed
* JeeLib: https://github.com/jcw/jeelib (CURRENT emonTH V1.5)
* RFu_JeeLib: https://github.com/openenergymonitor/RFu_jeelib (OLD emonTH V1.4)
* Temperature control library: http://download.milesburton.com/Arduino/MaximTemperature/ (version 372 works with Arduino 1.0) and OneWire library: http://www.pjrc.com/teensy/td_libs_OneWire.html
* DHT22 Sensor Library  https://github.com/adafruit/DHT-sensor-library - be sure to rename the sketch folder to remove the '-'


## EmonTH Firmware

**emonTH_DHT22_DS18B20_RFM69CW_Pulse**  Current main emonTH temperature and humidity sensing firmware (Nov2015). Searches for either DHT22 or DS18B20 and reads temperature and humidity once per min (by default) and tx's data back to the emonBase via RFM69CW. If both sensors are detected temperature will be sensed from DS18B20 and humidity from DHT22. Supports on-board RF nodeID setting via DIP switch selectors. Now supports optical counting sensor. See Wiki for more details http://wiki.openenergymonitor.org/index.php/EmonTH_V1.5

**Note:**
* Default RFM12B/RFM69CW settings: 433Mhz, network: 210, Node: 23 (node ID can be changed using [on-board DIP switches](https://wiki.openenergymonitor.org/index.php/EmonTH_V1.5#DIP_Switch_node_ID))
* Readings are converted to integer when sent over RF multiple by 0.1 in emoncms to restore reading
* As the JeeLib library sends out packets in individual bytes, 16 bit integers are split into two received values according to Arduino's "little endian" topology

# License
The EmonTH firmware is released under the GNU GPL V3 license.
 
