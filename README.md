
# Efergy2MQTT
When we purchased our new home (in Australia), I discovered a curious white device (photo below) in the meter box.  After doing a google image search, I discovered it was an Efergy energy meter, which sounds pretty handy even though the wireless display was nowhere to be found.
![Efergy E2](https://imgur.com/a/HVmc2C8)

Given this was 433Mhz device, I replaced the batteries and started exploring the 433Mhz band with my RTL-SDR dongle, surprisingly I found it's signal, as well as other useful devices owned by my neighbours :)

Thanks to [Nathaniel Elijah's work](https://rtlsdr-dongle.blogspot.com/2013/11/finally-complete-working-prototype-of.html) to decode the data sent from an Efergy E2 using the RTL-SDRe, I created this simple (well very rough) python wrapper that creates a json payload from the output of the EfergyRPI_log binary and produces a mqtt message for consumption by HomeAssitant, NodeRed etc...

I have included Nathaniel's source code and a pre-compiled ELF 64binary as a convenience, as the link to the source on his blog is broken. Compilation instructions and the code can be found on Gough Lui's [Techzone blog](https://goughlui.com/2013/11/14/efergy-energy-monitor-decoding-with-rtl-sdr/)

## Parts list

* RTL-SDR Dongle - I got my RTL2832U from https://www.rtl-sdr.com/buy-rtl-sdr-dvb-t-dongles/
* RTL-SDR software - https://github.com/steve-m/librtlsdr
* A linux PC, RPi etc..
* An Efergy E2 - Not sure if you can still buy these, or where they are sold.
* Some patience - depending on the location of your meter, and the RTL-SDR antenna, you may need to play with the rtl_fm values a bit to tweak the signal attenuation and reduce noise.
* MQTT server - I am using Mosquitto
* Python3, pipenv and the Paho.mqtt library, if you don't want to use pipenv, just pip install paho.mqtt 

Optional
* HomeAssistant - For home automation I am using [HomeAssistant](https://www.home-assistant.io/getting-started/) (aka hassio) on a RaspberryPi 3+ using [hassos](https://www.home-assistant.io/installation/raspberrypi)
* Grafana
* InfluxDB

### Running
```
git clone https://github.com/3tc/efergy2mqtt.git
cd efergy2mqtt

# Install pipenv
python3 -m pip --user pipenv

# Setup the virtualenv and install dependencies
pipenv install

# Run using pipenv run
./run.sh
```

### Grafana
![Grafana dashboard](https://imgur.com/a/Jt8xlyt)
I already had HomeAssistant pushing temperature readings from the home into InfluxDB to chart temperature readings in Grafana.  Adding the energy consumption data to influx was a simple config change in HomeAsssitant, and once the data was being populated in Infux, setting up a dashboard was trivial.

