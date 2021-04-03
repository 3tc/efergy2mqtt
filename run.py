import subprocess
import re
import json
import paho.mqtt.publish as publish
from os import environ

# rtl_fm -f 433510000 -s 200000 -r 96000 -g 50 | ./EfergyRPI_log

env = dict(environ)
efergy_bin=env["EFERGY_BINARY"]
mqtt_host=env["MQTT_HOST"]
mqtt_user=env["MQTT_USER"]
mqtt_pass=env["MQTT_PASS"]

try:
    pattern = re.compile("^[0-9]+.*")
    rtl_output = subprocess.Popen(["rtl_fm", "-f 433510000", "-s 200000", "-r 96000", "-g 50"], stdout=subprocess.PIPE)
    efergy = subprocess.Popen([efergy_bin], stdin=rtl_output.stdout, stdout=subprocess.PIPE)
    for line in iter(efergy.stdout.readline, ''):
        str_line = line.decode('utf-8')
        if pattern.match(str_line):
            record = str_line.split(",")
            if len(record) == 3:
                try:
                    float_val = round(float(record[2].strip()), 2)
                    if float_val < 10000: # filter out erroneous values
                        json_msg = {
                                "consumption_watts": float_val
                        }
                        print(json.dumps(json_msg))
                        
                        publish.single("house/energy", json.dumps(json_msg), hostname = mqtt_host, auth = {"username": mqtt_user, "password": mqtt_pass})
                except:
                    print('Failed to connect to mqtt server')

finally:
    efergy.kill()
    rtl_output.kill()

