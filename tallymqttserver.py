import sys
import os
import os.path
import time
from rpi_ws281x import *
import argparse
# LED strip configuration:
LED_COUNT = 4      # Number of LED pixels.
LED_PIN = 18      # GPIO pin connected to the pixels (18 uses PWM!).
LED_TALLY = 24
# LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA = 10      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 250    # Set to 0 for darkest and 255 for brightest
# True to invert the signal (when using NPN transistor level shift)
LED_INVERT = False
LED_CHANNEL = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53
import RPi.GPIO as GPIO

# Initialize Raspberry PI GPIO

GPIO.setmode(GPIO.BCM)

GPIO.setup(LED_TALLY, GPIO.OUT)
strip = Adafruit_NeoPixel(
        LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
    # Intialize the library (must be called once before other functions).
strip.begin()


try:
    import paho.mqtt.client as mqtt
except ImportError:
    # This part is only required to run the example from within the examples
    # directory when the module itself is not installed.
    #
    # If you have the module installed, just use "import paho.mqtt.client"
    import os
    import inspect
    cmd_subfolder = os.path.realpath(os.path.abspath(os.path.join(os.path.split(inspect.getfile( inspect.currentframe() ))[0],"../src")))
    if cmd_subfolder not in sys.path:
        sys.path.insert(0, cmd_subfolder)
    import paho.mqtt.client as mqtt

    strip.begin()

def on_connect(mqttc, obj, flags, rc):
    print("rc: "+str(rc))

def on_message(mqttc, obj, msg):

    strip.begin()

    print(msg.topic+" "+str(msg.qos)+" "+str(msg.payload))
    if msg.payload == 'OFF':
            GPIO.output(LED_TALLY, False)
            colorWipe(strip, Color(0, 0, 0))  # OFF wipe
    if msg.payload == 'ON':
            GPIO.output(LED_TALLY, True)
            colorWipe(strip, Color(255, 0, 0))  # RED wipe
    if msg.payload == 'ONG':
            GPIO.output(LED_TALLY, False)
            colorWipe(strip, Color(0, 255, 0))  # Green wipe
    if msg.payload == 'Reboot':
            GPIO.output(LED_TALLY, False)
            colorWipe(strip, Color(0, 0, 0))  # OFF
            os.system("sudo reboot")

# Define functions which animate LEDs in various ways.
def colorWipe(strip, color, wait_ms=50):
    """Wipe color across display a pixel at a time."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
        strip.show()
        time.sleep(wait_ms/1000.0)



def on_publish(mqttc, obj, mid):
    print("mid: "+str(mid))

def on_subscribe(mqttc, obj, mid, granted_qos):
    print("Subscribed: "+str(mid)+" "+str(granted_qos))

def on_log(mqttc, obj, level, string):
    print(string)

# If you want to use a specific client id, use
# mqttc = mqtt.Client("client-id")
# but note that the client id must be unique on the broker. Leaving the client
# id parameter empty will generate a random id for you.
mqttc = mqtt.Client('Cam1')
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe
# Uncomment to enable debug messages
#mqttc.on_log = on_log
mqttc.connect("192.168.155.10", 1883, 60)
mqttc.subscribe("cam1/tally", 0)


mqttc.loop_forever()
