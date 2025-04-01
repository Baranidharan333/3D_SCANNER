import machine
import time

# Define LED pin (GPIO 2 is the built-in LED on many ESP32 boards)
led = machine.Pin(2, machine.Pin.OUT)

while True:
    led.value(1)  # Turn LED ON
    print("LED ON")
    time.sleep(10)  # Wait for 10 seconds
    led.value(0)  # Turn LED OFF
    print("LED OFF")
    time.sleep(10)  # Wait for another 10 seconds
