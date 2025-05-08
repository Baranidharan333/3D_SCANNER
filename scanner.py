# MicroPython code for ESP32 3D Scanner with OLED - Optimized Version
from machine import Pin, ADC, SPI, I2C
from sdcard import SDCard
from os import mount
from ssd1306 import SSD1306_I2C
import time, math
import network
import os
import socket
import sdcard


# Editable Variables
scan_amount = 10  # Reduced for faster scanning
file_name = "scan_1.txt"

z_axis_height = 5  # in cm
step_delay = 400  # Faster stepping
z_layer_height = 1  # in mm
steps_per_rotation_for_motor = 200
distance_to_center = 9  # in cm

# I/O Pins
button = Pin(5, Pin.IN, Pin.PULL_UP)  # Reset/Homing button
limit_switch = Pin(18, Pin.IN, Pin.PULL_UP)  # Limit switch for homing

dir_z = Pin(13, Pin.OUT)
step_z = Pin(12, Pin.OUT)
enable_z = Pin(14, Pin.OUT)

dir_r = Pin(27, Pin.OUT)
step_r = Pin(26, Pin.OUT)
enable_r = Pin(25, Pin.OUT)


# ADC for Distance Sensor
sensor_pin = 34  # VOUT connected to GPIO34
adc = ADC(Pin(sensor_pin))
adc.atten(ADC.ATTN_11DB)
# Circular buffer for averaging (50 samples)
buffer = []
buffer_size = 50  

# OLED Setup
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
oled = SSD1306_I2C(128, 64, i2c)

# SD Card Setup
spi = SPI(1, baudrate=1000000, polarity=0, phase=0, sck=Pin(16), mosi=Pin(15), miso=Pin(17))
sd = SDCard(spi, Pin(4))
mount(sd, "/sd")
time.sleep_ms(100)

# Variables
scan = 0
angle = 0
x, y, z = 0.0, 0.0, 0.0
steps_z_height = 150
RADIANS = (math.pi / 180.0) * (360 / steps_per_rotation_for_motor)

# Initial Message
oled.fill(0)
oled.text("Press", 10, 20)
oled.text("Home", 10, 30)
oled.text("Button...!", 10, 40)
oled.show()


# Custom map function (like Arduino map)
def map_value(x, in_min, in_max, out_min, out_max):
    """ Maps a value from one range to another """
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Distance mapping function
def get_distance_map(adc_value):
    """ Maps ADC value to distance using ranges and linear interpolation """
    if adc_value >= 2850:  # 3 cm to 5 cm range
        return map_value(adc_value, 2850, 2890, 3, 5)
    elif 2600 <= adc_value < 2850:  # 5 cm to 8 cm range
        return map_value(adc_value, 2600, 2850, 5, 8)
    elif 2300 <= adc_value < 2600:  # 8 cm to 10 cm range
        return map_value(adc_value, 2300, 2600, 8, 10)
    elif 2100 <= adc_value < 2300:  # 10 cm to 12 cm range
        return map_value(adc_value, 2100, 2300, 10, 12)
    elif 1800 <= adc_value < 2100:  # 12 cm to 15 cm range
        return map_value(adc_value, 1800, 2100, 12, 15)
    else:
        return 15  # Default to max range if out of bounds

# Calculate moving average
def calculate_average(data):
    return sum(data) / len(data) if data else 0


# OLED Display Function
def update_oled(status, z_height, distance, angle_step):
    oled.fill(0)
    oled.text("Status: " + status, 0, 0)
    oled.text(f"Z: {z_height:.1f} mm", 0, 10)
    oled.text(f"D: {distance:.2f} cm", 0, 20)
    oled.text(f"Angle: {angle_step:.1f}", 0, 30)
    oled.show()

# Homing Z-axis function
def home_z_axis():
    global scan, z
    oled.fill(0)
    oled.text("Homing Z-axis...", 0, 0)
    oled.show()

    while limit_switch.value():  # Move down until limit switch is pressed
        enable_z.value(0)
        dir_z.value(0)  # Move down
        step_z.value(1)
        time.sleep(0.001)
        step_z.value(0)
        time.sleep(0.001)

    # Homed, disable motors
    enable_z.value(1)
    update_oled("Homed", 0, 0, 0)
    print("Z-axis Homed")

    # Enable scanning after homing
    scan = 1
    z = 0  # Reset Z-axis height after homing
    time.sleep(1)

# Get distance from sensor
# Get distance from sensor and calculate (x, y)
# Mapping function similar to Arduino's map()
def map_value(x, in_min, in_max, out_min, out_max):
    """ Maps a value from one range to another """
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Get distance from sensor and calculate (x, y)
# Get distance from sensor and calculate (x, y)
def get_distance(angle):
    adc_value = adc.read()

    # Get distance from mapped ADC value
    distance = get_distance_map(adc_value)

    # Add distance to buffer
    if len(buffer) >= buffer_size:
        buffer.pop(0)  # Remove oldest value if buffer is full
    buffer.append(distance)

    # Calculate average distance
    average_distance = calculate_average(buffer)
    
    # Ensure distance is positive and relative to the center
    distance = abs(distance_to_center - average_distance)  # Make sure distance is non-negative

    # Compute X, Y coordinates
    x = math.sin(angle) * distance
    y = math.cos(angle) * distance

    return x, y, distance  # Return values instead of just distance


# Save data to SD card
def write_to_SD(SDx, SDy, SDz):
    with open("/sd/" + file_name, "a") as file:
        file.write(f"{SDx:.2f},{SDy:.2f},{SDz:.2f}\n")

# Serve files over HTTP (with chunked file sending)
def serve_files():
    addr = socket.getaddrinfo("0.0.0.0", 80)[0][-1]
    s = socket.socket()
    s.bind(addr)
    s.listen(5)

    print("Web server running. Access via browser.")

    while True:
        conn, addr = s.accept()
        print("Client connected from:", addr)
        request = conn.recv(1024).decode()

        # Parse GET request to get file name
        request_line = request.split("\n")[0]
        if "GET" in request_line:
            file_name = request_line.split()[1].lstrip("/")  # Remove leading "/"

            if file_name == "":
                # Serve index page listing files
                conn.send(b"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n")
                conn.send(b"<html><body><h1>3D SCANNER3 SD Card File Server</h1>")
                conn.send(b"<p>Available Files:</p><ul>")

                for f in os.listdir("/sd"):
                    conn.send(f'<li><a href="/{f}">{f}</a></li>'.encode())

                conn.send(b"</ul></body></html>")

            else:
                file_path = "/sd/" + file_name

                if file_name in os.listdir("/sd"):  
                    conn.send(b"HTTP/1.1 200 OK\r\n")
                    conn.send(b"Content-Disposition: attachment; filename=" + file_name.encode() + b"\r\n")
                    conn.send(b"Content-Type: application/octet-stream\r\n\r\n")

                    # Stream file in 512-byte chunks (prevents MemoryError)
                    try:
                        with open(file_path, "rb") as f:
                            while True:
                                chunk = f.read(512)  # Read 512 bytes at a time
                                if not chunk:
                                    break
                                conn.send(chunk)
                    except Exception as e:
                        print("File Read Error:", e)
                        conn.send(b"HTTP/1.1 500 Internal Server Error\r\n\r\nError reading file")

                else:
                    conn.send(b"HTTP/1.1 404 Not Found\r\n\r\nFile Not Found")

        conn.close()


# Main Loop
while True:
    # Check button press to home Z-axis
    if not button.value():  # Button pressed
        print("Button Pressed: Homing Z-axis...")
        home_z_axis()

    if scan == 1:
        if z < z_axis_height:
            for loop_cont in range(steps_per_rotation_for_motor):
                x, y, distance = get_distance(angle)  # Capture updated values
                
                # Rotate motor
                enable_r.value(0)
                dir_r.value(0)
                step_r.value(1)
                time.sleep_us(step_delay)
                step_r.value(0)
                time.sleep_us(step_delay)
                
                angle += RADIANS  # Update angle step
                
                # Save updated values to SD card
                write_to_SD(x, y, z)
                update_oled("Scanning", z, distance, angle * 180 / math.pi)
            
            angle = 0  # Reset angle for next layer
            
            # Move Z-axis up for next layer
            for _ in range(steps_z_height):
                enable_z.value(0)
                dir_z.value(1)  # Move Z up
                step_z.value(1)
                time.sleep(0.01)
                step_z.value(0)
                time.sleep(0.01)  # Reduced Z-axis delay
            z += z_layer_height
        else:
            enable_z.value(1)
            enable_r.value(1)
            update_oled("Scan Complete", z, 0, 0)
            scan = 0  # Stop scanning after completion
            break
serve_files()





