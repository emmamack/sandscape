import serial
import time
import queue
import threading

NANO_SERIAL_PORT_NAME = "COM4"
NANO_BAUD_RATE = 9600
SENSOR_THETA_MASK = [i*22.5 for i in range(16)]

def read_from_port(serial_port, stop_event, data_queue):
    """
    Reads data from the serial port line by line in a dedicated thread.

    Args:
        ser (serial.Serial): The initialized serial port object.
        stop_event (threading.Event): The event to signal the thread to stop.
    """
    last_msg_timestamp = time.time()
    time_since_last_msg = time.time() - last_msg_timestamp
    last_msg_timestamp = time.time()
    print("Reader thread started.")
    while not stop_event.is_set():
        try:
            # The readline() function will block until a newline character
            # is received, or until the timeout (set during port initialization)
            # is reached.
            line = serial_port.readline()

            # If a line was actually read (i.e., not a timeout)
            if line:
                # Decode the bytes into a string, using UTF-8 encoding.
                # 'errors='ignore'' will prevent crashes on decoding errors.
                # .strip() removes leading/trailing whitespace, including the newline.
                time_since_last_msg = time.time() - last_msg_timestamp
                last_msg_timestamp = time.time()
                decoded_line = line.decode('utf-8', errors='ignore').strip()
                # print(f"{time.time():.5f} | {time_since_last_msg:.5f}s | Received: {decoded_line}")
                if len(decoded_line) > 0:
                    data_queue.put(decoded_line)
    
        except serial.SerialException as e:
            # Handle cases where the serial port is disconnected or an error occurs
            print(f"Serial port error: {e}. Stopping thread.")
            break
        except Exception as e:
            # Handle other potential exceptions
            print(f"An unexpected error occurred: {e}")
            if not stop_event.is_set():
                # Avoid flooding the console with error messages
                time.sleep(1)

    print("Reader thread finished.")


print("Establishing serial connection to Arduino Nano...")
nano_serial_port = serial.Serial(NANO_SERIAL_PORT_NAME, NANO_BAUD_RATE, timeout=1)
time.sleep(2) # Wait for connection to establish!

# Set up sensor monitoring thread
stop_event = threading.Event()
sensor_data_queue = queue.Queue()
sensor_reader_thread = threading.Thread(target=read_from_port, args=(nano_serial_port, stop_event, sensor_data_queue))
sensor_reader_thread.daemon = True
sensor_reader_thread.start()
time.sleep(2)

def parse_sensor_data(raw):
    touch_sensor_status = [int(char) for char in raw[0:16]]
    prox_status = int(raw[16])

    return touch_sensor_status, prox_status

def calculate_next_point(touch_sensor_status):
    selected_thetas = [j for i,j in zip(touch_sensor_status, SENSOR_THETA_MASK) if i==1]
    print(selected_thetas)
    avg_theta = sum(selected_thetas) / len(selected_thetas)
    selected_point = (280, avg_theta)
    print(selected_point)

    


curr_point = (0, 0)
while True:
    if not sensor_data_queue.empty():
        sensor_data_raw = sensor_data_queue.get()
        if len(sensor_data_raw) == 17:
            touch_sensor_status, prox_status = parse_sensor_data(sensor_data_raw)
            print(touch_sensor_status)

            if touch_sensor_status != [0]*16:
                calculate_next_point(touch_sensor_status)


