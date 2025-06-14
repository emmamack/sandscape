import serial
import threading
import time

def read_from_port(ser, stop_event):
    """
    Reads data from the serial port line by line in a dedicated thread.

    Args:
        ser (serial.Serial): The initialized serial port object.
        stop_event (threading.Event): The event to signal the thread to stop.
    """
    print("Reader thread started.")
    while not stop_event.is_set():
        try:
            # The readline() function will block until a newline character
            # is received, or until the timeout (set during port initialization)
            # is reached.
            line = ser.readline()

            # If a line was actually read (i.e., not a timeout)
            if line:
                # Decode the bytes into a string, using UTF-8 encoding.
                # 'errors='ignore'' will prevent crashes on decoding errors.
                # .strip() removes leading/trailing whitespace, including the newline.
                decoded_line = line.decode('utf-8', errors='ignore').strip()
                print(f"Received: {decoded_line}")

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

def main():
    """
    Main function to set up the serial port and manage the reader thread.
    """
    # --- IMPORTANT: CONFIGURE YOUR SERIAL PORT HERE ---
    # Find the correct port name for your device.
    # Examples:
    # - Windows: "COM3", "COM4", etc.
    # - Linux:   "/dev/ttyUSB0", "/dev/ttyACM0", etc.
    # - macOS:   "/dev/cu.usbmodem14201", "/dev/cu.wchusbserial1420", etc.
    # You can find the port name in your system's device manager or by running
    # `python -m serial.tools.list_ports` in your terminal.
    SERIAL_PORT = "/dev/ttyUSB0"  # <--- CHANGE THIS
    BAUD_RATE = 9600

    ser = None
    reader_thread = None
    stop_event = threading.Event()

    try:
        # Initialize the serial port.
        # The 'timeout' parameter is crucial. readline() will wait for this
        # many seconds for a newline character before returning. If it's
        # None, it will wait forever. A value of 1 second is a good starting point.
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Successfully opened and listening on {SERIAL_PORT} at {BAUD_RATE} baud.")
        
        # Create and start the reader thread.
        # We make it a daemon thread so it automatically exits when the main
        # program finishes, just in case shutdown is not clean.
        reader_thread = threading.Thread(target=read_from_port, args=(ser, stop_event))
        reader_thread.daemon = True
        reader_thread.start()

        # Keep the main thread alive to wait for user interruption
        print("Press Ctrl+C to stop the program.")
        while True:
            time.sleep(1)

    except serial.SerialException as e:
        print(f"Error: Could not open serial port '{SERIAL_PORT}'. {e}")
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Shutting down.")
    finally:
        if reader_thread and reader_thread.is_alive():
            print("Stopping reader thread...")
            stop_event.set()
            # Wait for the thread to finish its current operation
            reader_thread.join(timeout=2)

        if ser and ser.is_open:
            print("Closing serial port.")
            ser.close()
        
        print("Program terminated.")

if __name__ == "__main__":
    main()
