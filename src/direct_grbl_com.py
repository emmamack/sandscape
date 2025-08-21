from main import *
from grbl import *

# --- ARDUINO UNO MOTOR CONTROLLER CONFIGURATION ---
UNO_SERIAL_PORT_NAME = 'COM4'
UNO_BAUD_RATE = 115200

program_start_time = time.time()

print("Establishing serial connection to Arduino Uno...")
uno_serial_port = serial.Serial(UNO_SERIAL_PORT_NAME, UNO_BAUD_RATE, timeout=1)
time.sleep(2) # Wait for connection to establish!

data_queue = queue.Queue()
stop_event = threading.Event()
reader_thread = threading.Thread(target=read_from_port, args=(uno_serial_port, stop_event, data_queue, False))
reader_thread.daemon = True
reader_thread.start()
time.sleep(1)

# Do pre-programmed actions here

# for key, value in grbl_settings.items():
#     x = bytes(f"${key}={value}\n",  'utf-8')
#     print(f"Sending: {x}")
#     uno_serial_port.write(x)
#     time.sleep(0.01)

# Allow user to interact with GRBL
grbl_commands_list = [member.name for member in GrblCmd]
print(f"Send a move in the form 'G1 X__ Z__ F__' or a command from the list below.")
print(f"Available GRBL commands: {grbl_commands_list}")
while True:
    user_input = input(">>")
    if user_input in grbl_commands_list:
        x = getattr(GrblCmd, user_input).value
        print(f"Sending: {x}")
        uno_serial_port.write(x)
    else:
        x = bytes(f"{user_input}\n",  'utf-8')
        print(f"Sending: {x}")
        uno_serial_port.write(x)
    time.sleep(0.1)


