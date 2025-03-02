import serial
import time
import ur_realtime_two_way as urrt 


def mapFloat(value, from_min, from_max, to_min, to_max):
    return (value - from_min) / (from_max - from_min) * (to_max - to_min) + to_min

def connect_to_serial(port, baud_rate):
    try:
        return serial.Serial(port, baud_rate, timeout=1)
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return None

def send_data(ser, data):
    try:
        ser.write((str(data) + '\n').encode('utf-8'))
    except Exception as e:
        print(f"Error sending data: {e}")

def read_data_line(ser):
    try:
        return ser.readline().decode('utf-8').strip()
    except Exception as e:
        print(f"Error reading data: {e}")
        return None

def map_output_to_force(output):
    return mapFloat(output, -1000, 1000, -2, 2)

def main():
    serial_port = 'COM7'
    baud_rate = 115200
    send_delay_ms = 50
    ser = connect_to_serial(serial_port, baud_rate)
    hubbie = urrt.URRealTimeSimplified()

    if ser:
        print(f"Connected to {serial_port} at {baud_rate} baud.")
        try:
            while True:
                currentForce = 0
                line = read_data_line(ser)
                if line is not None and line.startswith(">"):
                    print("Received:", line)
                    # Remove the leading '>' and any trailing commas or whitespace
                    cleaned_line = line.lstrip('>').rstrip(',')
                    pid_values = []
                    # Split by comma to separate each "name:pid" pair
                    for token in cleaned_line.split(','):
                        token = token.strip()
                        if token:
                            parts = token.split(':')
                            if len(parts) == 2:
                                try:
                                    pid = float(parts[1])
                                    pid_values.append(pid)
                                except ValueError:
                                    # If conversion fails, skip the token
                                    continue
                    if pid_values:
                        average_pid = sum(pid_values) / len(pid_values)
                        currentForce = map_output_to_force(average_pid)
                        print("Average PID:", average_pid)
                    else:
                        print("No valid PID values found.")

                # Read the current width from the hubbie device
                currentWidth = hubbie.read_2FG_width()
                send_data(ser, currentWidth)
                hubbie.set_2FG_gripperwidth(currentWidth + currentForce)

                print(f"Force: {currentForce} - Sent: {currentWidth}")
                time.sleep(send_delay_ms / 1000.0)
        except KeyboardInterrupt:
            print("Exiting program.")
        finally:
            ser.close()

if __name__ == "__main__":
    main()
