import time
from networktables import NetworkTables
import keyboard
import winsound

# Define a list of IP addresses to check
ip_addresses = ["10.38.47.2", "10.85.15.2", "127.0.0.1", "172.22.11.2"]  # Replace with your IP addresses

keys_to_monitor = {
    "RecordMatch": "F9",
    "StopRecording": "F10"
}
sound_to_play = "Beep"

# Function to connect to NetworkTables using an IP address
def connect_to_networktables(ip):
    # Connect to NetworkTables using the specified IP address
    NetworkTables.initialize(server=ip)
    time.sleep(2) # Wait for NetworkTables to connect
    
    if not NetworkTables.isConnected():
        # Failed to connect, return False
        return False

    # Successful connection, return True
    return True

#main program
if __name__ == "__main__":
    connected = False
    try:
        while True:
            while not connected:
                for ip in ip_addresses:
                    connected = connect_to_networktables(ip)
                    if connected:
                        print(f"Connected to NetworkTables at IP address: {ip}")
                        break
                    else:
                        print(f"Connection failed for IP address: {ip}")
                
                if not connected:
                    print("No connection established. Retrying in 5 seconds...")
                    time.sleep(5)
                    
            print("Starting NT Monitoring program")
            nt = NetworkTables.getTable("SmartDashboard")
            # Create a dictionary to store the current values of the networktables keys
            current_values = {}
            for key, value in keys_to_monitor.items():
                current_values[key] = nt.getBoolean(key, False)
            
            while connected:
                # Create a dictionary to store the updated values of the networktables keys
                new_values = {}
                for key, value in keys_to_monitor.items():
                    new_values[key] = nt.getBoolean(key, False)
                # Check each key to see if its value has changed from false to true
                for key, value in keys_to_monitor.items():
                    # If the value has changed from false to true, send a keyboard press
                    if current_values[key] != new_values[key]:
                        current_values[key] = new_values[key]
                        print(key + " changed to: " + str(new_values[key]))
                        if new_values[key]:
                            keyboard.press(value)
                            print("Pressed: " + value)
                            time.sleep(0.1)
                            keyboard.release(value)
                            winsound.PlaySound(sound_to_play, winsound.SND_ALIAS)

                # Check if the network table connection is still active
                if not NetworkTables.isConnected():
                    # Failed to connect, return False
                    print("NetworkTables disconnected")
                    NetworkTables.shutdown()
                    connected = False
                    
                # Wait for a short time before checking the values again
                time.sleep(0.1)
                
    except KeyboardInterrupt:
        pass
    finally:
        NetworkTables.shutdown()