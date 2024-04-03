import time
import pygame
import os
import sys
from networktables import NetworkTables

#Need to 
# pip install pygame 
# pip install pynetworktables
# Use https://elevenlabs.io/speech-synthesis to generate audio files

# Define a list of IP addresses to check
ip_addresses = ["10.38.47.2", "10.85.15.2", "127.0.0.1", "172.22.11.2"]  # Replace with your IP addresses

# Initialize pygame and networkTables instance
pygame.mixer.init()

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

#Allows it to work with pyinstaller or auto_py_to_exe
def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.abspath(".")

    return os.path.join(base_path, relative_path)

# Define the path to the audio files
audio_folder = "british/"
audio_files = {
    60: "60.mp3",
    30: "30.mp3",
    15: "15.mp3",
    14: "14.mp3",
    13: "13.mp3",
    12: "12.mp3",
    11: "11.mp3",
    10: "10.mp3",
    9: "9.mp3",
    8: "8.mp3",
    7: "7.mp3",
    6: "6.mp3",
    5: "5.mp3",
    4: "4.mp3",
    3: "3.mp3",
    2: "2.mp3",
    1: "1.mp3"
}

# Function to play the countdown audio for a given number of seconds
def play_countdown_audio(seconds):
    audio_file = audio_files.get(seconds)
    if audio_file:
        audio_file = audio_folder + audio_file
        pygame.mixer.music.load(resource_path(audio_file))
        pygame.mixer.music.set_volume(1.0)
        pygame.mixer.music.play()

        # Wait for the audio to finish
        time.sleep(0.99)
        pygame.mixer.music.stop()
        
def aunnounce_audio(seconds):
    audio_file = audio_files.get(seconds)
    if audio_file:
        audio_file = audio_folder + audio_file
        pygame.mixer.music.load(resource_path(audio_file))
        pygame.mixer.music.set_volume(1.0)
        pygame.mixer.music.play()

        # Wait for the audio to finish
        time.sleep(3)
        pygame.mixer.music.stop()

if __name__ == "__main__":
    connected = False
    try:
        while True:
            while not connected: #Get network tables connection
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
                    
            print("Starting countdown program")
            table = NetworkTables.getTable("/AdvantageKit/DriverStation/")
            startCount = 15
            count = 0
            
            while connected:  # main loop
                
                # Read the countdown value from the network table
                countdown_seconds = int(table.getNumber("MatchTime", -1))
                
                # If the countdown value is greater than the startCount value, reset the countdown
                # This lets us skip counting in auton
                if count <= 0 and countdown_seconds > startCount:
                    count = startCount
                    print("Resetting countdown...")
                
                # Check if the countdown value is within the range 1-15
                if 1 <= countdown_seconds <= count:
                    print(f"Counting down: {countdown_seconds} seconds...")
                    play_countdown_audio(countdown_seconds)
                    count = countdown_seconds - 1

                # Announce on 60 and 30 seconds remaining
                if countdown_seconds == 60:
                    print("1 minute remaining")
                    aunnounce_audio(60)
                
                if countdown_seconds == 30:
                    print("30 seconds remaining")
                    aunnounce_audio(30)
                
                # Check if the network table connection is still active
                if not NetworkTables.isConnected():
                    # Failed to connect, return False
                    print("NetworkTables disconnected")
                    NetworkTables.shutdown()
                    connected = False
                
                # Sleep for a short duration to avoid continuous polling
                time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        pygame.mixer.quit()
        NetworkTables.shutdown()