import paramiko
from scp import SCPClient
import time

# pip install paramiko scp

# Define the IP addresses of the remote machines
remote_ips = ["10.38.47.2", "10.85.15.2"]

# Define the directory to copy from the remote machine
remote_directory = "/u"

# Define the local directory where the files will be copied
local_directory = "./u"

# SSH username and password (you can also use key-based authentication)
ssh_username = "admin"
ssh_password = ""

retry_delay = 1  # 1 seconds

while True:
    any_connection_successful = False  # Flag to track if any connection was successful
    
    for ip in remote_ips:
        try:
            print(f"Connecting to {ip}...")
            # Create an SSH client
            ssh_client = paramiko.SSHClient()
            ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            
            # Connect to the remote machine
            ssh_client.connect(ip, username=ssh_username, password=ssh_password)
            
            # Create an SCP client
            with SCPClient(ssh_client.get_transport()) as scp:
                # Copy the remote directory to the local machine
                scp.get(remote_directory, local_directory, recursive=True)
            
            print(f"Successfully copied directory from {ip} to {local_directory}")
            
            # Close the SSH connection
            ssh_client.close()
            
            any_connection_successful = True  # At least one connection succeeded
            break  # No need to continue trying other IPs
        except Exception as e:
            print(f"Failed to copy directory from {ip}: {str(e)}")
    
    if any_connection_successful:
        break  # At least one successful connection, exit the loop
    
    print(f"Retrying in {retry_delay} seconds...")
    time.sleep(retry_delay)