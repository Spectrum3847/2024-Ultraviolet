import paramiko

# pip install paramiko scp

# Get the target IP address from user input
target_ip = input("Enter the target IP address: ")

# SSH username and password (you can also use key-based authentication)
ssh_username = "admin"
ssh_password = ""

try:
    # Create an SSH client
    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    # Connect to the remote machine
    ssh_client.connect(target_ip, username=ssh_username, password=ssh_password)
    
    # Reverse Task 1: Rename the file "/usr/local/natinst/etc/init.d/systemWebServer.bak" back to "/usr/local/natinst/etc/init.d/systemWebServer"
    ssh_client.exec_command("mv /usr/local/natinst/etc/init.d/systemWebServer.bak /usr/local/natinst/etc/init.d/systemWebServer")
    
    # Reverse Task 2: Rename the file "/usr/local/natinst/labview/ni-lvrt" back to "/usr/local/natinst/labview/lvrt"
    ssh_client.exec_command("mv /usr/local/natinst/labview/ni-lvrt /usr/local/natinst/labview/lvrt")
    
    print(f"FreeRIOMemory Changes reversed on {target_ip}")
    
    # Close the SSH connection
    ssh_client.close()
except Exception as e:
    print(f"Failed to reverse changes on {target_ip}: {str(e)}")