import socket
import time

ROBOT_IP = "192.168.40.128"
DASHBOARD_SERVER_PORT = 29999

try:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((ROBOT_IP, DASHBOARD_SERVER_PORT))
        response = s.recv(1024).decode('utf-8')
        print(response)


        
        while(True):
            s.sendall("robotmode\n".encode('utf-8'))
            current_status = s.recv(1024).decode('utf-8')
            print("Current status is ", current_status)

            if current_status.find("POWER_OFF") != -1:
                s.sendall("power on\n".encode('utf-8'))
                response = s.recv(1024).decode('utf-8')
                print(response)

                while current_status.find("IDLE") < 0:
                    s.sendall("robotmode\n".encode('utf-8'))
                    current_status = s.recv(1024).decode('utf-8')
            
                print("Robot is powered on; status is ", response)

            else:
                print("Assuming robot is powered on; status is ", current_status)
        
            if current_status.find("RUNNING") == -1:
                s.sendall("brake release\n".encode('utf-8'))
                response = s.recv(1024).decode('utf-8')
                print(response)

                while current_status.find("RUNNING") < 0:
                    s.sendall("robotmode\n".encode('utf-8'))
                    current_status = s.recv(1024).decode('utf-8')
        
                print("Brakes disengaged; status is ", response)
            else:
                print("Assuming brakes are already disengaged; status is ", current_status)
            
            time.sleep(5)

            print("powering off and repeating this cycle")
            s.sendall("power off\n".encode('utf-8'))

            while current_status.find("POWER_OFF") == -1:
                s.sendall("robotmode\n".encode('utf-8'))
                current_status = s.recv(1024).decode('utf-8')            

except Exception as e:
    print(f"Error: {e}")