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
            s.sendall("power on\n".encode('utf-8'))
            response = s.recv(1024).decode('utf-8')
            print(response)

            while response.find("IDLE") == -1:
                s.sendall("robotmode\n".encode('utf-8'))
                response = s.recv(1024).decode('utf-8')
            
            print("Robot is powered on; status is ", response)
        
            s.sendall("brake release\n".encode('utf-8'))
            response = s.recv(1024).decode('utf-8')
            print(response)

            while response.find("RUNNING") == -1:
                s.sendall("robotmode\n".encode('utf-8'))
                response = s.recv(1024).decode('utf-8')
        
            print("Brakes disengaged; status is ", response)
            time.sleep(3)

            print("powering off and repeating this cycle")
            s.sendall("power off\n".encode('utf-8'))
            time.sleep(1)

            

except Exception as e:
    print(f"Error: {e}")