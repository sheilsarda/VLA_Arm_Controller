from socket import socket, AF_INET, SOCK_STREAM
from time import sleep
import threading

ROBOT_IP = "192.168.40.128"
DASHBOARD_SERVER_PORT = 29999

class RobotInitializer:
    def __init__(self):
        self.status = {
            'safetystatus': '',
            'robotmode': '',
            'running': ''
        }

        self.create_socket_and_initialize_robot()
        self.monitoring_thread = threading.Thread(target=self._monitoring_thread)
        self.monitoring_thread.start()

    def _monitoring_thread(self) -> None:
        """
        Continuously monitor robot so controller knows if anything goes wrong
        """
        try:
            with socket(AF_INET, SOCK_STREAM) as s:
                s.connect((ROBOT_IP, DASHBOARD_SERVER_PORT))
                response = s.recv(1024).decode('utf-8')
                print(response)

                while True:
                    s.sendall("robotmode\n".encode('utf-8'))
                    self.status['robotmode'] = s.recv(1024).decode('utf-8')
                    s.sendall("safetystatus\n".encode('utf-8'))
                    self.status['safetystatus'] = s.recv(1024).decode('utf-8')
                    s.sendall("running\n".encode('utf-8'))
                    self.status['running'] = s.recv(1024).decode('utf-8')
            
        except ConnectionRefusedError as e:
            print(f"Error: {e}")


    def power_on_and_initialize_robot_demo_loop(self) -> None:
        """
        Powers on the robot listening at ROBOT_IP / DASHBOARD_SERVER_PORT, 
        - waits for it to be powered on
        - disengages the brakes
        - powers off the robot
        - repeats this cycle every 5 seconds.

        Is able to handle robot starting from any state, including powered on, brake disengaged, etc.
        """
        try:
            with socket(AF_INET, SOCK_STREAM) as s:
                s.connect((ROBOT_IP, DASHBOARD_SERVER_PORT))
                response = s.recv(1024).decode('utf-8')
                print(response)

                while True:
                    self.power_on_and_initialize_robot(s)
                    
                    time.sleep(5)

                    print("powering off and repeating this cycle")
                    s.sendall("power off\n".encode('utf-8'))

                    s.sendall("robotmode\n".encode('utf-8'))
                    current_status = s.recv(1024).decode('utf-8')     

                    while current_status.find("POWER_OFF") == -1:
                        s.sendall("robotmode\n".encode('utf-8'))
                        current_status = s.recv(1024).decode('utf-8')            

        except ConnectionRefusedError as e:
            print(f"Error: {e}")

    def create_socket_and_initialize_robot(self) -> None:
        """
        Creates a socket and initializes the robot.
        """
        try:
            with socket(AF_INET, SOCK_STREAM) as s:
                s.connect((ROBOT_IP, DASHBOARD_SERVER_PORT))
                response = s.recv(1024).decode('utf-8')
                print(response)

                self.power_on_and_initialize_robot(s)
        except ConnectionRefusedError as e:
            print(f"Error: {e}")

    def power_on_and_initialize_robot(self, s: socket):
        """
        Powers on the robot listening at ROBOT_IP / DASHBOARD_SERVER_PORT, 
        - waits for it to be powered on
        - disengages the brakes

        Is able to handle robot starting from any state, including powered on, brake disengaged, etc.
        """

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