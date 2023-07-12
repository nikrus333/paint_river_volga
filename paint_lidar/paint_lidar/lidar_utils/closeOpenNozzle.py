import serial
import time

class closeOpen:
    def __init__(self, port = '/dev/ttyACM3', baudrate = 57600) -> None:
        try:
            self.port = port
            self.baudrate = baudrate
            self.nozzle_open = False
            self.SerialObj = serial.Serial(port=self.port, baudrate=self.baudrate)
            print("Loading...")
            self.SerialObj.readline()
            print("Ready!")
            time.sleep(0.2)
            self.serial_is_open = True

        except serial.SerialException:
            print(f"Not found {self.port}")
            self.serial_is_open = False
            pass

    def close(self):
        if self.nozzle_open != False:
            data_str = "Off"
            out = data_str.encode('utf-8')
            self.SerialObj.write(out)
            self.nozzle_open = False
            print("Close")

    def open(self):
        if self.nozzle_open != True:
            data_str = "On"
            out = data_str.encode('utf-8')
            self.SerialObj.write(out)
            self.nozzle_open = True
            print("Open")

    def closePort(self):
        self.SerialObj.close()