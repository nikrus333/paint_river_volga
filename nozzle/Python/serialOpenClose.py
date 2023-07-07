import serial
import time

class closeOpen:
    def __init__(self) -> None:
        self.SerialObj = serial.Serial(port='/dev/ttyACM0', baudrate=57600)
        print("Loading...")
        self.SerialObj.readline()
        print("Ready!")
        time.sleep(0.2)
        pass

    def close(self):
        data_str = "Off"
        out = data_str.encode('utf-8')
        self.SerialObj.write(out)
        print("Close")

    def open(self):
        data_str = "On"
        out = data_str.encode('utf-8')
        self.SerialObj.write(out)
        print("Open")

    def closePort(self):
        self.SerialObj.close()

if __name__ == "__main__":
    a = closeOpen()
    while KeyboardInterrupt:
        a.open()
        time.sleep(30)
        a.close()
        time.sleep(30)
    a.closePort()
    
