import serial
import time

class closeOpen:
    def __init__(self) -> None:
        self.SerialObj = serial.Serial(port="/dev/ttyACM1", baudrate=115200)
        #self.SerialObj.readline()
        print('start')
        time.sleep(10)

    def close(self):
        data_str = "Off"
        out = data_str.encode('utf-8')
        self.SerialObj.write(out)

    def open(self):
        data_str = "On"
        out = data_str.encode('utf-8')
        self.SerialObj.write(out)

        
if __name__ == '__main__':
    a = closeOpen()
    while True:
        print('open')
        a.open()
        time.sleep(5)
        print('close')
        a.close()
        time.sleep(5)
