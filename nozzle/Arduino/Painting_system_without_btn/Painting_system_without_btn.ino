#include <DxlMaster.h>

DynamixelDevice button(0x17);
DynamixelDevice key(0x11);

bool btn1State = false;

unsigned long timer = 0;

void setup() {
  DxlMaster.begin(57600);
  Serial.begin(57600);
  Serial.setTimeout(3);
  
  Serial.println(key.init());
  Serial.println();

  timer = millis();
}

void loop() {

  if (Serial.available()) {
    String command = Serial.readString();
    command.trim(); //remove \r\n
    if (command == "On" || command == "on") {
      btn1State = true;
    }

    if (command == "Off" || command == "off") {
      btn1State = false;
    }

    if (btn1State) {
      key.ping();
      key.write(28, 255);
      Serial.println("State 1");
      delay(10);
    }
    else {
      key.ping();
      key.write(28, 0);
      Serial.println("State 0");
      delay(10);
    }
  }
}
