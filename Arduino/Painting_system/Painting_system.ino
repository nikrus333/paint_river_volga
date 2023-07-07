#include <DxlMaster.h>

DynamixelDevice button(0x17);
DynamixelDevice key(0x11);


uint8_t btn1 = 0;
bool btn1State = false;
bool btn1LastState = false;
bool button1On = false;

unsigned long timer = 0;

void setup() {
  DxlMaster.begin(57600);
  Serial.begin(57600);
  Serial.setTimeout(3);

  Serial.println(button.init());
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
      Serial.println("State: 1");
    }
    else {
      key.ping();
      key.write(28, 0);
      Serial.println("State: 0");
    }
  }

  static int button_count = 0;
  if (millis() - timer > 10) {   //  Опрос кнопки на вентиляцию.
    timer = millis();

    button.ping();
    button.read((uint8_t)27, (uint8_t)1, &btn1);
    //Serial.print("btn1: ");
    //Serial.println(btn1);
    if (btn1 == 1) {
      button_count++;
      //Serial.println(button_count);
    } else {
      button_count = 0;
    }

    if (button_count == 5) {
      btn1State = !btn1State;
      Serial.print("State: ");
      Serial.println(btn1State);
    }

    if (btn1State) {      // В зависимости от состояния замыкаем силовые ключи,
      key.ping();
      key.write(28, 255);
      //Serial.println("BTN On");
    }
    else {
      key.ping();
      key.write(28, 0);
      //Serial.println("BTN Off");
    }
  }
}
