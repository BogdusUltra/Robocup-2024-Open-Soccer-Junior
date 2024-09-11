#include <Servo.h>
Servo motor;

String inn = "";
bool RaspLoad = false;
int sens = 0;

String blIN = "999";
String blOUT = "999";
long timerFlagBL = millis();
long timerDrib = millis();
long timerSens = millis();

int sp1 = 0;
int sp2 = 0;
int sp3 = 0;
int sp4 = 0;
int sp5 = 0;
int old_sp5 = 0;

int flag_kon_old = 0;
int flag_kon_old1 = 0;
int flag_kon = 0;
int flag_KON = 0;

int max_drib = 2300;
int min_drib = 800;

long timerKick = millis();
long timerBl = millis();
int flagKick = 0;
bool flagKick_start = true;
bool flagKick_end = false;
int flagZummer = 0;

void setup() {
  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial.begin(9600);

  pinMode(6, OUTPUT);  // моторы
  pinMode(8, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(9, OUTPUT);

  pinMode(10, OUTPUT);  // кикер

  pinMode(11, OUTPUT);  //светодиоды
  pinMode(13, OUTPUT);

  pinMode(A2, OUTPUT);  // зуммер

  pinMode(A0, INPUT_PULLUP);  // кнопки
  pinMode(A1, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);

  pinMode(A14, INPUT);  // датчики
  pinMode(A15, INPUT);

  motor.attach(12);
  motor.writeMicroseconds(max_drib);
  delay(2000);
  motor.writeMicroseconds(min_drib);
  delay(2000);
  for (int i = 800; i < 900; i++) {
    motor.writeMicroseconds(i);
    Serial.println(i);
    delay(20);
  }
  motor.writeMicroseconds(800);


  digitalWrite(13, 1);
  Serial.println("started");
}

void M1(int speed) {
  speed = int(speed * 255 / 100);
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(6, speed);
    digitalWrite(8, 0);
  } else if (speed < 0) {
    analogWrite(8, -speed);
    digitalWrite(6, 0);
  } else {
    digitalWrite(6, 0);
    digitalWrite(8, 0);
  }
}

void M2(int speed) {
  speed = int(speed * 255 / 100);
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(2, speed);
    digitalWrite(4, 0);
  } else if (speed < 0) {
    analogWrite(4, -speed);
    digitalWrite(2, 0);
  } else {
    digitalWrite(2, 0);
    digitalWrite(4, 0);
  }
}

void M3(int speed) {
  speed = int(speed * 255 / 100);
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(5, speed);
    digitalWrite(3, 0);
  } else if (speed < 0) {
    analogWrite(3, -speed);
    digitalWrite(5, 0);
  } else {
    digitalWrite(5, 0);
    digitalWrite(3, 0);
  }
}

void M4(int speed) {
  speed = int(speed * 255 / 100);
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(9, speed);
    digitalWrite(7, 0);
  } else if (speed < 0) {
    analogWrite(7, -speed);
    digitalWrite(9, 0);
  } else {
    digitalWrite(7, 0);
    digitalWrite(9, 0);
  }
}

void sensors() {
  int sens1 = analogRead(A14);
  int sens2 = analogRead(A15);
  // Serial.print(sens1);
  // Serial.print(" ");
  // Serial.println(sens2);
  if ((sens1 < 450 or sens2 < 450)) {
    timerSens = millis();
    sens = 1;
  } 
  else{
    if (timerSens + 200 < millis()){
      sens = 0;
    }
  
  }
}

void checkKicker() {
  flag_kon_old1 = flag_kon_old;
  flag_kon_old = flag_kon;
  flag_kon = digitalRead(A3);

  if (flag_kon == 1 and flag_kon_old == 1 and flag_kon_old1 == 1) {
    flag_KON = 1;
  }
  if (flag_kon == 0 and flag_kon_old == 0 and flag_kon_old1 == 0) {
    flag_KON = 0;
  }


  if (flagKick == 1) {
    flagKick_start = true;
  
    for (int i = sp5; i > 800; i=i-2) {
      motor.writeMicroseconds(i);
      delay(1);
    }
    delay(500);
    M1(100);
    M2(100);
    M3(100);
    M4(100);
    delay(300);
    analogWrite(10, 255);
    delay(200);
    M1(0);
    M2(0);
    M3(0);
    M4(0);
    while(Serial2.available() > 0){
      Serial2.read();
    }
    flagKick = 0;
  }

  else if (flagKick == 2) {
    flagKick_start = true;
  
    for (int i = sp5; i > 800; i=i-2) {
      motor.writeMicroseconds(i);
      delay(1);
    }
    delay(500);
    analogWrite(10, 255);
    delay(500);
    while(Serial2.available() > 0){
      Serial2.read();
    }
    flagKick = 0;
  }

  if (flag_KON == 1) {
    flagKick_start = false;
    analogWrite(10, 255);
  }

  if (flag_KON == 0 and flagKick_start == false) {
    if (flagKick_end == false) {
      delay(10);
    }
    // delay(5);
    analogWrite(10, 0);
    flagKick_end = true;
  } else {
    flagKick_end = false;
  }

  // if (flagKick_end == true) {
  //   analogWrite(10, 0);
  // }
}

void dribler(int sp) {
  if (timerDrib + 1 < millis() ) {
    timerDrib = millis();
    if (sp < old_sp5) {
      old_sp5 = old_sp5 - 2;
    } else if (sp > old_sp5) {
      old_sp5 = old_sp5 + 2;
    }
    motor.writeMicroseconds(old_sp5);
  }
}

void readuart() {
  if (Serial2.available() > 0) {
    inn = Serial2.readStringUntil('$');
    // Serial.println(inn);
    if (RaspLoad == false) {
      sp1 = 0;
      sp2 = 0;
      sp3 = 0;
      sp4 = 0;
      sp5 = 0;
      flagKick = 0;
      flagZummer = 0;
      if (inn == "2002002002001000999") {
        RaspLoad = true;
        digitalWrite(11, 1);
        digitalWrite(A2, 1);
        delay(300);
        digitalWrite(13, 0);
      }

    } else {
      if (inn.length() == 19) {
        sp1 = inn.substring(0, 3).toInt() - 200;
        sp2 = inn.substring(3, 6).toInt() - 200;
        sp3 = inn.substring(6, 9).toInt() - 200;
        sp4 = inn.substring(9, 12).toInt() - 200;
        // sp5 = 1500 + int((inn.substring(12, 14).toInt() - 10) * (max_drib - min_drib - 200) / 99 / 2);
        sp5 = map(inn.substring(12, 14).toInt(), 10, 99, 800, 2300);
        // sp5 = constrain(sp5, 800, 2300);
        flagKick = inn.substring(14, 15).toInt();
        flagZummer = inn.substring(15, 16).toInt();
        blOUT = inn.substring(16, 19);
      }
      sensors();
      // bluetooth();
      String mes = String(digitalRead(A0)) + String(digitalRead(A1)) + String(sens) + blIN + "$";
      // Serial.println(mes);
      Serial2.print(mes);
    }
  }
}

void bluetooth() {
  if (Serial3.available() > 0) {
    inn = Serial3.readStringUntil('$');
    if (inn.length() == 3) {
      blIN = inn;
    }
  }
  if (timerBl + 300 < millis()) {
    Serial3.print(blOUT + "$");
    timerBl = millis();
  }
}

void loop() {
  bluetooth();
  readuart();
  digitalWrite(A2, flagZummer);
  M1(sp1);
  M2(sp2);
  M3(sp3);
  M4(sp4);

  checkKicker();
  dribler(sp5);
}
