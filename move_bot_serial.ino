#define L_MOTOR_POSITIVE 9
#define L_MOTOR_NEGATIVE 10
#define L_MOTOR_ENABLE 3
#define R_MOTOR_POSITIVE 6
#define R_MOTOR_NEGATIVE 11
#define R_MOTOR_ENABLE 5
#define MAX_SPEED_R 240
#define MAX_SPEED_L 180

/*
 * Movements:
 *     W := forward
 *     S := stop
 *     D := right
 *     A := left
 *     R := extreme right
 *     L := extreme left
 *     X := reverse
 */

void setup() {
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  Serial.begin(9600);
}

int incomingByte = Serial.read();

void loop() {
  if (Serial.available() > 0) {
    incomingByte = Serial.read();

    Serial.print("I received: ");
    Serial.println(incomingByte);
    if (incomingByte == 'A') {
      digitalWrite(L_MOTOR_POSITIVE, LOW);
      digitalWrite(L_MOTOR_NEGATIVE, LOW);

      digitalWrite(R_MOTOR_POSITIVE, HIGH);
      digitalWrite(R_MOTOR_NEGATIVE, LOW);

      analogWrite(L_MOTOR_ENABLE, MAX_SPEED_L);
      analogWrite(R_MOTOR_ENABLE, MAX_SPEED_R);
    }
    else if (incomingByte == 'D') {
      digitalWrite(L_MOTOR_POSITIVE, HIGH);
      digitalWrite(L_MOTOR_NEGATIVE, LOW);

      digitalWrite(R_MOTOR_POSITIVE, LOW);
      digitalWrite(R_MOTOR_NEGATIVE, LOW);

      analogWrite(L_MOTOR_ENABLE, MAX_SPEED_L);
      analogWrite(R_MOTOR_ENABLE, MAX_SPEED_R);
    }
    else if (incomingByte == 'W') {
      digitalWrite(L_MOTOR_POSITIVE, HIGH);
      digitalWrite(L_MOTOR_NEGATIVE, LOW);

      digitalWrite(R_MOTOR_POSITIVE, HIGH);
      digitalWrite(R_MOTOR_NEGATIVE, LOW);

      analogWrite(L_MOTOR_ENABLE, MAX_SPEED_L);
      analogWrite(R_MOTOR_ENABLE, MAX_SPEED_R);
    }
    else if (incomingByte == 'S') {
      digitalWrite(L_MOTOR_POSITIVE, LOW);
      digitalWrite(L_MOTOR_NEGATIVE, LOW);

      digitalWrite(R_MOTOR_POSITIVE, LOW);
      digitalWrite(R_MOTOR_NEGATIVE, LOW);

      analogWrite(L_MOTOR_ENABLE, MAX_SPEED_L);
      analogWrite(R_MOTOR_ENABLE, MAX_SPEED_R);
    }
    else if (incomingByte == 'R') {
      digitalWrite(R_MOTOR_POSITIVE, LOW);
      digitalWrite(R_MOTOR_NEGATIVE, HIGH);

      digitalWrite(L_MOTOR_POSITIVE, HIGH);
      digitalWrite(L_MOTOR_NEGATIVE, LOW);

      analogWrite(L_MOTOR_ENABLE, MAX_SPEED_L);
      analogWrite(R_MOTOR_ENABLE, MAX_SPEED_R);
    }
    else if (incomingByte == 'L') {
      digitalWrite(R_MOTOR_POSITIVE, HIGH);
      digitalWrite(R_MOTOR_NEGATIVE, LOW);

      digitalWrite(L_MOTOR_POSITIVE, LOW);
      digitalWrite(L_MOTOR_NEGATIVE, HIGH);

      analogWrite(L_MOTOR_ENABLE, MAX_SPEED_L);
      analogWrite(R_MOTOR_ENABLE, MAX_SPEED_R);
    }
    else if (incomingByte == 'X') {
      digitalWrite(R_MOTOR_POSITIVE, LOW);
      digitalWrite(R_MOTOR_NEGATIVE, HIGH);

      digitalWrite(L_MOTOR_POSITIVE, LOW);
      digitalWrite(L_MOTOR_NEGATIVE, HIGH);

      analogWrite(L_MOTOR_ENABLE, MAX_SPEED_L);
      analogWrite(R_MOTOR_ENABLE, MAX_SPEED_R);
    }
  }
}

