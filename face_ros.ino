#include <Servo.h>

Servo servoX;  // Servo for X-axis
Servo servoY;  // Servo for Y-axis

int xpos = 90; // Default servo positions
int ypos = 90;

void setup() {
    Serial.begin(9600);
    servoX.attach(9);  // Connect X servo to pin 9
    servoY.attach(11); // Connect Y servo to pin 10

    servoX.write(xpos);
    servoY.write(ypos);

    //Serial.println("Arduino ready to receive data...");
}

void loop() {
    if (Serial.available() > 0) {
        String data = Serial.readStringUntil('\n');  // Read incoming serial data
        data.trim();  // Remove any trailing spaces or newlines

        Serial.println("Received: " + data);  // Echo received data

        // Parse incoming data
        if (data.startsWith("X") && data.indexOf(" Y") != -1) {
            int xIndex = data.indexOf("X") + 1;
            int yIndex = data.indexOf("Y") + 1;

            int xVal = data.substring(xIndex, data.indexOf(" ", xIndex)).toInt();
            int yVal = data.substring(yIndex).toInt();

            xpos = constrain(xVal, 0, 180);
            ypos = constrain(yVal, 0, 180);

            servoX.write(xpos);
            servoY.write(ypos);

            //Serial.println("Moved Servos -> X: " + String(xpos) + " Y: " + String(ypos));
        }
    }
}
