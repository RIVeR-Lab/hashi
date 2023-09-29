// This is for the dual z endstop with the Adafruit QT Py M0 (SAMD)

int button0 = 1;
int button1 = 2;
int buttonState0 = 0;
int buttonState1 = 1;

void setup()
{
    pinMode(button0, INPUT_PULLUP);
    pinMode(button1, INPUT_PULLUP);
    Serial.begin(9600);

}

void loop()
{
    buttonState0 = digitalRead(button0);
    buttonState1 = digitalRead(button1);

    if (buttonState0 == LOW && buttonState1 == HIGH) { // first button pushed, second button not pushed
        Serial.write(1);
        // Serial.println(1);
        delay(10);
    }
    else if (buttonState1 == LOW) { // button 2 is pushed
        Serial.write(2);
        // Serial.println(2);
        delay(10);
    }
    else {
        Serial.write(3);
        // Serial.println(3);
        delay(10);
    }

}
