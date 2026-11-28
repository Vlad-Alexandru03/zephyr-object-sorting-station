const int ENB = 1;
const int IN3 = 2;
const int IN4 = 14;

void setup() {
    pinMode(ENB, OUTPUT);
    digitalWrite(ENB, LOW);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    Serial.begin(115200);
    Serial.println("Running motor for one second.");
    digitalWrite(ENB, HIGH);
    delay(1000);
    digitalWrite(ENB, LOW);
    Serial.println("Motor stopped.");
}

void loop() {
    delay(1000);
}
