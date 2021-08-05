int s0 = 17;
int s1 = 16;
int s2 = 4;
int s3 = 2;

String s0Out;
String s1Out;
String s2Out;
String s3Out;

int z = 13;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(z, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int o = 0; o < 16; o++) {

    uint8_t bitsCount = sizeof( o );
    char str[ bitsCount + 1 ];

    uint8_t i = 0;
    while ( bitsCount-- )
      str[ i++ ] = bitRead( o, bitsCount ) + '0';
    str[ i ] = '\0';

    s0Out = String(str[0]);
    s1Out = String(str[1]);
    s2Out = String(str[2]);
    s3Out = String(str[3]);

    digitalWrite(s0, LOW);
    digitalWrite(s1, LOW);
    digitalWrite(s2, LOW);
    digitalWrite(s3, HIGH);

    Serial.print(o);
    Serial.print(" = ");
    Serial.println(analogRead(z));
    
    delay(500);
  }
}
