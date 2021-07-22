void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int num = 1234;

  uint8_t bitsCount = sizeof( num ) * 8;
  char str[ bitsCount + 1 ];

  uint8_t i = 0;
  while ( bitsCount-- )
    str[ i++ ] = bitRead( num, bitsCount ) + '0';
  str[ i ] = '\0';

  Serial.println( str );
}
