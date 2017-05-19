#define buttonAmount 12
#define toggleAmount 10

int button[] = {52,50,48,46,44,42,40,38,36,34,32,30}; 
int toggle[] = {43,37,53,49,41,39,51,45,47,35}; 
int outData[27];

void setup() {
  Serial.begin(9600);
  for( byte a = 0; a<buttonAmount; a++){
    pinMode(button[a],INPUT);
  }
  for( byte a = 0; a<toggleAmount; a++){
    pinMode(toggle[a],INPUT);
  }

}

void loop() {
  outData[0] = 254; 
  for( int a = 0; a < buttonAmount; a++){
    outData[a+1] = digitalRead(button[a]);
  }
  for( int a = 0; a < toggleAmount; a++){
    outData[a+13] = digitalRead(toggle[a]);
  }
  outData[23] = map(analogRead(A0),0,1023,0,250);
  outData[24] = map(analogRead(A1),0,1023,0,250);
  outData[25] = map(analogRead(A2),0,1023,0,250);
  outData[26] = 255;

  for(int a = 0; a < sizeof(outData); a++){
    Serial.write(outData[a]);
  }
}
