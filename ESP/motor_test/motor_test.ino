/*
  const int IS_1 = A0;
  const int IN_1 = 3;
  const int INH_1 = 12;

  const int IS_2 = A1;
  const int IN_2 = 11;
  const int INH_2 = 13;
*/
const int Ret1 = 2; // Gray
const int Ret2 = 4; //  Blue
const int PWM = 6; //Yellow



int input = 0;


void setup() {
  Serial.begin(9600);
  pinMode(Ret1, OUTPUT);
  pinMode(Ret2, OUTPUT);
  pinMode(PWM, OUTPUT);
  Serial.println("Setup done");
}
/*
  void loop(){
  //Choose whichever function you want to have in your setup !
  Serial.println("Making low");
  //biDirectionPower(0);
  delay(1000);
  Serial.begin("making high");
  biDirectionPower(255*0.98);
  delay(1000);
  }
*/

void loop() {
  while (Serial.available() == 0) {}
  input = int(Serial.parseInt());     //Register input as Int 
  Serial.print("You typed: " );
  Serial.println(input);              //Output the registered input
  if (input != 0 && abs(input) < 255) { //Checks if the value is usefull for the motor at a value between -255 and 255
    biDirectionPower(input*255/100);            // Rotate the motor
    Serial.print("Power set to: " );    //Output raw value through Serial for visual feedback
    Serial.println(input);
    //Serial.print(" Same as PWM: " );    //Output PWM value through Serial for visual feedback
    //Serial.print(input * 100 / 255);
    //Serial.println(" %");
  }
  else if ( input == 1000) {        //Stop movement
    biDirectionPower(0);
    Serial.print("Power set to: " );
    Serial.println(0);
  }
}



/*
void uniDirectionPowerOne(int inputUniPower) {
  analogWrite(IN_1, inputUniPower);
}
void uniDirectionPowerTwo(int inputUniPower) {
  analogWrite(IN_2, inputUniPower);
}*/
void biDirectionPower(int inputBiPower) {
  if (inputBiPower == 0 ) {       // Stop movement
    digitalWrite(Ret1, 0);
    digitalWrite(Ret2, 0);
    analogWrite(PWM, 0);
  }
  else if (inputBiPower > 0) {    // Move forward (Tighten)
    digitalWrite(Ret1, 0);
    digitalWrite(Ret2, 1);
    analogWrite(PWM, inputBiPower);
  }
  else {                          //Move backwards (Loosen)
    digitalWrite(Ret1, 1);
    digitalWrite(Ret2, 0);
    analogWrite(PWM, abs(inputBiPower));
  }
}
