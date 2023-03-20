/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

const int pwmPin1 = D5;
const int pwmPin2 = D6;
const int pwmPin3 = D0; //D0 is the same as the BUILT IN LED pin. The built in LED will pulse opposite D0 PWM Signal.
const int pwmPin4 = D8;


void setup() {
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(pwmPin3, OUTPUT);
  pinMode(pwmPin4, OUTPUT);
}

void loop() {
  // increase the LED brightness
  for(int dutyCycle = 0; dutyCycle < 255; dutyCycle++){   
    // changing the LED brightness with PWM
    analogWrite(pwmPin1, dutyCycle);
    analogWrite(pwmPin2, dutyCycle);
    analogWrite(pwmPin3, dutyCycle);
    analogWrite(pwmPin4, dutyCycle);
    delay(1);
  }

  // decrease the LED brightness
  for(int dutyCycle = 255; dutyCycle > 0; dutyCycle--){
    // changing the LED brightness with PWM
    analogWrite(pwmPin1, dutyCycle);
    analogWrite(pwmPin2, dutyCycle);
    analogWrite(pwmPin3, dutyCycle);
    analogWrite(pwmPin4, dutyCycle);
    delay(1);
  }
}