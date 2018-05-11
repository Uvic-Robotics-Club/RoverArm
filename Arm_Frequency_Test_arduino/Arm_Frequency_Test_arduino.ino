#define UPPER_PWM 6
//define UPPER_PWM LED_BUILTIN
#define UPPER_DIR 5
#include <TimerOne.h>

volatile int duty = 0;
volatile int duty2 = 0;
double percent = 0;
const int full_time = 500;

long period = 0; // 10,000us = 100Hz
double temp_period = 0;
int frequency = 52; // found by trial and error at 13.5V
//int frequency = 1;
volatile char output1 = 0;
volatile int counting = 0;


void blink_callback_function(void) {
  output1 = PORTD;
  output1 = (counting > duty) ? output1 & B10111111 : output1 | B01000000;
  output1 = (counting > duty2) ? output1 & B01111111 : output1 | B10000000;
  PORTD = output1;
  counting = counting > full_time ? 0 : counting + 1;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(UPPER_DIR, OUTPUT);
  pinMode(UPPER_PWM, OUTPUT);
  analogReference(INTERNAL);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  Serial.begin(115200);
  period = frequency_to_microsecond_period(frequency * full_time);
  Timer1.initialize(period);
  Timer1.attachInterrupt(blink_callback_function);
  //Timer1.pwm(UPPER_PWM, 0);
  Serial.print("the built in led is on pin ");
  Serial.println(LED_BUILTIN);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(analogRead(A0));
  Serial.print(" ");
  Serial.print(analogRead(A1));
  Serial.print(" ");
  Serial.print(analogRead(A2));
  Serial.print(" ");
  Serial.println(analogRead(A3));
  //delay(300);

}

long frequency_to_microsecond_period(int frequency) {
  temp_period = frequency;
  temp_period = 1 / temp_period;
  temp_period = temp_period * 1000000; // convert from seconds into microseconds
  return long(temp_period);
}

void serialEvent() {
  duty = Serial.parseInt();

  //Serial.println(duty);
  digitalWrite(UPPER_DIR, (duty > 0 ? 1 : 0));
  duty = abs(duty);

  duty2 = Serial.parseInt();
  if (false and Serial.available() > 1) {
    period = frequency_to_microsecond_period(frequency);
    Timer1.setPeriod(period);
  }
  percent = duty;
  percent = percent / 100;
  duty = percent * full_time;
  percent = duty2;
  percent = percent / 100;
  duty2 = percent * full_time;
  
  //Timer1.pwm(UPPER_PWM, duty);
}
