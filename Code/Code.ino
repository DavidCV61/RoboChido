#include <QTRSensors.h>

int in3=10;
int in4=5; //Motor izquierdo
int EnB=11;


int in1=4;
int in2=9; //Motor derecho
int EnA=3;

int led=6;
int boton=2;

int  derivativo=0, proporcional=0, integral=0; //parametros
int  salida_pwm=0, proporcional_pasado=0;

int velocidad=120;     
float Kp=0.18, Kd=4, Ki=0.001;
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup(){Serial.begin(9600);
  // configure the sensors
  pinMode(8,INPUT);
  pinMode(6,OUTPUT);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5,A6,A7}, SensorCount);
  qtr.setEmitterPin(13);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  digitalWrite(led,HIGH);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
    Serial.println("calibrando...");
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  parpadeo();
  // print the calibration minimum values measured when emitters were on
  
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

  while(true)
{
    int x=digitalRead(boton); //leemos y guardamos el valor
                                      // del boton en variable x
    delay(100);
    if(x==0) //si se presiona boton 
    {

        break; 
    }
}
parpadeo();
}

void loop()
{
 uint16_t position = qtr.readLineBlack(sensorValues);


  proporcional = (position) - 3500; // set point es 3500, asi obtenemos el error
  integral=integral + proporcional_pasado; //obteniendo integral
  derivativo = (proporcional - proporcional_pasado); //obteniedo el derivativo
  
  if (integral>1000) integral=1000; //limitamos la integral para no causar problemas
  if (integral<-1000) integral=-1000;
  
  
salida_pwm =( proporcional * Kp ) + ( derivativo * Kd )+(integral*Ki);

  
  if (  salida_pwm > velocidad )  salida_pwm = velocidad; //limitamos la salida de pwm
  if ( salida_pwm < -velocidad )  salida_pwm = -velocidad;

  Serial.print(position);
  Serial.print(" ");
  Serial.print(salida_pwm);  
  Serial.print(" ");
  Serial.print(velocidad+salida_pwm);
  Serial.print(" ");
  Serial.println(velocidad-salida_pwm);
   if (salida_pwm < 0)
 {
  motores(velocidad+salida_pwm, velocidad);
 }
 if (salida_pwm >0)
 {
  motores(velocidad, velocidad-salida_pwm);
 }

 proporcional_pasado = proporcional;
}

void motores(int motor_izq, int motor_der)
{
  if ( motor_izq >= 0 )  //motor izquierdo
 {
  analogWrite(EnA,255-motor_izq);
  digitalWrite(in1,HIGH); 
  digitalWrite(in2,LOW);
   
 }
 else
 {
  
  motor_izq = motor_izq*(-1); 
  analogWrite(EnA,motor_izq);
  digitalWrite(in1,LOW); 
  digitalWrite(in2,HIGH);

   
 }

  if ( motor_der >= 0 ) //motor derecho
 {
 analogWrite(EnB,255-motor_der);
  digitalWrite(in3,HIGH); 
  digitalWrite(in4,LOW);
 }
 else
 {

  motor_der= motor_der*(-1);
  analogWrite(EnB,motor_der);
  digitalWrite(in3,LOW); 
  digitalWrite(in4,HIGH);

 }

  
}

void parpadeo(){
digitalWrite(led,LOW);
delay(100);
digitalWrite(led,HIGH);
delay(100);
digitalWrite(led,LOW);
delay(100);
digitalWrite(led,HIGH);
delay(100);
digitalWrite(led,LOW);
delay(100);
}
