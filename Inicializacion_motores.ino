#include<Servo.h>
// Crear objetos de la clase Servo para cada motor 
Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;
 
int vel = 1000; //amplitud del pulso
 
void setup()
{
  //Asignar pines a cada ESC
  ESC1.attach(8);
  ESC2.attach(9);
  ESC3.attach(10);
  ESC4.attach(11);
  
  //Activacion de los ESC
  ESC1.writeMicroseconds(2000);
  ESC2.writeMicroseconds(2000);
  ESC3.writeMicroseconds(2000);
  ESC4.writeMicroseconds(2000);//1000 = 1ms
  //Cambia el 1000 anterior por 2000 si
  //tu ESC se activa con un pulso de 2ms
  delay(5000); //Esperar 5 segundos para hacer la activacion
   
  //Iniciar puerto serial
  Serial.begin(9600);
  Serial.setTimeout(10); 
}
 
void loop()
{
  if(Serial.available() >= 1)
  {
    vel = Serial.parseInt(); //Leer un entero por serial
    if(vel != 0)
    {
      //Generar pulsos con los umeros recibidos 
      ESC1.writeMicroseconds(vel);
      ESC2.writeMicroseconds(vel);
      ESC3.writeMicroseconds(vel);
      ESC4.writeMicroseconds(vel);
    }
  }
}
