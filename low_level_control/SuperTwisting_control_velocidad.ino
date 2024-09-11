
hw_timer_t *timer = NULL;
//Declaración de Variables
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Pin 2 y 3 para señales A y B de encoder
const int interruptPinA = 19;
const int interruptPinB = 21;

// Contador: cuenta las interrupciones debido a un flanco de subida en el encoder
double counter = 0; 

// Variable para registrar la velocidad del motor.
double VelocidadEnRPM = 0.0; 
// Tiempo transcurrido 
double tiempo=0.0;  
double tiempo_ant =0.0;    
// Señal de entrada
double DutyCycle = 0.0;  
// Señales de control
double Error = 0, m=0, Error_D=0, Error_Ant=0;
double VelReq = 60; 

double s = 0;
double u = 0;
double d_w = 0;
double w = 0;

//double c = 70;
double alpha = 500;
double beta = 21;

int act_control = 0;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Configuración de Pines

const int Enable = 16;
const int In1 = 18;
const int In2 = 23;

int i = 11;

// Configuraciòn PWM
const int PWM_Channel1 = 0;
const int PWM_Freq = 31000;
const int PWM_Res = 8;

//
double dt = 0.02;

int signa(float s){
  if (s > 0) {
    return 1;
  }
  else if (s < 0) {
    return (-1);
  }
  else {
    return 0;
  }
}

void setup() {
  //pinMode(Enable,OUTPUT);
  Serial.begin(115200); 
  Serial.println("Ready!");
  pinMode(In1,OUTPUT);
  pinMode(In2,OUTPUT);

  // El Pin interruptPinA es entrada, se conecta a la señal del encoder Canal A (tren de pulsos).
  pinMode(interruptPinA, INPUT);
  // El Pin interruptPinB es entrada, se conecta a la señal del encoder Canal B (tren de pulsos).
  pinMode(interruptPinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_countingA, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_countingB, RISING);

  // Timer1: Se dispara cada 20000 microsegundos=0.02 Segundos.
  timer=timerBegin(2,80,true); 
  timerAttachInterrupt(timer, &ISR_RevolucionesPorMinuto, true); 
  // Activa la interrupción: Ejecuta la función RevolucionesPorMinuto            
  timerAlarmWrite(timer, 20000, true);
  timerAlarmEnable(timer);

  ledcSetup(PWM_Channel1, PWM_Freq, PWM_Res);
  ledcAttachPin(Enable, PWM_Channel1);

}

void loop() {
  // Generación de señal de entrada
  // if (tiempo <= 3){
  //   VelReq = 0;
  // }
  // else{
  //   // if (i<70 && i>10){
  //   //   i++;
  //   // }
  //   // else {
  //   //   i = 11;
  //   // }
  //   // VelReq = i;
  //   VelReq = 50;
  // }
 //VelReq = 80;
 VelReq =60+15*sin(2*3.141592*0.5*tiempo_ant*1000);

  if (act_control == 1){
  dt = (millis()-tiempo_ant)/1000;
  VelocidadEnRPM = 60*counter/(dt*495*2);   
                              
  tiempo = tiempo + dt; 

  //m = 120;
  //m = VelReq;

  
  Error = VelReq - VelocidadEnRPM;
  Error_D = (Error - Error_Ant)/dt;
  Error_Ant = Error;

  // m = kp*Error + ki*Error_Int + kd*Error_D;

  //s = Error_D + c*Error;
  d_w = signa(Error) * dt;
  w = w + d_w;
  u = alpha * sqrt(abs(Error)) * signa(Error) + beta * w;

  
  if(u>255){
    u = 255;
  }
  else if(u<0){
    u = 0;
  }
  
  DutyCycle = u;
  //DutyCycle = 170;

  
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  ledcWrite(PWM_Channel1, DutyCycle);
  
  
  // Se reinicia el contador para que vuelva a contar hasta la siguiente interrupción.
  
  counter = 0;
  act_control = 0;
  tiempo_ant = millis();
  }
  
  // Se imprime el valor del DutyCycle y de la velocidad del motor en RPM
  //Serial.println("DutyCycle");
  Serial.print(DutyCycle);
  //Serial.println("\t");
  //Serial.println("VelReq");
  Serial.print(",");
  Serial.print(VelReq);
  Serial.print(",");
  //Serial.println("\t");
  //Serial.println("VelocidadEnRPM");
  Serial.println(VelocidadEnRPM);
  //Serial.println("\t");




  
}

void ISR_countingA() {
  //Contador
  if(digitalRead(interruptPinB)){
    
    counter++;                
  }
  else{
    counter--;
  }         
}

void ISR_countingB() {
  //Contador
  if(digitalRead(interruptPinA)){
    counter--;                   
  }
  else{
    counter++;
 }         
}

void ISR_RevolucionesPorMinuto(){
  // Velocidad en Revoluciones por minuto: esta parte "(counter/CuentasPorRevolucion)/Ts" 
  // da las revoluciones por segundo a las que gira el encoder, 
  // se multiplica por 60 segundos para obtener esta velocidad en RPM
  act_control = 1;
                                                        
}
