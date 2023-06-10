#include <Wire.h>
#include <INA226_WE.h>
#define I2C_ADDRESS 0x40

#define sampleTime 5000                             // Muestreo
#define in1HBridge 33                               // Puente H  
#define in2HBridge 25   
// Pines INA
// sda   pin 21
// scl   pin 22

//   Variables
float ref = 0, ref2 = 0;
float ent;
float ramp = 0;
double cv = 0, cv1 = 0, cv2 = 0;
double error = 0, error1 = 0, error2 = 0;
float Tm = 0.1;

//   Comunicacion serial
byte cmd1 = 0, cmd2 = 0;                            // Byte para la comunicación serie (cmd = comando).
int control;
int senal;

//   Timer
hw_timer_t * timer = NULL;
volatile int samples = 0;
volatile bool flagSample = 0;
bool direction = false;

//   INA
INA226_WE ina226 = INA226_WE(I2C_ADDRESS);
float current_mA = 0.0; 
portMUX_TYPE synch = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer(){
  portENTER_CRITICAL_ISR(&timerMux0);
  flagSample = 1;
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void setup() {
  Serial.begin(115200);
  pinMode(in1HBridge, OUTPUT);
  pinMode(in2HBridge, OUTPUT);
  ledcSetup(0, 1000, 8);
  
  //   Config INA
  Wire.begin();
  ina226.init();
  ina226.setAverage(AVERAGE_256);                                    // Modo
  ina226.setConversionTime(CONV_TIME_140);                           // Tiempo de conversión
  ina226.setMeasureMode(CONTINUOUS);                                 // Modo
  ina226.setCurrentRange(MA_800);                                    // Ganancia
  ina226.setCorrectionFactor(0.9);                                   // Factor de correccion
  ina226.waitUntilConversionCompleted();                             // Si se comenta el primer valor de lectura puede ser cero
 
  // Inicializacion Timer
  timer = timerBegin(0, 80, true);                                   // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer, &onTimer, true);                       // Triggered 
  timerAlarmWrite(timer, sampleTime, true);                          // En microseconds
  timerAlarmEnable(timer);                                           // Habilitar
}

void loop() {
  CmdControl();
  if(flagSample == 1)
  {
    flagSample = 0;
    ina226.readAndClearFlags();
    current_mA = ina226.getCurrent_mA();
    int u;

    // Descomentar para tener los diferentes tipos de entrada (Cuadrado, Rampa y senoidal)
    error = 200 - current_mA; // Si se descomenta Switch senal, comentar esta linea
    // switch(senal)
    // {
    //   case 1: error = ref - current_mA; ent = ref;                                                               // Cuadrada
    //     break;
    //   case 2: ramp = ramp+0.05; if(ramp >= (ref*1.5)){ ramp = 0;} ent = ramp; error = ramp - current_mA;         // Rampa
    //     break;
    //   case 3: ref2 = ref2 + 0.01; ent = (sin(ref2)*ref); error = (sin(ref2)*(ref*1.5)) - current_mA;             // Senoidal
    //     break;
    // }

    switch(control)
    {
      case 1: cv = 0.0955*error + 0.0683*error1 + 0.9437*cv1 - 0.9437*cv2;                                          // Continuo Discreto  (oscilla)
        break;
      case 2: cv = (0.001321*error - 0.00127886*error1 + 0.000313784*error2) + 1.3816*cv1 - 0.3816*cv2;             // Discreto Discreto 
        break;
      case 3: cv = 0.003974*(error - 1.00402617*error1 + 0.0448695*error2) + 1.3816*cv1 - 0.3816*cv2;               // LGR
        break;
      case 4: cv = 0.009849*error - 0.00815*error1 + 0.00002*error2 + 1.3816*cv1 - 0.3816*cv2;                      // Oscilaciones muertas 
        break;
      case 5: cv = 0.003974*error - 0.00399*error1 + 0.0001771*error2 + 1.3816*cv1 - 0.3816*cv2;                    // Anulacion de planta
        break;
      case 6: cv = 0;                                                                                               // Servosistema F
        break;
    }

    cv2 = cv1;
    cv1 = cv;
    error2 = error1;
    error1 = error;
    
    //SATURAMOS LA SALIDA DEL PID
    if(cv > 255){
      u = 255;
      
      digitalWrite(in1HBridge, abs(u));
      digitalWrite(in2HBridge, LOW);
    }
    if(cv < -255){  
      u = 255;
      
      digitalWrite(in1HBridge, LOW);
      digitalWrite(in2HBridge, abs(u)); 
    }
    if(cv >= 0 && cv <= 255){
      u = map(cv, 0, 12, 0, 255);

      digitalWrite(in1HBridge, abs(u));
      digitalWrite(in2HBridge,LOW);
    }
    if(cv <= 0 && cv >= -255){
      u = map(cv, -12, 0, -255, 0);

      digitalWrite(in1HBridge, abs(u));
      digitalWrite(in2HBridge,LOW);
    }
      
//    Serial.print(current_mA/10);// Salida
//    Serial.print(",");
//    Serial.print(ent); // Referencia
//    Serial.print(",");
//    Serial.println(cv); // Control
//    Serial.println(String(current_mA) + "," + String(ent) + "," + String(cv));
      Serial.println(String(float (current_mA),2)+';'+String(float (ent),2)+';'+String(double (error),2)+';');
  }  
}

void CmdControl()
{
  if (Serial.available() > 0)                             // Comprueba si ha recibido algún dato por el terminal serie.
  {
    cmd1 = 0;                                             // Por seguridad se limpia cmd.
    cmd1 = Serial.read();                                 // cmd guarda el byte recibido. (AX0.0001) = (PI CD, Tren de pulsos y 0.001 de torque)

    if (cmd1 > 31)
    {
      byte flags = 0;                                     // Borramos la bandera que decide lo que hay que imprimir.
   
      if (cmd1 >  'Z') cmd1 -= 32;                        // Si una letra entra en minúscula la covierte en mayúscula.
      
      switch(cmd1)                                                                          
      {                                                                                      
        case 'A': control = 1; CmdSenal(); break; 
        case 'B': control = 2; CmdSenal(); break; 
        case 'C': control = 3; CmdSenal(); break; 
        case 'D': control = 4; CmdSenal(); break; 
        case 'E': control = 5; CmdSenal(); break; 
        case 'F': control = 6; CmdSenal(); break;  
      }     
    }
  }
}

void CmdSenal()
{
  cmd2 = Serial.read();
  if (cmd2 >  'Z') cmd2 -= 32;
  switch(cmd2)
  {
    case 'X': senal = 1; ref = Serial.parseFloat()*100.0; break;
    case 'Y': senal = 2; ref = Serial.parseFloat()*100.0; break;
    case 'Z': senal = 3; ref = Serial.parseFloat()*100.0; break;
  }
}
