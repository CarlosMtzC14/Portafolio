//Librerías
#include <max6675.h>
#include <AccelStepper.h>
#include "BluetoothSerial.h"

// Crear objeto BluetoothSerial
BluetoothSerial SerialBT; 

//------------------------------------------------------------------------  Constantes  ---------------------------------------------------------------------------------
//Pines
const int ENMP1 = 4;//
const int DIRMP1 = 5;//
const int SCK2 = 12;// PIN 17 (de preferencia usarlo) 12
const int CS2 = 14;//
const int VENT = 16;//
const int PULMP1 = 18;//
const int ENMP2 = 19;//
const int DIRMP2 = 21;//
const int PULMP2 = 22;//
const int LIMSW2 = 23;//
const int RPWM = 25;//
const int LPWM = 26;//
const int SCK1 = 27;//
const int NTC2 = 32;//
const int CS1 = 33;//
const int MISO1 = 34;//
const int MISO2 = 35;//
const int LIMSW1 = 13;// antes 36 ahora 13
const int NTC1 = 39;//


//Termopares
MAX6675 termopar1(SCK1, CS1, MISO1);
MAX6675 termopar2(SCK2, CS2, MISO2);

AccelStepper motor1(AccelStepper::DRIVER, PULMP1, DIRMP1);
AccelStepper motor2(AccelStepper::DRIVER, PULMP2, DIRMP2);

//****************************************************************  Definición de variables y constantes  ********************************************************************

//Temperatura interna termistores
const float Vcc = 3.3;
const float Rfija = 5000.0;

//Control, interrupciones y giro de motor
float TempParrilla;
bool RACT = true; 
bool LACT = true;
const int GIRO_IZQ = 1; //Constantes para función mandaPWM()
const int GIRO_DER = 2;
float error;
float error_1;
float integral = 0;
float derivativo = 0;

//Volteo de parrillas
int Parrilla1TdV = 0;
int Parrilla1TT = 0;
int Parrilla1Modo = 0;
int Parrilla2TdV = 0;
int Parrilla2TT = 0;
int Parrilla2Modo = 0;
float Parrilla1Ter = 0;
float Parrilla2Ter = 0;
unsigned long tInicialP1;
unsigned long tInicialP2;
bool bajandoParrilla = false;
bool parrillaAbajo = false;
bool controlActivo = true;


unsigned long previousMillis = 0;
unsigned long previousMillisBrasero = 0;

int EstadoParrilla1 = 0;
int EstadoParrilla2 = 0;
volatile bool detenerIzquierda = false;
volatile bool detenerDerecha = false;
volatile bool seguirIzquierda = false;
volatile bool seguirDerecha = false;

//Tabla de temperaturas termopar
const int numPuntos = 9;
float temperaturas[numPuntos] = {25, 30, 40, 50, 60, 70, 80, 90};
float resistencias[numPuntos] = {10, 8.314, 5.835, 4.168, 3.033, 2.24, 1.678, 1.275};

//------------------------------------------------------------  Funciones  -------------------------------------------------------------------------

//Volteo de parrillas
float leeTempNTC(int sensor);
float interpolarTemperatura(float Rmedida);
void monitoreaParrilla(int Parr);
void monitoreaTemperatura(int Parr);
void monitoreaTiempo(int Parr);

//Control de temperatura
float leeTempTermopares();
void mandaPWM(int dir, int valorPWM);
void controlTemp();

//Datos Bluetooth
void recibeDatos();
void iniciaTiempo(int Parr);

//Interrupciones
//void verificaInterrupciones();
void verificaLS();

/*
void IRAM_ATTR detieneGiroIzquierda() 
{
  bool estado = digitalRead(LIMSW1);
  
  Serial.println("Int LS2");

  if (estado) 
  {
    detenerIzquierda = true;
  } 
  else 
  {
    seguirIzquierda = true;
  }
}
void IRAM_ATTR detieneGiroDerecha() 
{
  bool estado = digitalRead(LIMSW2); //Lee estado de pin para detectar RISING o FALLING

  Serial.println("Int LS2");

  if (estado) 
  {
    detenerDerecha = true;
    digitalWrite(VENT, HIGH);
  } 
  else 
  {
    detenerDerecha = false;
    seguirDerecha = true;
    digitalWrite(VENT, LOW);
  }
}
*/
//------------------------------------------------------------------------------  Setup  ---------------------------------------------------------------------------------
void setup() 
{
  //Inicio de comunicación serial y serial Bluetooth.
  Serial.begin(115200);
  SerialBT.begin("BisTech");

  delay(500);
  Serial.println("Inicializando...");

  //Pines de salida digital
  pinMode(ENMP1, OUTPUT);
  pinMode(DIRMP1, OUTPUT);
  pinMode(VENT, OUTPUT);
  pinMode(PULMP1, OUTPUT);
  pinMode(ENMP2, OUTPUT);
  pinMode(DIRMP2, OUTPUT);
  pinMode(PULMP2, OUTPUT);

  //Pines de entrada digital
  pinMode(LIMSW1, INPUT);
  pinMode(LIMSW2, INPUT);

  //Pines PWM (usa ledc para ESP32)
  ledcAttach(RPWM, 1000, 8);
  ledcAttach(LPWM, 1000, 8);

  analogReadResolution(12);

  //Limit switch
  pinMode(LIMSW1, INPUT_PULLDOWN);
  pinMode(LIMSW2, INPUT_PULLDOWN);
  

  motor1.setMaxSpeed(100);
  motor1.setAcceleration(50);

  motor2.setMaxSpeed(100);
  motor2.setAcceleration(50);

  delay(1000);
}


//***************************************************************  CÓDIGO PRINCIPAL  ************************************************************************************

void loop() 
{
  unsigned long currentMillis = millis(); //Obtiene tiempo corrido en milisegundos
  //verificaInterrupciones(); //Bloquea o desloquea giros del motor

  verificaLS();

  if (SerialBT.available()) recibeDatos(); //Si hay mensaje en el buffer llama a función de recepción
  //Control cada 30 segundos
  if ((currentMillis - previousMillis) >= (5 * 1000)) //Condición que solo se activa cada 30 segundos       MODIFICAR TIEMPO DE CONTROL
  {
    previousMillis = currentMillis; //Actualización de valores
    //controlTemp();
    if (controlActivo == true)
    {
      controlTemp(); //Llamada a función de control de temperatura
      Serial.println("Control hecho.");
    }
    else
    {
      Serial.println("Control deshabilitado.");
    }

    //Monitoreo de parrillas y envío de datos
    int P = 1;
    monitoreaParrilla(P);
    float TempParr1 = leeTempNTC(P);
    P = 2;
    monitoreaParrilla(P);
    float TempParr2 = leeTempNTC(P);

    Serial.println(" ");
    P = 1;
    float TempBrasero = (termopar1.readCelsius());
    String mensaje = String(TempBrasero) + ";" + String(TempParr1) + ";" + String(TempParr2);

    SerialBT.println(mensaje);

    Serial.println(LACT);
    Serial.println(RACT);
  }
  
  currentMillis = millis();
  if (bajandoParrilla)
  {
    unsigned long tiempo = (currentMillis - previousMillisBrasero);

    if ((tiempo) >= (60000))
    {
      parrillaAbajo = true;
      controlActivo = true;
      bajandoParrilla = false;
      Serial.println("Brasero llegó abajo");
    }
  }
  motor1.run();
  motor2.run();
}

float leeTempNTC(int sensor) //SENSOR NTC 1 O 2
{
  int lecturaADC;

  if (sensor == 1)
  {
    lecturaADC = analogRead(NTC1);
  }
  else if (sensor == 2)
  {
    lecturaADC = analogRead(NTC2);
  }
  
  float Vout = (lecturaADC * Vcc / 4095.0) + 0.175;

  float Rterm = (Vout * Rfija) / (Vcc - Vout);
  float Rterm_k = Rterm / 1000.0;

  float temperatura = interpolarTemperatura(Rterm_k);
  Serial.print("Temp estimada en termistor "); Serial.print(sensor); Serial.print(": "); Serial.print(temperatura); Serial.println(" °C");

  return temperatura;
}

float leeTempTermopares()
{
  float TTK = (termopar1.readCelsius());
  Serial.print("Tempratura en parrillas: ");
  Serial.print(TTK);
  Serial.println(" °C");

  return TTK;
}

void mandaPWM(int dir, int valorPWM)
{
  if (dir==GIRO_IZQ && LACT)
  {
    ledcWrite(RPWM, 0);
    ledcWrite(LPWM, valorPWM);
    /////Serial.println("Giro a izquierda");
  }
  else if (dir==GIRO_DER && RACT)
  {
    ledcWrite(LPWM, 0);
    ledcWrite(RPWM, valorPWM);
    /////Serial.println("Giro a derecha");
  }
  else
  {
    ledcWrite(LPWM, 0);
    ledcWrite(RPWM, 0);
    /////Serial.println("Giro bloqueado");
  }
}

void controlTemp()
{
  float kp = 2.5, ki = 0.06, kd = 0.85, tdm = 60;

  TempParrilla = leeTempTermopares();
  ///////Serial.println("Termopares leídos.");

  error_1 = error;
  error = 185 - TempParrilla;

  //integral += ki*tdm;
  //derivativo = (error - error_1) / tdm;

  //////Serial.print("Error: ");
  //////Serial.println(error);

  if (abs(error) > 10)
  {
    int PWM = kp*error;// + integral*ki + derivativo*kd;
    int signo = PWM;
    PWM = constrain(PWM, 0, 255);

    /////Serial.print("PWM: ");
    /////Serial.println(PWM);
    if (signo < 0) 
    {
      mandaPWM(GIRO_IZQ, PWM);
    }
    else
    {
      mandaPWM(GIRO_DER, PWM);
    }
  }
  else 
  {
    int PWM = 0;
    mandaPWM(GIRO_DER, PWM);
    mandaPWM(GIRO_IZQ, PWM);
  }
}

/*
void verificaInterrupciones()
{
  if (detenerIzquierda) 
  {
    detenerIzquierda = false;
    LACT = false;
    ledcWrite(LPWM, 0);
    Serial.println("Giro a izquierda bloqueado.");
  }

  if (detenerDerecha) 
  {
    detenerDerecha = false;
    RACT = false;
    ledcWrite(RPWM, 0);
    Serial.println("Giro a derecha bloqueado.");
  }

  if (seguirIzquierda) 
  {
    seguirIzquierda = false;
    LACT = true;
    Serial.println("Giro a izquierda habilitado.");
    controlTemp();
  }

  if (seguirDerecha) 
  {
    seguirDerecha = false;
    RACT = true;
    Serial.println("Giro a derecha habilitado.");
    controlTemp();
  }
}
*/

void verificaLS()
{
  if (digitalRead(LIMSW1))
  {
    LACT = false;
    ledcWrite(LPWM, 0);
  }
  else
  {
    LACT = true;
    //controlTemp();
  }

  if (digitalRead(LIMSW2))
  {
    RACT = false;
    ledcWrite(RPWM, 0);
    digitalWrite(VENT, HIGH);
  }
  else
  {
    RACT = true;
    digitalWrite(VENT, LOW);
    //controlTemp();
  }
}

void recibeDatos()
{
  int Parrilla = 0, TiempoVolteo = 0, TiempoTotal = 0;;
  float Ter;

  String comando = SerialBT.readStringUntil('\n');  // Lee hasta el '\n'
  comando.trim(); // Elimina espacios y saltos extras
  Serial.println(comando);

  //Modo automático
  //Obtiene primer caracter para número de parrilla y segundo para corte y término
  if (comando.length() == 2)
  {
    int com1 = comando.charAt(0) - '0';
    char com2 = comando.charAt(1);
    Parrilla = com1;
    switch (com2) 
    {
      case 'A':
        Ter = 59; //Término medio
        Serial.print("Parrilla ");
        Serial.print(Parrilla);
        Serial.println(" configurada en modo automático a término medio.");
        break;

      case 'B':
        Ter = 65; //Término 3/4
        Serial.print("Parrilla ");
        Serial.print(Parrilla);
        Serial.println(" configurada en modo automático a término 3/4.");
        break;
      
      case 'C':
        Ter = 71; //Bien hecho
        Serial.print("Parrilla ");
        Serial.print(Parrilla);
        Serial.println(" configurada en modo automático a término bien cocido.");
        break;
      
      case 'X':
        //Cancela parrilla
        if (Parrilla == 1)
        {
          Parrilla1Modo = 0;
        }
        else if (Parrilla == 2)
        {
          Parrilla2Modo = 0;
        }
        Serial.print("Parrilla ");
        Serial.print(Parrilla);
        Serial.println(" deshabilitada.");
        Parrilla = 55;
    }
    if (Parrilla == 1) 
    {
      Parrilla1Modo = 1;
      Parrilla1Ter = Ter;
    }
    else if (Parrilla == 2) 
    {
      Parrilla2Modo = 1;
      Parrilla2Ter = Ter;
    }


  }

  //Modo manual
  //Obtiene parrilla y tiempo de volteo
  if (comando.length() > 2)
  {
    int pos1 = comando.indexOf(';');
    if (pos1 != -1) 
    {
      Parrilla = comando.substring(0, pos1).toInt();  // Extraer el primer número
      comando = comando.substring(pos1 + 1);  // Eliminar el primer número y ';'
    }

    int pos2 = comando.indexOf(';');  // Buscar el siguiente ';'
    if (pos2 != -1) 
    {
      TiempoVolteo = comando.substring(0, pos2).toInt();  // Extraer el segundo número
      comando = comando.substring(pos2 + 1);  // Eliminar el segundo número y ';'
    }
    TiempoTotal = comando.toInt();

    if (Parrilla == 1)
    {
      Parrilla1TdV = TiempoVolteo;
      Parrilla1TT = TiempoTotal;
      Parrilla1Modo = 2; //Modo manual
      iniciaTiempo(Parrilla);
      Serial.println("Parrilla 1 configurada en modo manual");
      Serial.print("Tiempo de volteo: ");
      Serial.println(Parrilla1TdV);
      Serial.print("Tiempo total: ");
      Serial.println(Parrilla1TT);
    }
    else if (Parrilla == 2)
    {
      Parrilla2TdV = TiempoVolteo;
      Parrilla2TT = TiempoTotal;
      Parrilla2Modo = 2; //Modo manual
      iniciaTiempo(Parrilla);
      Serial.println("Parrilla 2 configurada en modo manual");
      Serial.print("Tiempo de volteo: ");
      Serial.println(Parrilla2TdV);
      Serial.print("Tiempo total: ");
      Serial.println(Parrilla2TT);
    }
  }
  
}

void monitoreaParrilla(int Parr)
{
  
  if (Parr == 1)
  {
    if (Parrilla1Modo == 1)
    {
      monitoreaTemperatura(Parr);
    }
    else if (Parrilla1Modo == 2)
    {
      monitoreaTiempo(Parr);
    }
  }

  if (Parr == 2)
  {
    if (Parrilla2Modo == 1)
    {
      monitoreaTemperatura(Parr);
    }
    else if (Parrilla2Modo == 2)
    {
      monitoreaTiempo(Parr);
    }
  }
}

void monitoreaTemperatura(int Parr)
{
  float temp;

  if (Parr == 1)
  {
    temp = leeTempNTC(Parr);
    Serial.println(EstadoParrilla1);
    Serial.println(Parrilla1Ter);
    Serial.println(" ");
    if (temp >= Parrilla1Ter*0.7 && EstadoParrilla1 == 0)
    {
      mediaVueltaParrilla(Parr, EstadoParrilla1);
      Serial.println("Primera vuelta a parrilla 1.");
    }
    if (temp >= Parrilla1Ter && EstadoParrilla1 == 1)
    {
      mediaVueltaParrilla(Parr, EstadoParrilla1);
      Serial.println("Segunda vuelta a parrilla 1.");
    }
  }
  else if (Parr == 2)
  {
    temp = leeTempNTC(Parr);
    Serial.println(EstadoParrilla2);
    if (temp >= Parrilla2Ter*0.7 && EstadoParrilla2 == 0)
    {
      mediaVueltaParrilla(Parr, EstadoParrilla2);
      Serial.println("Primera vuelta a parrilla 2.");
    }
    if (temp >= Parrilla2Ter && EstadoParrilla2 == 1)
    {
      mediaVueltaParrilla(Parr, EstadoParrilla2);
      Serial.println("Segunda vuelta a parrilla 2.");
    }
  }
}

void monitoreaTiempo(int Parr)
{
  unsigned long tActual = millis();
  if (Parr == 1)
  {
    if ((tActual - tInicialP1 >= Parrilla1TdV*60*1000) && EstadoParrilla1 == 0)
    {
      mediaVueltaParrilla(Parr, EstadoParrilla1);
    }
    if ((tActual - tInicialP1 >= Parrilla1TT*60*1000) && EstadoParrilla1 == 1)
    {
      mediaVueltaParrilla(Parr, EstadoParrilla1);
    }
  }
  else if (Parr == 2)
  {
    if ((tActual - tInicialP2 >= Parrilla2TdV*60*1000) && EstadoParrilla2 == 0)
    {
      mediaVueltaParrilla(Parr, EstadoParrilla2);
    }
    if ((tActual - tInicialP1 >= Parrilla2TT*60*1000) && EstadoParrilla2 == 1)
    {
      mediaVueltaParrilla(Parr, EstadoParrilla2);
    }
  }
}

void iniciaTiempo(int Parr)
{
  unsigned long tInicial = millis();
  if (Parr == 1)
  {
    tInicialP1 = tInicial;
  }
  else if (Parr == 2)
  {
    tInicialP2 = tInicial;
  }
}

void mediaVueltaParrilla(int Parr, int Estado)
{
  //Bandera para bajar charola de ceniza.
  //Si ya terminó, habilitar vuelta de parrilla.
  //Verificar estado de parrilla para saber si gira a un lado u otro.
  if (parrillaAbajo)
  {
    if (Parr == 1)
    {
      if (Estado == 0)
      {
        motor1.move(200);
        Serial.println("Dando primera vuelta a parrilla 1.");
        EstadoParrilla1 = 1;
      }
      else if (Estado == 1)
      {
        motor1.move(-200);
        Serial.println("Dando segunda vuelta a parrilla 1.");
        EstadoParrilla1 = 0;
        Parrilla1Modo = 0;

        String mensaje = "1L";
        SerialBT.println(mensaje);
      }
    }
    else if (Parr == 2)
    {
      if (Estado == 0)
      {
        motor2.move(200);
        Serial.println("Dando primera vuelta a parrilla 2.");
        EstadoParrilla2 = 1;
      }
      else if (Estado == 1)
      {
        motor2.move(-200);
        Serial.println("Dando segunda vuelta a parrilla 2.");
        EstadoParrilla2 = 0;
        Parrilla2Modo = 0;

        String mensaje = "2L";
        SerialBT.println(mensaje);
      }
    }
    parrillaAbajo = false;
  }
  else
  {
    if (bajandoParrilla == false)
    {
      int PWM = 255;
      mandaPWM(GIRO_IZQ, PWM);
      previousMillisBrasero = millis();
      Serial.println("Inicio de timer 60 seg.");
      controlActivo = false;
    }
    bajandoParrilla = true;
  }
}

float interpolarTemperatura(float Rmedida) 
{
  for (int i = 0; i < numPuntos - 1; i++) {
    if (Rmedida <= resistencias[i] && Rmedida >= resistencias[i + 1]) {
      // Interpolación lineal entre dos puntos
      float x0 = resistencias[i];
      float x1 = resistencias[i + 1];
      float y0 = temperaturas[i];
      float y1 = temperaturas[i + 1];

      float temp = y0 + (Rmedida - x0) * (y1 - y0) / (x1 - x0);
      return temp;
    }
  }
  // Si está fuera del rango, se devuelve el límite más cercano
  if (Rmedida > resistencias[0]) return temperaturas[0];
  if (Rmedida < resistencias[numPuntos - 1]) return temperaturas[numPuntos - 1];

  return -1; // Valor no válido
}
