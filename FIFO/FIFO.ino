#include "SparkFun_ADXL345.h"
#include <arduinoFFT.h>
#define Int2 19
#define base 2
#define samplingFrequency 1600 /* Hertz*/
#define N 512 /*Stack overflow con N=1024*/

ADXL345 adxl=ADXL345();//  Para uso con I2C
arduinoFFT FFT;

int M[3][N],pila=0;
double vReal[N];
bool detener=false;
byte samples=base*14; 
int fifo=0;

void IRAM_ATTR ISR(){fifo=1;}

void setup() {
  Serial.begin(115200);
  adxl.powerOn();
  add_config();
  RSI_conf();
  presentacion();
  interrupts();
}

void loop(){
  if(fifo==1){lee_fifo();}
  if(detener){muestra();}
}
//********************************************************************************************************************************//
//**************************** Configuraciones y funciones ***********************************************************************//
//********************************************************************************************************************************//
void presentacion(){
  Serial.println("*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/");
  Serial.print("Modo de operación:");Serial.println(adxl.getMode());
  Serial.print("Tasa de muestreo (Hz):");Serial.println(adxl.getRate());
  Serial.println("*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/");
}
void RSI_conf(){
  pinMode(Int2,INPUT_PULLDOWN);
  attachInterrupt(Int2, ISR, RISING);
}
void add_config(void){

  adxl.setActivityXYZ(1, 1, 1);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityThreshold(0);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)
 
  adxl.setInactivityXYZ(1, 1, 1);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setInactivityThreshold(1);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  adxl.setTimeInactivity(10);         // How many seconds of no activity is inactive?

  adxl.setTapDetectionOnXYZ(1, 1, 1); // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)
 
  // Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
  adxl.setTapThreshold(1);           // 62.5 mg per increment
  adxl.setTapDuration(1);            // 625 μs per increment
  adxl.setDoubleTapLatency(1);       // 1.25 ms per increment
  adxl.setDoubleTapWindow(1);       // 1.25 ms per increment
 
  // Set values for what is considered FREE FALL (0-255)
  adxl.setFreeFallThreshold(1);       // (5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(1);       // (20 - 70) recommended - 5ms per increment

  
  //------------- Turn on Interrupts for each mode (1 == ON, 0 == OFF)-------------------------------------//
  adxl.InactivityINT(0);
  adxl.ActivityINT(0);
  adxl.FreeFallINT(0);
  adxl.doubleTapINT(0);
  adxl.singleTapINT(0);
  adxl.WaterMarkINT(1);
  adxl.DataReadyINT(0);

  //----------------FIFO Configuración--------------------------------------------------------------------//
  adxl.setMode(ADXL345_MODE_FIFO);//stream: no func.

  adxl.setImportantInterruptMapping(1,2,1,1,1,1,1);
  adxl.setInterruptMapping(ADXL345_INT_WATERMARK_BIT,ADXL345_INT2_PIN);

  adxl.setWatermark(samples);
  //------------------------------------------------------------------------------------------------------//
  adxl.set_bw(ADXL345_BW_800);
  adxl.setRangeSetting(0);//Rango
}

void lee_fifo(void){
  fifo=0;
  Serial.println("Adquiere...");
  adxl.readAccel(&M[0][pila], &M[1][pila], &M[2][pila],samples);//adxl.readAccel(&vector[pila]+.0,samples);
  pila=pila+base*14;
  if(pila>=N){
    detener=true;
  }
}

void muestra(void){
  Serial.println("Listo.");delay(1000);
  
  for (int i = 0; i < N; i++) {
      if(i<N){
        vReal[i]=static_cast<double>(M[0][i]);
      }
    }
  //FFT_Fbruta(&vector);
  FFT_libraries();
  while(1){ }
}
void FFT_Fbruta(void){
  Serial.println("FFT con algoritmo propio: ");
  int fft[N];
  unsigned long startTime = millis(); // Para microsegundos
  delay(10);
  unsigned long endTime = millis(); // Para microsegundos
  unsigned long totalTime = endTime - startTime; // Para microsegundos
  Serial.print("Duración del algoritmo de fuerza bruta (us): ");
  Serial.println(totalTime);
}

void FFT_libraries(void){
  Serial.println("FFT con librerías. Transformando el vector: ");
  double vImag[N];
  for (int i = 0; i < N; i++) {
    Serial.print(vReal[i]);Serial.print(", ");
  }
  Serial.println("");
  for (int i = 0; i < N; i++) {
    vImag[i] = 0.0;
  }
  
  unsigned long startTime = micros();
  
  /*No está claro, pero al parecer reescribe los vectores, derectamente, con el resultado de la FFT*/
  FFT = arduinoFFT(vReal, vImag, N, samplingFrequency); /* Create FFT object */
  FFT.Compute(FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude();
  
  unsigned long endTime = micros();
  unsigned long totalTime = endTime - startTime; // Corregir el nombre
  Serial.print("Duración del algoritmo de librería (us): ");
  Serial.println(totalTime);
  for (int i=0; i<N; i++) {
    Serial.print(vReal[i]); Serial.print(", ");
  }
}