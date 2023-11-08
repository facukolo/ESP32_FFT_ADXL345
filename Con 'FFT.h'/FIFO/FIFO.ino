#include "SparkFun_ADXL345.h"

#include "FFT.h" // include the library
#define FFT_N 256 // Must be a power of 2
float fft_input[FFT_N];
float fft_output[FFT_N];

#define Int2 19
#define base 2
#define samplingFrequency 1600 /* Hertz*/


ADXL345 adxl=ADXL345();//  Para uso con I2C

int M[3][FFT_N],pila=0;
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
  //Serial.println("Adquiere...");
  adxl.readAccel(&M[0][pila], &M[1][pila], &M[2][pila],samples);//adxl.readAccel(&vector[pila]+.0,samples);
  pila=pila+base*14;
  if(pila>=FFT_N){
    detener=true;
  }
}

void muestra(void){
  //Serial.println("Listo.");
  delay(1000);
  
  for (int i = 0; i < FFT_N; i++) {
      if(i<FFT_N){
        Serial.print(static_cast<double>(M[0][i]));Serial.print(", ");
      }
    }
  Serial.println("");
  
  FFT_libraries();
  while(1){ }
}


void FFT_libraries(void){
  char print_buf[300];
  fft_config_t *real_fft_plan = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fft_input, fft_output);
  Serial.println("FFT con librerías. Transformando el vector: ");

  for (int k = 0 ; k < FFT_N ; k++){
    real_fft_plan->input[k] = (float)(M[0][k]);
  }
  
  long int t1 = micros();
  fft_execute(real_fft_plan);/*Execute transformation*/
  long int t2 = micros();

  Serial.print("N: ");Serial.print(FFT_N);
  Serial.print(" - Time taken: ");Serial.print((t2-t1)*1.0/1000);Serial.println(" milliseconds!");

  for (int k = 1 ; k < real_fft_plan->size / 2 ; k++) /*Print the output*/
    {
    sprintf(print_buf,"%f, ",sqrt(pow(real_fft_plan->output[2*k],2) + pow(real_fft_plan->output[2*k+1],2))/1);
    Serial.print(print_buf);
   }

  fft_destroy(real_fft_plan);
}