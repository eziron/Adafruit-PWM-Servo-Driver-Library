#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

PCA9685 pwm = PCA9685();
//PCA9685 pwm = PCA9685(0x41);
//PCA9685 pwm = PCA9685(0x40, Wire);

#define feedback_pin 12
#define pca_port 0

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  pwm.begin();
  
  delay(5000);
  Serial.println("Start");
  delay(1000);
  calibracion(feedback_pin,pca_port,1000,true,50);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   
  delay(1000);                      
  digitalWrite(LED_BUILTIN, LOW); 
  delay(1000);   
}


void calibracion(uint8_t feedback_pin,uint8_t feedback_port,uint32_t sample_time_ms,bool print_samp, uint16_t min_samples){
  double samples[255][2];
  uint8_t n_samp = 0;

  double frec_acu = 0.0;
  double frec_prom = 0.0;
  int samples_count = 0;
  bool estado = false;

  uint32_t sample_time_ref = 0;
  uint32_t time_ref = 0;
  uint32_t pass_time_ref = 0;

  uint8_t i = 0;
  uint8_t j = 0;

  pinMode(feedback_pin,INPUT);
  for(i=3;i < 255;i++){
    pwm.setPrescale(i);
    pwm.setPWM(feedback_port,0,2048);
    delay(100);

    frec_acu = 0.0;
    frec_prom = 0.0;
    samples_count = 0;
    estado = 0;
    time_ref = 0;
    pass_time_ref = 0;

    sample_time_ref = millis();
    while(millis()-sample_time_ref < sample_time_ms || samples_count < min_samples){
      if(digitalRead(feedback_pin) && !estado){
        time_ref = micros();
        if(pass_time_ref > 0){
          frec_acu += 1000000.0/(double)(time_ref-pass_time_ref);
          samples_count++;
        }
        pass_time_ref = time_ref;
        estado = 1;
      }
      else if(!digitalRead(feedback_pin) && estado){
        estado = 0;
      }
    }

    if(samples_count > 0){
      frec_prom = frec_acu/samples_count;
      if(n_samp > 0){
        if(samples[n_samp-1][0]>frec_prom){
          samples[n_samp][0] = frec_prom;
          samples[n_samp][1] = (double)i;
          n_samp++;
        }
      }
      else{
        samples[n_samp][0] = frec_prom;
        samples[n_samp][1] = (double)i;
        n_samp++;
      }
      
      if(print_samp){
        Serial.print("prescale = ");
        Serial.print(i);
        Serial.print(" / freq_prom = ");
        Serial.print(frec_prom,2);
        Serial.print(" hz / n_samples = ");
        Serial.println(samples_count);
      }
    }
  }

  samples_count = 0;

  double A = 0.0;
  double A_prom = 0.0;
  double clk = 0.0;
  double clk_0 = 0.0;
  double clk_1 = 0.0;
  double clk_prom = 0.0;

  for(i = 0; i<n_samp-1;i++){
    for (j = i+1; j < n_samp; j++){
      A = ((samples[i][0]*samples[i][1])-(samples[j][0]*samples[j][1]))/(samples[j][0]-samples[i][0]);
      clk_0 = 4096.0*(samples[i][1]+A)*samples[i][0];
      clk_1 = 4096.0*(samples[j][1]+A)*samples[j][0];

      if(clk_0 == clk_1){
        clk = clk_0;
      }
      else{
        clk = (clk_0+clk_1)/2.0;
      }

      if(A >= 0.0 and A <= 2.0){
        samples_count++;
        A_prom += A;
        clk_prom += clk;
      }
    }
  }

  A = A_prom/samples_count;
  clk = clk_prom/samples_count;

  pwm.setOscillatorFrequency(clk,A);
  pwm.setPWMFreq(50);
  pwm.setPWM(feedback_port,0,0);

  Serial.print("clk = ");
  Serial.print(clk,10);
  Serial.print(" hz  /  A = ");
  Serial.print(A,10);
}