#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

PCA9685 pwm = PCA9685();
//PCA9685 pwm = PCA9685(0x41);
//PCA9685 pwm = PCA9685(0x40, Wire);

#define fb_pin 12
#define pca_port 0

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(fb_pin,INPUT);

  Serial.begin(115200);
  pwm.begin();
  
  delay(5000);
  Serial.println("Start");
  delay(1000);
  calibracion(fb_pin,pca_port,1000,true,50);
  calibracion2(fb_pin,pca_port,100,true);
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

  double clk = 0.0;
  double clk_acu = 0.0;
  double clk_prom = 0.0;

  for(i=3;i < 235;i++){
    pwm.setPrescale(i);
    pwm.setPWM(feedback_port,0,2048);
    pwm.setPWM(15,0,2048);
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
      clk = 4096.0*frec_prom*(i+1);
      clk_acu += clk;
      n_samp++;

      if(print_samp){
        Serial.print("prescale = ");
        Serial.print(i);
        Serial.print(" / freq_prom = ");
        Serial.print(frec_prom,2);
        Serial.print(" hz / n_samples = ");
        Serial.print(samples_count);
        Serial.print(" / clk = ");
        Serial.println(clk);
      }
    }
  }

  clk_prom = round((clk_acu/n_samp)/1000)*1000;

  pwm.setOscillatorFrequency(clk);
  pwm.setPWMFreq(333);
  pwm.writeMicroseconds(feedback_port,1500);

  Serial.print("clk = ");
  Serial.print(clk_prom);
  Serial.println(" hz");
}

void calibracion2(uint8_t feedback_pin,uint8_t feedback_port, uint n_samp,bool print_samp){
  float pwm_freq = 333.3; 
  uint32_t target_duty_us = 1500;

  double clk = round(pwm.getOscillatorFrequency()/1000)*1000;
  pwm.setOscillatorFrequency(clk);
  pwm.setPWMFreq(pwm_freq);
  pwm.writeMicroseconds(feedback_port,target_duty_us);
  delay(500);

  uint32_t pass_time_ref = 0;
  uint32_t time_ref = 0;

  double duty_acu = 0;
  uint32_t duty_prom = 0;
  uint duty_count = 0;
  bool estado_bucle = true;
  bool estado = true;
  while (estado_bucle){
    if(digitalRead(feedback_pin) && !estado){
      //flanco de subida
      pass_time_ref = micros();
      estado = 1;
    }
    else if(!digitalRead(feedback_pin) && estado){
      //flanco de bajada
      time_ref = micros();
      if(pass_time_ref > 0){
        duty_acu += (double)(time_ref-pass_time_ref);
        duty_count++;

        if(duty_count >= n_samp){
          duty_prom = round(duty_acu/duty_count);
          duty_acu = 0;
          duty_count = 0;

          if(duty_prom == target_duty_us){
            estado_bucle = false;
          }
          else{
            if(print_samp){
              Serial.print("Clk = ");
              Serial.print(clk);
              Serial.print(" hz, duty = ");
              Serial.println(duty_prom);
            } 
            if(duty_prom > target_duty_us){
              clk -= 1000;
            }
            else{
              clk += 1000;
            }
            pwm.setOscillatorFrequency(clk);
            //pwm.setPWMFreq(pwm_freq);
            pwm.writeMicroseconds(feedback_port,target_duty_us);
            delay(50);
          }
        }
      }
      pass_time_ref = 0;
      estado = 0;
    }
  }

  Serial.print("Clk = ");
  Serial.print(clk);
}