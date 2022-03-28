# Mod Adafruit PCA9685 PWM Servo Driver Library

Esta es una librería modificada para controlar servos con el chip PCA9685

La principal diferencia con la librería original es que incorpora funciones para actualizar los valores de los 16 canales del PCA9685 a la vez, estas funciones tienen un rendimiento mayor al de actualizar todos los canales de forma individual, ya que envía todos los valores en solo 1 transmisión de datos, de forma que el I2C no tiene que estar iniciando y terminando la transmisión por cada canal que se actualice

Tambien cambie el nombre de la clase a PCA9685

```C++
PCA9685();
PCA9685(const uint8_t addr);
PCA9685(const uint8_t addr,TwoWire &i2c);
```

### funciones principales

```C++
void begin(uint16_t PWM_freq, double clk_freq);
//PWM_freq: Es la frecuencia del PWM en Hz, para servos se usa de 50hz a 333hz
//clk_freq: Es la frecuencia del reloj en Hz, por defecto es 25Mhz

=======================================================

//fija el largo de pulso de un servo en particular
void writeMicroseconds(uint8_t num, uint16_t Microseconds);
//num: es el número del canal en el chip PCA, va de 0 a 15
//Microseconds: es el largo de pulso en us

=======================================================

//fija el largo de pulso en us de todos los servos a la vez
void writeAllMicroseconds(uint16_t Microseconds_array[16]);
//Microseconds_array: es un array que lleva los valores de largo de pulso en us para todos los servos
//Microseconds_array = {CH0,CH1,CH2....,CH14,CH15}

=======================================================

//fija el ángulo en grados de un servo particular
void write(uint8_t num,float angle);
//num: es el número del canal en el chip PCA, va de 0 a 15
//angle: ángulo del servo, va desde 0 a 180

=======================================================

//fija los ángulos para todos los canales del PCA a la vez
void writeALL(float angle_array[16]);
//angle_array: es un array que lleva los valores de ángulo en grados para todos los servos
//angle_array = {CH0,CH1,CH2....,CH14,CH15}

=======================================================

//configura los largos de pulso para su conversión desde un ángulo para un servo en particular
void setconf(uint8_t num, uint16_t min_duty_us,uint16_t max_duty_us);
//num: es el número del canal en el chip PCA, va de 0 a 15
//min_duty_us: largo de pulso para que el servo está en 0 grados, por defecto es 500us
//max_duty_us: largo de pulso para que el servo está en 180 grados, por defecto es 2500us

=======================================================

//configura los largos de pulso para su conversión desde un ángulo para todos los servos por igual
void setAllconf(uint16_t min_duty_us,uint16_t max_duty_us);
//min_duty_us: largo de pulso para que el servo está en 0 grados, por defecto es 500us
//max_duty_us: largo de pulso para que el servo está en 180 grados, por defecto es 2500us
```
### funciones extra
```C++
    void reset(); 
    void sleep();
    void wakeup();
    void enableExtClk(); //habilita la entrada de reloj externa
    void setPWMFreq(double freq);
    void setOutputMode(bool totempole);
    void setPin(uint8_t num, uint16_t val, bool invert);
    void setPrescale(uint8_t prescale);
    uint8_t readPrescale(void);
    double getOscillatorFrequency(void);
    void setOscillatorFrequency(double freq);
    uint8_t getPWM(uint8_t num);
    void setPWM(uint8_t num, uint16_t on, uint16_t off);
    void setAllPWM(uint16_t PWM_array[16][2]);
```

