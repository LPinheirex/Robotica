#include <Wire.h>
#include <VL53L0X.h>
//Pololu
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include "math.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//#include <ESP8266WiFi.h>

int enMotor = 4;
int dir1Motor = 16;
int dir2Motor = 17;
int enLidar = 19;
int dir1Lidar = 18;
int dir2Lidar = 13;
int rstSensor = 23;
int servoEsqPin = 27;
int servoDirPin = 26;
int enc1Motor = 25;
int enc2Motor = 33;
int enc1Lidar = 32;
int enc2Lidar = 35;
int gpioLidar = 34;
int volta = 39;
int led = 14;

Adafruit_MPU6050 mpu;

unsigned long tempo;
unsigned long tempoVel;
Servo servoDir;  // create servo object to control a servo
Servo servoEsq;  // create servo object to control a servo
double pos = 0;  // variable to store the servo position
int posEsq;
int posDir;
int potMotor;
float vel;
float accErro;
int countLidar;
float pulsos_por_volta;
int amostragem = 100;
int mode = 1;
bool fLidar;
bool fVolta;
unsigned long aux;
unsigned long auxTempo;
int fdir;
int distOut[20];
float accVel[5];
float velBase;
float velMeta;
bool fVel = false;
int pot;
bool fMed = true;

unsigned long t1Motor;
unsigned long t2Motor;
unsigned long tempoLidar;
bool fMotor;
unsigned long tempoVolta;

double x = 128.9;
double y = 162.967;
double pi = 3.141592;

int readCount;
int points = 16;

// IP address to send UDP data to.
// it can be ip address of the server or
// a network broadcast address
// here is broadcast address
const char *udpAddress = "192.168.4.23";  // your pc ip
int udpPort = 8080;                       //port server

//char leitura[50];

IPAddress local_IP(192, 168, 4, 22);
IPAddress gateway(192, 168, 4, 9);
IPAddress subnet(255, 255, 255, 0);

//create UDP instance
WiFiUDP udp;

// Cria uma instancia do sensor
VL53L0X sensor;

char buffer[100];

void calculaAngulo(double a) {
  if (a >= 0) {
    a = a * (pi / 180.0);
    double d = tan((pi / 2.0) - a) * y;
    posEsq = (int)((((pi / 2.0) - atan((d - (x / 2.0)) / y)) * (180.0 / pi)) * (53.0 / 33.0) + 93);
    posDir = (int)((((pi / 2.0) - atan((d + (x / 2.0)) / y)) * (180.0 / pi)) * (53.0 / 33.0) + 92);
  } else {
    a = a * -(pi / 180.0);
    double d = tan((pi / 2.0) - a) * y;
    posDir = (int)(-(((pi / 2.0) - atan((d - (x / 2.0)) / y)) * (180.0 / pi)) * (53.0 / 33.0) + 92);
    posEsq = (int)(-(((pi / 2.0) - atan((d + (x / 2.0)) / y)) * (180.0 / pi)) * (53.0 / 33.0) + 93);
  }
}

void IRAM_ATTR pulseMotor() {
  t2Motor = t1Motor;
  t1Motor = micros();
  if (digitalRead(enc2Motor) == LOW) {
    if (fdir < 2) {
      fdir++;
    }
  } else {
    if (fdir > -1) {
      fdir--;
    }
  }
  //  fdir = digitalRead(enc2Motor);
  fMotor = true;
}
void IRAM_ATTR pulseLidar() {
  fLidar = true;
  countLidar++;
}
void IRAM_ATTR pulseVolta() {
  fVolta = true;
}


void setup() {
  // Inicializa a comunicação serial
  Serial.begin(115200);

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoDir.setPeriodHertz(50);              // standard 50 hz servo
  servoDir.attach(servoDirPin, 500, 2500);  // attaches the servo on pin 18 to the servo object
  servoEsq.setPeriodHertz(50);              // standard 50 hz servo
  servoEsq.attach(servoEsqPin, 500, 2500);  // attaches the servo on pin 18 to the servo object
  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep

  servoDir.write(92);
  servoEsq.write(93);

  pinMode(enMotor, OUTPUT);
  pinMode(dir1Motor, OUTPUT);
  pinMode(dir2Motor, OUTPUT);
  pinMode(enLidar, OUTPUT);
  pinMode(dir1Lidar, OUTPUT);
  pinMode(dir2Lidar, OUTPUT);
  pinMode(rstSensor, OUTPUT);
  //  pinMode(servoEsqPin, OUTPUT);
  //  pinMode(servoDirPin, OUTPUT);
  pinMode(enc1Motor, INPUT);
  pinMode(enc2Motor, INPUT);
  pinMode(enc1Lidar, INPUT);
  pinMode(enc2Lidar, INPUT);
  pinMode(gpioLidar, INPUT);
  pinMode(volta, INPUT);
  pinMode(led, OUTPUT);

  digitalWrite(rstSensor, HIGH);



  /*
          digitalWrite(dir2Lidar, LOW);
        digitalWrite(dir1Lidar, HIGH);
        analogWrite(enLidar, 100);
*/

/*  fVel = false;
  potMotor = -150;

  analogWrite(enMotor, -potMotor);
  digitalWrite(dir2Motor, HIGH);
  digitalWrite(dir1Motor, LOW);
*/

  if (!mpu.begin()) {
    //   Serial.println("Failed to find MPU6050 chip");
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  /*  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
*/

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  /*  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }
*/
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  /*  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
*/

  //  Serial.print("Setting soft-AP configuration ... ");
  //  if (WiFi.softAPConfig(local_IP, gateway, subnet, (uint32_t)3, (uint32_t)0)) {
  if (WiFi.softAPConfig(local_IP, gateway, subnet)) {
    //    Serial.println("Ready");
  } else {
    //    Serial.println("Erro");
  };

  //  Serial.print("Setting soft-AP ... ");
  WiFi.softAP("UDP_Comm", 0, 1, 0, 1, false) ? "Ready" : "Failed!";
  (IPAddress) WiFi.softAPIP();

  udp.begin(udpPort);

  // Inicializa a comunicação I2C
  Wire.begin();

  // Inicializa o sensor
  sensor.init();
  // Define um timeout de 500mS para a leitura do sensor
  // Em caso de erro, este será o tempo máximo de espera da resposta do sensor
  sensor.setTimeout(500);
  sensor.setMeasurementTimingBudget(20000);

  // sensor.setSignalRateLimit(0.1);
  // sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  // sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  attachInterrupt(enc1Motor, pulseMotor, RISING);
  attachInterrupt(enc2Lidar, pulseLidar, RISING);
  attachInterrupt(volta, pulseVolta, RISING);

  tempo = millis();
  tempoVel = millis();
}

void loop() {

  memset(buffer, 0, 50);
  //processing incoming packet, must be called before reading the buffer
  if (udp.parsePacket() > 0) {
    //receive response from server, it will be HELLO WORLD
    if (udp.read(buffer, 50) > 0) {
      int i = 0;
      char comando[20];
      while (!isdigit(buffer[i]) && buffer[i] != '-') {
        comando[i] = buffer[i];
        i++;
      }
      comando[i] = '\0';  // Terminamos a string do comando
      float valor = atof(buffer + i);
      //      Serial.println(comando);
      //    Serial.println(valor);
      //  Serial.println();

      if (strcmp(comando, "go") == 0) {
        digitalWrite(dir2Lidar, LOW);
        digitalWrite(dir1Lidar, HIGH);
        analogWrite(enLidar, (int)valor);
        digitalWrite(led, HIGH);
        //        digitalWrite(enLidar, HIGH);
      } else if (strcmp(comando, "Parar") == 0) {
        accErro = 0;
        fVel = false;
        digitalWrite(led, LOW);
        analogWrite(enMotor, 0);
        potMotor = 0;
      } else if (strcmp(comando, "Points") == 0) {
        points = (int)valor;
      } else if (strcmp(comando, "Amostragem") == 0) {
        amostragem = (int)valor;
      } else if (strcmp(comando, "Mode") == 0) {
        mode = (int)valor;
      } else if (strcmp(comando, "Deg") == 0) {
        accErro = 0;
        accVel[0] = 0;
        accVel[1] = 0;
        accVel[2] = 0;
        accVel[3] = 0;
        accVel[4] = 0;
        vel = 0;
        auxTempo = millis();
        pot = (int)valor;
        digitalWrite(dir2Motor, LOW);
        digitalWrite(dir1Motor, HIGH);
        analogWrite(enMotor, pot);
      } else if (strcmp(comando, "Vel") == 0) {
        accErro = 0;
        auxTempo = millis();
        fVel = true;
        digitalWrite(led, HIGH);
        //        pot = (int)(valor / 3);
        velBase = valor / 1000.0;
        /*        if (valor > 0) {
         digitalWrite(dir2Motor, LOW);
         digitalWrite(dir1Motor, HIGH);
         analogWrite(enMotor, pot);
         } else {
         pot = -pot;
         digitalWrite(dir2Motor, HIGH);
         digitalWrite(dir1Motor, LOW);
         analogWrite(enMotor, pot);
         }
         */
      } else if (strcmp(comando, "Acelerar") == 0) {
        accErro = 0;
        fVel = false;
        potMotor += 56;
        if (potMotor > 0 && potMotor < 112) {
          potMotor = 112;
        }
        if (potMotor > 255) {
          potMotor = 255;
        }
        if (potMotor > 0) {
          analogWrite(enMotor, potMotor);
          digitalWrite(dir2Motor, LOW);
          digitalWrite(dir1Motor, HIGH);
        } else {
          analogWrite(enMotor, -potMotor);
          digitalWrite(dir2Motor, HIGH);
          digitalWrite(dir1Motor, LOW);
        }
      } else if (strcmp(comando, "Re") == 0) {
        accErro = 0;
        potMotor -= 56;
        if (potMotor < 0 && potMotor > -112) {
          potMotor = -112;
        }
        if (potMotor < -255) {
          potMotor = -255;
        }
        if (potMotor > 0) {
          analogWrite(enMotor, potMotor);
          digitalWrite(dir2Motor, LOW);
          digitalWrite(dir1Motor, HIGH);
        } else {
          analogWrite(enMotor, -potMotor);
          digitalWrite(dir2Motor, HIGH);
          digitalWrite(dir1Motor, LOW);
        }
      } else if (strcmp(comando, "Dir") == 0) {
        pos = valor;
        if (pos < -30) {
          pos = -30;
        }
        if (pos > 30) {
          pos = 30;
        }
        calculaAngulo(pos);
        servoDir.write(posDir);
        servoEsq.write(posEsq);
      } else if (strcmp(comando, "Direita") == 0) {
        pos -= 6;
        if (pos < -30) {
          pos = -30;
        }
        calculaAngulo(pos);
        servoDir.write(posDir);
        servoEsq.write(posEsq);
      } else if (strcmp(comando, "Esquerda") == 0) {
        pos += 6;
        if (pos > 30) {
          pos = 30;
        }
        calculaAngulo(pos);
        servoDir.write(posDir);
        servoEsq.write(posEsq);
      } else if (strcmp(comando, "off") == 0) {
        analogWrite(enLidar, 0);
        //        digitalWrite(enLidar, LOW);
      }
    }
  }

  //Degrau do encoder do Motor
  if (fMotor == true) {
    fMotor = false;

    for (int n = 4; n > 0; n--) {
      accVel[n] = accVel[n - 1];
    }
    if (fdir > 0) {
      accVel[0] = ((pi * 0.063) / (11.0 * 34.02)) / ((float)(t1Motor - t2Motor) / 1000000.0);
    } else {
      accVel[0] = -((pi * 0.063) / (11.0 * 34.02)) / ((float)(t1Motor - t2Motor) / 1000000.0);
    }
    //    vel = (accVel[0] + accVel[1] + accVel[2] + accVel[3] + accVel[4]) / 5.0;
    vel = accVel[0];
  }

  //Volta
  if (fVolta == true) {
    fVolta = false;
    //    Serial.println(countLidar);

    if (countLidar > 500) {
      pulsos_por_volta = countLidar;
      /*      unsigned long tAux = millis() - tempoVolta;
      tempoVolta = millis();
      memset(buffer, 0, 100);
      sprintf(buffer, "%ld", tAux);
      udp.beginPacket(udpAddress, udpPort);
      udp.print(buffer);
      udp.endPacket();*/
      if (mode == 1) {
        memset(buffer, 0, 100);
        sprintf(buffer, "%lu_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d",
                micros(), points, distOut[0], distOut[1], distOut[2], distOut[3], distOut[4], distOut[5], distOut[6], distOut[7], distOut[8], distOut[9], distOut[10], distOut[11], distOut[12], distOut[13], distOut[14], distOut[15]);
        udp.beginPacket(udpAddress, udpPort);
        udp.print(buffer);
        udp.endPacket();
        //  Serial.println(buffer);
        for (int p = 0; p < 16; p++) {
          distOut[p] = 0;
        }
      }

      //      aux = countLidar;
      countLidar = 0;
    }
    readCount = 0;
  }

  //Degrau do encoder do LiDAR
  if (fLidar == true) {
    fLidar = false;
    if (mode == 1) {
      if (countLidar >= (561.0 * readCount) / points && fMed == true) {
        fMed = false;
        sensor.startContinuous(23);
        tempoLidar = millis();
      }
    }
  }

  if (millis() - tempoLidar > 25 && fMed == false) {
    distOut[readCount] = sensor.readRangeContinuousMillimeters();
    if (millis() - tempoLidar > 29) {
      distOut[readCount] = 0;
    }
    readCount++;
    sensor.stopContinuous();
    fMed = true;
  }

  if (millis() - tempoVel > 50) {
    tempoVel = millis();
    if (fVel == true) {
      if (pos > 0) {
        double a = pos * (pi / 180.0);
        double d = tan((pi / 2.0) - a) * y;
        velMeta = velBase * ((d + (x / 2)) / d);
      } else if (pos < 0) {
        double a = -pos * (pi / 180.0);
        double d = tan((pi / 2.0) - a) * y;
        velMeta = velBase * ((d - (x / 2)) / d);
      } else {
        velMeta = velBase;
      }

      if (pot < 255 || velMeta - vel < 0) {
        accErro = (velMeta - vel) * 1000 + accErro;
      }
      //      pot = (int)((velMeta - vel) * 68 + ((accErro * 0.068) / 1.5));
      pot = (int)((velMeta - vel) * 1100 + (accErro * 0.22));
      if (pot > 0) {
        pot = constrain(pot, 0, 255);
        digitalWrite(dir2Motor, LOW);
        digitalWrite(dir1Motor, HIGH);
        analogWrite(enMotor, pot);
      } else {
        int auxPot = -pot;
        auxPot = constrain(auxPot, 0, 255);
        digitalWrite(dir1Motor, LOW);
        digitalWrite(dir2Motor, HIGH);
        analogWrite(enMotor, auxPot);
      }
    }
    //    Serial.println(vel, 3);
  }


  if (millis() - tempo > amostragem) {
    tempo = millis();
    if (mode == 1) {

      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      /*Serial.print("Acceleration X: ");
  Serial.print(-a.acceleration.y);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.y);
  Serial.print(", Y: ");
  Serial.print(-g.gyro.x);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
*/

      // Faz a medição da distância e retorna um valor em milímetros
      //    float angulo = countLidar * (360 / (1.5 * 34.02 * 11));
      //    float angulo = countLidar * (360 / (1.5 * 33.57 * 11));
      //    aux = millis() - auxTempo;
      //    int dist = sensor.readRangeContinuousMillimeters();
      //    dist /= 10;
      float xAcc = -a.acceleration.x;
      float yAcc = -a.acceleration.y;
      float zAcc = a.acceleration.z;
      float xRot = -g.gyro.x;
      float yRot = -g.gyro.y;
      float zRot = g.gyro.z;
      memset(buffer, 0, 100);
      //    sprintf(buffer, "%.3f_%ld_%ld_%ld_%.3f_%.1f", vel, dist, aux, pot, velMeta, accErro);
      sprintf(buffer, "%lu_%.3f_%.1Lf_%.3f_%.3f_%.3f_%.3f_%3.f_%.3f", micros(), vel, pos, xAcc, yAcc, zAcc, xRot, yRot, zRot);
      udp.beginPacket(udpAddress, udpPort);
      udp.print(buffer);
      udp.endPacket();
//      Serial.println(buffer);
      vel = 0.0;
      /*        Serial.print(vel*1000);
       Serial.print("\t");
       Serial.println(pot);*/
    }
  }
}

/*
Em C:\Users\lucas\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.11\libraries\WiFi\src
Modificar WiFiGeneric.cpp 
        dhcp_ipaddr = dhcp_ipaddr == 0 ? ap_ipaddr + 3 : dhcp_ipaddr;//25
        lease.end_ip.addr = lease.start_ip.addr + 0;//apenas 1 ip
*/