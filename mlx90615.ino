#include "MLX90615.h"
#include <I2cMaster.h>
#include <SPI.h>
#include <Wire.h>
#include <MAX31865.h>
#include "Adafruit_MLX90614.h"
#include "MPU6050.h"

#define CHANNEL_ORDER_NEW
//#define FOOT6 
//#define USE_DUE
#define USE_MEGA2560
#define DTPOINT_TOTAL 120 //1300  //1780  //2400   //do not use algebra here
//#define SERIAL_OUTPUT

#ifdef FOOT6
  #define CHANNEL_TOTAL 13-1  //5 //4
  #define CHANNEL_MUTLI 12
  #define VSENSOR_PT100  15
  #define SENSOR_TOTAL 15  //increase to 15 on 20201117 //12 //8  
  #define COMMVERSION 0x13
#else
  #define CHANNEL_TOTAL 7-1  
  #define CHANNEL_MUTLI 6
  #define VSENSOR_PT100  9
  #define SENSOR_TOTAL 9   
  #define COMMVERSION 0x3
#endif

#define RTD_CS_PIN   A1 //22
#define READ_INTERVAL 30    //6s    //multilply RUN_DELAY_MILLI
#define RUN_DELAY_MILLI 200  //200ms //1000ms 
#define EXTRA_VIRTUAL_SENSOR 1
#define RREF 430

SoftI2cMaster* pi2c[CHANNEL_TOTAL];
MLX90615* pmlx90615[SENSOR_TOTAL-3];   //last 3 refer to multi channel which is accessed by mlx90615multi
Adafruit_MLX90614 mlx90615multi[3];
MAX31865_RTD rtd( MAX31865_RTD::RTD_PT100, RTD_CS_PIN, 430 );
short Data[DTPOINT_TOTAL][(SENSOR_TOTAL+EXTRA_VIRTUAL_SENSOR) * 2];
long Time[DTPOINT_TOTAL];  
//long iStartmilli;
//long iStartTm;
int ireadpointer = 0;
int iwritepointer = 0;
long iruntimes=0;
////////////////////////////////
int16_t accelCount[3];           // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;                // Stores the real accel value in g's
int16_t gyroCount[3];            // Stores the 16-bit signed gyro sensor output
float gx, gy, gz;                // Stores the real gyro value in degrees per seconds
float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
int16_t tempCount;               // Stores the internal chip temperature sensor output 
float temperature;               // Scaled temperature in degrees Celsius
float SelfTest[6];               // Gyro and accelerometer self-test sensor output
uint32_t count = 0;
float aRes, gRes; // scale resolutions per LSB for the sensors
MPU6050lib mpu;
int igyroMax;
int igyroVal[DTPOINT_TOTAL];

void setup()
{
  int i;
  Serial.begin(115200);
  Serial.println("Setup...");
  
  Serial2.begin(115200);

#ifdef CHANNEL_ORDER_NEW
  for (i = 0; i < 3; i++) {
    pi2c[i] = new SoftI2cMaster(2 * i + 6 + 3, 2 * i + 6 + 2);
  }
  for (i = 3; i < 6; i++) {
    pi2c[i] = new SoftI2cMaster(2 * i -6 + 3, 2 * i -6 + 2);
  }
#else
  for (i = 0; i < 6; i++) {
    pi2c[i] = new SoftI2cMaster(2 * i + 3, 2 * i + 2);
  }
#endif
#ifdef FOOT6
  for (i = 6; i < 9; i++) {
    //pi2c[i] = new SoftI2cMaster(4* i -1, 4 * i +1);  //from 23,25 to 31,33
    pi2c[i] = new SoftI2cMaster(4 * i + 12 +7, 4* i + 12 + 9);  //from 43,45 to 51,53
  }
  for (i = 9; i < 12; i++) {
    //pi2c[i] = new SoftI2cMaster(4 * i + 7, 4* i + 9);  //from 43,45 to 51,53
    pi2c[i] = new SoftI2cMaster(4* i -12 -1, 4 * i -12 +1);  //from 23,25 to 31,33
  }
  for (i = 0; i < 12; i++) {  //0-11
    //pmlx90615[i] = new MLX90615(i+1, pi2c[i]); 
    pmlx90615[i] = new MLX90615(0, pi2c[i]); 
  }
#else
  for (i = 0; i < 6; i++) {  //0-11
    //pmlx90615[i] = new MLX90615(i+1, pi2c[i]); 
    pmlx90615[i] = new MLX90615(0, pi2c[i]); 
  }  
#endif

#ifdef USE_DUE
  //pi2c[CHANNEL_MUTLI]  = new SoftI2cMaster(70, 71);
  Wire1.begin();
#else
  //pi2c[CHANNEL_MUTLI]  = new SoftI2cMaster(20, 21);
  Wire.begin();
#endif
  mlx90615multi[0].setAddr(CHANNEL_MUTLI+1);
  mlx90615multi[1].setAddr(CHANNEL_MUTLI+2);
  mlx90615multi[2].setAddr(CHANNEL_MUTLI+3);
  
  /*for (i = 0; i < 2; i++) {
    pmlx90615[2 * i] = new MLX90615(0x5b, pi2c[i]);
    pmlx90615[2 * i + 1] = new MLX90615(0x5b, pi2c[i]);
  }*/

  /*pmlx90615[CHANNEL_MUTLI] = new MLX90615(CHANNEL_MUTLI+1, pi2c[CHANNEL_MUTLI]);   
  pmlx90615[CHANNEL_MUTLI+1] = new MLX90615(CHANNEL_MUTLI+2, pi2c[CHANNEL_MUTLI]);    
  pmlx90615[CHANNEL_MUTLI+2] = new MLX90615(CHANNEL_MUTLI+3, pi2c[CHANNEL_MUTLI]);    */
  
  /* Initialize SPI communication. */
  SPI.begin( );
  SPI.setClockDivider( SPI_CLOCK_DIV64 );
  SPI.setDataMode( SPI_MODE3 );
    /* Allow the MAX31865 to warm up. */
  delay( 500 );

  /* Configure:
       V_BIAS enabled
       Auto-conversion
       1-shot disabled
       3-wire enabled
       Fault detection:  automatic delay
       Fault status:  auto-clear
       50 Hz filter
       Low threshold:  0x0000
       High threshold:  0x7fff
  */
  rtd.configure( true, true, false, false, MAX31865_FAULT_DETECTION_NONE,
                 true, true, 0x0000, 0x7fff );
                 
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RTD_CS_PIN, OUTPUT);

  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t c = mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
  Serial.print("I AM ");
  Serial.print(c, HEX);  
  Serial.print(" I Should Be ");
  Serial.println(0x68, HEX); 

  if (c == 0x68) // WHO_AM_I should always be 0x68
  {  
    Serial.println("MPU6050 is online...");
    
    mpu.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");

    if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
    Serial.println("Pass Selftest!");  
      
    mpu.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    mpu.initMPU6050(); Serial.println("MPU6050 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

   }
   else
   {
    Serial.print("Could not connect to MPU6050: 0x");
    Serial.println(c, HEX);
    //while(1) ; // Loop forever if communication doesn't happen
   }

  }
  
  delay(2000);
}

/****************************************************************
   * Function Name: crc8_msb
   * Description:  CRC8 check to compare PEC data
   * Parameters: poly - x8+x2+x1+1, data - array to check, array size
   * Return: 0 – data right; 1 – data Error
****************************************************************/
uint8_t crc8Msb(uint8_t poly, uint8_t* data, int size)  {
  uint8_t crc = 0x00;
  int bit;

  while (size--)
  {
    crc ^= *data++;
    for (bit = 0; bit < 8; bit++)
    {
      if (crc & 0x80) {
        crc = (crc << 1) ^ poly;
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

void loop()
{
  int i, ip, iend;
  short *pTo;
  short *pTa;
  byte crc;
  String comdata = "";
  unsigned char cinbuf1[4];
  unsigned char cinbuf2[16];
  unsigned char coutbuf[4];
  unsigned char coutbuf2[4 + (SENSOR_TOTAL+EXTRA_VIRTUAL_SENSOR) * 2 * 2 + 1];
  short *pData;
  double tPt100;
  //unsigned long timepassed;

    /*mpu.readAccelData(accelCount);  // Read the x/y/z adc values
    aRes=mpu.getAres();
    
    // Now we'll calculate the accleration value into actual g's
    /×、、ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - accelBias[1];   
    az = (float)accelCount[2]*aRes - accelBias[2];  */
   
    mpu.readGyroData(gyroCount);  // Read the x/y/z adc values
    gRes=mpu.getGres();
 
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes - gyroBias[1];  
    gz = (float)gyroCount[2]*gRes - gyroBias[2];   
    if (abs(gx)>igyroMax) igyroMax=abs(gx);
    if (abs(gy)>igyroMax) igyroMax=abs(gy);
    if (abs(gz)>igyroMax) igyroMax=abs(gz);

    tempCount = mpu.readTempData();  // Read the x/y/z adc values
    temperature = ((float) tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
    
      // Print acceleration values in milligs!
      //Serial.print("X-acceleration: "); Serial.print(1000*ax); Serial.print(" mg "); 
      //Serial.print("Y-acceleration: "); Serial.print(1000*ay); Serial.print(" mg "); 
      //Serial.print("Z-acceleration: "); Serial.print(1000*az); Serial.println(" mg"); 
#ifdef SERIAL_OUTPUT   
      // Print gyro values in degree/sec
      Serial.print("X-gyro rate: "); Serial.print(gx, 1); Serial.print(" degrees/sec "); 
      Serial.print("Y-gyro rate: "); Serial.print(gy, 1); Serial.print(" degrees/sec "); 
      Serial.print("Z-gyro rate: "); Serial.print(gz, 1); Serial.print(" degrees/sec "); 
      
     // Print temperature in degrees Centigrade      
      Serial.print("Temperature is ");  Serial.print(temperature, 2);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
      Serial.println("");
#endif

  rtd.read_all( );
  if (iruntimes%READ_INTERVAL==0) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    Time[iwritepointer] = (long)(millis() / 1000.0);
    if (igyroMax>255) igyroMax=255;
    igyroVal[iwritepointer]=igyroMax;
    igyroMax=0;
    pTo = Data[iwritepointer];
    pTa = (short*)((const char*)pTo + (SENSOR_TOTAL+EXTRA_VIRTUAL_SENSOR) * 2);
    for (i = 0; i < SENSOR_TOTAL-3; i++) {
      pTo[i] = pmlx90615[i]->getRawTemperature(MLX90615_OBJECT_TEMPERATURE);
      pTa[i] = pmlx90615[i]->getRawTemperature(MLX90615_AMBIENT_TEMPERATURE);
#ifdef SERIAL_OUTPUT
      Serial.print(pTo[i]);
      Serial.print("\t");
#endif
    }
    for (i = 0; i < 3; i++) {
      pTo[SENSOR_TOTAL-3+i] = mlx90615multi[i].readRawTemp(MLX90615_OBJECT_TEMPERATURE);
      pTa[SENSOR_TOTAL-3+i] = mlx90615multi[i].readRawTemp(MLX90615_AMBIENT_TEMPERATURE);
      //Serial.print(pTo[SENSOR_TOTAL-3+i]);
      //Serial.print("\t");
    }
    //Serial.print("\n");
    pTo[VSENSOR_PT100]=0;
    pTa[VSENSOR_PT100]=0;

    if( rtd.status( ) == 0 )
    {
      tPt100 = rtd.temperature( );
      pTo[VSENSOR_PT100]=(short)((tPt100+273.15)*50);  //as if it's a 90615
    }
    else 
    {
#ifdef SERIAL_OUTPUT
      Serial.print( "RTD fault register: " );
      Serial.print( rtd.status( ) );
      Serial.print( ": " );
      if( rtd.status( ) & MAX31865_FAULT_HIGH_THRESHOLD )
      {
        Serial.println( "RTD high threshold exceeded" );
      }
      else if( rtd.status( ) & MAX31865_FAULT_LOW_THRESHOLD )
      {
        Serial.println( "RTD low threshold exceeded" );
      }
      else if( rtd.status( ) & MAX31865_FAULT_REFIN )
      {
        Serial.println( "REFIN- > 0.85 x V_BIAS" );
      }
      else if( rtd.status( ) & MAX31865_FAULT_REFIN_FORCE )
      {
        Serial.println( "REFIN- < 0.85 x V_BIAS, FORCE- open" );
      }
      else if( rtd.status( ) & MAX31865_FAULT_RTDIN_FORCE )
      {
        Serial.println( "RTDIN- < 0.85 x V_BIAS, FORCE- open" );
      }
      else if( rtd.status( ) & MAX31865_FAULT_VOLTAGE )
      {
        Serial.println( "Overvoltage/undervoltage fault");
      }
      else
      {
        Serial.println( "Unknown fault; check connection" );
      }
#endif
    }
  
    iwritepointer++;
    if (iwritepointer >= DTPOINT_TOTAL) iwritepointer = 0;
    if (iwritepointer==ireadpointer) ireadpointer=((iwritepointer+1)%DTPOINT_TOTAL);  //must be +1, otherwise ireadpointer==iwritepointer, only one piece when be send when requested
#ifdef SERIAL_OUTPUT
    Serial.print("write to buffer, writepointer=");
    Serial.print(iwritepointer);
    Serial.print("  fTa=");
    Serial.println(pTa[0]);
    Serial.print("  Tpt100=");
    Serial.println(tPt100);
#endif
  }
  
  cinbuf1[0] = 0;
  cinbuf1[1] = 0;
  cinbuf1[2] = 0;
  cinbuf1[3] = 0;
  if (Serial2.available() > 0)
  {
    //comdata += char(Serial2.read());
    Serial2.readBytes(cinbuf1, 4);
    if (cinbuf1[3] > 0 && cinbuf1[3] < 16) {
      Serial2.readBytes(cinbuf2, cinbuf1[3]);
    }
  }
  //Serial2.println(cinbuf);
  if (cinbuf1[0] != 0xaa || cinbuf1[1] != 0xaa) { //realign
    while (Serial2.available() > 0) {
      Serial2.read();
      delay(2);
    }
  }
  if (cinbuf1[0] == 0xaa && cinbuf1[1] == 0xaa)
  {
    Serial.println("request received...");
    if (cinbuf1[2] == 1) {
      /*if (iStartTm == 0) {
        memcpy(&iStartTm, cinbuf2, 4);
        iStartmilli = millis();
      }*/
      unsigned long cursec;

      cursec=(millis() / 1000.0);
      coutbuf[0] = 0xaa;
      coutbuf[1] = 0xaa;
      coutbuf[2] = 0x01;
      coutbuf[3] = 5;
      crc = crc8Msb(7,(unsigned byte*)&cursec, 4);
      Serial2.write((const byte*)coutbuf, 4);
      Serial2.write((const byte*)&cursec, 4);
      Serial2.write(&crc, 1);
      Serial.println("millis send.");
    }
    if (cinbuf1[2] == 2) {
      coutbuf[0] = 0xaa;
      coutbuf[1] = 0xaa;
      coutbuf[2] = COMMVERSION;   //new version number, first version 0x02, second version 0x12
      coutbuf[3] = (SENSOR_TOTAL+EXTRA_VIRTUAL_SENSOR) * 2 * 2 + 4 + 1 + 1; //add time and crc and movespeed
      if (ireadpointer <= iwritepointer) {      //must be <= otherwise, since when mobile phone read multi times, ireadpointer==iwritepointer
        iend = iwritepointer;
      } else {
        iend = iwritepointer + DTPOINT_TOTAL;
      }
      if (iend>ireadpointer+200) {
        iend=ireadpointer+200;
      }
      for (i = ireadpointer; i < iend; i++) {
        ip = i;
        if (i >= DTPOINT_TOTAL) ip = i % DTPOINT_TOTAL;
        pData = Data[ip];
        
        memcpy(coutbuf2, &(Time[ip]), sizeof(long));
        memcpy(coutbuf2+sizeof(long),pData,(SENSOR_TOTAL+EXTRA_VIRTUAL_SENSOR)*2*2);
        //memcpy(coutbuf2 + 2, pData, SENSOR_TOTAL * 2);
        //memcpy(coutbuf2 + 2 + SENSOR_TOTAL * 2, &(Time[0]), DTPOINT_TOTAL * 2);
        coutbuf2[sizeof(long)+(SENSOR_TOTAL+EXTRA_VIRTUAL_SENSOR)*2*2]=igyroVal[ip];
        
        crc = crc8Msb(7,(unsigned byte*)coutbuf2, sizeof(long) + (SENSOR_TOTAL+EXTRA_VIRTUAL_SENSOR) * 2 * 2 + 1);
        Serial2.write((const byte*)coutbuf, 4);
        Serial2.write((const byte*)coutbuf2, sizeof(long) + (SENSOR_TOTAL+EXTRA_VIRTUAL_SENSOR) * 2 * 2 + 1);  
        Serial2.write(&crc, 1);
        Serial2.flush();
        Serial.print("data send. curpointer=");
        Serial.println(ip);
      }
      //ireadpointer = (iend + DTPOINT_TOTAL) % DTPOINT_TOTAL; //iend may be 0, so add DTPOINT_TOTAL first  //do not sub 1 otherwise will repeat one piece of data
      ireadpointer = iend % DTPOINT_TOTAL; 
    }
  }
  iruntimes++;
  digitalWrite(LED_BUILTIN, LOW); 
  delay(RUN_DELAY_MILLI);
}

//20200610 一致起来，第一个通道不再使用5b而用1
//20200804 修改了通道号，从1开始到6，然后7-8，10-12分别为两个三探头
//20201105 增加了读31865的代码，通道总数变为16个，PT100在第16个
//20201116 修改了规约号为13，通道顺序也修改正确了（同时修改了app通道顺序）
//20201117 修改探头总数为15，这样如果1-6号探头I/O口损坏可以在7-12替代，三探头挪到13-15，增加了PT100开始置0
//20210105 上月已经增加了一个max31865
//20210109 将RTD的片选从22改到A1（贴片的是A0）
//20210118 更改IIC分组代码，SCL SDA地址和新电路板对应，除了13-15，其他探头都用0来访问
//20210421 将几个关键地方的int改成Long，这样2560,due的代码在这个细节就一样了
//20210430 修改了多个通道对应的sda scl端口，这样才能和新电路板一致
//20210525 将13-15三探头改为用wire（或wire1）访问，增加了读mpu6050的代码而且在数据末尾加了一字节，增加了有/无脚六经的处理，将loop改为0.2秒以防漏掉移动
//                 规约号3代表没有脚六经 13则有，增加移动数据那个字节没有单独区分规约号
