// Basic demo for accelerometer readings from Adafruit LIS3DH

#ifndef PARTICLE
#include <Wire.h>
#include <SPI.h>
#endif
#include <Adafruit_LIS3DH.h>

#include "math.h"

// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
// Used for hardware & software SPI
#define LIS3DH_CS 10

#define G_EPSILON 0.08 //unit is m/s^2
#define G 0.98 //unit is m/s^2

struct vec3
{   
    float x;
    float y;
    float z;
    vec3() : x(0), y(0), z(0)
    { 
    }
    
    vec3(float a, float b, float c) : x(a), y(b), z(c)
    {
    }
    
    vec3& operator=(const vec3& a)
    {
        x = a.x;
        y = a.y;
        z = a.z;
    }
    
    vec3 operator+(const vec3& a)
    {
        return vec3(x+a.x, y+a.y, z+a.z);
    }
    
    vec3 operator-(const vec3& a)
    {
        return vec3(x-a.x, y-a.y, z-a.z);
    }
    
    vec3 operator*(const float a)
    {
        return vec3(x*a, y*a, z*a);
    }
    
    vec3 operator/(const float a)
    {
        return vec3(x/a, y/a, z/a);
    }
    
    float length()
    {
        return sqrt(x*x + y*y + z*z);
    }
    
};

vec3 operator*(const float a, vec3 v)
{
    return vec3(v.x*a, v.y*a, v.z*a);
}

float dot(const vec3& a, const vec3& b)
{
    return a.x*b.x + a.y*b.y + a.z*b.z;
}
    
vec3 cross(const vec3& a, const vec3& b)
{
    return vec3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}
    
vec3 normalize(vec3 a)
{
    float l = a.length();
    return a / l;
}

// software SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

void UpdateTimerHandler();
void ProcessTimerHandler();

Timer updateTimer(200, UpdateTimerHandler);
Timer processTimer(1000, ProcessTimerHandler);

vec3 down;
vec3 current;

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif

void setup(void) {
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  Serial.begin(9600);
  Serial.println("LIS3DH test!");

  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH found!");

  lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");
  
  updateTimer.start();
  processTimer.start();
}

bool IsG(vec3 a)
{
    float diff = a.length() - G;
    Serial.println(diff);
    float diffAbs = diff > 0 ? diff : -diff;
    Serial.print(diffAbs);
    Serial.print(" ? ");
    Serial.println(G_EPSILON);
    if(diffAbs < G_EPSILON)
        return true;
    else
        return false;
}

void loop() {
  
  

  //delay(200);
}

float GetDeltaAngerScale(vec3 a)
{
    vec3 norDown = normalize(down);
    float projDownScalar = dot(norDown, current);//we need this
    vec3 projDown = projDownScalar * norDown;
    vec3 projPlane = current - projDown;
    float projPlaneLength = projPlane.length();//we need this
    
    Serial.print("projDownScalar:"); Serial.println(projDownScalar, 6);
    Serial.print("projPlaneLength:"); Serial.println(projPlaneLength, 6);
}

void UpdateTimerHandler()
{
  lis.read();      // get X Y and Z data at once
  
  vec3 raw(lis.x, lis.y, lis.z);
  
  // Then print out the raw data
  /*Serial.print("X: "); Serial.print(raw.x);//lis.x);
  Serial.print(", Y: "); Serial.print(raw.y);//lis.y);
  Serial.print(", Z: "); Serial.print(raw.z);//lis.z);
  Serial.println(" raw data");*/
  /* Or....get a new sensor event, normalized */
  sensors_event_t event;
  lis.getEvent(&event);
  
  vec3 cooked(event.acceleration.x, event.acceleration.y, event.acceleration.z);

  /* Display the results (acceleration is measured in m/s^2) */
  /*Serial.print("X: "); Serial.print(cooked.x);//event.acceleration.x);
  Serial.print(", Y: "); Serial.print(cooked.y);//event.acceleration.y);
  Serial.print(", Z: "); Serial.print(cooked.z);//event.acceleration.z);
  Serial.println(" m/s^2");*/

  //Serial.println();
  
  current = cooked;
}

void ProcessTimerHandler()
{
    Serial.print("X: "); Serial.print(current.x);
    Serial.print(", Y: "); Serial.print(current.y);
    Serial.print(", Z: "); Serial.print(current.z);
    Serial.println(" m/s^2");
    
    if(IsG(current))
    {
        down = current;
        Serial.println("isG");
    }
    else
    {
        GetDeltaAngerScale(current);
    }
}
