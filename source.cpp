#include "Particle.h"

#ifndef PARTICLE
#include <Wire.h>
#include <SPI.h>
#endif
#include <Adafruit_LIS3DH.h>

// led include
#include <neopixel.h>

#include "math.h"

// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
// Used for hardware & software SPI
#define LIS3DH_CS 10

#define G_EPSILON 0.05//unit is m/s^2
#define ROCKING_BIAS 1
#define G 0.98 //unit is m/s^2
#define UPDATE_TIME 200
#define PROCESS_TIME 1000
#define RECOVER_TIME 2000
#define SEND_TIME 3000

#define EGGA //egg a

//LED defines
#define PIXEL_PIN D4
#define PIXEL_COUNT 1
#define PIXEL_TYPE WS2812B

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

//multithread
SYSTEM_THREAD(ENABLED);

//accelerometer
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

//led strip
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);

void UpdateTimerHandler();
void ProcessTimerHandler();
void RecoverTimerHandler();
void SendTimerHandler();
//void AngerEventHandler(const char *event, const char *data);

Timer updateTimer(UPDATE_TIME, UpdateTimerHandler);
Timer processTimer(PROCESS_TIME, ProcessTimerHandler);
Timer recoverTimer(RECOVER_TIME, RecoverTimerHandler);
Timer sendTimer(SEND_TIME, SendTimerHandler);

Thread threadMotor("threadMotor", threadMotorProc);
Thread threadLed("threadLed", threadLedProc);

int camPin = D5;//temporary
int vibePin = D6;//temporary

volatile float anger;//volatile for shared variable
vec3 down;
vec3 curAccel;
bool readyToSend;
bool isG;
float angerDeltaSend;
const int angerRecoverStep = 15;
const float angerInit = 50.f;
const float angerMax = 100.f;
const float angerMin = 0.f;
const float angerDeltaSendFactor = 0.8f;
const float angerDeltaHorizontalFactor = 30.f;
const float angerDeltaVerticalFactor = 9.f;

#ifdef EGGA
const char* angerSendEvent = "Egg_A_Send";
const char* angerReceiveEvent = "Egg_B_Send";
#endif

#ifdef EGGB
const char* angerSendEvent = "Egg_B_Send";
const char* angerReceiveEvent = "Egg_A_Send";
#endif

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif

void setup(void) {
    // strip initialization
    strip.begin();
    strip.setBrightness(20);
    strip.show();
    
    // initialization
    down = vec3(0,0,-1);
    curAccel = vec3(0,0,-1);
    readyToSend = false;
    anger = angerInit;
    angerDeltaSend = 0.f;
    
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

  lis.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!

  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");
  
  //start timers
  updateTimer.start();
  processTimer.start();
  recoverTimer.start();
  sendTimer.start();
  
#if defined EGGA || defined EGGB
  Particle.subscribe(angerReceiveEvent, AngerEventHandler);
#endif
}

bool IsG(vec3 a)
{
    float diff = a.length() - G;
    //Serial.println(diff);
    float diffAbs = diff > 0 ? diff : -diff;
    //Serial.print(diffAbs);
    //Serial.print(" ? ");
    //Serial.println(G_EPSILON);
    if(diffAbs < G_EPSILON)
        return true;
    else
        return false;
}

void loop() {
    if(readyToSend)
    {
        #if defined EGGA || defined EGGB
        Particle.publish(angerSendEvent, String(angerDeltaSend), PUBLIC);
        #endif
        angerDeltaSend = 0;
        readyToSend = false;
    }
}

float GetDeltaAnger(vec3 a)
{
    vec3 norDown = normalize(down);
    float projDownScalar = dot(norDown, curAccel);//we need this
    vec3 projDown = projDownScalar * norDown;
    vec3 projPlane = curAccel - projDown;
    float projPlaneLength = projPlane.length();//we need this
    
    //Serial.print("projDownScalar:"); Serial.println(projDownScalar, 6);
    //Serial.print("projPlaneLength:"); Serial.println(projPlaneLength, 6);
    
    float verticalAbs = projDownScalar - down.length();
    if(verticalAbs < 0)
        verticalAbs = -verticalAbs;
    float horizontalAbs = projPlaneLength;
    
    //Serial.print("vertical:"); Serial.println(verticalAbs, 6);
    //Serial.print("horizontal:"); Serial.println(horizontalAbs, 6);
    
    //decide shaking or rocking
    float tangentSlope = 100;//init with infinity
    if(horizontalAbs>0)
        tangentSlope = verticalAbs / horizontalAbs;
    
    if((verticalAbs < 0.3 || (tangentSlope > 0 && tangentSlope < 1.f + ROCKING_BIAS)) //tan(45 degree) = 1
    && horizontalAbs < 0.5) //to prevent horizontal shake
    {
        //rocking
        float deltaAnger = -angerDeltaHorizontalFactor * horizontalAbs;
        Serial.print("    Rocking = ");
        Serial.println(deltaAnger, 6);
        return deltaAnger;
    }
    else
    {
        //shaking
        float deltaAnger = angerDeltaVerticalFactor * verticalAbs;
        Serial.print("    Shaking = ");
        Serial.println(deltaAnger, 6);
        return deltaAnger;
    }
}

void UpdateTimerHandler()
{
    lis.read();      // get X Y and Z data at once
    
    vec3 raw(lis.x, lis.y, lis.z);
    
    sensors_event_t event;
    lis.getEvent(&event);
  
    vec3 cooked(event.acceleration.x, event.acceleration.y, event.acceleration.z);
  
    curAccel = cooked;
    
    if(IsG(curAccel))
    {
        isG = true;
        down = curAccel;
        //Serial.println("isG");
    }
    else
        isG = false;
}

void ProcessTimerHandler()
{
    //Serial.print("X: "); Serial.print(curAccel.x);
    //Serial.print(", Y: "); Serial.print(curAccel.y);
    //Serial.print(", Z: "); Serial.print(curAccel.z);
    //Serial.println(" m/s^2");
    
    //if(IsG(curAccel))
    //{
        //down = curAccel;
        //Serial.println("isG");
    //}
    if(isG)
    {
        //do nothing
    }
    else
    {
        float angerDelta = GetDeltaAnger(curAccel);
        angerDeltaSend += angerDeltaSendFactor * angerDelta;
        anger += angerDelta;
        if(anger<angerMin) anger = angerMin;
        if(anger>angerMax) anger = angerMax;
    }
    
    Serial.print("Anger = ");
    Serial.println(anger, 6);
}

void RecoverTimerHandler()
{
    float angerRecover = (angerInit - anger) / angerRecoverStep;
    //Serial.print(" *Anger to be recovered = ");
    //Serial.println(angerRecover, 6);
    anger += angerRecover;
    if(anger<angerMin) anger = angerMin;
    if(anger>angerMax) anger = angerMax;
    //Serial.print(" *Anger after recovery = ");
    //Serial.println(anger, 6);
}

void SendTimerHandler()
{
    readyToSend = true;
}

void AngerEventHandler(const char *event, const char *data)
{
    float angerDeltaReceive = atof(data);
    Serial.print("AngerDeltaReceive = ");
    Serial.println(angerDeltaReceive, 6);
}

//*MULTI-THREADING STARTS HERE*//

//motor thread
void threadMotorProc()
{
    while(true)
    {
        delay(10000);
        Serial.printlnf("thread motor out put %d *-*", anger);
    }
}

//led thread
void threadLedProc()
{
    while(true)
    {
        AngerRanges(anger);
    }
}

void AngerRanges(int x) {
    if (x > 15 && x <= 25) {} //chilled out bro
    else if (x > 25 && x <= 35) {} //calm + blue/darkpurple
    else if (x > 35 && x <= 45) {} //calm/neut +green, cyan, blue
    else if (x > 45 && x <= 55) { NeutralBreathe(); } //cyan,lite purple,peach
    else if (x > 55 && x <= 65) { MildAnnoyed(); } //mildly annoyed + magenta
    else if (x > 65 && x <= 75) { Agitated(); }//agitated + orange/magenta
    else if (x > 75 && x <= 85) { Rage_Mode(); } //wtf dude +red/yellow
}

void NeutralBreathe(){
  for(int j = 0; j < 3; j++ ) { 
    // Fade IN
    for(int k = 0; k < 256; k++) { 
      switch(j) { 
        case 0: setAll(102,255,255, k); break; //cyan
        case 1: setAll(204,153,255, k); break; //purple
        case 2: setAll(255,153,153, k); break; //peach
      }
      strip.show();
    }
    // Fade OUT
    for(int k = 255; k >= 0; k--) { 
      switch(j) { 
        case 0: setAll(102,255,255, k); break; //cyan
        case 1: setAll(204,153,255, k); break; //purple
        case 2: setAll(255,153,153, k); break; //peach
      }
      strip.show();
      //timer?
    }
  }
}

void MildAnnoyed(){
  for(int j = 0; j < 3; j++ ) { 
    // Fade IN
    for(int k = 0; k < 256; k++) { 
      switch(j) { 
        case 0: setAll(255,0,127, k); break; //magenta
        case 1: setAll(204,153,255, k); break; //purple
        case 2: setAll(255,153,153, k); break; //peach
      }
      strip.show();
    }
    // Fade OUT
    for(int k = 255; k >= 0; k--) { 
      switch(j) { 
        case 0: setAll(255,0,127, k); break; //magenta
        case 1: setAll(204,153,255, k); break; //purple
        case 2: setAll(255,153,153, k); break; //peach
      }
      strip.show();
    }
  }
}

void Agitated(){
  for(int j = 0; j < 3; j++ ) { 
    // Fade IN
    for(int k = 0; k < 256; k++) { 
      switch(j) { 
        case 0: setAll(255,0,127, k); break; //magenta
        case 1: setAll(255,145,0, k); break; //orange
        case 2: setAll(255,153,153, k); break; //peach
      }
      strip.show();
    }
    // Fade OUT
    for(int k = 255; k >= 0; k--) { 
      switch(j) { 
        case 0: setAll(255,0,127, k); break; //magenta
        case 1: setAll(255,145,0, k); break; //orange
        case 2: setAll(255,153,153, k); break; //peach
      }
      strip.show();
    }
  }
}

void Rage_Mode(){
  for(int j = 0; j < 3; j++ ) { 
    // Fade IN
    for(int k = 0; k < 256; k++) { 
      switch(j) { 
        case 0: setAll(255,0,0, k); break; //red
        case 1: setAll(255,145,0, k); break; //orange
        case 2: setAll(254,203,0, k); break; //yellow
      }
      strip.show();
    }
    // Fade OUT
    for(int k = 255; k >= 0; k--) { 
      switch(j) { 
        case 0: setAll(255,0,0, k); break; //red
        case 1: setAll(255,145,0, k); break; //orange
        case 2: setAll(254,203,0, k); break; //yellow
      }
      strip.show();
    }
  }
}

void setAll(uint8_t r, uint8_t g, uint8_t b, uint16_t brightness) {
    for (int f = 0; f < PIXEL_COUNT; f++) {
        strip.setPixelColor(f, (brightness*r/255) , (brightness*g/255), (brightness*b/255)); //maintain color while fading brightness
        delay(5);
    }
}