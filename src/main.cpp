#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <u8g2lib.h>
#include <Wire.h>
#include <ESP32Servo.h>
//#include <LittleFS.h> I believe this is an overkill for this project
//#include <SPIFFS.h> //so lnog the settings won't be changed 100k times, we're safe just by using SPIFFs, ofc, without wear leveling >:(

//non volatility magik ;)
const uint32_t mem_Offset = 0x290000; //avoiding spiffs atm because I want some direct manipulation of the bits in dat SPI flash, lmoufid; hadi 3ibara 3an offset parceque we're literally writing into the SPIFFS partition.
 
template<typename T> //note: for each write cycle we're gonna increase and index and save it within the written packet to act as some sort of wear and tear leveling mechanism, hence giving the device's memory a longer life 
void FlashWrite(uint32_t address, const T& value) {
  //ESP.flashEraseSector((mem_Offset+address)/4096); // /4096 to select the sector row :)
  ESP.flashWrite(mem_Offset+address, (uint32_t*)&value, sizeof(value));
}

void FlashClear()
{
    for(int i=4096; i<=(0x5000); i+=4096)
    ESP.flashEraseSector((mem_Offset+i)/4096);
}
 
template<typename T>
void FlashRead(uint32_t address, T& value) {
  ESP.flashRead(mem_Offset+address, (uint32_t*)&value, sizeof(value));
}

//motor Stuff
U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0);
Servo servo;

// Static Local IP
IPAddress local_IP(4,4,4,100);
IPAddress gateway(4,4,4,100);
IPAddress subnet(255,255,255,0);
const char* ssid     = "DipCoater@1";
const char* password = "ASCE-123#";

//Socket times :)
WebSocketsServer webSocket = WebSocketsServer(80);



//PinDefs
// Stepper Pulses
#define STPm1  16
#define STPm2  17
#define STPm3  18
#define STPDir 13
#define Step   14
#define Servo_Pin  19

//Endstops
#define EndStop 4
//Encoders
#define EncoderClk    25
#define EncoderData   26
#define EncoderButton 23
#define LED_Indicator  2
#define Buzzer_Pin    27

//Func defs
#define debug(x) Serial.println(x); 
#define sound(freq) ledcWriteTone(0,freq)
#define blink(intensity) ledcWrite(1,intensity)
#define degToMicrosec(deg) ((deg*1000)/180)
#define rotate(deg) servo.write(deg)

#define scroll_sound 0
#define button_press_sound 1
#define start_sound 2
#define cancel_sound 3
#define homing_sound 4


//Defs
#define textFont u8g2_font_torussansbold8_8r 


//Timers
hw_timer_t *encoderAction_Handle = NULL;
hw_timer_t *stepperAction_Handle = NULL;


//Stepper Action G(Old)
unsigned long step_period; //in microseconds, this controls the speed of the stepper motor, min = 2ms, max is undefined.
unsigned long step_duration=1; //1ms duration this will require some experimenting  ?
unsigned long ticks;
bool isStepping;


#define rev_to_step(mstep) (200*pow(2,mstep))
#define speed_to_step_period(speed,mstep) (33333.33/(speed*rev_to_step(mstep))) //per sec, speed = rev/s
#define rev_to_distance(revs) revs*38.3708 //we usin GT2 pulley, 12.22mm+pitch/2, neglect that, we get 3.14*12.22 = circumference 
#define distance_to_revs(dist) dist/38.3708
#define distance_to_steps(distance,mstep) (distance_to_revs(distance)*rev_to_step(mstep))




//operation Vars
#define max_microstepping_value = 5;
#define microstepping_to_step ("1/"+ String((int)pow(2,microSteppingValue)))

//operation functions
void setMicrosteps(byte Microsteps)
{
  digitalWrite(STPm1,Microsteps&1);
  digitalWrite(STPm2,(Microsteps>>1)&1);
  digitalWrite(STPm3,(Microsteps>>2)&1);
}

bool isModifying;


byte microSteppingValue = 5; //6th = auto, auto = depends on the speed

#define configurable_cycles 11

int configured_cycles=0;
int cycle_index;
int dip_cycles[12] = {1,1,1,1,1,1,1,1,1,1,1,1}; //let's give em like what ? 12 cycles ?
int dip_distance[12] = {200,200,200,200,200,200,200,200,200,200,200,200};
int dry_distance[12] = {100,100,100,100,100,100,100,100,100,100,100,100};
int dip_duration[12];
int dry_duration[12];
int dry_speed[12] = {5,5,5,5,5,5,5,5,5,5,5,5};
int dip_counts;
int dip_speed[12] = {5,5,5,5,5,5,5,5,5,5,5,5};
byte solution_pose[12];

int profile_dip_cycles[6]={1,1,1,1,1,1};;
int profile_dip_distance[6]={200,200,200,200,200,200};
int profile_dry_distance[6]={100,100,100,100,100,100};
int profile_dip_duration[6];
int profile_dry_duration[6];
int profile_dry_speed[6]={5,5,5,5,5,5};;
int profile_dip_speed[6];
int profile_solution_pose[6];



unsigned long dip_time_reference; //it's really an overkill G, in terms of seonds, it'll need 130 years to fill up this long counter.
unsigned int dip_step=0;

bool isTargeting;
bool isHoming;
bool isOperational;

// void *value_pointers[menu_pages][menu_element_counts]={{void, new int()}, void};

//visuals
const unsigned char progress_icon [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x07, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xff, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xff, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x7e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x7e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x7e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xff, 0xff, 0xff, 0xff, 0x1f, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xff, 0xff, 0xff, 0xff, 0x1f, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xff, 0xff, 0xff, 0xff, 0x1f, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xff, 0x7f, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xff, 0x7f, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xff, 0x7f, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xff, 0x7f, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xff, 0x7f, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xff, 0x7f, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xff, 0x7f, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xff, 0x7f, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xff, 0x7f, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xff, 0x7f, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x7e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x7e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x7e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x7e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x7e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x7e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x7e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x7e, 0x00, 0x00, 0xfc, 0x03, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x7e, 0x00, 0x00, 0xfc, 0x03, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x7e, 0x00, 0x00, 0xfc, 0x03, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x7e, 0x00, 0x00, 0xfc, 0x03, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x07, 
	0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// 'Layer 4', 16x33px
const unsigned char rotate_icon [] PROGMEM = {
	0x80, 0x1f, 0x00, 0x80, 0x7f, 0x00, 0xc0, 0xff, 0x00, 0xc0, 0xff, 0x03, 0xc0, 0xff, 0x07, 0xc0, 
	0x7f, 0x0f, 0xc0, 0xff, 0x00, 0xc0, 0xf3, 0x01, 0xc0, 0xe3, 0x03, 0x80, 0xc1, 0x07, 0x80, 0x81, 
	0x07, 0x00, 0x80, 0x0f, 0x00, 0xe0, 0x07, 0x00, 0xf8, 0x07, 0x00, 0xfe, 0x03, 0x80, 0xff, 0x01, 
	0xe0, 0x7f, 0x00, 0xf8, 0x1f, 0x00, 0xfc, 0x03, 0x00, 0x3f, 0x00, 0x00
};
// 'Layer 5', 16x33px
const unsigned char arrow_down [] PROGMEM = {
	0xe0, 0x03, 0xe0, 0x03, 0xe0, 0x03, 0xe0, 0x03, 0xe0, 0x03, 0xe0, 0x03, 0xe0, 0x03, 0xe0, 0x03, 
	0xe0, 0x03, 0xe0, 0x03, 0xe0, 0x03, 0xe0, 0x03, 0xe1, 0x43, 0xe3, 0x63, 0xe7, 0x73, 0xe7, 0x33, 
	0xee, 0x3b, 0xfe, 0x3f, 0xfc, 0x1f, 0xfc, 0x1f, 0xfc, 0x0f, 0xf8, 0x0f, 0xf8, 0x0f, 0xf8, 0x07, 
	0xf0, 0x07, 0xf0, 0x07, 0xf0, 0x03, 0xe0, 0x03, 0xe0, 0x01, 0xc0, 0x01, 0xc0, 0x01, 0xc0, 0x00, 
	0x80, 0x00
};
// 'Layer 6', 16x33px
const unsigned char arrow_up [] PROGMEM = {
	0x80, 0x01, 0x80, 0x01, 0xc0, 0x01, 0xc0, 0x03, 0xc0, 0x03, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 
	0xf0, 0x0f, 0xf0, 0x0f, 0xf8, 0x0f, 0xf8, 0x1f, 0xf8, 0x1f, 0xfc, 0x1f, 0xfc, 0x3f, 0xfe, 0x3b, 
	0xee, 0x3b, 0xe6, 0x73, 0xe3, 0x63, 0xe3, 0xc3, 0xe1, 0xc3, 0xe0, 0x03, 0xe0, 0x03, 0xe0, 0x03, 
	0xe0, 0x03, 0xe0, 0x03, 0xe0, 0x03, 0xe0, 0x03, 0xe0, 0x03, 0xe0, 0x03, 0xe0, 0x03, 0xe0, 0x03, 
	0x00, 0x02
};

// 'New Project', 128x64px
const unsigned char ASCE [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 
	0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0xc0, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x1f, 0x00, 
	0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0xf0, 0x1f, 0x00, 0x80, 0x7f, 0x00, 0x00, 0xfe, 0x3f, 0x00, 
	0x00, 0x00, 0x80, 0x1f, 0x00, 0x00, 0xfc, 0x1f, 0x00, 0xe0, 0xff, 0x01, 0x80, 0xff, 0x3f, 0x00, 
	0x00, 0x00, 0xc0, 0x1f, 0x00, 0x00, 0xfe, 0x1f, 0x00, 0xf8, 0xff, 0x01, 0xc0, 0xff, 0x3f, 0x00, 
	0x00, 0x00, 0xf0, 0x1f, 0x00, 0x80, 0xff, 0x0f, 0x00, 0xfc, 0xff, 0x03, 0xe0, 0xff, 0x3f, 0x00, 
	0x00, 0x00, 0xf0, 0x7f, 0x00, 0xc0, 0xff, 0x07, 0x00, 0xfe, 0xff, 0x03, 0xf0, 0x0f, 0x3c, 0x00, 
	0x00, 0x00, 0xf0, 0xff, 0x01, 0xe0, 0xe7, 0x01, 0x80, 0x7f, 0xf0, 0x03, 0xf8, 0x01, 0x00, 0x00, 
	0x00, 0x00, 0xf0, 0xff, 0x01, 0xf0, 0xfb, 0x00, 0xc0, 0x0f, 0xc0, 0x03, 0xfc, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0xf8, 0xff, 0x03, 0xf8, 0x7d, 0x00, 0xc0, 0x07, 0xc0, 0x03, 0x7e, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0xf8, 0xff, 0x03, 0xfc, 0x1c, 0x00, 0xe0, 0x03, 0xc0, 0x03, 0x7e, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0xf8, 0xff, 0x03, 0x7c, 0x00, 0x00, 0xf0, 0x01, 0xc0, 0x03, 0x3e, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0xfc, 0xf8, 0x03, 0x7e, 0x00, 0x00, 0xf8, 0x00, 0xe0, 0x03, 0x3f, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0xfc, 0xf0, 0x03, 0x3e, 0x00, 0x00, 0x7c, 0x00, 0xf0, 0x03, 0x3f, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x7e, 0xf0, 0x03, 0x3f, 0x00, 0x00, 0x7c, 0x00, 0xf0, 0x03, 0x3f, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x7e, 0xe0, 0x03, 0x3f, 0x00, 0x00, 0x3e, 0x00, 0xf8, 0x01, 0x3f, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x3e, 0xe0, 0x03, 0x7f, 0x00, 0x00, 0x3e, 0x00, 0xf8, 0x81, 0x1f, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x3f, 0xe0, 0x03, 0x7f, 0x00, 0x00, 0x1f, 0x00, 0xf8, 0x81, 0x1f, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x1f, 0xe0, 0x03, 0xff, 0x00, 0x00, 0x1f, 0x00, 0xf8, 0x81, 0x1f, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x1f, 0xe0, 0x03, 0xff, 0x00, 0x80, 0x1f, 0x00, 0xfc, 0x81, 0x1f, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x1f, 0xe0, 0x03, 0xfe, 0x01, 0x80, 0x0f, 0x00, 0xfc, 0x80, 0x0f, 0xfe, 0x07, 0x00, 
	0x00, 0x80, 0x0f, 0xe0, 0x03, 0xfc, 0x03, 0x80, 0x0f, 0x00, 0xfc, 0xc0, 0x8f, 0xff, 0x07, 0x00, 
	0x00, 0x80, 0x0f, 0xe0, 0x03, 0xfc, 0x07, 0x80, 0x0f, 0x00, 0xfc, 0xc0, 0xcf, 0xff, 0x07, 0x00, 
	0x00, 0x80, 0x0f, 0xe0, 0x03, 0xf8, 0x0f, 0xc0, 0x0f, 0x00, 0x7c, 0xc0, 0xff, 0xff, 0x07, 0x00, 
	0x00, 0xc0, 0x07, 0xe0, 0x03, 0xf0, 0x1f, 0xc0, 0x0f, 0x00, 0x7c, 0xc0, 0xff, 0xff, 0x07, 0x00, 
	0x00, 0xc0, 0x03, 0xe0, 0x07, 0xe0, 0x1f, 0xc0, 0x07, 0x00, 0x3c, 0xc0, 0xff, 0x00, 0x00, 0x00, 
	0x00, 0xe0, 0x03, 0xfc, 0x07, 0xc0, 0x3f, 0xc0, 0x07, 0x00, 0x3c, 0xc0, 0x1f, 0x00, 0x00, 0x00, 
	0x00, 0xe0, 0xff, 0xff, 0x03, 0x80, 0x7f, 0xe0, 0x07, 0x00, 0x00, 0xc0, 0x0f, 0x00, 0x00, 0x00, 
	0x00, 0xf8, 0xff, 0xff, 0x03, 0x00, 0xff, 0xe0, 0x07, 0x00, 0x00, 0xc0, 0x07, 0x00, 0x00, 0x00, 
	0x00, 0xfc, 0xff, 0xef, 0x03, 0x00, 0xfe, 0xe1, 0x07, 0x00, 0x00, 0xc0, 0x03, 0x00, 0x00, 0x00, 
	0x00, 0xfe, 0xff, 0xc0, 0x03, 0x00, 0xfc, 0xe1, 0x07, 0x00, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x00, 
	0x00, 0xfe, 0x1f, 0xc0, 0x03, 0x00, 0xfc, 0xc1, 0x07, 0x00, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x00, 
	0x00, 0xfe, 0x03, 0xc0, 0x03, 0x00, 0xf8, 0xc3, 0x07, 0x00, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x00, 
	0x00, 0xfe, 0x00, 0xe0, 0x03, 0x00, 0xf0, 0xe3, 0x0f, 0x00, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x00, 
	0x00, 0x7e, 0x00, 0xe0, 0x03, 0x00, 0xf0, 0xc3, 0x0f, 0x00, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x00, 
	0x00, 0x78, 0x00, 0xe0, 0x03, 0x00, 0xf0, 0xc3, 0x1f, 0x00, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x00, 
	0x00, 0x3c, 0x00, 0xe0, 0x01, 0x00, 0xf0, 0xcf, 0x1f, 0x00, 0x00, 0xe2, 0x01, 0x00, 0x18, 0x00, 
	0x00, 0x3c, 0x00, 0xe0, 0x01, 0x00, 0xf8, 0x8f, 0x1f, 0x00, 0x00, 0xe3, 0x01, 0x00, 0x3f, 0x00, 
	0x00, 0x3c, 0x00, 0xe0, 0x01, 0x80, 0xfb, 0x8f, 0x3f, 0x00, 0xc0, 0xe7, 0x01, 0xc0, 0x3f, 0x00, 
	0x00, 0x1e, 0x00, 0xe0, 0x01, 0xe0, 0x7d, 0x80, 0x7f, 0x00, 0xf0, 0xe3, 0x01, 0xfe, 0x1f, 0x00, 
	0x00, 0x1e, 0x00, 0xe0, 0x01, 0xf8, 0x3e, 0x00, 0xff, 0x00, 0xf8, 0xe0, 0xff, 0xff, 0x07, 0x00, 
	0x00, 0x1e, 0x00, 0xe0, 0x01, 0xfe, 0x0f, 0x00, 0xfe, 0x07, 0x7e, 0xe0, 0xff, 0xff, 0x01, 0x00, 
	0x00, 0x0e, 0x00, 0xe0, 0x01, 0xff, 0x07, 0x00, 0xfe, 0xff, 0x1f, 0xe0, 0xff, 0x7f, 0x00, 0x00, 
	0x00, 0x0f, 0x00, 0xe0, 0x80, 0xff, 0x03, 0x00, 0xfc, 0xff, 0x07, 0xc0, 0xff, 0x07, 0x00, 0x00, 
	0x00, 0x0f, 0x00, 0xe0, 0xc0, 0xff, 0x00, 0x00, 0xf0, 0xff, 0x01, 0x80, 0xff, 0x00, 0x00, 0x00, 
	0x00, 0x0f, 0x00, 0xe0, 0x80, 0x3f, 0x00, 0x00, 0x80, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x07, 0x00, 0x40, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

//Audio Core
#define sound_partitions 6
#define sound_IDs 16

#define audio_scroll_up 0
#define audio_scroll_down 1
#define audio_value_inc 2
#define audio_value_dec 3
#define audio_boot 4
#define audio_shut_down 5
#define audio_home 6
#define audio_value_edit 7
#define audio_button_push 8
#define audio_cycle 9
#define audio_fail 10
#define audio_start 11
#define audio_finished 12
#define audio_connected 13
#define audio_disconnected 14
#define audio_webCMD 15

int sound_sets[sound_IDs][sound_partitions]={
{760},                  //scroll up 0
{560},                  //scroll down 1
{780,1290},             //inc 2
{1290,780},             //dec 3
{1000,760},             //boot 4
{1000,840},             //shut down 5
{560},                  //home 6
{1033,0,1033},          //edit 7
{450,670},              //Button 8
{310,0,310,0,310},      //cycle 9
{80,0,80},              //fail 10
{840,770,0,840,1000},   //start 11
{445,660,890}           //finished 12
,{660,890}              //Connected 13
,{445,660},             //Disconnected 14
{890}                   //Coms 15
}; 

unsigned long soundMillis[sound_IDs][sound_partitions]={
{180}                      //scroll up 0      
,{180}                      //scroll down 1
,{120,80}                   //inc 2
,{120,80}                   //dec 3
,{300,200}                  //boot 4
,{200,300}                  //shut down 5
,{160}                      //home 6
,{150,150,150}              //edit 7
,{200,50}                   //button 8
,{200,200,200,200,200}      //cycle 9
,{200,200,350}              //fail 10
,{200,100,300,100,200}      //start 11
,{500,400,300}              //finished 12
,{500,400,300}              //Connceted 13
,{500,400,300}              //Disconnected 14
,{200}                      //Coms 15
};                
                                  
byte sound_set_width[sound_IDs] = {1,1,2,2,2,2,1,3,2,5,3,5,3,2,2,1};
byte sound_priority[sound_IDs]; //later to be implemented, which sound cancels which :)
bool isSounding;
unsigned long sound_partition_reference;
byte sound_partition_index; //3 stages of sound or beyond, each stage acts like an index for the array of type of sounds, reset this index when overriding a sound
byte sound_ID;
bool willPlay=1;
bool audioConflict=1;

//Graphics Core
#define mute 0
#define alarms 1
#define minimal 2
#define all 3
byte audio_mode = 3;

void play_audio(byte sound_to_play)
{
    sound_partition_index=0;
    isSounding=1;
    sound_ID=sound_to_play;
    sound_partition_reference=millis();

    switch (audio_mode)
    {
    case mute:
    willPlay=0;
    break;

    case alarms:
    sound_ID >10? willPlay=1:willPlay=0; 
    break;

    case minimal:
    sound_ID>6?willPlay=1:willPlay=0;
    if(sound_ID!=audio_button_push){audioConflict=0;} //we giving the priority to these sounds except button push
    break;
    
    default:
    willPlay=1;
    break;
    }

    if(willPlay){sound(sound_sets[sound_ID][sound_partition_index]);}
}

void performAudioFeedback()
{
    if(isSounding && willPlay)
    {
             //this is quite unoptimized as it'll keep filling the PWM buffer with this freq each loop cycle but eh, can optimize later when this becomes a problem
            if(millis()-sound_partition_reference>=soundMillis[sound_ID][sound_partition_index])
            {
                sound_partition_index++;
                sound(sound_sets[sound_ID][sound_partition_index]);
                sound_partition_reference=millis();
            }
            
            if(sound_partition_index>sound_set_width[sound_ID]){isSounding=0;}
    }else
    {
        sound(0);
        audioConflict=1;
    }
}

int servoSpeed=0;
bool WiFistate=1;
byte easeMode;
#define ease_cubic 2
#define ease_out 1
#define ease_linear 0


//Coms Conveniencessssss
uint8_t coms_status; //if I'd ever need to do something with a coms Flag
uint8_t precoms_status;
#define cStatus_offline 0
#define cStatus_disconnected 1
#define cStatus_connected 2
#define cStatus_transmission 3
#define cStatus_Reception 4
#define cStatus_Error 5
String msgBuffer;
int msgBufferIndex;
int entryIndex;

void socketEvent(uint8_t num,WStype_t type,uint8_t *payload,size_t length) //note: payload pointer appears to be of 8 bits, meaning this may have constrains if not used properly. just a thought of mine :)
{
        switch(type) //do stuff based on the socket events, that the library somewhat handles for us already
        {
            case WStype_DISCONNECTED:
            play_audio(audio_disconnected);
            coms_status=cStatus_disconnected; 
            precoms_status = coms_status;
            break;

            case WStype_CONNECTED:
            play_audio(audio_connected);
            coms_status=cStatus_connected;
            precoms_status = coms_status;
            // Serial.println(webSocket.remoteIP(num).toString());
            break;

            case WStype_ERROR:
            play_audio(audio_fail);
            coms_status = cStatus_Error;
            break;

            case WStype_TEXT:
            play_audio(audio_webCMD); //for debug purposes, since we won't be sending text I guess.
            coms_status=cStatus_Reception;
            msgBuffer = String((char *)payload);

            if(msgBuffer.length()>=2)
            {
                if(msgBuffer.substring(0,2).equals("ss"))
                {
//0 Entry Num, //1 Total Entries, //2 Dip Speed, //3 DrySped, //4 Dip Count, //5 Solution, //6 Dip dist, //7 Dry dist, //8 Dip Dur, //9 Dry Dur 
                            entryIndex=msgBuffer.toInt();

                            msgBufferIndex=msgBuffer.indexOf(",",msgBufferIndex+1);
                            configured_cycles=msgBuffer.substring(msgBufferIndex+1).toInt();

                            msgBufferIndex=msgBuffer.indexOf(",",msgBufferIndex+1);
                            dip_speed[entryIndex]=msgBuffer.substring(msgBufferIndex+1).toInt();

                            msgBufferIndex=msgBuffer.indexOf(",",msgBufferIndex+1);
                            dry_speed[entryIndex]=msgBuffer.substring(msgBufferIndex+1).toInt();

                            msgBufferIndex=msgBuffer.indexOf(",",msgBufferIndex+1); //speak about boiler plate
                            dip_cycles[entryIndex]=msgBuffer.substring(msgBufferIndex+1).toInt();

                            msgBufferIndex=msgBuffer.indexOf(",",msgBufferIndex+1);
                            solution_pose[entryIndex]=msgBuffer.substring(msgBufferIndex+1).toInt();

                            msgBufferIndex=msgBuffer.indexOf(",",msgBufferIndex+1);
                            dip_distance[entryIndex]=msgBuffer.substring(msgBufferIndex+1).toInt();

                            msgBufferIndex=msgBuffer.indexOf(",",msgBufferIndex+1);
                            dry_distance[entryIndex]=msgBuffer.substring(msgBufferIndex+1).toInt();

                            msgBufferIndex=msgBuffer.indexOf(",",msgBufferIndex+1);
                            dip_duration[entryIndex]=msgBuffer.substring(msgBufferIndex+1).toInt();

                            msgBufferIndex=msgBuffer.indexOf(",",msgBufferIndex+1);
                            dry_duration[entryIndex]=msgBuffer.substring(msgBufferIndex+1).toInt();
                }else
                if(msgBuffer.substring(0,2).equals("sp"))
                {
                            entryIndex=msgBuffer.toInt();

                            msgBufferIndex=msgBuffer.indexOf(",",msgBufferIndex+1);
                            profile_dip_speed[entryIndex]=msgBuffer.substring(msgBufferIndex+1).toInt();

                            msgBufferIndex=msgBuffer.indexOf(",",msgBufferIndex+1);
                            profile_dry_speed[entryIndex]=msgBuffer.substring(msgBufferIndex+1).toInt();

                            msgBufferIndex=msgBuffer.indexOf(",",msgBufferIndex+1); //speak about boiler plate
                            profile_dip_cycles[entryIndex]=msgBuffer.substring(msgBufferIndex+1).toInt();

                            msgBufferIndex=msgBuffer.indexOf(",",msgBufferIndex+1);
                            profile_solution_pose[entryIndex]=msgBuffer.substring(msgBufferIndex+1).toInt();

                            msgBufferIndex=msgBuffer.indexOf(",",msgBufferIndex+1);
                            profile_dip_distance[entryIndex]=msgBuffer.substring(msgBufferIndex+1).toInt();

                            msgBufferIndex=msgBuffer.indexOf(",",msgBufferIndex+1);
                            profile_dry_distance[entryIndex]=msgBuffer.substring(msgBufferIndex+1).toInt();

                            msgBufferIndex=msgBuffer.indexOf(",",msgBufferIndex+1);
                            profile_dip_duration[entryIndex]=msgBuffer.substring(msgBufferIndex+1).toInt();

                            msgBufferIndex=msgBuffer.indexOf(",",msgBufferIndex+1);
                            profile_dry_duration[entryIndex]=msgBuffer.substring(msgBufferIndex+1).toInt();
                }else
                if(msgBuffer.substring(0,2).equals("se"))
                {
                //insert settings code here.
                }else
                if(msgBuffer.substring(0,2).equals("st")){ //start
                //insert start code here
                }else
                if(msgBuffer.substring(0,2).equals("cn")){
                //insert cancel code here
                } //cancel
            } 
            //SS = SetSettings
            // Serial.printf("[%u] Text: %s\n",num,payload);
            break;
        }
}

//LED indication
int blinkMillis[][6] 
{
    {0},
    {0}, //disconnected
    {100,100,200,200,100,100},//connected
    {400,100,600,300}, //transmission
    {0}, //reception
    {0} //error
};
int blinkIntensities[][6]
{
    {0},//offline
    {0},//disconnected
    {200,800,2000,800,200,0},//connected
    {2000,0,1000,0},//transmission
    {0},//reception
    {0}//error
}; //currently, we're not concerned with stuff other than connection status so. . . audio will do the rest.

int blinkIndex;
unsigned long blinkInstance;
unsigned long comsInstance;

void performBlink()
{
        if(millis()-blinkInstance>= blinkMillis[coms_status][blinkIndex])
        {
            blink(blinkIntensities[coms_status][blinkIndex]);
            blinkIndex>4?blinkIndex=0:blinkIndex++;
            blinkInstance=millis();
        }
        if(millis()-comsInstance>= 1200)
        {
            coms_status=precoms_status;
            comsInstance=millis();
        }
}



//Menu Vars
#define menu_pages 2
#define menu_element_counts 7
#define menu_element_count_page1 6
#define menu_element_count_page2 6 //pages >100 represent custom tile screens
byte menu_element_count[menu_pages] = {6,6}; //six indexes per page ? 

#define iconFont u8g2_font_open_iconic_all_1x_t
#define icon_return "\u00fa"
#define icon_audio "\u0115"
#define icon_solution "\u005c"
#define icon_dip "\u004f"
#define icon_dry "\u0052"
#define icon_page_up "\u004c"
#define icon_page_down "\u0049"
#define icon_reset "\u00f3"
#define icon_alarm "\u005d"
#define icon_confirm "\u0073"
#define icon_settings "\u011a"
#define icon_muted "\u0117"
#define icon_minimal_audio "\u0116"
#define icon_WiFi_Off "\u0057"
#define icon_WiFi_On "\u00f8"
#define icon_abort "\u011b"
#define icon_fail "\u0118"

int encoderBuffer;
int encoderBuffer_upper_constrain;
int encoderBuffer_lower_constrain;

byte page_index=100; //>100 menu pages, 0,1,2...etc = submenu pages
int element_index=0;

int preset_index[12];
int profile;
bool isPreset;

String menu_elements[menu_pages][menu_element_counts] = {{"Load: ","Dip count: ","Dip Dur: ","Dry Dur: ","Solution: ","Confirm   "},{"DipSpd:","Dry Spd:","DipDepth:","DryDepth:","reset   ","return   "}}; 



void drawMenu() //some drastic changes will happen to menu and page systems :)
{        
        for(int j=0;j<menu_element_count[page_index];j++)
        {   
            oled.setFont(textFont);
            if(j==element_index)
            {
            if(isModifying){oled.drawBox(1,j*9,123,10);}else{oled.drawFrame(1,j*9,123,10);}
            }
            oled.setDrawColor(2);
            oled.setFontMode(1);
            oled.setCursor(2,j*9+8);
            oled.print(menu_elements[page_index][j]);
            oled.setCursor(oled.getStrWidth(menu_elements[page_index][j].c_str())+2,j*9+8);
            if(page_index==0){
            switch (j)
            {
            case 0:
            oled.print("profile "); oled.print(isPreset? profile : preset_index[cycle_index]);
            break;

            case 1:
            oled.print(isPreset? profile_dip_cycles[profile] :dip_cycles[cycle_index]);
            break;

            case 2:
            oled.print(isPreset? profile_dip_duration[profile] : dip_duration[cycle_index]); oled.print("s");
            break;

            case 3:
            oled.print(isPreset? profile_dry_duration[profile] : dry_duration[cycle_index]); oled.print("s");
            break;

            case 4: 
            oled.print(isPreset? profile_solution_pose[profile] : solution_pose[cycle_index]);
            oled.print(" "); oled.setFont(iconFont); oled.print(icon_solution);
            break;

            case 5:
            oled.setFont(iconFont); oled.print(icon_confirm);oled.setCursor(118,59);oled.print(icon_page_down);
            break;

            }
            }
            else
            if(page_index==1)
            {
            switch (j)
            {
            case 0:
            oled.print(isPreset? profile_dip_speed[profile] :dip_speed[cycle_index]); 
            oled.print("mm/s"); oled.setFont(iconFont); oled.print(icon_dip);
            break;

            case 1:
            oled.print(isPreset? profile_dry_speed[profile] :dry_speed[cycle_index]); 
            oled.print("mm/s"); oled.setFont(iconFont); oled.print(icon_dry);
            break;

            case 2:
            oled.print(isPreset? profile_dip_distance[profile] :dip_distance[cycle_index]); 
            oled.print("mm"); oled.setFont(iconFont); oled.print(icon_dip);
            break;

            case 3:
            oled.print(isPreset? profile_dry_distance[profile] :dry_distance[cycle_index]); 
            oled.print("mm"); oled.setFont(iconFont); oled.print(icon_dry);
            break;

            case 4:
            oled.setFont(iconFont);oled.print(icon_reset);
            break;

            case 5:
            oled.setFont(iconFont);oled.print(icon_return); oled.setCursor(118,59);oled.print(icon_page_up);
            break;

            }
            }
    }
}


String settings_elements[7]={"Presets","Sounds:","µStepping:","ServoSpd:","Ease mode:","Toggle WiFi","return   "}; //rotation opens a submenu to select each rotation pose manually, shit should be saved in progmem but eh;
void drawSettings()
{
        for(int j=0;j<7;j++)
        {  
            oled.setFont(textFont);
            if(j==element_index)
            {
            if(isModifying){oled.drawBox(1,j*9,123,10);}else{oled.drawFrame(1,j*9,123,10);}
            }
            oled.setDrawColor(2);
            oled.setFontMode(1);
            oled.setCursor(2,j*9+8);
            oled.print(settings_elements[j]);
            oled.setCursor(oled.getStrWidth(settings_elements[j].c_str())+2,j*9+8);
            switch (j)
            {
            case 1:
                switch (audio_mode)
                {
                case mute:
                oled.print("mute "); oled.setFont(iconFont); oled.print(icon_muted);
                break;
                case alarms:
                oled.print("alarm "); oled.setFont(iconFont); oled.print(icon_alarm);
                break;
                case minimal:
                oled.print("minimal"); oled.setFont(iconFont); oled.print(icon_minimal_audio);
                break;
                case all:
                oled.print("all "); oled.setFont(iconFont); oled.print(icon_audio);
                }
            break;

            case 2:
            oled.print(microstepping_to_step);
            break;

            case 3:
            oled.print((int)servoSpeed); oled.print("deg/s");
            break;

            case 4:
                switch (easeMode)
                {
                case ease_cubic:
                oled.print("cubic");
                break;
                case ease_out:
                oled.print("out");
                break;
                case ease_linear:
                oled.print("linear");
                break;
                }
            break;

            case 5: 
            oled.print("   "); oled.setFont(iconFont); WiFistate? oled.print(icon_WiFi_On) : oled.print(icon_WiFi_Off);
            break;

            case 6:
            oled.setFont(iconFont); oled.print(icon_return);
            break;
            }
        }
}


int x_cur;
int y_cur;
void drawCycles()//Array style ? mayhaps
{
  oled.setDrawColor(2);
  oled.setFontMode(1);
  x_cur=0;
  y_cur=0;
  oled.setFont(textFont);
  for (int i = 0; i <= configured_cycles; i++)
  {
    if(i==element_index)
    {
        oled.drawBox(x_cur*20,y_cur*10+3,10,10);
    }else
    {
        oled.drawFrame(x_cur*20,y_cur*10+3,10,10);
    }
    oled.setCursor(x_cur*20+2,y_cur*10+12);
    oled.setDrawColor(2);
    oled.setFontMode(1);
    oled.print(i);
    x_cur++;
    if(x_cur>5){
        x_cur=0;y_cur++;}
  }
  if(element_index==100){oled.drawRBox(0,29,26,10,4);}else if(element_index==101){oled.drawRBox(70,29,26,10,4);}else if(element_index==102){oled.drawBox(0,39,128,12);}else if(element_index==103){oled.drawBox(0,49,128,12);}
oled.drawHLine(0,28,128);
  oled.setDrawColor(2);
  oled.setFontMode(1);
  oled.setCursor(4,37);
  oled.print("+");
  oled.setCursor(76,37);
  oled.print("-");
  oled.setDrawColor(2);
  oled.setFontMode(1);
  oled.setCursor(4,48);
  oled.print("Start");
  oled.setCursor(4,58);
  oled.print("Clear & Return");
}

static const unsigned char image_Power_25x27_bits[] U8X8_PROGMEM = {0xf8,0xff,0x3f,0x00,0x04,0x00,0x40,0x00,0x02,0x00,
0x80,0x00,0x01,0x00,0x00,0x01,0x01,0x10,0x00,0x01,0x01,0x38,0x00,0x01,0x01,0x38,0x00,0x01,0x81,0x39,0x03,0x01,0xc1,
0x39,0x07,0x01,0xe1,0x38,0x0e,0x01,0x61,0x38,0x0c,0x01,0x61,0x38,0x0c,0x01,0x61,0x10,0x0c,0x01,0x61,0x00,0x0c,0x01,
0x61,0x00,0x0c,0x01,0x61,0x00,0x0c,0x01,0xe1,0x00,0x0e,0x01,0xc1,0x01,0x07,0x01,0x81,0x83,0x03,0x01,0x01,0xff,0x01,
0x01,0x01,0x7c,0x00,0x01,0x01,0x00,0x00,0x01,0x01,0x00,0x00,0x01,0x03,0x00,0x80,0x01,0x06,0x00,0xc0,0x00,0xfc,0xff,0x7f,0x00,0xf8,0xff,0x3f,0x00};
static const unsigned char image_Power_hvr_25x27_bits[] U8X8_PROGMEM = {0xf8,0xff,0x3f,0x00,0xfc,0xff,0x7f,0x00,0xfe,
0xff,0xff,0x00,0xff,0xff,0xff,0x01,0xff,0xef,0xff,0x01,0xff,0xc7,0xff,0x01,0xff,0xc7,0xff,0x01,0x7f,0xc6,0xfc,0x01,
0x3f,0xc6,0xf8,0x01,0x1f,0xc7,0xf1,0x01,0x9f,0xc7,0xf3,0x01,0x9f,0xc7,0xf3,0x01,0x9f,0xef,0xf3,0x01,0x9f,0xff,0xf3,0x01,
0x9f,0xff,0xf3,0x01,0x9f,0xff,0xf3,0x01,0x1f,0xff,0xf1,0x01,0x3f,0xfe,0xf8,0x01,0x7f,0x7c,0xfc,0x01,0xff,0x00,0xfe,0x01,
0xff,0x83,0xff,0x01,0xff,0xff,0xff,0x01,0xff,0xff,0xff,0x01,0xff,0xff,0xff,0x01,0xfe,0xff,0xff,0x00,0xfc,0xff,0x7f,0x00,0xf8,0xff,0x3f,0x00};

void drawMain()
{

oled.setBitmapMode(1);
oled.setDrawColor(2);
oled.setFontMode(1);
oled.setFont(u8g2_font_helvB08_tr);
oled.drawStr(2, 9, "DipCoater  Beta");
oled.drawLine(0, 11, 125, 11);
oled.setFont(u8g2_font_haxrcorp4089_tr);
oled.drawStr(1, 22, "Settings"); oled.setFont(iconFont); oled.setCursor(50,22); oled.print(icon_settings); oled.setFont(textFont);
oled.setFont(u8g2_font_profont22_tr);
oled.drawStr(63, 54, "Start");

switch (element_index)
{
case 0:
    oled.drawFrame(0, 13, 128, 13);
    oled.drawXBMP( 5, 34, 25, 27, image_Power_25x27_bits);
    break;
case 1:
    oled.drawXBMP( 5, 34, 25, 27, image_Power_hvr_25x27_bits);
    break;
case 2:

    oled.drawFrame(54, 32, 73, 28);
    oled.drawXBMP( 5, 34, 25, 27, image_Power_25x27_bits);
    break;
default:
    break;
}

}

#define drying_status 0
#define dipping_status 1
#define rotating_status 2
#define homing_status 3
#define finished_status 4

byte operation_Status=4; 

void drawProgress()
{
    oled.setBitmapMode(1);
    oled.drawXBMP(1,1,125,63,progress_icon);
    oled.setCursor(62,9);
    oled.print("Cycle:");
    oled.setCursor(62,19);
    oled.print(cycle_index);oled.print("/");oled.print(configured_cycles);
    oled.drawHLine(62,21,63);
    oled.setCursor(62,31);
    oled.print("Dip:");
    oled.setCursor(62,40);
    oled.print(dip_counts);oled.print("/");oled.print(dip_cycles[cycle_index]);
    oled.drawHLine(62,42,63);
    oled.setCursor(62,51);

    switch(operation_Status)
    {
        case drying_status:
        oled.print("Dry Time:");
        oled.setCursor(62,61);
        oled.print((((millis()-dip_time_reference)/1000) <= dry_duration[cycle_index])?((millis()-dip_time_reference)/1000):dry_duration[cycle_index]);oled.print("/");oled.print(dry_duration[cycle_index]);oled.print("s");
        oled.drawXBMP(18,16,15,33,arrow_up);
        break;
        
        case dipping_status:
        oled.print("Dip Time:");
        oled.setCursor(62,61);
        oled.print((((millis()-dip_time_reference)/1000) <= dip_duration[cycle_index])?((millis()-dip_time_reference)/1000):dip_duration[cycle_index]);oled.print("/");oled.print(dip_duration[cycle_index]);oled.print("s");
        oled.drawXBMP(18,16,15,33,arrow_down);

        break;

        case rotating_status:
        oled.drawXBMP(38,7,20,20,rotate_icon);
        oled.setCursor(62,51);
        oled.print("Changing");
        oled.setCursor(62,61);
        oled.print("Solution");
        break;

        case homing_status:
        oled.setCursor(64,57);
        oled.print("Homing");
        break;

        case finished_status:
        oled.setCursor(61,57);
        oled.print("Finished");
        break;

        default:
        break;
    }
        oled.setCursor(20,8);oled.setFont(iconFont);
        oled.print(icon_solution);oled.setFont(textFont); oled.print(solution_pose[cycle_index]);
        
    switch (element_index)
    {
    case 0:
         operation_Status == finished_status? oled.drawStr(11,64,"return") : oled.drawStr(11,64,"abort");
         oled.setFontMode(0);
         oled.setDrawColor(0);
         oled.drawStr(11,56,"edit");
        break;
    case 1:
    oled.drawStr(11,56,"edit");
    oled.setFontMode(0);
    oled.setDrawColor(0);
    operation_Status == finished_status? oled.drawStr(11,64,"return") : oled.drawStr(11,64,"abort");
    break;
    default:
    oled.drawStr(11,56,"edit");
    operation_Status == finished_status? oled.drawStr(11,64,"return") : oled.drawStr(11,64,"abort");
        break;
    }
    oled.setDrawColor(2);
    oled.setFontMode(1);

}

void 
renderScreen() //do we draw the menu ? do we draw the status ? :o what do we draw here ?!
{
    oled.clear();
    switch(page_index)
    {
        case 100:
        drawMain();
        break;

        case 101:
        drawCycles();
        break;

        case 102:
        drawProgress();
        break;

        case 103:
        drawSettings();
        break;

        default:
        drawMenu();
        break;
    }
    oled.sendBuffer();
}

//ISRs
#define encoder_value_action 0
#define encoder_scroll_action 1
#define encoder_no_action 2
int encoderCount;
byte encoderActionMode=encoder_scroll_action;

void IRAM_ATTR encoderClockISR() //no need to put deadtime since we have a hardware debouncer
{
    if(digitalRead(EncoderData)==1){encoderCount--;}else{encoderCount++;}
}

void IRAM_ATTR retrieveEncoderAction()
{
    switch(encoderActionMode)
    {
        case encoder_no_action:
        break;

        case encoder_scroll_action:
        if(encoderCount>0)
        {
            element_index++;
            play_audio(audio_scroll_up);
            if(page_index<100){
            if(element_index>=menu_element_count[page_index])
            {
                element_index=0;
                ((page_index>=menu_pages-1))? page_index=0: page_index++;
            }}else
            {
                switch (page_index)
                {
                case 100:
                if(element_index>2)
                {
                    element_index=0;
                }
                    break;
                case 101:
                if(element_index==configured_cycles+1){element_index=100;}
                if(element_index>103){element_index=0;}
                break;
                case 102:
                if(element_index>1){element_index=0;}
                    break;
                case 103:
                if(element_index>6){element_index=0;}
                break;
                }
            }
        }else if(encoderCount<0)
        {
            element_index--;
            play_audio(audio_scroll_down);
            if(page_index<100){
            if(element_index<0)
            {
                ((page_index<=0))? page_index=menu_pages-1: page_index--;
                element_index=menu_element_count[page_index]-1;
            }}else
            {
                switch (page_index)
                {
                case 100:
                if(element_index<0)
                {
                    element_index=2;
                }
                    break;
                case 101:
                if(element_index<0){element_index=103;}else if(element_index<100 && !(element_index<=configured_cycles)){element_index=configured_cycles;}
                break;
                case 102:
                if(element_index<0){element_index=1;}
                    break;
                case 103:
                if(element_index<0){element_index=6;}
                break;
                }
            }
        }
        encoderCount=0;
        break;

        case encoder_value_action:
        if(encoderCount>0){play_audio(audio_value_inc);}else if(encoderCount<0){play_audio(audio_value_dec);}
        if(abs(encoderCount)>5)
        {   
            encoderBuffer+=encoderCount*4;

        }else
        {
            encoderBuffer+=encoderCount;
            
        }
        if(encoderBuffer>encoderBuffer_upper_constrain)
        {
            encoderBuffer=encoderBuffer_upper_constrain;
            play_audio(audio_fail);
        }else if(encoderBuffer<encoderBuffer_lower_constrain)
        {
            encoderBuffer=encoderBuffer_lower_constrain;
            play_audio(audio_fail);
        }

        encoderCount=0;
        break;

    }
}

double targetSteps;
double halftargetSteps;
double stepCounts;

void IRAM_ATTR step()
{
  if(isStepping){
  ticks++;
  if(ticks>=step_duration){digitalWrite(Step,0);}//reset stepper motor
  if(ticks>=step_period){ticks=0;digitalWrite(Step,1);stepCounts++;
  } //commence the
  }
  else
  {
    digitalWrite(Step,0);
  } 
}

void setSpeed(float deploy_speed)
{   

    setMicrosteps(microSteppingValue);
    step_period=speed_to_step_period(distance_to_revs(deploy_speed),microSteppingValue);

}

unsigned long interpolationInstance;
double interpolationTarget;

void setTargetPose(int target,bool direction)
{
    if(!isHoming){
    
    interpolationTarget = target;
    isTargeting=1;
    stepCounts=0;
    targetSteps= distance_to_steps(target,microSteppingValue);  //in mm
    halftargetSteps = targetSteps/2; //to save some clock cycles in the interrupt service routine stuff. 
    isStepping=1;
    digitalWrite(STPDir,direction);
    interpolationInstance = millis();
    }
}

int stepper_speed;
double halftemporalTarget;
double temporalTarget;

void checkPose()
{
    if(isTargeting){

        if(stepCounts>=targetSteps)
        {
            isTargeting=0;
            isStepping=0;   //do the rest of what u do after arrivin to the next cycle
        }

                        temporalTarget=(interpolationTarget/stepper_speed)*1000;//in msecs {:)
                        switch(easeMode){
                        case ease_linear:
                        setSpeed(stepper_speed);
                        break;
                        
                        case ease_out:
                        setSpeed(
                             constrain(
                            ( interpolationTarget*(3/2))*(pow<double>(  ((2.00/temporalTarget)*(millis()-interpolationInstance))  ,2)),
                        1,60
                        ));
                        break;

                        case ease_cubic: //this part has indeeeed proven to be much more difficult than man thought, but it goes like this:

                        if((millis()-interpolationInstance)<(temporalTarget/4.00)){

                        setSpeed(
                             constrain(
                            ( interpolationTarget*(3/2))*(pow<double>(  ((2.00/temporalTarget)*(millis()-interpolationInstance))  ,2)),
                        1,100
                        ));
                        }
                        else {
                            setSpeed(

                                constrain((interpolationTarget*3/2) * pow(((2.00/temporalTarget)*((temporalTarget/4.00))),2) - (( interpolationTarget*(3/2))*(pow<double>(  ((2.00/temporalTarget)*(millis()-interpolationInstance)),2))),5,60)

                            );
                             }

                        //try
                        //{
                        //setSpeed(interpolationTarget/(3*(interpolationTarget/stepper_speed)*pow(((2*((millis()-interpolationInstance)/1000)/10)-1 ),2/3) ));
                        //}catch(error_t e){break;}
                        // (millis()-interpolationInstance) > ((interpolationTarget/stepper_speed)/2) ? setSpeed((interpolationTarget*(1/3)*sqrt((1/(interpolationTarget/stepper_speed))*((-(millis()-interpolationInstance))+(interpolationTarget/stepper_speed)/2)))) : 
                        // setSpeed(interpolationTarget*(1/3)*sqrt((1/(interpolationTarget/stepper_speed))*(millis()-interpolationInstance))) ;
                         //again, heavy stuff on the cpu >:(
                        break;
                        }

            }
    else{
        if(!isHoming){
        isStepping=0; 
        setSpeed(0);
        stepCounts=0;}
        }
}

bool isRotating; //this flag will halt every operation to do with the stepper unless the servo isn't rotating to the solution.
double servoTarget;
unsigned long servoMillis; //for default, let's go with 20 degrees each second :)
byte solution_poses[6]={0,38,73,109,144,180};//here put the angles of the servo to meet required solutions, this will be calculated in accordance with the mechanical design

int lastServoPose;
//figure out the change in position of the servo per milliSecond depending on the speed; 
void servoAction() //change the servo action stuff into microseconds for smoother operation
{
    if(isRotating)
    {
     if(servoTarget != lastServoPose)
     {
        if(servoSpeed==0){
        lastServoPose=servoTarget;
        servo.write(servoTarget);}else{
        if(millis()-servoMillis>=(1000/servoSpeed))
        {
            servoTarget > lastServoPose ? rotate(lastServoPose+1): rotate(lastServoPose-1);
            servoTarget > lastServoPose ? lastServoPose++ : lastServoPose--;
            servoMillis = millis();
        }}
     }else
     {
        isRotating=0;
     } 
    }
}

//Update this with a control system.
void select_solution(byte solution_pose)
{
    isRotating=1;
    servoTarget = solution_pose;
    if(servoSpeed==0){
    lastServoPose=servoTarget;
    servo.write(servoTarget);}
}


void home()
{
        isTargeting=0; //maybe combine targeting and homing and dipping and lifting into one byte with #definitionsss
        isHoming=1;
        setSpeed(10);
        digitalWrite(STPDir,1);
        isStepping=1;

}
void checkHome()
{
    if(isHoming){
     if(!digitalRead(EndStop)){isHoming=0;isStepping=0;play_audio(audio_home);if(operation_Status==finished_status){select_solution(solution_poses[0]);}}
    }
}


bool armButton;
unsigned long lastInterruptMillis;

void IRAM_ATTR buttonAction()
{
    if(millis()-lastInterruptMillis>=50)
    {
    armButton=1;
    lastInterruptMillis=millis();
    }
}

void pushToMemory() //this is gonna be a bit difficult on the cpu but hey, nobody said shit gon be ez <:);
{
    FlashClear();
    FlashWrite<int[6]>(4096, profile_dip_cycles); //I guess wear and tear can wait for now because I have school, but I should gain 2x the lifetime of the sector if I just yk, implement an index and basculate between +4096 (sector size) and + 0
    FlashWrite<int[6]>(sizeof(int[6])+4096, profile_dip_distance);
    FlashWrite<int[6]>(2*sizeof(int[6])+4096, profile_dip_duration);
    FlashWrite<int[6]>(3*sizeof(int[6])+4096, profile_dip_speed);
    FlashWrite<int[6]>(4*sizeof(int[6])+4096, profile_dry_distance);
    FlashWrite<int[6]>(5*sizeof(int[6])+4096, profile_dry_duration);
    FlashWrite<int[6]>(6*sizeof(int[6])+4096, profile_dry_speed);
    FlashWrite<int[6]>(7*sizeof(int[6])+4096, profile_solution_pose);
    FlashWrite<bool>(8*sizeof(int[6])+4096,WiFistate);
    FlashWrite<byte>(9*sizeof(int[6])+4096,audio_mode);
    FlashWrite<byte>(9*sizeof(int[6])+sizeof(byte)+4096,easeMode);
    FlashWrite<int>(9*sizeof(int[6])+2*sizeof(byte)+4096,servoSpeed);
    FlashWrite<byte>(9*sizeof(int[6])+2*sizeof(byte)+sizeof(int)+4096,microSteppingValue); //welp, this is a catastrophic way to implement non volatility but lah ghaleb kho man's just in a rush >:(
}

void armValues()
{
    FlashRead<int[6]>(4096, profile_dip_cycles);
    FlashRead<int[6]>(2*sizeof(int[6])+4096, profile_dip_duration);
    FlashRead<int[6]>(3*sizeof(int[6])+4096, profile_dip_speed);
    FlashRead<int[6]>(4*sizeof(int[6])+4096, profile_dry_distance);
    FlashRead<int[6]>(5*sizeof(int[6])+4096, profile_dry_duration);
    FlashRead<int[6]>(6*sizeof(int[6])+4096, profile_dry_speed);
    FlashRead<int[6]>(7*sizeof(int[6])+4096, profile_solution_pose);
    FlashRead<bool>(8*sizeof(int[6])+4096,WiFistate);
    FlashRead<byte>(9*sizeof(int[6])+4096,audio_mode);
    FlashRead<byte>(9*sizeof(int[6])+sizeof(byte)+4096,easeMode);
    FlashRead<int>(9*sizeof(int[6])+2*sizeof(byte)+4096,servoSpeed);
    FlashRead<byte>(9*sizeof(int[6])+2*sizeof(byte)+sizeof(int)+4096,microSteppingValue); 
}

bool dipPoseLock;

void dipCoat()
{
    if(isOperational)
    {
        switch (dip_step)
        {
        case 0:
            operation_Status=homing_status;
            play_audio(audio_start);
            home();
            dip_step++;
            break;
        case 1:
            if(!isHoming){dip_step++;}
            break;
        case 2:
        operation_Status=rotating_status;
        select_solution(solution_poses[solution_pose[cycle_index]]);
        dip_step++;
        break;
        case 3:
        if(!isRotating)
        {
            dip_step++;
        }
        break;
        case 4:
            stepper_speed = dip_speed[cycle_index]; 
            dipPoseLock ? setTargetPose(dip_distance[cycle_index]-dry_distance[cycle_index],0) : setTargetPose(dip_distance[cycle_index],0);
            operation_Status=dipping_status;
            dip_step++;
            break;
        case 5:
            stepper_speed = dip_speed[cycle_index]; //yk, so we can change speed in real time I guess.  . . nervous about this ngl.
            if(!isTargeting){dip_time_reference=millis();dip_step++;} //in before, this was to set the time reference after the solution //time reference starts after the sample is rinsed in the solution
            break;
        case 6:
            if(millis()-dip_time_reference>=dip_duration[cycle_index]*1000)
            {   
                stepper_speed =dry_speed[cycle_index];
                dipPoseLock=1;
                setTargetPose(dip_distance[cycle_index]-dry_distance[cycle_index],1);
                operation_Status=drying_status;
                dip_time_reference=millis(); //time reference starts right after it leaves the solution
                dip_step++;
            }
            break;
        case 7:
             stepper_speed =dry_speed[cycle_index];
            if(!isTargeting){dip_step++;}
            break;
        case 8:
            if(millis()-dip_time_reference>=dry_duration[cycle_index]*1000)
            {
                dip_counts++;
                if(dip_counts>=dip_cycles[cycle_index]){
                dip_step++;
                }else{                
                dip_step=2;}
            }
            break;
        case 9:
            if(cycle_index>=configured_cycles){
            home();
            isOperational=0;
            operation_Status=finished_status;
            play_audio(audio_finished);
            }else
            {
            cycle_index++;
            play_audio(audio_cycle);
            }
            dipPoseLock=0;
            dip_step=0;
            dip_counts=0;
            break;
        }
    }
}

//Before da button n Page Stuff, we gotta memorize the last Index or the Last Page stuff;
int previous_page_index;
int previous_element_index;

void performButtonAction()
{
    if(armButton && !isHoming){
    
    if(audioConflict){play_audio(audio_button_push);}

    armButton=0;

    switch(page_index)
    {
        case 0: //page index here >:(

    //menu_elements[menu_pages][menu_element_counts] = {{"Load:","Dip count:","Dip duration:","Dry duration:","Solution:","Confirm"},{"Dip speed:","Dry speed:","Dip depth:","Dry height:","reset","return"}}; 
            switch(element_index)
            {

                case 0:  //Use the code below as a snippet for how this stuff woooks
                if(isModifying){
                    play_audio(audio_value_edit);

                if(!isPreset){
                dip_speed[cycle_index]    =                profile_dip_speed[preset_index[cycle_index]];
                dry_speed[cycle_index]    =                profile_dry_speed[preset_index[cycle_index]];
                dip_cycles[cycle_index]   =                profile_dip_cycles[preset_index[cycle_index]];
                dip_duration[cycle_index] =                profile_dip_duration[preset_index[cycle_index]];
                dry_duration[cycle_index] =                profile_dry_duration[preset_index[cycle_index]];
                if(operation_Status == finished_status){
                dip_distance[cycle_index] =                profile_dip_distance[preset_index[cycle_index]];
                dry_distance[cycle_index] =                profile_dry_distance[preset_index[cycle_index]];
                solution_pose[cycle_index]=                profile_solution_pose[preset_index[cycle_index]];}
                }

                    isModifying=0;
                    encoderActionMode=encoder_scroll_action;
                }else{ //What IF, Man's not modifying >:(
                    isPreset?   encoderBuffer=profile:   encoderBuffer=preset_index[cycle_index]; //Pooot Hee the intended Value to modify, yk, for scientific stuff or something dunno.}
                    encoderBuffer_lower_constrain = 0; encoderBuffer_upper_constrain = 5;
                    encoderActionMode=encoder_value_action;
                    isModifying=1;
                }
                break;


                case 1:
                if(isModifying){
                    play_audio(audio_value_edit);
                    isModifying=0;
                    encoderActionMode=encoder_scroll_action;
                }else{ //What IF, Man's not modifying >:(
                    isPreset?   encoderBuffer=profile_dip_cycles[profile]    :   encoderBuffer=dip_cycles[cycle_index];
                    encoderBuffer_lower_constrain = 1; encoderBuffer_upper_constrain = 100;
                    encoderActionMode=encoder_value_action;
                    isModifying=1;
                }
                break;


                case 2:
                if(isModifying){
                    play_audio(audio_value_edit);
                    isModifying=0;
                    encoderActionMode=encoder_scroll_action;
                }else{ //What IF, Man's not modifying >:(
                    isPreset?   encoderBuffer=profile_dip_duration[profile]    :   encoderBuffer=dip_duration[cycle_index]; 
                    encoderBuffer_lower_constrain = 0; encoderBuffer_upper_constrain = 900; //900 corresponds to 10 minutes ig
                    encoderActionMode=encoder_value_action;
                    isModifying=1;
                }
                break;


                case 3:
                if(isModifying){
                    play_audio(audio_value_edit);
                    isModifying=0;
                    encoderActionMode=encoder_scroll_action;
                }else{ //What IF, Man's not modifying >:(
                    isPreset?   encoderBuffer=profile_dry_duration[profile]    :   encoderBuffer=dry_duration[cycle_index]; 
                    encoderBuffer_lower_constrain = 0; encoderBuffer_upper_constrain = 900;
                    encoderActionMode=encoder_value_action;
                    isModifying=1;
                }
                break;

//menu_elements[menu_pages][menu_element_counts] = {{"Load:","Dip count:","Dip duration:","Dry duration:","Solution:","Confirm"},{"Dip speed:","Dry speed:","Dip depth:","Dry height:","reset","return"}}; 
                case 4:
                if(isModifying){
                    play_audio(audio_value_edit);
                    isModifying=0;
                    encoderActionMode=encoder_scroll_action;
                }else{ //What IF, Man's not modifying >:(
                    if(operation_Status == finished_status || isPreset){
                    isPreset?   encoderBuffer=profile_solution_pose[profile]    :   encoderBuffer=solution_pose[cycle_index];
                    encoderBuffer_lower_constrain = 0; encoderBuffer_upper_constrain = 5;
                    encoderActionMode=encoder_value_action;
                    isModifying=1;}else
                    {
                        play_audio(audio_fail);
                    }
                }
                break;


                case 5:
                page_index=previous_page_index;
                isPreset? element_index=0 :  element_index=previous_element_index;
                if(isPreset){pushToMemory();}
                break;

            }
        break;


        case 1: //2nd page of ta menu

            switch(element_index)
            {
//{"Dip speed:","Dry speed:","Dip depth:","Dry height:","reset","return"}}; 
                case 0:
                if(isModifying){
                    play_audio(audio_value_edit);
                    isModifying=0;
                    encoderActionMode=encoder_scroll_action;
                }else{ //What IF, Man's not modifying >:(
                    isPreset?   encoderBuffer=profile_dip_speed[profile]    :   encoderBuffer=dip_speed[cycle_index];
                    encoderBuffer_lower_constrain = 1; encoderBuffer_upper_constrain = 40; //max 1.2cm/s min = 1mm/s
                    encoderActionMode=encoder_value_action;
                    isModifying=1;
                }
                break;


                case 1:
                if(isModifying){
                    play_audio(audio_value_edit);
                    isModifying=0;
                    encoderActionMode=encoder_scroll_action;
                }else{ //What IF, Man's not modifying >:(
                    isPreset?   encoderBuffer=profile_dry_speed[profile]    :   encoderBuffer=dry_speed[cycle_index];
                    encoderBuffer_lower_constrain = 1; encoderBuffer_upper_constrain = 40;
                    encoderActionMode=encoder_value_action;
                    isModifying=1;
                }
                break;


                case 2:
                if(isModifying){
                    play_audio(audio_value_edit);
                    if(dip_distance[cycle_index]<dry_distance[cycle_index]+10){dip_distance[cycle_index]=dry_distance[cycle_index]+10;play_audio(audio_fail);}
                    isModifying=0;
                    encoderActionMode=encoder_scroll_action;
                }else{ //What IF, Man's not modifying >:(
                    if(operation_Status == finished_status || isPreset){
                    isPreset?   encoderBuffer=profile_dip_distance[profile]    :   encoderBuffer=dip_distance[cycle_index];
                    encoderBuffer_lower_constrain = dry_distance[cycle_index]+10; encoderBuffer_upper_constrain = 310;
                    encoderActionMode=encoder_value_action;
                    isModifying=1;}
                    else{play_audio(audio_fail);}
                }
                break;


                case 3:
                if(isModifying){
                    play_audio(audio_value_edit);
                    if(dry_distance[cycle_index]>dip_distance[cycle_index]-10){dry_distance[cycle_index]=dip_distance[cycle_index]-10;play_audio(audio_fail);}
                    isModifying=0;
                    encoderActionMode=encoder_scroll_action;
                }else{ //What IF, Man's not modifying >:(
                    if(operation_Status == finished_status || isPreset){
                    isPreset?   encoderBuffer=profile_dry_distance[profile]    :   encoderBuffer=dry_distance[cycle_index];
                    encoderBuffer_lower_constrain = 10; encoderBuffer_upper_constrain = dip_distance[cycle_index]-10; //verify the behaviour of this thing, it requires more than just constrains to make it work in operation 
                    //make it throw an error, and make it dependent on the value of the dipping distance (negative feedback)
                    //add an icon for this (Fail icon);
                    encoderActionMode=encoder_value_action;
                    isModifying=1;
                    }else
                    {
                        play_audio(audio_fail);
                    }
                }
                break;

//{"Dip speed:","Dry speed:","Dip depth:","Dry height:","reset","return"}}; 
                case 4:
                if(isPreset)
                {
                profile_dip_speed[profile]=5;
                profile_dry_speed[profile]=5;
                profile_dip_duration[profile]=5;
                profile_dry_duration[profile]=5;
                profile_dip_cycles[profile]=1;
                profile_dip_distance[profile]=200;
                profile_dry_distance[profile]=150;
                profile_solution_pose[profile]=0;
                }else{
                dip_speed[cycle_index]=5;
                dry_speed[cycle_index]=5;
                dip_duration[cycle_index]=5;
                dry_duration[cycle_index]=5;
                dip_cycles[cycle_index]=1;
                if(operation_Status == finished_status){
                dip_distance[cycle_index]=200;
                dry_distance[cycle_index]=150;
                solution_pose[cycle_index]=0;}
                }
                break;

                case 5:
                page_index=previous_page_index;
                isPreset? element_index=0 :  element_index=previous_element_index;
                if(isPreset){pushToMemory();}
                break;
            }
        break;


    //main page stuff;
    case 100:
        switch (element_index)
        {
        case 0: //takes us to the global settings page;
        page_index=103;
        element_index=0;
        break;

        case 1: //this is to turn off I guess ?
        oled.clearBuffer();
        oled.setBitmapMode(1);
        oled.drawXBMP(1,1,128,64,ASCE);
        oled.sendBuffer();
        blink(4096);
        delay(200);
        play_audio(audio_shut_down);
        if(audio_mode==all){
        while(isSounding)
        {
        performAudioFeedback();
        }}
        blink(0);
        oled.clearBuffer();
        oled.sendBuffer();
        detachInterrupt(EncoderButton);
        gpio_wakeup_enable(GPIO_NUM_23,GPIO_INTR_LOW_LEVEL); 
        esp_sleep_enable_gpio_wakeup();
        digitalWrite(Step,0); digitalWrite(STPDir,0);
        esp_light_sleep_start();//we'll just go for light sleep to avoid using the ULP co processor and doing some assembly stuff that may increase the cost of development for the client for no reason.
        esp_restart();
        break;

        case 2: //This one does the magic
        page_index=101;
        element_index=100;
        break;
        }
    break;


    //SelectionPageStuff;
    case 101:
        switch (element_index)
        {
        case 100:
        if(configured_cycles!=configurable_cycles){
        configured_cycles++;}
        break;
        
        case 101: 
        if(configured_cycles!=0){
        configured_cycles--;}
        break;

        case 102:
        isOperational=1;cycle_index=0;dip_step=0;dip_counts=0;
        element_index=0;
        page_index=102;
        break;

        case 103:
        page_index=100;
        element_index=0;
        configured_cycles=0;
        break;
        
            default:
            if(element_index>=0 && element_index<= configured_cycles){
            cycle_index=element_index; previous_element_index = element_index;
            element_index=0;
            previous_page_index=page_index; page_index=0;
            isPreset=0;
            }
            break;
        } 
    break;


    //progress page Stuff;
    case 102:
        switch (element_index)
        {
        case 1:
        operation_Status=finished_status;home();isOperational=0;cycle_index=0;dip_step=0;dip_counts=0;page_index=101;element_index=100; //double checkin this stuff man, clock cycles and flash memory is givin yk
        break;

        case 0:
         //we're gonna have to implement a last page index buffer, and a last element index buffer, for accessibility and stuff
         previous_element_index = element_index;
         previous_page_index = page_index;
         page_index=0; isPreset=0;

        break;

        }
    break;

    //settings stuff
//settings_elements[7]={"Presets","Sounds:","µStepping:","Servo Speed:","Ease mode:","Toggle WiFi","return"};
    case 103:
        switch (element_index)
        {
        case 0:
        previous_page_index=page_index; page_index=0;
        isPreset=1;
        break;

        case 1:
                if(isModifying){
                    play_audio(audio_value_edit);
                    isModifying=0;
                    pushToMemory();
                    encoderActionMode=encoder_scroll_action;
                }else{ //What IF, Man's not modifying >:(
                    encoderBuffer=audio_mode;
                    encoderBuffer_lower_constrain = 0; encoderBuffer_upper_constrain = 3;
                    encoderActionMode=encoder_value_action;
                    isModifying=1;
                }
        break;

        case 2:
                if(isModifying){
                    play_audio(audio_value_edit);
                    isModifying=0;
                    pushToMemory();
                    encoderActionMode=encoder_scroll_action;
                }else{ //What IF, Man's not modifying >:(
                    encoderBuffer=microSteppingValue;
                    encoderBuffer_lower_constrain = 0; encoderBuffer_upper_constrain = 5;
                    encoderActionMode=encoder_value_action;
                    isModifying=1;
                }
        break;

        case 3:
                if(isModifying){
                    play_audio(audio_value_edit);
                    isModifying=0;
                    pushToMemory();
                    encoderActionMode=encoder_scroll_action;
                }else{ //What IF, Man's not modifying >:(
                    encoderBuffer=servoSpeed;
                    encoderBuffer_lower_constrain = 0; encoderBuffer_upper_constrain = 180;
                    encoderActionMode=encoder_value_action;
                    isModifying=1;
                }
        break;

        case 4:
                if(isModifying){
                    play_audio(audio_value_edit);
                    isModifying=0;
                    pushToMemory();
                    encoderActionMode=encoder_scroll_action;
                }else{ //What IF, Man's not modifying >:(
                    encoderBuffer=easeMode;
                    encoderBuffer_lower_constrain = 0; encoderBuffer_upper_constrain = 2;
                    encoderActionMode=encoder_value_action;
                    isModifying=1;
                }
        break;

        case 5:
        WiFistate ? WiFistate=0 : WiFistate=1; 
        pushToMemory();
        esp_restart();
        break;

        case 6:
        page_index=100;element_index=0;
        break;        
        }

    break;
    }


    }
}


void performValueAction()
{
    //lez see heer ig
    if(encoderActionMode==encoder_value_action)
    {
        switch (page_index)
        {
        //menu Page Stuff
        //{{"Load:","Dip count:","Dip duration:","Dry duration:","Solution:","Confirm"},{"Dip speed:","Dry speed:","Dip depth:","Dry height:","reset","return"}}; 
        case 0:
            switch (element_index)
            {
            case 0:
                isPreset? profile=encoderBuffer : preset_index[cycle_index]=encoderBuffer ;
            break;

            case 1:
                isPreset? profile_dip_cycles[profile]=encoderBuffer : dip_cycles[cycle_index]=encoderBuffer ;
            break;

            case 2:
                isPreset? profile_dip_duration[profile]=encoderBuffer : dip_duration[cycle_index]=encoderBuffer ;
            break;

            case 3:
                isPreset? profile_dry_duration[profile]=encoderBuffer : dry_duration[cycle_index]=encoderBuffer ;
            break;

            case 4:
            if(operation_Status == finished_status){
                isPreset? profile_solution_pose[profile]=encoderBuffer : solution_pose[cycle_index]=encoderBuffer ;} else {play_audio(audio_fail);encoderActionMode=encoder_scroll_action;}
            break;

            }
            break;

        
        case 1:
        switch (element_index)
            {
            case 0:
                isPreset? profile_dip_speed[profile]=encoderBuffer : dip_speed[cycle_index]=encoderBuffer ;
            break;

            case 1:
                isPreset? profile_dry_speed[profile]=encoderBuffer : dry_speed[cycle_index]=encoderBuffer ;
            break;

            case 2:
                if(operation_Status == finished_status){
                isPreset? profile_dip_distance[profile]=encoderBuffer : dip_distance[cycle_index]=encoderBuffer ;}else{play_audio(audio_fail);isModifying=0;encoderActionMode=encoder_scroll_action;}
            break;

            case 3:
            if(operation_Status == finished_status){
                isPreset? profile_dry_distance[profile]=encoderBuffer : dry_distance[cycle_index]=encoderBuffer ;}else{play_audio(audio_fail); isModifying=0;encoderActionMode=encoder_scroll_action;}
            break;
            
        }
        break;
       
       //Rest of Pages Stuff

       case 103:
        switch (element_index)
        {
        case 1:
        audio_mode=encoderBuffer;
        break;
        case 2:
        microSteppingValue=encoderBuffer;
        break;
        case 3:
        servoSpeed=encoderBuffer;
        break;
        case 4:
        easeMode=encoderBuffer;
        break;
        }
       break;
    }
    }
}

void setup()
{
    setCpuFrequencyMhz(240);


    //Serial Dayz
    // Serial.begin(115200);
    // Serial.println("Alive for now at: ");

    //defaulting
    armValues();
    
    //Allowing the allocations of timers for the servo library for some reason
	ESP32PWM::allocateTimer(3);


    //Beginnin el servo Action
    servo.setPeriodHertz(50);
    servo.attach(Servo_Pin,500,2400);


    //Display initials
    oled.begin();
    oled.setFont(textFont);

    oled.clear();
    oled.sendBuffer();
    oled.setBitmapMode(1);
    oled.drawXBMP(0,0,128,64,ASCE);
    oled.sendBuffer();
    //boot stuff

    //Coms Stuff
    if(WiFistate){

    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP(ssid,password,2,0,4);

    webSocket.begin(); //initiate the websocket only when there's connectivity, save resources otherwise.
    webSocket.onEvent(socketEvent);
    oled.clear();
    oled.print("Connectivity Enabled\n at 1.1.1.1:80\n use HTTP entries:\n-Defaults\nto reset values.");
    oled.sendBuffer();
    delay(600);
    }

    //Port Direction
    pinMode(Step,OUTPUT);
    pinMode(STPDir,OUTPUT);
    pinMode(STPm1,OUTPUT);
    pinMode(STPm2,OUTPUT);
    pinMode(STPm3,OUTPUT);
    pinMode(EndStop,INPUT_PULLDOWN);
    pinMode(Buzzer_Pin,OUTPUT);
    pinMode(LED_Indicator,OUTPUT);
    pinMode(EncoderClk,INPUT_PULLDOWN);
    pinMode(EncoderData,INPUT_PULLDOWN);
    pinMode(EncoderButton,INPUT_PULLUP);
    
    //Interrupts
    attachInterrupt(EncoderClk,encoderClockISR,RISING);
    attachInterrupt(EncoderButton,buttonAction,RISING);

    //Timers
    encoderAction_Handle =timerBegin(0, 240, true);
    timerAttachInterrupt(encoderAction_Handle, &retrieveEncoderAction, true);
    timerAlarmWrite(encoderAction_Handle, 50000, true); //this timer counts @ resolution of 250ms, or 0.25s, it is used to fetch encoder action and count time.
    timerAlarmEnable(encoderAction_Handle);

    stepperAction_Handle =timerBegin(1, 240, true);
    timerAttachInterrupt(stepperAction_Handle, &step, true);
    timerAlarmWrite(stepperAction_Handle, 10, true); //this timer counts @ resolution of 250ms, or 0.25s, it is used to fetch encoder action and count time.
    timerAlarmEnable(stepperAction_Handle);

    //Connectivity indication (onboard LED)
    ledcSetup(1,10000,12);
    ledcAttachPin(LED_Indicator,1);
    blink(4096);
    //Audio and indication
    ledcSetup(0,10000,12);
    ledcAttachPin(Buzzer_Pin,0);
    play_audio(audio_boot);
    if(audio_mode==all){
    while(isSounding)
    {
    performAudioFeedback();
    }}
    blink(0);
    
    //defaults
    setMicrosteps(microSteppingValue);
    home();
    oled.enableUTF8Print();
}



void loop()
{   
    if(WiFistate)
    {
    webSocket.loop();
    } //run callback on any event
    performBlink();
    performAudioFeedback();
    servoAction();
    checkHome();
    performButtonAction();
    performValueAction();
    checkPose();
    dipCoat();
    renderScreen();
}
