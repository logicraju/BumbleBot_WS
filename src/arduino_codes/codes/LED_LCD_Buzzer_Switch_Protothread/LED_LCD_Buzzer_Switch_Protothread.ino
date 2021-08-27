//Macros
#define led_red_pin 5
#define led_yellow_pin 7
#define led_green_pin 6
#define push_button_pin 12
#define buzzer_pin 2

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0

#include <ros.h>
#include <pt.h> //Protothread library
#include <Wire.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <LiquidCrystal_I2C.h>

//ROS NodeHandle
ros::NodeHandle nh;

//ROS Message Variables
std_msgs::String str_msg;
std_msgs::Bool push_button_msg;

//Global Variables
static struct pt pt1, pt2, pt3; // each protothread needs one of these
LiquidCrystal_I2C lcd(0x27, 16, 2);
int push_button_value = 0;
long publisher_timer_push_button, timer_led, timer_buzzer;
bool led_blink_flag = false, led_toggle_flag = false;
String led_color = "", previous_lcd_msg = "";


//PASSIVE BUZZER - PLAY CHRISTMAS SONG
void play_christmas_song() 
{
  int tempo = 200; // change this to make the song slower or faster

  int melody[] = {
    NOTE_C5,4,
    NOTE_F5,4, NOTE_F5,8, NOTE_G5,8, NOTE_F5,8, NOTE_E5,8,
    NOTE_D5,4, NOTE_D5,4, NOTE_D5,4,
    NOTE_G5,4, NOTE_G5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F5,8,
    NOTE_E5,4, NOTE_C5,4, NOTE_C5,4,
    NOTE_A5,4, NOTE_A5,8, NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8,
    NOTE_F5,4, NOTE_D5,4, NOTE_C5,8, NOTE_C5,8,
    NOTE_D5,4, NOTE_G5,4, NOTE_E5,4,
    NOTE_F5,2
  };

  int notes = sizeof(melody) / sizeof(melody[0]) / 2;
  int wholenote = (60000 * 4) / tempo;
  int divider = 0, noteDuration = 0;
  
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
    divider = melody[thisNote + 1];
    if (divider > 0) 
  {
      noteDuration = (wholenote) / divider;
    } 
  else if (divider < 0) 
  {
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5;
    }
    tone(buzzer_pin, melody[thisNote], noteDuration * 0.9);
    delay(noteDuration);
    noTone(buzzer_pin);
  }
}

//DISPLAY IN LCD
void display_lcd(String message)
{
  String string1, string2;
  if(!previous_lcd_msg.equals(message))
  {
    if(message.length() > 16)
    {
      string1 = message.substring(0,16);
      string2 = message.substring(16,32);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(string1);
      lcd.setCursor(0,1);
      lcd.print(string2);
    }
    else
    {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(message);
    }
    previous_lcd_msg = message;
  }
}

//BLINK LED
void blink_led()
{
  int pin_value;
  if(led_blink_flag)
  {
    if(led_toggle_flag)
    {
      pin_value = HIGH;
    }
    else
    {
      pin_value = LOW;
    }
    led_toggle_flag = !led_toggle_flag;
    if(led_color == "red")
    {
      digitalWrite(led_yellow_pin, LOW);
      digitalWrite(led_green_pin, LOW);
      digitalWrite(led_red_pin, pin_value);
    }
    if(led_color == "yellow")
    {
      digitalWrite(led_green_pin, LOW);
      digitalWrite(led_red_pin, LOW);
      digitalWrite(led_yellow_pin, pin_value);
    }
    if(led_color == "green")
    {
      digitalWrite(led_red_pin, LOW);
      digitalWrite(led_yellow_pin, LOW);
      digitalWrite(led_green_pin, pin_value);
    }
    if(led_color == "all")
    {
      digitalWrite(led_red_pin, pin_value);
      digitalWrite(led_yellow_pin, pin_value);
      digitalWrite(led_green_pin, pin_value);
    }
    if(led_color == "none")
    {
      digitalWrite(led_red_pin, LOW);
      digitalWrite(led_yellow_pin, LOW);
      digitalWrite(led_green_pin, LOW);
    }
  }
}

//Subscriber Callback Function - Flash Light
void buzzer_callback(const std_msgs::Bool& buzzer_msg)
{
  if(buzzer_msg.data)
  {
    play_christmas_song();
  }
}

//Subscriber Callback Function - LEDs
void led_callback(const std_msgs::String& led_msg)
{
  if(led_msg.data == "black")
  {
    led_blink_flag = false;
  }
  else
  {
    led_color = led_msg.data;
    led_blink_flag = true;
  }
}

//Subscriber Callback Function - LCD
void lcd_callback(const std_msgs::String& lcd_msg)
{
  display_lcd(lcd_msg.data);
}

//ROS Variables
ros::Subscriber<std_msgs::String> led_subscriber("led", led_callback);
ros::Subscriber<std_msgs::Bool> buzzer_subscriber("buzzer", buzzer_callback);
ros::Subscriber<std_msgs::String> lcd_subscriber("lcd", lcd_callback);
ros::Publisher push_button_publisher("push_button", &push_button_msg);

//Publisher - Push Button
void push_button_read()
{
  bool data = false;
  push_button_value = digitalRead(push_button_pin);
  if (push_button_value == HIGH) 
  {
    //digitalWrite(led_green_pin, HIGH);
    data = true;
  } 
  else 
  {
    //digitalWrite(led_green_pin, LOW);
    data = false;
  }
  push_button_msg.data = data;
  push_button_publisher.publish( &push_button_msg );
}

//Constructor
void setup()
{
  //Debug
  Serial.begin(57600);
  //LCD
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  display_lcd("I am Bumblebot  Hello :)");

  //LEDs
  pinMode(led_red_pin, OUTPUT);
  pinMode(led_yellow_pin, OUTPUT);
  pinMode(led_green_pin, OUTPUT);
  pinMode(buzzer_pin, OUTPUT);
  pinMode(push_button_pin, INPUT);

  //ROS NodeHandle
  nh.initNode();
  
  //ROS Publishers and Subscribers
  nh.subscribe(led_subscriber);
  nh.subscribe(buzzer_subscriber);
  nh.subscribe(lcd_subscriber);
  nh.advertise(push_button_publisher);

  //ProtoThread Variables
  //---------------------
  PT_INIT(&pt1);  // initialise protothread variable
  PT_INIT(&pt2);  // initialise protothread variable
  PT_INIT(&pt3);  // initialise protothread variable
}

//ProtoThread 1 - LED
//-------------------
static int protothread1(struct pt *pt, int interval) 
{
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(1) 
  {
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis(); // take a new timestamp
    
    //Write operations here
    if (millis() > timer_led)
    {
      blink_led();
      timer_led = millis() + 100; //10 times per second
    }
  }
  PT_END(pt);
}

//ProtoThread 2 - Push Button Read
//--------------------------------
static int protothread2(struct pt *pt, int interval) 
{
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(1) 
  {
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis(); // take a new timestamp
    
    //Write operations here
    if (millis() > publisher_timer_push_button)
    {
      push_button_read();
      publisher_timer_push_button = millis() + 100; //publish ten times per second
    }
  }
  PT_END(pt);
}

//ProtoThread 3 - Buzzer
//----------------------
static int protothread3(struct pt *pt, int interval) 
{
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(1) 
  {
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis(); // take a new timestamp
    
    //Write operations here
    if (millis() > timer_buzzer)
    {
      play_christmas_song();
      timer_buzzer = millis() + 500; //two times per second
    }
  }
  PT_END(pt);
}

//Main
void loop()
{
  //Schedule the protothreads by calling them infinitely
  protothread1(&pt1, 5);  //LED Thread - Execute for 5ms
  protothread2(&pt2, 5);  //Push Button Thread - Execute for 5ms
  //protothread3(&pt3, 5);  //Buzzer Thread - Execute for 5ms
  nh.spinOnce();//Waiting
}
