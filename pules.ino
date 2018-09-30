#define PROCESSING_VISUALIZER 1//定义类型，类似等号，PROCESSING_VISUALIZER就是1
#define SERIAL_PLOTTER  2//同上
#include "U8glib.h"
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);  // I2C 接口1306显示屏
#include <SoftwareSerial.h>//调用软串口库
SoftwareSerial BTSerial(11, 10);//软串口进行无线通信
int pulsePin = 1;                 // 心率模块连接到A0口
int blinkPin = 5;                // 用板载led（13号口）开关量模拟心跳
int fadePin = 13;                  // 用5号口的pwm信号模拟心跳
int fadeRate = 0;                 // 通过pwm信号进行淡出的效果
volatile int BPM;                   // 心率
volatile int Signal;                // 保持前一个输入数数据
volatile int IBI = 600;             // 采样的时间隔
volatile boolean Pulse = false;     // 判断是否为心跳的标志，true为有，false为没有
volatile boolean QS = false;        // 判断心跳拍子有效的标志



void draw() {
  
//显示
  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(0,10);            //起始像素点
  u8g.print("BPM =");u8g.print(BPM);u8g.print("/min");//显示心率相关信息
   u8g.setPrintPos(0,22);
  u8g.print("IBI = ");u8g.print(IBI);  u8g.print("ms");//显示心跳的时间间隔
}

static int outputType = PROCESSING_VISUALIZER;// 串口输出类型选择
                                              // PROCESSING_VISUALIZER 用于PROCESSING上位机
                                              // SERIAL_PLOTTER 用于arduino的串口示波器
void setup() {
  BTSerial.begin(115200);
  pinMode(blinkPin, OUTPUT);        // 用直接闪烁的形式进行心跳的模拟
  pinMode(fadePin, OUTPUT);         // 用淡入淡出的形式进行心跳的模拟
  Serial.begin(115200);             // 串口传输时上位机的波特率
  interruptSetup();                 // 保持2ms的采样间隔
}
void loop() {

  serialOutput();//串口输出子程序；可以选择不同的输出类型
  //send_bt_Serial();//蓝牙发送数据子程序
  if (QS == true) {    // 发现一个心跳拍
    // 确定有节拍
    // 将节拍标志确定
    fadeRate = 255;         // 打开pwm的衰减信号并从255开始衰减                             /
    serialOutputWhenBeatHappens();   // 将拍子信号输出到串口
    QS = false;                      // 将标志初始化
 u8g.firstPage();  
 do {
    draw();
 } while( 
    u8g.nextPage()
   ); 
  }
  ledFadeToBeat();                      // led开始衰减变化
  delay(20);                             //延时
}                                       //主程序结束

void ledFadeToBeat()//pwm控制的led灯输出子程序
{
  fadeRate -= 15;                         //  设置每次衰减的大小（15个单位）
  fadeRate = constrain(fadeRate, 0, 255); //  将输出的大小约束再0到255之间
  analogWrite(fadePin, fadeRate);         //  将当前值输出到用pwm的led表示心跳的引脚上
}



void serialOutput() // 此子程序可以选择输出类型
{  
  switch (outputType) {
    case PROCESSING_VISUALIZER:
      sendDataToSerial('S', Signal);     //直接输出给PROCESSING上位机，子程序在下面
      break;
    case SERIAL_PLOTTER:  //输出到arduino的串口绘图仪
      Serial.print(BPM);
      Serial.print(",");
      Serial.print(IBI);
      Serial.print(",");
      Serial.println(Signal);
      break;
    default:
      break;
  }
}

// 决定如何输出BPM和IBI数据的子程序
void serialOutputWhenBeatHappens() {
  switch (outputType) {
    case PROCESSING_VISUALIZER:    // find it here https://github.com/WorldFamousElectronics/PulseSensor_Amped_Processing_Visualizer
      sendDataToSerial('B', BPM);  // send heart rate with a 'B' prefix
      sendDataToSerial('Q', IBI);  // send time between beats with a 'Q' prefix
      break;

    default:
      break;
  }
}


//  将数据发送至上位机
void sendDataToSerial(char symbol, int data )//设定输出类型
{ BTSerial.print(symbol);
  BTSerial.println(data);
  Serial.print(symbol);
   Serial.println(data);
}

//以下是心率算法，用来计算BPM IBI Signal等

volatile int rate[10];                    // array to hold last ten IBI values
volatile unsigned long sampleCounter = 0;          // used to determine pulse timing
volatile unsigned long lastBeatTime = 0;           // used to find IBI
volatile int P = 512;                     // used to find peak in pulse wave, seeded
volatile int T = 512;                     // used to find trough in pulse wave, seeded
volatile int thresh = 530;                // used to find instant moment of heart beat, seeded
volatile int amp = 0;                   // used to hold amplitude of pulse waveform, seeded
volatile boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
volatile boolean secondBeat = false;      // used to seed rate array so we startup with reasonable BPM
void interruptSetup() 
{ // CHECK OUT THE Timer_Interrupt_Notes TAB FOR MORE ON INTERRUPTS
  // Initializes Timer2 to throw an interrupt every 2mS.
  TCCR2A = 0x02;     // DISABLE PWM ON DIGITAL PINS 3 AND 11, AND GO INTO CTC MODE
  TCCR2B = 0x06;     // DON'T FORCE COMPARE, 256 PRESCALER
  OCR2A = 0X7C;      // SET THE TOP OF THE COUNT TO 124 FOR 500Hz SAMPLE RATE
  TIMSK2 = 0x02;     // ENABLE INTERRUPT ON MATCH BETWEEN TIMER2 AND OCR2A
  sei();             // MAKE SURE GLOBAL INTERRUPTS ARE ENABLED
}


// THIS IS THE TIMER 2 INTERRUPT SERVICE ROUTINE.
// Timer 2 makes sure that we take a reading every 2 miliseconds
ISR(TIMER2_COMPA_vect) {                        // triggered when Timer2 counts to 124
  cli();                                      // disable interrupts while we do this
  Signal = analogRead(pulsePin);              // 读取传感器接口的模拟量值
  sampleCounter += 2;                         // keep track of the time in mS with this variable
  int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise

  //  find the peak and trough of the pulse wave
  if (Signal < thresh && N > (IBI / 5) * 3) { // avoid dichrotic noise by waiting 3/5 of last IBI
    if (Signal < T) {                       // T is the trough
      T = Signal;                         // keep track of lowest point in pulse wave
    }
  }

  if (Signal > thresh && Signal > P) {        // thresh condition helps avoid noise
    P = Signal;                             // P is the peak
  }                                        // keep track of highest point in pulse wave

  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
  if (N > 250) {                                  // avoid high frequency noise
    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI / 5) * 3) ) {
      Pulse = true;                               // set the Pulse flag when we think there is a pulse
      digitalWrite(blinkPin, HIGH);               // turn on pin 13 LED
      IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
      lastBeatTime = sampleCounter;               // keep track of time for next pulse

      if (secondBeat) {                      // if this is the second beat, if secondBeat == TRUE
        secondBeat = false;                  // clear secondBeat flag
        for (int i = 0; i <= 9; i++) {       // seed the running total to get a realisitic BPM at startup
          rate[i] = IBI;
        }
      }

      if (firstBeat) {                       // if it's the first time we found a beat, if firstBeat == TRUE
        firstBeat = false;                   // clear firstBeat flag
        secondBeat = true;                   // set the second beat flag
        sei();                               // enable interrupts again
        return;                              // IBI value is unreliable so discard it
      }


      // keep a running total of the last 10 IBI values
      word runningTotal = 0;                  // clear the runningTotal variable

      for (int i = 0; i <= 8; i++) {          // shift data in the rate array
        rate[i] = rate[i + 1];                // and drop the oldest IBI value
        runningTotal += rate[i];              // add up the 9 oldest IBI values
      }

      rate[9] = IBI;                          // add the latest IBI to the rate array
      runningTotal += rate[9];                // add the latest IBI to runningTotal
      runningTotal /= 10;                     // average the last 10 IBI values
      BPM = 60000 / runningTotal;             // how many beats can fit into a minute? that's BPM!
      QS = true;                              // set Quantified Self flag
      // QS FLAG IS NOT CLEARED INSIDE THIS ISR
    }
  }

  if (Signal < thresh && Pulse == true) {  // when the values are going down, the beat is over
    digitalWrite(blinkPin, LOW);           // turn off pin 13 LED
    Pulse = false;                         // reset the Pulse flag so we can do it again
    amp = P - T;                           // get amplitude of the pulse wave
    thresh = amp / 2 + T;                  // set thresh at 50% of the amplitude
    P = thresh;                            // reset these for next time
    T = thresh;
  }

  if (N > 2500) {                          // if 2.5 seconds go by without a beat
    thresh = 530;                          // set thresh default
    P = 512;                               // set P default
    T = 512;                               // set T default
    lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date
    firstBeat = true;                      // set these to avoid noise
    secondBeat = false;                    // when we get the heartbeat back
  }
  sei();                                   // enable interrupts when youre done!
}
