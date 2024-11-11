#include <math.h>
#include <soc/gpio_struct.h>
#define PHASE_SIZE 120 // 正弦波の解像度
#define TIMER_SIZE 2000 // 正弦波の周波数*10の値。タイマーの割り込み周波数変更に使う。

const uint8_t Pin_UH = 27; // #1正弦波pwm用のピン
const uint8_t Pin_UL = 19;
const uint8_t Pin_VH = 14; // #2
const uint8_t Pin_VL = 18;
const uint8_t Pin_WH = 12; // #3
const uint8_t Pin_WL = 5;
const uint8_t Pin_ctl = 32;

int triValue[PHASE_SIZE]; // 比較用三角波配列(非同期モードで使用)
int sinValuesU_H[PHASE_SIZE]; // 三相の正弦波用配列の宣言
int sinValuesU_L[PHASE_SIZE];
int sinValuesV_H[PHASE_SIZE];
int sinValuesV_L[PHASE_SIZE];
int sinValuesW_H[PHASE_SIZE];
int sinValuesW_L[PHASE_SIZE];

int triPhase_three[PHASE_SIZE]; // 3パルス時の正弦波の位相に対応した三角波の位相
int triPhase_five[PHASE_SIZE]; // 5パルス時の正弦波に対応した三角波の位相
int triPhase_nine[PHASE_SIZE]; // 9パルス時の正弦波に対応した三角波の位相

int sin_max = 1024,tri_max = 1024;
float speed_max = 200;

int timer_turn1[TIMER_SIZE]; // 回したい周波数の時のタイマーclockの値を格納
int timer_turn2[TIMER_SIZE];
hw_timer_t *timer1 = NULL; // 正弦波の位相を進め、GPIOのHighまたはLowを出力するためのタイマー
hw_timer_t *timer2 = NULL; // 三角波の位相を進めるためのタイマー（非同期）
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
const uint32_t timer_clk1=4000000; // 割り込み用タイマー1の周波数
const uint32_t timer_clk2=10000000; // 割り込み用タイマー1の周波数

volatile int sync_mode = 0; // 回転モード-0:非同期、1 : 1パルス、3 : 3パルス、5 : 5パルス、9 : 9パルス
volatile float speed = 0; // 正弦波の周波数
volatile int sin_phase = 0; // 正弦波の位相(V_H)
volatile int tri_phase = 0; // 三角波の位相

volatile boolean stop = true;

void IRAM_ATTR onTimer1(){
  // 正弦波用　#1
  if (sin_phase >= (PHASE_SIZE-1)){
    sin_phase = 0;}
  else{
    sin_phase++;}

  //if (sync_mode != 0){timerEnd(timer2);}
  if (sync_mode == 1){tri_phase = sin_phase;}
  else if (sync_mode == 3){tri_phase = triPhase_three[sin_phase];}
  else if (sync_mode == 5){tri_phase = triPhase_five[sin_phase];}
  else if (sync_mode == 9){tri_phase = triPhase_nine[sin_phase];}

  SwitchGPIO(sinValuesU_H[sin_phase], triValue[tri_phase], Pin_UH);
  SwitchGPIO(sinValuesU_L[sin_phase], triValue[tri_phase], Pin_UL);
  SwitchGPIO(sinValuesV_H[sin_phase], triValue[tri_phase], Pin_VH);
  SwitchGPIO(sinValuesV_L[sin_phase], triValue[tri_phase], Pin_VL);
  SwitchGPIO(sinValuesW_H[sin_phase], triValue[tri_phase], Pin_WH);
  SwitchGPIO(sinValuesW_L[sin_phase], triValue[tri_phase], Pin_WL);
}

void IRAM_ATTR onTimer2(){ // 非同期モードでのみこのタイマーを使用する。同期モードになったらタイマー停止する処理を書く↓（loop関数に）まだしてない
  // 三角波用
  if (tri_phase >= (PHASE_SIZE-1)){
    tri_phase = 0;}
  else{
    tri_phase++;
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(Pin_UH, OUTPUT);
  pinMode(Pin_UL, OUTPUT);
  pinMode(Pin_VH, OUTPUT);
  pinMode(Pin_VL, OUTPUT);
  pinMode(Pin_WH, OUTPUT);
  pinMode(Pin_WL, OUTPUT);
  // 三角波の値の配列
  for (int i = 0; i < PHASE_SIZE; i++){
    float valuefortri = (float)1.5 - (2.0 / PHASE_SIZE) * i;
    if (valuefortri > 1){valuefortri = (float)2.0 - valuefortri;}
    else if (valuefortri < 0){valuefortri = (float)-valuefortri;}
    triValue[i] = round(valuefortri * tri_max);
    Serial.println(triValue[i]);
  }
  // パルスモード時の正弦波の位相に応じた三角波用の位相配列の作成
  for (int j = 0; j < PHASE_SIZE; j++){
    triPhase_three[j] = (j * 3) % PHASE_SIZE;
    triPhase_five[j] = (j * 5) % PHASE_SIZE;
    triPhase_nine[j] = (j * 9) % PHASE_SIZE;
  }
  // 3相のsinの値用の配列の作成
  for (int x = 0; x < PHASE_SIZE; x++){ //xは0~119
    float s = (360 * x) / PHASE_SIZE;
    float valueU_H = (sin((s - 120)* DEG_TO_RAD) / 2 + 0.5) * sin_max; // 正弦波
    float valueU_L = (sin((s + 60)* DEG_TO_RAD) / 2 + 0.5) * sin_max;
    float valueV_H = (sin((s)* DEG_TO_RAD) / 2 + 0.5) * sin_max;
    float valueV_L = (sin((s + 180)* DEG_TO_RAD) / 2 + 0.5) * sin_max;
    float valueW_H = (sin((s + 120)* DEG_TO_RAD) / 2 + 0.5) * sin_max;
    float valueW_L = (sin((s - 60)* DEG_TO_RAD) / 2 + 0.5) * sin_max;
    sinValuesU_H[x] = round(valueU_H);
    sinValuesU_H[x] = round(valueU_L);
    sinValuesV_H[x] = round(valueV_H);
    sinValuesU_H[x] = round(valueV_L);
    sinValuesW_H[x] = round(valueW_H);
    sinValuesU_H[x] = round(valueW_L);
    Serial.println(round(valueU_H));
  }
  // speedに応じた割り込みクロックの計算と格納
  for (int i=0; i < TIMER_SIZE; i++) {
    timer_turn1[i] = round(float(float(timer_clk1 / PHASE_SIZE) / float(float(i+1)/10.0)));
    Serial.println(round(float(float(timer_clk1 / PHASE_SIZE) / float(float(i+1)/10.0))));
  }
  for (int k=0; k < TIMER_SIZE; k++) {
    timer_turn2[k] = round((float)(timer_clk2 / PHASE_SIZE) / (k+1));
  }
  // タイマーの登録
  timer1 = timerBegin(timer_clk1); // 使用するタイマー周波数
  timerAttachInterrupt(timer1, onTimer1); //割り込み関数の登録
  timer2 = timerBegin(timer_clk2); // 使用するタイマー周波数
  timerAttachInterrupt(timer2, onTimer2); //割り込み関数の登録

  ChangeFreq(800);
  Serial.println("finished successfully");
}

void loop() { //あとはcmdの実装だけ
  int cmd = analogRead(Pin_ctl);
  //Serial.println(cmd);
  if (speed >= -0.09){
    if (cmd >= 1500 && cmd <= 2500){delay(10);} // 定速
    else if (cmd < 500 && speed <= speed_max){speed += 0.1; Serial.println("3+"); delay(20);stop = false;} // 力行
    else if (cmd < 1000 && speed <= speed_max){speed += 0.1; Serial.println("2+"); delay(40);}
    else if (cmd < 1500 && speed <= speed_max){
      if (speed == 0){
        speed = 2;
        timer1 = timerBegin(timer_clk1); // 使用するタイマー周波数
        timerAttachInterrupt(timer1, onTimer1); //割り込み関数の再登録
      }
      speed += 0.1; Serial.println("1+"); delay(80);
      }
    
    else if (cmd > 3500 && speed > 0.01){speed -= 0.1; Serial.println("3-"); delay(20);} // 減速
    else if (cmd > 3000 && speed > 0.01){speed -= 0.1; Serial.println("2-"); delay(40);}
    else if (cmd > 2500 && speed > 0.01){speed -= 0.1; Serial.println("1-"); delay(80);}

    //else if (speed == 0){Serial.println("speed: 0");}
    //else {Serial.println("cmd_Error Out of range");} // その他（設計上起こること自体がおかしいが）

    if (speed < speed_max){keikyu1000(speed);} // 最高速度でPWMの更新が実行されると変な音の原因となる
    else {speed = speed_max;} // 速度の矯正
    Serial.println(speed);
  }
  //if (speed > 0.01 && speed <= 0.2){ReadyPWM();}

  if (stop == false && speed <= 0.001){timerEnd(timer1);}
  if (speed <= 0.001){ StopPWM(); speed = 0; stop = true;} // // 速度の矯正
  else if (speed <= speed_max){stop = false; timerAlarm(timer1,timer_turn1[int(speed*10)] , true, 0);} //speed!=0

  
}

// 高速なGPIO操作をしたい
void SwitchGPIO(int sinV, int triV, int Pin){
  if (sinV >= triV){
    GPIO.out_w1ts = 1 << Pin;}
  else{
    GPIO.out_w1tc = 1 << Pin;
  }
}

void ChangeFreq(int freq){
  timerAlarm(timer2, timer_turn2[freq] , true, 0);
}

void StopPWM(){
  GPIO.out_w1tc = 1 << Pin_UH;
  GPIO.out_w1tc = 1 << Pin_UL;
  GPIO.out_w1tc = 1 << Pin_VH;
  GPIO.out_w1tc = 1 << Pin_VL;
  GPIO.out_w1tc = 1 << Pin_WH;
  GPIO.out_w1tc = 1 << Pin_WL;
}

void keikyu1000(float speed){ // 京急1000型もどき
  // ここは非同期モード
  if (speed > 0.1 && speed <= 5.5){sync_mode=0;}
  else if (speed > 5.5 && speed <= 8.5){sync_mode=0;}
  else if (speed > 8.5 && speed <= 11.5){sync_mode=0;}
  else if (speed > 11.5 && speed <= 14){sync_mode=0;}
  else if (speed > 14 && speed <= 40){sync_mode=0;ChangeFreq(900);}
  else if (speed > 40 && speed <= 50){sync_mode=9;}
  else if (speed > 50 && speed <= 60){sync_mode=5;}
  else if (speed > 60 && speed <= 80){sync_mode=3;}

  else if (speed > 80){sync_mode = 1;}


  // ここから同期モード
}
