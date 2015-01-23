

#include <Arduino.h>
#include <stddef.h>
#include <avr/pgmspace.h>
#include <SPI.h>
#include <rf.h>
#include <SD.h>
#include <genieArduino.h>
//#include <string.h>

#define MAX_RETRIES 5
#define csn_pin 53
#define PORT_SPI_MISO  PINB
#define BIT_SPI_MISO  3
#define PORT_GDO0  PINE
#define BIT_GDO0  4

#define RESETLINE 7 

#define FLASHDONE 77
#define FLASHNOTDONE 88
#define FLASHNUMBER 55
#define ERASENUMBER 66
#define MEASUREMENTS 190 //38(bytes in the line ) *5 
#define BYTESINLINE 12
Genie genie;


//SD card CS
const int SDchipSelect = 49;


//rf variables 
char transmitSuccess=0;
char readSuccess=0;

//lcd variables 
int store_button = 0;
int wake_sensor_button =0;
int down_button =0;
int up_button =0;

unsigned int date = 1204;
unsigned int time = 1720;
unsigned int temperature = 0;
unsigned int humidity = 0;
unsigned int soil_temp = 0;
unsigned int node_number =0;
unsigned int battery =99;
unsigned int previousCrc;
unsigned int crcIn;
const int index = 0;  //HARD CODED TO READ FROM Index = 0
unsigned int rf_command =1;
char dataRate=0;
unsigned long bytes_in_flash;

void setup() {


  Serial.begin(115200);
  Serial.println("starting");
  Serial2.begin(200000);  // Serial0 @ 200000 (200K) Baud

  rf_init(dataRate); //init for WOR mode 

    genie.Begin(Serial2);   // Use Serial0 for talking to the Genie Library, and to the 4D Systems display
  genie.AttachEventHandler(myGenieEventHandler); // Attach the user function Event Handler for processing events

  pinMode(RESETLINE, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
  digitalWrite(RESETLINE, 0);  // Reset the Display via D4
  delay(100);
  digitalWrite(RESETLINE, 1);  // unReset the Display via D4


  delay (3500); //let the display start up after the reset (This is important)

  //Turn the Display on (Contrast) - (Not needed but illustrates how)
  genie.WriteContrast(15); // 1 = Display ON, 0 = Display OFF.

  char result;
  pinMode(csn_pin,OUTPUT);

  rf_debug();
  attachInterrupt(0, packet_end_seen_ISR, FALLING);
  //////SD card inits 
  pinMode(SDchipSelect, OUTPUT);  //SD chip select 

  if (!SD.begin(SDchipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

}

void loop(){
  char hist_data_done;  
  update_ALL();

  if(wake_sensor_button){

    transmitSuccess = rf_wor_tx(node_number,rf_command);
    if((transmitSuccess &&(rf_command==1))){
      get_current_data();
    }
    else if((transmitSuccess &&(rf_command==2))){
      hist_data_done= get_historical_data();
      if(hist_data_done){
        Serial.println("done getting the historical data");
        Serial.println(); 
        genie.WriteObject(GENIE_OBJ_WINBUTTON,0x01,0); //put button back to "wake sensor " mode 
        genie.WriteObject(GENIE_OBJ_WINBUTTON,0x00,0); //put STORE button back to 0
        wake_sensor_button =0;  
        rf_command =1; // default value for rf_commnd (just current measurements)
        get_lines_from_sd(0);  
      }
    } 

  }
}

/*
void loop(){
 rf_read_only();
 }
 */
void update_ALL()
{
  genie.DoEvents(); // Check for any Message Reports from the 4D Systems display.
  update_main_page(); // Writes and reads required data to and from the objects on the 4D Systems display.
  genie.DoEvents(); // Check for any Message Reports from the 4D Systems display.
}

void update_main_page()
{


  // Update all of the neccessary objects on the display with values from the VSD modbus reads and the user speed input slider.
  //Serial.println(date);
  date = reply.datetime.month *100 +reply.datetime.day;
  time = reply.datetime.hour *100 + reply.datetime.min;

  float adcTemp = reply.sensor.val_5; 
  float temp = ((adcTemp/65536)*175.72) -46.85; 

  float adcHumid = reply.sensor.val_0; 
  float humid = ((adcHumid/65536)*125) -6;
  humidity = humid *10;
  temperature = temp*10;
  if(humidity<0)humidity=0;
  if(temperature<0)temperature=0;
  //Serial.println(temp);
  //Serial.println(humid);
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, date); //DATE
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x01, time);//TIME
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x02, temperature);//Temp
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x03, humidity); //humidity
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x04, soil_temp);//soil temp 
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x05, node_number);//node 
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x1F, battery);//node 

}


void myGenieEventHandler(void)
{
  //Serial.println("in event handler");
  genieFrame Event;
  genie.DequeueEvent(&Event);

  //Read the store button and pass that info to the rf 
  if (genie.EventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_WINBUTTON, index))//|| genie.EventIs(&Event, GENIE_REPORT_OBJ,   GENIE_OBJ_WINBUTTON, index))
  {
    store_button = genie.GetEventData(&Event);  // Receive the event data from the winbutton0
    Serial.print("store_button value is");
    Serial.println(store_button);
    rf_command =2;
  }

  if (genie.EventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_WINBUTTON, index+1))//|| genie.EventIs(&Event, GENIE_REPORT_OBJ,   GENIE_OBJ_WINBUTTON, index+1))
  {
    wake_sensor_button = genie.GetEventData(&Event);  // Receive the event data from the winbutton1
    Serial.print("wake_sensor_button  value is");
    Serial.println(wake_sensor_button);

  }

  if (genie.EventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_4DBUTTON, index))//|| genie.EventIs(&Event, GENIE_REPORT_OBJ,   GENIE_OBJ_4DBUTTON, index))
  {
    down_button = genie.GetEventData(&Event);  // Receive the event data from the Slider0
    Serial.print("down button pressed ");
    Serial.println(down_button);

    if (node_number==0){
      genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x05, 0);//node     
    }
    else{
      node_number--;  
      genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x05, node_number);//node  
    }
  }


  if (genie.EventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_4DBUTTON, index+1))//|| genie.EventIs(&Event, GENIE_REPORT_OBJ,   GENIE_OBJ_4DBUTTON, index+1))
  {
    up_button = genie.GetEventData(&Event);  // Receive the event data from the Slider0
    Serial.print("up button pressed ");
    Serial.println(up_button);
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x05, node_number+1);//node  
    node_number++;  
  }  
} 


void write_to_SD(void){

  File myFile = SD.open("data.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    //myFile.print(reply.src);
    //myFile.print(",");
    //myFile.print(reply.ID);
    //myFile.print(",");
    myFile.print(reply.sensor.val_5);
    myFile.print(",");
    myFile.print(reply.sensor.val_0);
    myFile.print(",");
    myFile.print(reply.sensor.val_1);
    myFile.print(",");
    myFile.print(reply.sensor.val_2);
    myFile.print(",");
    myFile.print(reply.sensor.val_3);
    myFile.print(",");
    myFile.println(reply.sensor.val_4);

    // close the file:
    myFile.close();
    Serial.println("done.");
  } 
  else {
    // if the file didn't open, print an error:
    Serial.println("error opening data.txt");
  }
}

unsigned int write_flash_to_SD(unsigned int previousCrc){
  int flag=0;

  crcIn = calc_crc(flash.txbuffer,20,0xffff); //Calculate CRC on-the-fly   
  Serial.print(" CRC in = "); 
  Serial.print(crcIn,HEX);
  Serial.print("   Previous crc = ");
  Serial.println(previousCrc,HEX);

  if(crcIn != previousCrc){
    File myFile = SD.open("data.txt", FILE_WRITE);

    // if the file opened okay, write to it:
    if (myFile) {

      for(int j=0;j<20;j=j++)
      {
        flag=0;
        Serial.print("[");
        Serial.print(j);
        Serial.print("] ");
        Serial.print(flash.txbuffer[j],HEX);
        Serial.print(",");

        if((flash.txbuffer[j] ==254) &&(flash.txbuffer[j+1]==254)){
          Serial.println("demarcate found");
          myFile.println();
          flag =1;
          j = j+1; //skip writing the 0xfe 0xfe
        }

        if((flash.txbuffer[j] == 255) && (flash.txbuffer[j+1] == 255))
        {
          bytes_in_flash = myFile.position();
          Serial.print("position in the file is "); 
          Serial.println(bytes_in_flash);
          // close the file:
          myFile.close();
          Serial.println(" done.");
          return(crcIn); 
          break;
        }
        if(!flag){
          //if((flash.txbuffer[j] !=254) &&(flash.txbuffer[j+1] !=254)){
          if(flash.txbuffer[j]<16)myFile.print(0);
          myFile.print(flash.txbuffer[j],HEX);
          myFile.print(",");
          //}
        }

      }
      //myFile.println();

      // close the file:
      myFile.close();
      Serial.println(" done.");


      return(crcIn);  
    } 
    else {
      // if the file didn't open, print an error:
      Serial.println("error opening data.txt");
    }
  }
}
void get_current_data(void){
  Serial.println("getting current data");
  readSuccess=rf_wait(MAX_RETRIES); //rf_check_receive();
  if(readSuccess){
    genie.WriteObject(GENIE_OBJ_WINBUTTON,0x01,0); //put button back to "wake sensor " mode 
    genie.WriteObject(GENIE_OBJ_WINBUTTON,0x00,0); //put STORE button back to 0
    wake_sensor_button =0;  
    rf_command =1; // default value for rf_commnd (just current measurements)

    temperature =  reply.sensor.val_5; 
    humidity = reply.sensor.val_0; 
    soil_temp = reply.sensor.val_1;  
    //write_to_SD(); 
  }
}

char get_historical_data(void){
  unsigned int flashChunks=0;
  unsigned int currentCrc=0;
  Serial.println("getting historical data");

  readSuccess=rf_wait_flash(MAX_RETRIES); //make sure the node has woken up and sent us the first measurement 
  delay(350); //needs this because the first measurement is @ 1.2 kbps

  if(readSuccess){
    currentCrc=write_flash_to_SD(currentCrc);
    //write_history_page_1(flashChunks); //write the first reading to row 0 of page 1
    flashChunks++; 
    if(flash.done == FLASHDONE)
    {
      return 1;
    }
    ///switch to high baud
    Serial.println("high data rate"); 
    dataRate=1; //higher baud for cont data transfer
    rf_init(dataRate);  
    delay(100);
    /////
    while (flash.done != FLASHDONE){

      readSuccess=rf_wait_flash(MAX_RETRIES); //rf_check_receive();
      if(readSuccess){ 
        currentCrc=write_flash_to_SD(currentCrc);
        //write_history_page_1(flashChunks); //write the first reading to row 0 of page 1
        flashChunks++;  
        //Serial.println(flashChunks);
        //delay(200);  
        if(flash.done ==FLASHDONE){
          dataRate=0; //put it back to WOR mode 
          rf_init(dataRate);  
          delay(100);
          return 1;
        }
      } 
      //delay(10);
    }  

  }

  return 0;
}


void write_history_page_1(int row_number,unsigned char val[]){
  unsigned int soil_temp;
  date = val[1]*100 +val[2];
  time = val[3]*100 +val[4];
  
  float adcTemp = (val[6]<<8) | val[7]; 
  float temp = ((adcTemp/65536)*175.72) -46.85; 
  float adcHumid = (val[8]<<8) | val[9] ; 
  float humid = ((adcHumid/65536)*125) -6;
  humidity = humid *10;
  temperature = temp*10;
  Serial.print(temperature);Serial.println(humidity);
  soil_temp =(val[10]<<8) | val[11] ; 
  
  if(row_number ==0){
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x06, date); //DATE
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x07, time);//TIME
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x08, temperature);//Temp
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x09,  humidity); //humidity
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x0A, soil_temp);//soil temp 
  }
  else if(row_number ==1){
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x0B, date); //DATE
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x0C, time);//TIME
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x0D, temperature);//Temp
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x0E, humidity); //humidity
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x0F, soil_temp);//soil temp 
  }
  else if(row_number ==2){
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x10, date); //DATE
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x11, time);//TIME
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x12, temperature);//Temp
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x13, humidity); //humidity
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x14, soil_temp);//soil temp 
  }
  else if(row_number ==3){
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x15, date); //DATE
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x16, time);//TIME
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x17, temperature);//Temp
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x18, humidity); //humidity
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x19, soil_temp);//soil temp 
  }
  else if(row_number ==4){
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x1A, date); //DATE
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x1B, time);//TIME
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x1C, temperature);//Temp
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x1D, humidity); //humidity
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x1E, soil_temp);//soil temp 
  }

}

void rf_read_only(void){
  char read_result =0;
  read_result = rf_wait(MAX_RETRIES);
  delay(200);
}


uint16_t calc_crc(unsigned char *msg,int n,uint16_t init)
{
  uint16_t x = init;

  while(n--)
  {
    x = crc_xmodem_update(x, (uint16_t)*msg++);
  }

  return(x);
}

uint16_t crc_xmodem_update (uint16_t crc, uint8_t data)
{
  int i;

  crc = crc ^ ((uint16_t)data << 8);
  for (i=0; i<8; i++)
  {
    if (crc & 0x8000)
      crc = (crc << 1) ^ 0x1021; //(polynomial = 0x1021)
    else
      crc <<= 1;
  }
  return crc;
}

void get_lines_from_sd(int line_number){
  char aChar;
  int row=0;
  unsigned long seek_position = bytes_in_flash -  MEASUREMENTS; //190 at the moment (38*5) 

  unsigned int i=0;
  char inData[80];
  int data_index=0;
  File myFile = SD.open("data.txt", FILE_READ);
  unsigned char value[30];
  // if the file opened okay, write to it:
  if (myFile) {
    myFile.seek(seek_position);

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      aChar=myFile.read();

      if(aChar == '\n')
      {
        int j=0;
        for(i=0;i<BYTESINLINE;i++){
          value[i] = get_char_value(inData[j],inData[j+1]);
          Serial.print(value[i],HEX); 
          Serial.print(",");
          j+=2;
        }
        Serial.println();
        write_history_page_1(row,value);
        data_index = 0;
        inData[data_index] = NULL; 
        row++; 
      }
      else
      {
        if(aChar != ','){
          //Serial.print(aChar);
          inData[data_index] = aChar;
          data_index++;
          //Serial.print("data index is at"),Serial.print(data_index);
          //inData[data_index] = '\0'; // Keep the string NULL terminated
          //Serial.print(inData[index]); 
        } 
      }
    }  


    myFile.close();
  } 
  else {
    // if the file didn't open, print an error:
    Serial.println("error opening data.txt");
  }

}

unsigned char get_char_value(char tens,char units){
  unsigned char high_nibble,low_nibble,value;
  high_nibble = h2d(tens);
  low_nibble = h2d(units);

  value = (high_nibble << 4) | low_nibble;


  return value;
}

unsigned char h2d(unsigned char hex)
{
  if(hex > 0x39) hex -= 7; // adjust for hex letters upper or lower case
  return(hex & 0xf);
}







