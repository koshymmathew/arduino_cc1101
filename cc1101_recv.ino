

#include <Arduino.h>
#include <stddef.h>
#include <avr/pgmspace.h>
#include <SPI.h>



#include <rf.h>
#include <SD.h>
#include <genieArduino.h>

#define MAX_RETRIES 5
#define csn_pin 53
#define PORT_SPI_MISO  PINB
#define BIT_SPI_MISO  3
#define PORT_GDO0  PINE
#define BIT_GDO0  4

#define RESETLINE 7 



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
const int index = 0;  //HARD CODED TO READ FROM Index = 0
unsigned int rf_command =1;
char dataRate=0;
void setup() {
 

  Serial.begin(115200);
  Serial.println("starting");
  Serial1.begin(200000);  // Serial0 @ 200000 (200K) Baud
  
  rf_init(dataRate); //init for WOR mode 
  
  genie.Begin(Serial1);   // Use Serial0 for talking to the Genie Library, and to the 4D Systems display
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
      }
    } 

  }
}

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
    myFile.print(reply.src);
    myFile.print(",");
    myFile.print(reply.ID);
    myFile.print(",");
    myFile.print(reply.sensor.temperature);
    myFile.print(",");
    myFile.print(reply.sensor.region0);
    myFile.print(",");
    myFile.print(reply.sensor.region1);
    myFile.print(",");
    myFile.print(reply.sensor.region2);
    myFile.print(",");
    myFile.print(reply.sensor.region3);
    myFile.print(",");
    myFile.println(reply.sensor.region4);

    // close the file:
    myFile.close();
    Serial.println("done.");
  } 
  else {
    // if the file didn't open, print an error:
    Serial.println("error opening data.txt");
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

    temperature =  reply.sensor.temperature; 
    humidity = reply.sensor.region0; 
    soil_temp = reply.sensor.region1;  
    write_to_SD(); 
  }
}

char get_historical_data(void){
  unsigned int flashChunks=0;
  Serial.println("getting historical data");
  
  readSuccess=rf_wait(MAX_RETRIES); //make sure the node has woken up and sent us the first measurement 
  delay(200);
  
  if(readSuccess){
    write_to_SD();
    flashChunks++; 
    ///switch to high baud
    dataRate=1; //higher baud for cont data transfer
    rf_init(dataRate);  
    delay(100);
    /////
    while (flashChunks<24){
      readSuccess=rf_wait(MAX_RETRIES); //rf_check_receive();
      if(readSuccess){ 
        write_to_SD();
        flashChunks++;  
        //Serial.println(flashChunks);
        //delay(200);  
        if(flashChunks ==24){
            dataRate=0; //put it back to WOR mode 
            rf_init(dataRate);  
            delay(100);
            return 1;
        }
      } 
    }  

  }

  return 0;
}








