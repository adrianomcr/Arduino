/* Node vf1
 *  Authors:
 *    Adriano Rezende
 *    Elias José Freitas
*/ 


//Node definitions
// ----------  ----------  ----------  ----------  ----------
//Current node ID
#define ID_NODE  2
#define ID_NODE_SIM  8

//Maximum number of messages received from one bus in orther t shut down one that is not active
#define LIM_WATCHDOG 12
//Amount to increase the "trust variable" when desagreement is detected (penalization)
#define INCREASE_TRUST_VAR 30
//Amount to decrease the "trust variable" when agreement is detected
#define DECREASE_TRUST_VAR 1

//Filter for trusting in position command of a bus
#define LIM_TRUST 100

//Tolerance (maximum standard deviation) to consider that commands are in agreement
#define TOLERANCE 300 //TOLERANCE = 1 is equivalent to 360/4096 degrees

//Definition of the frequency that the nodde will get information from the servo
#define READ_TEMP_INTERVAL 1000 //frequency of temperature value update (ms)
#define READ_POS_INTERVAL 200 //frequency of position value update (ms)
#define READ_LOAD_INTERVAL 1234 //frequency of load value update (ms)

//Definitio of the baud rate
//#define BAUD_RATE 57142 //Baud rate (Hz)
//#define BAUD_RATE 111111 //Baud rate (Hz)
//#define BAUD_RATE 200000 //Baud rate (Hz)
//#define BAUD_RATE 500000 //Baud rate (Hz)
#define BAUD_RATE 333333 //Baud rate (Hz)
// ----------  ----------  ----------  ----------  ----------




#define BIT_TIME 1000000/BAUD_RATE //Time interval of a single bit (us)
#define BYTE_TIME 9*BIT_TIME //Time interval of a byte plus the bit start (us)

//#define TIMEOUT_RX_TX 150*BIT_TIME //Time-out while waiting for response to a read request (us)
#define TIMEOUT_RX_TX 200*BYTE_TIME //Time-out while waiting for response to a read request (us)


//Dynamixel Definition
#define AX_READ_DATA                2
#define AX_WRITE_DATA               3

#define AX_GOAL_POSITION_L          30

#define AX_PRESENT_POSITION_L       36
#define AX_PRESENT_POSITION_H       37 //not used
#define AX_PRESENT_SPEED_L          38 //not used
#define AX_PRESENT_SPEED_H          39 //not used
#define AX_PRESENT_LOAD_L           40
#define AX_PRESENT_LOAD_H           41 //not used
#define AX_PRESENT_VOLTAGE          42 //not used
#define AX_PRESENT_TEMPERATURE      43

//Specials
#define AX_TEM_LENGTH               4
#define AX_BYTE_READ                1
#define AX_BYTE_READ_POS            2
#define AX_START                    255
#define AX_POS_LENGTH               4
#define AX_GOAL_LENGTH              5



//Program definition
#define STATE_DETECTING_MENSAGE0 0
#define STATE_DETECTING_MENSAGE1 1
#define STATE_CHECK_ID   2
#define STATE_LENGTH_DATA  3
#define STATE_TYPE_DATA  4
#define STATE_DESCRIPTION_DATA 5
#define STATE_GET_DATA 6
#define STATE_CHECKSUM 7
#define STATE_END_MENSAGE 8


// Number of the direction pin for the three buses
#define DIR_PIN_BUS_1  11
#define DIR_PIN_BUS_2  3
#define DIR_PIN_BUS_3  30
#define DIR_PIN_BUS_SERVO 6





// ---------  ---------  ---------  ---------  ---------  ---------  ---------
// ---------  ---------  ---------  ---------  ---------  ---------  ---------





//STRUCT FOR COMMANDS
typedef struct CMD_SERVO {
  int   id_bus; //id for the bus 1, 2 ,3
  int   type; //read or write command
  int   description; //goal position, led status etc
  int   value1; //value command
  int   value2;
  char  cmd_use_2values; // = 1 if the cmd use 2 values information
  char  isprocessed; //if the command is processed
} CMD_SERVO;


//STRUCT FOR MENSAGE
typedef struct MSG_DATA {
  CMD_SERVO new_cmd;
  char isreceived;
  int last_state;
  int checksum;
  int length_data;
  int data1;
  int data2;

} MSG_DATA;

//STRUCT FOR CHECKER
typedef struct MSG_CHECK {
  int isbus1_received;
  int isbus2_received;
  int isbus3_received;

  int bus1_data1;
  int bus1_data2;

  int bus2_data1;
  int bus2_data2;

  int bus3_data1;
  int bus3_data2;

  int watchdog_bus1;
  int watchdog_bus2;
  int watchdog_bus3;

  int trust_bus1;
  int trust_bus2;
  int trust_bus3;

  int result_checksum;
  int result_length_data;
  int result_data1;
  int result_data2;

} MSG_CHECK;

//GLOBAL VARIABLES

CMD_SERVO cmd_servo_bus1;
CMD_SERVO cmd_servo_bus2;
CMD_SERVO cmd_servo_bus3;

MSG_DATA msg_data_bus1;
MSG_DATA msg_data_bus2;
MSG_DATA msg_data_bus3;

MSG_CHECK check_move;


// Variaveis de medicao do servo ----------  ----------  ----------
int temperature; //current temperature of the sevo
int load_low; //least significant byte of the servos load
int load_high; //most significant byte of the servos load
int load_long; //current load of the servo
int pos_low; //least significant byte of the servos position
int pos_high; //most significant byte of the servos position
int pos_long; //current position of the servo

long int time_count_temp; //time counter for temperature measurement
long int time_count_load; //time counter for load measurement
long int time_count_pos; //time counter for position measurement

long int TX_count_move = 0; //time counter while sending position comand to the servo
long int RX_count_move = 0; //time counter while receiving data 
int waiting_move_cmd = 0; //flag that indicates the existence of a position command to be sent to the servo
int valor_waiting; //position value waiting to be sent to the servo

long int TX_count_temp = 0; //time counter while sending data to request temperture
long int RX_count_temp = 0; //time counter while receiving data with the current temperature
long int TX_count_load = 0; //time counter while sending data to request load
long int RX_count_load = 0; //time counter while receiving data with the current load
long int TX_count_pos = 0; //time counter while sending data to request position
long int RX_count_pos = 0; //time counter while receiving data with the current position
//----------  ----------  ----------  ----------  ----------



// Error/Emergency variables ----------  ----------  ----------
#define ERROR_MOVE_NO_RESPONSE 68 //Code to indicate that the servo is not responding       ---------     CHECK IF THIS THERE IS A PREDEFINED NUMBER FOR THIS
int error_move; //number capturated from the error code that comes from the servo after a position command is sent
int respond_error_move_BUS1; //is set to 1 when the BUS 1 is waitting an error message from after a move command
int respond_error_move_BUS2; //is set to 1 when the BUS 1 is waitting an error message from after a move command
int respond_error_move_BUS3; //is set to 1 when the BUS 1 is waitting an error message from after a move command



int EMERGENCY = 0; // when 1 indicates emergency state
#define AX_EMERGENCY_DATA 77 // Number that representes a emergency type data
#define EMERGENCY_POSITIOIN 2048 // Defalt position to be commanded in a emergency situation
#define ENTRY_EMERGENCY 11 // Data that indicates the request to entry in an emergency mode
#define LEAVE_EMERGENCY 22 // Data that indicates the request to exit in an emergency mode



long emergency_move_count = 0; //Counter to send the EMERGENCY_POSITIOIN frequently
//----------  ----------  ----------  ----------  ----------










//Setup
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------
void setup() {

  //Bus servo
  Serial.begin(BAUD_RATE);

  //Bus control 1
  Serial1.begin(BAUD_RATE);

  //Bus control 2
  Serial2.begin(BAUD_RATE);

  //bus control 3
  Serial3.begin(BAUD_RATE);
  

  msg_data_bus1.isreceived = 0;
  msg_data_bus1.length_data = 0;
  msg_data_bus1.checksum = 0;
  msg_data_bus1.data1 = 0;
  msg_data_bus1.data2 = 0;
  msg_data_bus1.last_state = STATE_DETECTING_MENSAGE0;
  msg_data_bus1.new_cmd.id_bus = 1;

  msg_data_bus2.isreceived = 0;
  msg_data_bus2.length_data = 0;
  msg_data_bus2.checksum = 0;
  msg_data_bus2.data1 = 0;
  msg_data_bus2.data2 = 0;
  msg_data_bus2.last_state = STATE_DETECTING_MENSAGE0;
  msg_data_bus2.new_cmd.id_bus = 2;

  msg_data_bus3.isreceived = 0;
  msg_data_bus3.length_data = 0;
  msg_data_bus3.checksum = 0;
  msg_data_bus3.data1 = 0;
  msg_data_bus3.data2 = 0;
  msg_data_bus3.last_state = STATE_DETECTING_MENSAGE0;
  msg_data_bus3.new_cmd.id_bus = 3;

  check_move.isbus1_received = 0;
  check_move.watchdog_bus1 = 0;
  check_move.trust_bus1 = 0;
  check_move.isbus2_received = 0;
  check_move.watchdog_bus2 = 0;
  check_move.trust_bus2 = 0;
  check_move.isbus3_received = 0;
  check_move.watchdog_bus3 = 0;
  check_move.trust_bus3 = 0;

  temperature = 0;
  load_low = 0;
  pos_low = 0;

  time_count_temp = millis(); //initialization of time counter
  time_count_load = millis(); //initialization of time counter
  time_count_pos = millis(); //initialization of time counter


  //AAAAAAAAAAAAAAAAAAAA
  error_move = 0;
  respond_error_move_BUS1 = 0;
  respond_error_move_BUS2 = 0;
  respond_error_move_BUS3 = 0;
  //---------------


  

  pinMode(DIR_PIN_BUS_1, OUTPUT);
  digitalWrite(DIR_PIN_BUS_1, LOW);
  pinMode(DIR_PIN_BUS_2, OUTPUT);
  digitalWrite(DIR_PIN_BUS_2, LOW);
  pinMode(DIR_PIN_BUS_3,OUTPUT);
  digitalWrite(DIR_PIN_BUS_3, LOW);
  pinMode(DIR_PIN_BUS_SERVO,OUTPUT);
  digitalWrite(DIR_PIN_BUS_SERVO, LOW);



  //TESTES GERAIS
  pinMode(45, OUTPUT);
  digitalWrite(45, LOW);
  pinMode(13, OUTPUT); //Indicates emergency
  digitalWrite(13, LOW);

  ///*
  send_ok_msg();
  delay(1);
  send_ok_msg();
  delay(1);
  send_ok_msg();
  delay(1);
  send_ok_msg();
  delay(1);
  send_ok_msg();
  delay(1);
  send_ok_msg();
  delay(1);
  send_ok_msg();
  delay(1);
  //*/
  
}
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------










//Loop
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------
void loop() {
  if(EMERGENCY == 0){ // COMMON STATE (NOT EMERGENCY)
    digitalWrite(13,LOW);
    // ----------  ----------  ----------  ----------  ----------
    if (msg_data_bus1.isreceived) {
      cmd_servo_bus1 = msg_data_bus1.new_cmd;
      cmd_servo_bus1.isprocessed = 0;
      if (cmd_servo_bus1.type == AX_READ_DATA) { //READ COMMAND
        cmd_servo_bus1.isprocessed = sendValueServo_to_Controller(cmd_servo_bus1.description, 1);
      }
      else if (cmd_servo_bus1.type == AX_WRITE_DATA) { //WRITE COMMAND
        cmd_servo_bus1.isprocessed = selectDatatoCheck(&cmd_servo_bus1);
        Respond_To_Move_Cmd(1);
      }
      else if (cmd_servo_bus1.type == AX_EMERGENCY_DATA) {
        //if(msg_data_bus1.data1 == ENTRY_EMERGENCY){
        if(cmd_servo_bus1.description == ENTRY_EMERGENCY){
          go_to_emergency_state(2);
        }
      }
      msg_data_bus1.isreceived = 0;
    }//end bus1
    // ----------  ----------  ----------  ----------  ----------
    if (msg_data_bus2.isreceived) {
      cmd_servo_bus2 = msg_data_bus2.new_cmd;
      cmd_servo_bus2.isprocessed = 0;
      if (cmd_servo_bus2.type == AX_READ_DATA) { //READ COMMAND
        cmd_servo_bus2.isprocessed = sendValueServo_to_Controller(cmd_servo_bus2.description, 2);
      }
      else if (cmd_servo_bus2.type == AX_WRITE_DATA) { //WRITE COMMAND
        cmd_servo_bus2.isprocessed = selectDatatoCheck(&cmd_servo_bus2);
        Respond_To_Move_Cmd(2);
      }
      else if (cmd_servo_bus2.type == AX_EMERGENCY_DATA) {
        //if(msg_data_bus2.data1 == ENTRY_EMERGENCY){
        if(cmd_servo_bus2.description == ENTRY_EMERGENCY){
          go_to_emergency_state(2);
        }
      }
      msg_data_bus2.isreceived = 0;
    }//end bus2
    // ----------  ----------  ----------  ----------  ----------
    if (msg_data_bus3.isreceived) {
      cmd_servo_bus3 = msg_data_bus3.new_cmd;
      cmd_servo_bus3.isprocessed = 0;
      if (cmd_servo_bus3.type == AX_READ_DATA) { //READ COMMAND
        cmd_servo_bus3.isprocessed = sendValueServo_to_Controller(cmd_servo_bus3.description, 3);
      }
      else if (cmd_servo_bus3.type == AX_WRITE_DATA) { //WRITE COMMAND
        cmd_servo_bus3.isprocessed = selectDatatoCheck(&cmd_servo_bus3);
        Respond_To_Move_Cmd(3);
      }
      else if (cmd_servo_bus3.type == AX_EMERGENCY_DATA) {
        //if(msg_data_bus3.data1 == ENTRY_EMERGENCY){
        if(cmd_servo_bus3.description == ENTRY_EMERGENCY){
          go_to_emergency_state(2);
        }
      }
      msg_data_bus3.isreceived = 0;
    }//end bus3
    // ----------  ----------  ----------  ----------  ----------
  
  
    // !!!!!!!!!!**********!!!!!!!!!!
    //Apply redundancy policy to compuve a goal position command
    apply_redundancy();
    // !!!!!!!!!!**********!!!!!!!!!!
  
    //Attempt to measure servo states
    update_servo_info();
    
    //Controls the switching of the TX_ and RX_ variales
    TX_RX_switch();
  
    //Attempt to send position command
    Try_to_send_cmd_move(valor_waiting);
  }


  
  else{ // EMERGENCY STATE
    digitalWrite(13,HIGH);

    // ----------  ----------  ----------  ----------  ----------
    if (msg_data_bus1.isreceived) {
      cmd_servo_bus1 = msg_data_bus1.new_cmd;
      cmd_servo_bus1.isprocessed = 0;
      if (cmd_servo_bus1.type == AX_READ_DATA) { //READ COMMAND
        cmd_servo_bus1.isprocessed = sendValueServo_to_Controller(cmd_servo_bus1.description, 1);
      }
      else if (cmd_servo_bus1.type == AX_WRITE_DATA) { //WRITE COMMAND
        cmd_servo_bus1.isprocessed = selectDatatoCheck(&cmd_servo_bus1);
        Respond_To_Move_Cmd(1);
      }
      if (cmd_servo_bus1.type == AX_EMERGENCY_DATA) { //CHEK IF THERE IS A LEAVE EMERGENCY MESSAGE
        if(cmd_servo_bus1.description == LEAVE_EMERGENCY){
          leave_emergency_state(2);
        }
      }
      msg_data_bus1.isreceived = 0;
    }//end bus1
    // ----------  ----------  ----------  ----------  ----------
    if (msg_data_bus2.isreceived) {
      cmd_servo_bus2 = msg_data_bus2.new_cmd;
      cmd_servo_bus2.isprocessed = 0;
      if (cmd_servo_bus2.type == AX_READ_DATA) { //READ COMMAND
        cmd_servo_bus2.isprocessed = sendValueServo_to_Controller(cmd_servo_bus2.description, 2);
      }
      else if (cmd_servo_bus2.type == AX_WRITE_DATA) { //WRITE COMMAND
        cmd_servo_bus2.isprocessed = selectDatatoCheck(&cmd_servo_bus2);
        Respond_To_Move_Cmd(2);
      }
      if (cmd_servo_bus2.type == AX_EMERGENCY_DATA) { //CHEK IF THERE IS A LEAVE EMERGENCY MESSAGE
        if(cmd_servo_bus2.description == LEAVE_EMERGENCY){
          leave_emergency_state(2);
        }
      }
      msg_data_bus2.isreceived = 0;
    }//end bus2
    // ----------  ----------  ----------  ----------  ----------
    if (msg_data_bus3.isreceived) {
      cmd_servo_bus3 = msg_data_bus3.new_cmd;
      cmd_servo_bus3.isprocessed = 0;
      if (cmd_servo_bus3.type == AX_READ_DATA) { //READ COMMAND
        cmd_servo_bus3.isprocessed = sendValueServo_to_Controller(cmd_servo_bus3.description, 3);
      }
      else if (cmd_servo_bus3.type == AX_WRITE_DATA) { //WRITE COMMAND
        cmd_servo_bus3.isprocessed = selectDatatoCheck(&cmd_servo_bus3);
        Respond_To_Move_Cmd(3);
      }
      if (cmd_servo_bus3.type == AX_EMERGENCY_DATA) { //CHEK IF THERE IS A LEAVE EMERGENCY MESSAGE
        if(cmd_servo_bus3.description == LEAVE_EMERGENCY){
          leave_emergency_state(2);
        }
      }
      msg_data_bus3.isreceived = 0;
    }//end bus3
    // ----------  ----------  ----------  ----------  ----------



    valor_waiting = EMERGENCY_POSITIOIN;
    waiting_move_cmd = 0;
    if(millis() - emergency_move_count > 1000){
     waiting_move_cmd = 1;
     emergency_move_count = millis();
    }
      
    //Attempt to measure servo states
    update_servo_info();
    
    //Controls the switching of the TX_ and RX_ variales
    TX_RX_switch();

    //Attempt to send position command
    Try_to_send_cmd_move(valor_waiting);


    //Verify data coherency in orther to leave emergency state
    if(apply_redundancy() == 1){
      Sinalize();
      //EMERGENCY = 0;
      leave_emergency_state(1);
      send_ok_msg();
      send_ok_msg();
      send_ok_msg();
      send_ok_msg();
      send_ok_msg();
    }


    
  }



}//end loop()
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------





//Interruption to treat incoming data on bus 1
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------
void serialEvent1() {
  
  int num_data1 = Serial1.available(); //get number of bytes

  int buffer_int[num_data1];

  for (int i = 0; i < num_data1; i++) {
    char byte = (char)Serial1.read(); //get the byte
    buffer_int[i] = char_to_uint(byte);
  }

  processData(buffer_int, num_data1, &msg_data_bus1.new_cmd, &msg_data_bus1.last_state, &msg_data_bus1.data1, &msg_data_bus1.data2, &msg_data_bus1.checksum, &msg_data_bus1.length_data, &msg_data_bus1.isreceived);
}//end serialEvent1()
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------

//Interruption to treat incoming data on bus 2
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------
void serialEvent2() {

  int num_data2 = Serial2.available(); //get number of bytes

  int buffer_int[num_data2];

  for (int i = 0; i < num_data2; i++) {
    char byte = (char)Serial2.read(); //get the byte
    buffer_int[i] = char_to_uint(byte);
  }

  processData(buffer_int, num_data2, &msg_data_bus2.new_cmd, &msg_data_bus2.last_state, &msg_data_bus2.data1, &msg_data_bus2.data2, &msg_data_bus2.checksum, &msg_data_bus2.length_data, &msg_data_bus2.isreceived);
}//end serialEvent2()
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------

//Interruption to treat incoming data on bus 3
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------
///*
void serialEvent3() {

  int num_data3 = Serial3.available(); //get number of bytes

  int buffer_int[num_data3];

  for (int i = 0; i < num_data3; i++) {
    char byte = (char)Serial3.read(); //get the byte
    buffer_int[i] = char_to_uint(byte);
  }
  
  processData(buffer_int, num_data3, &msg_data_bus3.new_cmd, &msg_data_bus3.last_state, &msg_data_bus3.data1, &msg_data_bus3.data2, &msg_data_bus3.checksum, &msg_data_bus3.length_data, &msg_data_bus3.isreceived);
}//end serialEvent3()
//*/
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------





//State machine to process bytes received from buses 1 2 and 3
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------
//Process the sequence of bytes received
void processData(int buffer_int[], int num_data, struct CMD_SERVO *new_cmd, int* state, int* data1, int* data2, int* checksum, int* length_data, char* isreceived) {

  int received_int = 0;

  for (int i = 0; i < num_data; i++) {
    received_int = buffer_int[i];
    switch (*state) {

      case STATE_DETECTING_MENSAGE0:  //Wait for 2 bytes equal 255 to init the sequence of the data
        if (received_int == 255) {
          *state = STATE_DETECTING_MENSAGE1;
        }
        break;

      case STATE_DETECTING_MENSAGE1:
        if (received_int == 255) {
          *state = STATE_CHECK_ID;
        }
        else {
          *state = STATE_DETECTING_MENSAGE0;
        }
        break;
        
      case STATE_CHECK_ID:
        if (received_int == ID_NODE) {
          *state = STATE_LENGTH_DATA;;
          new_cmd->isprocessed = 0; //new command in process
          *checksum = *checksum + received_int;
        }
        else {
          *state = STATE_DETECTING_MENSAGE0;
          *checksum = 0;
          *data1 = 0;
          *data2 = 0;
          *length_data = 0;
        }
        break;

      case STATE_LENGTH_DATA:
        *length_data = received_int;
        *state =  STATE_TYPE_DATA;
        *checksum = *checksum + received_int;
        break;

      case STATE_TYPE_DATA:
        new_cmd->type = received_int;
        *state = STATE_DESCRIPTION_DATA;
        *length_data = *length_data - 1;
        *checksum = *checksum + received_int;
        break;

      case STATE_DESCRIPTION_DATA:
        new_cmd->description = received_int;
        *length_data = *length_data - 1;
        *checksum = *checksum + received_int;
        if(*length_data != 1){ //CHECK IF IS NOT EMERGENCY
          *state = STATE_GET_DATA;
          if (*length_data == 5) new_cmd->cmd_use_2values = 1;
          else new_cmd->cmd_use_2values = 0;
        }
        else{ //EMERGENCY MESSAGE
          *state = STATE_CHECKSUM;
          }
        break;

      case STATE_GET_DATA:
        switch (*length_data) {
          case 5:
            *data2 = *data2 + received_int;
            *length_data = *length_data - 1;
            break;
          case 4:
            *data2 = *data2 + received_int * 256;
            *length_data = *length_data - 1;
            break;
          case 3:
            *data1 = *data1 + received_int;
            *length_data = *length_data - 1;
            break;
          case 2:
            *data1 = *data1 + received_int * 256;
            *length_data = *length_data - 1;
            break;
        }
        *checksum = *checksum + received_int;
        if (*length_data == 1)
          *state = STATE_CHECKSUM;
        break;

      case STATE_CHECKSUM:
        *checksum = (~*checksum) & 0xFF;
        if (*checksum == received_int) {
          new_cmd->isprocessed = 1; //end correct command
          *isreceived = 1;
          new_cmd->value1 = *data1;
          new_cmd->value2 = *data2;
        }
        *state = STATE_DETECTING_MENSAGE0;
        *checksum = 0;
        *data1 = 0;
        *data2 = 0;
        *length_data = 0;
        break;

      default:
        *state = STATE_DETECTING_MENSAGE0;
        break;
    }
  }
}//end processData()
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------





// Send the desired information to the controller
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------
int sendValueServo_to_Controller(int description_command, int bus) {

  switch (description_command) {
    
    case AX_PRESENT_TEMPERATURE: //Send temperature information back to one of the controllers
      if (bus == 1) {
        digitalWrite(DIR_PIN_BUS_1, HIGH);
        Serial1.write((char)255);
        Serial1.write((char)255);
        Serial1.write((char)ID_NODE);
        Serial1.write((char)3); //length data
        Serial1.write((char)0); //no error byte
        Serial1.write(temperature);
        int checksum = (~(ID_NODE + 3 + 0 + temperature)) & 0xFF;
        Serial1.write(checksum);
        delayMicroseconds(7*BYTE_TIME+2);
        digitalWrite(DIR_PIN_BUS_1, LOW);
      }
      else if (bus == 2) {
        digitalWrite(DIR_PIN_BUS_2, HIGH);
        Serial2.write((char)255);
        Serial2.write((char)255);
        Serial2.write((char)ID_NODE);
        Serial2.write((char)3); //length data
        Serial2.write((char)0); //no error byte
        Serial2.write(temperature);
        int checksum = (~(ID_NODE + 3 + 0 + temperature)) & 0xFF;
        Serial2.write(checksum);
        delayMicroseconds(7*BYTE_TIME+2);
        digitalWrite(DIR_PIN_BUS_2, LOW);
      }
      else if (bus == 3) {
        digitalWrite(DIR_PIN_BUS_3, HIGH);
        Serial3.write((char)255);
        Serial3.write((char)255);
        Serial3.write((char)ID_NODE);
        Serial3.write((char)3); //length data
        Serial3.write((char)0); //no error byte
        Serial3.write(temperature);
        int checksum = (~(ID_NODE + 3 + 0 + temperature)) & 0xFF;
        Serial3.write(checksum);
        delayMicroseconds(7*BYTE_TIME+2);
        digitalWrite(DIR_PIN_BUS_3, LOW);
      }
      break;


    case AX_PRESENT_LOAD_L: //Send load information back to one of the controllers
      if (bus == 1) {
        digitalWrite(DIR_PIN_BUS_1, HIGH);
        Serial1.write((char)255);
        Serial1.write((char)255);
        Serial1.write((char)ID_NODE);
        Serial1.write((char)4); //length data
        Serial1.write((char)0); //no error byte
        Serial1.write(load_low);
        Serial1.write(load_high);
        int checksum = (~(ID_NODE + 4 + 0 + load_low + load_high)) & 0xFF;
        Serial1.write(checksum);
        delayMicroseconds(8*BYTE_TIME+2);        
        digitalWrite(DIR_PIN_BUS_1, LOW);        
      }
      else if (bus == 2) {
        digitalWrite(DIR_PIN_BUS_2, HIGH);
        Serial2.write((char)255);
        Serial2.write((char)255);
        Serial2.write((char)ID_NODE);
        Serial2.write((char)4); //length data
        Serial2.write((char)0); //no error byte
        Serial2.write(load_low);
        Serial2.write(load_high);
        int checksum = (~(ID_NODE + 4 + 0 + load_low + load_high)) & 0xFF;
        Serial2.write(checksum);
        delayMicroseconds(8*BYTE_TIME+2);
        digitalWrite(DIR_PIN_BUS_2, LOW);
      }
      else if (bus == 3) {
        digitalWrite(DIR_PIN_BUS_3, HIGH);
        Serial3.write((char)255);
        Serial3.write((char)255);
        Serial3.write((char)ID_NODE);
        Serial3.write((char)4); //length data
        Serial3.write((char)0); //no error byte
        Serial3.write(load_low);
        Serial3.write(load_high);
        int checksum = (~(ID_NODE + 4 + 0 + load_low + load_high)) & 0xFF;
        Serial3.write(checksum);
        delayMicroseconds(8*BYTE_TIME+2);
        digitalWrite(DIR_PIN_BUS_3, LOW);
      }
    break;


    case AX_PRESENT_POSITION_L: //Send position information back to one of the controllers
      if (bus == 1) {
        digitalWrite(DIR_PIN_BUS_1, HIGH);
        Serial1.write((char)255);
        Serial1.write((char)255);
        Serial1.write((char)ID_NODE);
        Serial1.write((char)4); //length data
        Serial1.write((char)0); //no error byte
        Serial1.write(pos_low);
        Serial1.write(pos_high);
        int checksum = (~(ID_NODE + 4 + 0 + pos_low + pos_high)) & 0xFF;
        Serial1.write(checksum);
        delayMicroseconds(8*BYTE_TIME+2);
        digitalWrite(DIR_PIN_BUS_1, LOW);
      }
      else if (bus == 2) {
        digitalWrite(DIR_PIN_BUS_2, HIGH);
        Serial2.write((char)255);
        Serial2.write((char)255);
        Serial2.write((char)ID_NODE);
        Serial2.write((char)4); //length data
        Serial2.write((char)0); //no error byte
        Serial2.write(pos_low);
        Serial2.write(pos_high);
        int checksum = (~(ID_NODE + 4 + 0 + pos_low + pos_high)) & 0xFF;
        Serial2.write(checksum);
        delayMicroseconds(8*BYTE_TIME+2);
        digitalWrite(DIR_PIN_BUS_2, LOW);
      }
      else if (bus == 3) {
        digitalWrite(DIR_PIN_BUS_3, HIGH);
        Serial3.write((char)255);
        Serial3.write((char)255);
        Serial3.write((char)ID_NODE);
        Serial3.write((char)4); //length data
        Serial3.write((char)0); //no error byte
        Serial3.write(pos_low);
        Serial3.write(pos_high);
        int checksum = (~(ID_NODE + 4 + 0 + pos_low + pos_high)) & 0xFF;
        Serial3.write(checksum);
        delayMicroseconds(8*BYTE_TIME+2);
        digitalWrite(DIR_PIN_BUS_3, LOW);
      }
    break;

    default:
      break;
  }

  return 1;
}//end sendValueServo_to_Controller()
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------



// Send the desired information to the controller
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------
int Respond_To_Move_Cmd(int bus) {


  if (bus == 1) {
    digitalWrite(DIR_PIN_BUS_1, HIGH);
    Serial1.write((char)255);
    Serial1.write((char)255);
    Serial1.write((char)ID_NODE);
    Serial1.write((char)2); //length data
    Serial1.write((char)error_move); //no error byte
    int checksum = (~(ID_NODE + 2 + error_move)) & 0xFF;
    Serial1.write(checksum);
    delayMicroseconds(6*BYTE_TIME+2);
    digitalWrite(DIR_PIN_BUS_1, LOW);
  }
  else if (bus == 2) {
    digitalWrite(DIR_PIN_BUS_2, HIGH);
    Serial2.write((char)255);
    Serial2.write((char)255);
    Serial2.write((char)ID_NODE);
    Serial2.write((char)2); //length data
    Serial2.write((char)error_move); //no error byte
    int checksum = (~(ID_NODE + 2 + error_move)) & 0xFF;
    Serial2.write(checksum);
    delayMicroseconds(6*BYTE_TIME+2);
    digitalWrite(DIR_PIN_BUS_2, LOW);
  }
  else if (bus == 3) {
    digitalWrite(DIR_PIN_BUS_3, HIGH);
    Serial3.write((char)255);
    Serial3.write((char)255);
    Serial3.write((char)ID_NODE);
    Serial3.write((char)2); //length data
    Serial3.write((char)error_move); //no error byte
    //Serial3.write((char)2);
    //Serial3.write((char)2);
    //Serial3.write((char)33);
    int checksum = (~(ID_NODE + 2 + error_move)) & 0xFF;
    Serial3.write(checksum);
    //int checksum = (~(2 + 2 + 33)) & 0xFF;
    //Serial3.write((char)checksum);
    delayMicroseconds(6*BYTE_TIME+2);
    digitalWrite(DIR_PIN_BUS_3, LOW);
    //Sinalize();
  }


  return 1;
}//end Respond_To_Move_Cmd()
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------





//Function to set the variable that controls the position sent to the servo
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------
int selectDatatoCheck(struct CMD_SERVO* cmd_servo){
  
  if (cmd_servo->description == AX_GOAL_POSITION_L) { 
    if (cmd_servo->id_bus == 1) {
      check_move.isbus1_received = 1;
      check_move.bus1_data1 = cmd_servo->value1;
      check_move.watchdog_bus1 = 0;
      check_move.watchdog_bus2 += 1;
      check_move.watchdog_bus3 += 1;
    }
    if (cmd_servo->id_bus == 2) {
      check_move.isbus2_received = 1;
      check_move.bus2_data1 = cmd_servo->value1;
      check_move.watchdog_bus1 += 1;
      check_move.watchdog_bus2 = 0;
      check_move.watchdog_bus3 += 1;
    }
    if (cmd_servo->id_bus == 3) {
      check_move.isbus3_received = 1;
      check_move.bus3_data1 = cmd_servo->value1;
      check_move.watchdog_bus1 += 1;
      check_move.watchdog_bus2 += 1;
      check_move.watchdog_bus3 = 0;
    }
  }
  return 1;
}//end selectDatatoCheck()
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------





//Function that attempts to read information of the states of the servo
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------
int update_servo_info(){

  //If the servo bus is not busy
  if(TX_count_temp==0 && RX_count_temp==0 && TX_count_load==0 && RX_count_load==0 && TX_count_pos==0 && RX_count_pos==0){
    if(TX_count_move == 0 && RX_count_move == 0){

      //Read temperatue if it is time
      if(millis() - time_count_temp >= READ_TEMP_INTERVAL){
        time_count_temp = millis();
        readTemperature_request(ID_NODE);
      }

      //Read position if it is time
      if(millis() - time_count_pos >= READ_POS_INTERVAL){
        time_count_pos = millis();
        readPosition_request(ID_NODE);
      }
      
      //Read load if it is time
      if(millis() - time_count_load >= READ_LOAD_INTERVAL){
        time_count_load = millis();
        readLoad_request(ID_NODE);
      }
      
    }
  }

  return 0;
}//end update_servo_info()
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------





//Convert char to unsigned integer
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------
unsigned int char_to_uint(char c) {
  unsigned int u;
  if (int(c) >= 0) {
    u = int(c);
  }
  else {
    u = 256 + int(c);
  }
  return (u);
}//end char_to_uint()
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------





//Function to controls the states of the servo bus by controllint the state of the direction pin
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------
int TX_RX_switch(){

  //Controls the direction pin when reading temperature
  //  ----------  ----------  ----------  ----------  ----------
  if(micros() - TX_count_temp > 8*BYTE_TIME+10 && TX_count_temp != 0){
    //digitalWrite(DIR_PIN_BUS_3,LOW);
    digitalWrite(DIR_PIN_BUS_SERVO,LOW);
    TX_count_temp = 0;
    RX_count_temp = micros();
    serial0Flush();
  }
  else if(micros() - RX_count_temp <= TIMEOUT_RX_TX && RX_count_temp != 0){
    //if(Serial3.available() > 6){
    if(Serial.available() > 6){
      readTemperature_receive();
      RX_count_temp = 0;
    }
  }
  else if(micros() - RX_count_temp > TIMEOUT_RX_TX && RX_count_temp != 0){
    RX_count_temp = 0;
  }
  //  ----------  ----------  ----------  ----------  ----------

  //Controls the direction pin when reading position
  //  ----------  ----------  ----------  ----------  ----------
  if(micros() - TX_count_pos > 8*BYTE_TIME+10 && TX_count_pos != 0){
    //digitalWrite(DIR_PIN_BUS_3,LOW);
    digitalWrite(DIR_PIN_BUS_SERVO,LOW);
    TX_count_pos = 0;
    RX_count_pos = micros();
    serial0Flush();
  }
  else if(micros() - RX_count_pos <= TIMEOUT_RX_TX && RX_count_pos != 0){ // 3000 eh o timeout
    //if(Serial3.available() > 7){
    if(Serial.available() > 7){
      readPosition_receive();
      RX_count_pos = 0;
    }
  }
  else if(micros() - RX_count_pos > TIMEOUT_RX_TX && RX_count_pos != 0){
    RX_count_pos = 0;
  }
  //  ----------  ----------  ----------  ----------  ----------

  //Controls the direction pin when reading load
  //  ----------  ----------  ----------  ----------  ----------
  if(micros() - TX_count_load > 8*BYTE_TIME+10 && TX_count_load != 0){
    //digitalWrite(DIR_PIN_BUS_3,LOW);
    digitalWrite(DIR_PIN_BUS_SERVO,LOW);
    TX_count_load = 0;
    RX_count_load = micros();
    serial0Flush();
  }
  else if(micros() - RX_count_load <= TIMEOUT_RX_TX && RX_count_load != 0){ // 3000 eh o timeout
    //if(Serial3.available() > 7){
    if(Serial.available() > 7){
      readLoad_receive();
      RX_count_load = 0;
    }
  }
  else if(micros() - RX_count_load > TIMEOUT_RX_TX && RX_count_load != 0){
    RX_count_load = 0;
  }
  //  ----------  ----------  ----------  ----------  ----------


  //Controls the direction pin when sending position goal
  //  ----------  ----------  ----------  ----------  ----------
  if(micros() - TX_count_move > 9*BYTE_TIME+10 && TX_count_move != 0){
  //if(micros() - TX_count_move > 300 && TX_count_move != 0){
    //digitalWrite(DIR_PIN_BUS_3,LOW);
    digitalWrite(DIR_PIN_BUS_SERVO,LOW);
    TX_count_move = 0;
    RX_count_move = micros();
    serial0Flush();
  } 
  else if(micros() - RX_count_move <= TIMEOUT_RX_TX && RX_count_move != 0){ // 2000 eh o timeout
    //if(Serial3.available() > 5){
    if(Serial.available() > 5){
      error_move = readError_move();
      RX_count_move = 0;
    }
  }
  else if(micros() - RX_count_move > TIMEOUT_RX_TX && RX_count_move != 0){
    RX_count_move = 0;
  }
  //  ----------  ----------  ----------  ----------  ----------
  
  return 0;
  
}//end TX_RX_switch()
//----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------




// Temperature reading
// ----------  ----------  ----------  ----------  ----------  ----------  ----------
//Temperature data request ----------  ----------
int readTemperature_request(unsigned char ID){

  unsigned char Checksum = (~(ID + AX_TEM_LENGTH  + AX_READ_DATA + AX_PRESENT_TEMPERATURE + AX_BYTE_READ))&0xFF;

  serial0Flush();

  digitalWrite(DIR_PIN_BUS_SERVO,HIGH);
  TX_count_temp = micros();
  Serial.write(AX_START);
  Serial.write(AX_START);
  Serial.write(ID);
  Serial.write(AX_TEM_LENGTH);
  Serial.write(AX_READ_DATA);
  Serial.write(AX_PRESENT_TEMPERATURE);
  Serial.write(AX_BYTE_READ);
  Serial.write(Checksum);
  
  return 0; 
}//end readTemperature_request()

//Reading temperature data  ----------  ----------
int readTemperature_receive(){

  int Error_Byte;
  
  unsigned char Incoming_Byte = Serial.read();
  if ( (Incoming_Byte == 255) & (Serial.peek() == 255) ){
    Serial.read();                            // Start Bytes
    Serial.read();                            // Ax-12 ID
    Serial.read();                            // Length
    if( (Error_Byte = Serial.read()) != 0 )   // Error
      return (Error_Byte*(-1));
      
    temperature = Serial.read();
    Serial.read();                             //chucksum
  }
  return 0;
  
}//end readTemperature_receive()
// ----------  ----------  ----------  ----------  ----------  ----------  ----------



// Position reading
// ----------  ----------  ----------  ----------  ----------  ----------  ----------
//Position data request  ----------  ----------
int readPosition_request(unsigned char ID){

  unsigned char Checksum = (~(ID + AX_POS_LENGTH  + AX_READ_DATA + AX_PRESENT_POSITION_L + AX_BYTE_READ_POS))&0xFF;

  serial0Flush();

  digitalWrite(DIR_PIN_BUS_SERVO,HIGH);
  TX_count_pos = micros();
  Serial.write(AX_START);
  Serial.write(AX_START);
  Serial.write(ID);
  Serial.write(AX_POS_LENGTH);
  Serial.write(AX_READ_DATA);
  Serial.write(AX_PRESENT_POSITION_L);
  Serial.write(AX_BYTE_READ_POS);
  Serial.write(Checksum);

  return 0;
  
}//end readPosition_request()

//Reading position data  ----------  ----------
int readPosition_receive(){

  int Error_Byte;
  
  unsigned char Incoming_Byte = Serial.read();
  if ( (Incoming_Byte == 255) & (Serial.peek() == 255) ){
    Serial.read();                            // Start Bytes
    Serial.read();                            // Ax-12 ID
    Serial.read();                            // Length
    if( (Error_Byte = Serial.read()) != 0 )   // Error
      return (Error_Byte*(-1));
  
    pos_low = Serial.read();            // Position Bytes
    pos_high = Serial.read();
    Serial.read();                             //chucksum
  }
  
  return 0;
  
}//end readPosition_receive()
// ----------  ----------  ----------  ----------  ----------  ----------  ----------



// Load reading
// ----------  ----------  ----------  ----------  ----------  ----------  ----------
//Load data request  ----------  ----------
int readLoad_request(unsigned char ID){

  unsigned char Checksum = (~(ID + AX_POS_LENGTH  + AX_READ_DATA + AX_PRESENT_LOAD_L + AX_BYTE_READ_POS))&0xFF;

  serial0Flush();

  digitalWrite(DIR_PIN_BUS_SERVO,HIGH);
  TX_count_load = micros();
  Serial.write(AX_START);
  Serial.write(AX_START);
  Serial.write(ID);
  Serial.write(AX_POS_LENGTH);
  Serial.write(AX_READ_DATA);
  Serial.write(AX_PRESENT_LOAD_L);
  Serial.write(AX_BYTE_READ_POS);
  Serial.write(Checksum);

  return 0;
  
}//end readLoad_request()

//Reading load data  ----------  ----------
int readLoad_receive(){

  int Error_Byte;

  unsigned char Incoming_Byte = Serial.read();
  if ( (Incoming_Byte == 255) & (Serial.peek() == 255) ){
    Serial.read();                            // Start Bytes
    Serial.read();                            // Ax-12 ID
    Serial.read();                            // Length
    if( (Error_Byte = Serial.read()) != 0 )   // Error
      return (Error_Byte*(-1));
  
    load_low = Serial.read();            // Position Bytes
    load_high = Serial.read();
    Serial.read();                             //chucksum
  }

  return 0;
  
}//end readLoad_receive()
// ----------  ----------  ----------  ----------  ----------  ----------  ----------


 


// Position command sending
// ----------  ----------  ----------  ----------  ----------  ----------  ----------
//Sending position command  ----------  ----------
int Dynamixel_move(int Position){

  unsigned char ID = ID_NODE;

  char Position_H,Position_L;
  Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
  Position_L = Position;
  unsigned char Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H))&0xFF;

  serial0Flush();

  digitalWrite(DIR_PIN_BUS_SERVO,HIGH);
  TX_count_move = micros();
  Serial.write(AX_START);                 // Send Instructions over Serial
  Serial.write(AX_START);
  Serial.write(ID);
  Serial.write(AX_GOAL_LENGTH);
  Serial.write(AX_WRITE_DATA);
  Serial.write(AX_GOAL_POSITION_L);
  Serial.write(Position_L);
  Serial.write(Position_H);
  Serial.write(Checksum);


  return (0);                 // Return the read error
   
}//end Dynamixel_move()

//Reading error messages  ----------  ----------
int readError_move(){


  int error;

  if(Serial.available() > 5){ //If the servo have responded to the command
    Serial.read(); //start
    Serial.read(); //start
    Serial.read(); //id
    Serial.read(); //length
    error = Serial.read(); //error
    Serial.read(); //checksum
  }
  else{ //If the servo have not responded to the position command
    error = ERROR_MOVE_NO_RESPONSE;
  }

  return error;
  
}//end readError_move()
// ----------  ----------  ----------  ----------  ----------  ----------  ----------





//Function that sends a position command to the servo if the servo bus is not busy
// ----------  ----------  ----------  ----------  ----------  ----------  ----------
void Try_to_send_cmd_move(int value){
  
  if(waiting_move_cmd != 0){
    
    if(TX_count_temp==0 && RX_count_temp==0 && TX_count_load==0 && RX_count_load==0 && TX_count_pos==0 && RX_count_pos==0){ //If the bus is not busy reand data
      if(TX_count_move == 0 && RX_count_move == 0){ //If the bus is not busy sending position command
        Dynamixel_move(value); // Send position command
        waiting_move_cmd = 0;
      }
    }
      
  }
}//end Try_to_send_cmd_move()
// ----------  ----------  ----------  ----------  ----------  ----------  ----------





//Function to clean the buffer
// ----------  ----------  ----------  ----------  ----------  ----------  ----------
void serial0Flush(){
  while(Serial.available() > 0) {
    Serial.read();
  }
}//end serial0Flush()
// ----------  ----------  ----------  ----------  ----------  ----------  ----------













//Function to apply the redundancy policy
// ----------  ----------  ----------  ----------  ----------  ----------  ----------
//void apply_redundancy(){
int apply_redundancy(){

  //If there is message in all three buses ----------  ----------  ----------  ----------  ----------  ----------
  if (check_move.isbus1_received + check_move.isbus2_received + check_move.isbus3_received == 3) {
    int media = (check_move.bus1_data1 + check_move.bus2_data1 + check_move.bus3_data1)/3; //Compute the average value
    float std_error = sqrt(((float)pow(check_move.bus1_data1-media,2)+(float)pow(check_move.bus2_data1-media,2)+(float)pow(check_move.bus3_data1-media,2))/2); //Compute the standard deviation
    int media12 = (check_move.bus1_data1 + check_move.bus2_data1)/2; //Compute the average value
    float std_error12 = sqrt(((float)pow(check_move.bus1_data1-media12,2)+(float)pow(check_move.bus2_data1-media12,2))/1); //Compute the standard deviation
    int media13 = (check_move.bus1_data1 + check_move.bus3_data1)/2; //Compute the average value
    float std_error13 = sqrt(((float)pow(check_move.bus1_data1-media13,2)+(float)pow(check_move.bus3_data1-media13,2))/1); //Compute the standard deviation
    int media23 = (check_move.bus2_data1 + check_move.bus3_data1)/2; //Compute the average value
    float std_error23 = sqrt(((float)pow(check_move.bus2_data1-media23,2)+(float)pow(check_move.bus3_data1-media23,2))/1); //Compute the standard deviation
    if(std_error < TOLERANCE){ //If the standard deviation is below a limit
      check_move.trust_bus1 = check_move.trust_bus1 - DECREASE_TRUST_VAR;
      if(check_move.trust_bus1<0) {check_move.trust_bus1 = 0;}
      check_move.trust_bus2 = check_move.trust_bus2 - DECREASE_TRUST_VAR;
      if(check_move.trust_bus2<0) {check_move.trust_bus2 = 0;}
      check_move.trust_bus3 = check_move.trust_bus3 - DECREASE_TRUST_VAR;
      if(check_move.trust_bus3<0) {check_move.trust_bus3 = 0;}
    }
    else{
      if(std_error12 < TOLERANCE){ //Bus 1 and Bus 2 agree
        check_move.trust_bus1 = check_move.trust_bus1 - DECREASE_TRUST_VAR;
        if(check_move.trust_bus1<0) {check_move.trust_bus1 = 0;}
        check_move.trust_bus2 = check_move.trust_bus2 - DECREASE_TRUST_VAR;
        if(check_move.trust_bus2<0) {check_move.trust_bus2 = 0;}
        check_move.trust_bus3 = check_move.trust_bus3 + INCREASE_TRUST_VAR;
        if(check_move.trust_bus3>LIM_TRUST) {check_move.trust_bus3 = LIM_TRUST;}
      }
      else if(std_error13 < TOLERANCE){ //Bus 1 and Bus 3 agree
        check_move.trust_bus1 = check_move.trust_bus1 - DECREASE_TRUST_VAR;
        if(check_move.trust_bus1<0) {check_move.trust_bus1 = 0;}
        check_move.trust_bus3 = check_move.trust_bus3 - DECREASE_TRUST_VAR;
        if(check_move.trust_bus3<0) {check_move.trust_bus3 = 0;}
        check_move.trust_bus2 = check_move.trust_bus2 + INCREASE_TRUST_VAR;
        if(check_move.trust_bus2>LIM_TRUST) {check_move.trust_bus2 = LIM_TRUST;}
      }
      else if(std_error23 < TOLERANCE){ //Bus 2 and Bus 3 agree
        check_move.trust_bus2 = check_move.trust_bus2 - DECREASE_TRUST_VAR;
        if(check_move.trust_bus2<0) {check_move.trust_bus2 = 0;}
        check_move.trust_bus3 = check_move.trust_bus3 - DECREASE_TRUST_VAR;
        if(check_move.trust_bus3<0) {check_move.trust_bus3 = 0;}
        check_move.trust_bus1 = check_move.trust_bus1 + INCREASE_TRUST_VAR;
        if(check_move.trust_bus1>LIM_TRUST) {check_move.trust_bus1 = LIM_TRUST;}
      }
      else{ //Commands from all buses disagree

        check_move.trust_bus1 = check_move.trust_bus1 + INCREASE_TRUST_VAR/5;
        if(check_move.trust_bus1>LIM_TRUST) {check_move.trust_bus1 = LIM_TRUST;}
        check_move.trust_bus2 = check_move.trust_bus2 + INCREASE_TRUST_VAR/5;
        if(check_move.trust_bus2>LIM_TRUST) {check_move.trust_bus2 = LIM_TRUST;}
        check_move.trust_bus3 = check_move.trust_bus3 + INCREASE_TRUST_VAR/5;
        if(check_move.trust_bus3>LIM_TRUST) {check_move.trust_bus3 = LIM_TRUST;}

        //SEND EMERGENCY SIGNAL BECAUSE A DISAGREEMENT IN THE COMMANDS
        if(EMERGENCY == 0){send_emergency_msg();} //PROVISORIO     AAAAAAAAAAAAAA     AAAAAAAAAAAAAA
        //EMERGENCY = 1;
        go_to_emergency_state(1);
      }
    
    }
    check_move.isbus1_received = 0;
    check_move.isbus2_received = 0;
    check_move.isbus3_received = 0;
    check_move.watchdog_bus1 = 0;
    check_move.watchdog_bus2 = 0;
    check_move.watchdog_bus3 = 0;
    if(check_move.trust_bus1+check_move.trust_bus2+check_move.trust_bus3==0){
      valor_waiting = media;
      waiting_move_cmd = 1;
      return 1;
    }
    else if(check_move.trust_bus1+check_move.trust_bus2==0){
      valor_waiting = media12;
      waiting_move_cmd = 1;
      return 1;
    }
    else if(check_move.trust_bus1+check_move.trust_bus3==0){
      valor_waiting = media13;
      waiting_move_cmd = 1;
      return 1;
    }
    else if(check_move.trust_bus2+check_move.trust_bus3==0){
      valor_waiting = media23;
      waiting_move_cmd = 1;
      return 1;
    }
    else if(check_move.trust_bus1==0){
      valor_waiting = check_move.bus1_data1;
      waiting_move_cmd = 1;
      return 1;
    }
    else if(check_move.trust_bus2==0){
      valor_waiting = check_move.bus2_data1;
      waiting_move_cmd = 1;
      return 1;
    }
    else if(check_move.trust_bus3==0){
      valor_waiting = check_move.bus3_data1;
      waiting_move_cmd = 1;
      return 1;
    }
    
  }
  //----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------


  //If there is message in two buses ----------  ----------  ----------  ----------  ----------  ----------
  else if (check_move.isbus1_received == 1 && check_move.isbus2_received == 1) {
    if(check_move.watchdog_bus3 >= LIM_WATCHDOG){
      int media12 = (check_move.bus1_data1 + check_move.bus2_data1)/2; //Compute the average value
      float std_error12 = sqrt(((float)pow(check_move.bus1_data1-media12,2)+(float)pow(check_move.bus2_data1-media12,2))/1); //Compute the standard deviation
      if(std_error12 < TOLERANCE){
        valor_waiting = media12;
        waiting_move_cmd = 1;
        return 1;
      }
      else{
        //SEND EMERGENCY SIGNAL BECAUSE A DISAGREEMENT IN THE COMMANDS
        if(EMERGENCY == 0){send_emergency_msg();} //PROVISORIO     AAAAAAAAAAAAAA
        //EMERGENCY = 1;
        go_to_emergency_state(1);
      }
      check_move.isbus1_received = 0;
      check_move.isbus2_received = 0;
      check_move.watchdog_bus1 = 0;
      check_move.watchdog_bus2 = 0;
    }
  }

  else if (check_move.isbus1_received == 1 && check_move.isbus3_received == 1) {
    if(check_move.watchdog_bus2 >= LIM_WATCHDOG){
      int media13 = (check_move.bus1_data1 + check_move.bus3_data1)/2; //Compute the average value
      float std_error13 = sqrt(((float)pow(check_move.bus1_data1-media13,2)+(float)pow(check_move.bus3_data1-media13,2))/1); //Compute the standard deviation
      if(std_error13 < TOLERANCE){
        valor_waiting = media13;
        waiting_move_cmd = 1;
        return 1;
      }
      else{
        //SEND EMERGENCY SIGNAL BECAUSE A DISAGREEMENT IN THE COMMANDS
        if(EMERGENCY == 0){send_emergency_msg();} //PROVISORIO     AAAAAAAAAAAAAA
        //EMERGENCY = 1;
        go_to_emergency_state(1);
      }
      check_move.isbus1_received = 0;
      check_move.isbus3_received = 0;
      check_move.watchdog_bus1 = 0;
      check_move.watchdog_bus3 = 0;
    }
  }

  else if (check_move.isbus2_received == 1 && check_move.isbus3_received == 1) {
    if(check_move.watchdog_bus1 >= LIM_WATCHDOG){
      int media23 = (check_move.bus2_data1 + check_move.bus3_data1)/2; //Compute the average value
      float std_error23 = sqrt(((float)pow(check_move.bus2_data1-media23,2)+(float)pow(check_move.bus3_data1-media23,2))/1); //Compute the standard deviation
      if(std_error23 < TOLERANCE){
        valor_waiting = media23;
        waiting_move_cmd = 1;
        return 1;
      }
      else{
        //SEND EMERGENCY SIGNAL BECAUSE A DISAGREEMENT IN THE COMMANDS
        if(EMERGENCY == 0){send_emergency_msg();} //PROVISORIO     AAAAAAAAAAAAAA
        //EMERGENCY = 1;
        go_to_emergency_state(1);
      }
      check_move.isbus2_received = 0;
      check_move.isbus3_received = 0;
      check_move.watchdog_bus2 = 0;
      check_move.watchdog_bus3 = 0;
    }
  }
  //----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------

  //If there is message in only one bus ----------  ----------  ----------  ----------  ----------  ----------
  else if (check_move.isbus1_received + check_move.isbus2_received + check_move.isbus3_received == 1) {
    if(check_move.watchdog_bus1 >= LIM_WATCHDOG && check_move.watchdog_bus2 >= LIM_WATCHDOG){
      valor_waiting = check_move.bus3_data1;
      waiting_move_cmd = 1;
      return 1;
      check_move.isbus3_received = 0;
      check_move.watchdog_bus3 = 0;
    }
    if(check_move.watchdog_bus1 >= LIM_WATCHDOG && check_move.watchdog_bus3 >= LIM_WATCHDOG){
      valor_waiting = check_move.bus2_data1;
      waiting_move_cmd = 1;
      return 1;
      check_move.isbus2_received = 0;
      check_move.watchdog_bus2 = 0;
    }
    if(check_move.watchdog_bus2 >= LIM_WATCHDOG && check_move.watchdog_bus3 >= LIM_WATCHDOG){
      valor_waiting = check_move.bus1_data1;
      waiting_move_cmd = 1;
      return 1;
      check_move.isbus1_received = 0;
      check_move.watchdog_bus1 = 0;
    }
    
  }
  //----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------


  return 0;
}//end apply_redundancy()
// ----------  ----------  ----------  ----------  ----------  ----------  ----------





//Function to start a emergency state
// ----------  ----------  ----------  ----------  ----------  ----------  ----------
void go_to_emergency_state(int type){
  switch(type){
    case 1:
      if (EMERGENCY == 0) {EMERGENCY = 1;}
      else if (EMERGENCY == 2) {EMERGENCY = 3;}
    break;
    case 2:
      if (EMERGENCY == 0) {EMERGENCY = 2;}
      else if (EMERGENCY == 2) {EMERGENCY = 3;}
    break;
  }
}//end go_to_emergency_state()
// ----------  ----------  ----------  ----------  ----------  ----------  ----------


//Function to leave a emergency state
// ----------  ----------  ----------  ----------  ----------  ----------  ----------
void leave_emergency_state(int type){
  switch(type){
    case 1:
      if (EMERGENCY == 1) {EMERGENCY = 0;}
      else if (EMERGENCY == 3) {EMERGENCY = 2;}
    break;
    case 2:
      if (EMERGENCY == 2) {EMERGENCY = 0;}
      else if (EMERGENCY == 3) {EMERGENCY = 1;}
    break;
  }
}//end leave_emergency_state()
// ----------  ----------  ----------  ----------  ----------  ----------  ----------



//Function to send a emergency message
// ----------  ----------  ----------  ----------  ----------  ----------  ----------
void send_emergency_msg(){

  //Wait until messages in the servo bus are complete
  int t = millis();
  while((TX_count_temp!=0 || RX_count_temp!=0 || TX_count_load!=0 || RX_count_load!=0 || TX_count_pos!=0 || RX_count_pos!=0 || TX_count_move!=0 || RX_count_move!=0) && (millis()-t<1000)){
    TX_RX_switch();
  }


  delay(5);


  int checksum = (~(ID_NODE_SIM + 3 + AX_EMERGENCY_DATA + ENTRY_EMERGENCY)) & 0xFF;
  
  //Send message repeately, 5 times
  for(int k = 0; k<5; k++){

    delay(1);
    //Send emergency data on bus 1 destinated to the symetric node
    digitalWrite(DIR_PIN_BUS_1, HIGH);
    Serial1.write((char)AX_START);
    Serial1.write((char)AX_START);
    Serial1.write((char)ID_NODE_SIM);
    Serial1.write((char)3);
    Serial1.write((char)AX_EMERGENCY_DATA);
    Serial1.write((char)ENTRY_EMERGENCY);
    //int checksum = (~(ID_NODE_SIM + 2 + ENTRY_EMERGENCY)) & 0xFF;
    Serial1.write(checksum);
    delayMicroseconds(7*BYTE_TIME+2);
    digitalWrite(DIR_PIN_BUS_1, LOW);
  
    //Send emergency data on bus 2 destinated to the symetric node
    digitalWrite(DIR_PIN_BUS_2, HIGH);
    Serial2.write((char)AX_START);
    Serial2.write((char)AX_START);
    Serial2.write((char)ID_NODE_SIM);
    Serial2.write((char)3);
    Serial2.write((char)AX_EMERGENCY_DATA);
    Serial2.write((char)ENTRY_EMERGENCY);
    //int checksum = (~(ID_NODE_SIM + 2 + ENTRY_EMERGENCY)) & 0xFF;
    Serial2.write(checksum);
    delayMicroseconds(7*BYTE_TIME+2);
    digitalWrite(DIR_PIN_BUS_2, LOW);
  
    //Send emergency data on bus 3 destinated to the symetric node
    digitalWrite(DIR_PIN_BUS_3, HIGH);
    Serial3.write((char)AX_START);
    Serial3.write((char)AX_START);
    Serial3.write((char)ID_NODE_SIM);
    Serial3.write((char)3);
    Serial3.write((char)AX_EMERGENCY_DATA);
    Serial3.write((char)ENTRY_EMERGENCY);
    //int checksum = (~(ID_NODE_SIM + 2 + ENTRY_EMERGENCY)) & 0xFF;
    Serial3.write(checksum);
    delayMicroseconds(7*BYTE_TIME+2);
    digitalWrite(DIR_PIN_BUS_3, LOW);
    
    delay(1);

  }


}//end send_emergency_msg()
// ----------  ----------  ----------  ----------  ----------  ----------  ----------




//Function to send a message to leave a emergency state
// ----------  ----------  ----------  ----------  ----------  ----------  ----------
void send_ok_msg(){

  //Wait until messages in the servo bus are complete
  int t = millis();
  while((TX_count_temp!=0 || RX_count_temp!=0 || TX_count_load!=0 || RX_count_load!=0 || TX_count_pos!=0 || RX_count_pos!=0 || TX_count_move!=0 || RX_count_move!=0) && (millis()-t<1000)){
    TX_RX_switch();
  }
  
  delay(1);

  int checksum = (~(ID_NODE_SIM + 3 + AX_EMERGENCY_DATA + LEAVE_EMERGENCY)) & 0xFF;

  //Send message repeately, 5 times
  for(int k = 0; k<5; k++){
    //Send emergency data on bus 1 destinated to the symetric node
    digitalWrite(DIR_PIN_BUS_1, HIGH);
    Serial1.write((char)AX_START);
    Serial1.write((char)AX_START);
    Serial1.write((char)ID_NODE_SIM);
    Serial1.write((char)3);
    Serial1.write((char)AX_EMERGENCY_DATA);
    Serial1.write((char)LEAVE_EMERGENCY);
    //int checksum = (~(ID_NODE_SIM + 2 + LEAVE_EMERGENCY)) & 0xFF;
    Serial1.write(checksum);
    delayMicroseconds(7*BYTE_TIME+2);
    digitalWrite(DIR_PIN_BUS_1, LOW);
  
    //Send emergency data on bus 2 destinated to the symetric node
    digitalWrite(DIR_PIN_BUS_2, HIGH);
    Serial2.write((char)AX_START);
    Serial2.write((char)AX_START);
    Serial2.write((char)ID_NODE_SIM);
    Serial2.write((char)3);
    Serial2.write((char)AX_EMERGENCY_DATA);
    Serial2.write((char)LEAVE_EMERGENCY);
    //int checksum = (~(ID_NODE_SIM + 2 + LEAVE_EMERGENCY)) & 0xFF;
    Serial2.write(checksum);
    delayMicroseconds(7*BYTE_TIME+2);
    digitalWrite(DIR_PIN_BUS_2, LOW);
  
    //Send emergency data on bus 3 destinated to the symetric node
    digitalWrite(DIR_PIN_BUS_3, HIGH);
    Serial3.write((char)AX_START);
    Serial3.write((char)AX_START);
    Serial3.write((char)ID_NODE_SIM);
    Serial3.write((char)3);
    Serial3.write((char)AX_EMERGENCY_DATA);
    Serial3.write((char)LEAVE_EMERGENCY);
    //int checksum = (~(ID_NODE_SIM + 2 + LEAVE_EMERGENCY)) & 0xFF;
    Serial3.write(checksum);
    delayMicroseconds(7*BYTE_TIME+2);
    digitalWrite(DIR_PIN_BUS_3, LOW);
    
    delay(1);
  }
  
}//end send_ok_msg()
// ----------  ----------  ----------  ----------  ----------  ----------  ----------















//Function used to realize tests
//Sinalize on pin 45
// ----------  ----------  ----------  ----------  ----------  ----------  ----------
///*
void Sinalize(){
  delayMicroseconds(1);
  digitalWrite(45,HIGH);
  delayMicroseconds(10);
  digitalWrite(45,LOW);
  delayMicroseconds(1);
}//end Sinalize()
//*/
// ----------  ----------  ----------  ----------  ----------  ----------  ----------



