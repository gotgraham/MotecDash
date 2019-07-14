
#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <EEPROM.h>

// Definitions

#define TITLE 24
#define BREAK 135
#define VALUE 152
#define UNITS 200

#define GENERAL 0
#define COOLANT_DETAIL 1
#define AIR_DETAIL 2
#define LAMBDA 3
#define AFR_DETAIL 4
#define FUEL 5
#define FUEL_PRESSURE_DETAIL 6
#define DUTY_CYCLE_DETAIL 7

#define uint16 unsigned int
#define uint8 unsigned char
#define uint32 unsigned long int

// Helpers

#define htons(x) ( ((x)<< 8 & 0xFF00) | \
                   ((x)>> 8 & 0x00FF) )
#define ntohs(x) htons(x)

#pragma pack(push, 1)  // push current alignment to stack

// Definition of the MoTec Dataset 1 packet
struct Packet
{
    uint16 rpm;
    uint16 throttle_pos;
    uint16 manifold_pressure;
    uint16 air_temperature;
    uint16 engine_temperature;
    uint16 lambda_1;
    uint16 lambda_2;
    uint16 exhaust_manifold_pressure;
    uint16 mass_air_flow;
    uint16 fuel_temperature;
    uint16 fuel_pressure;
    uint16 oil_temperature;
    uint16 oil_pressure;
    uint16 gear_voltage;
    uint16 knock_voltage;
    uint16 gear_shift_force;
    uint16 exhaust_temperature_1;
    uint16 exhaust_temperature_2;
    uint16 user_channel_1;
    uint16 user_channel_2;
    uint16 user_channel_3;
    uint16 user_channel_4;
    uint16 battery_voltage;
    uint16 ecu_temperature;
    uint16 digital_input_1_speed;
    uint16 digital_input_2_speed;
    uint16 digital_input_3_speed;
    uint16 digital_input_4_speed;
    uint16 drive_speed;
    uint16 ground_speed;
    uint16 slip;
    uint16 aim_slip;
    uint16 launch_rpm;
    uint16 lambda_1_short_term_trim;
    uint16 lambda_2_short_term_trim;
    uint16 lambda_1_long_term_trim;
    uint16 lambda_2_long_term_trim;
    uint16 aim_lambda_1;
    uint16 aim_lambda_2;
    uint16 fuel_cut_level;
    uint16 ignition_cut_level;
    uint16 ignition_advance;
    uint16 load_point;
    uint16 efficiency_point;
    uint16 fuel_used;
    uint8 auxillary_op_1_duty_cycle;
    uint8 auxillary_op_2_duty_cycle;
    uint8 auxillary_op_3_duty_cycle;
    uint8 auxillary_op_4_duty_cycle;
    uint8 auxillary_op_5_duty_cycle;
    uint8 auxillary_op_6_duty_cycle;
    uint8 auxillary_op_7_duty_cycle;
    uint8 auxillary_op_8_duty_cycle;
    uint16 fuel_actual_pulse_width;
    uint16 fuel_effective_pulse_width;
    uint16 fuel_injector_duty_cycle;
    uint16 gear;
    uint16 sync_position;
    uint16 fuel_comp_1;
    uint16 fuel_comp_2;
    uint8 diagnostic_error_group_1;
    uint8 diagnostic_error_group_2;
    uint8 diagnostic_error_group_3;
    uint8 diagnostic_error_group_4;
    uint8 diagnostic_error_group_5;
    uint8 diagnostic_error_group_6;
    uint8 diagnostic_error_group_7;
    uint8 diagnostic_error_group_8;
    uint8 diagnostic_error_group_9;
    uint8 diagnostic_error_group_10;
    uint8 diagnostic_error_group_11;
    uint8 diagnostic_error_group_12;
    uint8 diagnostic_error_group_13;
    uint8 diagnostic_error_group_14;
    uint8 diagnostic_error_group_15;
    uint8 diagnostic_error_group_16;
    uint8 diagnostic_error_group_17;
    uint8 diagnostic_error_group_18;
    uint8 diagnostic_error_group_19;
    uint8 status_flags_group_1;
    uint8 status_flags_group_2;
    uint8 status_flags_group_3;
    uint8 status_flags_group_4;
    uint8 status_flags_group_5;
    uint8 status_flags_group_6;
    uint8 status_flags_group_7;
    uint8 status_flags_group_8;
    uint8 number_of_bytes;
    uint8 marker_1;
    uint8 marker_2;
    uint8 marker_3;
    uint8 checksum;
};

#pragma pack(pop)   // restore original alignment from stack

struct ProcessedData
{
  // General
  float rpm;
  float air_temperature;
  float max_air_temperature;
  float engine_temperature;
  float max_engine_temperature;
  float ignition_advance;
  float battery_voltage;

  // Lambda
  float lambda_1;
  float lambda_1_short_term_trim;
  float lambda_1_long_term_trim;
  float aim_lambda_1;

  // Fueling
  float fuel_pressure;
  float fuel_actual_pulse_width;
  float fuel_effective_pulse_width;
  float fuel_injector_duty_cycle;
  float max_fuel_injector_duty_cycle;
} processed_data;

// Display
U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);

// Display state
uint8 displayState = GENERAL;

// Button Handing
const int buttonPin = 2;
int buttonState = HIGH;            
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// Print
void printEntry(uint8 pos, float value, uint8 precision = 1) {
  char str_value[16];
  dtostrf(value, 5, precision, str_value);
  u8g2.drawStr(VALUE, pos, str_value);
}

void printMaxEntry(float value, uint8 precision = 1) {  
  char str_value[16];
  dtostrf(value, 5, precision, str_value);
  u8g2.setFont(u8g2_font_torussansbold8_8r);
  u8g2.drawStr(208, 8, str_value);
}

void printEntryLarge(float value, uint8 precision = 1)
{
  char str_value[16];
  dtostrf(value, 5, precision, str_value);
  u8g2.setFont(u8g2_font_inb30_mf);
  u8g2.drawStr(55,50,str_value);
}

float decodePWData(uint16 value, uint16 base = 10) {
    return ((float)ntohs(value)* 0.5) / base;
}

float decodeData(uint16 value, uint16 base = 10) {
    return (float)ntohs(value) / base;
}

float decodeData(uint8 pos, uint8 value) {
    return (float)ntohs(value);
}

void pageGeneral()
{
  u8g2.setFont(u8g2_font_torussansbold8_8r);
  
  u8g2.drawStr(0,8,"General");
  
  u8g2.drawStr(TITLE,22,"RPM");
  u8g2.drawStr(TITLE,32,"Coolant Temp");
  u8g2.drawStr(TITLE,42,"Air Temp");
  u8g2.drawStr(TITLE,52,"Ignition Adv");
  u8g2.drawStr(TITLE,62,"Battery");

  u8g2.drawStr(BREAK,22,":");
  u8g2.drawStr(BREAK,32,":");
  u8g2.drawStr(BREAK,42,":");
  u8g2.drawStr(BREAK,52,":");
  u8g2.drawStr(BREAK,62,":");
  
  u8g2.drawStr(UNITS,32,"F");
  u8g2.drawStr(UNITS,42,"F");
  u8g2.drawStr(UNITS,52,"deg");
  u8g2.drawStr(UNITS,62,"v");
}

void updateGeneral()
{
  printEntry(22, processed_data.rpm, 0);
  printEntry(32, processed_data.engine_temperature);
  printEntry(42, processed_data.air_temperature);
  printEntry(52, processed_data.ignition_advance);
  printEntry(62, processed_data.battery_voltage);
}

void pageCoolantDetail()
{
  u8g2.setFont(u8g2_font_torussansbold8_8r);
  u8g2.drawStr(0,8,"Coolant");
  u8g2.drawStr(240,60,"F");
}

void updateCoolantDetail()
{
  printMaxEntry(processed_data.max_engine_temperature);
  printEntryLarge(processed_data.engine_temperature);
}

void pageAirDetail()
{
  u8g2.setFont(u8g2_font_torussansbold8_8r);
  u8g2.drawStr(0,8,"Intake Air");
  u8g2.drawStr(240,60,"F");
}

void updateAirDetail()
{
  printMaxEntry(processed_data.max_air_temperature);
  printEntryLarge(processed_data.air_temperature);
}

void pageLambda()
{  
  u8g2.setFont(u8g2_font_torussansbold8_8r);
  
  u8g2.drawStr(0,8,"Lambda");
  
  u8g2.drawStr(TITLE,22,"AFR");
  u8g2.drawStr(TITLE,32,"Aim AFR");
  u8g2.drawStr(TITLE,42,"Short Trim");
  u8g2.drawStr(TITLE,52,"Long Trim");

  u8g2.drawStr(BREAK,22,":");
  u8g2.drawStr(BREAK,32,":");
  u8g2.drawStr(BREAK,42,":");
  u8g2.drawStr(BREAK,52,":");
  
  u8g2.drawStr(UNITS,42,"%");
  u8g2.drawStr(UNITS,52,"%");
}

void updateLambda()
{  
  printEntry(22, processed_data.lambda_1);
  printEntry(32, processed_data.aim_lambda_1);
  printEntry(42, processed_data.lambda_1_short_term_trim);
  printEntry(52, processed_data.lambda_1_long_term_trim);
}

void pageAFRDetail()
{
  u8g2.setFont(u8g2_font_torussansbold8_8r);
  u8g2.drawStr(0,8,"AFR");
}

void updateAFRDetail()
{
  printEntryLarge(processed_data.lambda_1);
}

void pageFuel()
{  
  u8g2.setFont(u8g2_font_torussansbold8_8r);
  
  u8g2.drawStr(0,8,"Fuel");
  
  u8g2.drawStr(TITLE,22,"Fuel Pressure");
  u8g2.drawStr(TITLE,32,"Inj PW");
  u8g2.drawStr(TITLE,42,"Eff Inj PW");
  u8g2.drawStr(TITLE,52,"Inj Duty Cycle");

  u8g2.drawStr(BREAK,22,":");
  u8g2.drawStr(BREAK,32,":");
  u8g2.drawStr(BREAK,42,":");
  u8g2.drawStr(BREAK,52,":");

  u8g2.drawStr(UNITS,22,"psi");
  u8g2.drawStr(UNITS,32,"ms");
  u8g2.drawStr(UNITS,42,"ms");
  u8g2.drawStr(UNITS,52,"%");  
}

void updateFuel() 
{
  printEntry(22, processed_data.fuel_pressure);
  printEntry(32, processed_data.fuel_actual_pulse_width);
  printEntry(42, processed_data.fuel_effective_pulse_width);
  printEntry(52, processed_data.fuel_injector_duty_cycle);
}

void pageFuelPressureDetail()
{
  u8g2.setFont(u8g2_font_torussansbold8_8r);
  u8g2.drawStr(0,8,"Fuel Pressure");
  u8g2.drawStr(225,60,"PSI");
}

void updateFuelPressureDetail()
{
  printEntryLarge(processed_data.fuel_pressure);
}

void pageDutyCycleDetail()
{
  u8g2.setFont(u8g2_font_torussansbold8_8r);
  u8g2.drawStr(0,8,"Injector Duty Cycle");
  u8g2.drawStr(240,60,"%");
}

void updateDutyCycleDetail()
{
  printMaxEntry(processed_data.max_fuel_injector_duty_cycle);
  printEntryLarge(processed_data.fuel_injector_duty_cycle);
}

// Process the incoming byte stream until we detect the packet header
void waitForHeader(void) {
    uint32 header = 0;
    do
    {
      if (Serial1.available() > 0) {
        uint8 inByte = Serial1.read();
        header |= inByte;
        header = header << 8;
      }
    } while((header & 0xFFFFFF00) != 0xFCFBFA00);
}

// Read and process the packet
void processPacket() {
    waitForHeader();
  
    uint8 buffer[sizeof(Packet) + 1];
    Packet * packet = (Packet*)&buffer[1];
    Serial1.readBytes(buffer, sizeof(Packet));

    if(packet->number_of_bytes == 139 && packet->marker_1 == 0xFC && packet->marker_2 == 0xFB && packet->marker_3 == 0xFA) // Validate the "checksum". This will at least let us verify the packet alignment is ok
    {
      // General data
      processed_data.rpm = decodeData(packet->rpm, 1);
      
      processed_data.air_temperature = decodeData(packet->air_temperature);
      if (processed_data.air_temperature > processed_data.max_air_temperature) { processed_data.max_air_temperature = processed_data.air_temperature; }
      
      processed_data.engine_temperature = decodeData(packet->engine_temperature);
      if (processed_data.engine_temperature > processed_data.max_engine_temperature) { processed_data.max_engine_temperature = processed_data.engine_temperature; }
      
      processed_data.ignition_advance = decodeData(packet->ignition_advance);
      processed_data.battery_voltage = decodeData(packet->battery_voltage, 100);
  
      // Lambda
      processed_data.lambda_1 = decodeData(packet->lambda_1, 1000) * 14.7;
      processed_data.aim_lambda_1 = decodeData(packet->aim_lambda_1, 1000) * 14.7;
      processed_data.lambda_1_short_term_trim = decodeData(packet->lambda_1_short_term_trim);
      processed_data.lambda_1_long_term_trim = decodeData(packet->lambda_1_long_term_trim);
  
      // Fuel
      processed_data.fuel_pressure = decodeData(packet->fuel_pressure);
      processed_data.fuel_actual_pulse_width = decodePWData(packet->fuel_actual_pulse_width, 1000);
      processed_data.fuel_effective_pulse_width = decodePWData(packet->fuel_effective_pulse_width, 1000);
      
      processed_data.fuel_injector_duty_cycle = decodeData(packet->fuel_injector_duty_cycle);
      if (processed_data.fuel_injector_duty_cycle > processed_data.max_fuel_injector_duty_cycle) { processed_data.max_fuel_injector_duty_cycle = processed_data.fuel_injector_duty_cycle; }
     }
}

void setup(void)
{  
  Serial1.begin(19200);

  // Initialize Display
  u8g2.begin();

  // Clear the processed data
  memset(&processed_data, 0, sizeof(ProcessedData));

  // Mode changing button
  pinMode(buttonPin, INPUT);
  
  SPI.setClockDivider(2);

  displayState = EEPROM.read(0); // Recover the saved state

  drawPage();
}

void drawPage()
{
  u8g2.clearBuffer();
      
  switch(displayState)
  {
    case LAMBDA:
      pageLambda();
      break;
    case FUEL: 
      pageFuel();
    case COOLANT_DETAIL:
      pageCoolantDetail();
      break;
    case AIR_DETAIL:
      pageAirDetail();
      break;
    case AFR_DETAIL:
      pageAFRDetail();
      break;
    case FUEL_PRESSURE_DETAIL:
      pageFuelPressureDetail();
      break;
    case DUTY_CYCLE_DETAIL:
      pageDutyCycleDetail();
      break;
    case GENERAL:
    default:
      pageGeneral(); 
  }
}

void stateHandler() {
  int reading = digitalRead(buttonPin);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {     
    if (reading != buttonState) {
      buttonState = reading;
 
      if (buttonState == HIGH) {  
          
        if(displayState == DUTY_CYCLE_DETAIL){
          displayState = GENERAL;
        }
        else{
          ++displayState;
        }

        EEPROM.write(0, displayState);
                       
        drawPage();
      }
    }
  }
  lastButtonState = reading;
}

// Update page
void updatePage()
{
  switch(displayState)
  {
    case LAMBDA:
      updateLambda();
      break; 
    case AFR_DETAIL:
      updateAFRDetail();
      break;
    case FUEL:
      updateFuel();
      break; 
    case COOLANT_DETAIL:
      updateCoolantDetail();
      break;
    case AIR_DETAIL:
      updateAirDetail();
      break;
    case FUEL_PRESSURE_DETAIL:
      updateFuelPressureDetail();
      break;
    case DUTY_CYCLE_DETAIL:
      updateDutyCycleDetail();
      break;
    case GENERAL:
    default:
      updateGeneral();
  }
  u8g2.sendBuffer();
}

void loop(void)
{  
  stateHandler();
  updatePage();
  processPacket(); 
}

