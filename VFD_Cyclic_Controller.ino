/**
 * @file VFD_Cyclic_Controller.ino
 * @brief Cyclic Motor Controller using Modbus RTU, featuring S-Curve Acceleration
 * @author Rabin 
 * original year 2021
 * @date 2026 (Refactored/Updated)
 */

#include "LiquidCrystal_I2C.h"
#include <avr/pgmspace.h>
#include <string.h>

// --- Pin Definitions ---
#define ROTARY_PIN_A 12         
#define ROTARY_PIN_B 13         
#define BUTTON_START_STOP 14    // Analog A0 used as digital input
#define BUTTON_SET_ROTATION 2   
#define BUTTON_SET_CYCLE 3      
#define RUN_INDICATOR_PIN 11    

// --- Modbus VFD Configuration ---
#define VFD_SLAVE_ID 0x01       
#define MAX_SPEED_SCALE 50      // The max speed mapping value

// --- Operational Constants ---
#define ROTATING_CLOCKWISE 1
#define ROTATING_ANTICLOCKWISE 2
#define NO_ROTATION 0
#define PUSH_BUTTON_PRESS true
#define PUSH_BUTTON_NOT_PRESS false

// --- Target Variables ---
int number_of_rotation = 15;    
int number_of_cycle = 1;        

// --- S-Curve Variables ---
float start_speed = 0.0;
float target_speed = 0.0;
float current_speed = 0.0;
unsigned long ramp_start_time = 0;
unsigned long ramp_duration = 0;  
bool in_ramp = false;
unsigned long last_scurve_time = 0;
const unsigned long scurve_interval = 100; // Update VFD speed every 100ms

// --- Modbus Function Prototypes ---
void calculate_modbus_crc(uint8_t* pkt, int len);
uint16_t crc16_update(uint16_t crc, uint8_t a);
void forward_run(void);
void reverse_run(void);
void free_stop(void);
void set_speed(int16_t max_value, int16_t target_value);
void deceleration_stop(void);
int get_count_value(void);

// --- HMI & Control Prototypes ---
static void put_title(const char *str, uint8_t row);
void refresh_home(void);
uint8_t check_rotating_switch(void);
void set_target_speed_scurve(float target, unsigned long duration_ms);
void update_scurve_ramp(void);

// --- Button Class for Non-blocking Debounce ---
class push_button {
  private:
    uint8_t pin;
    bool debounce_flag;
    unsigned long reference_time;

  public:
    push_button(uint8_t digital_pin) {
      this->pin = digital_pin;
      this->debounce_flag = false;
      this->reference_time = 0;
      pinMode(this->pin, INPUT);
    }
    push_button() {}
    
    inline bool is_Button_push(void) {
      if (millis() > reference_time) {
        if (digitalRead(pin) == LOW && debounce_flag == true) {
          debounce_flag = false;  
          reference_time = millis() + 50; 
          return PUSH_BUTTON_PRESS;
        } else if (digitalRead(pin) == HIGH) {
          debounce_flag = true; 
          reference_time = millis() + 100;
        }
      }
      return PUSH_BUTTON_NOT_PRESS;
    }
    void reset_ref_timer(void) { reference_time = 0; }
};

// --- Object Declarations ---
LiquidCrystal_I2C lcd(0x27, 20, 4);
push_button button[3];

// --- State Machine & Tracking Variables ---
bool system_running = false;
uint8_t schedule_task_index = 0;  
int rotation_value;
int rotation_prevalue;
unsigned long state_timer = 0;
int current_cycle = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  
  button[0] = push_button(BUTTON_START_STOP);
  button[1] = push_button(BUTTON_SET_ROTATION);
  button[2] = push_button(BUTTON_SET_CYCLE);
  
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  refresh_home();
  
  pinMode(ROTARY_PIN_A, INPUT);
  pinMode(ROTARY_PIN_B, INPUT);
  pinMode(RUN_INDICATOR_PIN, OUTPUT);
}

void loop() {
  // 1. Process active speed curves constantly in the background
  update_scurve_ramp();

  // 2. Global cancel monitoring (E-Stop)
  if (system_running == true) {
    if (button[0].is_Button_push() == PUSH_BUTTON_PRESS) {
      schedule_task_index = 0;
      in_ramp = false; // Kill any active acceleration curve
      deceleration_stop();
      system_running = false;
      refresh_home();
    }
  }

  // 3. Primary State Machine
  switch (schedule_task_index) {
    case 0: // IDLE STATE
      if (system_running == false) {
        if (get_count_value() > 0) {
          deceleration_stop(); 
          digitalWrite(RUN_INDICATOR_PIN, HIGH);
          delay(100);
          digitalWrite(RUN_INDICATOR_PIN, LOW);
        }
      }
      
      // Start Execution Sequence
      if (button[0].is_Button_push() == PUSH_BUTTON_PRESS) {
        if (number_of_rotation > 0 && number_of_cycle > 0) {
          schedule_task_index = 3;
          state_timer = millis() + 100;
          digitalWrite(RUN_INDICATOR_PIN, HIGH);
          current_cycle = 0;
          
          // Trigger S-Curve Acceleration: 0 to 50 over 1500ms
          set_target_speed_scurve(MAX_SPEED_SCALE, 1500); 
          forward_run();
          system_running = true;
        }
      }
      // UI Settings Modes...
      else if (button[1].is_Button_push() == PUSH_BUTTON_PRESS) {
        schedule_task_index = 1;
        put_title("Number of Rotation", 1);
        put_title("", 2); put_title("", 3);
        rotation_value = number_of_rotation;
        rotation_prevalue = -1;
        state_timer = millis() + 5000; 
      }
      else if (button[2].is_Button_push() == PUSH_BUTTON_PRESS) { 
        schedule_task_index = 2;
        put_title("Number of Cycle", 1);
        put_title("", 2); put_title("", 3);
        rotation_value = number_of_cycle;
        rotation_prevalue = -1;
        state_timer = millis() + 5000; 
      }
      break;

    case 1: // EDIT STATE: Rotations
      {
        uint8_t status = check_rotating_switch();
        if (status == ROTATING_CLOCKWISE) rotation_value = (rotation_value < 50) ? rotation_value + 1 : 6;
        else if (status == ROTATING_ANTICLOCKWISE) rotation_value = (rotation_value > 6) ? rotation_value - 1 : 50;
        
        if (rotation_value != rotation_prevalue) {
          char buf[10];
          itoa(rotation_value, buf, 10);
          put_title(buf, 2);
          rotation_prevalue = rotation_value;
          state_timer = millis() + 5000; 
        }
        
        if (button[1].is_Button_push() == PUSH_BUTTON_PRESS) {
          schedule_task_index = 0;
          number_of_rotation = rotation_value;
          refresh_home();
        }
        if (millis() > state_timer) {
          schedule_task_index = 0; refresh_home();
        }
      }
      break;
   
    case 2: // EDIT STATE: Cycles
      {
        uint8_t status = check_rotating_switch();
        if (status == ROTATING_CLOCKWISE) rotation_value = (rotation_value < 500) ? rotation_value + 1 : 0;
        else if (status == ROTATING_ANTICLOCKWISE) rotation_value = (rotation_value > 0) ? rotation_value - 1 : 500;

        if (rotation_value != rotation_prevalue) {
          char buf[10];
          itoa(rotation_value, buf, 10);
          put_title(buf, 2);
          rotation_prevalue = rotation_value;
          state_timer = millis() + 5000;
        }

        if (button[2].is_Button_push() == PUSH_BUTTON_PRESS) {
          schedule_task_index = 0;
          number_of_cycle = rotation_value;
          refresh_home();
        }
        if (millis() > state_timer) {
          schedule_task_index = 0; refresh_home();
        }
      }
      break;

    case 3: // RUNNING STATE: Start Delay
      if (millis() > state_timer) {
        schedule_task_index = 4;
        digitalWrite(RUN_INDICATOR_PIN, LOW);
        refresh_home();
      }
      break;

    case 4: // RUNNING STATE: Monitoring Target
      {
        // When approaching target limit (-2 rotations), trigger S-Curve deceleration
        if ((get_count_value() + 2) >= number_of_rotation) { 
          schedule_task_index = 5;
          // Trigger S-Curve Deceleration: 50 down to 20 over 800ms
          set_target_speed_scurve(20, 800); 
        } 
      }
      break;

    case 5: // RUNNING STATE: Precision Stop
      {
        if (get_count_value() >= number_of_rotation) {
          in_ramp = false; // Kill curve if it's still sliding
          deceleration_stop();
          deceleration_stop(); 
          schedule_task_index = 6;  
          current_cycle++; 
          state_timer = millis() + 2000; // 2 sec idle between cycles
        }
      }
      break;

    case 6: // RUNNING STATE: Wait Between Cycles
      if (millis() > state_timer) {
        schedule_task_index = 7;
      }
      break;

    case 7: // RUNNING STATE: Loop check
      {
        if (current_cycle < number_of_cycle) {
          schedule_task_index = 3; 
          digitalWrite(RUN_INDICATOR_PIN, HIGH);
          
          // Trigger S-Curve Acceleration for next pass
          set_target_speed_scurve(MAX_SPEED_SCALE, 1500);
          forward_run(); 
          state_timer = millis() + 100; 
        } else {
          schedule_task_index = 0;
          system_running = false;
          refresh_home(); 
        }
      }
      break;
  }
}

// --- S-Curve Implementation ---

void set_target_speed_scurve(float target, unsigned long duration_ms) {
  if (target == target_speed) return; 
  
  start_speed = current_speed;
  target_speed = target;
  ramp_duration = duration_ms;
  ramp_start_time = millis();
  in_ramp = true;
}

void update_scurve_ramp() {
  if (!in_ramp) return;

  if (millis() - last_scurve_time >= scurve_interval) {
    last_scurve_time = millis();
    unsigned long elapsed = millis() - ramp_start_time;
    
    if (elapsed >= ramp_duration) {
      current_speed = target_speed;
      in_ramp = false; // Ramp finished
    } else {
      float t = (float)elapsed / (float)ramp_duration;
      float s = (1.0 - cos(t * PI)) / 2.0;
      current_speed = start_speed + (target_speed - start_speed) * s;
    }
    
    // Transmit new step to VFD
    set_speed(MAX_SPEED_SCALE, (int16_t)current_speed);
  }
}

// --- Modbus RTU Master Implementation ---

int get_count_value(void) {
  uint8_t pkt[8] = {VFD_SLAVE_ID, 0x03, 0x10, 0x0D, 0x00, 0x01, 0x00, 0x00};
  calculate_modbus_crc(pkt, 6);
  
  while (Serial.available()) Serial.read(); 
  Serial.write(pkt, 8);
  
  uint8_t rx_pkt[7]; 
  if (Serial.readBytes(rx_pkt, 7) == 7) {
    if (rx_pkt[0] == VFD_SLAVE_ID && rx_pkt[1] == 0x03 && rx_pkt[2] == 2) {
      uint16_t received_crc = ((uint16_t)rx_pkt[6] << 8) | rx_pkt[5];
      uint16_t calculated_crc = 0xFFFF;
      for (int i = 0; i < 5; i++) {
        calculated_crc = crc16_update(calculated_crc, rx_pkt[i]);
      }
      if (received_crc == calculated_crc) {
        uint16_t value = ((uint16_t)rx_pkt[3] << 8) | rx_pkt[4];
        delay(30); 
        return value;
      }
    }
  }
  delay(30); 
  return -1;
}

void forward_run(void) {
  uint8_t pkt[8] = {VFD_SLAVE_ID, 0x06, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00};
  calculate_modbus_crc(pkt, 6);
  while (Serial.available()) Serial.read();
  Serial.write(pkt, 8);
  delay(60); 
}

void reverse_run(void) {
  uint8_t pkt[8] = {VFD_SLAVE_ID, 0x06, 0x20, 0x00, 0x00, 0x02, 0x00, 0x00};
  calculate_modbus_crc(pkt, 6);
  while (Serial.available()) Serial.read();
  Serial.write(pkt, 8);
  delay(50); 
}

void free_stop(void) {
  uint8_t pkt[8] = {VFD_SLAVE_ID, 0x06, 0x20, 0x00, 0x00, 0x05, 0x00, 0x00};
  calculate_modbus_crc(pkt, 6);
  while (Serial.available()) Serial.read();
  Serial.write(pkt, 8);
  delay(50);
}

void deceleration_stop(void) {
  uint8_t pkt[8] = {VFD_SLAVE_ID, 0x06, 0x20, 0x00, 0x00, 0x06, 0x00, 0x00};
  calculate_modbus_crc(pkt, 6);
  while (Serial.available()) Serial.read();
  Serial.write(pkt, 8);
  delay(60);
}

void set_speed(int16_t max_value, int16_t target_value) {
  uint8_t pkt[8] = {VFD_SLAVE_ID, 0x06, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00};
  int16_t vfd_value = 10000 * ((float)target_value / max_value);
  
  pkt[4] = (uint8_t)(vfd_value >> 8);   
  pkt[5] = (uint8_t)(vfd_value & 0xFF); 
  
  calculate_modbus_crc(pkt, 6);
  while (Serial.available()) Serial.read();
  Serial.write(pkt, 8);
  delay(50);
}

// --- HMI Formatting & Utility Functions ---

void refresh_home(void) {
  put_title("Mantra Inc.", 0);
  char buf1[25], buf2[10];
  
  memset(buf1, 0, sizeof(buf1));
  memset(buf2, 0, sizeof(buf2));
  strcpy(buf1, "Rotation:");
  itoa(number_of_rotation, buf2, 10);
  strcat(buf1, buf2);
  put_title(buf1, 2);
  
  if (system_running == false) {
    put_title("Machine Status:Stop", 3);
    memset(buf1, 0, sizeof(buf1));
    memset(buf2, 0, sizeof(buf2));
    strcpy(buf1, "Target Cycle:");
    itoa(number_of_cycle, buf2, 10);
    strcat(buf1, buf2);
    put_title(buf1, 1);
  } else {
    put_title("Machine Status:Run", 3);
    memset(buf1, 0, sizeof(buf1));
    memset(buf2, 0, sizeof(buf2));
    strcpy(buf1, "Current Cycle:");
    itoa(current_cycle + 1, buf2, 10);
    strcat(buf1, buf2);
    put_title(buf1, 1);
  }
}

uint8_t check_rotating_switch(void) {
  static bool debounce_flag1 = false;
  static unsigned long ref_time1 = 0;

  if (millis() > ref_time1) {
    if (digitalRead(ROTARY_PIN_A) == LOW && digitalRead(ROTARY_PIN_B) == HIGH && debounce_flag1 == true) {
      debounce_flag1 = false; ref_time1 = millis() + 200; return ROTATING_CLOCKWISE;
    }  
    else if (digitalRead(ROTARY_PIN_A) == HIGH && digitalRead(ROTARY_PIN_B) == LOW && debounce_flag1 == true) {
      debounce_flag1 = false; ref_time1 = millis() + 200; return ROTATING_ANTICLOCKWISE;
    }
    else if (digitalRead(ROTARY_PIN_A) == HIGH && digitalRead(ROTARY_PIN_B) == HIGH) {
      debounce_flag1 = true;
    }
  }
  return NO_ROTATION; 
}

static void put_title(const char *str, uint8_t row) {
  uint8_t len = strlen(str);
  uint8_t start = (20 - len) / 2;
  lcd.setCursor(0, row);
  for (int i = 0; i < start; i++) lcd.print(" ");
  lcd.print(str);
  for (int i = start + len; i < 20; i++) lcd.print(" ");
}

// --- CRC16 Calculation Methods ---

uint16_t crc16_update(uint16_t crc, uint8_t a) {
  int i;
  crc ^= (uint16_t)a;
  for (i = 0; i < 8; ++i) {
    if (crc & 1) crc = (crc >> 1) ^ 0xA001;
    else crc = (crc >> 1);
  }
  return crc;
}

void calculate_modbus_crc(uint8_t* pkt, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) crc = crc16_update(crc, pkt[i]);
  pkt[len] = crc & 0xFF;         
  pkt[len + 1] = (crc >> 8) & 0xFF; 
}