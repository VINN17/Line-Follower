
/*
 * Line Follower Robot - Memory Optimized Version
 * RAM Usage: ~600 bytes (dari 800 bytes)
 * Flash Usage: ~26KB (dari 28KB)
 */

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <Wire.h>

#define COM A0
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pin definitions
#define BTN_LEFT 11
#define BTN_RIGHT 10
#define BTN_UP 9
#define BTN_DOWN 7
#define BTN_BACK 12
#define BTN_OK 8
#define BUZZER 130
#define ENA 6
#define IN1 4
#define IN2 3
#define ENB 5
#define IN3 2
#define IN4 13

// Multiplexer pins
#define S0 A6
#define S1 A3
#define S2 A2
#define S3 A1

// Constants stored in PROGMEM (Flash instead of RAM)
const uint8_t NUM_SENSORS = 8;
const uint8_t bufToCh[NUM_SENSORS] PROGMEM = { 14, 6, 12, 4, 10, 2, 8, 0 };
// const uint8_t bufToCh[NUM_SENSORS] PROGMEM = { 14, 12, 10, 8, 6, 4, 2, 0 };
// const uint8_t bufToCh[NUM_SENSORS] PROGMEM = { 6, 14, 4, 12, 2, 10, 0, 8 };
// byte bufToCh[NUM_SENSORS] = {6, 14, 4, 12, 2, 10, 0, 8};
// EEPROM addresses
#define EEPROM_MAGIC 0xAB
#define ADDR_INIT 0
#define ADDR_PID 10
#define ADDR_CALIB 50

// Sensor arrays
int sensorVal[8];
int sensorMin[8];
int sensorMax[8];
int threshold[8];
uint8_t sensorBin[8];
uint8_t sensorByte = 0;
// Menu enum
enum Menu : uint8_t {
  M_MAIN,
  M_CALIB,
  M_PID,
  M_TEST,
  M_TEST_ML,
  M_TEST_MR,
  M_TEST_SEN,
  M_START
};

// Global variables (optimized)
Menu currentMenu = M_MAIN;
uint8_t menuIdx = 0;
uint8_t testMenuIdx = 0;
uint8_t pidEditIdx = 0;
uint8_t calibStep = 0;

// Bit flags instead of multiple booleans (saves RAM)
struct {
  uint8_t editMode : 1;
  uint8_t calibrating : 1;
  uint8_t testingMotor : 1;
  uint8_t isRunning : 1;
  uint8_t unused : 4;
} flags = { 0 };

// PID values
float Kp = 2, Ki = 0.3, Kd = 1;
uint8_t baseSpeed = 150;
uint8_t testSpeed = 150;

// Position tracking
int pos = 0;
int lastPos = 0;

// Timing
uint32_t lastBtn = 0;
uint32_t calibStart = 0;

// ===== PARAMETER ROBOT =====
#define WHEEL_BASE 6.0      // Jarak antar roda (cm)
#define WHEEL_DIAMETER 3.3  // Diameter roda (cm)
#define WHEEL_RADIUS 1.65   // Radius roda (cm)
#define PI 3.14159265359
int indx = 0;
int index = 0, balik = 0;
int ip = 0;
String direct = "straight";
// pettern
uint8_t s;  // sensorByte (0-255)
bool grubKiri(uint8_t b) {
  return b & 0b11000000;  // sensor 0-1 kiri
}

bool grubTengah(uint8_t b) {
  return b & 0b00111100;  // sensor 2-5 tengah
}

bool grubKanan(uint8_t b) {
  return b & 0b00000011;  // sensor 6-7 kanan
}

bool smw_putih(uint8_t b) {
  return b == 0;
}

bool smw_hitam(uint8_t b) {
  return b == 0xFF;
}
// Lost line: tidak ada sensor yang membaca garis
bool lostLine(uint8_t b) {
  return b == 0;
}
bool detekCross(uint8_t b) {
  bool kiri = grubKiri(b);
  bool kanan = grubKanan(b);
  bool tengah = ((b & 0b00111100) == 0b00111100);  // tengah harus penuh
  return kiri && tengah && kanan;
}
bool crossLatched = false;
unsigned long crossTime = 0;

bool detectCrossLatch(uint8_t b) {
  if (detekCross(b) || detekT(b)) {
    if (!crossLatched) {
      crossLatched = true;
      crossTime = millis();
      return true;    // hanya true sekali saja
    }
  } else {
    // buka latch setelah 120ms
    if (crossLatched && millis() - crossTime > 120) {
      crossLatched = false;
    }
  }
  return false;
}

bool detekT(uint8_t b) {
  bool kiri = grubKiri(b);
  bool kanan = grubKanan(b);
  bool tengah = grubTengah(b);
  return kiri && tengah && kanan;
}
bool detekLkiri(uint8_t b) {
  return grubKiri(b) && !grubKanan(b);
}
bool detekLkanan(uint8_t b) {
  return grubKanan(b) && !grubKiri(b);
}
bool garisTengah(uint8_t b) {
  return grubTengah(b);  // minimal 1 sensor tengah baca garis
}
// ===== VARIABEL POSISI ROBOT =====
float robot_x = 0.0;        // Posisi X (cm)
float robot_y = 0.0;        // Posisi Y (cm)
float robot_heading = 0.0;  // Orientasi robot (radian)
// Menghitung keliling roda

// Function declarations
void handleButtons();
void displayMenu();
void processCalib();
void readSensors();
void updateDigital();
int calcPosition();
void stopMotors();
void setMotorL(int spd);
void setMotorR(int spd);
void saveEEPROM();
void loadEEPROM();
void beep(int freq, int dur);

float getWheelCircumference() {
  return PI * WHEEL_DIAMETER;
}
int16_t offside = 10;
float calculateDistance(unsigned long time_ms, float speed_pwm) {
  // Asumsi: PWM 255 = kecepatan maksimal (sesuaikan dengan motor Anda)
  // Contoh: PWM 255 = 10 cm/s (sesuaikan dengan kalibrasi motor Anda)
  float max_speed_cm_per_sec = 100.0 + offside;  // SESUAIKAN DENGAN MOTOR ANDA
  float actual_speed = (speed_pwm / 255.0) * max_speed_cm_per_sec;
  float distance = actual_speed * (time_ms / 1000.0);
  return distance;
}
// Menghitung waktu yang dibutuhkan untuk menempuh jarak tertentu

unsigned long calculateTime(float distance_cm, float speed_pwm) {
  // Asumsi: PWM 255 = kecepatan maksimal
  float max_speed_cm_per_sec = 100.0 + offside;  // SESUAIKAN DENGAN MOTOR ANDA
  float actual_speed = (speed_pwm / 255.0) * max_speed_cm_per_sec;

  if (actual_speed == 0) return 0;

  unsigned long time_ms = (distance_cm / actual_speed) * 1000.0;
  return time_ms;
}


// ===== FUNGSI GERAKAN DASAR =====

// Maju dengan jarak tertentu (cm)
void moveForward(float distance_cm) {
  float correction_factor = 1.15;
  float corrected_distance = distance_cm * correction_factor;
  unsigned long duration = (calculateTime(corrected_distance, baseSpeed));

  // Jalankan motor kiri dan kanan dengan kecepatan sama
  setMotorL(baseSpeed);
  setMotorR(baseSpeed);

  delay(duration);

  // Stop motor
  stopMotors();

  // Update posisi robot
  robot_x += distance_cm * cos(robot_heading);
  robot_y += distance_cm * sin(robot_heading);

  Serial.print("Forward: ");
  Serial.print(distance_cm);
  Serial.println(" cm");
  // printPosition();
}

// Mundur dengan jarak tertentu (cm)
void moveBackward(float distance_cm) {
  unsigned long duration = calculateTime(distance_cm, baseSpeed);

  // Jalankan motor kiri dan kanan mundur
  setMotorL(-baseSpeed);
  setMotorR(-baseSpeed);

  delay(duration);

  // Stop motor
  stopMotors();

  // Update posisi robot
  robot_x -= distance_cm * cos(robot_heading);
  robot_y -= distance_cm * sin(robot_heading);

  Serial.print("Backward: ");
  Serial.print(distance_cm);
  Serial.println(" cm");
  // printPosition();
}

void rot(float angle_degree) {
  float angle_rad = ((angle_degree * PI / 180.0)*3.8);

  // Hitung jarak yang harus ditempuh oleh satu roda
  // Keliling lingkaran rotasi = 2 * PI * (WHEEL_BASE/2)
  float arc_length = abs(angle_rad) * (WHEEL_BASE / 2.0);

  unsigned long duration = calculateTime(arc_length, baseSpeed);

  if (angle_degree > 0) {
    // Rotasi CCW (counter-clockwise)
    setMotorL(-250);
    setMotorR(250);
  } else {
    // Rotasi CW (clockwise)
    setMotorL(250);
    setMotorR(-250);
  }

  delay(duration);

  // Stop motor
  stopMotors();

  // Update heading
  robot_heading += angle_rad;

  // Normalisasi heading ke range -PI sampai PI
  while (robot_heading > PI) robot_heading -= 2 * PI;
  while (robot_heading < -PI) robot_heading += 2 * PI;

  Serial.print("Rotate: ");
  Serial.print(angle_degree);
  Serial.println(" degrees");
  // printPosition();
}
// Rotasi di tempat (derajat) - Positif = CCW, Negatif = CW
void rotate(float angle_degree) {
  float angle_rad = ((angle_degree * PI / 180.0)*3.8);

  // Hitung jarak yang harus ditempuh oleh satu roda
  // Keliling lingkaran rotasi = 2 * PI * (WHEEL_BASE/2)
  float arc_length = abs(angle_rad) * (WHEEL_BASE / 2.0);

  unsigned long duration = calculateTime(arc_length, baseSpeed);

  if (angle_degree > 0) {
    // Rotasi CCW (counter-clockwise)
    setMotorL(-200);
    setMotorR(200);
  } else {
    // Rotasi CW (clockwise)
    setMotorL(200);
    setMotorR(-200);
  }

  delay(duration);

  // Stop motor
  stopMotors();

  // Update heading
  robot_heading += angle_rad;

  // Normalisasi heading ke range -PI sampai PI
  while (robot_heading > PI) robot_heading -= 2 * PI;
  while (robot_heading < -PI) robot_heading += 2 * PI;

  Serial.print("Rotate: ");
  Serial.print(angle_degree);
  Serial.println(" degrees");
  // printPosition();
}

// Belok dengan radius tertentu (cm) dan sudut (derajat)
void turn(float radius_cm, float angle_degree) {
  float angle_rad = (angle_degree * PI / 180.0) * 2.5;

  // Hitung jarak untuk roda kiri dan kanan
  float inner_radius = radius_cm - (WHEEL_BASE / 2.0);
  float outer_radius = radius_cm + (WHEEL_BASE / 2.0);

  float inner_arc = abs(angle_rad) * inner_radius;
  float outer_arc = abs(angle_rad) * outer_radius;

  // Hitung kecepatan relatif
  float speed_ratio = inner_arc / outer_arc;

  unsigned long duration = calculateTime(outer_arc, baseSpeed);

  if (angle_degree > 0) {
    // Belok kiri
    setMotorL(baseSpeed * speed_ratio);
    setMotorR(baseSpeed);
  } else {
    // Belok kanan
    setMotorL(baseSpeed);
    setMotorR(baseSpeed * speed_ratio);
  }

  delay(duration);

  // Stop motor
  stopMotors();

  // Update posisi (simplified - asumsi bergerak di arc)
  float avg_distance = (inner_arc + outer_arc) / 2.0;
  robot_x += avg_distance * cos(robot_heading + angle_rad / 2.0);
  robot_y += avg_distance * sin(robot_heading + angle_rad / 2.0);
  robot_heading += angle_rad;

  // Normalisasi heading
  while (robot_heading > PI) robot_heading -= 2 * PI;
  while (robot_heading < -PI) robot_heading += 2 * PI;

  Serial.print("Turn: ");
  Serial.print(angle_degree);
  Serial.println(" degrees");
  // printPosition();
}
// ========== BUZZER (simplified) ==========
void beep(int freq, int dur) {
  tone(BUZZER, freq, dur);
}

void beepBtn() {
  beep(1000, 50);
}
void beepOK() {
  beep(1500, 80);
  delay(100);
  beep(2000, 60);
}
void beepBack() {
  beep(800, 80);
}
void beepStart() {
  beep(800, 80);
  delay(100);
  beep(1200, 80);
  delay(100);
  beep(1500, 200);
}
void beepDone() {
  beep(1000, 80);
  delay(100);
  beep(1500, 80);
  delay(100);
  beep(2000, 150);
}

// ========== MOTORS ==========
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}

void setMotorL(int spd) {
  if (spd > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, spd);
  } else if (spd < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(spd));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void setMotorR(int spd) {
  if (spd > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, spd);
  } else if (spd < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, abs(spd));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

// ========== SENSORS ==========
void setMuxCh(uint8_t ch) {
  digitalWrite(S0, ch & 1);
  digitalWrite(S1, (ch >> 1) & 1);
  digitalWrite(S2, (ch >> 2) & 1);
  digitalWrite(S3, (ch >> 3) & 1);
}

int readMuxCh(uint8_t ch) {
  setMuxCh(ch);
  delayMicroseconds(10);
  analogRead(COM);
  delayMicroseconds(10);
  return analogRead(COM);
}

void readSensors() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    uint8_t ch = pgm_read_byte(&bufToCh[i]);
    sensorVal[i] = readMuxCh(ch);
    // Serial.println("s:%d",s)
  }
  updateDigital();
}

void updateDigital() {
  sensorByte = 0;  // WAJIB: reset dulu setiap update

  for (uint8_t i = 0; i < 8; i++) {
    // convert analog → biner (1 = hitam)
    sensorBin[i] = (sensorVal[i] > threshold[i]) ? 1 : 0;

    // susun bit: sensor kiri = bit 7, sensor kanan = bit 0
    sensorByte |= (sensorBin[i] << (i));
  }

  // // --- DEBUG PRINT ---
  // Serial.print("Bin: ");
  // for (int i = 0; i < 8; i++) {
  //   Serial.print(sensorBin[i]);
  // }

  // Serial.print("   Byte: ");
  // Serial.println(sensorByte, BIN);
}


int calcPosition() {
  long wSum = 0, sum = 0;
  for (uint8_t i = 0; i < 8; i++) {
    if (sensorBin[i]) {
      wSum += (long)(i * 1000);
      sum += 1000;
    }
  }
  return (sum > 0) ? (wSum / sum) : lastPos;
}

// ========== EEPROM ==========
void saveEEPROM() {
  EEPROM.write(ADDR_INIT, EEPROM_MAGIC);

  int addr = ADDR_PID;
  EEPROM.put(addr, Kp);
  addr += sizeof(float);
  EEPROM.put(addr, Ki);
  addr += sizeof(float);
  EEPROM.put(addr, Kd);
  addr += sizeof(float);
  EEPROM.put(addr, baseSpeed);

  addr = ADDR_CALIB;
  for (uint8_t i = 0; i < 8; i++) {
    EEPROM.put(addr, sensorMin[i]);
    addr += sizeof(int);
  }
  for (uint8_t i = 0; i < 8; i++) {
    EEPROM.put(addr, sensorMax[i]);
    addr += sizeof(int);
  }
  for (uint8_t i = 0; i < 8; i++) {
    EEPROM.put(addr, threshold[i]);
    addr += sizeof(int);
  }
}

void loadEEPROM() {
  if (EEPROM.read(ADDR_INIT) != EEPROM_MAGIC) {
    // Set defaults
    for (uint8_t i = 0; i < 8; i++) {
      sensorMin[i] = 1023;
      sensorMax[i] = 0;
      threshold[i] = 512;
    }
    saveEEPROM();
    return;
  }

  int addr = ADDR_PID;
  EEPROM.get(addr, Kp);
  addr += sizeof(float);
  EEPROM.get(addr, Ki);
  addr += sizeof(float);
  EEPROM.get(addr, Kd);
  addr += sizeof(float);
  EEPROM.get(addr, baseSpeed);

  addr = ADDR_CALIB;
  for (uint8_t i = 0; i < 8; i++) {
    EEPROM.get(addr, sensorMin[i]);
    addr += sizeof(int);
  }
  for (uint8_t i = 0; i < 8; i++) {
    EEPROM.get(addr, sensorMax[i]);
    addr += sizeof(int);
  }
  for (uint8_t i = 0; i < 8; i++) {
    EEPROM.get(addr, threshold[i]);
    addr += sizeof(int);
  }
}
void setupTimer() {
  cli();  // disable interrupt

  TCCR1A = 0;  // normal mode
  TCCR1B = 0;

  // hitung: 16MHz / 64 prescaler = 250kHz
  // 250kHz / 100Hz = 2500
  OCR1A = 250;  // interrupt tiap 10ms

  TCCR1B |= (1 << WGM12);               // CTC mode
  TCCR1B |= (1 << CS11) | (1 << CS10);  // prescaler 64
  TIMSK1 |= (1 << OCIE1A);              // enable compare interrupt

  sei();  // enable interrupt
}

uint16_t tLF = 0;
int ftLF = 0;
// interrupt setiap 10ms (100Hz)
ISR(TIMER1_COMPA_vect) {
  // pidFlag = true;   // tandai PID siap dihitung
  if (flags.isRunning) {
    tLF++;
    ftLF = 1;
  }
}
// ========== SETUP ==========
void setup() {
  Serial.begin(9600);
  setupTimer();
  pinMode(BUZZER, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_OK, INPUT_PULLUP);
  pinMode(BTN_BACK, INPUT_PULLUP);

  stopMotors();
  loadEEPROM();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    beep(300, 500);
    while (1)
      ;
  }

  // Simple splash
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(25, 20);
  display.println(F("KEL 4"));
  display.setTextSize(1);
  display.setCursor(35, 45);
  display.println(F("v2.0 OPT"));
  display.display();
  beepStart();
  delay(1500);
}

// void loop() {
//   handleButtons();
//   readSensors();

//   // // updateSensorByte();      // <-- sensorByte harus update di awal!

//   if (flags.calibrating && (calibStep == 1 || calibStep == 3)) {
//     processCalib();
//   }

//   if (!flags.isRunning) {
//     index = 0;
//     ip = 0;
//   }

//   if (flags.isRunning) {

//     // // if (tLF > 10 && ftLF) {
//     // //   tLF = 0;
//     // //   ftLF = 0;
//     if (index == 0) {
//       moveForward(5);
//       index = 1;
//       // ip = 0;
//     } else if (index == 1) {
//       lurus();
//       if ((sensorByte == 0b00111100 || sensorByte == 0b11110000 || sensorByte == 0b00011111)) { index = 2; }
//     } else if (index == 2) {
//       // stopMotors();
//       turn(5, 90);
//       index = 3;
//     } else if (index == 3) {
//       // stopMotors();
//       lurus();
//       if ((sensorByte == 0b00111100 || sensorByte == 0b11110000 || sensorByte == 0b00011111)) { index = 4; }
//       // turn(5,90);
//       // index=4;
//     } else if (index == 4) {
//       stopMotors();
//       index = 5;
//     }
//   }

//   // // }

//   displayMenu();
//   // setMotorL(100);
// }


void loop() {
  handleButtons();
  readSensors();

  if (flags.calibrating && (calibStep == 1 || calibStep == 3)) {
    processCalib();
  }

  if (!flags.isRunning) {
    index = 0;
    ip = 0;
  }

  if (flags.isRunning) {
    loopLF();
    // if (index == 0) {
    //   moveForward(5);
    //   index = 1;
    // }
    // if (index == 1) {
    //   follow_PID();  // garis 5-2

    //   // kena garis 2x cross, kiri
    //   if (balik <= 2) {
    //     if (detekCross(sensorByte)) {
    //       turn(5, -90);
    //       stopMotors();
    //       // index = 1;
    //       balik += 1;
    //     }
    //   }
    //   // kena garis 2x cross, kanan
    //   else if (balik > 2 && balik <= 4) {
    //     if (detekCross(sensorByte)) {
    //       turn(5, 90);
    //       stopMotors();
    //       // index = 1;
    //       balik += 1;
    //     }
    //   }
    //   // kena garis cross
    //   else if (balik > 4) {
    //     if (detekCross(sensorByte)) {
    //       stopMotors();
    //       index = 2;
    //       balik=0;
    //     }
    //   }
    // }
    // if(index==2){
    //   rotate(180);
    //   stopMotors();
    //   index=3;
    // }
  }
  displayMenu();
}

// void lurus() {
//   float pid, error, lasterror, p, i, d, x;
//   int setpoint = 3;

//   pos = calcPosition();
//   lastPos = pos;
//   error = pos - setpoint;
//   // PID logic here
//   p = error;
//   i = i + error * 0.001;
//   if (i > 10) i = 10;
//   if (i < -10) i = -10;
//   d = error - lasterror;
//   pid = p * Kp + i * Ki + d * Kd;
//   if (pid > 100) pid = 100;
//   if (pid < -100) pid = -100;
//   setMotorL(baseSpeed + pid);
//   setMotorR(baseSpeed - pid);
//   lasterror = error;
// }

void follow_PID() {
  float pid, error, lasterror, p, i, d, x;
  int setpoint = 3;

  pos = calcPosition();
  lastPos = pos;
  error = pos - setpoint;
  // PID logic here
  p = error;
  i = i + error * 0.001;
  if (i > 10) i = 10;
  if (i < -10) i = -10;
  d = error - lasterror;
  pid = p * Kp + i * Ki + d * Kd;
  if (pid > 100) pid = 100;
  if (pid < -100) pid = -100;
  setMotorL(baseSpeed - pid);
  setMotorR(baseSpeed + pid);
  lasterror = error;
}

// ========== BUTTON HANDLER ==========
void handleButtons() {
  if (millis() - lastBtn < 200) return;

  if (digitalRead(BTN_UP) == LOW) {
    lastBtn = millis();
    beepBtn();

    if (currentMenu == M_MAIN) {
      menuIdx = (menuIdx == 0) ? 3 : menuIdx - 1;
    } else if (currentMenu == M_TEST) {
      testMenuIdx = (testMenuIdx == 0) ? 2 : testMenuIdx - 1;
    } else if (currentMenu == M_PID && flags.editMode) {
      if (pidEditIdx == 3) {
        baseSpeed = min(255, baseSpeed + 10);
      } else {
        float* vals[] = { &Kp, &Ki, &Kd };
        *vals[pidEditIdx] = min(100.0f, *vals[pidEditIdx] + 1.0f);
      }
      saveEEPROM();
    } else if (currentMenu == M_TEST_ML || currentMenu == M_TEST_MR) {
      testSpeed = min(255, testSpeed + 10);
    }
  } else if (digitalRead(BTN_DOWN) == LOW) {
    lastBtn = millis();
    beepBtn();

    if (currentMenu == M_MAIN) {
      menuIdx = (menuIdx == 3) ? 0 : menuIdx + 1;
    } else if (currentMenu == M_TEST) {
      testMenuIdx = (testMenuIdx == 2) ? 0 : testMenuIdx + 1;
    } else if (currentMenu == M_PID && flags.editMode) {
      if (pidEditIdx == 3) {
        baseSpeed = max(0, baseSpeed - 10);
      } else {
        float* vals[] = { &Kp, &Ki, &Kd };
        *vals[pidEditIdx] = max(0.0f, *vals[pidEditIdx] - 1.0f);
      }
      saveEEPROM();
    } else if (currentMenu == M_TEST_ML || currentMenu == M_TEST_MR) {
      testSpeed = max(0, testSpeed - 10);
    }
  } else if (digitalRead(BTN_LEFT) == LOW) {
    lastBtn = millis();
    beepBtn();

    if (currentMenu == M_PID && !flags.editMode) {
      pidEditIdx = (pidEditIdx == 0) ? 3 : pidEditIdx - 1;
    } else if (currentMenu == M_PID && flags.editMode && pidEditIdx != 3) {
      float* vals[] = { &Kp, &Ki, &Kd };
      *vals[pidEditIdx] = max(0.0f, *vals[pidEditIdx] - 0.1f);
      saveEEPROM();
    }
  } else if (digitalRead(BTN_RIGHT) == LOW) {
    lastBtn = millis();
    beepBtn();

    if (currentMenu == M_PID && !flags.editMode) {
      pidEditIdx = (pidEditIdx == 3) ? 0 : pidEditIdx + 1;
    } else if (currentMenu == M_PID && flags.editMode && pidEditIdx != 3) {
      float* vals[] = { &Kp, &Ki, &Kd };
      *vals[pidEditIdx] = min(100.0f, *vals[pidEditIdx] + 0.1f);
      saveEEPROM();
    }
  } else if (digitalRead(BTN_OK) == LOW) {
    lastBtn = millis();
    beepOK();

    if (currentMenu == M_MAIN) {
      switch (menuIdx) {
        case 0:
          currentMenu = M_CALIB;
          calibStep = 0;
          flags.calibrating = 0;
          break;
        case 1:
          currentMenu = M_PID;
          pidEditIdx = 0;
          flags.editMode = 0;
          break;
        case 2:
          currentMenu = M_TEST;
          testMenuIdx = 0;
          break;
        case 3:
          currentMenu = M_START;
          flags.isRunning = 1;
          beepStart();
          break;
      }
    } else if (currentMenu == M_TEST) {
      switch (testMenuIdx) {
        case 0:
          currentMenu = M_TEST_ML;
          flags.testingMotor = 0;
          testSpeed = 150;
          break;
        case 1:
          currentMenu = M_TEST_MR;
          flags.testingMotor = 0;
          testSpeed = 150;
          break;
        case 2: currentMenu = M_TEST_SEN; break;
      }
    } else if (currentMenu == M_CALIB) {
      if (calibStep == 0) {
        calibStep = 1;
        flags.calibrating = 1;
        calibStart = millis();
        for (uint8_t i = 0; i < 8; i++) sensorMin[i] = 1023;
      } else if (calibStep == 2) {
        calibStep = 3;
        flags.calibrating = 1;
        calibStart = millis();
        for (uint8_t i = 0; i < 8; i++) sensorMax[i] = 0;
      } else if (calibStep == 4) {
        calibStep = 0;
        currentMenu = M_MAIN;
      }
    } else if (currentMenu == M_PID) {
      flags.editMode = !flags.editMode;
    } else if (currentMenu == M_TEST_ML) {
      flags.testingMotor = !flags.testingMotor;
      if (flags.testingMotor) setMotorL(testSpeed);
      else stopMotors();
    } else if (currentMenu == M_TEST_MR) {
      flags.testingMotor = !flags.testingMotor;
      if (flags.testingMotor) setMotorR(testSpeed);
      else stopMotors();
    }
  } else if (digitalRead(BTN_BACK) == LOW) {
    lastBtn = millis();
    beepBack();

    if (currentMenu == M_TEST_ML || currentMenu == M_TEST_MR || currentMenu == M_TEST_SEN) {
      currentMenu = M_TEST;
      stopMotors();
      flags.testingMotor = 0;
    } else if (currentMenu != M_MAIN) {
      currentMenu = M_MAIN;
      flags.editMode = 0;
      flags.calibrating = 0;
      flags.isRunning = 0;
      stopMotors();
    }
  }
}

// ========== CALIBRATION ==========
void processCalib() {
  uint32_t elapsed = millis() - calibStart;

  if (elapsed < 3000) {
    readSensors();

    if (calibStep == 1) {
      for (uint8_t i = 0; i < 8; i++) {
        if (sensorVal[i] < sensorMin[i]) sensorMin[i] = sensorVal[i];
      }
    } else if (calibStep == 3) {
      for (uint8_t i = 0; i < 8; i++) {
        if (sensorVal[i] > sensorMax[i]) sensorMax[i] = sensorVal[i];
      }
    }
  } else {
    flags.calibrating = 0;

    if (calibStep == 1) {
      calibStep = 2;
      beepOK();
    } else if (calibStep == 3) {
      for (uint8_t i = 0; i < 8; i++) {
        threshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
      }
      saveEEPROM();
      calibStep = 4;
      beepDone();
    }
  }
}

// ========== DISPLAY (simplified) ==========
void displayMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  switch (currentMenu) {
    case M_MAIN:
      {
        display.setCursor(10, 0);
        display.println(F("LINE FOLLOWER"));
        const char* items[] = { "Kalibrasi", "Setting PID", "Testing", "START" };
        for (uint8_t i = 0; i < 4; i++) {
          display.setCursor(5, 15 + i * 12);
          if (i == menuIdx) display.print(F(">"));
          else display.print(F(" "));
          display.print(items[i]);
        }
        break;
      }

    case M_CALIB:
      {
        display.setCursor(20, 0);
        display.println(F("KALIBRASI"));

        if (calibStep == 0) {
          display.setCursor(10, 20);
          display.println(F("Taruh di PUTIH"));
          display.setCursor(10, 35);
          display.println(F("Tekan OK"));
        } else if (calibStep == 1) {
          display.setCursor(10, 20);
          display.println(F("Scanning..."));
          uint8_t prog = map(millis() - calibStart, 0, 3000, 0, 100);
          display.drawRect(10, 35, 108, 10, SSD1306_WHITE);
          display.fillRect(12, 37, prog * 104 / 100, 6, SSD1306_WHITE);
        } else if (calibStep == 2) {
          display.setCursor(10, 20);
          display.println(F("Taruh di HITAM"));
          display.setCursor(10, 35);
          display.println(F("Tekan OK"));
        } else if (calibStep == 3) {
          display.setCursor(10, 20);
          display.println(F("Scanning..."));
          uint8_t prog = map(millis() - calibStart, 0, 3000, 0, 100);
          display.drawRect(10, 35, 108, 10, SSD1306_WHITE);
          display.fillRect(12, 37, prog * 104 / 100, 6, SSD1306_WHITE);
        } else {
          display.setTextSize(2);
          display.setCursor(30, 25);
          display.println(F("DONE!"));
        }
        break;
      }

    case M_PID:
      {
        display.setCursor(20, 0);
        display.println(F("PID TUNING"));
        const char* names[] = { "Kp", "Ki", "Kd", "Spd" };

        for (uint8_t i = 0; i < 4; i++) {
          display.setCursor(5, 15 + i * 11);
          if (i == pidEditIdx) display.print(F(">"));
          else display.print(F(" "));
          display.print(names[i]);
          display.print(F(":"));
          display.setCursor(60, 15 + i * 11);

          if (i == 0) display.print(Kp, 1);
          else if (i == 1) display.print(Ki, 1);
          else if (i == 2) display.print(Kd, 1);
          else display.print(baseSpeed);

          if (i == pidEditIdx && flags.editMode) {
            display.print(F(" <"));
          }
        }
        break;
      }

    case M_TEST:
      {
        display.setCursor(30, 0);
        display.println(F("TESTING"));
        const char* items[] = { "Motor Kiri", "Motor Kanan", "Sensor" };
        for (uint8_t i = 0; i < 3; i++) {
          display.setCursor(10, 18 + i * 14);
          if (i == testMenuIdx) display.print(F(">"));
          else display.print(F(" "));
          display.print(items[i]);
        }
        break;
      }

    case M_TEST_ML:
      {
        display.setCursor(10, 0);
        display.println(F("TEST MOTOR L"));
        display.setCursor(10, 20);
        display.print(F("Speed: "));
        display.print(testSpeed);
        display.setCursor(10, 35);
        display.print(flags.testingMotor ? F("RUNNING") : F("STOPPED"));
        break;
      }

    case M_TEST_MR:
      {
        display.setCursor(10, 0);
        display.println(F("TEST MOTOR R"));
        display.setCursor(10, 20);
        display.print(F("Speed: "));
        display.print(testSpeed);
        display.setCursor(10, 35);
        display.print(flags.testingMotor ? F("RUNNING") : F("STOPPED"));
        break;
      }

    case M_TEST_SEN:
      {
        display.setCursor(20, 0);
        display.println(F("TEST SENSOR"));
        for (uint8_t i = 0; i < 8; i++) {
          int h = map(sensorVal[i], 0, 1023, 0, 20);
          display.fillRect(i * 15 + 4, 45 - h, 12, h, SSD1306_WHITE);
          display.drawRect(i * 15 + 4, 25, 12, 20, SSD1306_WHITE);
          if (sensorBin[i]) {
            display.fillRect(i * 15 + 5, 50, 10, 8, SSD1306_WHITE);
          }
        }
        break;
      }

    case M_START:
      {
        display.setCursor(20, 0);
        display.println(F("RUNNING"));
        display.setCursor(5, 15);
        display.print(F("Pos: "));
        display.print(pos);

        for (uint8_t i = 0; i < 8; i++) {
          if (sensorBin[i]) {
            display.fillRect(i * 15 + 4, 35, 12, 10, SSD1306_WHITE);
          } else {
            display.drawRect(i * 15 + 4, 35, 12, 10, SSD1306_WHITE);
          }
        }

        display.setCursor(5, 50);
        display.print(F("P:"));
        display.print(Kp, 1);
        display.print(F(" I:"));
        display.print(Ki, 1);
        display.print(F(" D:"));
        display.print(Kd, 1);
        break;
      }
  }

  display.display();
}

bool wasCross = false;
// bool crossLatched = false;
int turnx = 1;
int rotateLF=97;
// int rotateLF=110;
int lock=0;
// void loopLF() {

//   switch (index) {

//     case 0:
//       moveForward(8);
//       index = 1;
//       break;

//     case 1:


//       if (detekCross(sensorByte)) {
//         stopMotors();
//         turn(turnx, -90);  // kiri
//         stopMotors();
//         index = 2;
//       } else follow_PID();

//       break;

//     case 2:
//       // follow_PID();
//       if (detekCross(sensorByte)) {
//         stopMotors();
//         turn(turnx, -90);  // kiri
//         stopMotors();
//         index = 3;
//       } else follow_PID();
//       break;

//     case 3:

//       if (detekCross(sensorByte)) {
//         stopMotors();
//         turn(turnx, 110);  // kanan
//         stopMotors();
//         index = 4;
//       } else follow_PID();
//       break;
//     case 4:
//       // follow_PID();
//       if (detekCross(sensorByte) || detekT(sensorByte)) {
//         stopMotors();
//         turn(turnx, 110);  // kanan
//         stopMotors();
//         index = 5;
//       } else follow_PID();
//       break;
//     case 5:

//       if (detekCross(sensorByte)) {
//         stopMotors();
//         turn(turnx, -110);  // kiri
//         stopMotors();
//         index = 6;
//       } else follow_PID();
//       break;
//     case 6:

//       if (detekCross(sensorByte) || detekT(sensorByte)) {
//         stopMotors();
//         turn(turnx, -110);  // kiri
//         stopMotors();
//         index = 7;
//       } else follow_PID();
//       break;
//     case 7:

//       if (detekCross(sensorByte)) {
//         stopMotors();
//         turn(turnx, 110);  // kanan
//         stopMotors();
//         index = 8;
//       } else follow_PID();
//       break;
//     case 8:
//       // follow_PID();
//       if (detekCross(sensorByte) || detekT(sensorByte)) {
//         stopMotors();
//         turn(turnx, 110);  // kanan
//         stopMotors();
//         index = 9;
//       } else follow_PID();
//       break;
//     case 9:
//       // follow_PID();
//       if (detekCross(sensorByte)) {
//         stopMotors();
//         rotate(388);  // kanan
//         stopMotors();
//         index = 10;
//       } else follow_PID();
//       break;
//     case 10:
//       // follow_PID();
//       if (detekCross(sensorByte)) {
//         stopMotors();
//         turn(turnx, 110);  // kanan
//         // rotate(388);  // kanan
//         stopMotors();
//         index = 11;
//       } else follow_PID();
//       break;
//     case 11:
//       // follow_PID();
//       if (detekCross(sensorByte)) {
//         stopMotors();
//         turn(turnx, 110);  // kanan
//         // rotate(388);  // kanan
//         stopMotors();
//         index = 12;
//       } else follow_PID();
//       break;
//     case 12:
//       // follow_PID();
//       if (detekCross(sensorByte)) {
//         stopMotors();
//         turn(turnx, -110);  // kiri
//         stopMotors();
//         index = 13;
//       } else follow_PID();
//       break;
//     case 13:
//       // follow_PID();
//       if (detekCross(sensorByte)) {
//         stopMotors();
//         turn(turnx, -110);  // kiri
//         stopMotors();
//         index = 14;
//       } else follow_PID();
//       break;
//     case 14:
//       // follow_PID();
//       if (detekCross(sensorByte)) {
//         stopMotors();
//         turn(turnx, 110);  // kanan
//         stopMotors();
//         index = 15;
//       } else follow_PID();
//       break;
//     case 15:
//       // follow_PID();
//       if (detekCross(sensorByte)) {
//         stopMotors();
//         turn(turnx, 110);  // kanan
//         stopMotors();
//         index = 16;
//       } else follow_PID();
//       break;
//     case 16:
//       // follow_PID();
//       if (detekCross(sensorByte)) {
//         stopMotors();
//         turn(turnx, -110);  // kiri
//         stopMotors();
//         index = 17;
//       } else follow_PID();
//       break;
//     case 17:
//       // follow_PID();
//       if (detekCross(sensorByte)) {
//         stopMotors();
//         turn(turnx, -110);  // kiri
//         stopMotors();
//         index = 18;
//       } else follow_PID();
//       break;
//     case 18:
//       // follow_PID();
//       if (detekCross(sensorByte)) {
//         stopMotors();
//         rotate(388);  // kanan
//         stopMotors();
//         index = 19;
//       }else follow_PID();
//       break;
//   }
// }


void loopLF() {

  switch (index) {

    case 0:
      moveForward(10);
      // rotate(30);
      // rot(-90);
      index = 1;
      break;

    case 1:
      if (detectCrossLatch(sensorByte)||detekT(sensorByte)) {
        // stopMotors();stopMotors();stopMotors();
        // turn(turnx, -rotateLF);  // kiri
        // rotate(-rotateLF);
        rot(-rotateLF);
        // delay(20);
        // stopMotors();
        index = 2;
      } else follow_PID();

      break;

    case 2:
      // follow_PID();
      if (detectCrossLatch(sensorByte)) {
        stopMotors();stopMotors();stopMotors();
        // turn(turnx, -rotateLF);  // kiri
        // rotate(-rotateLF-20);
        rot(-rotateLF);
        // delay(20);
        stopMotors();
        index = 3;
      } else follow_PID();
      break;

    case 3:

      if (detectCrossLatch(sensorByte)) {
        stopMotors();
        // turn(turnx, rotateLF);  // kanan
        rotate(rotateLF-50); 
        // delay(20);
        stopMotors();
        index = 4;
      } else follow_PID();
      break;
    case 4:
      // follow_PID();
      if (detectCrossLatch(sensorByte)) {
        stopMotors();
        // turn(turnx, rotateLF);  // kanan
        rotate(rotateLF-50); 
        // delay(20);
        stopMotors();
        index = 5;
      } else follow_PID();
      break;
    case 5:

      if (detectCrossLatch(sensorByte)) {
        stopMotors();
        // turn(turnx, -rotateLF);  // kiri
        // rotate(-rotateLF); 
        rot(-rotateLF+10);
        // delay(20);
        stopMotors();
        index = 6;
      } else follow_PID();
      break;
    case 6:

      if (detectCrossLatch(sensorByte)||detekT(sensorByte)) {
        stopMotors();
        // turn(turnx, -rotateLF);  // kiri
        // rotate(-rotateLF-40); 
        rot(-rotateLF+25);
        // delay(20);
        stopMotors();
        index = 7;
      } else follow_PID();
      break;
    case 7:

      if (detectCrossLatch(sensorByte)) {
        stopMotors();
        // turn(turnx, rotateLF);  // kanan
        rotate(rotateLF-50); 
        // delay(20);
        stopMotors();
        index = 8;
      } else follow_PID();
      break;
    case 8:
      // follow_PID();
      if (detectCrossLatch(sensorByte)) {
        stopMotors();
        // turn(turnx, rotateLF);  // kanan
        rotate(rotateLF-50); 
        // delay(20);
        stopMotors();
        index = 9;
      } else follow_PID();
      break;
    case 9:
      // follow_PID();
      if (detectCrossLatch(sensorByte)||detekT(sensorByte)) {
        stopMotors();stopMotors();stopMotors();stopMotors();
        delay(100);
        rot(90-20);  // kanan
        stopMotors();stopMotors();stopMotors();
        delay(3000);
        index = 10;
      } else follow_PID();
      break;
    case 10:
      // follow_PID();
      if (detectCrossLatch(sensorByte)) {
        stopMotors();
        // turn(turnx, rotateLF);  // kanan
        // rot(rotateLF);
        rot(30);
        // rotate(388);  // kanan
        stopMotors();
        index = 11;
      } else follow_PID();
      break;
    case 11:
      // follow_PID();
      if (detectCrossLatch(sensorByte)||detekLkanan(sensorByte)) {
        stopMotors();
        // turn(turnx, rotateLF);  // kanan
        rotate(40); 
        // rotate(388);  // kanan
        stopMotors();
        index = 12;
      } else follow_PID();
      break;
    case 12:
      // follow_PID();
      if (detectCrossLatch(sensorByte)) {
        stopMotors();
        // turn(turnx, -rotateLF);  // kiri
        // rotate(-rotateLF-30); 
        rot(-rotateLF-30);
        stopMotors();
        index = 13;
      } else follow_PID();
      break;
    case 13:
      // follow_PID();
      if (detectCrossLatch(sensorByte)) {
        stopMotors();
        // turn(turnx, -rotateLF);  // kiri
        // rotate(-rotateLF); 
        rot(-rotateLF-30);
        stopMotors();
        index = 14;
      } else follow_PID();
      break;
    case 14:
      // follow_PID();
      if (detectCrossLatch(sensorByte)) {
        stopMotors();
        // turn(turnx, rotateLF);  // kanan
        rotate(rotateLF-50); 
        stopMotors();
        index = 15;
      } else follow_PID();
      break;
    case 15:
      // follow_PID();
      if (detectCrossLatch(sensorByte)) {
        stopMotors();
        // turn(turnx, rotateLF);  // kanan
        rotate(rotateLF-50); 
        stopMotors();
        index = 16;
      } else follow_PID();
      break;
    case 16:
      // follow_PID();
      if (detectCrossLatch(sensorByte)) {
        stopMotors();
        // turn(turnx, -rotateLF);  // kiri
        // rotate(-rotateLF); 
        rot(-rotateLF-30);
        stopMotors();
        index = 17;
      } else follow_PID();
      break;
    case 17:
      // follow_PID();
      if (detectCrossLatch(sensorByte)) {
        stopMotors();
        // turn(turnx, -rotateLF);  // kiri
        // rotate(-rotateLF); 
        rot(-rotateLF);
        stopMotors();
        index = 18;
      } else follow_PID();
      break;
    case 18:
      // follow_PID();
      if (detectCrossLatch(sensorByte)) {
        stopMotors();
        // rotate(388);  // kanan
        stopMotors();
        index = 19;
      }else follow_PID();
      break;
  }
  // setMotorR(-100);
}
