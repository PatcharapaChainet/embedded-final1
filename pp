#include <WiFi.h>      // ไลบรารีสำหรับจัดการ WiFi บน ESP32
#include <MQTT.h>      // ไลบรารีสำหรับการสื่อสารโปรโตคอล MQTT
#include <Stepper.h>   // ไลบรารีสำหรับควบคุม Stepper Motor

// --- การตั้งค่าเครือข่ายและ Broker ---
const char ssid[] = "netGu";              // ชื่อ WiFi
const char pass[] = "12345678";           // รหัสผ่าน WiFi
const char mqtt_broker[] = "test.mosquitto.org"; // ที่อยู่ MQTT Broker สาธารณะ
const char mqtt_client_id[] = "esp32AI14";       // ID ของอุปกรณ์เรา (ต้องไม่ซ้ำกับคนอื่น)
int MQTT_PORT = 1883;                     // พอร์ตมาตรฐานของ MQTT
unsigned long lastMsgTime = 0;            // ตัวแปรเก็บเวลาล่าสุดที่ส่งข้อความ
const long interval = 1000;               // ตั้งเวลาส่งข้อมูลทุก 1 วินาที (1000ms)

// --- ชื่อหัวข้อ (Topics) สำหรับรับ-ส่งข้อมูล ---
const char mqtt_status[]     = "group14/test1/status";      // ส่งสถานะการทำงาน (Working/Reverse)
const char mqtt_mode[]       = "group14/test1/mode";        // รับค่าเปลี่ยนโหมด (Auto/Manual)
const char mqtt_modestatus[] = "group14/test1/modestatus";  // ส่งสถานะโหมดปัจจุบัน
const char mqtt_motor[]      = "group14/test1/motor";       // รับค่าสั่งหมุนมอเตอร์ (Manual)
const char mqtt_motorstatus[] = "group14/test1/motorstatus"; // ส่งสถานะมอเตอร์
const char mqtt_ldr[]        = "group14/test1/ldr";         // ส่งค่าแสงที่อ่านได้
const char mqtt_angle[]      = "group14/test1/angle";       // ส่งองศาปัจจุบันของมอเตอร์
const char mqtt_reset[]      = "group14/test1/reset";       // รับคำสั่ง Reset กลับจุดเริ่มต้น
const char mqtt_lock[]       = "group14/test1/lock";        // รับคำสั่ง Lock การหมุน
const char mqtt_lockstatus[] = "group14/test1/lockstatus";  // ส่งสถานะ Lock/Unlock

// --- กำหนดขา Pin ---
#define LDR     32   // ขา Analog สำหรับอ่านค่าเซนเซอร์แสง
#define LED     17   // ขา Digital สำหรับไฟสถานะ
#define BTN     34   // ขา Digital สำหรับปุ่มกดเปลี่ยนโหมด

// --- ตั้งค่า Stepper Motor ---
const int stepsPerRevolution = 2048; // จำนวน Step ต่อการหมุน 1 รอบ (สำหรับ 28BYJ-48)
// กำหนดขาเชื่อมต่อ Stepper (IN1, IN3, IN2, IN4)
Stepper myStepper(stepsPerRevolution, 23, 22, 21, 19);

WiFiClient net;      // สร้าง Object สำหรับการเชื่อมต่อ Network
MQTTClient client;   // สร้าง Object สำหรับจัดการ MQTT

// --- ตัวแปรสถานะภายในโปรแกรม ---
bool autoMode = true;      // สถานะโหมด: true = อัตโนมัติ, false = แมนนวล
int motorState = 0;        // สถานะมอเตอร์: 0=หยุด, 1=หมุนตาม, 2=หมุนทวน
bool autoStarted = true;   // เช็คสถานะการเริ่มต้นโหมด Auto
bool btnLast = LOW;        // เก็บสถานะปุ่มกดก่อนหน้าเพื่อตรวจจับการกด
bool lock = false;         // สถานะการล็อคการหมุน
long currentAngle = 0;     // เก็บค่าสะสมขององศาที่หมุนไป
int ldr_value = 0;         // เก็บค่าแสงที่อ่านได้จาก LDR

// --- ฟังก์ชันรับข้อความจาก MQTT (Callback) ---
void messageReceived(String &topic, String &payload) { 
  
  // ถ้าได้รับข้อความในหัวข้อโหมด ให้สลับค่า autoMode
  if (topic == mqtt_mode) { 
    Serial.println("Incoming: " + topic + " - " + payload);
    autoMode = !autoMode;
  } 
  
  // ถ้าเป็นโหมด Manual และได้รับคำสั่งหมุนมอเตอร์ ให้เปลี่ยน motorState ตาม payload
  if (topic == mqtt_motor && !autoMode) {  
    Serial.println("Incoming: " + topic + " - " + payload);
    motorState = payload.toInt(); 
  }
  
  // ถ้าได้รับคำสั่ง Reset ให้เรียกฟังก์ชัน reset()
  if (topic == mqtt_reset) { 
    Serial.println("Incoming: " + topic + " - " + payload);
    if(payload == "reset"){ reset(); }
  }  
  
  // ถ้าได้รับคำสั่ง Lock ให้สลับสถานะ lock
  if (topic == mqtt_lock) { 
    Serial.println("Incoming: " + topic + " - " + payload);
    lock = !lock; 
  }  
}

// --- ฟังก์ชันคืนค่ามอเตอร์กลับไปที่องศา 0 ---
void reset() {
  if (currentAngle != 0) {
      long angleDiff = 0 - currentAngle; // คำนวณองศาที่ต้องหมุนกลับ
      Serial.print("Returning: ");
      Serial.println(angleDiff);

      // แปลงจากองศาเป็นจำนวน Step ที่ต้องใช้
      int stepsToMove = (int)((float)angleDiff / 360.0 * stepsPerRevolution);
      myStepper.step(stepsToMove); // สั่งมอเตอร์หมุนกลับ
      
      currentAngle = 0; // รีเซ็ตค่าองศาปัจจุบันเป็น 0
   }
   
   // ปิดโหมดอัตโนมัติเมื่อสั่งรีเซ็ต
   if(autoMode){
    autoMode = false;
    autoStarted = false;
   }
   motorState = 0; // หยุดมอเตอร์
}

// --- ฟังก์ชันเชื่อมต่อ WiFi และ MQTT ---
void connect() {
  // เชื่อมต่อ WiFi
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected.");

  // เชื่อมต่อ MQTT Broker
  Serial.print("Connecting to MQTT...");
  while (!client.connect(mqtt_client_id)) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nMQTT Connected.");

  // ติดตามหัวข้อ (Subscribe) เพื่อรอรับคำสั่งจาก Dashboard หรือแอป
  client.subscribe("group14/test1/#"); 
}

void setup() {
  Serial.begin(9600); // เริ่มต้น Serial Port
  WiFi.begin(ssid, pass); // เริ่มการเชื่อมต่อ WiFi
  
  myStepper.setSpeed(10); // ตั้งความเร็วหมุนมอเตอร์ (RPM)
  pinMode(LED, OUTPUT);   // กำหนดขา LED เป็น Output
  pinMode(BTN, INPUT);   // กำหนดขา Button เป็น Input

  client.begin(mqtt_broker, MQTT_PORT, net); // เตรียมการเชื่อมต่อ MQTT
  client.onMessage(messageReceived);         // กำหนดฟังก์ชันเมื่อมีข้อความเข้า
  
  connect(); // เรียกใช้ฟังก์ชันเชื่อมต่อ
}

void loop() {
  client.loop(); // รักษาการเชื่อมต่อและตรวจสอบข้อความ MQTT เข้า
  if (!client.connected()) connect(); // ถ้าหลุดให้เชื่อมต่อใหม่

  /* ===== ตรวจสอบการกดปุ่ม (Physical Button) ===== */
  bool btnNow = digitalRead(BTN);
  // ตรวจจับจังหวะที่ปุ่มถูกกดลง (Rising Edge)
  if (btnNow == HIGH && btnLast == LOW) { 
    autoMode = !autoMode; // สลับโหมด Auto/Manual
    delay(50); // ป้องกันสัญญาณรบกวน (Debounce)
  }
  btnLast = btnNow; // จำสถานะปุ่มปัจจุบันไว้ใช้รอบหน้า

  /* ===== อ่านค่าจากเซนเซอร์แสง ===== */
  ldr_value = analogRead(LDR);

  /* ===== ตรรกะโหมดอัตโนมัติ (Auto Mode Logic) ===== */
  if (autoMode) {
    if (!autoStarted) {
      autoStarted = true;
    }
    
    // ถ้ามืด (ค่า LDR สูง) ให้หมุนไปทางหนึ่ง
    if (ldr_value > 3500 && ldr_value < 4500) { 
      motorState = 1;    
    }
    // ถ้าสว่าง (ค่า LDR ต่ำลงในระดับที่กำหนด) ให้หมุนกลับ
    else if(ldr_value < 3500 && ldr_value > 2900) { 
      motorState = 2;  
    }
    // ถ้าอยู่นอกช่วงที่กำหนดให้หยุด
    else {
      motorState = 0; 
    }
  }
  else {
    // ถ้าสลับมาเป็น Manual ให้หยุดมอเตอร์ชั่วคราว
    if(autoStarted){ 
      motorState = 0;
      autoStarted = false;   
    }
  }

  /* ===== ส่วนควบคุมมอเตอร์ (Hardware Control) ===== */
  // หมุนตามเข็ม (Forward)
  if (motorState == 1 && lock == false) { 
    client.publish(mqtt_status, "Working");
    digitalWrite(LED, HIGH);
    
    myStepper.step(512); // หมุน 90 องศา (512 step)
    currentAngle += 90;  
    
  } 
  // หมุนทวนเข็ม (Reverse)
  else if (motorState == 2 && lock == false) {
    client.publish(mqtt_status, "Reverse"); 
    digitalWrite(LED, LOW);
    
    myStepper.step(-512); // หมุนกลับ 90 องศา
    currentAngle -= 90;  
  } 
  // หยุดหมุน (Stop)
  else if (motorState == 0 && lock == false) {
    // สั่งจ่ายไฟ LOW ทุกขาเพื่อไม่ให้มอเตอร์ร้อนและประหยัดไฟ
    digitalWrite(23, LOW);
    digitalWrite(21, LOW);
    digitalWrite(22, LOW);
    digitalWrite(19, LOW);
  }

  /* ===== ส่งข้อมูลกลับไปยัง MQTT Broker (Feedback) ===== */
  client.publish(mqtt_angle, String(currentAngle)); // ส่งองศาล่าสุด
  client.publish(mqtt_lockstatus, lock ? "lock" : "unclock"); // ส่งสถานะล็อค
  client.publish(mqtt_ldr, String(ldr_value)); // ส่งค่าแสง
  client.publish(mqtt_modestatus, autoMode ? "Auto" : "Manual"); // ส่งโหมดปัจจุบัน

}
