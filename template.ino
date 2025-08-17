#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <NewPing.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <ESPmDNS.h>

#define EEPROM_SIZE 1
#define ROBOT_NUM_ADDR 0

const char *base_ssid = "ESP32-AP-";
const char *base_ota_hostname = "ESP32-OTA-";
const char *ap_password = "12345678";

const int triggerPin = 5;
const int echoPin = 17;
const int control_servo_down = 32;
const int control_servo_up = 33;
#define MAX_DISTANCE 200

LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servodown;
Servo servoup;
NewPing sonar(triggerPin, echoPin, MAX_DISTANCE);

char ssid[32];
char ota_hostname[32];
uint8_t robot_number = 0;

TaskHandle_t otaTaskHandle = NULL;

#define EPSILON_START 0.9
#define EPSILON_END 0.01
#define EPSILON_DECAY 0.995
#define ALPHA 0.3
#define GAMMA 0.9

float current_epsilon = EPSILON_START;

void saveRobotNumber(uint8_t number)
{
  EEPROM.write(ROBOT_NUM_ADDR, number);
  EEPROM.commit();
  Serial.print("Saved robot number: ");
  Serial.println(number);
}

uint8_t readRobotNumber()
{
  uint8_t number = EEPROM.read(ROBOT_NUM_ADDR);
  if (number < 1 || number > 8)
  {
    Serial.println("Invalid or uninitialized robot number. Please set number (1-8).");
    lcd.clear();
    lcd.print("Set Robot Num: 1-8");
    while (!Serial.available())
    {
      delay(100);
    }
    number = Serial.parseInt();
    if (number >= 1 && number <= 8)
    {
      saveRobotNumber(number);
      lcd.clear();
      lcd.print("Robot Num Set: ");
      lcd.print(number);
      delay(2000);
    }
    else
    {
      Serial.println("Invalid input. Defaulting to robot number 1.");
      number = 1;
      saveRobotNumber(number);
    }
  }
  return number;
}

void setup_ap()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, ap_password);
  Serial.println("AP Started");
  Serial.print("AP SSID: ");
  Serial.println(ssid);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
}

void setup_ota()
{
  ArduinoOTA.setHostname(ota_hostname);

  ArduinoOTA.onStart([]()
                     {
    Serial.println("OTA Start");
    lcd.clear();
    lcd.print("OTA Update Start"); });
  ArduinoOTA.onEnd([]()
                   {
    Serial.println("\nOTA End");
    lcd.clear();
    lcd.print("OTA Update Done"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    lcd.clear();
    lcd.print("OTA Progress: ");
    lcd.setCursor(0, 1);
    lcd.print((progress / (total / 100)));
    lcd.print("%"); });
  ArduinoOTA.onError([](ota_error_t error)
                     {
    Serial.printf("Error[%u]: ", error);
    lcd.clear();
    lcd.print("OTA Error");
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed"); });
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
  Serial.print("OTA Hostname: ");
  Serial.println(ota_hostname);
}

void otaTask(void *parameter)
{
  for (;;)
  {
    ArduinoOTA.handle();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

float getDistance()
{
  float distance = sonar.ping_cm();
  if (distance == 0)
  {
    Serial.println("Warning: No echo received from SRF module.");
    return -1;
  }
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
}

void healthCheck()
{

  lcd.clear();
  lcd.print("Health Check Start");
  delay(1000);

  float distance = getDistance();
  lcd.clear();
  if (distance < 0)
  {
    lcd.print("SRF Error!");
  }
  else
  {
    lcd.print("Dist: ");
    lcd.print(distance);
    lcd.print(" cm");
  }
  delay(500);

  servodown.write(90);
  servoup.write(90);
  delay(1000);

  servodown.write(0);
  servoup.write(180);
  lcd.clear();
  lcd.print("Servos Reset");
  delay(1000);

  lcd.clear();
  lcd.print("Health Check Done");
  delay(1500);
  lcd.clear();
}

void moveServoSmooth(Servo &servo, int from, int to, int stepDelay = 10)
{
  if (from < to)
  {
    for (int a = from; a <= to; a += 2)
    {
      servo.write(a);
      delay(stepDelay);
    }
  }
  else
  {
    for (int a = from; a >= to; a -= 2)
    {
      servo.write(a);
      delay(stepDelay);
    }
  }
}

void setup()
{
  Serial.begin(9600);
  delay(1000);

  EEPROM.begin(EEPROM_SIZE);

  robot_number = readRobotNumber();
  snprintf(ssid, sizeof(ssid), "%s%d", base_ssid, robot_number);
  snprintf(ota_hostname, sizeof(ota_hostname), "%s%d", base_ota_hostname, robot_number);

  setup_ap();
  setup_ota();

  xTaskCreatePinnedToCore(
      otaTask,
      "OTATask",
      4096,
      NULL,
      1,
      &otaTaskHandle,
      1);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("RL Robot ");
  lcd.print(robot_number);
  lcd.setCursor(0, 1);
  lcd.print("Setup");
  delay(1000);

  servodown.attach(control_servo_down, 600, 2400);
  servoup.attach(control_servo_up, 600, 2400);

  servodown.write(0);
  servoup.write(180);

  lcd.clear();
  lcd.print("Setup Completed");
  delay(2000);
  healthCheck();
}

#define SERVO_UP_STATES 4
#define SERVO_DOWN_STATES 3
#define ACTIONS_NUM (SERVO_UP_STATES + SERVO_DOWN_STATES)

const int servo_up_angles[SERVO_UP_STATES] = {180, 144, 126, 108};
const int servo_down_angles[SERVO_DOWN_STATES] = {0, 66, 88};

int get_current_state()
{
  int up_angle = servoup.read();
  int down_angle = servodown.read();

  int up_index = 0;
  int down_index = 0;

  for (int i = 0; i < SERVO_UP_STATES; i++)
  {
    if (servo_up_angles[i] == up_angle)
    {
      up_index = i;
      break;
    }
  }

  for (int i = 0; i < SERVO_DOWN_STATES; i++)
  {
    if (servo_down_angles[i] == down_angle)
    {
      down_index = i;
      break;
    }
  }

  return up_index * SERVO_DOWN_STATES + down_index;
}

void perform_action(int action)
{
  if (action < SERVO_UP_STATES)
  { // Move servoup
    int target_angle = servo_up_angles[action];
    Serial.printf("Moving servoup to %d degrees\n", target_angle);
    moveServoSmooth(servoup, servoup.read(), target_angle);
  }
  else
  { // Move servodown
    int angle_index = action - SERVO_UP_STATES;
    int target_angle = servo_down_angles[angle_index];
    Serial.printf("Moving servodown to %d degrees\n", target_angle);
    moveServoSmooth(servodown, servodown.read(), target_angle);
  }
}

float q_table[SERVO_UP_STATES * SERVO_DOWN_STATES][ACTIONS_NUM];

int choose_action(int state)
{
  if ((float)random(100) / 100.0 < current_epsilon)
  {
    Serial.println("Action: Exploring (random)");
    return random(ACTIONS_NUM);
  }
  else
  {
    Serial.println("Action: Exploiting (best)");
    int best_action = 0;
    for (int i = 0; i < ACTIONS_NUM; i++)
    {
      if (q_table[state][i] > q_table[state][best_action])
      {
        best_action = i;
      }
    }
    return best_action;
  }
}

void decay_epsilon()
{
  current_epsilon *= EPSILON_DECAY;
  if (current_epsilon < EPSILON_END)
  {
    current_epsilon = EPSILON_END;
  }
  Serial.printf("Epsilon decayed to: %.4f\n", current_epsilon);
}

int calculate_reward(float distance_before, float distance_after)
{

  if (distance_after > 30)
  {
    return 100;
  }
  else{
    if (distance_after > distance_before)
    {
      return 25;
    }
    else
    {
      return -20;
    }
  }
}

void initialize_q_table()
{
  for (int i = 0; i < SERVO_UP_STATES * SERVO_DOWN_STATES; i++)
  {
    for (int j = 0; j < ACTIONS_NUM; j++)
    {
      q_table[i][j] = random(-100, 100) / 100.0;
    }
  }
}

void update_q_table(int state, int action, float reward, int new_state)
{
  float max_q_new_state = -1000;
  for (int i = 0; i < ACTIONS_NUM; i++)
  {
    if (q_table[new_state][i] > max_q_new_state)
    {
      max_q_new_state = q_table[new_state][i];
    }
  }

  q_table[state][action] += ALPHA * (reward + GAMMA * max_q_new_state - q_table[state][action]);
}

bool done_training = false;

void do_training(int epochs, int steps_per_epoch)
{
  const int NUM_STATES = SERVO_UP_STATES * SERVO_DOWN_STATES;
  const int NUM_ACTIONS = ACTIONS_NUM;

  Serial.printf("Training with %d epochs and %d steps per episode\n", epochs, steps_per_epoch);

  initialize_q_table();
  Serial.println("Q-table initialized for training");

  lcd.clear();
  lcd.print("Training...");

  for (int epoch = 0; epoch < epochs; epoch++)
  {
    Serial.printf("Epoch %d/%d\n", epoch + 1, epochs);
    lcd.setCursor(0, 1);
    lcd.printf("Epoch %d/%d", epoch + 1, epochs);

    for (int step = 0; step < steps_per_epoch; step++)
    {
      int state = get_current_state();

      float distance_before = getDistance();

      int action = choose_action(state);
      perform_action(action);

      delay(100);
      float distance_after = getDistance();

      float reward = calculate_reward(distance_before, distance_after);
      int new_state = get_current_state();
      update_q_table(state, action, reward, new_state);

      Serial.printf("Step %d: State: %d->%d, Action: %d, Reward: %.2f, Epsilon: %.4f\n",
                    step + 1, state, new_state, action, reward, current_epsilon);
    }
    
    // Decay epsilon after each epoch
    decay_epsilon();
  }
  Serial.println("Training completed.");
  lcd.clear();
  lcd.print("Training Done");
  delay(1000);
  done_training = true;
}

void do_action_after_training()
{
  int state = get_current_state();
  Serial.printf("Current state after training: %d\n", state);
  
  Serial.println("Q-values for current state:");
  for (int i = 0; i < ACTIONS_NUM; i++)
  {
    Serial.printf("Action %d: %.3f\n", i, q_table[state][i]);
  }

  int best_action = 0;
  float best_q_value = q_table[state][0];
  for (int i = 0; i < ACTIONS_NUM; i++)
  {
    if (q_table[state][i] > best_q_value)
    {
      best_action = i;
      best_q_value = q_table[state][i];
    }
  }

  Serial.printf("Performing best action: %d with Q-value: %.3f\n", best_action, best_q_value);
  
  // Display what this action will do
  if (best_action < SERVO_UP_STATES)
  {
    Serial.printf("This will move servo UP to %d degrees\n", servo_up_angles[best_action]);
  }
  else
  {
    int angle_index = best_action - SERVO_UP_STATES;
    Serial.printf("This will move servo DOWN to %d degrees\n", servo_down_angles[angle_index]);
  }
  
  // Measure distance before action
  float distance_before = getDistance();
  Serial.printf("Distance before action: %.2f cm\n", distance_before);
  
  perform_action(best_action);
  
  delay(500); 
}

void loop()
{
  lcd.clear();
  lcd.print("Robot ");
  lcd.print(robot_number);
  lcd.setCursor(0, 1);

  if (!done_training)
  {
    lcd.print("Training...");
    delay(500);
    do_training(16, 16);
    servodown.write(0);
    servoup.write(180);
  }
  else
  {
    int random_state = random(SERVO_UP_STATES * SERVO_DOWN_STATES);
    int up_index = random_state / SERVO_DOWN_STATES;
    int down_index = random_state % SERVO_DOWN_STATES;
    moveServoSmooth(servoup, servoup.read(), servo_up_angles[up_index]);
    moveServoSmooth(servodown, servodown.read(), servo_down_angles[down_index]);
    lcd.clear();
    lcd.print("Using Learned");
    delay(500);
    do_action_after_training();
  }

  delay(500);
}
