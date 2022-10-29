#include <arduino-timer.h>
#include <NewPing.h>
#include <Wire.h>
#include <ZumoShield.h>
#include <util/atomic.h>
#include <ArduinoQueue.h>
#include <NeoSWSerial.h>

#define RAD2DEG 180.0 / M_PI
#define COS_315_DEG cos(7.0 * M_PI / 4.0)

// program
#define SHOW_CHARACTER_WAIT_ms 1500

// planchette
#define PLANCH_CENTER_TO_SCREEN_cm 5

// board constants
#define BOARD_ORI_deg 348
#define BOARD_W_cm 85.1
#define BOARD_H_cm 57.15
#define BOARD_AREA_cm BOARD_W_cm * BOARD_H_cm
#define BOARD_LEFT_WALL_X -(BOARD_W_cm / 2)
#define BOARD_RIGHT_WALL_X BOARD_W_cm / 2
#define BOARD_TOP_WALL_Y BOARD_H_cm / 2
#define BOARD_BOTTOM_WALL_Y -(BOARD_H_cm / 2)

#define MOVE_X_LIMIT ((BOARD_W_cm / 2.0) - 16)
#define MOVE_Y_LIMIT ((BOARD_H_cm / 2.0) - 9)

#define COORD_UNDEF -32768.0

// ultrasonic sensors
#define USS_F_TRIG_PIN 4
#define USS_B_TRIG_PIN 5
#define USS_L_TRIG_PIN 2
#define USS_R_TRIG_PIN 11
#define USS_ECHO_PIN A2
#define USS_F 0
#define USS_R 1
#define USS_B 2
#define USS_L 3
// distance from sensor to center of bot
#define USS_DIST_ADJ_cm 7
#define BOARD_W_DEV_THRESHOLD 5.0
#define BOARD_H_DEV_THRESHOLD 3.0

#define USS_PING_INTERVAL 50
#define FIND_POSITION_INTERVAL 300
const float SOUND_cm_PER_uS_HALVED = 0.034 / 2;
//#define USS_SAMPLE_SIZE 10
#define MAX_DISTANCE_cm 100
const int MAX_DURATION_uS = (int) (50 / SOUND_cm_PER_uS_HALVED);

auto timer = timer_create_default();

NewPing uss_newping[4] = {
  NewPing(USS_F_TRIG_PIN, USS_ECHO_PIN, BOARD_W_cm),
  NewPing(USS_R_TRIG_PIN, USS_ECHO_PIN, BOARD_W_cm),
  NewPing(USS_B_TRIG_PIN, USS_ECHO_PIN, BOARD_W_cm),
  NewPing(USS_L_TRIG_PIN, USS_ECHO_PIN, BOARD_W_cm)
};

// softwareserial
#define SS_RX A0
#define SS_TX A3
#define SS_BAUDRATE 9600

NeoSWSerial nss(SS_RX, SS_TX);

// motors
#define MOTOR_CALIB_SPEED 200
#define MOTOR_CALIBRATION_SAMPLES 100
#define MOVE_INTERVAL 50
#define MOTOR_SPEED 200
#define TRACK_L_SPEED_ADJ 20
#define TURN_BASE_SPEED 150
#define TRACKS_DIST_FROM_CENTER_cm 4.3
//#define MOVE_cm_PER_s 8 // this is empirically determined with velocityTest()
#define MOVE_cm_PER_s 8.2
#define TRACK_L 0
#define TRACK_R 1
#define Y_POSITION_DEVIATION_THRESHOLD 3
#define X_POSITION_DEVIATION_THRESHOLD 2

ZumoMotors motors;

// compass
#define FIND_ORIENTATION_INTERVAL 45
//#define COMPASS_DEVIATION_THRESHOLD 5
#define HEADING_DEVIATION_THRESHOLD 2.0
#define LAX_HEADING_DEVIATION_THRESHOLD 5.0

ZumoIMU imu;
ZumoIMU::vector<int16_t> m_max, m_min;

// nav


// button
Pushbutton button(ZUMO_BUTTON);

struct point {
  int8_t x;
  int8_t y;
};

struct intermMove {
  virtual void start() {}
  virtual void handle() {}
  virtual bool isDone() {}
};

struct Globals {
  volatile uint8_t uss_curr = USS_F;
  volatile int8_t uss_distances[4];
  volatile int8_t uss_adj_distances[4];
  volatile point pos = {COORD_UNDEF, COORD_UNDEF};
  volatile int16_t theta;
  // move queue for POI on the board i.e. letters
  volatile ArduinoQueue<point> poi_queue = ArduinoQueue<point>(64);
  volatile point moving_to;
  volatile point moving_from;
  volatile ArduinoQueue<intermMove *> move_queue = ArduinoQueue<intermMove *>(6);
};
struct Globals global;

uint16_t angleDiff(int16_t source_deg, int16_t target_deg) {
  int16_t diff = (target_deg - source_deg) % 360;
  if (diff < 0) diff += 360;
  return diff;
}

struct Turn {
  uint16_t angle;
  int16_t direction;
};

struct Turn shortestTurn(int16_t source_deg, int16_t target_deg) {
  uint16_t diff = angleDiff(source_deg, target_deg);
  return {
    min(diff, 360 - diff),
    diff < 180 ? 1 : -1
  };
}

bool headingIsNot(uint16_t deg, bool lax) {
  return angleDiff(deg, global.theta) > (lax ? LAX_HEADING_DEVIATION_THRESHOLD : HEADING_DEVIATION_THRESHOLD);
}

bool headingIs(uint16_t deg, bool lax) {
  return angleDiff(deg, global.theta) <= (lax ? LAX_HEADING_DEVIATION_THRESHOLD : HEADING_DEVIATION_THRESHOLD);
}

void echoCheck() {
  uint8_t uss = global.uss_curr;
  if (uss_newping[uss].check_timer()) {
    global.uss_distances[uss] = uss_newping[uss].ping_result / US_ROUNDTRIP_CM;
    global.uss_adj_distances[uss] = global.uss_distances[uss] + USS_DIST_ADJ_cm;
  }
}

bool ussPingTimers(void *) {
  uss_newping[global.uss_curr].timer_stop();
  
  global.uss_curr = (global.uss_curr + 1) % 4;
  global.uss_distances[global.uss_curr] = -1;
  uss_newping[global.uss_curr].ping_timer(echoCheck);
  
  return true;
}

// Converts x and y components of a vector to a heading in degrees.
// This calculation assumes that the Zumo is always level.
template <typename T> float headingOf(ZumoIMU::vector<T> v)
{
  float x_scaled =  2.0 * (float)(v.x - m_min.x) / (m_max.x - m_min.x) - 1.0;
  float y_scaled =  2.0 * (float)(v.y - m_min.y) / (m_max.y - m_min.y) - 1.0;

  float angle = atan2(y_scaled, x_scaled) * RAD2DEG;
  if (angle < 0)
    angle += 360;
  return angle;
}

float averageHeading() {
  ZumoIMU::vector<int32_t> avg = {0, 0, 0};

  for(int i = 0; i < 10; i ++)
  {
    imu.readMag();
    avg.x += imu.m.x;
    avg.y += imu.m.y;
  }
  avg.x /= 10.0;
  avg.y /= 10.0;

  // avg is the average measure of the magnetic vector.
  return headingOf(avg);
}

bool findOrientation(void *) {
  // North is x-axis
  float heading_inv = 360 - averageHeading();
  // Adjust for board orientation
  float new_theta = heading_inv + (360 - BOARD_ORI_deg);
  if (new_theta > 360)
    new_theta -= 360;
  
  global.theta = new_theta;

  return true;
}

bool findPosition(void *) {
  if (headingIsNot(0, true)) {
    return true;
  }

  int8_t x = COORD_UNDEF, y = COORD_UNDEF,
      x_f, x_b, y_l, y_r;

  // x...
  int8_t dist_x_f = global.uss_adj_distances[USS_F],
      dist_x_b = global.uss_adj_distances[USS_B];
  if (dist_x_f != -1) {
    x_f = BOARD_RIGHT_WALL_X - dist_x_f;
    x = x_f;
  }
  
  if (dist_x_b != -1) {
    x_b = BOARD_LEFT_WALL_X + dist_x_b;
    x = x_b;
  }

  if (dist_x_f != -1 && dist_x_b != -1) {
    // use the more reliable shorter distance
    x = dist_x_f < dist_x_b ? x_f : x_b;
    
    if (abs((dist_x_f + dist_x_b) - BOARD_W_cm) > BOARD_W_DEV_THRESHOLD) {
//      Serial.print("x distances F="); Serial.print(dist_x_f);
//      Serial.print(" and B="); Serial.print(dist_x_b);
//      Serial.println(" don't add up");
    }
  }
  else if (dist_x_f == -1 && dist_x_b == -1) {
    Serial.println(F("x undef"));
    global.pos = {COORD_UNDEF, COORD_UNDEF};
    return true;
  }

  // y...
  int8_t dist_y_l = global.uss_adj_distances[USS_L],
      dist_y_r = global.uss_adj_distances[USS_R];
  if (dist_y_l != -1) {
    y_l = BOARD_TOP_WALL_Y - dist_y_l;
    y = y_l;
  }
  
  if (dist_y_r != -1) {
    y_r = BOARD_BOTTOM_WALL_Y + dist_y_r;
    y = y_r;
  }

  if (dist_y_l != -1 && dist_y_r != -1) {
    // use the more reliable shorter distance
    y = dist_y_l < dist_y_r ? y_l : y_r;
    
    if (abs((dist_y_l + dist_y_r) - BOARD_H_cm) > BOARD_H_DEV_THRESHOLD) {
//      Serial.print("y distances L="); Serial.print(dist_y_l);
//      Serial.print(" and R="); Serial.print(dist_y_r);
//      Serial.println(" don't add up");
    }
  }
  else if (dist_y_l == -1 && dist_y_r == -1) {
    Serial.println(F("y undef"));
    global.pos = {COORD_UNDEF, COORD_UNDEF};
    return true;
  }

  global.pos = {x, y};

  Serial.print(F("x=")); Serial.print(global.pos.x);
  Serial.print(F(", y=")); Serial.println(global.pos.y);
  
  return true;
}


struct WaitIntermMove: intermMove {
  void start() override {
    timer.in((uint16_t) SHOW_CHARACTER_WAIT_ms, nextMove);
  }
  void handle() override {
    motors.setSpeeds(0, 0);
  }
  bool isDone() override {
    return false;
  }
};

struct TurnIntermMove: intermMove {
  int16_t heading;
  TurnIntermMove(int16_t heading) {
    this->heading = heading;
  }
  
  void handle() override {
    turnToHeading(this->heading);
  }
  bool isDone() override {
    return headingIs(this->heading, false);
  }
};

struct GoStraightIntermMove: intermMove {
  float dist;
  int8_t direction;
  GoStraightIntermMove(float dist, int8_t direction) {
    this->dist = dist;
    this->direction = direction;
  }
  
  void start() override {
    Serial.print(F("[d] gostraight start ")); Serial.print(this->dist);
    Serial.print(' '); Serial.println(this->direction);
    timer.in((uint16_t) (this->dist / MOVE_cm_PER_s * 1000), nextMove);
  }
  void handle() override {
    goStraight(this->direction);
  }
  bool isDone() override {
    return false;
  }
};

struct PositioningIntermMove: intermMove {
  void start() override {
    timer.in((uint16_t) FIND_POSITION_INTERVAL + 20, nextMove);
  }
  void handle() override {}
  bool isDone() override {
    return false;
  }
};

struct MovesPrealloc {
  // the move queue has pointers to support polymorphism,
  // so these instances are preallocated
  volatile WaitIntermMove wait1;
  volatile GoStraightIntermMove goStraight1 = {-1, -1};
  volatile TurnIntermMove turn1 = {-1};
  volatile TurnIntermMove turn2 = {-1};
  volatile PositioningIntermMove positioning1;
};
struct MovesPrealloc moves_prealloc;

/**
 * Execute on POI queue, run motors etc.
 */
bool runMotors(void *) {
  if (!global.poi_queue.isEmpty() && global.move_queue.isEmpty()) {
    if (headingIsNot(0, false)) {
      turnToHeading(0);
    }
    else {
      findPosition(nullptr);

      point p = global.poi_queue.getHead();
      uint8_t dist_x = abs(p.x - global.pos.x);
      uint8_t dist_y = abs(p.y - global.pos.y);
      if (dist_x <= X_POSITION_DEVIATION_THRESHOLD && dist_y <= Y_POSITION_DEVIATION_THRESHOLD) {
        Serial.println(F("close enough"));

        sendArrived(global.poi_queue.getHead());

        // wait for any character on the screen to finish showing before continuing
        global.move_queue.enqueue(&(moves_prealloc.wait1));
        moves_prealloc.wait1.start();
    
        global.poi_queue.dequeue();

        if (global.poi_queue.isEmpty()) {
          // send smiley face (after last character is shown)
          timer.in(SHOW_CHARACTER_WAIT_ms, [](void *) -> bool {
            sendSmiley();
          });
        }
      }
      else if (calcMoves()) {
        intermMove *m = global.move_queue.getHead();
        m->start();
        doMove();
      }
    }
  }
  else if (!global.move_queue.isEmpty()) {
    doMove();
  }
  else if (headingIsNot(0, true)) {
    turnToHeading(0);
  }
  else {
    motors.setSpeeds(0, 0);
  }

  return true;
}

void doMove() {
  intermMove *m = global.move_queue.getHead();
  if (m->isDone()) {
    nextMove(nullptr);
  }
  else {
    m->handle();
  }
}

void goStraight(int16_t direction) {
  motors.setSpeeds(direction * (MOTOR_SPEED + TRACK_L_SPEED_ADJ), direction * MOTOR_SPEED);
}

bool nextMove(void *) {
  motors.setSpeeds(0, 0);
  global.move_queue.dequeue();
  intermMove **m = global.move_queue.getHeadPtr();
  
  if (m != nullptr) {
    (*m)->start();
  }
  else {
    global.moving_to = {COORD_UNDEF, COORD_UNDEF};
  }
  
  return false;
}

float distBetween(point *a, point *b) {
  return sqrt(pow(a->x - b->x, 2) + pow(a->y - b->y, 2));
}

/* returns true if there are moves */
bool calcMoves() {
  if (global.pos.x == COORD_UNDEF) {
    return false;
  }
  
  point *dst = global.poi_queue.getHeadPtr();
  if (dst->y == global.pos.y && dst->x == global.pos.x) {
    // wiggle around
    moves_prealloc.turn1.heading = 10;
    global.move_queue.enqueue(&(moves_prealloc.turn1));

    moves_prealloc.turn2.heading = 350;
    global.move_queue.enqueue(&(moves_prealloc.turn2));
  }

  global.moving_from = {global.pos.x, global.pos.y};
  
  float atan_to_dst = atan2(
      dst->y - global.pos.y,
      dst->x - global.pos.x) * RAD2DEG;
  float shortest_angle_from_x = atan_to_dst > 90 ? 180 - atan_to_dst :
      atan_to_dst < -90 ? -180 - atan_to_dst :
      atan_to_dst;
  bool two_point_turn = abs(shortest_angle_from_x) > 45;
  if (two_point_turn) {
    // interm. x move
    global.moving_to = {
      round((global.moving_from.x < 0 ? 1.0 : -1.0) * (abs(dst->y - global.moving_from.y) / 2.0) * 1.75),
      round((dst->y + global.moving_from.y) / 2.0)
    };
    atan_to_dst = atan2(
      global.moving_to.y - global.moving_from.y,
      global.moving_to.x - global.moving_from.x) * RAD2DEG;
  }
  else {
    global.moving_to = {dst->x, dst->y};
  }

  point from = {global.moving_from.x, global.moving_from.y};
  point to = {global.moving_to.x, global.moving_to.y};
  Serial.print("from.x="); Serial.print(from.x);
  Serial.print(", .y="); Serial.println(from.y);
  Serial.print("to.x="); Serial.print(to.x);
  Serial.print(", .y="); Serial.println(to.y);
  
  int16_t x_dir = from.x < to.x ? 1 : -1;
  float dist_x = abs(to.x - from.x);
  if (abs(to.y - from.y) <= Y_POSITION_DEVIATION_THRESHOLD) {
    moves_prealloc.goStraight1.dist = dist_x;
    moves_prealloc.goStraight1.direction = x_dir;
    global.move_queue.enqueue(&(moves_prealloc.goStraight1));
    return true;
  }

  int16_t y_dir = from.y < to.y ? 1 : -1;
  float dist_y = abs(to.y - from.y);
  Serial.print(F("atan_to_dst=")), Serial.println(atan_to_dst);
  float heading = atan_to_dst;
  if (heading < 0) {
    // atan2 returns negative when to.y < from.y
    heading = 360 + heading;
  }
  if (x_dir < 0) {
    // we're going backwards
    heading += 180;
  }

  Turn turn = shortestTurn(0, heading);
  moves_prealloc.turn1.heading = heading + ((turn.direction < 0 ? -1 : 1) * HEADING_DEVIATION_THRESHOLD);
  global.move_queue.enqueue(&(moves_prealloc.turn1));

  float dist_between = distBetween(&from, &to);
  moves_prealloc.goStraight1.dist = dist_between;
  moves_prealloc.goStraight1.direction = x_dir;
  global.move_queue.enqueue(&(moves_prealloc.goStraight1));

  turn = shortestTurn(heading, 0);
  moves_prealloc.turn2.heading = ((turn.direction < 0 ? -1 : 1) * HEADING_DEVIATION_THRESHOLD);
  global.move_queue.enqueue(&(moves_prealloc.turn2));

  global.move_queue.enqueue(&(moves_prealloc.positioning1));

  return true;
}


void turnToHeading(uint16_t deg) {
  Turn turn = shortestTurn(global.theta, deg);
//  Serial.print("[d] turn.angle="); Serial.print(turn.angle);
//  Serial.print(", turn.dir="); Serial.println(turn.direction);
  int16_t speed = turn.direction * MOTOR_SPEED
      // adding a bit to turn.angle to increase speed
      * (float) (turn.angle + 5) / 180.0
      + (turn.direction * TURN_BASE_SPEED);
//  Serial.print("[d] turn speed: "); Serial.println(speed);
  motors.setSpeeds(-speed, speed);
}

void debugCheckActiveTasks(void *) {
//  Serial.print("[d] active tasks: "); Serial.println(timer.size());
  return true;
}

void velocityTest() {
  Serial.print("Press button for 10s velocity test... ");
  button.waitForButton();
  delay(500);

  motors.setSpeeds(MOTOR_SPEED, MOTOR_SPEED);
  delay(10000);
  motors.setSpeeds(0, 0);
  Serial.println("done");
}

void sendArrived(point p) {
  nss.write("$arrived(");
  char x[4], y[4];
  itoa(p.x, x, 10);
  // board coordinate Y matches the POI Y (the screen should be over it)
  itoa(p.y + PLANCH_CENTER_TO_SCREEN_cm, y, 10);
  nss.write(x); nss.write(','); nss.write(y);
  nss.write(')');
}

void sendSmiley() {
  nss.write("$smiley()");
}

void sendCheck() {
  nss.write("$check()");
  nss.flush();
}

volatile char serial_read_stage[16];
volatile uint8_t serial_bytes_staged;
char poi_cmd[] = "$addPoi(";
uint8_t poi_cmd_x_idx = 8;
volatile bool reading = false;

bool readNSS(void *) {
  if (reading) {
    return true;
  }
  
  while (nss.available() > 0) {
    reading = true;
    
    uint8_t c = nss.read();
    Serial.print(F("[d] c=")); Serial.println((char) c);
    serial_read_stage[serial_bytes_staged++] = (char) c;
    if (serial_bytes_staged >= 16) {
      serial_bytes_staged = 0;
    }
    
    if (c == ')') {
      serial_bytes_staged = 0;
      
      bool is_add_poi = true;
      for (uint8_t i = 0; i < poi_cmd_x_idx; i++) {
        if (serial_read_stage[i] != poi_cmd[i]) {
          is_add_poi = false;
          break;
        }
      }
  
      if (is_add_poi) {
        point p;
        strtok(serial_read_stage, "(,");
        p.x = atoi(strtok(nullptr, "(,"));
        // board coordinate Y matches the POI Y (the screen should be over it)
        p.y = atoi(strtok(nullptr, "(,")) - PLANCH_CENTER_TO_SCREEN_cm;
  
        Serial.print(F("[d] enqing p.x=")); Serial.print(p.x);
        Serial.print(F(", p.y=")); Serial.println(p.y);
  
        global.poi_queue.enqueue(p);
      }
    }
  }

  reading = false;

  return true;
}

/**
 * Setup
 */
void setup() {

  Serial.begin(19200);
  Serial.println(F("Running"));
  
  nss.begin(SS_BAUDRATE);

  Wire.begin();
  
  imu.init();
  imu.enableDefault();
  imu.configureForCompassHeading();

//  velocityTest();

  // compass calibration
  button.waitForButton();

  Serial.println(F("sending serial check"));
  sendCheck();

  Serial.println(F("waiting 5s"));
  delay(5000);
  Serial.println(F("calibrating compass..."));

  ZumoIMU::vector<int16_t> running_min = {32767, 32767, 32767},
    running_max = {-32767, -32767, -32767};

  motors.setLeftSpeed(MOTOR_CALIB_SPEED);
  motors.setRightSpeed(-MOTOR_CALIB_SPEED);
  
  for (uint8_t i = 0; i < MOTOR_CALIBRATION_SAMPLES; i++) {
    imu.readMag();

    running_min.x = min(running_min.x, imu.m.x);
    running_min.y = min(running_min.y, imu.m.y);

    running_max.x = max(running_max.x, imu.m.x);
    running_max.y = max(running_max.y, imu.m.y);

    delay(50);
  }

  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  Serial.print("max.x   ");
  Serial.print(running_max.x);
  Serial.println();
  Serial.print("max.y   ");
  Serial.print(running_max.y);
  Serial.println();
  Serial.print("min.x   ");
  Serial.print(running_min.x);
  Serial.println();
  Serial.print("min.y   ");
  Serial.print(running_min.y);
  Serial.println();

  m_max.x = running_max.x;
  m_max.y = running_max.y;
  m_min.x = running_min.x;
  m_min.y = running_min.y;

  Serial.println("starting timers");
  
  // start USS pings
  timer.every(USS_PING_INTERVAL, ussPingTimers);

  timer.every(FIND_ORIENTATION_INTERVAL, findOrientation);

  timer.every(FIND_POSITION_INTERVAL, findPosition);

  delay(500);
  timer.every(MOVE_INTERVAL, runMotors);

  timer.every(500, readNSS);
}

void loop() {
  timer.tick<void>();
}
