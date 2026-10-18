#include "moves.h"
#include "globals.h"
#include "compass.h"
#include "position.h"
#include "link.h"

void doMove();
void goStraight(int16_t direction);
bool calcMoves();

struct WaitIntermMove: intermMove {
  void start() override {
    timer.in((uint16_t) SHOW_CHARACTER_WAIT_ms, nextMove);
  }
  void handle() override {
    setTracks(0, 0);
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
    Serial.print(F("[d] gostraight start: dist=")); Serial.print(this->dist);
    Serial.println();
    timer.in((uint16_t) (this->dist / MOVE_cm_PER_s * 1000.0), nextMove);
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

// State of the small-step settle below. A move taking over the tracks cancels it.
static bool settle_stepping = false;
static int8_t settle_dir = 0;
static float settle_start_deg = 0, settle_size_deg = 0;

// Turns the bot to heading 0 and holds it there. Returns true once the compass says it is within
// HEADING_DEVIATION_THRESHOLD of 0 on either side. Far off, it turns continuously. Close in, it turns in short
// steps that the gyro cuts off short of the target, then waits with the motors off while the compass re-measures:
// the continuous turn can't make a small correction, because its slowest speed is TURN_BASE_SPEED.
static bool settleToHeading0() {
  if (settle_stepping) {
    float moved = wrap180(headingDeg() - settle_start_deg) * settle_dir;
    if (moved >= settle_size_deg) {
      setTracks(0, 0);
      settle_stepping = false;
    }
    return false;
  }

  float err = angleError(0);  // positive: the heading has to increase
  if (fabs(err) > REST_FINE_ZONE_DEG) {
    turnToHeading(0);
    return false;
  }

  // close in: stop, and let the compass re-measure before deciding the next step
  if (!tracksIdleFor(MAG_SETTLE_MS + MAG_REFRESH_MS + 20)) {
    setTracks(0, 0);
    return false;
  }
  if (fabs(err) <= HEADING_DEVIATION_THRESHOLD) {
    return true;
  }

  settle_dir = err > 0 ? 1 : -1;
  settle_size_deg = fabs(err) * REST_STEP_FRACTION;
  settle_start_deg = headingDeg();
  settle_stepping = true;
  // same track directions as turnToHeading: a positive error is left backwards, right forwards
  setTracks(-settle_dir * REST_TURN_SPEED, settle_dir * REST_TURN_SPEED);
  return false;
}

/**
 * Execute on POI queue, run motors etc.
 */
bool runMotors(void *) {
  if (!global.poi_queue.isEmpty() && global.move_queue.isEmpty()) {
    if (!settleToHeading0()) {
      // still turning to heading 0
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
    settle_stepping = false;
    doMove();
  }
  else {
    // nothing to do: face heading 0 and keep the tracks off once there
    if (settleToHeading0()) {
      setTracks(0, 0);
    }
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
  setTracks(direction * (MOTOR_SPEED + TRACK_L_SPEED_ADJ), direction * MOTOR_SPEED);
}

bool nextMove(void *) {
  setTracks(0, 0);
  updateHeading(nullptr);
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
//   if (dst->y == global.pos.y && dst->x == global.pos.x) {
//     // wiggle around
//     moves_prealloc.turn1.heading = 10;
//     global.move_queue.enqueue(&(moves_prealloc.turn1));
//
//     moves_prealloc.turn2.heading = 350;
//     global.move_queue.enqueue(&(moves_prealloc.turn2));
//   }

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
//   if (abs(to.y - from.y) <= Y_POSITION_DEVIATION_THRESHOLD) {
//     moves_prealloc.goStraight1.dist = dist_x;
//     moves_prealloc.goStraight1.direction = x_dir;
//     global.move_queue.enqueue(&(moves_prealloc.goStraight1));
//     return true;
//   }

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
  setTracks(-speed, speed);
}

void velocityTest() {
  Serial.print("Press button for 10s velocity test... ");
  button.waitForButton();
  delay(500);

  setTracks(MOTOR_SPEED, MOTOR_SPEED);
  delay(10000);
  setTracks(0, 0);
  Serial.println("done");
}
