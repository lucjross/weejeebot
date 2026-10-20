#include "moves.h"
#include "globals.h"
#include "compass.h"
#include "position.h"
#include "link.h"

// State of the small-step settle below. A drive taking over the tracks cancels it.
static bool settle_stepping = false;
static int8_t settle_dir = 0;
static float settle_start_deg = 0, settle_size_deg = 0;
// True while the bot sits at rest at heading 0 with the tracks off, so that compass noise or a nudge smaller than
// REST_REENGAGE_DEG does not start a correction.
static bool resting = false;
// the direction (1 or -1) of the continuous turn in progress, 0 when there is none
static int8_t big_turn_dir = 0;
// when the compass first read more than REST_REENGAGE_DEG off while resting; 0 if it is not
static uint32_t off_target_since_ms = 0;

// Turns the bot to target_deg and holds it there. Returns true once the compass says it is within
// tolerance_deg of the target on either side. Far off, it turns continuously. Close in, it turns in short
// steps that the gyro cuts off short of the target, then waits with the motors off while the compass re-measures:
// the continuous turn can't make a small correction, because its slowest speed is TURN_BASE_SPEED.
static bool settleToHeading(uint16_t target_deg, float tolerance_deg) {
  if (settle_stepping) {
    float moved = wrap180(headingDeg() - settle_start_deg) * settle_dir;
    if (moved >= settle_size_deg) {
      setTracks(0, 0);
      settle_stepping = false;
    }
    return false;
  }

  float err = angleError(target_deg);  // positive: the heading has to increase
  if (big_turn_dir == 0 && fabs(err) > REST_FINE_ZONE_DEG) {
    big_turn_dir = err > 0 ? 1 : -1;
  }
  if (big_turn_dir != 0) {
    // keep turning until less than the fine zone minus the aim-past margin is left, or the target has been crossed
    if (err * big_turn_dir > REST_FINE_ZONE_DEG - TURN_AIM_PAST_DEG) {
      turnToHeading(target_deg);
      return false;
    }
    big_turn_dir = 0;
  }

  // close in: stop, and let the compass re-measure before deciding the next step
  if (!tracksIdleFor(MAG_SETTLE_MS + MAG_REFRESH_MS + 20)) {
    setTracks(0, 0);
    return false;
  }
  if (fabs(err) <= tolerance_deg) {
    return true;
  }

  Serial.print(F("settle to ")); Serial.print(target_deg); Serial.print(F(" err=")); Serial.println(err);
  settle_dir = err > 0 ? 1 : -1;
  settle_size_deg = fabs(err) * REST_STEP_FRACTION;
  settle_start_deg = headingDeg();
  settle_stepping = true;
  // same track directions as turnToHeading: a positive error is left backwards, right forwards
  setTracks(-settle_dir * REST_TURN_SPEED, settle_dir * REST_TURN_SPEED);
  return false;
}

// The left track runs faster by the same proportion at every speed, to keep the bot going straight.
static void goStraightAt(int16_t direction, int16_t speed) {
  setTracks(direction * (speed + speed * TRACK_L_SPEED_ADJ / MOTOR_SPEED), direction * speed);
}

// Full speed far from the target, slowing to DRIVE_MIN_SPEED over the last DRIVE_SLOW_ZONE_cm.
static int16_t driveSpeed(float distance_cm) {
  if (distance_cm >= DRIVE_SLOW_ZONE_cm) {
    return MOTOR_SPEED;
  }
  return DRIVE_MIN_SPEED + (int16_t) ((MOTOR_SPEED - DRIVE_MIN_SPEED) * (distance_cm / DRIVE_SLOW_ZONE_cm));
}

// ---- driving along one axis, steered by the sonars ----

enum DriveResult : uint8_t { DRIVE_RUNNING, DRIVE_DONE, DRIVE_FAILED };

static bool drive_moving = false;
// USS_F or USS_B, chosen when a drive starts and kept for all of it; -1 until then
static int8_t drive_sensor = -1;
static uint32_t drive_started_ms = 0, drive_stopped_ms = 0, drive_deadline_ms = 0;

static void driveReset() {
  drive_moving = false;
  drive_sensor = -1;
  drive_stopped_ms = 0;
  // the whole drive (all its bursts) gets a limit
  drive_deadline_ms = millis() + 20000UL;
}

// One tick of driving along x (y_axis false, heading 0) or y (heading Y_AXIS_HEADING) to target, in cm. The bot
// drives in bursts: it runs until it is DRIVE_STOP_LEAD_cm short of the target, stops, waits for fresh readings, and goes
// again if it is still more than tolerance away. Call it with the bot already squared up to the axis.
static DriveResult driveAxis(bool y_axis, float target, float tolerance) {
  uint32_t now = millis();
  if ((int32_t) (now - drive_deadline_ms) > 0) {
    setTracks(0, 0);
    return DRIVE_FAILED;
  }
  if (fabs(angleError(y_axis ? Y_AXIS_HEADING : 0)) > DRIVE_HEADING_ABORT_DEG) {
    setTracks(0, 0);
    return DRIVE_FAILED;
  }

  float at;
  bool have = axisPosition(y_axis, at, &drive_sensor);
  float err = target - at;  // positive: the target is toward the front sonar's wall

  if (drive_moving) {
    int8_t ahead = global.uss_distances[err > 0 ? USS_F : USS_B];
    bool wall = ahead >= 0 && ahead <= WALL_STOP_cm;
    // a burst longer than this means the readings are not moving as they should
    bool too_long = now - drive_started_ms > 4000;
    if (!have || wall || too_long || fabs(err) <= DRIVE_STOP_LEAD_cm) {
      setTracks(0, 0);
      drive_moving = false;
      drive_stopped_ms = now;
    } else {
      goStraightAt(err > 0 ? 1 : -1, driveSpeed(fabs(err)));
    }
    return DRIVE_RUNNING;
  }

  // stopped: wait for the tracks to coast to a halt and every sonar to report again
  if (now - drive_stopped_ms < DRIVE_MEASURE_MS || !have) {
    setTracks(0, 0);
    return DRIVE_RUNNING;
  }
  if (fabs(err) <= tolerance) {
    return DRIVE_DONE;
  }
  drive_moving = true;
  drive_started_ms = now;
  goStraightAt(err > 0 ? 1 : -1, driveSpeed(fabs(err)));
  return DRIVE_RUNNING;
}

// ---- the steps of getting to a point ----

enum Phase : uint8_t {
  PH_FACE_X,   // square up to heading 0, read the position, decide what is next
  PH_FACE_Y,   // turn to Y_AXIS_HEADING
  PH_DRIVE_Y,
  PH_DRIVE_X,
  PH_SHOW      // arrived: wait for the letter to finish showing
};

static Phase phase = PH_FACE_X;
static uint8_t attempts = 0;
static bool y_done = false;       // a drive along y has finished for this point
static uint32_t shown_at_ms = 0;

static void startPoint() {
  phase = PH_FACE_X;
  attempts = 0;
  y_done = false;
  driveReset();
}

static int8_t clampCoord(int8_t v, float limit) {
  return v > limit ? (int8_t) limit : v < -limit ? (int8_t) -limit : v;
}

/**
 * Execute on POI queue, run motors etc.
 */
bool runMotors(void *) {
  if (global.poi_queue.isEmpty()) {
    if (phase == PH_SHOW) {
      phase = PH_FACE_X;
    }
    // nothing to do: face heading 0 and keep the tracks off once there
    if (resting) {
      if (fabs(angleError(0)) <= REST_REENGAGE_DEG) {
        off_target_since_ms = 0;
        return true;
      }
      // a spike in the compass does not count; it has to stay off
      if (off_target_since_ms == 0) {
        off_target_since_ms = millis();
      }
      if (millis() - off_target_since_ms < REST_REENGAGE_MS) {
        return true;
      }
    }
    resting = false;
    off_target_since_ms = 0;
    if (settleToHeading(0, TURN_TOLERANCE_DEG)) {
      setTracks(0, 0);
      resting = true;
    }
    return true;
  }

  if (resting) {
    resting = false;
    startPoint();
  }
  point p = global.poi_queue.getHead();

  switch (phase) {
    case PH_FACE_X: {
      if (!settleToHeading(0, TURN_TOLERANCE_DEG)) {
        break;
      }
      findPosition(nullptr);
      if (global.pos.x == COORD_UNDEF) {
        break;
      }

      int16_t dx = p.x - global.pos.x;
      int16_t dy = p.y - global.pos.y;
      // the y after a drive along y was read by other sonars than this one, so allow a little more
      bool y_ok = abs(dy) <= Y_POSITION_DEVIATION_THRESHOLD ||
          (y_done && abs(dy) <= 2 * Y_POSITION_DEVIATION_THRESHOLD);
      bool x_ok = abs(dx) <= X_POSITION_DEVIATION_THRESHOLD;
      Serial.print(F("at ")); Serial.print(global.pos.x); Serial.print(',');  Serial.print(global.pos.y);
      Serial.print(F(" to ")); Serial.print(p.x); Serial.print(',');  Serial.println(p.y);

      if ((x_ok && y_ok) || attempts >= MAX_POINT_ATTEMPTS) {
        Serial.println(attempts >= MAX_POINT_ATTEMPTS ? F("giving up, close enough") : F("close enough"));
        sendArrived(p);
        shown_at_ms = millis();
        phase = PH_SHOW;
      } else if (!y_ok) {
        attempts++;
        phase = PH_FACE_Y;
      } else {
        attempts++;
        driveReset();
        phase = PH_DRIVE_X;
      }
      break;
    }

    case PH_FACE_Y:
      if (settleToHeading(Y_AXIS_HEADING, TURN_TOLERANCE_DEG)) {
        driveReset();
        phase = PH_DRIVE_Y;
      }
      break;

    case PH_DRIVE_Y: {
      DriveResult r = driveAxis(true, clampCoord(p.y, MOVE_Y_LIMIT), Y_POSITION_DEVIATION_THRESHOLD);
      if (r == DRIVE_DONE) {
        y_done = true;
        phase = PH_FACE_X;
      } else if (r == DRIVE_FAILED) {
        Serial.println(F("y drive failed"));
        phase = PH_FACE_X;
      }
      break;
    }

    case PH_DRIVE_X: {
      DriveResult r = driveAxis(false, clampCoord(p.x, MOVE_X_LIMIT), X_POSITION_DEVIATION_THRESHOLD);
      if (r != DRIVE_RUNNING) {
        if (r == DRIVE_FAILED) {
          Serial.println(F("x drive failed"));
        }
        phase = PH_FACE_X;
      }
      break;
    }

    case PH_SHOW:
      setTracks(0, 0);
      // wait for the character on the screen to finish showing before continuing
      if (millis() - shown_at_ms >= SHOW_CHARACTER_WAIT_ms) {
        global.poi_queue.dequeue();
        if (global.poi_queue.isEmpty()) {
          // send smiley face (after last character is shown)
          sendSmiley();
        }
        startPoint();
      }
      break;
  }

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
