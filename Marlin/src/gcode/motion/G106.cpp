#include "../gcode.h"
#include "../../module/planner.h"

void GcodeSuite::G106(){
  static constexpr int QUEUE_DEPTH = 5;                                // Insert up to this many movements
  static constexpr float target_lag = 0.25f,                           // Aim for 1/4 second lag
                          seg_time = target_lag / QUEUE_DEPTH;          // 0.05 seconds, short segments inserted every 1/20th of a second
  static constexpr millis_t timer_limit_ms = millis_t(seg_time * 500); // 25 ms minimum delay between insertions

  // The planner can merge/collapse small moves, so the movement queue is unreliable to control the lag
  static millis_t next_run = 0;
  if (PENDING(millis(), next_run)) return;
  next_run = millis() + timer_limit_ms;

  // Only inject a command if the planner has fewer than 5 moves
  if (planner.movesplanned() >= QUEUE_DEPTH)
    return;

  // Normalized jog values are 0 for no movement and -1 or +1 for as max feedrate (nonlinear relationship)
  // Jog are initialized to zero and handling input can update values but doesn't have to
  // You could use a two-axis joystick and a one-axis keypad and they might work together
  xyz_float_t norm_jog{0};

  LOOP_NUM_AXES(i) {
    if ( parser.seenval(AXIS_CHAR(i)) ) {
      norm_jog[i] = parser.value_axis_units((AxisEnum)i) * 0.01;
      if (norm_jog[i] < 0) {
        norm_jog[i] = -sq(norm_jog[i]);
      }
      else {
        norm_jog[i] = sq(norm_jog[i]);
      }
    }
  }

  // norm_jog values of [-1 .. 1] maps linearly to [-feedrate .. feedrate]
  xyz_float_t move_dist{0};
  float hypot2 = 0;
  LOOP_NUM_AXES(i) if (norm_jog[i]) {
    move_dist[i] = seg_time * norm_jog[i] * 2 * planner.settings.max_feedrate_mm_s[i];
    hypot2 += sq(move_dist[i]);
  }

  if (!UNEAR_ZERO(hypot2)) {
    current_position += move_dist;
    apply_motion_limits(current_position);
    const float length = sqrt(hypot2);
    PlannerHints hints(length);
    planner.buffer_line(current_position, length / seg_time, active_extruder, hints);
  }
}