#include "Plane.h"

/*
  Support for wing-in-ground-effect craft, which operate close enough to the
  surface that a coordinated bank would put a wingtip into the ground or water.

  Two related behaviours are provided:

   1) Flat turning. The navigation controller produces a lateral acceleration
      demand; ArduPlane normally realises it as a coordinated bank
      (calc_nav_roll). When flat turning is enabled the same demand is realised
      as yaw instead, and bank is clamped to WIG_FLAT_ROLL. On an airframe with
      no rudder the yaw output reaches the differential thrust mixer, so
      steering is produced by asymmetric thrust.

   2) A surface taxi phase. AUTO waypoints sequenced ahead of the NAV_TAKEOFF
      item are flown on the surface, using the same navigation solution but
      steering through the ground steering controller, with the takeoff
      inhibited and the speed and throttle limited.

  The guidance law is not reimplemented here. Both behaviours consume the
  existing navigation controller output; only the actuator that realises it
  differs.
*/

bool Plane::wig_option_is_set(WIGOption option) const
{
    return (g2.wig_options & uint16_t(option)) != 0;
}

/*
  true when the aircraft should be taxiing on the surface rather than
  preparing to fly: in an AUTO mission, armed, with the takeoff not yet run and
  the current navigation command something other than the takeoff itself.
 */
bool Plane::in_taxi_phase(void) const
{
    if (!wig_option_is_set(WIGOption::WIG_TAXI_PHASE)) {
        return false;
    }
    if (control_mode != &mode_auto) {
        return false;
    }
    if (!arming.is_armed_and_safety_off()) {
        return false;
    }
    /*
      Note that auto_state.takeoff_complete is NOT a usable test here:
      start_command() sets it true for every nav command, so it is already true
      while sequencing ordinary waypoints. taxi_takeoff_started is latched by
      do_takeoff() instead, and cleared when AUTO is entered.
     */
    if (taxi_takeoff_started) {
        // the mission has reached its takeoff item; we are flying
        return false;
    }
    // taxi only while the mission is still sequencing items ahead of the
    // takeoff. Reaching NAV_TAKEOFF releases the aircraft to fly.
    return mission.get_current_nav_cmd().id != MAV_CMD_NAV_TAKEOFF;
}

/*
  true when navigation turns should be flown flat, using yaw rather than bank
 */
bool Plane::flat_turn_active(void) const
{
    if (flight_stage == AP_FixedWing::FlightStage::TAXI) {
        // on the surface bank is never appropriate, independently of the
        // flat turn option
        return true;
    }
    if (!wig_option_is_set(WIGOption::WIG_FLAT_TURN)) {
        return false;
    }
    // leave the landing flare and its existing wings-level handling alone
    return !landing.is_flaring();
}

/*
  bank limit to apply while flat turning, in centidegrees

  Turn radius goes as V^2/(g*tan(roll)), so this limit is the turn radius. A
  zero limit gives a wings level turn driven only by sideslip side force, which
  on a large span aircraft is a very large radius; the parameter exists so the
  wingtip clearance against turn radius trade is made by the operator rather
  than fixed here.
 */
int32_t Plane::flat_turn_roll_limit_cd(void) const
{
    if (flight_stage == AP_FixedWing::FlightStage::TAXI) {
        // hold the wings level on the surface
        return 0;
    }
    return constrain_int32(g2.wig_flat_roll_max * 100, 0, roll_limit_cd);
}

/*
  steering output for the surface taxi phase

  This reuses the navigation controller's bearing error and the ground steering
  controller, which is the same path ArduPlane already uses to hold a course
  during a takeoff roll or a landing rollout.
 */
int16_t Plane::calc_taxi_steering(void)
{
    const int32_t bearing_error_cd = nav_controller->bearing_error_cd();
    int16_t steering = steerController.get_steering_out_angle_error(bearing_error_cd);
    if (stick_mixing_enabled()) {
        steering = channel_rudder->stick_mixing(steering);
    }
    return constrain_int16(steering, -4500, 4500);
}

/*
  yaw output for a flat turn in flight

  A lateral acceleration is realised in flight as a yaw rate, r = a_lat / V, so
  the navigation controller's lateral acceleration demand is converted to a
  rate demand and closed by the yaw rate controller. Ground speed is floored to
  avoid a singular rate demand at very low speed.
 */
int16_t Plane::calc_flat_turn_yaw(void)
{
    const float speed = MAX(ahrs.groundspeed(), float(aparm.airspeed_min));
    const float lat_accel = nav_controller->lateral_acceleration();
    const float desired_rate = lat_accel / speed;

    const float commanded_rudder = yawController.get_rate_out(degrees(desired_rate),
                                                              get_speed_scaler(),
                                                              false);
    return constrain_int16(commanded_rudder, -4500, 4500);
}

/*
  independent interlock against an unintended takeoff during a surface run

  This check does not depend on the taxi speed or throttle controllers being
  correct, which is the point of it: it exists to catch the case where they are
  not.
 */
void Plane::taxi_check_interlocks(void)
{
    // Diagnostic: when the taxi phase does not engage, report WHICH condition
    // is refusing, rather than leaving it to be inferred from behaviour.
    if (flight_stage != AP_FixedWing::FlightStage::TAXI) {
        static uint32_t last_report_ms;
        const uint32_t now = AP_HAL::millis();
        if (wig_option_is_set(WIGOption::WIG_TAXI_PHASE) &&
            control_mode == &mode_auto &&
            now - last_report_ms > 5000) {
            last_report_ms = now;
            gcs().send_text(MAV_SEVERITY_INFO,
                            "taxi? armed=%u tkoff_started=%u navcmd=%u supp=%u",
                            (unsigned)arming.is_armed_and_safety_off(),
                            (unsigned)taxi_takeoff_started,
                            (unsigned)mission.get_current_nav_cmd().id,
                            (unsigned)throttle_suppressed);
        }
        taxi_above_surface_ms = 0;
        return;
    }

    /*
      Height above home, computed the same way the GLOBAL_POSITION_INT
      relative_alt field is, because that is the measure that reads zero on the
      ground.

      Two other sources were tried first and both were unusable for an
      interlock this tight: relative_altitude and
      adjusted_relative_altitude_cm() each sat above 5 m for seconds on a
      stationary aircraft, aborting the taxi before the first waypoint. Both
      carry a terrain / field elevation adjustment (the vehicle reports
      "Field Elevation Set" and clamps a terrain offset at startup), which is
      appropriate for their normal users and wrong here, where what matters is
      simply "has the aircraft left the surface it started on".
     */
    const float height_above_surface = (current_loc.alt - home.alt) * 0.01f;

    /*
      Debounce before acting. The altitude estimate carries several metres of
      error while the EKF and barometer settle after startup, which is enough
      to trip this interlock on a stationary aircraft that has not moved -
      observed firing before the first waypoint was even reached. Require the
      condition to hold continuously so that a transient cannot abort a
      perfectly good taxi, while a real climb still trips it promptly.
     */
    const uint32_t now_ms = AP_HAL::millis();
    if (height_above_surface <= g2.wig_taxi_abort_alt) {
        taxi_above_surface_ms = 0;
        return;
    }
    if (taxi_above_surface_ms == 0) {
        taxi_above_surface_ms = now_ms;
        return;
    }
    if (now_ms - taxi_above_surface_ms > 1000) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,
                        "Taxi: left surface at %.1fm, aborting",
                        double(height_above_surface));
        /*
          Hand the aircraft to the pilot rather than cutting the throttle.
          Cutting thrust a couple of metres above the water drops the airframe
          back onto the surface, which is a worse outcome than flying away
          under control; leaving the taxi stage also releases the taxi throttle
          and steering limits. Continuing to drive a surface mission while
          airborne is the state this exists to prevent.
         */
        set_mode(mode_fbwa, ModeReason::UNKNOWN);
    }
}
