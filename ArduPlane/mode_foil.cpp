#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

/*
  FOIL mode: FBWA with a height-hold loop on a hydrofoil lift flap.

  See the class comment in mode.h for the behaviour. The loop runs from
  update() at the main loop rate with dt = G_Dt.

  Sign convention: positive flap = trailing edge down = more foil lift
  = craft rises. The height error is (target - height), so the PID
  output is positive when the craft is below the target.
 */

// a height reading older than this is not valid
#define FOIL_HEIGHT_TIMEOUT_MS 200U

// cutoff of the low-pass filter on the differenced height
#define FOIL_HDOT_FILTER_HZ 2.0f

// the output function is scaled set_angle(3000) in SRV_Channel_aux.cpp
#define FOIL_FLAP_ANGLE_LIMIT_DEG 30.0f

const AP_Param::GroupInfo ModeFoil::var_info[] = {

    // @Group: PID_
    // @Path: ../libraries/AC_PID/AC_PID.cpp
    AP_SUBGROUPINFO(pid, "PID_", 1, ModeFoil, AC_PID),

    // @Param: FLAP_MAX
    // @DisplayName: Foil flap maximum deflection
    // @Description: The height loop's flap demand is limited to plus and minus this angle. The FoilFlap servo output (SERVOn_FUNCTION 190) maps +/-30 degrees onto SERVOn_MAX/SERVOn_MIN with 0 degrees at SERVOn_TRIM, so values above 30 have no further effect. Positive flap is trailing edge down, which raises the craft.
    // @Units: deg
    // @Range: 0 30
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FLAP_MAX", 2, ModeFoil, flap_max, 30),

    // @Param: FLAP_NEUT
    // @DisplayName: Foil flap neutral angle
    // @Description: The flap angle commanded whenever the FOIL height loop is not active: in every other flight mode, and after FOIL mode has given up on an invalid height measurement. The flap moves to this angle at the FOIL_SLEW rate.
    // @Units: deg
    // @Range: -30 30
    // @Increment: 0.5
    // @User: Standard
    AP_GROUPINFO("FLAP_NEUT", 3, ModeFoil, flap_neutral, 0),

    // @Param: SLEW
    // @DisplayName: Foil flap slew rate limit
    // @Description: Maximum rate of change of the commanded foil flap angle. Zero disables the limit.
    // @Units: deg/s
    // @Range: 0 1000
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("SLEW", 4, ModeFoil, slew_rate, 120),

    // @Param: SPD_REF
    // @DisplayName: Foil gain schedule reference speed
    // @Description: Ground speed at which the FOIL_PID_ gains apply unscaled. The P, I and D contributions are multiplied by (FOIL_SPD_REF / max(ground speed, FOIL_SPD_MIN)) raised to the power FOIL_SPD_EXP, because foil flap lift authority grows with speed. Zero disables the schedule.
    // @Units: m/s
    // @Range: 0 40
    // @Increment: 0.5
    // @User: Standard
    AP_GROUPINFO("SPD_REF", 5, ModeFoil, speed_ref, 12),

    // @Param: SPD_MIN
    // @DisplayName: Foil gain schedule minimum speed
    // @Description: Ground speeds below this are treated as this speed by the gain schedule, which bounds the gain increase at low speed.
    // @Units: m/s
    // @Range: 1 40
    // @Increment: 0.5
    // @User: Standard
    AP_GROUPINFO("SPD_MIN", 6, ModeFoil, speed_min, 6),

    // @Param: SPD_EXP
    // @DisplayName: Foil gain schedule exponent
    // @Description: Exponent of the speed gain schedule. 0 gives fixed gains, 1 gives gains inversely proportional to speed and 2 inversely proportional to speed squared (constant loop gain for a lift force that scales with dynamic pressure).
    // @Range: 0 2
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SPD_EXP", 7, ModeFoil, speed_exp, 1.0),

    // @Param: HGT_SRC
    // @DisplayName: Foil height source
    // @Description: Source of the height measurement. 0 reads the downward facing rangefinder (orientation 25) directly and corrects it for roll and pitch; it is valid only while the rangefinder status is Good and the reading is less than 200ms old. 1 uses the AHRS height above home, which is for bench and simulator testing only and is never reported invalid.
    // @Values: 0:Rangefinder,1:AHRS height above home (test only)
    // @User: Advanced
    AP_GROUPINFO("HGT_SRC", 8, ModeFoil, height_source, 0),

    // @Param: HGT_MIN
    // @DisplayName: Foil minimum height target
    // @Description: The height target is never lower than this, whether it was captured or is following the height while the elevator stick is deflected. If the pilot flies the craft below this height the target stops following, so the loop closes again and opposes the descent. Set it to the hull clearance wanted.
    // @Units: m
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("HGT_MIN", 9, ModeFoil, height_min, 0.10),

    // @Param: HGT_MAX
    // @DisplayName: Foil maximum height target
    // @Description: The height target is never higher than this, whether it was captured or is following the height while the elevator stick is deflected. If the pilot flies the craft above this height the target stops following, so the loop closes again and opposes the climb before the foil broaches. Set it to about the strut length less 1.5 foil chords.
    // @Units: m
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("HGT_MAX", 10, ModeFoil, height_max, 0.40),

    // @Param: CAP_TC
    // @DisplayName: Foil height capture lookahead time
    // @Description: When the elevator stick returns to centre the target is captured as the measured height plus the vertical speed multiplied by this time, which reduces the overshoot of a capture made while climbing or sinking. Zero captures the measured height. The vertical speed is not known at the moment the mode is entered, so the capture on mode entry always uses the measured height.
    // @Units: s
    // @Range: 0 2
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("CAP_TC", 11, ModeFoil, capture_tc, 0),

    // @Param: DZ
    // @DisplayName: Foil elevator stick dead band
    // @Description: Elevator stick dead band as a fraction of full throw. While the stick is outside this band the pilot is changing height through pitch and the height target follows the measured height (within FOIL_HGT_MIN and FOIL_HGT_MAX). When the stick returns inside the band the height is captured and held. If FOIL_PTCH_MAX is not zero the pitch demand from the stick is also zero inside this band.
    // @Range: 0 0.5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DZ", 12, ModeFoil, stick_deadzone, 0.05),

    // @Param: LOSS_MS
    // @DisplayName: Foil height loss timeout
    // @Description: Time for which an invalid height measurement is tolerated with the flap held at its last angle. After this the flap goes to FOIL_FLAP_NEUT and the mode changes to FBWA.
    // @Units: ms
    // @Range: 0 5000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("LOSS_MS", 13, ModeFoil, loss_ms, 500),

    // @Param: OPTIONS
    // @DisplayName: Foil mode options
    // @Description: Bitmask of FOIL mode options. Bit 0 lets the integrator keep running while the elevator stick is deflected and the target is following the height; by default it is frozen.
    // @Bitmask: 0:Integrate while the stick is deflected
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 14, ModeFoil, options, 0),

    // @Param: PTCH_MAX
    // @DisplayName: Foil mode pitch at full elevator stick
    // @Description: Upper limit of the pitch angle demanded at full elevator stick while in FOIL mode, in both directions. The rate of climb of a foil-borne craft is roughly speed multiplied by the pitch change, so FBWA's pitch limits make a small stick movement a large height rate. Zero uses FBWA's PTCH_LIM_MAX_DEG and PTCH_LIM_MIN_DEG as the limit. FOIL_CLMB_MAX reduces the full stick pitch further as speed rises. STAB_PITCH_DOWN, the FBWA pitch limits and PTCH_TRIM_DEG still apply. If this and FOIL_CLMB_MAX are both zero the pitch demand is exactly FBWA's, with no dead band.
    // @Units: deg
    // @Range: 0 20
    // @Increment: 0.5
    // @User: Standard
    AP_GROUPINFO("PTCH_MAX", 15, ModeFoil, pitch_max, 3),

    // @Param: CLMB_MAX
    // @DisplayName: Foil mode climb rate at full elevator stick
    // @Description: Full elevator stick demands the pitch angle that gives about this rate of climb or descent: FOIL_CLMB_MAX divided by max(ground speed, FOIL_SPD_MIN), in radians, limited by FOIL_PTCH_MAX. This makes the stick to height rate relationship the same at every speed; a fixed pitch authority that is comfortable at low speed flies the foil out of the water at high speed. Zero disables the speed dependence, so FOIL_PTCH_MAX alone applies.
    // @Units: m/s
    // @Range: 0 2
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("CLMB_MAX", 16, ModeFoil, climb_max, 0.3),

    // @Param: GUARD_TC
    // @DisplayName: Foil height band guard lookahead time
    // @Description: While the elevator stick is deflected the height target follows the measured height only as long as the height predicted this far ahead (height plus vertical speed multiplied by this time) stays within FOIL_HGT_MIN and FOIL_HGT_MAX. When the prediction leaves the band the target is held back by the amount of the excess, so the loop starts to oppose the pilot before the band edge is reached. Zero holds the target at the band edge only once the height itself has left the band.
    // @Units: s
    // @Range: 0 2
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("GUARD_TC", 17, ModeFoil, guard_tc, 0.3),

    // @Param: FF_V2
    // @DisplayName: Foil flap speed feed forward
    // @Description: Feed forward of the trim flap angle against speed. The flap angle needed to carry the craft falls with the square of speed, so through an accelerating run the integrator otherwise has to follow the changing trim and a height error builds up. The angle added to the PID output is FOIL_FF_V2 multiplied by (1/V^2 - 1/Ve^2), where V is max(ground speed, FOIL_SPD_MIN) and Ve is the same quantity latched when the mode was entered, so the feed forward is zero at entry. The units are degrees multiplied by metres squared per second squared. Zero disables it.
    // @Range: 0 5000
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("FF_V2", 18, ModeFoil, ff_v2, 0),

    AP_GROUPEND
};

ModeFoil::ModeFoil() :
    ModeFBWA()
{
    AP_Param::setup_object_defaults(this, var_info);
    hdot_filter.set_cutoff_frequency(FOIL_HDOT_FILTER_HZ);
}

/*
  replace FBWA's pitch demand with one scaled to FOIL_PTCH_MAX and
  FOIL_CLMB_MAX at full stick. This repeats the pitch part of ModeFBWA::update() with a
  different gain, so STAB_PITCH_DOWN, the FBWA pitch limits and
  inverted flight are handled as there. PTCH_TRIM_DEG and the
  throttle to pitch feed forward are applied downstream in
  stabilize_pitch() and are not affected. Roll and throttle are not
  touched
 */
void ModeFoil::update_pitch_demand(float pitch_input)
{
    if (!is_positive(pitch_max.get()) && !is_positive(climb_max.get())) {
        // use FBWA's demand as it is
        return;
    }
    if (plane.failsafe.rc_failsafe && plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
        // FBWA failsafe glide: ModeFBWA::update() has zeroed the demands
        return;
    }

    // pitch at full stick, degrees
    float authority_deg;
    if (is_positive(pitch_max.get())) {
        authority_deg = pitch_max.get();
    } else if (pitch_input > 0) {
        authority_deg = plane.aparm.pitch_limit_max.get();
    } else {
        authority_deg = -plane.pitch_limit_min;
    }
    if (is_positive(climb_max.get())) {
        // height rate is speed times pitch change: hold the height
        // rate at full stick constant over speed
        const float speed_ms = MAX(ahrs.groundspeed(), MAX(speed_min.get(), 0.1f));
        authority_deg = MIN(authority_deg, degrees(climb_max.get() / speed_ms));
    }

    plane.nav_pitch_cd = pitch_input * authority_deg * 100.0f;
    plane.adjust_nav_pitch_throttle();
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
    if (plane.fly_inverted()) {
        plane.nav_pitch_cd = -plane.nav_pitch_cd;
    }
}

/*
  measure the height. Returns false if the source is not valid
 */
bool ModeFoil::get_height(float &height_m) const
{
    switch (HeightSource(height_source.get())) {
    case HeightSource::RANGEFINDER: {
#if AP_RANGEFINDER_ENABLED
        const RangeFinder &rf = plane.rangefinder;
        if (rf.status_orient(ROTATION_PITCH_270) != RangeFinder::Status::Good) {
            return false;
        }
        if (AP_HAL::millis() - rf.last_reading_ms(ROTATION_PITCH_270) >= FOIL_HEIGHT_TIMEOUT_MS) {
            return false;
        }
        // the sensor measures along the body Z axis. Project onto the
        // vertical: c.z of the body to NED rotation is cos(roll)*cos(pitch)
        const float cos_tilt = constrain_float(ahrs.get_rotation_body_to_ned().c.z, 0.0f, 1.0f);
        height_m = rf.distance_orient(ROTATION_PITCH_270) * cos_tilt;
        return isfinite(height_m);
#else
        return false;
#endif
    }
    case HeightSource::AHRS_HOME: {
        // at this tag this accessor returns void and falls back to
        // baro internally, so this source cannot be reported invalid
        float pos_d;
        ahrs.get_relative_position_D_home(pos_d);
        height_m = -pos_d;
        return isfinite(height_m);
    }
    }
    return false;
}

/*
  gain schedule multiplier from ground speed. Airspeed is not used as
  the craft this is for has no usable pitot
 */
float ModeFoil::speed_scale(float &speed_ms) const
{
    speed_ms = ahrs.groundspeed();
    if (!is_positive(speed_ref.get())) {
        return 1.0f;
    }
    const float speed_floor = MAX(speed_min.get(), 0.1f);
    return powf(speed_ref.get() / MAX(speed_ms, speed_floor), speed_exp.get());
}

/*
  capture a new target from the current height and vertical speed
 */
void ModeFoil::capture_target()
{
    const float predicted = height + hdot_filter.get() * MAX(capture_tc.get(), 0.0f);
    target = constrain_float(predicted, height_min.get(), MAX(height_min.get(), height_max.get()));
}

/*
  apply the slew limit and write the flap output
 */
void ModeFoil::write_flap(float flap_deg, float dt)
{
    if (flap_initialised && is_positive(slew_rate.get()) && is_positive(dt)) {
        const float max_step = slew_rate.get() * dt;
        flap_deg = constrain_float(flap_deg, last_flap_deg - max_step, last_flap_deg + max_step);
    }
    flap_deg = constrain_float(flap_deg, -FOIL_FLAP_ANGLE_LIMIT_DEG, FOIL_FLAP_ANGLE_LIMIT_DEG);
    last_flap_deg = flap_deg;
    flap_initialised = true;
    SRV_Channels::set_output_scaled(SRV_Channel::k_foil_flap, flap_deg * 100.0f);
}

/*
  slew the flap towards FOIL_FLAP_NEUT. Called from set_servos() on
  every loop in which FOIL is not the active mode
 */
void ModeFoil::output_neutral()
{
    write_flap(flap_neutral.get(), plane.G_Dt);
}

bool ModeFoil::_enter()
{
    float h;
    if (!get_height(h)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FOIL: no valid height");
        return false;
    }

    height = h;
    last_valid_ms = AP_HAL::millis();

    // The first update() seeds the height differencing and captures
    // the target. The vertical speed is not known at that point, so
    // the capture on entry is the measured height
    have_last_height = false;
    hdot_filter.reset(0.0f);
    capture_target();
    capture_pending = true;

    pid.reset_I();
    pid.reset_filter();

    // the speed feed forward is relative to the speed at entry
    engage_speed = MAX(ahrs.groundspeed(), MAX(speed_min.get(), 0.1f));

    tracking = false;
    saturated = false;
    fallback_failed = false;

    return true;
}

void ModeFoil::_exit()
{
    // the flap goes to FOIL_FLAP_NEUT: set_servos() calls
    // output_neutral() from this loop on, which continues the slew
    // from the last angle this mode wrote
    tracking = false;
    saturated = false;
}

void ModeFoil::update()
{
    // roll and pitch demands from the sticks, exactly as FBWA
    ModeFBWA::update();

    // elevator stick against the one dead band that both the pitch
    // demand and the capture logic use. This is the same un-dead-zoned
    // input that ModeFBWA::update() reads
    const float pitch_input = plane.channel_pitch->norm_input();
    const bool deflected = fabsf(pitch_input) > stick_deadzone.get();

    update_pitch_demand(deflected ? pitch_input : 0.0f);

    const float dt = plane.G_Dt;
    const uint32_t now_ms = AP_HAL::millis();

    float speed_ms;
    const float scale = speed_scale(speed_ms);

    float h;
    if (!get_height(h)) {
        // the next valid sample must not be differenced against one from before the gap
        have_last_height = false;
        if (fallback_failed || (now_ms - last_valid_ms >= uint32_t(MAX(loss_ms.get(), 0)))) {
            if (!fallback_failed) {
                gcs().send_text(MAV_SEVERITY_WARNING, "FOIL: height lost, switching to FBWA");
                if (plane.set_mode(plane.mode_fbwa, ModeReason::FAILSAFE)) {
                    // no longer the active mode: set_servos() drives the flap to neutral
                    return;
                }
                // the mode change was refused. Stay here with the loop off
                fallback_failed = true;
            }
            output_neutral();
        } else {
            // hold the output
            write_flap(last_flap_deg, dt);
        }
        write_log(0.0f, speed_ms, scale, STATE_SENSOR_INVALID | (saturated ? STATE_SATURATED : 0));
        return;
    }

    if (fallback_failed) {
        // the height is back after a refused fallback: start again as on entry
        fallback_failed = false;
        hdot_filter.reset(0.0f);
        capture_pending = true;
        pid.reset_I();
    }

    // vertical speed from the low-passed finite difference of the height
    if (have_last_height && is_positive(dt)) {
        hdot_filter.apply((h - height) / dt, dt);
    } else {
        // first sample after a gap: do not let the PID difference across it either
        pid.reset_filter();
    }
    height = h;
    have_last_height = true;
    last_valid_ms = now_ms;

    // elevator stick outside the dead band: the pilot is changing
    // height through pitch. Inside the height band the target follows
    // the height, so that the error is exactly zero and the flap sits
    // at the integrator value. At the edge of the band the target stops
    // following: the loop closes again and opposes a pilot who keeps
    // pulling towards a broach, or pushing towards the hull. The
    // integrator is frozen only while the target follows
    bool target_follows = false;
    if (capture_pending) {
        // first valid sample since entry
        capture_pending = false;
        capture_target();
    }
    if (deflected) {
        // the guard acts on the predicted height, because a clamped
        // target alone does not stop a craft that arrives at the band
        // edge with vertical speed
        const float band_top = MAX(height_min.get(), height_max.get());
        const float height_pred = height + hdot_filter.get() * MAX(guard_tc.get(), 0.0f);
        const float excess = height_pred - constrain_float(height_pred, height_min.get(), band_top);
        target = height - excess;
        target_follows = (height_pred >= height_min.get()) && (height_pred <= band_top);
    } else if (tracking) {
        // stick has returned to centre: capture
        capture_target();
    }
    tracking = deflected;

    // The speed schedule scales the error rather than the output, so
    // that P, I and D are all scheduled and the integrator state stays
    // in flap degrees. The PID is still run while the target follows to
    // keep its filters current; i_scale of zero then freezes the
    // integrator so that a filter residual cannot leak into it
    const float target_eff = height + scale * (target - height);
    const bool freeze_i = target_follows && !option_is_set(Option::INTEGRATE_WHILE_TRACKING);
    const float pid_out = pid.update_all(target_eff, height, dt, saturated, 1.0f, freeze_i ? 0.0f : 1.0f);

    // feed forward of the change in trim flap with speed since entry
    float demand = pid_out;
    if (is_positive(ff_v2.get())) {
        const float v = MAX(speed_ms, MAX(speed_min.get(), 0.1f));
        demand += ff_v2.get() * (1.0f / sq(v) - 1.0f / sq(engage_speed));
    }

    // saturate
    const float limit = constrain_float(flap_max.get(), 0.0f, FOIL_FLAP_ANGLE_LIMIT_DEG);
    const float flap_deg = constrain_float(demand, -limit, limit);
    saturated = fabsf(demand) > limit;

    // slew limit and output
    write_flap(flap_deg, dt);

    write_log(target_eff - height,
              speed_ms,
              scale,
              (tracking ? STATE_TRACKING : 0) | (saturated ? STATE_SATURATED : 0));
}

void ModeFoil::write_log(float error_m, float speed_ms, float scale, uint8_t state) const
{
#if HAL_LOGGING_ENABLED
    const AP_PIDInfo &info = pid.get_pid_info();

    // @LoggerMessage: FOIL
    // @Description: FOIL mode hydrofoil height loop
    // @Field: TimeUS: Time since system startup
    // @Field: Tar: height target
    // @Field: Hgt: measured height (last valid value while the sensor is invalid)
    // @Field: HDot: filtered vertical speed from the differenced height
    // @Field: Err: speed-scheduled height error fed to the PID
    // @Field: P: proportional part of the flap demand
    // @Field: I: integral part of the flap demand
    // @Field: D: derivative part of the flap demand
    // @Field: Out: flap angle written to the FoilFlap output after saturation and slew limiting
    // @Field: Spd: ground speed used by the gain schedule
    // @Field: Scl: gain schedule multiplier
    // @Field: St: state bitmask: 1 elevator stick deflected (the target follows the height inside the FOIL_HGT_MIN to FOIL_HGT_MAX band), 2 PID output saturated, 4 height measurement invalid
    AP::logger().WriteStreaming("FOIL", "TimeUS,Tar,Hgt,HDot,Err,P,I,D,Out,Spd,Scl,St",
                                "smmnmddddn--",
                                "F000000000--",
                                "QffffffffffB",
                                AP_HAL::micros64(),
                                (double)target,
                                (double)height,
                                (double)hdot_filter.get(),
                                (double)error_m,
                                (double)info.P,
                                (double)info.I,
                                (double)info.D,
                                (double)last_flap_deg,
                                (double)speed_ms,
                                (double)scale,
                                state);
#endif
}
