#![no_std]
#![no_main]
#![forbid(unsafe_code)]

mod dm_mc02;

use core::sync::atomic::{AtomicU32, Ordering};
use dm_mc02::Can1;
use embassy_executor::Spawner;
use embassy_stm32::{can::Frame};
use embassy_time::{Duration, Instant, Ticker};
use embedded_can::Id;


// This rig uses GM6020 voltage commands for motor IDs 1 to 4.
const MOTOR_COMMAND_ID: u16 = 0x1FF;

const YAW_MOTOR_ID: u16 = 1;
const PITCH_MOTOR_ID: u16 = 2;

// TODO: Fill these values after physical calibration.
const YAW_REFERENCE_RAD: f32 = 0.0;
const PITCH_REFERENCE_RAD: f32 = 0.0;

const PREPARE_MAX_RPM: f32 = 0.0;
const PREPARE_MAX_VOLTAGE: f32 = 0.0;

const PREPARE_ANGLE_TOLERANCE_RAD: f32 = 0.0;
const PREPARE_SPEED_TOLERANCE_RPM: f32 = 0.0;

const PREPARE_STABLE_TICKS: u32 = 0;
const PREPARE_TIMEOUT_TICKS: u32 = 0;

// TODO: Tune these values on the real gimbal.
const RUNNING_STICK_DEADBAND: i16 = 20;

const YAW_MAX_RATE_RAD_S: f32 = 0.0;
const PITCH_MAX_RATE_RAD_S: f32 = 0.0;

// TODO: Verify the directions on the real gimbal.
const YAW_DIRECTION: f32 = 1.0;
const PITCH_DIRECTION: f32 = 1.0;

const RUNNING_MAX_RPM: f32 = 0.0;
const RUNNING_MAX_VOLTAGE: f32 = 0.0;

// TODO: Tune these values on the real gimbal.
const STALL_MIN_TARGET_RPM: f32 = 0.0;
const STALL_MIN_VOLTAGE: f32 = 0.0;
const STALL_MAX_SPEED_RPM: f32 = 0.0;
const STALL_CONFIRM_TICKS: u32 = 500;

// TODO: Tune these values on the real gimbal.
const OSCILLATION_MIN_SPEED_RPM: f32 = 0.0;
const OSCILLATION_MIN_AMPLITUDE_RAD: f32 = 0.125 * core::f32::consts::PI;
const OSCILLATION_MIN_FREQUENCY_HZ: f32 = 5.0;
const OSCILLATION_CONFIRM_CYCLES: u32 = 10;

const YAW_FEEDBACK_CAN_ID: u16 = 0x204 + YAW_MOTOR_ID;
const PITCH_FEEDBACK_CAN_ID: u16 = 0x204 + PITCH_MOTOR_ID;

// Ticks without remote input before it counts as lost.
const REMOTE_SILENT_TICKS: u16 = 100;
const FEEDBACK_TIMEOUT_TICKS: u16 = 20;

const SWITCH_UP: u8 = 1;
const SWITCH_DOWN: u8 = 2;
const SWITCH_MIDDLE: u8 = 3;

const RECOVERY_BUTTON: u8 = SWITCH_UP;
const DISABLE_BUTTON: u8 = SWITCH_DOWN;
const ENABLE_BUTTON: u8 = SWITCH_MIDDLE;

const TICK: Duration = Duration::from_millis(1);
const TICK_S: f32 = 0.001;

/// Command frames dropped because the CAN transmit queue was full, for example while no
/// motor acknowledges frames.
pub static TX_FAILED: AtomicU32 = AtomicU32::new(0);

/// Feedback and voltage command for one motor.
#[derive(Default)]
pub struct Motor {
    pub temperature_c: u8,
    /// Raw single-turn encoder count: 0..=8191.
    pub encoder_count: u16,
    /// Continuous motor shaft angle in rad.
    pub angle_rad: f32,
    pub speed_rpm: f32,
    /// Feedback current as sent on the wire. The motor manual gives no scale to amperes.
    pub current_raw: i16,
    /// Voltage command counts, not volts; protocol range +/-25000.
    pub command_voltage_raw: f32,
}

impl Motor {
    /// Clear the voltage command in RAM. The motor receives zero when
    /// `Control::transmit` next runs, within 1 ms.
    pub fn disable(&mut self) {
        self.command_voltage_raw = 0.0;
    }

    /// Read the motor's feedback data.
    pub fn read(&mut self, data: &[u8; 8]) {
        // TODO: decode angle, speed and current as the manual describes.
        // Decode raw feedback values.
        self.encoder_count =
            u16::from_be_bytes([data[0], data[1]]);
        let speed_raw: i16 = i16::from_be_bytes([data[2], data[3]]);
        let current_raw: i16 = i16::from_be_bytes([data[4], data[5]]);

        // Store feedback values.
        self.speed_rpm = speed_raw as f32;
        self.current_raw = current_raw;
        self.temperature_c = data[6];
    }
    
}

#[derive(Default)]
pub struct OscillationDetector {
    /// Previous valid motor speed direction: -1 or 1.
    pub previous_speed_direction: i8,

    /// Whether a valid speed direction has been observed.
    pub speed_direction_initialized: bool,

    /// Previous extremum direction: 1 for maximum, -1 for minimum.
    pub last_extremum_direction: i8,

    /// Angle at the previous extremum.
    pub last_extremum_angle_rad: f32,

    /// Whether a previous extremum has been recorded.
    pub extremum_initialized: bool,

    /// Direction of the reference extremum for full-cycle measurement.
    pub oscillation_reference_direction: i8,

    /// Whether a reference extremum has been recorded.
    pub oscillation_reference_initialized: bool,

    /// Ticks since the previous reference extremum.
    pub oscillation_elapsed_ticks: u32,

    /// Number of consecutive qualifying oscillation cycles.
    pub oscillation_cycles: u32,
}

/// PID gains, plus the integral and history kept between ticks.
// TODO: add gains, history, an integral limit and a symmetric output limit `maxout`.
#[derive(Default)]
pub struct Pid {
    kp: f32,
    ki: f32,
    kd: f32,

    integral_limit: f32,

    previous_measured: f32,
    integral: f32,

    initialized: bool,
}

impl Pid {

    /// Create a new PID controller with parameters.
    pub fn new(kp: f32, ki: f32, kd: f32, integral_limit: f32) -> Self{
        Self {
            kp,
            ki,
            kd,
            integral_limit,
            previous_measured: 0.0,
            integral: 0.0,
            initialized: false,
        }
    }

    // TODO: clear the integral and history.
    pub fn reset(&mut self) {
        self.initialized = false;
        self.previous_measured = 0.0;
        self.integral = 0.0;
    }

    /// `target` and `measured` share a unit, set by the caller. The position loop takes rad
    /// and returns rpm; the speed loop takes rpm and returns voltage counts. `dt_s` is in seconds.
    /// If a position loop returns rad/s, convert it to rpm before passing it to the speed loop.
    pub fn calculate(&mut self, target: f32, measured: f32, dt_s: f32, maxout: f32) -> f32 {

        if !dt_s.is_finite() || dt_s <= 0.0 {
            return 0.0;
        }
        if maxout <= 0.0 {
            return 0.0;
        }

        // TODO: compute the PID with dt_s, and limit the integral and output.
        let error: f32 = target - measured;
        let p_term: f32 = self.kp * error;

        let candidate_integral: f32 = (self.integral + error * dt_s)
        .clamp(-self.integral_limit, self.integral_limit);

        let candidate_i_term: f32 = self.ki * candidate_integral;

        let d_term: f32 = if self.initialized {
            self.kd * -(measured - self.previous_measured) / dt_s
        } else {
            0.0
        };

        let candidate_output: f32 = p_term + candidate_i_term + d_term;

        let pushing_further_into_saturation: bool = (candidate_output > maxout && candidate_integral > 0.0)
            || (candidate_output < -maxout && candidate_integral < 0.0);

        if !pushing_further_into_saturation {
            self.integral = candidate_integral;
        }

        let i_term = self.ki * self.integral;

        let output = (p_term + i_term + d_term).clamp(-maxout, maxout);

        self.previous_measured = measured;
        self.initialized = true;

        return output;
    }
}

/// The gimbal's stage, which decides the allowed operations and outputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Zero output. Waits for valid remote input and fresh feedback from both motors,
    /// then for a new enable request.
    Waiting,
    /// Recenters both axes to the reference pose with limited speed and output.
    /// Enters `Running` once both axes settle.
    Prepare,
    /// The left stick moves each axis's `target_angle_rad`, and its cascade tracks it.
    Running,
    /// Zero output until a manual recovery.
    Error,
}

/// One gimbal axis: its motor, targets and cascaded PIDs.
#[derive(Default)]
pub struct Axis {
    pub motor: Motor,
    /// Position loop target in rad.
    pub target_angle_rad: f32,
    /// Speed loop target in rpm, set from the position loop output.
    pub target_rpm: f32,
    
    /// The previous encoder count.
    pub previous_encoder_count: u16,
    /// Whether the angle has been initialized.
    pub angle_initialized: bool,

    /// The minimum and maximum angles for this axis.
    pub min_angle_rad: f32,
    pub max_angle_rad: f32,

    /// Position loop: rad in, rpm out.
    pub position_pid: Pid,
    /// Speed loop: rpm in, voltage counts out.
    pub speed_pid: Pid,

    /// Number of consecutive ticks satisfying the stall conditions.
    pub stall_ticks: u32,
    /// Oscillation detector for this axis.
    pub oscillation_detector: OscillationDetector,

    pub fault: Option<FaultReason>,
}

impl Axis {
    pub fn update_angle(&mut self) {
        const ENCODER_COUNTS_PER_REV: f32 = 8192.0;
        const RAD_PER_COUNT: f32 =
            core::f32::consts::TAU / ENCODER_COUNTS_PER_REV;

        let current = self.motor.encoder_count;

        // First feedback: establish the initial angle.
        if !self.angle_initialized {
            self.motor.angle_rad = current as f32 * RAD_PER_COUNT;
            self.previous_encoder_count = current;
            self.angle_initialized = true;
            return;
        }

        // Calculate the difference between two encoder readings.
        let mut delta =
            current as i32 - self.previous_encoder_count as i32;

        // Unwrap the difference across the 8191 <-> 0 boundary.
        if delta > 4096 {
            delta -= 8192;
        } else if delta < -4096 {
            delta += 8192;
        }

        self.motor.angle_rad += delta as f32 * RAD_PER_COUNT;
        self.previous_encoder_count = current;
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaultReason {
    Stall,
    Oscillation,
}

/// State shared by every control tick.
// TODO: add feedback ages and a fault record.
pub struct Control {
    pub state: State,
    pub yaw: Axis,
    pub pitch: Axis,
    pub remote: Option<dm_mc02::Dr16>,
    pub remote_silent_ticks: u16,

    pub last_right_switch: Option<u8>,
    pub last_left_switch: Option<u8>,

    pub yaw_feedback_age: u16,
    pub pitch_feedback_age: u16,

    pub prepare_elapsed_ticks: u32,
    pub prepare_stable_ticks: u32,
}

/// Start in `Waiting` with no remote input.
/// Set the initial value of each field you add here, including the PID gains.
impl Default for Control {
    fn default() -> Self {
        Self {
            state: State::Waiting,

            yaw: Axis {
                position_pid: Pid::new(
                    0.0, 0.0, 0.0, 0.0
                ),
                speed_pid: Pid::new(
                    0.0, 0.0, 0.0, 0.0
                ),
                max_angle_rad: 0.0,
                min_angle_rad: 0.0,
                ..Default::default()
            },

            pitch: Axis {
                position_pid: Pid::new(
                    0.0, 0.0, 0.0, 0.0
                ),
                speed_pid: Pid::new(
                    0.0, 0.0, 0.0, 0.0
                ),
                max_angle_rad: 0.0,
                min_angle_rad: 0.0,
                ..Default::default()
            },

            remote: None,
            remote_silent_ticks: REMOTE_SILENT_TICKS,

            last_right_switch: None,
            last_left_switch: None,

            yaw_feedback_age: FEEDBACK_TIMEOUT_TICKS,
            pitch_feedback_age: FEEDBACK_TIMEOUT_TICKS,

            prepare_elapsed_ticks: 0,
            prepare_stable_ticks: 0,
        }
    }
}

impl Control {
    /// Clear both motors' voltage commands in RAM.
    pub fn disable_all(&mut self) {
        self.yaw.motor.disable();
        self.pitch.motor.disable();
    }

    /// Reset the position and speed PIDs for both axes.
    pub fn reset_pids(&mut self) {
        self.yaw.position_pid.reset();
        self.yaw.speed_pid.reset();
        self.pitch.position_pid.reset();
        self.pitch.speed_pid.reset();
    }

    /// Clamp the target angle to the axis's limits.
    pub fn clamp_target(
        &self,
        target: f32,
        min_limit: f32,
        max_limit: f32,
    ) -> f32 {
        target.clamp(min_limit, max_limit)
    }

    /// Normalize a stick input to the range [-1.0, 1.0].
    fn normalize_stick(&mut self, channel: i16) -> f32 {
        let channel = channel.clamp(-660, 660);
        let magnitude = channel.abs();

        if magnitude <= RUNNING_STICK_DEADBAND {
            return 0.0;
        }

        let sign = if channel < 0 { -1.0 } else { 1.0 };

        let magnitude = (magnitude - RUNNING_STICK_DEADBAND) as f32
            / (660 - RUNNING_STICK_DEADBAND) as f32;

        sign * magnitude
    }

    /// Update the stall detection for a given axis.
    fn update_stall_detection(axis: &mut Axis, feedback_fresh: bool) {
        // Zero configuration means the detector is not calibrated yet.
        if STALL_MIN_TARGET_RPM <= 0.0
            || STALL_MIN_VOLTAGE <= 0.0
            || STALL_MAX_SPEED_RPM <= 0.0
            || STALL_CONFIRM_TICKS == 0
        {
            axis.stall_ticks = 0;
            return;
        }

        // Do not detect a new fault if one has already been recorded.
        if axis.fault.is_some() {
            return;
        }

        // Stale feedback cannot be used to determine whether the motor is stalled.
        if !feedback_fresh {
            axis.stall_ticks = 0;
            return;
        }

        let has_motion_demand =
            axis.target_rpm.abs() > STALL_MIN_TARGET_RPM;

        let has_large_output =
            axis.motor.command_voltage_raw.abs() > STALL_MIN_VOLTAGE;

        let motor_is_slow =
            axis.motor.speed_rpm.abs() < STALL_MAX_SPEED_RPM;

        if has_motion_demand && has_large_output && motor_is_slow {
            axis.stall_ticks = axis.stall_ticks.saturating_add(1);
        } else {
            axis.stall_ticks = 0;
        }

        if axis.stall_ticks >= STALL_CONFIRM_TICKS {
            axis.fault = Some(FaultReason::Stall);
        }
    }

    fn update_oscillation_detection(
        axis: &mut Axis,
        feedback_fresh: bool,
    ) {
        // Zero configuration means the detector is not calibrated.
        if OSCILLATION_MIN_SPEED_RPM <= 0.0
            || OSCILLATION_MIN_AMPLITUDE_RAD <= 0.0
            || OSCILLATION_MIN_FREQUENCY_HZ <= 0.0
            || OSCILLATION_CONFIRM_CYCLES == 0
        {
            axis.oscillation_detector = OscillationDetector::default();
            return;
        }

        if axis.fault.is_some() {
            return;
        }

        if !feedback_fresh {
            axis.oscillation_detector = OscillationDetector::default();
            return;
        }
        
        axis.oscillation_detector.oscillation_elapsed_ticks =
        axis.oscillation_detector.oscillation_elapsed_ticks.saturating_add(1);

        let speed = axis.motor.speed_rpm;

        // Ignore low-speed noise instead of treating it as a direction.
        if speed.abs() < OSCILLATION_MIN_SPEED_RPM {
            return;
        }

        let current_direction: i8 = if speed > 0.0 { 1 } else { -1 };

        if !axis.oscillation_detector.speed_direction_initialized {
            axis.oscillation_detector.previous_speed_direction = current_direction;
            axis.oscillation_detector.speed_direction_initialized = true;
            return;
        }

        // No direction reversal: no new extremum.
        if current_direction == axis.oscillation_detector.previous_speed_direction {
            return;
        }

        // A reversal indicates an extremum.
        let extremum_direction = axis.oscillation_detector.previous_speed_direction;
        let extremum_angle = axis.motor.angle_rad;

        axis.oscillation_detector.previous_speed_direction = current_direction;

        // Record the first extremum.
        if !axis.oscillation_detector.extremum_initialized {
            axis.oscillation_detector.last_extremum_direction = extremum_direction;
            axis.oscillation_detector.last_extremum_angle_rad = extremum_angle;
            axis.oscillation_detector.extremum_initialized = true;
            return;
        }

        // Measure the distance between adjacent extrema.
        let amplitude =
            (extremum_angle - axis.oscillation_detector.last_extremum_angle_rad).abs();

        axis.oscillation_detector.last_extremum_direction = extremum_direction;
        axis.oscillation_detector.last_extremum_angle_rad = extremum_angle;

        if amplitude < OSCILLATION_MIN_AMPLITUDE_RAD {
            axis.oscillation_detector.oscillation_cycles = 0;
            axis.oscillation_detector.oscillation_reference_initialized = false;
            axis.oscillation_detector.oscillation_elapsed_ticks = 0;
            return;
        }

        // The first extremum becomes the reference for full-cycle measurement.
        if !axis.oscillation_detector.oscillation_reference_initialized {
            axis.oscillation_detector.oscillation_reference_direction = extremum_direction;
            axis.oscillation_detector.oscillation_reference_initialized = true;
            axis.oscillation_detector.oscillation_elapsed_ticks = 0;
            return;
        }

        // The opposite extremum is only half a cycle.
        if extremum_direction != axis.oscillation_detector.oscillation_reference_direction {
            return;
        }

        // We have returned to the same type of extremum: one full cycle.
        let period_ticks = axis.oscillation_detector.oscillation_elapsed_ticks;

        axis.oscillation_detector.oscillation_elapsed_ticks = 0;

        if period_ticks == 0 {
            axis.oscillation_detector.oscillation_cycles = 0;
            return;
        }

        let frequency_hz =
            1.0 / (period_ticks as f32 * TICK_S);

        if frequency_hz >= OSCILLATION_MIN_FREQUENCY_HZ {
            axis.oscillation_detector.oscillation_cycles =
                axis.oscillation_detector.oscillation_cycles.saturating_add(1);
        } else {
            axis.oscillation_detector.oscillation_cycles = 0;
        }

        if axis.oscillation_detector.oscillation_cycles >= OSCILLATION_CONFIRM_CYCLES {
            axis.fault = Some(FaultReason::Oscillation);
        }
    }

    /// Update fault detection counters using the latest feedback and commands.
    pub fn update_fault_detection(&mut self) {
        let active_state =
            self.state == State::Prepare || self.state == State::Running;

        if !active_state {
            self.yaw.stall_ticks = 0;
            self.pitch.stall_ticks = 0;

            self.yaw.oscillation_detector = OscillationDetector::default();
            self.pitch.oscillation_detector = OscillationDetector::default();

            return;
        }

        let yaw_feedback_fresh =
            self.yaw_feedback_age < FEEDBACK_TIMEOUT_TICKS;

        let pitch_feedback_fresh =
            self.pitch_feedback_age < FEEDBACK_TIMEOUT_TICKS;

        Self::update_stall_detection(&mut self.yaw, yaw_feedback_fresh);
        Self::update_stall_detection(&mut self.pitch, pitch_feedback_fresh);

        Self::update_oscillation_detection(&mut self.yaw, yaw_feedback_fresh);
        Self::update_oscillation_detection(&mut self.pitch, pitch_feedback_fresh);
    }

    /// Runs every 1 ms: read the remote and motor feedback, update the state, then
    /// compute and send the outputs. Missing remote input stops output; a latched Error is preserved.
    pub fn tick(&mut self, can: &mut Can1) {
        self.read_remote();
        self.read_feedback(can);
        self.update_fault_detection();
        self.update_mode();
        self.update();
        self.transmit(can);
    }

    /// Take the latest input published by the remote task. After an invalid frame, or
    /// `REMOTE_SILENT_TICKS` ticks without one, clear the commands; return to Waiting unless already in Error.
    pub fn read_remote(&mut self) {
        self.remote_silent_ticks = self.remote_silent_ticks.saturating_add(1);
        if let Some(remote) = dm_mc02::REMOTE.try_take() {
            self.remote = remote;
            if self.remote.is_some() {
                self.remote_silent_ticks = 0;
            }
        }
        if self.remote_silent_ticks >= REMOTE_SILENT_TICKS {
            self.remote = None;
        }
        if self.remote.is_none() {
            self.last_left_switch = None;
            self.last_right_switch = None;

            if self.state != State::Error {
                self.switch_mode(State::Waiting);
            }
        }
    }

    /// Read every queued CAN frame without waiting for more.
    pub fn read_feedback(&mut self, can: &mut Can1) {
        let receiver = can.reader();
        self.yaw_feedback_age = self.yaw_feedback_age.saturating_add(1);
        self.pitch_feedback_age = self.pitch_feedback_age.saturating_add(1);
        while let Ok(received) = receiver.try_receive() {
            match received {
                Ok(envelope) => {
                    // TODO: check the ID and length, pass the data to the matching axis's
                    // `Motor::read`, and record its feedback age.
                    let frame = envelope.frame;

                    // Only standard IDs are used for feedback; extended IDs are ignored.
                    let can_id = match frame.id() {
                        Id::Standard(id) => id.as_raw(),
                        Id::Extended(_) => continue,
                    };

                    // GM6020 feedback frames are always 8 bytes long; ignore any other length.
                    let data: &[u8; 8] = match frame.data().try_into() {
                        Ok(data) => data,
                        Err(_) => continue,
                    };

                    match can_id {
                        YAW_FEEDBACK_CAN_ID => {
                            let feedback_was_stale: bool =
                                self.yaw_feedback_age >= FEEDBACK_TIMEOUT_TICKS;
                            
                            self.yaw.motor.read(data);

                            if feedback_was_stale {
                                // Reinitialize the yaw angle to the new feedback.
                                self.yaw.angle_initialized = false;
                            }

                            self.yaw.update_angle();
                            self.yaw_feedback_age = 0;
                        }

                        PITCH_FEEDBACK_CAN_ID => {
                            let feedback_was_stale: bool =
                                self.pitch_feedback_age >= FEEDBACK_TIMEOUT_TICKS;

                            self.pitch.motor.read(data);

                            if feedback_was_stale {
                                // Reinitialize the pitch angle to the new feedback.
                                self.pitch.angle_initialized = false;
                            }

                            self.pitch.update_angle();
                            self.pitch_feedback_age = 0;
                        }

                        _ => {
                            // Ignore any other CAN IDs.
                        }
                    }
                }
                Err(error) => dm_mc02::log_can_error(error),
            }
        }
    }

    /// Choose the state from the remote, feedback and fault conditions.
    // TODO: the transitions, and initialising targets and PIDs on entering a state.
    pub fn update_mode(&mut self) {
        // No valid remote input: disarm and wait for a fresh switch sequence.
        let remote: dm_mc02::Dr16 = match self.remote {
            Some(remote) => remote,
            None => {
                self.last_right_switch = None;
                self.last_left_switch = None;

                if self.state != State::Error {
                    self.switch_mode(State::Waiting);
                }
                return;
            }
        };

        let switch_r: u8 = remote.right_switch;
        let switch_l: u8 = remote.left_switch;

        let enable_position: bool = switch_r == ENABLE_BUTTON;

            // A fault has higher priority than a normal feedback timeout.
        let fault_detected =
            self.yaw.fault.is_some() || self.pitch.fault.is_some();

        if fault_detected
        {
            self.switch_mode(State::Error);
        }


        let feedback_timeout: bool =
            self.yaw_feedback_age >= FEEDBACK_TIMEOUT_TICKS
            || self.pitch_feedback_age >= FEEDBACK_TIMEOUT_TICKS;
        
        if feedback_timeout && self.state != State::Error {
            self.switch_mode(State::Waiting);
        }
        
        match self.state {
            State::Waiting => {
                // TODO: enter Prepare after valid feedback and a new enable request.
                // The lower position arms the next enable operation.
                if self.last_right_switch == Some(DISABLE_BUTTON)
                 && enable_position && !feedback_timeout {
                    self.switch_mode(State::Prepare);
                }
            }
            State::Prepare => {
                // TODO: enter Running once recentering settles; stop on a timeout or fault.
                if !enable_position {
                    self.switch_mode(State::Waiting);
                } else if self.prepare_stable_ticks >= PREPARE_STABLE_TICKS {
                    self.switch_mode(State::Running);
                } else if self.prepare_elapsed_ticks >= PREPARE_TIMEOUT_TICKS {
                    self.switch_mode(State::Waiting);
                }
            }
            State::Running => {
                // TODO: handle disable/feedback loss; stall or sustained high-frequency oscillation enters Error.
                if !enable_position {
                    self.switch_mode(State::Waiting);
                }
            }
            State::Error => {
                // TODO: keep zero output until a new remote recovery gesture; hold current angles,
                // clear PID history and detection counters, then return to Running.
                if self.last_left_switch == Some(SWITCH_DOWN)
                    && switch_l == RECOVERY_BUTTON
                {
                    if enable_position && !feedback_timeout {
                        self.switch_mode(State::Running);
                    } else {
                        self.switch_mode(State::Waiting);
                    }
                }
            }
        }
        
        self.last_left_switch = Some(switch_l);
        self.last_right_switch = Some(switch_r);
    }

    pub fn switch_mode(&mut self, new_state: State) {
        if self.state == new_state {
            return;
        }

        let previous_state: State = self.state;
        self.state = new_state;

        self.yaw.stall_ticks = 0;
        self.pitch.stall_ticks = 0;

        match new_state{
            State::Waiting => {
                self.disable_all();
                self.reset_pids();

                self.yaw.target_rpm = 0.0;
                self.pitch.target_rpm = 0.0;

                self.yaw.target_angle_rad = self.yaw.motor.angle_rad;
                self.pitch.target_angle_rad = self.pitch.motor.angle_rad;

                self.yaw.angle_initialized = false;
                self.pitch.angle_initialized = false;

                self.yaw.fault = None;
                self.pitch.fault = None;

                self.prepare_elapsed_ticks = 0;
                self.prepare_stable_ticks = 0;

                self.yaw.oscillation_detector = OscillationDetector::default();
                self.pitch.oscillation_detector = OscillationDetector::default();
            }
            State::Prepare => {
                self.yaw.target_angle_rad = self.clamp_target(
                    YAW_REFERENCE_RAD,
                    self.yaw.min_angle_rad,
                    self.yaw.max_angle_rad,
                );

                self.pitch.target_angle_rad = self.clamp_target(
                    PITCH_REFERENCE_RAD,
                    self.pitch.min_angle_rad,
                    self.pitch.max_angle_rad,
                );

                self.yaw.target_rpm = 0.0;
                self.pitch.target_rpm = 0.0;

                self.reset_pids();
                self.disable_all();

                self.prepare_elapsed_ticks = 0;
                self.prepare_stable_ticks = 0;
            }
            State::Running => {
                if previous_state == State::Error {
                    // Recovery: hold the current motor positions.
                    self.yaw.target_angle_rad = self.yaw.motor.angle_rad;
                    self.pitch.target_angle_rad = self.pitch.motor.angle_rad;

                    self.yaw.target_rpm = 0.0;
                    self.pitch.target_rpm = 0.0;

                    self.reset_pids();

                    self.yaw.fault = None;
                    self.pitch.fault = None;
                }
            }
            State::Error => {
                self.disable_all();
                self.reset_pids();
            }
        }
    }

    /// Set targets for the current state, run each axis's PIDs and write the voltage commands.
    // TODO: limited recentering in Prepare, remote cascade control in Running, and
    // `disable_all` in Waiting and Error. Each tick is 0.001 s.
    pub fn update(&mut self) {
        match self.state {
            State::Waiting => {
                self.disable_all();
            }
            State::Prepare => {
                self.yaw.target_rpm = self.yaw.position_pid.calculate(
                    self.yaw.target_angle_rad,
                    self.yaw.motor.angle_rad,
                    TICK_S,
                    PREPARE_MAX_RPM,
                );

                // Yaw speed loop.
                self.yaw.motor.command_voltage_raw =
                    self.yaw.speed_pid.calculate(
                        self.yaw.target_rpm,
                        self.yaw.motor.speed_rpm,
                        TICK_S,
                        PREPARE_MAX_VOLTAGE,
                    );

                // Pitch position loop.
                self.pitch.target_rpm = self.pitch.position_pid.calculate(
                    self.pitch.target_angle_rad,
                    self.pitch.motor.angle_rad,
                    TICK_S,
                    PREPARE_MAX_RPM,
                );

                // Pitch speed loop.
                self.pitch.motor.command_voltage_raw =
                    self.pitch.speed_pid.calculate(
                        self.pitch.target_rpm,
                        self.pitch.motor.speed_rpm,
                        TICK_S,
                        PREPARE_MAX_VOLTAGE,
                    );

                // Check whether both axes are stable.
                let yaw_stable: bool =
                    (self.yaw.target_angle_rad - self.yaw.motor.angle_rad).abs()
                        < PREPARE_ANGLE_TOLERANCE_RAD
                    && self.yaw.motor.speed_rpm.abs()
                        < PREPARE_SPEED_TOLERANCE_RPM;

                let pitch_stable: bool =
                    (self.pitch.target_angle_rad - self.pitch.motor.angle_rad).abs()
                        < PREPARE_ANGLE_TOLERANCE_RAD
                    && self.pitch.motor.speed_rpm.abs()
                        < PREPARE_SPEED_TOLERANCE_RPM;

                if yaw_stable && pitch_stable {
                    self.prepare_stable_ticks =
                        self.prepare_stable_ticks.saturating_add(1);
                } else {
                    self.prepare_stable_ticks = 0;
                }

                self.prepare_elapsed_ticks =
                    self.prepare_elapsed_ticks.saturating_add(1);
            }
            State::Running => {
                if let Some(remote) = self.remote {
                    // Normalize the left stick input.
                    let yaw_input: f32 = self.normalize_stick(remote.channels[2]);
                    let pitch_input: f32 = self.normalize_stick(remote.channels[3]);

                    // Update the target angles using the commanded angular rates.
                    self.yaw.target_angle_rad +=
                        yaw_input
                        * YAW_DIRECTION
                        * YAW_MAX_RATE_RAD_S
                        * TICK_S;

                    self.pitch.target_angle_rad +=
                        pitch_input
                        * PITCH_DIRECTION
                        * PITCH_MAX_RATE_RAD_S
                        * TICK_S;

                    // Clamp target angles to the mechanical limits.
                    self.yaw.target_angle_rad = self.clamp_target(
                        self.yaw.target_angle_rad,
                        self.yaw.min_angle_rad,
                        self.yaw.max_angle_rad,
                    );

                    self.pitch.target_angle_rad = self.clamp_target(
                        self.pitch.target_angle_rad,
                        self.pitch.min_angle_rad,
                        self.pitch.max_angle_rad,
                    );
                }

                // Yaw position loop.
                self.yaw.target_rpm = self.yaw.position_pid.calculate(
                    self.yaw.target_angle_rad,
                    self.yaw.motor.angle_rad,
                    TICK_S,
                    RUNNING_MAX_RPM,
                );

                // Yaw speed loop.
                self.yaw.motor.command_voltage_raw = self.yaw.speed_pid.calculate(
                    self.yaw.target_rpm,
                    self.yaw.motor.speed_rpm,
                    TICK_S,
                    RUNNING_MAX_VOLTAGE,
                );

                // Pitch position loop.
                self.pitch.target_rpm = self.pitch.position_pid.calculate(
                    self.pitch.target_angle_rad,
                    self.pitch.motor.angle_rad,
                    TICK_S,
                    RUNNING_MAX_RPM,
                );

                // Pitch speed loop.
                self.pitch.motor.command_voltage_raw = self.pitch.speed_pid.calculate(
                    self.pitch.target_rpm,
                    self.pitch.motor.speed_rpm,
                    TICK_S,
                    RUNNING_MAX_VOLTAGE,
                );
            }
            State::Error => {
                self.disable_all();
            }
        }
    }

    /// Queue one group command frame on CAN1 without waiting. This runs every tick, so a
    /// cleared command reaches the motor within 1 ms. A full queue drops the frame and
    /// counts it in `TX_FAILED`.
    pub fn transmit(&mut self, can: &mut Can1) {
        let mut data: [u8; 8] = [0u8; 8];
        // TODO: clamp each voltage count output within the bench limit (protocol +/-25000),
        // encode signed i16 big-endian and write it into the
        // bytes for its motor ID as the manual describes. Other motor IDs stay zero.

        // Limit the voltage counts to the protocol range of +/-25000, then convert to i16.
        let yaw_voltage: i16 = self.yaw.motor.command_voltage_raw
            .clamp(-25000.0, 25000.0) as i16;

        let pitch_voltage: i16 = self.pitch.motor.command_voltage_raw
            .clamp(-25000.0, 25000.0) as i16;

        // Convert the voltage counts to big-endian byte arrays
        let yaw_bytes: [u8; 2] = yaw_voltage.to_be_bytes();
        let pitch_bytes: [u8; 2] = pitch_voltage.to_be_bytes();

        // Fill the data array with the voltage bytes for each motor ID
        let yaw_index: usize = ((YAW_MOTOR_ID - 1) * 2) as usize;
        let pitch_index: usize = ((PITCH_MOTOR_ID - 1) * 2) as usize;

        data[yaw_index..yaw_index + 2].copy_from_slice(&yaw_bytes);
        data[pitch_index..pitch_index + 2].copy_from_slice(&pitch_bytes);

        // Create a CAN frame with the command ID and data, and attempt to write it to the CAN bus.
        let frame: Frame = match Frame::new_standard(MOTOR_COMMAND_ID, &data) {
            Ok(frame) => frame,
            Err(_) => {
                TX_FAILED.fetch_add(1, Ordering::Relaxed);
                return;
            }
        };
        
        if can.writer().try_write(frame).is_err() {
            TX_FAILED.fetch_add(1, Ordering::Relaxed);
        }
    }
}

// Set up RTT and the board, then start the remote and control tasks.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_target::rtt_init_print!(rtt_target::ChannelMode::NoBlockTrim, 1024);
    let (can, watchdog, remote) = dm_mc02::init();
    rtt_target::rprintln!("Gimbal control started");
    spawner.spawn(dm_mc02::remote_task(remote).unwrap());
    spawner.spawn(control_task(can, watchdog).unwrap());
}

// Run `tick` at 1 kHz and feed the watchdog after each one. If a tick hangs, the watchdog
// resets the MCU after 100 ms.
#[embassy_executor::task]
async fn control_task(mut can: Can1, mut watchdog: dm_mc02::Watchdog) {
    let mut control = Control::default();
    let mut ticker = Ticker::every(TICK);
    let mut last = Instant::now();
    loop {
        ticker.next().await;
        // After a stall, such as the 20 ms halt when a debug probe attaches, start the
        // schedule again instead of running every missed tick back to back.
        let now = Instant::now();
        if now - last > TICK * 2 {
            ticker.reset();
        }
        last = now;
        control.tick(&mut can);
        watchdog.pet();
    }
}
