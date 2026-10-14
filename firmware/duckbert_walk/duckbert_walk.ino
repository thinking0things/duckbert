// Duckbert - open-loop walk on the real robot. ESP32-C3 SuperMini driving 6x SG90 directly.
//
// This runs the controller from models/none_noimu exactly as the simulation ran it: the same
// fourteen numbers, the same 50 Hz, the same 3 rad/s slew limit. There is no IMU on this
// board and the controller was searched with its balance gains pinned to zero, so nothing is
// missing - it walks off its own clock or not at all.
//
// Before it walks it has to be calibrated, because the sketch knows what angle each joint
// should be at and nothing about which way your servo turns or where its horn ended up.
// Type 'h' over serial for the calibration commands. Do that first, with the robot held off
// the ground, and save with 'w'.
//
// Wiring - no servo driver board, the C3's own LEDC peripheral makes all six pulses.
//   GPIO3, GPIO4, GPIO6 -> L hip roll, L hip pitch, L knee   (signal wires only)
//   GPIO0, GPIO1, GPIO2 -> R hip roll, R hip pitch, R knee
//
// L knee is on GPIO6 rather than the GPIO5 that would complete the run, because on this build
// the servo on GPIO5 would not move while the other five did. GPIO5 itself tested clean - it
// drove high and low, read back correctly, and carried a good 1450 us pulse - so the fault was
// downstream of the pin, and moving the joint was quicker than chasing it. 'P 2 5' moves it
// back if that ever gets sorted out.
//   Servo V+ -> its own 5 V supply, NOT the board's 3V3 pin
//   Servo GND -> that supply's ground, tied to the board's GND
//
// The servo supply and the board must share a ground, and the board must not try to feed the
// servos: six stalled SG90s pull several amps and will brown out the regulator mid-step,
// which looks exactly like a controller bug and is not one.
//
// Two things to know about driving servos off bare C3 pins. GPIO2 is a strapping pin, so if
// anything on that line holds it low at reset the chip will not boot - a servo signal input is
// high impedance and normally fine, but that is the first thing to unplug if the board stops
// coming up. And the C3 swings 3.3 V where an SG90 expects 5 V logic; they almost always
// trigger anyway, but a servo that twitches or ignores pulses wants a level shifter, not a
// different gait.

#include <Preferences.h>
#include <driver/gpio.h>
#include <Wire.h>
#include <esp_system.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <lwip/sockets.h>

// ---------------------------------------------------------------- output: USB and Wi-Fi
// Every line the sketch prints goes to USB and, when a client is connected over Wi-Fi, to it too.
// The network side must never make the control loop wait: NetworkClient::write retries a busy
// socket up to ten times with a one second select each, so a stalled link could freeze the gait and
// the balance loop for ten seconds. So output is collected here and handed to the socket once per
// pass of loop() with MSG_DONTWAIT; what the socket will not take now waits for the next pass. The
// first version sent every printf on its own - a state report is a dozen tiny segments - and lwIP's
// send queue refused most of them: 9 of 15 reports never arrived whole. Only a buffer that is still
// full after that is dropped - a lost status line costs nothing, a frozen robot costs a fall.
int net_fd = -1;
void exec_line(char *s);
char net_out[4096];
size_t net_out_len = 0;
class Tee : public Print {
 public:
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override {
    // Only hand USB what fits. Once a host has read the port, core 3.3.8's HWCDC::write waits for
    // room in 1 ms steps, and with a zero TX timeout its retry counter wraps instead of expiring: a
    // cable left plugged in with nobody reading froze loop() after ~4 KB, and with it Wi-Fi.
    if (Serial.availableForWrite() >= (int)n) Serial.write(b, n);
    if (net_fd >= 0) {
      const size_t k = min(n, sizeof(net_out) - net_out_len);
      memcpy(net_out + net_out_len, b, k);
      net_out_len += k;
    }
    return n;
  }
};
Tee out;
#include "gait_params.h"
#include "shift_params.h"
#include "standup_params.h"
#include <Adafruit_GFX.h>
#include <hal/gpio_ll.h>
#include <esp_rom_sys.h>
#include <FluxGarage_RoboEyes.h>
// RoboEyes defines one-letter macros for moods and compass positions; keep the values this sketch
// uses under its own names, then remove the macros before they can collide with anything else.
static const unsigned char EYES_DEFAULT = DEFAULT, EYES_TIRED = TIRED, EYES_HAPPY = HAPPY;
static const unsigned char EYES_POS_E = E, EYES_POS_W = W, EYES_POS_S = S;
#undef N
#undef NE
#undef E
#undef SE
#undef S
#undef SW
#undef W
#undef NW

static const uint16_t SERVO_HZ = 50;
// The C3's LEDC timers are 14 bits wide (SOC_LEDC_TIMER_BIT_WIDTH), and ledcAttach
// REFUSES anything wider - it returns false and every later ledcWrite goes nowhere, so
// the sketch runs, answers serial, and never makes a pulse. 14 bits at 50 Hz is a 1.2 us
// step, against an SG90 deadband of 5-10 us, so nothing is lost by staying inside it.
static const uint8_t  PWM_BITS = 14;              // LEDC duty resolution - C3 maximum
static const uint32_t CONTROL_HZ = 50;
static const uint32_t CONTROL_US = 1000000UL / CONTROL_HZ;

// SG90 pulse widths. These are the population average; the per-servo correction lives in the
// calibration below, which is why they are not worth arguing about.
static const uint16_t PULSE_MIN_US = 500;   // nominally -90 deg
static const uint16_t PULSE_MAX_US = 2400;  // nominally +90 deg

// Calibration poses with a landmark the eye can actually judge. The CAD zero has no straight
// line anywhere in it - the thigh rakes 25 deg back and the shin 25 deg forward - so matching it
// by eye is guesswork. In each of these poses one segment stands at exactly 90 deg to the trunk,
// which a set square held against the trunk checks directly.
//
// Both are measured against the TRUNK, not against gravity. A plumb line only agrees with the
// trunk while the trunk is level, and a robot held in a hand never is.
//
// The knee pose used to be +19.12 deg, where the centre of the sole box sits under the knee
// axis. That is not what a shin looks like when it is straight: the printed shin runs from the
// knee axis to the foot bolts, and that line is square to the trunk at +24.81. Tuning the knee
// until the shin looked straight at the old command left it 5.69 deg out, and in the simulator
// that is enough to put the robot on its back in the neutral stance. Both numbers come from the
// CAD stance constants in cad/microduck9.py: hip pitch (4, 100), knee (-13.8, 62), foot (7, 17).
static const float POSE_THIGH_SQUARE_DEG = -25.10f;   // hip pitch: thigh square to the trunk
static const float POSE_SHIN_SQUARE_DEG  = +24.81f;   // knee: shin square to the trunk

// Crouch: hip pitch +deg, knee -deg. With the two equal and opposite the sole stays flat, the trunk
// drops, and the centre of mass moves from the heel towards the middle of the foot - on the
// feet-in body at 25 deg the margin to the heel grows from 19.8 to 31.7 mm, which is the side this
// robot falls to. It is also where the robot is woken and parked, because it is the pose that
// changes least when the servos let go.
static const float CROUCH_DEG = 25.0f;
// 10 Oct: wake into a deep crouch, not the 25 deg one - low and steady while the servos come up one by one.
static const float WAKE_CROUCH_DEG = 80.0f;
// ...and leaning forward (11 Oct): hip that many degrees less than the crouch, so the trunk tips nose down
// (hip - is nose down, as with the lean L). Park ends in the same pose, so the robot is put down, not dropped.
static const float WAKE_LEAN_DEG = 10.0f;
void rest_pose(float out[6]) {
  crouch_pose(WAKE_CROUCH_DEG, out);
  out[1] -= WAKE_LEAN_DEG * PI / 180.0f; out[4] -= WAKE_LEAN_DEG * PI / 180.0f;
}
// Raised to 80 on request, with hip pitch and knee travel widened to +-80 in gait_params.h to match.
// In simulation on the feet-in body a crouch eased down to 80 held with the soles flat (0.2 deg),
// the trunk 66.5 mm up and nothing but the soles on the floor. The simulator does not collide the
// robot's own parts with each other, so a thigh meeting a shin is not something it would have shown.
static const float CROUCH_MAX_DEG = 80.0f;

// Head tilt. There is no neck on this body, so the face tilts with the whole trunk: both hip rolls
// turn TILT_ROLL_DEG one way and one leg crouches TILT_DIFF_DEG deeper than the other, which rolls
// the trunk while leaving both soles flat. Solved on the feet-in body from a 25 deg crouch: trunk
// roll 8.4 deg, soles within 0.2 deg of flat. It holds with the centre of mass 4 mm back and with
// both hip rolls 3 deg out of calibration (then 6.5 / 10.3 deg), and it only tips over at twice
// the crouch difference. Raised from 8 / 4.5 on request to 10.5 / 5.25: trunk 10.5 deg, soles still
// within 0.1 deg of flat, and 10.8 / 8.6 deg with the centre of mass 4 mm back and both rolls 3 deg
// out. 11 is the hip roll's travel, so this is as far as the face can tilt on this body.
static const float TILT_ROLL_DEG = 10.5f;
static const float TILT_DIFF_DEG = 5.25f;

// Waking up. An SG90 cannot report where its horn is, so the first pulse it gets sends it to the
// commanded angle at full speed - there is no easing in without knowing where it starts. Two things
// soften that. The joints come on one at a time, WAKE_STAGGER_S apart, so there is only ever one
// moving and one drawing current. And each joint starts on one pulse in WAKE_SPARSE and ramps to
// every pulse over WAKE_RAMP_S: an analog servo only drives its motor while pulses arrive, so a
// sparse train moves it in short bursts. A digital servo holds between pulses and will simply
// snap - which is still one joint at a time.
static const float WAKE_STAGGER_S = 0.40f;
static const float WAKE_RAMP_S = 1.20f;
static const int   WAKE_SPARSE = 6;
// Left and right in pairs, joint type by joint type - rolls, then hips, then knees - so the robot is
// never holding one leg stiff while the other still hangs, which would tip it sitting on the floor.
static const int   WAKE_ORDER[6] = {0, 3, 1, 4, 2, 5};

static const char *JOINT_NAME[6] = {
    "L_hip_roll", "L_hip_pitch", "L_knee", "R_hip_roll", "R_hip_pitch", "R_knee"};

struct Calibration {
  uint8_t pin;       // C3 GPIO carrying this joint's pulse
  int8_t  sign;      // +1 if the servo turns the way the model expects, -1 if mirrored
  float   trim_rad;  // where this joint actually sits when the model says zero
};

// The signs are the ones cad/out_none/manifest.json implies: sign = axis . shaft_axis, which
// is +1 for both hip rolls (those two servos are placed in the same orientation, not mirrored)
// and opposite side to side for hip pitch and knee (w = (0,-s,0) in microduck9.py). Whether the
// whole column wants flipping depends on which way your SG90s turn for a rising pulse, which no
// table can know - so confirm every one of them with 'j' and 'k' before letting the robot take
// weight, and 'i' any that are backwards.
Calibration cal[6] = {
    {3, +1, 0.0f},  // L_hip_roll
    {4, -1, 0.0f},  // L_hip_pitch
    {6, -1, 0.0f},  // L_knee  (GPIO5 skipped - see the wiring note at the top)
    {0, +1, 0.0f},  // R_hip_roll
    {1, +1, 0.0f},  // R_hip_pitch
    {2, +1, 0.0f},  // R_knee
};

Preferences prefs;
bool pwm_ok[6] = {false};   // did ledcAttach take? no pulses at all if not

enum Mode { HOLD, WALK, LIMP, STAND, WAKE, ROCK, STEP };
// ROCK shifts the weight from one foot to the other and does nothing else: the hip rolls lean the
// trunk sideways while one leg is made shorter than the other, at an amplitude and a period that
// are asked for. No stepping, so a foot leaving the floor can be watched, timed and measured -
// which is the one thing the bench has never measured, and the reason the walk rocks 14 deg while
// the gait asks for 7.
// Power-up is LIMP: no pulses at all until someone asks. Starting in HOLD used to drive every
// servo to zero on the first frame, at full speed, from wherever the horns had been left.
Mode mode = LIMP;

float command_rad[6] = {0, 0, 0, 0, 0, 0};   // where the servos are being told to be
float gait_clock = 0.0f;
float rock_deg = 0.0f, rock_period = 1.2f, rock_clock = 0.0f, rock_diff = 0.5f;
int gait = 0;                                // which of GAITS[] 'g' runs; saved with 'w'
float amp = 0.70f;                            // step size: scales the gait's swings, not its stance
float sway = 1.0f;                           // hip roll rock only, on top of amp
// Trunk lean while walking, deg, added to both hip pitches: + leans the trunk back. Every feet-in gait
// walks about 12 deg nose down - its knee mid-point sits near -14 deg - and on the bench that tipped
// the robot over forwards. In simulation +1 deg of hip moves the trunk back about 1 deg. The same
// simulation had robust falling backwards at +5, so this is set on the bench, a little at a time.
float walk_lean = 0.0f;
// Steering. A gait that is symmetric on paper turns on a real robot: one servo sits a degree off,
// one sole grips better. This scales the hip swing of one leg up and the other down, so one foot
// takes the longer step - positive turns the robot towards its own right, which is what a drift to
// the left needs. It rides on top of every gait and is saved with the rest.
float walk_turn = 0.0f;                      // -1 .. 1, asymmetry of the hip swing (-0.5 .. 0.5 until 8 Oct)
// 'Z <pct>': how much of the sequence walks' stride goes forward. 100 walks; 0 leaves only the steering,
// so with W the left foot steps forward and the right one back (W > 0) - a turn on the spot. Not saved.
float walk_fwd = 1.0f;
// one step at a time ('m', 'o'); see step_control()
// models/step_b (train_step.py on plant_fit_v13): 34/36 upright over five steps, foot 7-13 mm up
float stp[12] = {0.164f, -0.085f, 0.239f, 0.331f, 0.116f, 0.112f, 0.803f, 0.390f, 0.150f, 1.156f, 0.050f, 1.0f};
int step_front = 0;            // leg in front at rest: 0 left, 1 right
bool step_moving = false;      // a step is under way
float step_clock = 0.0f;
int step_count = 0;
// 11 Oct evening: lifting the feet on the spot with stagger 0 fell every time - feet side by side leave no
// base fore and aft, and the 'o' sent mid-step snapped the feet together at once. In place now: the stagger
// stays, the foot the rock has unloaded goes up and comes down where it was, the other one next time.
bool step_inplace = false;
// 12 Oct 23:20: it already creeps forward just rocking - do a little more, just move the free foot a bit
// forward while it leans. No discrete steps, no lift: the rock unloads a foot, and while it is unloaded it slides
// forward and the loaded one back (the hip swinging over the planted foot is what moves the body on). The feet
// follow the roll: x = +-stride * roll / wanted rock, stride = the stagger number of 'o'; feet parallel at rest.
bool step_shuffle = false;
float step_shuffle_x = 0.0f;
// 13 Oct: good, but it hardly lifts the feet. The free foot now also goes up a little, most at the end
// of the lean (squared, so none near the middle): a quarter of the step's lift (0.625 rad -> ~9 deg; 0.4 fell).
float step_shuffle_lift[2] = {0.0f, 0.0f};
// 13 Oct, the final page (the drive page): it goes while the arrow is held and stops when let go. 'f <0..100>' sets how
// much of the slide's stride and lift to use; the robot moves to it at 0.8 per second (a stop or a start takes
// ~1.2 s), so a press or a release never jumps the feet. 100 = as before (the the step settings page never sends it).
float step_go = 1.0f, step_go_now = 1.0f;
float step_lean_now = 0.0f;    // the steering lean actually applied, deg (rate-limited, see step_control)
int step_lifting = 0;          // in place: the leg in the air (0 left, 1 right)
int step_last_lift = -1;
bool step_skip_shift = false;  // a step fired by the rock: shift and sink ease in during the swing (no jump)
// 12 Oct: the 2 s averages behind the pitch and roll trims lived in function statics and were never reset, so a
// run started with the average of the previous one - after a fall, ~-20 deg of pitch. The trim then bent both
// hips 8 deg forward in the first 2 s with the robot standing still (walk_20261012_162553: hips -21 -> -30,
// pitch 9 -> 19.5, then it fell). Each run now starts clean.
float step_pitch_center = 0.0f, step_rock_center = 0.0f, step_last_hi = 0.0f, step_last_lo = 0.0f;
// Stepping on the rock (8 Oct). In the staggered stance, pushed once, the robot keeps
// rocking from foot to foot on its own gyro feedback. 'v thr amp mode':
//   mode 0: 'm' steps at once (as before)
//   mode 1: 'm' only arms the step; it goes when the trunk has rolled thr deg towards the front foot and is
//           still moving that way - the weight is on it - and never if that does not happen
//   mode 2: the same, every time, foot after foot: walking on the rock
// amp: the rock the feedback aims at: the roll gain rises while the rock is smaller and falls while it is
// bigger (0.3x .. 2.5x), so the rock keeps going at that size instead of dying out or growing into a fall.
float rock_thr_deg = 4.0f, rock_amp_deg = 6.0f, rock_lead_s = 0.12f;
// 9 Oct: the angle feedback alone, even at 2.5x, let the rock die after each push. The rock is now fed the
// way a swing is pushed - in time with its speed: both hip rolls get -pump * (roll rate / 15 deg/s, capped at 1),
// which leans the trunk further the way it is already going. pump (rad) grows while the rock is smaller
// than wanted and shrinks, down to negative - braking - while it is bigger. Fore-aft is the opposite:
// rock_pdamp (rad per rad/s) is a plain damping on the pitch rate.
float rock_pump = 0.06f, rock_pdamp = 0.0f;
// 9 Oct: the feet stepped on each other. Both legs stand this many degrees wider in step mode (outwards).
float step_splay_deg = 4.0f;
// 11 Oct, after recalibrating the right leg a spline tooth: legs nearly straight (knees ~0) and over backwards in
// 1.5 s. crouch more. Hip +c, knee -c (soles stay level), in step mode only.
float step_crouch_deg = 15.0f;
// 11 Oct 14:13: the rock itself drives the fore-aft motion - with the feet staggered, every swing onto the
// front foot pitches the trunk forward and every swing back pitches it back, +-5 deg at up to 170 deg/s - and
// on top of that the mean pitch crept backwards (-5, -9, -16 deg over 5 s) until it went over. The window for
// L was only -20..-30. So the lean finds itself: the 2 s average of the pitch is integrated into the hips the
// way the roll centre is (hip - is nose down), up to 15 deg either way, on top of L.
float pitch_trim_deg = 0.0f;
// 11 Oct: a good rock that does not fall, not a lot of walking. Every version so far drove the rock
// with positive feedback on the roll ANGLE (1.45 rad/rad); it grows by itself in a second, the hip rolls slam
// at +-16 for 30-60% of the time and the limiters arrive late. 'Pump' style instead: no angle feedback at all,
// and the hip rolls swing in time with the roll RATE, like pushing a swing - u = -A * clamp(rate / 15 deg/s) -
// with A (deg) raised while the rock is smaller than wanted and lowered, down to braking, while it is bigger.
// The rocker shoes set the frequency; this only feeds or drains energy, so it cannot run away.
int rock_style = 1;            // 0 = the angle feedback of 10 Oct, 1 = pump
// 11 Oct evening: "upright" in step mode was the attitude at the moment 'm' was pressed, so it changed with how
// the robot happened to be standing (or held). The 76 s run started at pitch +3.3 deg; the four that fell started
// at +9.5..+14 - and the pitch correction then held the trunk there. A fixed pitch reference instead (deg,
// absolute, + nose down); 99 = the old way, the attitude at entry.
float step_pitch_ref_deg = 3.5f;
float rock_hip_amp = 4.0f;     // deg, the pump's swing
// 13 Oct: not a higher foot - a bigger rock, so the weight comes off the foot more. Every rock since
// 11 Oct spans ~20 deg (p5..p95) with the hip-roll pump at its limit; only the angle feedback of 10 Oct made
// 42 deg (walk_20261010_105234) - and then grew past 30 and fell. So a share of that angle feedback on top of
// the pump ('v' 11th number, 0..1; 0 = off), cut to wanted/actual when the rock is bigger than wanted (as the
// 10 Oct limiter did) and gone while a foot is up. (Briefly the 11th number was a leg-length pump; dropped.)
float rock_angle_share = 0.0f;
float rock_trim_deg = 0.0f;    // hip roll that keeps the rock centred (see step_control)
// Heading in step mode (10 Oct): the rock alone turned it 37 deg left in 13 s, no step taken - each time a
// foot unloads it slips. That slip is the lever: the left foot a little forward of the right (rad, as hip
// pitch) turns the trunk left while both are down, and the unloaded foot then slides back, so the turn
// ratchets on. step_turn is that offset; with the heading hold on ('H 1') it steers to yaw_target, and 'D'
// moves the target ('D -30' thirty degrees to the left), so turning is just a new heading.
float step_turn = 0.0f;
int rock_mode = 0;
bool step_armed = false;
float rock_peak = 0.0f;        // deg, decaying peak of |roll| - the size of the rock
float rock_gain = 1.0f;        // what the roll feedback is multiplied by right now
// Toe clearance. The knee is the only joint that lifts a foot - there is no ankle - so this scales
// the knee's swing on its own, leaving the hips and the stance alone. Above 1 the swinging foot
// folds further out of the way; it also digs the stance leg in deeper, so it is not free.
float knee_scale = 1.0f;                     // 0.5 .. 2.0
// Heading hold. The steering trim above is a fixed guess; this is the same knob driven by the gyro,
// which is the one thing on the robot that can tell it is curling away. Target heading is taken
// when the walk starts, and 'D <deg>' moves it - so a turn on the spot is the same controller with
// a target that is not straight ahead. Off by default: it can only help a walk that stays up.
// A gait searched around a gyroscope keeps its own feedback here, applied inside the pattern the
// way the search applied it: pitch into both hips, roll into both hip rolls, with no rate limit of
// its own because the whole target goes through the joint slew afterwards. This is separate from
// the 'B' loop, which is a correction bolted on top of any gait; turn it off with 'I 0' to hear
// what the gait does bare, which is also the only honest way to compare the two.
int gait_imu = 1;
// What the bench showed on the rocker shoes: at 5.7 steps a second the gyro reads the impacts, 127
// deg/s rms of pitch rate, and the gait's rate gain sent 8.5 deg rms of that straight back into the
// hips - the robot vibrated instead of walking. In simulation those rates are smooth. So the rates
// this feedback uses are low-passed at ~4 Hz, above the 2.8 Hz stride and below the hammering; the
// angles are measured from the attitude the walk set off with, as the 'B' loop learned to do; and
// the whole thing can be dosed, because a gain searched on a perfect servo is not the right gain
// for an SG90 with play in its gears.
// 'J kp kd': the sequence walks' pitch feedback with gains of its own, replacing the searched p[13]/p[16]
// (which F scales). On 8 Oct the bench showed the searched sign on G10 drives the backward fall: as the
// trunk tips back it raises the hips, which tips it further. 'J' alone goes back to the searched gains.
bool pitch_own = false;
float pitch_kp = 0.0f, pitch_kd = 0.0f;
float gait_fb_scale = 1.0f;                  // 'F <pct>': share of the searched gains, 0 .. 3 (1.5 until 8 Oct)
float g_rate_roll = 0, g_rate_pitch = 0;     // the rates the gait feedback sees, filtered
float g_ref_roll = 0, g_ref_pitch = 0;       // attitude at the start of this walk
static const float G_RATE_TAU = 0.04f;       // s, about 4 Hz
int yaw_hold = 0;
float yaw_target = 0.0f, yaw_kp = 0.6f, yaw_kd = 0.0f, turn_corr = 0.0f;
// What the bench showed on 6 October: every walk with the hold on fell, backwards, while the same walk
// without it stood. The steering it asked for swung between -100% and +62% - the trunk twists with
// every step and the old gains answered each twist - where a fixed -10% had already been enough to
// walk straight. So the hold now steers on the heading averaged over about a second, with half the
// gain and no rate term, and never asks for more than 25% unless it has been told to turn.
float yaw_f = 0.0f;                          // heading, low-passed
static const float YAW_TAU = 1.0f;           // s: longer than a stride, shorter than a drift
static const float YAW_CAP = 0.25f;          // most steering the hold may use to keep a line
static const float YAW_CAP_TURN = 0.6f;      // when 'D' has asked for a real turn
// The bigger cap belongs only to a turn that was asked for. The first version switched to it whenever
// the heading was 20 deg off, and on 6 October that turned a drift into a spin: the hold steered the
// wrong way, the error grew past 20 deg, and it got 60% to keep doing it.
bool yaw_turning = false;

// ---------------------------------------------------------------- IMU balance feedback
// The same law as gait9.command: hip rolls get -kPr*roll - kDr*roll_rate, hip pitches get
// -kPp*pitch - kDp*pitch_rate, on top of whatever the gait or the pose commands. Angles in the
// simulator's convention - roll + leans to the robot's right, pitch + is nose down - so gains
// found there carry over. In simulation, with IMU readings a tick late and the corrections clamped
// as below, roll gains 0.30 / 0.03 took mid at 70% through 4 of 6 sets of lateral kicks against 0
// without; 60 ms of extra delay erased that, and the pitch terms did not help at all. It starts off.
// fb_mode: 0 off, 1 while walking, 2 while walking and while holding a pose (the bench sign test).
int   fb_mode = 0;
const char *reset_reason = "?";

// Wi-Fi. Credentials live in flash, set once over USB with "N ssid|password" - never in this file,
// which is published. Joining is non-blocking: the robot works the same with no network at all.
WiFiServer net_server(2323);
WiFiClient net_client;
String wifi_ssid;
bool wifi_configured = false, wifi_up = false;
float wifi_txp = 15.0f;                      // dBm, stored; see wifi_start
char net_buf[128];
uint8_t net_len = 0;
float fb_kpr = 0.30f, fb_kdr = 0.03f, fb_kpp = 0.0f, fb_kdp = 0.0f;
// Two things the first version of the walking balance got wrong, both visible in the bench logs.
// It drove the trunk towards zero, but the trunk's own resting attitude in the crouch is 8 deg nose
// down, so it pushed all the time; and it answered the stride's own 14 deg rock, at a gain that
// saturated the rolls within one step and fed the rock instead of damping it - 51 deg of rock and
// a dive forward in 2 s, against 8 to 32 s of walking with the loop switched off. So: the error is
// measured from the attitude the robot had when it set off, and it is low-passed, which leaves the
// loop the slow drift into a fall and leaves the step pattern alone. fb_tau 0 restores the old law.
float fb_tau = 1.2f;                         // s: how slow a lean has to be before the loop answers
float fb_ref_roll = 0, fb_ref_pitch = 0;     // the attitude this walk started from
float fb_err_roll = 0, fb_err_pitch = 0, fb_rate_roll = 0, fb_rate_pitch = 0;
bool fb_ref_init = false;
float corr_roll = 0.0f, corr_pitch = 0.0f;   // rad, what is being added right now

// ---------------------------------------------------------------- walk recorder
// One line a tick while it runs: what the gait asked of each joint, what the robot ended up doing
// with it, and how hard the balance loop was pulling. Whole numbers in tenths of a degree, ~110
// bytes a line - 5.5 KB/s at 50 Hz, which the Wi-Fi link carries and the USB side drops unread.
bool log_on = false;
uint32_t log_t0 = 0, log_n = 0;
float corr_diff = 0.0f;
enum PoseKind { POSE_OTHER, POSE_CROUCH, POSE_TILT_L, POSE_TILT_R };
PoseKind pose_kind = POSE_CROUCH;            // what the robot is holding, for the face
float pose_crouch_deg = 25.0f;                       // rad: left leg crouches this much deeper, right this much less

// Push reaction, standing still. Answers a push on the trunk by leaning back against it with the hip
// rolls and the leg-length difference the head tilt uses, around a reference that follows the
// robot's attitude over about 4 s - so it answers a quick push and leaves a chosen pose alone.
// It looks alive; it does not make the robot stand harder pushes. In simulation, crouched at 25 deg,
// the largest push it survived was 1.36 N sideways and 0.84 N fore-aft for 0.15 s with or without
// any reaction - hip rolls, leg-length lever, hip pitch or hip and knee together, at 3 rad/s or 6:
// a push that tips it past the edge of a foot is decided before a servo can move, and one that
// does not, it rides out alone. Standing up to more would need a step. These gains were neutral
// there; the "obvious" pitch sign made it fall from 0 N, so pitch starts at 0.
bool  push_on = false;
float push_kpr = 1.0f, push_kdr = 0.03f, push_kpp = 0.0f, push_kdp = 0.0f;
static const float PUSH_DIFF = 0.5f;          // leg-length difference per unit of roll correction
static const float PUSH_CLAMP_DIFF = 10.0f * PI / 180.0f;
float push_ref_roll = 0.0f, push_ref_pitch = 0.0f;
bool  push_ref_init = false;
// Roll correction may use the hip rolls' whole travel; write_joint still stops at +-11. The first
// clamp of 6 deg was narrower than the room there is, and there is little: mid at step 70% with the
// hip sway at 100% already puts a hip roll 0.4 deg past its travel before any correction, which is
// why cutting the sway made balancing work better on the bench. In simulation, with the sway at
// 60%, roll gains 0.5 / 0.03 under this clamp took 5 of 8 sets of kicks against 3 of 8 under the old.
static const float FB_CLAMP_ROLL = 11.0f * PI / 180.0f, FB_CLAMP_PITCH = 12.0f * PI / 180.0f;
// Fall limits per direction (8 Oct). Backwards 40: with the R80 shoes and the remounted IMU the robot comes to
// rest on its back at 40-53 deg, so at 50 a backward fall often never counted. Forwards 60: the hand-tuned G10
// walks with the trunk 15 deg forward and pitches through 40 while still walking. Sideways stays at 50.
static const float FALL_RAD = 50.0f * PI / 180.0f;
static const float FALL_BACK_RAD = 40.0f * PI / 180.0f, FALL_FWD_RAD = 60.0f * PI / 180.0f;
float att_roll = 0.0f, att_pitch = 0.0f, rate_roll = 0.0f, rate_pitch = 0.0f;
float acc_roll = 0.0f, acc_pitch = 0.0f;     // the same angles from gravity alone, unfiltered
// Heading has no gravity to lean on, so it is the gyro integrated and nothing else: good for the
// tens of seconds a walk lasts, which is all that is asked of it. It says how fast the robot turns
// while it walks - on the bench it drifts to its own left, and nothing in the log measured that.
float rate_yaw = 0.0f, yaw_rad = 0.0f;
bool att_valid = false;
float stand_clock = 0.0f;
float stand_way[5][6];                       // waypoint 0 is wherever the servos were
float stand_edge[5];
float stand_scale = 1.0f;                    // 1 = the searched move; <1 = slowed dry run

const char *mode_name() {
  return mode == WALK ? "WALK" : mode == HOLD ? "HOLD" : mode == LIMP ? "LIMP" :
         mode == WAKE ? "WAKE" : mode == ROCK ? "ROCK" : mode == STEP ? "STEP" : "STAND";
}
float speed = 0.60f;                          // stride rate multiplier, for a first cautious try
int selected = 0;                            // joint the calibration keys act on
uint32_t last_control = 0;

// ---------------------------------------------------------------- the gait
// The open-loop half of gait9.command(). Both legs share the joint sign convention, so the
// walking antiphase is carried by the pattern rather than by mirrored joint frames - which is
// why the mirroring has to happen in the calibration table instead.
// Gaits are numbered in one run: the sums of sines first, then the sequence walks. Everything that
// needs a period, a slew or a name asks these instead of indexing GAITS directly.
static inline bool is_shift() { return gait >= N_GAITS; }
static inline const ShiftSpec &shift_spec() { return SHIFTS[gait - N_GAITS]; }
float gait_period() { return is_shift() ? shift_spec().p[0] : GAITS[gait].p[0]; }
float gait_slew() { return is_shift() ? shift_spec().slew : GAITS[gait].slew; }
const char *gait_name() { return is_shift() ? shift_spec().name : GAITS[gait].name; }

// Hand-tuning knobs for the sequence walks, from the bench (6 October): on G8 the weight never left
// the foot, and G8 has no leg-length rock at all and nearly straight legs. These add what the rocking
// simulation said is the lever - one leg shorter while the weight goes the other way - plus room for
// it, and the lift that was wanted: higher, quicker, with the hip opening as the foot comes up.
float sh_crouch = 0.0f;     // deg added to the hip and taken off the knee: bent legs, room to shorten one
float sh_rock = 0.0f;       // deg of leg-length rock in time with the sway, signed
float sh_sharp = 1.0f;      // 1 = the searched lift; higher rises and drops faster and holds up longer
float sh_abd = 0.0f;        // deg the swinging hip opens at the top of its lift, + outwards
// Sideways trim of the trunk: both hip rolls by the same amount, on every gait. On the rocker shoes
// the trunk sat 2-2.5 deg to the right in every run of 6 October and the robot slid that way; this
// is the knob to put it back over the middle. + moves the trunk to the robot's left - measured, not
// assumed: the first version had it backwards, and +2.5 left the trunk at +3.3 deg where -2.5 brought
// it to +0.9 and gave the best run of the day (21 s, 15 cm).
float walk_side = 0.0f;

static float bump01(float u) {               // 0 at both ends, 1 in the middle, flat at the ends
  const float v = sinf(PI * constrain(u, 0.0f, 1.0f));
  return v * v;
}

// shift_gait.command, line for line, plus the same knobs the sine gaits take: step size scales the
// stride and the lift, steering lengthens one leg's stride, lean tilts both hips, and the gyro terms
// get the same filtering, reference and dose as the sine gaits' feedback.
void shift_targets(float t, float out[6]) {
  const float *p = shift_spec().p;
  const float T = p[0], ds = p[1];
  float phase = fmodf(t / T, 1.0f); if (phase < 0) phase += 1.0f;
  const float sv = sinf(2.0f * PI * (phase + p[5]));
  const float sw = sway * p[4] * copysignf(powf(fabsf(sv), p[6]), sv);   // hip sway scales it
  const float turn = constrain(walk_turn + turn_corr, -1.0f, 1.0f);
  const float lean_rad = walk_lean * PI / 180.0f;
  float lifted[2] = {0.0f, 0.0f};
  for (int leg = 0; leg < 2; ++leg) {
    float u = fmodf(phase - (leg ? 0.5f : 0.0f) + 1.0f, 1.0f) * 2.0f;
    const float start = ds / 2.0f, end = 1.0f - ds / 2.0f;
    float along, lift;
    if (u < 1.0f) {
      const float w = constrain((u - start) / max(1e-6f, end - start), 0.0f, 1.0f);
      along = -cosf(PI * w);
      lift = (u >= start && u <= end) ? bump01(w) : 0.0f;
    } else {
      along = 1.0f - 2.0f * (u - 1.0f);
      lift = 0.0f;
    }
    if (sh_sharp > 1.0f) lift = powf(lift, 1.0f / sh_sharp);   // a plateau: up fast, hold, down fast
    lifted[leg] = lift;
    const float rock = (p[14] + sh_rock * PI / 180.0f) * sv * (leg == 0 ? 1.0f : -1.0f);
    const float stride = amp * p[7] * (walk_fwd + (leg == 0 ? turn : -turn));
    const float up = amp * knee_scale * lift;    // 'E' raises the foot on these walks
    const float c = sh_crouch * PI / 180.0f;
    const float hip = p[2] + c - stride * along + p[9] * up + rock + lean_rad;
    const float knee = p[3] - c + (stride + p[8]) * along - (p[9] + p[10]) * up - rock;
    out[3 * leg + 1] = hip;
    out[3 * leg + 2] = knee;
  }
  // +hip_roll moves both feet to the robot's left: outwards is + for the left leg, - for the right
  const float abd = sh_abd * PI / 180.0f;
  const float side = walk_side * PI / 180.0f;
  out[0] = sw + p[11] + abd * lifted[0] + side;
  out[3] = sw - p[11] - abd * lifted[1] + side;
  // The leg carrying the weight opens outwards while the other foot is up (shift_gait p[17], p[18]):
  // a fixed amount plus a share of how far the trunk rolls towards the lifted side. With the foot
  // planted that tips the pelvis back over it and raises the swing side. It only ever opens.
  if (p[17] > 0.0f || p[18] > 0.0f) {
    const float r = (gait_imu && att_valid) ? gait_fb_scale * p[18] * (att_roll - g_ref_roll) : 0.0f;
    out[0] += lifted[1] * max(0.0f, p[17] + r);
    out[3] -= lifted[0] * max(0.0f, p[17] - r);
  }
  if (gait_imu && att_valid) {
    const float f = gait_fb_scale;
    const float br = f * (p[12] * (att_roll - g_ref_roll) + p[15] * g_rate_roll);
    const float bp = pitch_own ? pitch_kp * (att_pitch - g_ref_pitch) + pitch_kd * g_rate_pitch
                               : f * (p[13] * (att_pitch - g_ref_pitch) + p[16] * g_rate_pitch);
    out[0] -= br; out[3] -= br;
    out[1] -= bp; out[4] -= bp;
  }
}

void gait_targets(float t, float out[6]) {
  if (is_shift()) { shift_targets(t, out); return; }
  const float *GAIT = GAITS[gait].p;
  const float ph = 2.0f * PI * t / GAIT[0];
  const float s = sinf(ph), c = cosf(ph);
  const float s2 = sinf(2.0f * ph), c2 = cosf(2.0f * ph);

  // amp scales every oscillating term - both harmonics of hip and knee, and the lateral rock -
  // while the mid-points and the hip splay stay where the search put them. Smaller steps, same
  // stance. In simulation mid at 0.7 still walks with the trunk's centre of mass 8 mm behind the
  // model, where at full size it falls; slowing it down instead makes it fall almost at once.
  const float swing_h = amp * (GAIT[3] * s + GAIT[4] * c);
  const float even_h  = amp * (GAIT[5] * s2 + GAIT[6] * c2);
  const float swing_k = amp * knee_scale * (GAIT[7] * s + GAIT[8] * c);
  const float even_k  = amp * knee_scale * (GAIT[9] * s2 + GAIT[10] * c2);
  // sway scales only the sideways rock of the hip rolls. In simulation it does not calm the trunk:
  // mid rocks 12-13 deg whatever it is set to, because the rock comes from lifting a foot on a
  // 4 mm stance, not from the rolls - and below 60% it falls sooner when slowed. Kept because it
  // is a fair thing to try on the bench, not because it is expected to help.
  const float lean    = amp * sway * (GAIT[11] * s + GAIT[12] * c);

  // The gait's own gyro feedback, in the order the search used: pitch P, pitch D, roll P, roll D.
  // Model and board agree on both signs - + pitch is nose down and + roll is leaning right in each,
  // measured on the model rather than assumed.
  float bal_p = 0.0f, bal_r = 0.0f;
  const float *K = GAITS[gait].imu;
  if (gait_imu && att_valid && (K[0] || K[1] || K[2] || K[3])) {
    const float f = gait_fb_scale;
    bal_p = -f * (K[0] * (att_pitch - g_ref_pitch) + K[1] * g_rate_pitch);
    bal_r = -f * (K[2] * (att_roll - g_ref_roll) + K[3] * g_rate_roll);
  }
  out[0] = lean + GAIT[13] + bal_r + walk_side * PI / 180.0f;   // L_hip_roll
  const float lean_rad = walk_lean * PI / 180.0f;
  // At 1 the swinging leg takes the whole step and the other takes none, which is as far as an
  // asymmetry can go before it stops being a gait.
  const float turn = constrain(walk_turn + turn_corr, -1.0f, 1.0f);
  out[1] = GAIT[1] + swing_h * (1 + turn) + even_h + lean_rad + bal_p; // L_hip_pitch
  out[2] = GAIT[2] + swing_k + even_k;            // L_knee
  out[3] = lean - GAIT[13] + bal_r + walk_side * PI / 180.0f;   // R_hip_roll
  out[4] = GAIT[1] - swing_h * (1 - turn) + even_h + lean_rad + bal_p; // R_hip_pitch
  out[5] = GAIT[2] - swing_k + even_k;            // R_knee
}

// ---------------------------------------------------------------- servo output
static const float ROLL_INWARD_MAX_RAD = 12.0f * PI / 180.0f;
void write_joint(int j, float rad) {
  if (mode == LIMP) return;                  // limp means no pulses; the pose is still tracked
  float clamped = constrain(rad, JOINT_MIN_RAD[j], JOINT_MAX_RAD[j]);
  // Past 14.5 deg inwards the hip bracket rubs the hip block (CAD sweep, every block version), so each
  // hip roll stops 12 deg short inwards while keeping its full travel outwards. + roll moves a foot to
  // the robot's left: inwards is - on the left hip, + on the right.
  if (j == 0) clamped = max(clamped, -ROLL_INWARD_MAX_RAD);
  if (j == 3) clamped = min(clamped, ROLL_INWARD_MAX_RAD);
  const float shaft = cal[j].sign * clamped + cal[j].trim_rad;
  const float frac = constrain((shaft + PI / 2) / PI, 0.0f, 1.0f);   // -90..+90 deg -> 0..1
  const float us = PULSE_MIN_US + frac * (PULSE_MAX_US - PULSE_MIN_US);
  // Duty counts for a pulse of `us` out of the 20 ms frame.
  ledcWrite(cal[j].pin, (uint32_t)(us * (1UL << PWM_BITS) * SERVO_HZ / 1000000.0f));
}

// Corrections ride on top of the pose here, at the output, so nothing that writes command_rad -
// the gait, a jog, a crouch - has to know about them.
float joint_out(int j) {
  switch (j) {
    case 0: case 3: return command_rad[j] + corr_roll;
    case 1: return command_rad[j] + corr_pitch + corr_diff;
    case 4: return command_rad[j] + corr_pitch - corr_diff;
    case 2: return command_rad[j] - corr_diff;
    case 5: return command_rad[j] + corr_diff;
  }
  return command_rad[j];
}
void write_all() {
  for (int j = 0; j < 6; ++j) write_joint(j, joint_out(j));
}

// Move to a pose over a given time instead of jumping to it. An SG90 handed a large step
// snaps to it at full speed, which on a standing robot is enough to throw it over.
extern bool imu_ok;
bool imu_read();
void imu_update(float dt);
void ease_to(const float target[6], uint32_t millis_total) {
  pose_kind = POSE_OTHER;                    // the caller names the pose it was, after the move
  float from[6];
  for (int j = 0; j < 6; ++j) from[j] = command_rad[j];
  const uint32_t started = millis();
  // 12 Oct: the IMU used to stop here. Into step mode the trunk turns ~20 deg in these 900 ms (crouch to
  // straight legs) and the gyro never saw it: the attitude stayed at the crouch's, crept to the truth over
  // ~2 s through the accelerometer - and step mode took the stale value as "upright", then fought an error
  // that was not there (walk_20261012_162553: pitch "rose" 8.6 -> 19.5 standing still, fell back). The
  // attitude now keeps following during the move.
  uint32_t imu_t = micros();
  for (;;) {
    const uint32_t elapsed = millis() - started;
    const float a = elapsed >= millis_total ? 1.0f : (float)elapsed / millis_total;
    for (int j = 0; j < 6; ++j) command_rad[j] = from[j] + a * (target[j] - from[j]);
    write_all();
    if (imu_ok && imu_read()) {
      const uint32_t t = micros();
      imu_update(constrain((t - imu_t) * 1e-6f, 0.001f, 0.05f));
      imu_t = t;
    }
    if (a >= 1.0f) break;
    delay(1000 / CONTROL_HZ);
  }
  // This blocked for a second or more. Without this the control loop sees a backlog of missed
  // 50 Hz ticks and burns through all of them at once the moment the gait starts, which spends
  // the whole slew limit in a few milliseconds and throws the robot.
  last_control = micros();
}

void crouch_pose(float deg, float out[6]) {
  const float r = constrain(deg, 0.0f, CROUCH_MAX_DEG) * PI / 180.0f;
  const float pose[6] = {0, r, -r, 0, r, -r};
  for (int j = 0; j < 6; ++j) out[j] = pose[j];
}

// side +1 tilts the face to the robot's own left, -1 to its right, 0 is the plain crouch.
void tilt_pose(int side, float out[6]) {
  const float c = CROUCH_DEG * PI / 180.0f;
  const float r = side * TILT_ROLL_DEG * PI / 180.0f;
  const float d = side * TILT_DIFF_DEG * PI / 180.0f;
  const float pose[6] = {r, c + d, -(c + d), r, c - d, -(c - d)};
  for (int j = 0; j < 6; ++j) out[j] = pose[j];
}

// ---------------------------------------------------------------- IMU (GY-521 / MPU6050)
// On the I2C pins the old AlbertMini sketch drove its OLED from, on this same board. Both are C3
// strapping pins that must be high at reset, which the GY-521's pull-ups guarantee. Nothing here
// is used for control yet: 'M 1' streams the raw sensor-frame readings so the mounting can be
// checked against the CAD before any balance loop trusts them.
uint8_t IMU_SDA = 8, IMU_SCL = 9, MPU_ADDR = 0x68;   // 'M 2' can move them to wherever the module is
bool imu_ok = false, imu_stream = false;
uint8_t imu_who = 0;
uint32_t imu_errors = 0;
int16_t imu_raw[6];                          // accel x y z (16384 = 1 g), gyro x y z (65.5 = 1 deg/s)

bool mpu_write(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(v);
  return Wire.endTransmission() == 0;
}
int mpu_read(uint8_t reg, uint8_t *buf, int n) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  const int got = Wire.requestFrom((int)MPU_ADDR, n);
  for (int i = 0; i < got; ++i) buf[i] = Wire.read();
  return got;
}
void imu_begin() {
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(20);                       // a loose wire must not hang the 50 Hz loop
  uint8_t who = 0;
  imu_ok = false;
  // WHO_AM_I: 0x68 on a genuine part, clones differ. AD0 pulled high moves the module to 0x69.
  if (mpu_read(0x75, &who, 1) != 1) {
    MPU_ADDR = MPU_ADDR == 0x68 ? 0x69 : 0x68;
    if (mpu_read(0x75, &who, 1) != 1) { MPU_ADDR = 0x68; return; }
  }
  imu_who = who;
  if (!mpu_write(0x6B, 0x01)) return;        // wake, clock from the X gyro PLL
  delay(50);
  imu_ok = mpu_write(0x19, 0x04)             // 200 Hz sample rate
        && mpu_write(0x1A, 0x03)             // 44 Hz low-pass: servo buzz out, body motion in
        && mpu_write(0x1B, 0x08)             // gyro +-500 deg/s
        && mpu_write(0x1C, 0x00);            // accel +-2 g
}
bool imu_read() {
  uint8_t b[14];
  if (mpu_read(0x3B, b, 14) != 14) { ++imu_errors; return false; }
  for (int k = 0; k < 3; ++k) {
    imu_raw[k]     = (int16_t)((b[2 * k] << 8) | b[2 * k + 1]);          // accel, 0x3B..0x40
    imu_raw[3 + k] = (int16_t)((b[8 + 2 * k] << 8) | b[9 + 2 * k]);      // gyro,  0x43..0x48
  }
  return true;
}

// Mounting: robot = IMU_MAP * sensor, robot axes x forward, y left, z up. The default is the board on
// 14_imu_bracket as measured from gravity - upright, nose down, left side down, the two tilts agreeing
// within 2 deg: X points back, Y to the robot's right, Z up. That is the old mount (X left, Y back)
// turned 90 deg; going by the description "X to the right" instead had the axes wrong.
// "U" replaces it with nine numbers and stores them, so the next remount is a calibration, not a flash.
float IMU_MAP[3][3] = {{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}};
float gbias[3] = {0, 0, 0};                  // deg/s, sensor frame
float a_lp[3] = {0, 0, 0};                   // low-passed accel, sensor frame, for "is it still"
bool a_lp_init = false;
float integ_roll = 0, integ_pitch = 0;       // rad/s: gyro offset the accelerometer has uncovered
int imu_zero_ticks = 0, fall_ticks = 0;

void imu_update(float dt) {
  const float sa[3] = {imu_raw[0] / 16384.0f, imu_raw[1] / 16384.0f, imu_raw[2] / 16384.0f};
  if (fabsf(sa[0]) + fabsf(sa[1]) + fabsf(sa[2]) < 0.05f) {
    // An MPU6050 that browns out resets into sleep and reads exactly zero. Wake it again rather
    // than steer on zeros; a loose VCC wire did exactly this on the bench.
    att_valid = false;
    if (++imu_zero_ticks % 50 == 10) { imu_begin(); out.println(F("IMU read zeros - woke it again")); }
    return;
  }
  imu_zero_ticks = 0;
  const float sg[3] = {imu_raw[3] / 65.5f, imu_raw[4] / 65.5f, imu_raw[5] / 65.5f};
  // Gyro offset, learned only while the robot is genuinely still: gravity reads 1 g and is not
  // changing, and no axis turns faster than an offset could. The first version averaged the first
  // second after boot and then only followed readings near its own estimate - the board reset while
  // the robot was slumping, took that motion for offset, and locked itself out: pitch sat 50 deg off.
  if (!a_lp_init) { for (int k = 0; k < 3; ++k) a_lp[k] = sa[k]; a_lp_init = true; }
  float dev = 0, san = 0;
  for (int k = 0; k < 3; ++k) {
    a_lp[k] += 0.1f * (sa[k] - a_lp[k]);
    dev += (sa[k] - a_lp[k]) * (sa[k] - a_lp[k]);
    san += sa[k] * sa[k];
  }
  san = sqrtf(san);
  const bool still = mode != WALK && mode != STAND && san > 0.95f && san < 1.05f && dev < 0.0004f &&
                     fabsf(sg[0]) < 20 && fabsf(sg[1]) < 20 && fabsf(sg[2]) < 20;
  if (still) for (int k = 0; k < 3; ++k) gbias[k] += 0.02f * (sg[k] - gbias[k]);
  float a[3], w[3];
  for (int i = 0; i < 3; ++i) {
    a[i] = w[i] = 0;
    for (int j = 0; j < 3; ++j) { a[i] += IMU_MAP[i][j] * sa[j]; w[i] += IMU_MAP[i][j] * (sg[j] - gbias[j]); }
    w[i] *= PI / 180.0f;
  }
  const float roll_acc = atan2f(a[1], a[2]);
  const float pitch_acc = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));
  acc_roll = roll_acc; acc_pitch = pitch_acc;
  if (!att_valid) { att_roll = roll_acc; att_pitch = pitch_acc; integ_roll = integ_pitch = 0; att_valid = true; }
  // Complementary filter, ~1 s time constant. The accelerometer is ignored while it reads well
  // away from 1 g - a step, a knock - and the gyro carries the estimate through.
  const float an = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  const float k = (an > 0.7f && an < 1.3f) ? 0.02f : 0.0f;
  if (k > 0) {
    // Integral term: whatever offset is left makes a steady disagreement with gravity, and this
    // turns it into a rate correction - the same job as the Kalman's bias state. Tested offline on
    // a boot during a fall with 15 deg/s of offset: under 1 deg tipped by hand, under 1.7 walking.
    integ_roll = constrain(integ_roll + 1.0f * (roll_acc - att_roll) * dt, -0.5f, 0.5f);
    integ_pitch = constrain(integ_pitch + 1.0f * (pitch_acc - att_pitch) * dt, -0.5f, 0.5f);
  }
  att_roll = (1 - k) * (att_roll + (w[0] + integ_roll) * dt) + k * roll_acc;
  att_pitch = (1 - k) * (att_pitch + (w[1] + integ_pitch) * dt) + k * pitch_acc;
  rate_roll = w[0];
  rate_pitch = w[1];
  rate_yaw = w[2];
  yaw_rad += w[2] * dt;
  const float ga = dt / (G_RATE_TAU + dt);
  g_rate_roll += ga * (rate_roll - g_rate_roll);
  g_rate_pitch += ga * (rate_pitch - g_rate_pitch);
}

void fb_tick(float dt) {
  float want_r = 0, want_p = 0, want_d = 0;
  const bool pushing = att_valid && push_on && mode == HOLD;
  const bool on = !pushing && att_valid && ((fb_mode >= 1 && mode == WALK) || (fb_mode == 2 && mode == HOLD));
  if (pushing) {
    if (!push_ref_init) { push_ref_roll = att_roll; push_ref_pitch = att_pitch; push_ref_init = true; }
    push_ref_roll += 0.005f * (att_roll - push_ref_roll);        // ~4 s: follows poses, not pushes
    push_ref_pitch += 0.005f * (att_pitch - push_ref_pitch);
    const float u = -push_kpr * (att_roll - push_ref_roll) - push_kdr * rate_roll;
    want_r = constrain(u, -FB_CLAMP_ROLL, FB_CLAMP_ROLL);
    want_d = constrain(PUSH_DIFF * u, -PUSH_CLAMP_DIFF, PUSH_CLAMP_DIFF);
    want_p = constrain(-push_kpp * (att_pitch - push_ref_pitch) - push_kdp * rate_pitch, -FB_CLAMP_PITCH, FB_CLAMP_PITCH);
  } else {
    push_ref_init = false;
    if (on) {
      if (!fb_ref_init) {
        fb_ref_roll = att_roll; fb_ref_pitch = att_pitch;
        fb_err_roll = fb_err_pitch = fb_rate_roll = fb_rate_pitch = 0;
        fb_ref_init = true;
      }
      const float er = att_roll - fb_ref_roll, ep = att_pitch - fb_ref_pitch;
      if (fb_tau > 0.01f) {
        const float a = min(1.0f, dt / fb_tau);
        fb_err_roll += (er - fb_err_roll) * a;
        fb_err_pitch += (ep - fb_err_pitch) * a;
        fb_rate_roll += (rate_roll - fb_rate_roll) * a;
        fb_rate_pitch += (rate_pitch - fb_rate_pitch) * a;
      } else {
        fb_err_roll = er; fb_err_pitch = ep;
        fb_rate_roll = rate_roll; fb_rate_pitch = rate_pitch;
      }
      want_r = constrain(-fb_kpr * fb_err_roll - fb_kdr * fb_rate_roll, -FB_CLAMP_ROLL, FB_CLAMP_ROLL);
      want_p = constrain(-fb_kpp * fb_err_pitch - fb_kdp * fb_rate_pitch, -FB_CLAMP_PITCH, FB_CLAMP_PITCH);
    } else {
      fb_ref_init = false;
    }
  }
  const float cs = 3.0f * dt;                // same slew as the servos; switching off fades out
  corr_roll += constrain(want_r - corr_roll, -cs, cs);
  corr_pitch += constrain(want_p - corr_pitch, -cs, cs);
  corr_diff += constrain(want_d - corr_diff, -cs, cs);
  if (mode == HOLD && (on || pushing || fabsf(corr_roll) > 1e-4f || fabsf(corr_pitch) > 1e-4f ||
                       fabsf(corr_diff) > 1e-4f)) write_all();

  // Down is down. Walking on after a fall only grinds the servos against the floor. Both the
  // filtered angle and gravity alone have to agree: a filter estimate that had gone wrong on its
  // own once cut the servos the moment WALK was pressed, with the robot standing upright.
  // + pitch is nose down
  const bool tipped = (fabsf(att_roll) > FALL_RAD && fabsf(acc_roll) > FALL_RAD) ||
                      (att_pitch > FALL_FWD_RAD && acc_pitch > FALL_FWD_RAD) ||
                      (att_pitch < -FALL_BACK_RAD && acc_pitch < -FALL_BACK_RAD);
  if ((mode == WALK || mode == ROCK || mode == STEP || (mode == HOLD && push_on)) && att_valid && tipped) {
    if (++fall_ticks >= 10) {
      mode = LIMP; fall_ticks = 0; corr_roll = corr_pitch = corr_diff = 0; rock_deg = 0;
      for (int j = 0; j < 6; ++j) ledcWrite(cal[j].pin, 0);
      out.println(F("fell over - servos limp; 'e' to wake"));
    }
  } else fall_ticks = 0;
}

void wifi_start() {
  prefs.begin("net", true);
  wifi_ssid = prefs.getString("ssid", "");
  const String pass = prefs.getString("pass", "");
  prefs.end();
  wifi_configured = wifi_ssid.length() > 0;
  wifi_up = false;
  if (!wifi_configured) return;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                      // modem sleep adds 100+ ms to every packet in
  // Transmit power. 8.5 dBm is the usual cure for C3 SuperMinis that will not join at full power, and
  // it was the first setting here - but this board joins fine, and at 8.5 its own transmissions were
  // what got lost: it saw every connection arrive while its SYN-ACKs, data and ping replies went
  // missing for seconds at a time. 15 dBm by default, and "N p <dBm>" tries another level live.
  prefs.begin("net", true);
  wifi_txp = constrain(prefs.getFloat("txp", wifi_txp), 2.0f, 19.5f);
  prefs.end();
  WiFi.setTxPower((wifi_power_t)(int)(wifi_txp * 4));       // the enum is in quarter-dBm steps
  WiFi.begin(wifi_ssid.c_str(), pass.c_str());
}

void net_flush() {
  if (net_fd < 0) { net_out_len = 0; return; }
  if (!net_out_len) return;
  const int sent = send(net_fd, net_out, net_out_len, MSG_DONTWAIT);
  if (sent > 0) {
    memmove(net_out, net_out + sent, net_out_len - sent);
    net_out_len -= sent;
  } else if (sent < 0) {
    // EAGAIN is the ordinary "not now". ENOMEM is lwIP saying the same thing when its send queue is
    // out of pbufs - treating that as fatal closed the connection in the middle of a fast exchange.
    const int e = errno;
    if (e != EAGAIN && e != EWOULDBLOCK && e != ENOMEM) {
      net_fd = -1; net_out_len = 0; net_client.stop();
      out.printf("wifi client dropped: send errno %d\n", e);
    }
  }
}

void net_poll() {
  static uint32_t last = 0;
  net_flush();
  if (!wifi_configured) return;
  if (millis() - last > 1000) {
    last = millis();
    const bool up = WiFi.status() == WL_CONNECTED;
    if (up && !wifi_up) {
      wifi_up = true;
      MDNS.begin("duckbert");
      MDNS.addService("duckbert", "tcp", 2323);
      net_server.begin();
      net_server.setNoDelay(true);
      out.printf("wifi connected to \"%s\", ip %s - duckbert.local:2323\n", wifi_ssid.c_str(),
                 WiFi.localIP().toString().c_str());
    } else if (!up && wifi_up) {
      wifi_up = false;
      if (net_fd >= 0) { net_fd = -1; net_client.stop(); }
      out.println(F("wifi lost - rejoining"));
    }
  }
  if (!wifi_up) return;
  if (net_server.hasClient()) {
    WiFiClient c = net_server.accept();
    if (c) {                                 // one client at a time; a new one replaces the old
      if (net_fd >= 0) net_client.stop();
      net_client = c;
      net_client.setNoDelay(true);
      net_fd = net_client.fd();
      net_len = 0;
      net_out_len = 0;
      out.println(F("wifi client connected"));
    }
  }
  if (net_fd < 0) return;
  for (int budget = 64; budget > 0; --budget) {
    char ch;
    const int n = recv(net_fd, &ch, 1, MSG_DONTWAIT);
    if (n == 1) {
      if (ch == '\n' || ch == '\r') {
        if (net_len) { net_buf[net_len] = 0; net_len = 0; exec_line(net_buf); return; }
      } else if (net_len < sizeof(net_buf) - 1) {
        net_buf[net_len++] = ch;
      }
    } else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
      const int e = n == 0 ? 0 : errno;
      net_fd = -1; net_client.stop();
      out.printf("wifi client gone (%s %d)\n", n == 0 ? "closed by the other end" : "recv errno", e);
      return;
    } else {
      return;                                // nothing more waiting
    }
  }
}

// ---------------------------------------------------------------- face: 1.3 in SH1106 OLED
// The C3 has one I2C controller and the IMU has it on GPIO 8/9. The face gets its own two pins and
// a bit-banged bus, driven from a task below the control loop's priority: a frame is ~1 KB, which
// ties up the CPU for tens of milliseconds, and it only ever gets the time the 50 Hz loop leaves.
// I2C is clocked by the master, so being preempted mid-byte only makes a frame take longer.
static const uint8_t OLED_SDA = 10, OLED_SCL = 7, OLED_ADDR = 0x3C;
bool oled_ok = false;

static inline void swi_sda(uint32_t v) { gpio_ll_set_level(&GPIO, OLED_SDA, v); }
static inline void swi_scl(uint32_t v) { gpio_ll_set_level(&GPIO, OLED_SCL, v); }
static inline void swi_dly() { esp_rom_delay_us(4); }   // margin for weak pull-ups and long wires
uint32_t oled_frames = 0, oled_nacks = 0;
const char *oled_fault = "not probed";
volatile bool oled_test = false;
static void swi_start() { swi_sda(1); swi_scl(1); swi_dly(); swi_sda(0); swi_dly(); swi_scl(0); swi_dly(); }
static void swi_stop() { swi_sda(0); swi_dly(); swi_scl(1); swi_dly(); swi_sda(1); swi_dly(); }
static bool swi_byte(uint8_t b) {
  for (int i = 7; i >= 0; --i) { swi_sda((b >> i) & 1); swi_dly(); swi_scl(1); swi_dly(); swi_scl(0); }
  swi_sda(1); swi_dly(); swi_scl(1); swi_dly();
  const bool ack = gpio_ll_get_level(&GPIO, OLED_SDA) == 0;
  swi_scl(0); swi_dly();
  return ack;
}

// Just enough of a display for RoboEyes, which only calls clearDisplay, display, fillRoundRect and
// fillTriangle. The buffer is laid out the way the SH1106 takes it: one byte per column per 8-row page.
class OledSW : public Adafruit_GFX {
 public:
  OledSW() : Adafruit_GFX(128, 64) {}
  uint8_t buf[128 * 8];
  void drawPixel(int16_t x, int16_t y, uint16_t c) override {
    if (x < 0 || y < 0 || x >= 128 || y >= 64) return;
    if (c) buf[x + (y >> 3) * 128] |= 1 << (y & 7);
    else buf[x + (y >> 3) * 128] &= ~(1 << (y & 7));
  }
  void clearDisplay() { memset(buf, 0, sizeof(buf)); }
  bool command(const uint8_t *c, size_t n) {
    swi_start();
    bool ok = swi_byte(OLED_ADDR << 1) && swi_byte(0x00);
    for (size_t i = 0; ok && i < n; ++i) ok = swi_byte(c[i]);
    swi_stop();
    if (!ok) ++oled_nacks;
    return ok;
  }
  void display() {
    for (uint8_t p = 0; p < 8; ++p) {
      // Page p, then the column as two nibble commands: 0x10|high, 0x00|low. Column 2 is where the
      // SH1106's 132-column RAM puts the first visible pixel. (The first version sent 0x12, 0x00 -
      // column 32 - and would have drawn the face 30 pixels to the right, wrapped.)
      const uint8_t addr[] = {(uint8_t)(0xB0 + p), 0x10, 0x02};
      command(addr, sizeof(addr));
      swi_start();
      bool ok = swi_byte(OLED_ADDR << 1) && swi_byte(0x40);
      for (int x = 0; ok && x < 128; ++x) ok = swi_byte(buf[x + p * 128]);
      swi_stop();
      if (!ok) ++oled_nacks;
    }
    ++oled_frames;
  }
  bool begin() {
    // Route both pads to the GPIO matrix first. gpio_set_direction does not: the first version skipped
    // this, and GPIO7 comes up as the JTAG MTDO pad, so writing its output register never moved SCL.
    // The display never saw a clock, stayed stuck holding SDA low, and that low read back as an
    // acknowledge on every byte - "found, 0 refused" with a dark screen. The module itself was fine: on
    // a plain ESP32's hardware I2C it lit up as an SH1106.
    gpio_reset_pin((gpio_num_t)OLED_SDA);
    gpio_reset_pin((gpio_num_t)OLED_SCL);
    gpio_set_direction((gpio_num_t)OLED_SDA, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction((gpio_num_t)OLED_SCL, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode((gpio_num_t)OLED_SDA, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)OLED_SCL, GPIO_PULLUP_ONLY);
    swi_sda(1); swi_scl(1); delay(5);
    // SCL has to follow us, or nothing below means anything.
    swi_scl(0); swi_dly();
    const bool scl_low = gpio_ll_get_level(&GPIO, OLED_SCL) == 0;
    swi_scl(1); swi_dly(); delay(1);
    const bool scl_high = gpio_ll_get_level(&GPIO, OLED_SCL) == 1;
    if (!scl_low || !scl_high) { oled_fault = scl_low ? "SCL will not go high (no pull-up, or shorted low)" : "SCL will not go low (pin not driving)"; return false; }
    // Bus recovery: a device left mid-byte holds SDA low until it is clocked out.
    for (int i = 0; i < 9 && gpio_ll_get_level(&GPIO, OLED_SDA) == 0; ++i) { swi_scl(0); swi_dly(); swi_scl(1); swi_dly(); }
    swi_stop(); delay(1);
    if (gpio_ll_get_level(&GPIO, OLED_SDA) == 0) { oled_fault = "SDA held low (module unpowered? VCC/GND swapped?)"; return false; }
    swi_start(); const bool present = swi_byte(OLED_ADDR << 1); swi_stop();
    if (!present) { oled_fault = "no answer at 0x3C (wiring, or SDA/SCL swapped)"; return false; }
    oled_fault = "none";
    // Adafruit_SH1106G's start-up sequence, plus 0x8D 0x14: the SSD1306 charge pump. Some "1.3 inch"
    // modules carry an SSD1306 instead, which stays dark without it; the SH1106 ignores the pair.
    static const uint8_t init[] = {0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0xAD, 0x8B, 0x8D, 0x14,
                                   0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xFF, 0xD9, 0x1F, 0xDB, 0x40, 0x33, 0xA6,
                                   0x20, 0x10, 0xA4};
    if (!command(init, sizeof(init))) return false;
    clearDisplay(); display();
    delay(100);
    const uint8_t on = 0xAF;
    return command(&on, 1);
  }
};
OledSW oled;
RoboEyes<OledSW> eyes(oled);

enum Face { FACE_DEFAULT, FACE_ASLEEP, FACE_SLEEPY, FACE_CURIOUS_L, FACE_CURIOUS_R, FACE_WORRIED, FACE_HAPPY };
static const char *FACE_NAME[] = {"default", "asleep", "sleepy", "curious-left", "curious-right", "worried", "happy"};
volatile int face_want = FACE_ASLEEP;
uint32_t worried_until = 0;

void face_tick() {
  // Worried while the push reaction is visibly working, and for a moment after, so a push reads
  // as a scare rather than a flicker.
  if (push_on && mode == HOLD && (fabsf(corr_roll) > 2.0f * PI / 180.0f || fabsf(corr_diff) > 1.5f * PI / 180.0f))
    worried_until = millis() + 1500;
  int f;
  if (mode == LIMP) f = FACE_ASLEEP;
  else if (mode == WALK) f = FACE_HAPPY;
  else if (mode == WAKE || mode == STAND) f = FACE_DEFAULT;
  else if (millis() < worried_until) f = FACE_WORRIED;
  else if (pose_kind == POSE_TILT_L) f = FACE_CURIOUS_L;
  else if (pose_kind == POSE_TILT_R) f = FACE_CURIOUS_R;
  else if (att_valid && fabsf(att_roll) > 15.0f * PI / 180.0f)       // tipped over sideways by hand
    f = att_roll > 0 ? FACE_CURIOUS_R : FACE_CURIOUS_L;
  else if (pose_kind == POSE_CROUCH && pose_crouch_deg >= 20.0f) f = FACE_SLEEPY;
  else f = FACE_DEFAULT;
  face_want = f;
}

void apply_face(int f) {
  eyes.setSweat(false); eyes.setHFlicker(false); eyes.setCuriosity(false);
  if (f != FACE_ASLEEP) eyes.open();
  switch (f) {
    case FACE_ASLEEP:
      // Closed eyes are one-pixel lines, and TIRED draws its eyelids in black over the top of the
      // eye - over the whole of a closed one. That made "asleep" a blank screen. Plain mood, eyes shut.
      eyes.setMood(EYES_DEFAULT); eyes.setIdleMode(false); eyes.setAutoblinker(false); eyes.setPosition(EYES_POS_S);
      eyes.close();
      break;
    case FACE_SLEEPY:
      eyes.setMood(EYES_TIRED); eyes.setIdleMode(false); eyes.setPosition(EYES_POS_S); eyes.setAutoblinker(true, 5, 3);
      break;
    case FACE_CURIOUS_L: case FACE_CURIOUS_R:
      // The face looks out of the robot, so its own left is the viewer's right: east on the screen.
      eyes.setMood(EYES_DEFAULT); eyes.setIdleMode(false); eyes.setCuriosity(true);
      eyes.setPosition(f == FACE_CURIOUS_L ? EYES_POS_E : EYES_POS_W); eyes.setAutoblinker(true, 3, 2);
      break;
    case FACE_WORRIED:
      eyes.setMood(EYES_TIRED); eyes.setIdleMode(false); eyes.setPosition(EYES_DEFAULT);
      eyes.setSweat(true); eyes.setHFlicker(true, 2); eyes.setAutoblinker(true, 2, 1);
      break;
    case FACE_HAPPY:
      eyes.setMood(EYES_HAPPY); eyes.setIdleMode(true, 2, 2); eyes.setAutoblinker(true, 3, 2); eyes.anim_laugh();
      break;
    default:
      eyes.setMood(EYES_DEFAULT); eyes.setIdleMode(true, 3, 2); eyes.setAutoblinker(true, 3, 2); eyes.setPosition(EYES_DEFAULT);
      break;
  }
}

void face_task(void *) {
  // The display is looked for again and again, not once. The first version probed at power-up only and
  // deleted this task if nothing answered - so a face plugged in after boot, or knocked loose and pushed
  // back, stayed dark until the next reset. Now: no answer, try again in 2 s; a display that stops
  // acknowledging for ~20 frames in a row is treated as unplugged and searched for again, and the
  // start-up sequence is sent afresh, which a module that lost power needs anyway.
  for (;;) {
    oled_ok = oled.begin();
    if (!oled_ok) { vTaskDelay(pdMS_TO_TICKS(2000)); continue; }
    eyes.begin(128, 64, 12);
    apply_face(FACE_DEFAULT);                // eyes open for two seconds, so a working display shows it
    for (uint32_t t0 = millis(); millis() - t0 < 2000;) { eyes.update(); vTaskDelay(pdMS_TO_TICKS(5)); }
    int shown = -1, bad = 0;
    for (;;) {
      if (oled_test) {                       // Y 1: every pixel on for three seconds, then back
        const uint8_t allon = 0xA5, resume = 0xA4;
        oled.command(&allon, 1); vTaskDelay(pdMS_TO_TICKS(3000)); oled.command(&resume, 1);
        oled_test = false;
      }
      const int f = face_want;
      if (f != shown) { apply_face(f); shown = f; }
      eyes.setFramerate(mode == WALK || push_on ? 8 : 12);   // less face while the robot is busy
      const uint32_t frames = oled_frames, nacks = oled_nacks;
      eyes.update();
      if (oled_frames != frames) bad = oled_nacks != nacks ? bad + 1 : 0;
      if (bad >= 20) break;                  // gone: back to looking for it
      vTaskDelay(pdMS_TO_TICKS(5));
    }
    oled_ok = false;
  }
}

float wake_clock = 0.0f;
int wake_tick = 0;

void begin_wake() {
  // command_rad already holds the pose to wake into.
  for (int j = 0; j < 6; ++j) ledcWrite(cal[j].pin, 0);
  wake_clock = 0.0f;
  wake_tick = 0;
  mode = WAKE;
  last_control = micros();
  out.printf("waking into a %.0f deg crouch, one joint at a time, %.1f s\n",
                command_rad[1] * 180.0f / PI, 5 * WAKE_STAGGER_S + WAKE_RAMP_S);
}

void go_neutral(uint32_t ms = 1200) {
  const float zero[6] = {0, 0, 0, 0, 0, 0};
  ease_to(zero, ms);
}

// ---------------------------------------------------------------- calibration storage
void load_trims() {
  prefs.begin("duckbert", true);
  for (int j = 0; j < 6; ++j) {
    char key[8];
    snprintf(key, sizeof(key), "t%d", j);
    cal[j].trim_rad = prefs.getFloat(key, cal[j].trim_rad);
    snprintf(key, sizeof(key), "s%d", j);
    cal[j].sign = prefs.getChar(key, cal[j].sign);
    snprintf(key, sizeof(key), "p%d", j);
    cal[j].pin = prefs.getUChar(key, cal[j].pin);
  }
  // Speed used to live in RAM only, so every reset - including unplugging the USB cable - put
  // the gait back to full pace behind your back. It is part of how this robot is set up,
  // so it saves with 'w' like the trims do.
  speed = constrain(prefs.getFloat("spd", speed), 0.1f, 1.5f);
  gait = constrain((int)prefs.getUChar("gait", gait), 0, N_GAITS + N_SHIFTS - 1);
  amp = constrain(prefs.getFloat("amp", amp), 0.1f, 1.0f);
  sway = constrain(prefs.getFloat("sway", sway), 0.0f, 1.0f);
  walk_lean = constrain(prefs.getFloat("lean", walk_lean), -30.0f, 30.0f);
  walk_turn = constrain(prefs.getFloat("turn", walk_turn), -1.0f, 1.0f);
  knee_scale = constrain(prefs.getFloat("knee", knee_scale), 0.5f, 2.0f);
  gait_fb_scale = constrain(prefs.getFloat("gfs", gait_fb_scale), 0.0f, 3.0f);
  pitch_own = prefs.getBool("jp", pitch_own);
  pitch_kp = constrain(prefs.getFloat("jkp", pitch_kp), -3.0f, 3.0f);
  pitch_kd = constrain(prefs.getFloat("jkd", pitch_kd), -0.3f, 0.3f);
  sh_crouch = constrain(prefs.getFloat("shc", sh_crouch), 0.0f, 35.0f);
  sh_rock = constrain(prefs.getFloat("shr", sh_rock), -20.0f, 20.0f);
  sh_sharp = constrain(prefs.getFloat("shs", sh_sharp), 1.0f, 4.0f);
  sh_abd = constrain(prefs.getFloat("sha", sh_abd), -15.0f, 15.0f);
  walk_side = constrain(prefs.getFloat("side", walk_side), -10.0f, 10.0f);
  fb_kpr = prefs.getFloat("kpr", fb_kpr); fb_kdr = prefs.getFloat("kdr", fb_kdr);
  fb_tau = constrain(prefs.getFloat("fbt", fb_tau), 0.0f, 5.0f);
  fb_kpp = prefs.getFloat("kpp", fb_kpp); fb_kdp = prefs.getFloat("kdp", fb_kdp);
  push_kpr = prefs.getFloat("pkr", push_kpr); push_kdr = prefs.getFloat("pkd", push_kdr);
  push_kpp = prefs.getFloat("ppp", push_kpp); push_kdp = prefs.getFloat("ppd", push_kdp);
  prefs.end();
}

void save_trims() {
  prefs.begin("duckbert", false);
  for (int j = 0; j < 6; ++j) {
    char key[8];
    snprintf(key, sizeof(key), "t%d", j);
    prefs.putFloat(key, cal[j].trim_rad);
    snprintf(key, sizeof(key), "s%d", j);
    prefs.putChar(key, cal[j].sign);
    snprintf(key, sizeof(key), "p%d", j);
    prefs.putUChar(key, cal[j].pin);
  }
  prefs.putFloat("spd", speed);
  prefs.putUChar("gait", (uint8_t)gait);
  prefs.putFloat("amp", amp);
  prefs.putFloat("sway", sway);
  prefs.putFloat("lean", walk_lean); prefs.putFloat("turn", walk_turn); prefs.putFloat("knee", knee_scale); prefs.putFloat("gfs", gait_fb_scale); prefs.putBool("jp", pitch_own); prefs.putFloat("jkp", pitch_kp); prefs.putFloat("jkd", pitch_kd); prefs.putFloat("shc", sh_crouch); prefs.putFloat("shr", sh_rock); prefs.putFloat("shs", sh_sharp); prefs.putFloat("sha", sh_abd); prefs.putFloat("side", walk_side);
  prefs.putFloat("kpr", fb_kpr); prefs.putFloat("kdr", fb_kdr); prefs.putFloat("fbt", fb_tau);
  prefs.putFloat("kpp", fb_kpp); prefs.putFloat("kdp", fb_kdp);
  prefs.putFloat("pkr", push_kpr); prefs.putFloat("pkd", push_kdr);
  prefs.putFloat("ppp", push_kpp); prefs.putFloat("ppd", push_kdp);
  prefs.end();
  out.println(F("saved"));
}

void print_state() {
  out.printf("mode %s  speed %.2f  selected %s\n",
                mode_name(), speed, JOINT_NAME[selected]);
  for (int j = 0; j < 6; ++j)
    out.printf("  %-12s GPIO%-2d sign %+d trim %+6.1f deg   now %+6.1f deg%s\n",
                  JOINT_NAME[j], cal[j].pin, cal[j].sign,
                  cal[j].trim_rad * 180.0f / PI, joint_out(j) * 180.0f / PI,
                  pwm_ok[j] ? "" : "   <-- NO PWM");
}

// The same state, in one parse per line, for duckbert_console.py.
void print_machine() {
  bool all_ok = true;
  for (int j = 0; j < 6; ++j) all_ok &= pwm_ok[j];
  out.printf("STATE mode=%s speed=%.2f sel=%d pwm=%d gait=%d stand=%d amp=%.2f sway=%.2f imu=%d "
                "txp=%.1f lean=%.1f turn=%.0f knee=%.0f gimu=%d gfs=%.0f sc=%.1f sr=%.1f ss=%.2f sa=%.1f side=%.1f rock=%.1f rockt=%.2f rockd=%.2f yaw=%.1f hold=%d ytgt=%.1f tc=%.0f rec=%d oled=%d oledf=%lu oledn=%lu face=%s imumap=%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f wifi=%d ip=%s rst=%s push=%d pkr=%.3f pkd=%.3f ppp=%.3f ppd=%.3f cd=%.1f att=%d roll=%.1f pitch=%.1f aroll=%.1f apitch=%.1f fb=%d kpr=%.3f kdr=%.3f kpp=%.3f kdp=%.3f fbt=%.2f cr=%.1f cp=%.1f jp=%d jkp=%.3f jkd=%.3f fwd=%.0f rkm=%d rkt=%.1f rka=%.1f rkl=%.2f rkp=%.1f rkg=%.2f rku=%.3f rkd=%.3f rkc=%.1f pkt=%.1f rks=%d rkh=%.1f stt=%.3f yerr=%.1f arm=%d stc=%d\n",
                mode_name(), speed, selected, all_ok, gait, (int)STANDUP_AVAILABLE, amp, sway, (int)imu_ok,
                wifi_txp, walk_lean, walk_turn * 100.0f, knee_scale * 100.0f, gait_imu, gait_fb_scale * 100.0f,
                sh_crouch, sh_rock, sh_sharp, sh_abd, walk_side, rock_deg, rock_period, rock_diff, yaw_rad * 180.0f / PI,
                yaw_hold, yaw_target * 180.0f / PI, turn_corr * 100.0f, (int)log_on, (int)oled_ok, (unsigned long)oled_frames, (unsigned long)oled_nacks, FACE_NAME[face_want], IMU_MAP[0][0], IMU_MAP[0][1], IMU_MAP[0][2], IMU_MAP[1][0],
                IMU_MAP[1][1], IMU_MAP[1][2], IMU_MAP[2][0], IMU_MAP[2][1], IMU_MAP[2][2],
                (int)wifi_up, wifi_up ? WiFi.localIP().toString().c_str() : "-", reset_reason, (int)push_on, push_kpr, push_kdr, push_kpp, push_kdp, corr_diff * 180.0f / PI,
                (int)att_valid, att_roll * 180.0f / PI, att_pitch * 180.0f / PI,
                acc_roll * 180.0f / PI, acc_pitch * 180.0f / PI, fb_mode,
                fb_kpr, fb_kdr, fb_kpp, fb_kdp, fb_tau, corr_roll * 180.0f / PI, corr_pitch * 180.0f / PI,
                (int)pitch_own, pitch_kp, pitch_kd, walk_fwd * 100.0f,
                rock_mode, rock_thr_deg, rock_amp_deg, rock_lead_s, rock_peak, rock_gain, rock_pump, rock_pdamp, rock_trim_deg, pitch_trim_deg, rock_style, rock_hip_amp, step_turn, (yaw_target - yaw_f) * 180.0f / PI, (int)step_armed, step_count);
  for (int j = 0; j < 6; ++j)
    out.printf("J %d %s pin=%d sign=%d trim=%.2f now=%.2f min=%.1f max=%.1f\n",
                  j, JOINT_NAME[j], cal[j].pin, cal[j].sign,
                  cal[j].trim_rad * 180.0f / PI, command_rad[j] * 180.0f / PI,
                  JOINT_MIN_RAD[j] * 180.0f / PI, JOINT_MAX_RAD[j] * 180.0f / PI);
  out.println(F("END"));
}

void help() {
  out.println(F(
      "\n  Commands are one per line - type the letter and press Enter.\n"
      "  g  walk      s  stop and hold      n  ease to neutral\n"
      "  l  limp (stop sending pulses - the robot will collapse, hold it)\n"
      "  j [0-5]  select joint, or step to the next one\n"
      "  [ ]  trim selected by -/+ 1 deg      i  invert the selected joint's sign\n"
      "  k  drive selected to its limits, slowly\n"
      "  a <j> <deg>  ease joint j to an angle   t <j> <deg>  set joint j's trim\n"
      "  z <j> <+1|-1>  set joint j's sign (eases to zero first, so nothing snaps)\n"
      "  P <j> <gpio>  move a joint to another pin    d  LEDC readback per pin\n"
      "  x <gpio>  pad test: drive high/low and read the pin back\n"
      "  u <gpio> <us>  raw servo pulse, bypassing joint/sign/trim/limits\n"
      "  c 0|1|2  calibration pose: 0 CAD zero, 1 thigh square, 2 shin square\n"
      "  POWER-UP IS LIMP: nothing moves until 'e'\n"
      "  e  wake into a crouch, one joint at a time     q [deg]  crouch (default 25, max 80)\n"
      "  y  park: crouch, then limp - do this before switching off\n"
      "  T 1 | T -1 | T 0  tilt the head to its left / its right / straight (from a crouch)\n"
      "  M 1 | M 0  stream raw IMU readings at 50 Hz / stop.  M  IMU status\n"
      "  B <0|1|2> [kPr kDr kPp kDp]  balance off / walking / walking+holding, gains. Off at power-up\n"
      "  K <0|1> [kPr kDr kPp kDp]  push reaction while holding a pose. Off at power-up\n"
      "  N ssid|password  join Wi-Fi (stored in flash)   N -  forget   N  status. TCP on duckbert.local:2323\n"
      "  N p <dBm>  Wi-Fi transmit power, 2-19.5 (stored)\n"
      "  U m00..m22  IMU mounting, robot = M * sensor (stored)   face: 1.3 in OLED, SDA GPIO10 SCL GPIO7\n"
      "  Y  face status   Y 1  display test: every pixel on for 3 s\n"
      "  L <deg>  trunk lean while walking, + = back (saved with 'w')\n"
      "  R 1/0    record: stream one line a tick of commands, attitude and corrections\n"
      "  W <pct>  steering: hip swing asymmetry, + turns right (saved with 'w')\n"
      "  Q <deg> [s] [share]  rock the weight from foot to foot, no stepping; Q 0 stops\n"
      "  I 1/0    the gait's own gyro feedback, for the gaits that were searched with it\n"
      "  F <pct>  how much of that feedback: 100 = as searched (saved with 'w')\n"
      "  C <crouch> <rock> <sharp> <open>  sequence walks: bent legs, leg-length rock, quick lift, hip opening\n"
      "  E <pct>  knee swing: toe clearance, 100 = the gait's own\n"
      "  H 1/0    heading hold: steer on the gyro to keep the line it set off on\n"
      "  D <deg>  turn by that many degrees, + to its right\n"
      "  G <n>  choose gait n (stops walking first)   S  stand up from face down\n"
      "  S 0.25  the same stand-up 4x slower, as a dry run held in the air\n"
      "  A <0.3-1.0>  step size - smaller steps, same stance. Saved with 'w'\n"
      "  O <0.2-1.0>  hip sway only    V <0.2-1.5>  set gait speed directly\n"
      "  w  save calibration    r  reload saved    p  print state    ?  machine state\n"
      "  -  slower    +  faster    h  this\n"
      "\n  Calibrate with the robot OFF the ground:\n"
      "   Zero is NOT legs straight. At zero the robot stands in the CAD neutral pose:\n"
      "   thigh raked back about 25 deg, shin forward about 25 deg, sole flat. Standing,\n"
      "   the trunk floor is 117 mm above the table and the knee axis sits 62 mm up and\n"
      "   18 mm behind the hip pitch axis. Compare with cad/out_none/previews/side.png.\n"
      "   1. Hip roll is the easy one: at zero the hip bracket is horizontal. Trim it there.\n"
      "   2. 'c 1' puts the THIGH SQUARE TO THE TRUNK (hip pitch -25.10). Set square against\n"
      "      the trunk, edge along the line from the hip screw to the knee screw. Trim hip\n"
      "      pitch with [ ] until it is square. A horn only fits the spline in whole teeth,\n"
      "      so get within one tooth mechanically first - the trim is for the remainder.\n"
      "   3. 'c 2' puts the SHIN SQUARE TO THE TRUNK (knee +24.81). Same square, edge along\n"
      "      the shin from the knee screw to the foot bolts. Trim knee until it is square.\n"
      "      Measure against the trunk, not with a plumb line.\n"
      "   Signs, before any of that - 'k' on each joint, checked against the model, which is\n"
      "   NOT the intuitive one. Positive hip pitch moves that foot BACKWARD. Positive knee\n"
      "   ALSO moves the foot backward, and slightly DOWN - it is negative knee that lifts.\n"
      "   Positive hip roll moves BOTH feet toward the robot's left. The right leg answers\n"
      "   exactly like the left: the legs are not mirrored in the model, the walking antiphase\n"
      "   lives in the pattern, so a mirror-symmetric sign table cancels it. Wrong way: 'i'.\n"
      "   3. 'w' to save, then stand it up and 'g'.\n"));
}

// ---------------------------------------------------------------- command parsing
// Flipping a sign mirrors the joint about its zero, so a joint held away from zero would
// snap by twice its angle. Go to zero first and the flip costs no motion at all.
void set_sign(int j, int8_t sgn) {
  float pose[6];
  for (int k = 0; k < 6; ++k) pose[k] = command_rad[k];
  pose[j] = 0.0f;
  ease_to(pose, 400);
  cal[j].sign = sgn < 0 ? -1 : +1;
  write_all();
}

void jog(int j, float deg) {
  float pose[6];
  for (int k = 0; k < 6; ++k) pose[k] = command_rad[k];
  pose[j] = constrain(deg * PI / 180.0f, JOINT_MIN_RAD[j], JOINT_MAX_RAD[j]);
  const float moved = fabsf(pose[j] - command_rad[j]) * 180.0f / PI;
  mode = HOLD;
  ease_to(pose, constrain((uint32_t)(120 + moved * 15), 120u, 1500u));
}

void exec_line(char *s) {
  while (*s == ' ') ++s;
  const char cmd = *s;
  if (!cmd) return;
  int ji = -1;
  float val = 0.0f;
  const int nargs = sscanf(s + 1, "%d %f", &ji, &val);
  const bool has_joint = nargs >= 1 && ji >= 0 && ji <= 5;

  // Anything that drives the servos has to bring them back under power first, otherwise the
  // robot snaps from wherever gravity left it to a pose the sketch only thinks it is holding.
  // Nothing drives a servo out of LIMP except wake. Calibration numbers can still be changed
  // while limp - they are stored and applied at the next wake - but anything that moves waits.
  if (mode == LIMP && strchr("gsnkacSqyT", cmd)) {
    out.println(F("limp - press wake ('e') first"));
    return;
  }
  if (mode == WAKE && !strchr("l?phdM", cmd)) {
    out.println(F("still waking up - wait, or 'l' to stop"));
    return;
  }

  switch (cmd) {
    case 'g': {
      // Ease into where the gait starts instead of letting the slew limit jump there: from a
      // crouch that is 25 deg of hip and knee, which at 3 rad/s would be taken in a sixth of a second.
      if (mode != WALK) {
        float start[6];
        gait_targets(0.0f, start);
        for (int j = 0; j < 6; ++j) start[j] = constrain(start[j], JOINT_MIN_RAD[j], JOINT_MAX_RAD[j]);
        mode = HOLD;
        ease_to(start, 900);
      }
      gait_clock = 0.0f; last_control = micros(); mode = WALK;
      yaw_target = yaw_rad; yaw_f = yaw_rad; turn_corr = 0.0f; yaw_turning = false;   // straight ahead: where it points now
      g_ref_roll = att_roll; g_ref_pitch = att_pitch;   // and upright means: how it stands now
      g_rate_roll = rate_roll; g_rate_pitch = rate_pitch;
      out.println(F("walking"));
      break;
    }
    case 'e':
      if (mode != LIMP) { out.println(F("already powered")); break; }
      rest_pose(command_rad);
      begin_wake();
      pose_kind = POSE_CROUCH; pose_crouch_deg = WAKE_CROUCH_DEG;
      break;
    case 'q': {
      float deg = CROUCH_DEG;
      if (nargs >= 1) deg = (float)ji;
      float pose[6];
      crouch_pose(deg, pose);
      // Time the move by how far it goes: a slider step of a degree or two should answer at once,
      // and a jump from standing to a deep crouch should still be slow. A fixed 1.5 s made every
      // slider tick block for 1.5 s, and the queued ticks then played back one after another.
      float moved = 0.0f;
      for (int j = 0; j < 6; ++j) moved = max(moved, fabsf(pose[j] - command_rad[j]));
      const uint32_t ms = constrain((uint32_t)(200 + 55.0f * moved * 180.0f / PI), 200u, 2200u);
      mode = HOLD;
      ease_to(pose, ms);
      pose_kind = POSE_CROUCH; pose_crouch_deg = constrain(deg, 0.0f, CROUCH_MAX_DEG);
      out.printf("crouch %.0f deg\n", constrain(deg, 0.0f, CROUCH_MAX_DEG));
      print_machine();
      break;
    }
    case 'T': {
      const int side = nargs >= 1 ? (ji > 0) - (ji < 0) : 1;
      float pose[6];
      tilt_pose(side, pose);
      mode = HOLD;
      ease_to(pose, 1200);
      pose_kind = side > 0 ? POSE_TILT_L : side < 0 ? POSE_TILT_R : POSE_CROUCH;
      pose_crouch_deg = CROUCH_DEG;
      out.println(side > 0 ? F("head tilted to its left") : side < 0 ? F("head tilted to its right")
                              : F("head straight, crouch"));
      print_machine();
      break;
    }
    case 'y': {
      // Down to the wake pose (deep crouch, leaning forward) slowly, settle, then let go - from 25 deg it fell.
      float pose[6];
      rest_pose(pose);
      mode = HOLD; step_moving = false; step_armed = false;
      ease_to(pose, 2500);
      pose_kind = POSE_CROUCH; pose_crouch_deg = WAKE_CROUCH_DEG;
      delay(600);
      mode = LIMP;
      for (int j = 0; j < 6; ++j) ledcWrite(cal[j].pin, 0);
      out.println(F("parked in a crouch and limp - safe to switch off; 'e' wakes it into the same pose"));
      print_machine();
      break;
    }
    case 's': mode = HOLD; step_moving = false; step_armed = false; write_all(); out.println(F("holding")); break;
    case 'm': {
      if (mode == LIMP || mode == WAKE || mode == STAND) { out.println(F("wake it and stand it first")); break; }
      if (mode != STEP) {
        // into the staggered rest pose, left foot in front, slowly; the next 'm' is the first step
        float q[6];
        step_front = 0; step_moving = false; step_count = 0; step_last_lift = -1; rock_gain = 1.0f; rock_peak = 0.0f; rock_pump = 0.06f; rock_trim_deg = 0.0f;
        rock_hip_amp = 4.0f;
        step_turn = 0.0f; yaw_target = yaw_rad; yaw_f = yaw_rad;   // straight on: where it points now
        pitch_trim_deg = 0.0f;
        step_pitch_center = step_rock_center = step_last_hi = step_last_lo = 0.0f; step_lean_now = 0.0f;
        step_pose(false, 0.0f, step_front, q);
        for (int j = 0; j < 6; ++j) q[j] = constrain(q[j], JOINT_MIN_RAD[j], JOINT_MAX_RAD[j]);
        mode = HOLD;
        ease_to(q, 900);
        last_control = micros();
        g_ref_roll = att_roll;
        g_ref_pitch = att_pitch;     // a fixed reference is reached gradually in step_control()
        mode = STEP;
        out.println(F("step mode: left foot in front - 'm' takes one step, 's' leaves"));
      } else if (!step_moving && rock_mode > 0) {
        step_armed = true;
        out.printf("step armed: goes when the trunk leans %.1f deg towards the %s foot\n", rock_thr_deg, step_front == 0 ? "left" : "right");
      } else if (!step_moving && !step_shuffle) {
        step_clock = 0.0f; step_moving = true; step_skip_shift = false;
        step_lifting = step_last_lift == 0 ? 1 : 0;
        out.printf("step %d: %s foot steps\n", step_count + 1, step_front == 0 ? "right" : "left");
      }
      print_machine();
      break;
    }
    case 'v': {
      float a, b, d, e, f, g, pr, lg; int c, st, ip;
      const int n = sscanf(s + 1, "%f %f %d %f %f %f %f %d %f %d %f", &a, &b, &c, &d, &e, &f, &g, &st, &pr, &ip, &lg);
      if (n >= 3) {
        rock_thr_deg = constrain(a, 0.5f, 20.0f); rock_amp_deg = constrain(b, 1.0f, 60.0f); rock_mode = constrain(c, 0, 2);   // amp up to 60 on request (13 Oct)
        if (n >= 4) rock_lead_s = constrain(d, 0.0f, 0.4f);
        if (n >= 5) rock_pdamp = constrain(e, -0.3f, 0.3f);
        if (n >= 6) step_splay_deg = constrain(f, 0.0f, 10.0f);
        if (n >= 7) step_crouch_deg = constrain(g, 0.0f, 50.0f);
        if (n >= 8) rock_style = constrain(st, 0, 2);
        if (n >= 9) {
          step_pitch_ref_deg = constrain(pr, -99.0f, 99.0f);
        }
        rock_angle_share = n >= 11 ? constrain(lg, 0.0f, 1.0f) : 0.0f;
        if (!step_moving) { step_inplace = n >= 10 && ip == 1; step_shuffle = n >= 10 && ip == 2; }   // never under a foot in the air
        step_armed = false;
      }
      out.printf("rock steps: %s, mode %d, step when %.1f deg is reached %.2f s ahead, rock held to %.1f deg, pitch damping %.3f, legs %.1f deg wider, crouch %.0f deg, %s, pitch ref %.1f, angle share %.2f\n", step_shuffle ? "feet slide with the rock" : step_inplace ? "feet up in place" : "stepping", rock_mode, rock_thr_deg, rock_lead_s, rock_amp_deg, rock_pdamp, step_splay_deg, step_crouch_deg, rock_style == 2 ? "soft pump" : rock_style ? "pump" : "angle feedback", step_pitch_ref_deg, rock_angle_share);
      print_machine();
      break;
    }
    case 'o': {
      float v[12];
      const int n = sscanf(s + 1, "%f %f %f %f %f %f %f %f %f %f %f %f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8], &v[9], &v[10], &v[11]);
      if (n == 11) v[11] = 1.0f;                   // the first version's eleven numbers: the stance foot pushes
      if (n >= 11) {
        for (int i = 0; i < 12; ++i) stp[i] = v[i];
        stp[11] = constrain(stp[11], 0.0f, 1.0f);
        stp[6] = constrain(stp[6], 0.1f, 2.0f); stp[7] = constrain(stp[7], 0.08f, 1.0f); stp[8] = constrain(stp[8], 0.1f, 2.0f);
      }
      out.printf("step: hip0 %.3f knee0 %.3f stagger %.3f lift %.3f shift %.3f sink %.3f  shift %.2f s swing %.2f s land %.2f s  kP roll %.2f pitch %.2f  push %.2f\n",
                 stp[0], stp[1], stp[2], stp[3], stp[4], stp[5], stp[6], stp[7], stp[8], stp[9], stp[10], stp[11]);
      break;
    }
    case 'n': mode = HOLD; go_neutral(); out.println(F("neutral")); break;
    case 'l':
      mode = LIMP;
      for (int j = 0; j < 6; ++j) ledcWrite(cal[j].pin, 0);   // no pulse at all -> servo releases
      out.println(F("limp - hold the robot"));
      break;
    case 'j':
      selected = has_joint ? ji : (selected + 1) % 6;
      out.println(JOINT_NAME[selected]);
      break;
    case '[': cal[selected].trim_rad -= PI / 180.0f; write_all(); print_machine(); break;
    case ']': cal[selected].trim_rad += PI / 180.0f; write_all(); print_machine(); break;
    case 'i': set_sign(selected, -cal[selected].sign); print_machine(); break;
    case 'k': {
      mode = HOLD;
      float pose[6];
      for (int j = 0; j < 6; ++j) pose[j] = command_rad[j];
      // +-30 deg, not the full joint range. The sweep exists to show which way a joint turns,
      // and the feet-in body opened travel to +-60 hip and +-75 knee - driving there on every
      // direction check would put horns at their stops for no information.
      const float swing = 30.0f * PI / 180.0f;
      pose[selected] = min(JOINT_MAX_RAD[selected], swing);
      ease_to(pose, 900);
      pose[selected] = max(JOINT_MIN_RAD[selected], -swing);
      ease_to(pose, 1800);
      pose[selected] = 0.0f;
      ease_to(pose, 900);
      print_machine();
      break;
    }
    case 'a':
      if (nargs < 2 || !has_joint) { out.println(F("usage: a <joint 0-5> <deg>")); break; }
      selected = ji;
      jog(ji, val);
      print_machine();
      break;
    case 't':
      if (nargs < 2 || !has_joint) { out.println(F("usage: t <joint 0-5> <deg>")); break; }
      // A trim is added on top of the joint angle, so a wild one can push the horn past the
      // printed stops that JOINT_MIN/MAX are there to respect. 30 deg is already generous.
      cal[ji].trim_rad = constrain(val, -30.0f, 30.0f) * PI / 180.0f;
      selected = ji;
      write_all();
      print_machine();
      break;
    case 'z':
      if (nargs < 2 || !has_joint) { out.println(F("usage: z <joint 0-5> <+1|-1>")); break; }
      set_sign(ji, val < 0 ? -1 : +1);
      selected = ji;
      print_machine();
      break;
    case 'd':
      // Ask the LEDC peripheral itself what it is doing on each pin. An attach that took, a
      // duty in range and a frequency near 50 means the chip IS making that pulse, and a servo
      // that still does not move is wiring, power, or the servo - not this sketch.
      for (int j = 0; j < 6; ++j)
        out.printf("D %d %-12s gpio=%-2d attach=%d duty=%lu freq=%lu\n",
                      j, JOINT_NAME[j], cal[j].pin, pwm_ok[j],
                      (unsigned long)ledcRead(cal[j].pin),
                      (unsigned long)ledcReadFreq(cal[j].pin));
      out.println(F("END"));
      break;
    case 'P': {
      // Move a joint to a different GPIO without recompiling, for when a pin turns out to be
      // dead or wanted by something else. Saved with 'w' like the rest of the calibration.
      if (nargs < 2 || !has_joint) { out.println(F("usage: P <joint 0-5> <gpio>")); break; }
      const int gpio = (int)val;
      if (gpio < 0 || gpio > 21) { out.println(F("gpio out of range")); break; }
      if (gpio == IMU_SDA || gpio == IMU_SCL || gpio == OLED_SDA || gpio == OLED_SCL) {
        out.printf("GPIO%d is the IMU's or the face's bus\n", gpio);
        break;
      }
      for (int k = 0; k < 6; ++k)
        if (k != ji && cal[k].pin == gpio) {
          out.printf("GPIO%d already drives %s\n", gpio, JOINT_NAME[k]);
          return;
        }
      ledcDetach(cal[ji].pin);
      cal[ji].pin = (uint8_t)gpio;
      pwm_ok[ji] = ledcAttach(cal[ji].pin, SERVO_HZ, PWM_BITS);
      out.printf("%s -> GPIO%d, attach %s\n", JOINT_NAME[ji], gpio,
                    pwm_ok[ji] ? "ok" : "FAILED");
      write_all();
      print_machine();
      break;
    }
    case 'x': {
      // Does the pad itself move? The LEDC registers can look perfect while the pin is shorted,
      // damaged, or held by something else. Drive it with the input buffer left on and read back
      // what the pad is actually at, then let the internal pulls fight whatever is out there.
      //   healthy pin, servo signal input attached:  hi=1 lo=0 pullup=1 pulldown=0
      //   shorted to ground / dead low driver:       hi=0 lo=0 pullup=0 pulldown=0
      //   shorted to 3V3:                            hi=1 lo=1 pullup=1 pulldown=1
      if (nargs < 1 || ji < 0 || ji > 21 || (ji > 10 && ji < 20)) {
        out.println(F("usage: x <gpio 0-10 or 20-21>  (11-19 are flash and USB)"));
        break;
      }
      const gpio_num_t gp = (gpio_num_t)ji;
      int owner = -1;
      for (int k = 0; k < 6; ++k) if (cal[k].pin == ji) owner = k;
      if (owner >= 0) ledcDetach(ji);

      gpio_reset_pin(gp);
      gpio_set_direction(gp, GPIO_MODE_INPUT_OUTPUT);
      gpio_set_level(gp, 1); delayMicroseconds(400);
      const int hi = gpio_get_level(gp);
      gpio_set_level(gp, 0); delayMicroseconds(400);
      const int lo = gpio_get_level(gp);
      gpio_set_direction(gp, GPIO_MODE_INPUT);
      gpio_set_pull_mode(gp, GPIO_PULLUP_ONLY);   delay(3);
      const int pu = gpio_get_level(gp);
      gpio_set_pull_mode(gp, GPIO_PULLDOWN_ONLY); delay(3);
      const int pd = gpio_get_level(gp);
      gpio_set_pull_mode(gp, GPIO_FLOATING);

      out.printf("X gpio=%d drive_hi=%d drive_lo=%d pullup=%d pulldown=%d owner=%s\n",
                    (int)ji, hi, lo, pu, pd, owner >= 0 ? JOINT_NAME[owner] : "-");
      if (owner >= 0) {
        pwm_ok[owner] = ledcAttach(ji, SERVO_HZ, PWM_BITS);
        write_joint(owner, command_rad[owner]);
      }
      break;
    }
    case 'u': {
      // A raw pulse on a raw pin, with no joint, sign, trim or limit in the way. 1500 us is
      // centre, 1000 and 2000 are the ends of a normal SG90 sweep. If a servo will not answer
      // this, nothing in the calibration is what is wrong with it.
      if (nargs < 2 || ji < 0 || ji > 21 || (ji > 10 && ji < 20)) {
        out.println(F("usage: u <gpio> <microseconds 400-2600>"));
        break;
      }
      const int us = (int)constrain(val, 400.0f, 2600.0f);
      int owner = -1;
      for (int k = 0; k < 6; ++k) if (cal[k].pin == ji) owner = k;
      if (owner < 0) ledcAttach(ji, SERVO_HZ, PWM_BITS);
      ledcWrite(ji, (uint32_t)(us * (1UL << PWM_BITS) * SERVO_HZ / 1000000.0f));
      out.printf("U gpio=%d us=%d%s\n", (int)ji, us,
                    owner >= 0 ? "  (next joint command overwrites it)" : "");
      break;
    }
    case 'c': {
      const int which = nargs >= 1 ? ji : 0;
      float pose[6] = {0, 0, 0, 0, 0, 0};
      const char *what;
      if (which == 1) {
        pose[1] = pose[4] = POSE_THIGH_SQUARE_DEG * PI / 180.0f;
        what = "thigh (hip screw to knee screw) square to the trunk - trim hip pitch here";
      } else if (which == 2) {
        pose[2] = pose[5] = POSE_SHIN_SQUARE_DEG * PI / 180.0f;
        what = "shin (knee screw to foot bolts) square to the trunk - trim knee here";
      } else {
        what = "CAD zero - thigh 25 deg back, shin 25 deg forward, sole flat";
      }
      mode = HOLD;
      ease_to(pose, 900);
      out.printf("pose %d: %s\n", which, what);
      print_machine();
      break;
    }
    case 'A': {
      float v;
      if (sscanf(s + 1, "%f", &v) != 1) { out.println(F("usage: A <0.3-1.0>  step size")); break; }
      amp = constrain(v, 0.1f, 1.0f);            // live: the slew limit smooths the change
      out.printf("step size %.2f\n", amp);
      print_machine();
      break;
    }
    case 'O': {
      float v;
      if (sscanf(s + 1, "%f", &v) != 1) { out.println(F("usage: O <0.2-1.0>  hip sway")); break; }
      sway = constrain(v, 0.0f, 1.0f);
      out.printf("hip sway %.2f\n", sway);
      print_machine();
      break;
    }
    case 'V': {
      // Set the speed directly; '+' and '-' step it by 0.1, which is too coarse here - in
      // simulation mid still walks at 0.95 and 0.9 and goes over sideways within a second at 0.85.
      float v;
      if (sscanf(s + 1, "%f", &v) != 1) { out.println(F("usage: V <0.2-1.5>  gait speed")); break; }
      speed = constrain(v, 0.1f, 1.5f);
      out.printf("speed %.2f\n", speed);
      print_machine();
      break;
    }
    case 'G':
      if (nargs < 1 || ji < 0 || ji >= N_GAITS + N_SHIFTS) {
        out.printf("usage: G <0-%d>\n", N_GAITS - 1);
        break;
      }
      // Switching period mid-stride jumps the phase, so a change of gait always stops first.
      if (mode == WALK) { mode = HOLD; out.println(F("stopped to change gait")); }
      gait = ji;
      if (is_shift()) {                            // a sequence walk runs as it was searched
        amp = 1.0f; speed = 1.0f;
      } else if (GAITS[gait].amp0 > 0.01f) {       // the pair the gait was validated with
        amp = constrain(GAITS[gait].amp0, 0.1f, 1.0f);
        speed = constrain(GAITS[gait].speed0, 0.1f, 1.5f);
      }
      out.printf("gait %d '%s', stride %.3f s, step %.2f, cadence %.2f, own gyro feedback %s\n",
                 gait, gait_name(), gait_period(), amp, speed,
                 is_shift() ? (gait_imu ? "on" : "OFF")
                 : (GAITS[gait].imu[0] || GAITS[gait].imu[1] || GAITS[gait].imu[2] || GAITS[gait].imu[3])
                   ? (gait_imu ? "on" : "OFF") : "none");
      print_machine();
      break;
    case 'S': {
      if (!STANDUP_AVAILABLE) {
        out.println(F("no stand-up controller verified for this body yet - see standup_params.h"));
        break;
      }
      // Same move the search ran: waypoint 0 is the pose the servos are already in, then four
      // searched poses joined by smoothstep and put through the same slew limit. The four IMU
      // balance gains are not here - there is no IMU - and the controller was only exported
      // after it stood up without them.
      for (int j = 0; j < 6; ++j) stand_way[0][j] = command_rad[j];
      for (int w = 0; w < 4; ++w) {
        const float *q = STANDUP + 4 + 7 * w;
        const float v[6] = {q[0] + q[4], q[1] + q[5], q[2] + q[6],
                            -q[0] + q[4], q[1] - q[5], q[2] - q[6]};
        for (int j = 0; j < 6; ++j)
          stand_way[w + 1][j] = constrain(v[j], JOINT_MIN_RAD[j], JOINT_MAX_RAD[j]);
      }
      // 'S 0.25' is a dry run: the same poses at a quarter of the speed, durations stretched and
      // slew cut to match, to hold in the air and watch for a horn or bracket that binds. The
      // move reaches -54 deg of hip pitch and -65 of knee, past the travel the first body had.
      // Slowed down it will not stand anyone up - that is not what it is for.
      stand_scale = 1.0f;
      {
        float sc = 1.0f;
        if (sscanf(s + 1, "%f", &sc) == 1) stand_scale = constrain(sc, 0.1f, 1.0f);
      }
      stand_edge[0] = 0.0f;
      for (int k = 0; k < 4; ++k) stand_edge[k + 1] = stand_edge[k] + STANDUP[k] / stand_scale;
      stand_clock = 0.0f;
      last_control = micros();
      mode = STAND;
      out.printf("%s, %.2f s\n", stand_scale < 1.0f ? "stand-up DRY RUN - hold it in the air"
                                                        : "standing up from face down", stand_edge[4]);
      break;
    }
    case 'B': {
      int md; float a1, a2, a3, a4, a5;
      const int n = sscanf(s + 1, "%d %f %f %f %f %f", &md, &a1, &a2, &a3, &a4, &a5);
      if (n >= 1) fb_mode = constrain(md, 0, 2);
      if (n >= 5) { fb_kpr = a1; fb_kdr = a2; fb_kpp = a3; fb_kdp = a4; }
      if (n >= 6) fb_tau = constrain(a5, 0.0f, 5.0f);
      fb_ref_init = false;                        // the next walk sets its own upright
      if (fb_mode && !att_valid) out.println(F("no attitude yet - the IMU is not reading"));
      out.printf("balance %s  roll kP %.3f kD %.3f  pitch kP %.3f kD %.3f  answers leans slower than %.2f s\n",
                    fb_mode == 0 ? "OFF" : fb_mode == 1 ? "while walking" : "while walking AND holding",
                    fb_kpr, fb_kdr, fb_kpp, fb_kdp, fb_tau);
      print_machine();
      break;
    }
    case 'N': {
      // "N ssid|password" stores and joins, "N -" forgets, "N" reports. The password is never printed.
      char *arg = s + 1;
      while (*arg == ' ') ++arg;
      if (arg[0] == 'p' && arg[1] == ' ') {
        wifi_txp = constrain((float)atof(arg + 2), 2.0f, 19.5f);
        prefs.begin("net", false); prefs.putFloat("txp", wifi_txp); prefs.end();
        if (wifi_configured) WiFi.setTxPower((wifi_power_t)(int)(wifi_txp * 4));
        out.printf("wifi transmit power %.1f dBm (stored)\n", wifi_txp);
        break;
      }
      if (*arg == '-') {
        prefs.begin("net", false); prefs.clear(); prefs.end();
        if (net_fd >= 0) { net_fd = -1; net_client.stop(); }
        WiFi.disconnect(true); wifi_configured = wifi_up = false; wifi_ssid = "";
        out.println(F("wifi forgotten"));
      } else if (*arg) {
        char *bar = strchr(arg, '|');
        if (!bar) { out.println(F("usage: N ssid|password")); break; }
        *bar = 0;
        prefs.begin("net", false); prefs.putString("ssid", arg); prefs.putString("pass", bar + 1); prefs.end();
        wifi_start();
        out.printf("wifi saved for \"%s\" - joining\n", arg);
      }
      out.printf("wifi %s ssid \"%s\" ip %s rssi %d txp %.1f client %d\n",
                 !wifi_configured ? "not configured" : wifi_up ? "connected" : "joining",
                 wifi_ssid.c_str(), wifi_up ? WiFi.localIP().toString().c_str() : "-",
                 wifi_up ? WiFi.RSSI() : 0, wifi_txp, (int)(net_fd >= 0));
      break;
    }
    case 'R': {
      const bool want = atoi(s + 1) != 0;
      if (want && !log_on) {
        log_t0 = millis(); log_n = 0;
        out.printf("DHDR hz=%d gait=%d period=%.3f amp=%.2f sway=%.2f speed=%.2f lean=%.1f knee=%.0f gimu=%d gfs=%.0f sc=%.1f sr=%.1f ss=%.2f sa=%.1f side=%.1f fb=%d "
                   "kpr=%.3f kdr=%.3f kpp=%.3f kdp=%.3f push=%d units=0.1deg "
                   "turn=%.0f cols=ms,phase,mode,q0,q1,q2,q3,q4,q5,roll,pitch,aroll,apitch,rroll,rpitch,ryaw,yaw,tc,cr,cp,cd,sat\n",
                   (int)CONTROL_HZ, gait, gait_period(), amp, sway, speed, walk_lean, knee_scale * 100.0f, gait_imu, gait_fb_scale * 100.0f, sh_crouch, sh_rock, sh_sharp, sh_abd, walk_side, fb_mode,
                   fb_kpr, fb_kdr, fb_kpp, fb_kdp, (int)push_on, walk_turn * 100.0f);
        // heading is measured from the start of the run - and the hold's target and filter move with it (13 Oct:
        // ● registra after "tieni la direzione" left the target at +249 deg and the heading at 0, and the hold
        // swung it round at full steer: walk_20261013_154410, the first 9 s at +15 deg/s)
        yaw_target -= yaw_rad; yaw_f -= yaw_rad;
        yaw_rad = 0;
      } else if (!want && log_on) {
        out.printf("log stopped, %lu lines in %.1f s\n", (unsigned long)log_n,
                   (millis() - log_t0) / 1000.0f);
      }
      log_on = want;
      break;
    }
    case 'H': {
      int md; float a1, a2;
      const int n = sscanf(s + 1, "%d %f %f", &md, &a1, &a2);
      if (n >= 1) yaw_hold = md ? 1 : 0;
      // a negative gain flips the steering: which way the stride asymmetry turns this robot is measured
      if (n >= 3) { yaw_kp = constrain(a1, -5.0f, 5.0f); yaw_kd = constrain(a2, 0.0f, 2.0f); }
      if (yaw_hold) { yaw_target = yaw_rad; yaw_f = yaw_rad; turn_corr = 0.0f; }
      if (yaw_hold && !att_valid) out.println(F("no attitude yet - the IMU is not reading"));
      out.printf("heading hold %s, kP %.2f kD %.2f, target %+.0f deg\n",
                 yaw_hold ? "ON" : "off", yaw_kp, yaw_kd, yaw_target * 180.0f / PI);
      print_machine();
      break;
    }
    case 'D': {
      float v;
      if (sscanf(s + 1, "%f", &v) != 1) {
        out.println(F("usage: D <deg>  turn: + to its right, on top of the current heading"));
        break;
      }
      yaw_target -= v * PI / 180.0f;               // + deg means turn right, and right is -yaw
      yaw_turning = true;
      if (!yaw_hold) { yaw_hold = 1; out.println(F("heading hold switched on to do it")); }
      out.printf("turning %+.0f deg, target %+.0f deg\n", v, yaw_target * 180.0f / PI);
      print_machine();
      break;
    }
    case 'E': {
      float v;
      if (sscanf(s + 1, "%f", &v) != 1) {
        out.println(F("usage: E <50..200>  knee swing, percent: lifts the toe clear"));
        break;
      }
      knee_scale = constrain(v / 100.0f, 0.5f, 2.0f);
      out.printf("knee swing %.0f%% of the gait's own\n", knee_scale * 100.0f);
      print_machine();
      break;
    }
    case 'I': {
      const int v = atoi(s + 1);
      gait_imu = v ? 1 : 0;
      out.printf("the gait's own gyro feedback %s\n", gait_imu ? "on" : "off");
      print_machine();
      break;
    }
    case 'X': {
      float v;
      if (sscanf(s + 1, "%f", &v) != 1) {
        out.println(F("usage: X <deg>  sideways trunk trim, + moves the trunk to its left"));
        break;
      }
      walk_side = constrain(v, -10.0f, 10.0f);
      out.printf("sideways trim %+.1f deg (+ = trunk to its left)\n", walk_side);
      print_machine();
      break;
    }
    case 'C': {
      float a1, a2, a3, a4;
      if (sscanf(s + 1, "%f %f %f %f", &a1, &a2, &a3, &a4) != 4) {
        out.println(F("usage: C <crouch deg> <rock deg> <lift sharpness 1-4> <hip opening deg>  (sequence walks)"));
        break;
      }
      sh_crouch = constrain(a1, 0.0f, 35.0f);
      sh_rock = constrain(a2, -20.0f, 20.0f);
      sh_sharp = constrain(a3, 1.0f, 4.0f);
      sh_abd = constrain(a4, -15.0f, 15.0f);
      out.printf("sequence walk: crouch %.1f, rock %+.1f, lift sharpness %.2f, hip opening %+.1f\n",
                 sh_crouch, sh_rock, sh_sharp, sh_abd);
      print_machine();
      break;
    }
    case 'J': {
      float kp, kd;
      if (sscanf(s + 1, "%f %f", &kp, &kd) == 2) {
        pitch_own = true;
        pitch_kp = constrain(kp, -3.0f, 3.0f);
        pitch_kd = constrain(kd, -0.3f, 0.3f);
        out.printf("pitch feedback on its own gains: kP %.3f kD %.3f (negative lowers the hips as the trunk tips back)\n", pitch_kp, pitch_kd);
      } else {
        pitch_own = false;
        out.println(F("pitch feedback back to the walk's searched gains"));
      }
      print_machine();
      break;
    }
    case 'Z': {
      float v;
      if (sscanf(s + 1, "%f", &v) != 1) v = 100.0f;
      walk_fwd = constrain(v / 100.0f, 0.0f, 1.0f);
      out.printf("forward share of the stride %.0f%% (0 + W = turn on the spot)\n", walk_fwd * 100.0f);
      print_machine();
      break;
    }
    case 'F': {
      float v;
      if (sscanf(s + 1, "%f", &v) != 1) {
        out.println(F("usage: F <0..150>  share of the gait's own gyro gains, percent"));
        break;
      }
      gait_fb_scale = constrain(v / 100.0f, 0.0f, 3.0f);
      out.printf("the gait's own gyro feedback at %.0f%% of its searched gains\n", gait_fb_scale * 100.0f);
      print_machine();
      break;
    }
    case 'Q': {
      float deg, per = rock_period, diff = rock_diff;
      const int n = sscanf(s + 1, "%f %f %f", &deg, &per, &diff);
      if (n < 1) {
        out.println(F("usage: Q <deg> [period_s] [leg-length share]  rock the weight side to side, 0 stops"));
        break;
      }
      if (mode == LIMP || mode == WAKE) { out.println(F("wake it first")); break; }
      rock_deg = constrain(deg, 0.0f, 20.0f);
      if (n >= 2) rock_period = constrain(per, 0.3f, 4.0f);
      if (n >= 3) rock_diff = constrain(diff, 0.0f, 1.5f);
      if (rock_deg <= 0.01f) {
        if (mode == ROCK) { mode = HOLD; crouch_pose(pose_crouch_deg, command_rad); }
        out.println(F("rocking stopped"));
      } else {
        rock_clock = 0.0f; last_control = micros(); mode = ROCK;
        out.printf("rocking +-%.1f deg every %.2f s, legs %.2f\n", rock_deg, rock_period, rock_diff);
      }
      print_machine();
      break;
    }
    case 'f': {
      float v;
      if (sscanf(s + 1, "%f", &v) == 1) step_go = constrain(v / 100.0f, 0.0f, 1.0f);
      out.printf("go %.0f%% (now %.0f%%)\n", step_go * 100.0f, step_go_now * 100.0f);
      break;
    }
    case 'W': {
      float v;
      if (sscanf(s + 1, "%f", &v) != 1) {
        out.println(F("usage: W <-100..100>  steering, percent of hip swing, + turns right"));
        break;
      }
      walk_turn = constrain(v / 100.0f, -1.0f, 1.0f);
      out.printf("steering %+.0f%% (+ = turns to its right)\n", walk_turn * 100.0f);
      print_machine();
      break;
    }
    case 'L': {
      float v;
      if (sscanf(s + 1, "%f", &v) != 1) { out.println(F("usage: L <-30..30>  trunk lean while walking, + = back")); break; }
      walk_lean = constrain(v, -30.0f, 30.0f);     // live while walking: the slew limit smooths it
      out.printf("walking lean %+.1f deg (+ = trunk back)\n", walk_lean);
      print_machine();
      break;
    }
    case 'Y':
      if (nargs >= 1 && ji == 1) oled_test = true;
      out.printf("oled %s (%s)  SDA GPIO%d SCL GPIO%d  frames %lu  nacks %lu  face %s%s\n", oled_ok ? "found" : "NOT FOUND",
                 oled_fault, OLED_SDA, OLED_SCL, (unsigned long)oled_frames, (unsigned long)oled_nacks, FACE_NAME[face_want],
                 oled_test ? "  - all pixels on for 3 s" : "");
      break;
    case 'U': {
      float v[9];
      if (sscanf(s + 1, "%f %f %f %f %f %f %f %f %f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8]) != 9) {
        out.println(F("usage: U m00 m01 m02 m10 m11 m12 m20 m21 m22  (robot = M * sensor)"));
        break;
      }
      // Rows must be unit length and the whole a rotation, not a mirror: a mirrored map reverses
      // one axis and turns the balance loop's corrections the wrong way round.
      bool okm = true;
      for (int i = 0; i < 3; ++i) okm &= fabsf(sqrtf(v[3*i]*v[3*i] + v[3*i+1]*v[3*i+1] + v[3*i+2]*v[3*i+2]) - 1.0f) < 0.15f;
      const float det = v[0]*(v[4]*v[8]-v[5]*v[7]) - v[1]*(v[3]*v[8]-v[5]*v[6]) + v[2]*(v[3]*v[7]-v[4]*v[6]);
      if (!okm || det < 0.7f) { out.printf("rejected: not a rotation (det %.2f)\n", det); break; }
      for (int i = 0; i < 9; ++i) IMU_MAP[i / 3][i % 3] = v[i];
      prefs.begin("duckbert", false); prefs.putBytes("imumap", IMU_MAP, sizeof(IMU_MAP)); prefs.end();
      att_valid = false;                         // start the attitude estimate over in the new frame
      out.println(F("IMU mounting stored"));
      print_machine();
      break;
    }
    case 'K': {
      int on; float a1, a2, a3, a4;
      const int n = sscanf(s + 1, "%d %f %f %f %f", &on, &a1, &a2, &a3, &a4);
      if (n >= 1) push_on = on != 0;
      if (n == 5) { push_kpr = a1; push_kdr = a2; push_kpp = a3; push_kdp = a4; }
      if (push_on && !att_valid) out.println(F("no attitude yet - the IMU is not reading"));
      if (push_on && mode != HOLD) out.println(F("push reaction acts while holding a pose - wake it and crouch"));
      out.printf("push reaction %s  roll kP %.3f kD %.3f  pitch kP %.3f kD %.3f\n", push_on ? "ON" : "OFF",
                    push_kpr, push_kdr, push_kpp, push_kdp);
      print_machine();
      break;
    }
    case 'M':
      if (nargs >= 1 && ji == 2) {
        // Bus scan over every free pin pair, both ways round. The servo pins (0-4, 6) are left out
        // so the scan cannot detach a servo's PWM. A pair whose lines are held low times out on its
        // first probe and is skipped, so a wrong guess costs 20 ms, not two seconds.
        static const uint8_t cand[] = {5, 7, 8, 9, 10, 20, 21};
        int found_sda = -1, found_scl = -1, found_addr = -1;
        for (uint8_t a : cand) for (uint8_t b : cand) {
          if (a == b) continue;
          Wire.end();
          Wire.begin(a, b); Wire.setClock(100000); Wire.setTimeOut(20);
          String hits;
          for (uint8_t addr = 1; addr < 127; ++addr) {
            Wire.beginTransmission(addr);
            const uint8_t e = Wire.endTransmission();
            if (e == 0) { hits += String(" 0x") + String(addr, HEX);
                          if ((addr == 0x68 || addr == 0x69) && found_addr < 0) { found_sda = a; found_scl = b; found_addr = addr; } }
            else if (e == 5) break;               // timeout: this pair is not a bus
          }
          if (hits.length()) out.printf("scan SDA=GPIO%d SCL=GPIO%d:%s\n", a, b, hits.c_str());
        }
        Wire.end();
        if (found_addr >= 0) { IMU_SDA = found_sda; IMU_SCL = found_scl; MPU_ADDR = found_addr; }
        out.printf("scan done%s\n", found_addr >= 0 ? " - using the MPU found above" : " - no MPU at 0x68/0x69 on any free pair");
        imu_begin();
      } else if (nargs >= 1) imu_stream = ji != 0;
      if (!imu_ok) imu_begin();                  // asking again retries a module plugged in late
      out.printf("imu %s who=0x%02X addr=0x%02X SDA=GPIO%d SCL=GPIO%d errors=%lu stream=%d\n",
                    imu_ok ? "found" : "NOT FOUND", imu_who, MPU_ADDR, IMU_SDA, IMU_SCL,
                    (unsigned long)imu_errors, (int)imu_stream);
      if (!imu_ok) imu_stream = false;
      break;
    case 'w': save_trims(); print_machine(); break;
    case 'r': load_trims(); write_all(); print_machine(); break;
    case 'p': print_state(); break;
    case '?': print_machine(); break;
    case '-': speed = max(0.1f, speed - 0.1f); print_machine(); break;
    case '+': speed = min(1.5f, speed + 0.1f); print_machine(); break;
    case 'h': help(); break;
    default: out.printf("unknown command '%c' - 'h' for help\n", cmd); break;
  }
}

// ---------------------------------------------------------------- setup and loop
void setup() {
  // The CDC transmit buffer defaults to 256 bytes, and with the zero timeout below a write that does
  // not fit is dropped rather than waited for - 'h' came out with the middle missing. 4 KB holds the
  // whole help text; it has to be set before begin(), which only allocates one if none exists.
  Serial.setTxBufferSize(4096);
  Serial.begin(115200);
  // On the C3 Serial is USB CDC, and a write blocks while no host is draining it. Off a
  // battery, with no console attached, the banner below would otherwise stall setup() before
  // it ever reaches go_neutral() and the robot would look dead for a different reason.
  Serial.setTxTimeoutMs(0);
  delay(300);

  load_trims();
  bool all_ok = true;
  for (int j = 0; j < 6; ++j) {
    pwm_ok[j] = ledcAttach(cal[j].pin, SERVO_HZ, PWM_BITS);
    all_ok &= pwm_ok[j];
  }
  if (!all_ok) {
    out.println(F("\n*** LEDC ATTACH FAILED - THE SERVOS WILL NOT MOVE ***"));
    for (int j = 0; j < 6; ++j)
      if (!pwm_ok[j]) out.printf("    GPIO%d (%s)\n", cal[j].pin, JOINT_NAME[j]);
    out.println(F("    a pin already in use, or a resolution the chip will not take\n"));
  }

  out.println(F("\nDuckbert open-loop walk. Direct GPIO servo drive, no IMU on this board."));
  out.printf("gait %d/%d '%s', stride %.3f s, stand-up %s\n", gait, N_GAITS + N_SHIFTS, gait_name(),
                gait_period(), STANDUP_AVAILABLE ? STANDUP_NAME : "not available");
  help();
  // No pulses at power-up. The servos stay wherever they were left until 'e' wakes them - one at
  // a time, into the crouch they were parked in - instead of all jumping to a pose at once.
  crouch_pose(CROUCH_DEG, command_rad);
  for (int j = 0; j < 6; ++j) ledcWrite(cal[j].pin, 0);
  // Why the board last started. A robot that "resets when it falls" is either browning out on a
  // supply that cannot carry six stalled servos, or losing USB power outright, or crashing.
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  reset_reason = "POWERON"; break;    // power applied, or USB power cut and back
    case ESP_RST_BROWNOUT: reset_reason = "BROWNOUT"; break;   // supply sagged below ~2.8 V
    case ESP_RST_PANIC:    reset_reason = "PANIC"; break;      // firmware crash
    case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: reset_reason = "WATCHDOG"; break;
    case ESP_RST_SW:       reset_reason = "SOFTWARE"; break;
    case ESP_RST_EXT:      reset_reason = "RESET_PIN"; break;
    case ESP_RST_USB:      reset_reason = "USB"; break;        // reset requested over USB, e.g. a flash
    default:               reset_reason = "OTHER"; break;
  }
  out.printf("reset reason: %s\n", reset_reason);
  out.println(F("\nServos are LIMP. Nothing moves until 'e' (wake)."));
  imu_begin();
  out.printf("IMU %s (WHO_AM_I 0x%02X)\n", imu_ok ? "found" : "not found", imu_who);
  prefs.begin("duckbert", true);
  if (prefs.getBytesLength("imumap") == sizeof(IMU_MAP)) prefs.getBytes("imumap", IMU_MAP, sizeof(IMU_MAP));
  prefs.end();
  // Below the loop task's priority 1: the face only runs when the control loop is waiting.
  xTaskCreate(face_task, "face", 6144, nullptr, 0, nullptr);
  wifi_start();
  out.printf("wifi %s\n", wifi_configured ? "joining in the background" : "not configured - N ssid|password over USB");
}

// Line buffered, so it works the same from a plain serial monitor (type, press Enter) and
// from duckbert_console.py. Bounded per call so a flood cannot starve the 50 Hz loop.
char line_buf[128];
uint8_t line_len = 0;

void handle_serial() {
  for (int budget = 64; budget > 0 && Serial.available(); --budget) {
    const char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (line_len) {
        line_buf[line_len] = 0;
        line_len = 0;
        exec_line(line_buf);
        return;                       // one command per call; 'k' and 'n' block for seconds
      }
    } else if (line_len < sizeof(line_buf) - 1) {
      line_buf[line_len++] = ch;
    }
  }
}

void control_step(float dt);
void log_tick();

void loop() {
  handle_serial();
  net_poll();

  const uint32_t now = micros();
  if ((uint32_t)(now - last_control) < CONTROL_US) {
    // Hand the CPU on while there is nothing to do. loop() used to spin, and a spinning loop starves
    // anything below it - the face task among them. A 1 ms tick is the whole cost in control jitter.
    vTaskDelay(1);
    return;
  }
  last_control += CONTROL_US;
  const float dt = 1.0f / CONTROL_HZ;

  // 13 Oct: flashed over USB with the battery off, the board booted with the IMU unpowered, never found it, and
  // never looked again - a whole run with roll and pitch frozen at 0 and no rock at all (walk_20261013_112414),
  // until 'M' asked for it. Not found, it is now asked for once a second.
  static uint32_t imu_retry_ms = 0;
  if (!imu_ok && millis() - imu_retry_ms > 1000) {
    imu_retry_ms = millis();
    imu_begin();
    if (imu_ok) out.println(F("IMU found again"));
  }
  if (imu_ok && imu_read()) {
    imu_update(dt);
    if (imu_stream)
      out.printf("I %lu %d %d %d %d %d %d\n", (unsigned long)millis(), imu_raw[0], imu_raw[1],
                    imu_raw[2], imu_raw[3], imu_raw[4], imu_raw[5]);
  }
  fb_tick(dt);
  face_tick();

  control_step(dt);
  log_tick();
}

// Everything the current mode does with this tick. Split out of loop() so that the recorder
// below sees the commands of the tick it is logging, not of the one before.
// ---------------------------------------------------------------- one step at a time ('m')
// single_step.py line for line. At rest one foot is ahead of the other with the weight between them;
// 'm' takes one step: the weight shifts onto the front foot (both hip rolls lean the trunk over it and
// that leg shortens a little), the back foot lifts high and passes forward fast while the stance foot
// pushes back, it lands in front, and the robot waits for the next 'm'. 's' leaves.
//   stp: hip0 knee0 stagger lift shift sink t_shift t_swing t_land kp_roll kp_pitch push   ('o' sets them)
// Second version (8 Oct): push 0 keeps the stance foot still while the other is up (the body moves forward
// in the landing, both feet down), and + shift is + hip roll when the left foot carries.
static const float STEP_SLEW_RAD_S = 7.0f;   // fast on purpose: an SG90 does about 10

static float smooth01(float u) { u = constrain(u, 0.0f, 1.0f); return u * u * (3.0f - 2.0f * u); }

// Joint targets for rest (moving=false) or t seconds into a step whose stance leg is `front`.
bool step_pose(bool moving, float t, int front, float q[6]) {
  const float h0 = stp[0], k0 = stp[1], s = stp[2], A = stp[3], sh = stp[4], sk = stp[5];
  const float ta = stp[6], tb = stp[7], tc = stp[8], push = stp[11];
  float x[2] = {front == 0 ? s : -s, front == 0 ? -s : s};
  // + turns it left. Sign from the bench (10 Oct, 17:37/17:38): the left foot ahead turned it RIGHT, -70 deg
  // in 17 s with the hold asking for left the whole time; the first guess, from the geometry, was backwards.
  if (step_shuffle && !moving) {
    // 13 Oct (walk_20261013_095344): a fixed offset only changes how it stands - the hold sat at its limit for
    // 40 s and it kept turning +2.7 deg/s, exactly as without it. Sliding, it steers the way a walk does: the
    // outer foot slides further. Turning left (+), the right foot's swing grows and the left one's shrinks.
    x[0] = step_shuffle_x; x[1] = -step_shuffle_x;   // 13 Oct: steering by leaning now (step_control), not by stride
  } else {
    x[0] -= step_turn; x[1] += step_turn;
  }
  float lift[2] = {0, 0}, sink[2] = {0, 0}, roll = 0;
  if (step_shuffle && !moving) { lift[0] = step_shuffle_lift[0]; lift[1] = step_shuffle_lift[1]; }
  bool done = false;
  if (moving && step_inplace) {
    // feet stay where they are; shift and sink ease in over the first third of the swing (the rock has
    // already moved the weight - jumping them in at once is a kick sideways)
    const int st = 1 - step_lifting, sw = step_lifting;
    const float side = st == 0 ? 1.0f : -1.0f;
    if (t < ta) t = ta;
    if (t < ta + tb) {
      const float u = (t - ta) / tb, e = smooth01(min(1.0f, 3.0f * u));
      roll = side * sh * e; sink[st] = sk * e;
      const float b = sinf(PI * min(1.0f, u)); lift[sw] = A * b * b;
    } else if (t < ta + tb + tc) {
      const float u = smooth01((t - ta - tb) / tc);
      roll = side * sh * (1 - u); sink[st] = sk * (1 - u);
    } else {
      done = true;
    }
  } else if (moving) {
    const int st = front, sw = 1 - front;
    const float side = st == 0 ? 1.0f : -1.0f;     // + shift: + hip roll when the left foot carries
    if (t < ta) {
      const float u = smooth01(t / ta); roll = side * sh * u; sink[st] = sk * u;
    } else if (t < ta + tb) {
      const float u = (t - ta) / tb, w = smooth01(u);
      // 12 Oct: fired by the rock, shift and sink used to jump in at once - one of the jerks seen on the robot
      const float e = step_skip_shift ? smooth01(min(1.0f, 3.0f * u)) : 1.0f;
      roll = side * sh * e; sink[st] = sk * e;
      x[sw] = -s + 2 * s * w; x[st] = s - 2 * s * w * push;
      const float b = sinf(PI * min(1.0f, u)); lift[sw] = A * b * b;
    } else if (t < ta + tb + tc) {
      const float u = smooth01((t - ta - tb) / tc);
      roll = side * sh * (1 - u); sink[st] = sk * (1 - u);
      x[sw] = s; x[st] = s - 2 * s * (push + (1 - push) * u);
    } else {
      x[sw] = s; x[st] = -s; done = true;
    }
  }
  const float cr = step_crouch_deg * PI / 180.0f;
  for (int leg = 0; leg < 2; ++leg) {
    const float up = lift[leg] + sink[leg];
    q[3 * leg + 1] = h0 + cr - x[leg] + up + walk_lean * PI / 180.0f;
    q[3 * leg + 2] = k0 - cr + x[leg] - up;
  }
  q[0] = q[3] = roll + walk_side * PI / 180.0f;
  q[0] += step_splay_deg * PI / 180.0f;            // + is outwards on the left hip roll,
  q[3] -= step_splay_deg * PI / 180.0f;            // - on the right one
  return done;
}

void step_control(float dt) {
  float q[6];
  const float er_deg = att_valid ? (att_roll - g_ref_roll) * 180.0f / PI : 0.0f;
  const float rr_deg = rate_roll * 180.0f / PI;
  // The rock's size is |roll| at each turning point (the filtered rate changes sign), averaged over the
  // last few; with no turning point for 1.5 s it fades. With a rock mode on, the roll gain chases the
  // target size both ways (8 Oct: keep it rocking): too small, more gain - the gain is what keeps
  // the rock going in the first place; too big, less. 0.3x .. 2.5x the step's own gain.
  static float prev_rate = 0.0f, since_peak = 0.0f;
  since_peak += dt;
  // 11 Oct: |roll| at a turning point counted a robot standing still but tilted 18 deg to one side as an 18 deg
  // rock, so the pump braked and the rock stopped "by itself". The size is now half the distance between the
  // last high and the last low turning point: a tilt with no swing is no rock.
  if ((g_rate_roll > 0.0f) != (prev_rate > 0.0f)) {
    if (prev_rate > 0.0f) step_last_hi = er_deg; else step_last_lo = er_deg;
    const float half = 0.5f * fabsf(step_last_hi - step_last_lo);
    rock_peak = 0.5f * rock_peak + 0.5f * half;
    since_peak = 0.0f;
  }
  prev_rate = g_rate_roll;
  if (since_peak > 1.5f) rock_peak *= expf(-dt / 1.0f);
  // 10 Oct: wanting 6 deg, the rock reached 15-20 deg in a second and 30-37 later, hip rolls slammed at +-16:
  // the angle feedback (1.45 rad/rad) is the real pump, and the speed pump alone could not brake it. Now one
  // factor scales the angle feedback too, and it moves fast: from 1 to 0 in about 1.5 s when the rock is three
  // times too big, back up as it shrinks. Past twice the wanted size the angle feedback turns into a brake.
  // 10 Oct, the verdict: the first version (9 Oct evening, walk_20261010_105234) was the best - the
  // angle feedback at full strength and only the speed pump adapting - it just grew past +-30 and fell.
  // So that, plus one limit: while the rock is bigger than wanted, the angle feedback is cut to wanted/actual
  // (never below 0.3). Below the wanted size nothing changes. No centring.
  rock_gain = (rock_mode > 0 && rock_peak > rock_amp_deg) ? max(0.3f, rock_amp_deg / rock_peak) : 1.0f;
  if (rock_mode > 0) rock_pump = constrain(rock_pump + 0.01f * (rock_amp_deg - rock_peak) * dt, -0.08f, 0.20f);
  if (rock_mode > 0 && rock_style >= 1 && !(step_shuffle && step_go <= 0.0f)) {   // frozen at rest (the drive page)
    rock_hip_amp = constrain(rock_hip_amp + 1.5f * (rock_amp_deg - rock_peak) * dt, -6.0f, 12.0f);
    if (fabsf(er_deg) > 2.0f * rock_amp_deg) rock_hip_amp = min(rock_hip_amp, -3.0f);   // far too big: brake now
  }
  const float rock_brake = 0.0f;
  // It still fell to the right: the rock ran -5..+18 deg, centred 4-6 deg right, until one swing went over.
  // The centre (2 s average of the roll) is pulled back to zero with both hip rolls (+ moves the trunk left).
  // 12 Oct: with a fixed reference (0 deg) it tended to fall right at the start: it stands at ~8 deg
  // when Avanti is pressed, and the whole 8 deg error hit at once. The reference now starts where it stands and
  // slides to the fixed one at 3 deg/s.
  if (fabsf(step_pitch_ref_deg) < 60.0f) {
    const float goal = step_pitch_ref_deg * PI / 180.0f, v = 3.0f * PI / 180.0f * dt;
    g_ref_pitch += constrain(goal - g_ref_pitch, -v, v);
  }
  {
    const float ep_now = att_valid ? (att_pitch - g_ref_pitch) * 180.0f / PI : 0.0f;
    step_pitch_center += (ep_now - step_pitch_center) * min(1.0f, dt / 2.0f);
    pitch_trim_deg = constrain(pitch_trim_deg + 0.8f * step_pitch_center * dt, -15.0f, 15.0f);
  }
  // 13 Oct, steering by leaning (an experiment): a longer stride on one side did a few deg/s at most, often
  // the wrong way, against a drift of 2..13 deg/s that follows where the rock sits - leaning right, it turned
  // right. So steering now moves the rock's centre: + step_turn (turn left) centres the rock 6 deg to the left
  // at full steer, and the robot stands more on its left foot. Sliding only; the stride stays symmetric.
  // First test (walk_20261013_115729): steering right +15 deg/s, steering left stopped the usual right drift
  // (-1..+4 deg/s) but did not turn it left - the robot's own drift is to its right. 6 -> 9 deg at full steer.
  // 13 Oct 15:26 (walk_20261013_152546), steering with the legs: leaning LEFT turned it RIGHT (+4..+21 deg/s),
  // leaning right held it straight (-1..0). Whatever the mechanism (the loaded foot pivots the other way), the
  // sign is the measured one now: turning left (+) leans right.
  // Left needs more (154410: left +6.5 deg/s, still drifting right; right +15..+17): 14 deg leaning right to turn
  // left, 9 leaning left to turn right.
  const float steer_frac = constrain(step_turn / 0.10f, -1.0f, 1.0f);
  // 13 Oct 16:00: right and left still swapped. Read again with + yaw = the robot's LEFT (the 10 Oct
  // convention - "alla sua destra" earlier was most likely seen from the front), every leg-lean run agrees:
  // leaning left turned it left (152546: +4..+21 deg/s), leaning right less so (154410: +6.5 vs +17), and the
  // robot drifts to its left. So: + step_turn (turn left) leans LEFT; turning right needs the bigger lean.
  const float steer_roll_want = step_shuffle ? -(steer_frac > 0.0f ? 9.0f : 14.0f) * steer_frac : 0.0f;
  // 13 Oct 16:13 (walk_20261013_161232_guida): from a 14 deg lean one way to 9 the other in ~0.7 s it went over.
  // The lean moves at most 12 deg/s.
  step_lean_now += constrain(steer_roll_want - step_lean_now, -12.0f * dt, 12.0f * dt);
  const float steer_roll_deg = step_lean_now;
  if (rock_mode > 0) {
    step_rock_center += (er_deg - steer_roll_deg - step_rock_center) * min(1.0f, dt / 2.0f);
    // style 2 (12 Oct): every rock leaned right, -3..+17 deg from rest; the centring gets 10 deg instead of 6
    // 13 Oct 11:45: with the sign right, the centring sat at its 6 deg limit in every run and the rock still leaned
    // right (0..+18 around a rest of ~2): the robot stands on its right foot more and pivots on it - it turned right
    // 7..13 deg/s whatever the steering asked (walk_20261013_113822, 114243). 10 deg for the plain pump too.
    // ...and the centring itself ran to -11 and stayed there - the rock's mean sits right of the reference
    // whatever the trim does - leaving the pump too little travel: the rock shrank to 10 deg. Back to 6 when
    // sliding (the legs steer now); 10 for the staggered rock.
    // 13 Oct 19:05 (walk_20261013_190510_guida): sliding, the centring sits at its limit in every run (-6) and that
    // offset plus the pump's 12 and the splay runs the hip rolls into the -16 stop - the right one hard - so the
    // rock is clipped (17-20 deg). The rock's centre just is right of the reference; the trim cannot move it, it
    // only costs travel. 2 deg when sliding.
    const float tmax = step_shuffle ? 2.0f : 10.0f;
    // 12 Oct 23:05: the sign was backwards. "+ moves the trunk left" was a guess; the runs say a + common hip
    // roll moves it RIGHT: trim ~+3 rocked around +9.5 deg (225423), trim ~-10 around 0 (230359) - so a rock
    // leaning right pushed itself further right (every lopsided rock since 10 Oct), and with 10 deg of room
    // instead of 6 it went over to lean left ("tutto inclinato dall'altra parte"). Now it pulls back.
    rock_trim_deg = constrain(rock_trim_deg - 0.4f * step_rock_center * dt, -tmax, tmax);
  }
  // heading
  yaw_f += (yaw_rad - yaw_f) * min(1.0f, dt / YAW_TAU);
  {
    float want = -walk_turn * 0.10f;                             // W: + turns right
    // 13 Oct: + yaw is the robot turning to ITS RIGHT. It was seen to veer right while the yaw climbed
    // (walk_20261013_100103, +3 deg/s; 100646, +13 deg/s with the hold off). "-yaw is right" (10 Oct) was read
    // off the log, never seen - so the hold pushed the drift along. Sliding, + step_turn makes the right foot's
    // swing the bigger one, which turns it left: a yaw above the target asks for +.
    // + yaw is left (see steer_roll_deg below): a heading left of the target asks for a right turn (-)
    // Sliding, the hold waits until the stride is up (the push that starts the rock jerks the heading 20-30 deg
    // and the hold, already on, answered it with a full lean - 161232) and keeps its target fresh until then.
    if (yaw_hold && step_shuffle && step_go_now < 0.3f) yaw_target = yaw_f;
    else if (yaw_hold) want += constrain(0.8f * (yaw_target - yaw_f), -0.10f, 0.10f);
    want = constrain(want, -0.12f, 0.12f);
    // back towards straight twice as fast as into a turn (13 Oct: letting go of an arrow should stop it soon)
    const float tr = fabsf(want) < fabsf(step_turn) ? 0.4f : 0.2f;
    step_turn += constrain(want - step_turn, -tr * dt, tr * dt);
  }
  // It starts by itself too: standing still for 2 s with a rock mode on, it gives itself the little push a
  // hand would - both hip rolls swing out and back once over 0.3 s - and the gain takes it from there.
  static float still_for = 0.0f, kick_t = -1.0f;
  float kick = 0.0f;
  // 13 Oct, the drive page: when I let go it must really stop, not rock. Sliding with 'f 0' is now a rest: the
  // pump brakes the rock out, no self-start push, nothing adapts. Pressed again ('f' > 0) from a still robot it
  // gives itself the push at once, and the stride waits until the rock is there (step_go_now below).
  const bool resting = step_shuffle && step_go <= 0.0f;
  static bool was_resting = false;
  if (was_resting && !resting && rock_peak < 2.0f) still_for = 2.0f;   // the push, now
  was_resting = resting;
  if (rock_mode > 0 && !step_moving && !resting && rock_peak < 1.0f) still_for += dt;
  else if (resting || rock_peak >= 1.0f) still_for = 0.0f;
  if (still_for > 2.0f && kick_t < 0.0f) { kick_t = 0.0f; still_for = 0.0f; out.println(F("rock: giving it a push")); }
  if (kick_t >= 0.0f) {
    kick = 0.15f * sinf(PI * kick_t / 0.3f);
    kick_t += dt;
    if (kick_t > 0.3f) kick_t = -1.0f;
  }
  if (step_shuffle) {
    // + roll leans right: the left foot is free and goes forward
    // measured from the rock's own centre (12 Oct 23:15): the rock sat ~4 deg right of the reference, so the feet
    // sat off-centre too (hips -6..+4 instead of +-5) - one foot ahead on average, and it turned (+40 deg in 20 s)
    // 13 Oct 10:05: scaled by the WANTED rock (19 deg) the feet used under half their stride and a fifth of their
    // lift - the real rock is +-8..9 (walk_20261013_100103: hips +-0.8, knees +-1.5: "alza poco i
    // piedi"). Scaled by the rock it actually makes now (its size, followed over ~2 s), a full lean is a full step.
    static float slide_amp = 8.0f;
    slide_amp += (max(3.0f, rock_peak) - slide_amp) * min(1.0f, dt / 2.0f);
    const float u = constrain((er_deg - steer_roll_deg - step_rock_center) / slide_amp, -1.0f, 1.0f);   // around the steered centre
    // starts at 0.8/s (~1.2 s), stops at 3/s (~0.3 s) - stop sooner when I let go
    // up only once the rock is going (half the wanted size or 4 deg), so the feet do not slide on a still robot
    const float go_to = rock_peak > min(4.0f, 0.5f * rock_amp_deg) ? step_go : min(step_go, step_go_now);
    step_go_now += constrain(go_to - step_go_now, -3.0f * dt, 0.8f * dt);
    // 13 Oct 19:02: the toe catches the floor and it goes over forwards. The foot slid forward in step
    // with the lean (x ~ u) but only rose with its square, so near the middle it moved forward still on the
    // ground. Now it slides with u*|u| too: forward mostly while it is up (same stride at a full lean).
    const float want = stp[2] * u * fabsf(u) * step_go_now;
    step_shuffle_x += (want - step_shuffle_x) * min(1.0f, dt / 0.05f);
    // a step for the page's counter: each lean that goes past half way to the other side than the last one
    static int slide_side = 0;
    if (u > 0.5f && slide_side != 1) { slide_side = 1; ++step_count; }
    else if (u < -0.5f && slide_side != -1) { slide_side = -1; ++step_count; }
    // 13 Oct 10:20: "fa fatica a staccare il piede" - knees moved only 8 deg (walk_20261013_101458);
    // 0.4 of the lift (~14 deg) fell forward twice in 25 s (walk_20261013_102034, pitch +70..+80 while a foot was
    // up) - back to the quarter that walked 84 s
    // 13 Oct: lift the feet a bit better - 0.25 -> 0.32 of the step's lift (0.4 fell forwards twice);
    // none at rest
    const float lift_k = 0.32f * stp[3] * step_go_now;
    const float up[2] = {u > 0.0f ? lift_k * u * u : 0.0f, u < 0.0f ? lift_k * u * u : 0.0f};
    for (int k = 0; k < 2; ++k) step_shuffle_lift[k] += (up[k] - step_shuffle_lift[k]) * min(1.0f, dt / 0.05f);
    step_armed = false;
  } else { step_shuffle_x = 0.0f; step_shuffle_lift[0] = step_shuffle_lift[1] = 0.0f; }
  if (!step_shuffle && !step_moving && rock_mode > 0 && (step_armed || rock_mode == 2) && att_valid) {
    // + roll is leaning right; the left foot in front wants the trunk over to the left
    // in place: whichever side the trunk is on, the other foot goes up - never the same foot twice running
    const float toward = step_inplace ? (step_last_lift == 1 ? -1.0f : step_last_lift == 0 ? 1.0f : (er_deg < 0.0f ? -1.0f : 1.0f))
                                      : (step_front == 0 ? -1.0f : 1.0f);
    // Angle and its rate: where the trunk will be in rock_lead_s, so the foot is
    // already leaving when the weight arrives; a slow drift that would never get there does not count, and
    // the rock must have been big enough recently to carry the weight over at all.
    const float rrf_deg = g_rate_roll * 180.0f / PI;
    const float ahead = er_deg + rock_lead_s * rrf_deg;
    // 9 Oct: the prediction alone fired with the trunk still on the WRONG side (+3.3 deg right, -63 deg/s,
    // left foot in front): fast enough to be "there" in 0.12 s, but the weight was on the foot that lifted,
    // and it fell. The trunk must already be past half the threshold on the front foot's side as well.
    // 12 Oct 22:40: the rock leans to the right only (-3..+17 deg from rest, every run, the 76 s one too), so on
    // the left the trunk never gets past ~3 deg - and half the threshold (2 deg) plus the speed fired the step as
    // the trunk swept through the middle at -114 deg/s: the right foot went up still loaded, and it fell
    // (walk_20261012_224054). The trunk must now be past the whole threshold itself - no weight, no step.
    if (toward * er_deg > rock_thr_deg && toward * ahead > rock_thr_deg && toward * rrf_deg > 5.0f &&
        rock_peak > 0.6f * rock_thr_deg) {
      step_armed = false; step_moving = true;
      step_clock = stp[6];                       // the rock has already moved the weight: straight to the swing
      step_skip_shift = true;
      step_lifting = toward < 0.0f ? 1 : 0;      // trunk over to the left: the right foot is free
      if (step_inplace) out.printf("lift %d on the rock: %+.1f deg at %+.0f deg/s, %s foot up\n", step_count + 1, er_deg, rrf_deg, step_lifting == 0 ? "left" : "right");
      else out.printf("step %d on the rock: %+.1f deg at %+.0f deg/s (%+.1f ahead), %s foot steps\n", step_count + 1, er_deg, rrf_deg, ahead, step_front == 0 ? "right" : "left");
    }
  }
  if (step_moving) {
    step_clock += dt;
    if (step_pose(true, step_clock, step_front, q)) {
      step_moving = false; ++step_count;
      if (step_inplace) step_last_lift = step_lifting;
      else { step_front = 1 - step_front;
             out.printf("step %d done, %s foot in front - 'm' for the next\n", step_count, step_front == 0 ? "left" : "right"); }
    }
  } else {
    step_pose(false, 0.0f, step_front, q);
  }
  if (att_valid) {
    const float er = att_roll - g_ref_roll, ep = att_pitch - g_ref_pitch;
    if (rock_mode > 0 && rock_style >= 1) {
      // style 2, the soft pump (12 Oct: jerky, and very fast moves when lifted): full push only past
      // 40 deg/s instead of 15, so the hip rolls swing over instead of snapping +-12 deg at every turn, and gyro
      // noise in the air no longer flips them
      const float sat_dps = rock_style == 2 ? 40.0f : 15.0f;
      float u = -rock_hip_amp * PI / 180.0f * constrain(g_rate_roll * 180.0f / PI / sat_dps, -1.0f, 1.0f);
      // 12 Oct 23:10 (walk_20261012_225423): with a foot in the air the pump kept pumping - the trunk, 4 deg over
      // the left foot at the lift, was driven back over to the right (+37 deg) onto the foot that was not there.
      // The rock is ~2.5 Hz here, a half swing ~0.2 s, shorter than the 0.3 s swing. While a foot is up the pump
      // turns into a brake: it opposes the roll, so the trunk stays over the standing foot.
      if (step_moving) u = fabsf(rock_hip_amp) * PI / 180.0f * constrain(g_rate_roll * 180.0f / PI / sat_dps, -1.0f, 1.0f);
      // rest: the pump simply off. A "brake" (the pump's sign turned round) threw it over within a second of
      // stepping in, twice (161232) - the sign that damps is not the obvious one here either. It rocks out alone.
      if (step_shuffle && step_go <= 0.0f) u = 0.0f;
      q[0] += u; q[3] += u;
      if (rock_angle_share > 0.0f && !step_moving) {
        // the 10 Oct law, the same sign (q -= gain * stp[9] * roll error), a share of it
        // 13 Oct 16:46 (walk_20261013_164638_guida): at 30% from a standing start the rock went from 0 to +-13 deg
        // in a second and the pitch to -41 - over backwards. Sliding, the share now comes in with the stride
        // (step_go_now: none until the pump's own rock is there, then ~1.2 s), and goes at once on a stop.
        const float share = step_shuffle ? rock_angle_share * step_go_now : rock_angle_share;
        const float a = share * rock_gain * stp[9] * er;
        q[0] -= a; q[3] -= a;
      }
    } else {
      q[0] -= (rock_gain + rock_brake) * stp[9] * er; q[3] -= (rock_gain + rock_brake) * stp[9] * er;
    }
    q[0] += kick; q[3] += kick;
    q[0] += rock_trim_deg * PI / 180.0f; q[3] += rock_trim_deg * PI / 180.0f;
    q[1] += pitch_trim_deg * PI / 180.0f; q[4] += pitch_trim_deg * PI / 180.0f;
    if (rock_mode > 0 && rock_style == 0) {
      const float drive = -rock_pump * constrain(g_rate_roll * 180.0f / PI / 15.0f, -1.0f, 1.0f);
      q[0] += drive; q[3] += drive;
    }
    // 13 Oct 15:18: leaning with the hip rolls ate the pump's travel - centring -11.5 plus the pump's 12 ran the
    // left hip roll into its -16 stop, the swing was clipped on one side and the rock died (span 22 -> 10 deg,
    // walk_20261013_151746: "dopo un po si smorza"). The lean now comes from the legs: the leg on the
    // side to lean to crouches deeper, the other stands taller, by about the lean wanted (~10 mm over the
    // ~60 mm between the feet is ~10 deg). The centring target still moves with it, so it does not fight it.
    if (step_shuffle && steer_roll_deg != 0.0f) {
      const float d = -steer_roll_deg * PI / 180.0f;   // + : lean left, left leg shorter
      q[1] += d; q[2] -= d; q[4] -= d; q[5] += d;
    }
    q[1] -= rock_pdamp * g_rate_pitch; q[4] -= rock_pdamp * g_rate_pitch;
    q[1] -= stp[10] * ep; q[4] -= stp[10] * ep;
  }
  const float lim = STEP_SLEW_RAD_S * dt;
  for (int j = 0; j < 6; ++j) {
    const float w = constrain(q[j], JOINT_MIN_RAD[j], JOINT_MAX_RAD[j]);
    // style 2: hip rolls at most 3 rad/s (24 deg in 0.14 s), the rest as before
    const float l = (rock_style == 2 && (j == 0 || j == 3)) ? 3.0f * dt : lim;
    command_rad[j] += constrain(w - command_rad[j], -l, l);
  }
  write_all();
}

void control_step(float dt) {
  if (mode == STEP) { step_control(dt); return; }
  if (mode == WAKE) {
    const float t = wake_clock;
    wake_clock += dt;
    ++wake_tick;
    bool done = true;
    for (int k = 0; k < 6; ++k) {
      const int j = WAKE_ORDER[k];
      const float local = t - k * WAKE_STAGGER_S;
      if (local < 0.0f) { ledcWrite(cal[j].pin, 0); done = false; continue; }
      const float u = min(1.0f, local / WAKE_RAMP_S);
      if (u < 1.0f) done = false;
      const int every = max(1, (int)lroundf(WAKE_SPARSE - (WAKE_SPARSE - 1) * u));
      if (wake_tick % every == 0) write_joint(j, command_rad[j]);
      else ledcWrite(cal[j].pin, 0);
    }
    if (done) {
      mode = HOLD;
      write_all();
      out.println(F("awake - holding the crouch"));
    }
    return;
  }

  if (mode == STAND) {
    // Evaluate at the clock BEFORE advancing it: standup9 commands t = i*DT on tick i, starting
    // at 0. Advancing first ran the whole move a tick late, 20 ms against a first segment of 110.
    const float t = stand_clock;
    stand_clock += dt;
    float want[6];
    if (t >= stand_edge[4]) {
      for (int j = 0; j < 6; ++j) want[j] = stand_way[4][j];
    } else {
      int k = 0;
      while (k < 3 && t >= stand_edge[k + 1]) ++k;
      float u = (t - stand_edge[k]) / max(1e-6f, stand_edge[k + 1] - stand_edge[k]);
      u = u * u * (3.0f - 2.0f * u);                     // smoothstep, as in standup9.command
      for (int j = 0; j < 6; ++j)
        want[j] = stand_way[k][j] + (stand_way[k + 1][j] - stand_way[k][j]) * u;
    }
    const float st = STANDUP_SLEW_RAD_S * stand_scale * dt;
    for (int j = 0; j < 6; ++j) {
      const float w = constrain(want[j], JOINT_MIN_RAD[j], JOINT_MAX_RAD[j]);
      command_rad[j] += constrain(w - command_rad[j], -st, st);
    }
    write_all();
    if (t >= stand_edge[4] + 1.0f) {                    // a second of hold, then plain HOLD
      mode = HOLD;
      out.println(F("stand-up finished"));
    }
    return;
  }

  if (mode == ROCK) {
    rock_clock += dt;
    const float r = rock_deg * PI / 180.0f * sinf(2.0f * PI * rock_clock / max(0.2f, rock_period));
    const float c = pose_crouch_deg * PI / 180.0f;
    const float d = rock_diff * r;               // one leg shorter than the other, same sign as the lean
    const float want[6] = {r, c + d, -(c + d), r, c - d, -(c - d)};
    const float step = 3.0f * dt;                // the servos' own rate, as everywhere else
    for (int j = 0; j < 6; ++j) {
      const float w = constrain(want[j], JOINT_MIN_RAD[j], JOINT_MAX_RAD[j]);
      command_rad[j] += constrain(w - command_rad[j], -step, step);
    }
    write_all();
    return;
  }

  if (mode != WALK) return;

  // Positive yaw is to the robot's own left, positive steering turns it right, so a heading that
  // has run left of target asks for positive steering. Rate-limited like everything else that
  // reaches a servo: a step of asymmetry inside one stride would just trip it.
  if (yaw_hold) {
    yaw_f += (yaw_rad - yaw_f) * min(1.0f, dt / YAW_TAU);
    const float err = yaw_f - yaw_target;
    if (yaw_turning && fabsf(err) < 10.0f * PI / 180.0f) yaw_turning = false;   // arrived
    const float cap = yaw_turning ? YAW_CAP_TURN : YAW_CAP;
    const float want = constrain(yaw_kp * err + yaw_kd * rate_yaw, -cap, cap);
    turn_corr += constrain(want - turn_corr, -0.3f * dt, 0.3f * dt);
  } else if (turn_corr != 0.0f) {
    turn_corr -= constrain(turn_corr, -0.5f * dt, 0.5f * dt);
  }
  gait_clock = fmodf(gait_clock + dt * speed, gait_period());
  float target[6];
  gait_targets(gait_clock, target);

  // The same rate limit the search ran under. Without it the servos chase every step of the
  // pattern at full speed and the robot walks differently from the one that was tested.
  const float step = gait_slew() * dt;
  for (int j = 0; j < 6; ++j) {
    const float want = constrain(target[j], JOINT_MIN_RAD[j], JOINT_MAX_RAD[j]);
    command_rad[j] += constrain(want - command_rad[j], -step, step);
  }
  write_all();
}


void log_tick() {
  if (!log_on) return;
  const float period = gait_period();
  int sat = 0;
  for (int j = 0; j < 6; ++j) {
    const float o = joint_out(j);
    if (o <= JOINT_MIN_RAD[j] + 1e-4f || o >= JOINT_MAX_RAD[j] - 1e-4f) sat |= 1 << j;
  }
  if (fabsf(corr_roll) >= FB_CLAMP_ROLL - 1e-4f) sat |= 1 << 6;
  if (fabsf(corr_pitch) >= FB_CLAMP_PITCH - 1e-4f) sat |= 1 << 7;
  const float D = 10.0f * 180.0f / PI;                 // radians to tenths of a degree
  out.printf("D %lu %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
             (unsigned long)(millis() - log_t0),
             (int)lroundf(1000.0f * gait_clock / max(1e-3f, period)),
             mode == LIMP ? 0 : mode == HOLD ? 1 : mode == WALK ? 2 : mode == WAKE ? 3 : mode == ROCK ? 5 : mode == STEP ? 6 : 4,
             (int)lroundf(joint_out(0) * D), (int)lroundf(joint_out(1) * D),
             (int)lroundf(joint_out(2) * D), (int)lroundf(joint_out(3) * D),
             (int)lroundf(joint_out(4) * D), (int)lroundf(joint_out(5) * D),
             (int)lroundf(att_roll * D), (int)lroundf(att_pitch * D),
             (int)lroundf(acc_roll * D), (int)lroundf(acc_pitch * D),
             (int)lroundf(rate_roll * D), (int)lroundf(rate_pitch * D),
             (int)lroundf(rate_yaw * D), (int)lroundf(yaw_rad * D), (int)lroundf((mode == STEP ? step_turn : turn_corr) * 1000),   // step mode: its steering (13 Oct)
             (int)lroundf(corr_roll * D), (int)lroundf(corr_pitch * D),
             (int)lroundf(corr_diff * D), sat);
  ++log_n;
}
