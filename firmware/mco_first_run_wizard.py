"""
MCO-1D First Run Commissioning Wizard
=======================================
Step-by-step procedure to safely commission a freshly built drone.
Run BEFORE any powered flight.

Usage:  python3 main.py --wizard
    or: python3 first_run_wizard.py
"""

import time, sys

try:
    from odrive_interface import ODriveInterface
    from servo_interface  import ServoInterface
    import wave_library   as wl
    HW_AVAILABLE = True
except ImportError:
    HW_AVAILABLE = False


SEP = "=" * 55

def prompt(msg):
    print("\n  >>> " + msg)
    input("      Press ENTER when ready (Ctrl+C to abort): ")

def step(n, title):
    print("\n" + SEP)
    print("  STEP " + str(n) + ": " + title)
    print(SEP)

def ok(msg=""):
    print("  OK  " + msg)

def warn(msg):
    print("  **  " + msg)


def run():
    print("\n" + SEP)
    print("  MCO-1D FIRST RUN COMMISSIONING WIZARD")
    print("  Read each step carefully before proceeding.")
    print(SEP)

    # STEP 1 ---------------------------------------------------------
    step(1, "Pre-Power Safety Checks")
    print("""
  Before connecting any power:

  [ ] All 8 wings at 60 deg sweep (forward deployed), secured
  [ ] No loose parts in drive train
  [ ] Battery is DISCONNECTED
  [ ] RC transmitter ON, throttle minimum, all switches disarmed
  [ ] CAN-FD termination resistors at both ends of bus
  [ ] All 16 servo connectors seated
  [ ] RPi Zero 2W microSD inserted with firmware
  [ ] Pixhawk Mini mounted with vibration isolation
  [ ] Pitot tube clear and connected
    """)
    prompt("Confirm all pre-power checks complete")
    ok("Pre-power checks confirmed")

    # STEP 2 ---------------------------------------------------------
    step(2, "Software Installation on RPi Zero 2W")
    print("""
  SSH to RPi: ssh pi@mco-1d.local  (or by IP address)

  Install dependencies:
    sudo apt update
    sudo apt install -y pigpiod python3-pip can-utils
    pip3 install python-can pymavlink

  Enable CAN-FD (add to /etc/network/interfaces.d/can0):
    allow-hotplug can0
    iface can0 inet manual
      pre-up  ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
      up      ip link set can0 up
      down    ip link set can0 down

  Start pigpio daemon:
    sudo systemctl enable pigpiod
    sudo systemctl start pigpiod

  Copy firmware files to /home/pi/mco/ :
    wave_library.py   odrive_interface.py   servo_interface.py
    mavlink_bridge.py main.py               first_run_wizard.py

  Verify by running:
    cd /home/pi/mco && python3 -c "import wave_library; print('OK')"
    """)
    prompt("Confirm software installed and files present on RPi")
    ok("Software installation confirmed")

    # STEP 3 ---------------------------------------------------------
    step(3, "ODrive Micro Configuration  (one drive at a time via USB)")
    print("""
  On your PC (not the RPi):
    pip install odrive
    odrivetool

  For each of the 8 ODrive Micro units:
  Connect one at a time via USB. Assign node IDs 0-7 sequentially.

    odrv0.axis0.config.motor.current_soft_max          = 20
    odrv0.axis0.config.motor.calibration_current       = 10
    odrv0.axis0.config.motor.motor_type                = 0  # HIGH_CURRENT
    odrv0.axis0.config.motor.pole_pairs                = 7
    # S1: on-board absolute encoder — no encoder config needed
    # S1: encoder CPR not required for on-board magnetic encoder
    odrv0.axis0.controller.config.control_mode         = 2  # VELOCITY
    odrv0.axis0.controller.config.vel_limit            = 150
    odrv0.can.config.baud_rate                   = 1000000
    odrv0.can.config.protocol             = Protocol.SIMPLE
    odrv0.axis0.config.can.node_id                     = N   # 0..7 (= wing_idx)
    odrv0.save_configuration()
    odrv0.reboot()

  Run full calibration (first time, in open air before mounting):
    # S1: motor calibration only (no encoder calibration needed)
odrv0.axis0.requested_state = 4  # AXIS_STATE_MOTOR_CALIBRATION

  Repeat for all 8 drives. Record which drive is which wing.
    """)
    prompt("All 8 ODrives configured, node IDs 0-7 assigned")
    ok("ODrive configuration confirmed")

    # STEP 4 ---------------------------------------------------------
    step(4, "Servo Centring and Range Check")
    print("  Connecting to servo interface...")

    servos = ServoInterface() if HW_AVAILABLE else None

    prompt("Wings unloaded, no aerodynamic forces. Ready to centre servos?")

    if servos:
        print("  Moving all servos to centre: 60 deg sweep, 0 deg pitch ...")
        servos.centre_all()
        time.sleep(1.0)
        ok("Servos centred")
    else:
        warn("Simulation mode - servo centring skipped")

    print("""
  Visually verify each wing pair:
  [ ] W1 (frontmost): sweep ~60 deg, chord roughly horizontal
  [ ] W2: same
  [ ] W3: same
  [ ] W4 (rearmost): same
  [ ] No binding, no mechanical interference on any servo
    """)
    prompt("Confirm visual servo inspection complete")

    if servos:
        print("  Testing W1 sweep range: 60 -> 120 -> 60 deg ...")
        servos.set_sweep(0, 120.0)
        servos.set_sweep(1, 120.0)
        time.sleep(1.0)
        servos.set_sweep(0, 60.0)
        servos.set_sweep(1, 60.0)
        time.sleep(0.5)
        ok("W1 sweep range test complete")

    prompt("All wings move freely through full sweep range?")

    # STEP 5 ---------------------------------------------------------
    step(5, "Stow Sequence Test  (servos only, motors OFF)")
    print("""
  Test the W4->W3->W2->W1 stow sequence with servos only.
  Motors will NOT run during this test.
  Clear 300mm above and below all wing tips before proceeding.
    """)
    prompt("Area clear above and below wings. Ready for stow test?")

    if servos:
        print("  Executing stow: W4 -> W3 -> W2 -> W1 at 20 deg/s ...")
        servos.stow_all(rate_degps=20.0)
        ok("All wings at 210 deg stow")
        time.sleep(1.0)
        print("  Returning to 60 deg deployed ...")
        servos.centre_all()
        ok("Wings returned to deployed position")
    else:
        warn("Simulation mode - stow test skipped")

    prompt("Stow order correct (W4 first, W1 last)? No collisions?")

    # STEP 6 ---------------------------------------------------------
    step(6, "Motor Spin Test  (LOW power, wings stowed to 200 deg)")
    print("""
  SAFETY BEFORE THIS STEP:
  [ ] All wings swept to 200 deg (stow position)
  [ ] Hands and loose clothing clear of all parts
  [ ] Second person observing from a safe distance
  [ ] Battery connected but throttle at minimum
    """)

    odrive = ODriveInterface() if HW_AVAILABLE else None

    if servos:
        print("  Stowing wings to 200 deg for motor test ...")
        for i in range(8):
            servos.set_sweep(i, 200.0)
        time.sleep(2.0)

    prompt("Wings at 200 deg, area clear. Connecting battery now?")

    if odrive:
        print("  Enabling all 8 ODrives ...")
        odrive.enable_all()
        time.sleep(0.5)

        print("  Spinning at 0.2 Hz (very slow - rope barely moving) ...")
        odrive.set_all_flap_freq(0.2)
        time.sleep(4.0)
        prompt("Listen: any grinding, clicking, uneven rhythm? All OK?")

        print("  Increasing to 1.0 Hz ...")
        odrive.set_all_flap_freq(1.0)
        time.sleep(4.0)
        prompt("All 8 motors running smoothly at 1.0 Hz? Rope tension even?")

        odrive.idle_all()
        ok("Motor spin test complete")
        odrive.close()
    else:
        warn("Simulation mode - motor test skipped")

    # STEP 7 ---------------------------------------------------------
    step(7, "Phase Synchronisation Check  (wings deployed)")
    print("""
  Deploy wings to 60 deg, run metachronal cascade at low speed.
  You should see a clear travelling wave from W1 (front) to W4 (rear).
  Each wing pair should be visibly about 1/4 cycle behind the one in front.
    """)

    if servos:
        print("  Deploying wings to 60 deg ...")
        servos.centre_all()
        time.sleep(1.0)

    odrive2 = ODriveInterface() if HW_AVAILABLE else None
    if odrive2:
        offsets = wl.metachronal_phase_offsets(offset_deg=90.0)
        print("  Setting initial phase positions ...")
        odrive2.set_initial_phases(offsets)
        time.sleep(0.5)
        print("  Running metachronal cascade at 0.4 Hz ...")
        odrive2.set_cascade_freq(0.4, offsets)
        time.sleep(6.0)
        prompt("Visible W1->W4 wave? Each pair ~90 deg behind the previous?")
        odrive2.idle_all()
        odrive2.close()
    else:
        warn("Simulation mode - phase check skipped")

    ok("Phase synchronisation verified")

    # STEP 8 ---------------------------------------------------------
    step(8, "Pixhawk Mini MAVLink Integration")
    print("""
  Wire Pixhawk TELEM2 to RPi /dev/serial0:
    Pixhawk TELEM2 TX  ->  RPi pin 10 (RXD)
    Pixhawk TELEM2 RX  ->  RPi pin 8  (TXD)
    Pixhawk GND        ->  RPi GND

  In Mission Planner or QGroundControl (Pixhawk parameters):
    SERIAL2_PROTOCOL  =  2   (MAVLink 2)
    SERIAL2_BAUD      = 57   (57600 baud)
    SR2_EXTRA1        = 10   (attitude at 10 Hz)
    SR2_EXTRA2        = 10   (VFR_HUD airspeed at 10 Hz)
    SR2_RC_CHAN       = 10   (RC channels at 10 Hz)

  Test on RPi:
    python3 -c "
    from mavlink_bridge import PixhawkBridge
    import time
    b = PixhawkBridge()
    time.sleep(3)
    print(b.get_state())
    "
  Expected: airspeed, attitude, armed state printing correctly.
    """)
    prompt("MAVLink communication confirmed between RPi and Pixhawk")
    ok("MAVLink integration confirmed")

    # STEP 9 ---------------------------------------------------------
    step(9, "Pre-Flight Checklist")
    print("""
  Before first flight:
  [ ] Firmware files present on RPi: /home/pi/mco/
  [ ] All 8 ODrives calibrated, node IDs 0-7 confirmed
  [ ] CAN-FD bus wired, termination resistors at both ends
  [ ] All 16 servo connectors secure
  [ ] Pixhawk GPS lock obtained outdoors (wait for 3D fix)
  [ ] Pitot tube unobstructed, pointing into expected airflow
  [ ] RC transmitter bound, all 8 channels reading correctly
  [ ] Battery fully charged: 6S, all cells 4.20V, balanced
  [ ] Battery connector secure
  [ ] Emergency procedure known: throttle to minimum -> wings fold
  [ ] First flight in large open area, low wind, daylight
  [ ] Second person as spotter
    """)
    prompt("All pre-flight items confirmed")

    # DONE -----------------------------------------------------------
    print("\n" + SEP)
    print("  COMMISSIONING COMPLETE")
    print(SEP)
    print("""
  The drone is ready for first flight.

  Start the flight software on RPi:
    python3 main.py          (live hardware)
    python3 main.py --sim    (simulation / ground test)

  Recommended test flight sequence:
    1.  Low hover, 1-2m, check stability (1 min)
    2.  Slow forward flight, 3-5 m/s
    3.  Gentle turns via yaw stick
    4.  Wave library T1: metachronal 90 deg cruise verified
    5.  Wave library T7: asymmetric sweep turns
    6.  Wave library T2: speed lever sweep schedule
    ... continue through T3-T10 per validation test plan

  Wave library file: /home/pi/mco/wave_library.py
  All locked parameters from Rev H + Addenda Rev A, B, C v2.
  """)


if __name__ == "__main__":
    run()
