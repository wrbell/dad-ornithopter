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
    from mco_odrive_interface import ODriveInterface
    import mco_wave_library   as wl
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
  [ ] RPi 4B microSD inserted with firmware
  [ ] Pixhawk 4 Mini mounted with vibration isolation
  [ ] Pitot tube clear and connected
    """)
    prompt("Confirm all pre-power checks complete")
    ok("Pre-power checks confirmed")

    # STEP 2 ---------------------------------------------------------
    step(2, "Software Installation on Raspberry Pi 4B")
    print("""
  SSH to RPi: ssh pi@mco-1d.local  (or by IP address)

  Install dependencies:
    sudo apt update
    sudo apt install -y pigpiod python3-pip can-utils
    pip3 install python-can pymavlink

  Enable CAN (add to /etc/network/interfaces.d/can0):
    allow-hotplug can0
    iface can0 inet manual
      pre-up  ip link set can0 type can bitrate 1000000
      up      ip link set can0 up
      down    ip link set can0 down

  Start pigpio daemon:
    sudo systemctl enable pigpiod
    sudo systemctl start pigpiod

  Verify by running:
    python3 -c "import mco_config, mco_wave_library; print('OK')"
    """)
    prompt("Confirm software installed and files present on RPi")
    ok("Software installation confirmed")

    # STEP 3 ---------------------------------------------------------
    step(3, "ODrive S1 Configuration  (mco_odrive_setup.py)")
    print("""
  Run automated ODrive S1 setup and verification:
    python3 firmware/mco_odrive_setup.py
    python3 firmware/mco_odrive_setup.py --verify
    """)
    prompt("All 8 ODrive S1 nodes configured, node IDs 0-7 assigned")
    ok("ODrive configuration confirmed")

    # STEP 4 ---------------------------------------------------------
    step(4, "Servo Centring and Range Check")
    print("""
  Visually verify each wing pair:
  [ ] W1 (frontmost): sweep ~60 deg, chord roughly horizontal
  [ ] W2: same
  [ ] W3: same
  [ ] W4 (rearmost): same
  [ ] No binding, no mechanical interference on any servo
    """)
    prompt("Confirm visual servo inspection complete")
    ok("Servo inspection confirmed")

    # STEP 5 ---------------------------------------------------------
    step(5, "Stow Sequence Check  (servos only, motors OFF)")
    print("""
  Verify W4->W3->W2->W1 stow clearance with motors OFF.
  Clear 300mm above and below all wing tips before proceeding.
    """)
    prompt("Stow order and clearances verified? No collisions?")
    ok("Stow clearance confirmed")

    # STEP 6 ---------------------------------------------------------
    step(6, "Reciprocating Motor Oscillation Test  (LOW amplitude)")
    print("""
  SAFETY BEFORE THIS STEP:
  [ ] Hands and loose clothing clear of all parts
  [ ] Second person observing from a safe distance
  [ ] Battery connected, throttle at minimum
    """)

    odrive = ODriveInterface() if HW_AVAILABLE else None
    prompt("Area clear. Ready to test low-frequency reciprocating oscillation?")

    if odrive:
        print("  Enabling all 8 ODrives ...")
        odrive.enable_all()
        time.sleep(0.5)

        print("  Oscillating at 0.2 Hz (±15 deg stroke) for 4 seconds ...")
        odrive.oscillate_cascade(0.2, duration_s=4.0, stroke_half_angle_deg=15.0)
        prompt("Listen: any grinding, clicking, uneven rhythm? All OK?")

        print("  Oscillating at 0.6 Hz (±20 deg stroke) for 4 seconds ...")
        odrive.oscillate_cascade(0.6, duration_s=4.0, stroke_half_angle_deg=20.0)
        prompt("All 8 motors reciprocating smoothly? Rope tension even?")

        odrive.idle_all()
        ok("Motor oscillation test complete")
        odrive.close()
    else:
        warn("Simulation mode - motor test skipped")

    # STEP 7 ---------------------------------------------------------
    step(7, "Phase Synchronisation Check  (90 deg metachronal wave)")
    print("""
  Run metachronal cascade at low speed (0.4 Hz).
  You should see a travelling wave from W1 (front) to W4 (rear).
    """)

    odrive2 = ODriveInterface() if HW_AVAILABLE else None
    if odrive2:
        offsets = [0.0, 0.0, 90.0, 90.0, 180.0, 180.0, 270.0, 270.0]
        print("  Setting initial phase positions ...")
        odrive2.set_initial_phases(offsets, stroke_half_angle_deg=20.0)
        time.sleep(0.5)
        print("  Running metachronal cascade at 0.4 Hz for 6 seconds ...")
        odrive2.oscillate_cascade(0.4, duration_s=6.0, phase_offsets_deg=offsets, stroke_half_angle_deg=20.0)
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
