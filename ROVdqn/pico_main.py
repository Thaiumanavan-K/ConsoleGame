"""
╔══════════════════════════════════════════════════════════════════════════╗
║  pico_main.py — Raspberry Pi Pico Worker (MicroPython)                  ║
║  Project : Autonomous AI-Driven Plume Tracker (DQN + Triple-MCU)       ║
║  Role    : Sensor acquisition, motor execution, UART comms             ║
║  Packet  : "SCAN:d1,d2,d3,d4,d5|SMOKE:v|BATT:v\n"                      ║
╚══════════════════════════════════════════════════════════════════════════╝
"""

from machine import Pin, PWM, UART, ADC, Timer
import utime
import sys

# ─────────────────────── PIN CONFIGURATION ───────────────────────

# --- L298N Motor Driver ---
ENA = PWM(Pin(2))   # Left motor speed (PWM)
ENB = PWM(Pin(7))   # Right motor speed (PWM)
IN1 = Pin(3, Pin.OUT)  # Left motor direction
IN2 = Pin(4, Pin.OUT)  # Left motor direction
IN3 = Pin(5, Pin.OUT)  # Right motor direction
IN4 = Pin(6, Pin.OUT)  # Right motor direction

ENA.freq(1000)
ENB.freq(1000)

# --- HC-SR04 Ultrasonic ---
TRIG = Pin(14, Pin.OUT)
ECHO = Pin(13, Pin.IN)

# --- SG90 Servo (for ultrasonic sweep) ---
SERVO = PWM(Pin(15))
SERVO.freq(50)  # 50 Hz for standard servo

# --- MQ-2 Smoke Sensor (Analog) ---
MQ2 = ADC(Pin(26))

# --- UART0 to ESP32 Bridge ---
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

# --- Onboard LED for status ---
led = Pin(25, Pin.OUT)

# ─────────────────────── CONSTANTS ───────────────────────

MOTOR_SPEED       = 50000       # Default PWM duty (0-65535 range)
MOTOR_SPEED_TURN  = 40000       # Reduced speed for turning
WATCHDOG_TIMEOUT  = 500         # ms — kill motors if no command received
SWEEP_ANGLES      = [0, 45, 90, 135, 180]  # 5-point sweep
SCAN_INTERVAL_MS  = 200         # Time between sensor packets
US_TIMEOUT        = 30000       # Ultrasonic echo timeout in µs (~5m max)

# ─────────────────────── STATE VARIABLES ───────────────────────

last_cmd_time     = utime.ticks_ms()
last_scan_time    = utime.ticks_ms()
scan_distances    = [0.0, 0.0, 0.0, 0.0, 0.0]
current_sweep_idx = 0
sweep_state       = "MOVING"    # "MOVING" or "READING"
servo_move_start  = 0
boot_time         = utime.ticks_ms()

# ─────────────────────── HELPER FUNCTIONS ───────────────────────


def log(msg):
    """Verbose debug logging to UART (prefixed for filtering)."""
    timestamp = utime.ticks_diff(utime.ticks_ms(), boot_time)
    debug_line = "[PICO {:>8}ms] {}\n".format(timestamp, msg)
    # Print to USB serial for direct debugging
    sys.stdout.write(debug_line)


def set_servo_angle(angle):
    """
    Set SG90 servo to a specific angle (0-180).
    Duty cycle: 0° ≈ 1ms pulse, 180° ≈ 2ms pulse.
    At 50Hz, period = 20ms.
      1ms / 20ms = 5.0%  → duty = 3276
      2ms / 20ms = 10.0% → duty = 6553
    """
    min_duty = 1638   # ~0.5ms pulse (safe lower bound)
    max_duty = 8192   # ~2.5ms pulse (safe upper bound)
    duty = int(min_duty + (max_duty - min_duty) * angle / 180)
    SERVO.duty_u16(duty)


def read_distance_cm():
    """
    Trigger HC-SR04 and return distance in cm.
    Returns -1.0 on timeout (no echo received).
    """
    # Ensure trigger is low
    TRIG.low()
    utime.sleep_us(2)

    # Send 10µs trigger pulse
    TRIG.high()
    utime.sleep_us(10)
    TRIG.low()

    # Wait for echo to go HIGH (start of pulse)
    start_wait = utime.ticks_us()
    while ECHO.value() == 0:
        if utime.ticks_diff(utime.ticks_us(), start_wait) > US_TIMEOUT:
            log("WARN: Echo start timeout")
            return -1.0

    # Measure how long echo stays HIGH
    pulse_start = utime.ticks_us()
    while ECHO.value() == 1:
        if utime.ticks_diff(utime.ticks_us(), pulse_start) > US_TIMEOUT:
            log("WARN: Echo pulse timeout")
            return -1.0

    pulse_end = utime.ticks_us()
    pulse_duration = utime.ticks_diff(pulse_end, pulse_start)

    # Speed of sound ≈ 343 m/s → distance = (time_µs * 0.0343) / 2
    distance_cm = (pulse_duration * 0.0343) / 2.0

    # Clamp to reasonable range
    if distance_cm > 400.0:
        distance_cm = 400.0
    if distance_cm < 2.0:
        distance_cm = -1.0  # Too close, likely noise

    return round(distance_cm, 1)


def read_smoke():
    """
    Read MQ-2 analog value (0-65535 on Pico ADC).
    Returns raw 16-bit value. Higher = more smoke.
    """
    # Average 5 samples to reduce noise
    total = 0
    for _ in range(5):
        total += MQ2.read_u16()
        utime.sleep_us(100)
    return total // 5


def read_battery():
    """
    Placeholder for battery voltage reading.
    In production, use a voltage divider on an ADC pin.
    Returns 0.0 if not wired.
    """
    return 0.0


# ─────────────────────── MOTOR CONTROL ───────────────────────


def motors_stop():
    """Kill all motor outputs immediately."""
    IN1.low()
    IN2.low()
    IN3.low()
    IN4.low()
    ENA.duty_u16(0)
    ENB.duty_u16(0)


def motors_forward():
    """Drive both motors forward."""
    IN1.high()
    IN2.low()
    IN3.high()
    IN4.low()
    ENA.duty_u16(MOTOR_SPEED)
    ENB.duty_u16(MOTOR_SPEED)


def motors_backward():
    """Drive both motors backward."""
    IN1.low()
    IN2.high()
    IN3.low()
    IN4.high()
    ENA.duty_u16(MOTOR_SPEED)
    ENB.duty_u16(MOTOR_SPEED)


def motors_left():
    """Differential left turn: right motor forward, left motor backward."""
    IN1.low()
    IN2.high()
    IN3.high()
    IN4.low()
    ENA.duty_u16(MOTOR_SPEED_TURN)
    ENB.duty_u16(MOTOR_SPEED_TURN)


def motors_right():
    """Differential right turn: left motor forward, right motor backward."""
    IN1.high()
    IN2.low()
    IN3.low()
    IN4.high()
    ENA.duty_u16(MOTOR_SPEED_TURN)
    ENB.duty_u16(MOTOR_SPEED_TURN)


# Command dispatch table
CMD_MAP = {
    'W': motors_forward,
    'S': motors_backward,
    'A': motors_left,
    'D': motors_right,
    'X': motors_stop,
}


def handle_command(cmd_char):
    """
    Parse and execute a single-character motor command.
    Updates the watchdog timer on any valid command.
    """
    global last_cmd_time
    cmd = cmd_char.strip().upper()

    if cmd in CMD_MAP:
        CMD_MAP[cmd]()
        last_cmd_time = utime.ticks_ms()
        log("CMD: '{}' executed".format(cmd))
    else:
        log("WARN: Unknown command '{}'".format(cmd))


# ─────────────────────── NON-BLOCKING SWEEP ───────────────────────


def sweep_tick():
    """
    Non-blocking 5-point ultrasonic sweep state machine.

    States:
      MOVING  — Servo is moving to next angle, wait for settle
      READING — Servo has settled, take distance measurement

    This is called every loop iteration. It only acts when
    enough time has elapsed for the servo to reach position.
    """
    global current_sweep_idx, sweep_state, servo_move_start

    now = utime.ticks_ms()

    if sweep_state == "MOVING":
        # Move servo to the current angle
        target_angle = SWEEP_ANGLES[current_sweep_idx]
        set_servo_angle(target_angle)
        servo_move_start = now
        sweep_state = "SETTLING"

    elif sweep_state == "SETTLING":
        # Wait 80ms for servo to reach position (SG90 is ~100ms for 60°)
        if utime.ticks_diff(now, servo_move_start) >= 80:
            sweep_state = "READING"

    elif sweep_state == "READING":
        # Take distance measurement at current angle
        dist = read_distance_cm()
        scan_distances[current_sweep_idx] = dist

        # Advance to next angle
        current_sweep_idx = (current_sweep_idx + 1) % len(SWEEP_ANGLES)
        sweep_state = "MOVING"


# ─────────────────────── PACKET FORMATTING ───────────────────────


def build_packet():
    """
    Build the strict sensor data packet.
    Format: "SCAN:d1,d2,d3,d4,d5|SMOKE:v|BATT:v\n"

    Example: "SCAN:25.3,40.1,100.0,55.2,30.8|SMOKE:12450|BATT:7.4\n"
    """
    scan_str = ",".join("{:.1f}".format(d) for d in scan_distances)
    smoke_val = read_smoke()
    batt_val = read_battery()

    packet = "SCAN:{}|SMOKE:{}|BATT:{:.1f}\n".format(
        scan_str, smoke_val, batt_val
    )
    return packet


# ─────────────────────── WATCHDOG ───────────────────────


def check_watchdog():
    """
    500ms Watchdog Failsafe.
    If no valid UART command has been received within WATCHDOG_TIMEOUT ms,
    immediately kill all motors for safety.
    """
    elapsed = utime.ticks_diff(utime.ticks_ms(), last_cmd_time)
    if elapsed > WATCHDOG_TIMEOUT:
        motors_stop()
        # Only log occasionally to avoid flooding
        if elapsed % 2000 < 50:
            log("WATCHDOG: No command for {}ms — motors killed".format(elapsed))


# ─────────────────────── MAIN LOOP ───────────────────────


def main():
    """
    Main non-blocking event loop.

    Timeline per iteration:
      1. Check for incoming UART commands → execute motor actions
      2. Advance the ultrasonic sweep state machine (non-blocking)
      3. Periodically transmit sensor packet over UART
      4. Check watchdog timer → kill motors if stale
    """
    global last_scan_time, last_cmd_time

    log("═══════════════════════════════════════════════")
    log("  PICO WORKER — Plume Tracker v1.0")
    log("  UART: 115200 baud on GP0/GP1")
    log("  Motors: L298N on GP2-GP7")
    log("  Sonar: HC-SR04 on GP13/GP14, Servo GP15")
    log("  Smoke: MQ-2 on GP26 (ADC0)")
    log("═══════════════════════════════════════════════")

    # Initialize: center servo and stop motors
    set_servo_angle(90)
    motors_stop()
    utime.sleep_ms(500)
    log("INIT: Servo centered, motors stopped")
    log("INIT: Waiting for UART commands...")

    # Reset watchdog
    last_cmd_time = utime.ticks_ms()

    while True:
        now = utime.ticks_ms()

        # ── 1. READ UART COMMANDS ──
        if uart.any():
            try:
                raw = uart.readline()
                if raw:
                    decoded = raw.decode('utf-8').strip()
                    if decoded:
                        log("UART RX: '{}'".format(decoded))
                        # Process each character as a potential command
                        # (supports single char or multi-char burst)
                        for ch in decoded:
                            handle_command(ch)
            except Exception as e:
                log("ERROR: UART decode failed: {}".format(e))

        # ── 2. ADVANCE ULTRASONIC SWEEP ──
        sweep_tick()

        # ── 3. TRANSMIT SENSOR PACKET ──
        if utime.ticks_diff(now, last_scan_time) >= SCAN_INTERVAL_MS:
            packet = build_packet()
            try:
                uart.write(packet.encode('utf-8'))
                log("UART TX: {}".format(packet.strip()))
            except Exception as e:
                log("ERROR: UART TX failed: {}".format(e))
            last_scan_time = now

        # ── 4. WATCHDOG CHECK ──
        check_watchdog()

        # ── 5. HEARTBEAT LED ──
        # Blink LED every ~1s to show the loop is alive
        led.value((now // 500) % 2)

        # Small yield to prevent tight-loop CPU saturation
        utime.sleep_ms(5)


# ─────────────────────── ENTRY POINT ───────────────────────

if __name__ == "__main__":
    log("BOOT: Starting main loop...")
    try:
        main()
    except KeyboardInterrupt:
        motors_stop()
        log("SHUTDOWN: KeyboardInterrupt — motors stopped")
    except Exception as e:
        motors_stop()
        log("FATAL: {} — motors stopped".format(e))
        sys.print_exception(e)
