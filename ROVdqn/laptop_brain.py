"""
╔══════════════════════════════════════════════════════════════════════════╗
║  laptop_brain.py — Laptop DQN Brain (Python / Stable Baselines3)        ║
║  Project : Autonomous AI-Driven Plume Tracker (DQN + Triple-MCU)       ║
║  Role    : Gymnasium environment + DQN training for plume tracking     ║
║  Comms   : UDP ↔ ESP32 Bridge ↔ Pico Worker                            ║
╚══════════════════════════════════════════════════════════════════════════╝

Usage:
  Training:   python laptop_brain.py --mode train --timesteps 100000
  Inference:  python laptop_brain.py --mode infer --model plume_tracker_dqn.zip
"""

import socket
import time
import struct
import argparse
import logging
import sys
import os
from collections import deque
from datetime import datetime

import numpy as np
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import DQN
from stable_baselines3.common.callbacks import (
    BaseCallback,
    CheckpointCallback,
    EvalCallback,
)
from stable_baselines3.common.monitor import Monitor

# ─────────────────────── LOGGING SETUP ───────────────────────

logging.basicConfig(
    level=logging.DEBUG,
    format="[BRAIN %(asctime)s] [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("PlumeTracker")

# ─────────────────────── CONFIGURATION ───────────────────────

# Network
ESP32_IP        = "10.35.225.200"   # ESP32 bridge static IP
UDP_PORT        = 1234               # UDP port for comms
LAPTOP_IP       = "0.0.0.0"         # Listen on all local interfaces
SOCKET_TIMEOUT  = 2.0               # seconds

# Sensor ranges (for normalization)
MAX_DISTANCE    = 400.0             # HC-SR04 max range (cm)
MAX_SMOKE       = 65535             # MQ-2 16-bit ADC max
MIN_SAFE_DIST   = 15.0             # cm — obstacle penalty threshold

# Training
DEFAULT_TIMESTEPS       = 100_000
CHECKPOINT_FREQ         = 10_000
MODEL_SAVE_NAME         = "plume_tracker_dqn"
LOG_DIR                 = "./tb_logs"
CHECKPOINT_DIR          = "./checkpoints"

# Action mapping: DQN discrete action → motor command character
ACTION_TO_CMD = {
    0: 'W',  # Forward
    1: 'S',  # Backward
    2: 'A',  # Left
    3: 'D',  # Right
    4: 'X',  # Stop
}

ACTION_NAMES = {
    0: 'Forward',
    1: 'Backward',
    2: 'Left',
    3: 'Right',
    4: 'Stop',
}


# ─────────────────────── UDP COMMUNICATION ───────────────────────


class UDPComms:
    """
    Handles bidirectional UDP communication with the ESP32 bridge.

    Send motor commands TO the rover.
    Receive sensor packets FROM the rover.
    """

    def __init__(self, esp32_ip=ESP32_IP, udp_port=UDP_PORT, timeout=SOCKET_TIMEOUT):
        self.esp32_ip = esp32_ip
        self.udp_port = udp_port
        self.timeout = timeout
        self.sock = None
        self.packet_count = 0
        self.last_raw_packet = ""

    def connect(self):
        """Create and bind the UDP socket."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((LAPTOP_IP, self.udp_port))
            self.sock.settimeout(self.timeout)
            log.info(f"UDP socket bound to {LAPTOP_IP}:{self.udp_port}")
            log.info(f"ESP32 target: {self.esp32_ip}:{self.udp_port}")
            return True
        except Exception as e:
            log.error(f"UDP socket creation failed: {e}")
            return False

    def send_command(self, cmd_char):
        """Send a single-character motor command to the ESP32 bridge."""
        if self.sock is None:
            log.warning("Socket not initialized, cannot send command")
            return False
        try:
            self.sock.sendto(cmd_char.encode('utf-8'), (self.esp32_ip, self.udp_port))
            log.debug(f"UDP TX → '{cmd_char}' to {self.esp32_ip}:{self.udp_port}")
            return True
        except Exception as e:
            log.error(f"UDP send failed: {e}")
            return False

    def receive_packet(self):
        """
        Receive and parse a sensor packet from the Pico (via ESP32 bridge).

        Expected format: "SCAN:d1,d2,d3,d4,d5|SMOKE:v|BATT:v\n"

        Returns:
            dict with keys: 'distances' (list[5]), 'smoke' (int), 'battery' (float)
            or None on failure.
        """
        if self.sock is None:
            return None
        try:
            data, addr = self.sock.recvfrom(1024)
            raw = data.decode('utf-8').strip()
            self.last_raw_packet = raw
            self.packet_count += 1
            log.debug(f"UDP RX ← [{self.packet_count}] from {addr}: '{raw}'")

            return self._parse_packet(raw)
        except socket.timeout:
            log.warning("UDP receive timeout — no data from rover")
            return None
        except Exception as e:
            log.error(f"UDP receive error: {e}")
            return None

    def _parse_packet(self, raw):
        """
        Parse the strict packet format.
        "SCAN:25.3,40.1,100.0,55.2,30.8|SMOKE:12450|BATT:7.4"
        """
        try:
            # Validate structure
            if not raw.startswith("SCAN:") or "|SMOKE:" not in raw or "|BATT:" not in raw:
                log.warning(f"Malformed packet (structure): '{raw}'")
                return None

            # Split into sections
            parts = raw.split("|")
            if len(parts) != 3:
                log.warning(f"Malformed packet (sections): '{raw}'")
                return None

            # Parse SCAN:d1,d2,d3,d4,d5
            scan_str = parts[0].replace("SCAN:", "")
            distances = [float(d) for d in scan_str.split(",")]
            if len(distances) != 5:
                log.warning(f"Expected 5 distances, got {len(distances)}")
                return None

            # Parse SMOKE:v
            smoke_str = parts[1].replace("SMOKE:", "")
            smoke_val = int(smoke_str)

            # Parse BATT:v
            batt_str = parts[2].replace("BATT:", "")
            batt_val = float(batt_str)

            return {
                'distances': distances,
                'smoke': smoke_val,
                'battery': batt_val,
            }

        except (ValueError, IndexError) as e:
            log.warning(f"Packet parse error: {e} — raw: '{raw}'")
            return None

    def close(self):
        """Close the UDP socket."""
        if self.sock:
            self.sock.close()
            log.info("UDP socket closed")


# ─────────────────────── GYMNASIUM ENVIRONMENT ───────────────────────


class PlumeTrackerEnv(gym.Env):
    """
    Custom Gymnasium environment for the Plume Tracker rover.

    Observation Space (6-element float32 vector):
        [dist_0°, dist_45°, dist_90°, dist_135°, dist_180°, smoke_value]
        All values normalized to [0, 1].

    Action Space (Discrete 5):
        0 = Forward (W)
        1 = Backward (S)
        2 = Left (A)
        3 = Right (D)
        4 = Stop (X)

    Reward Function:
        - delta_smoke = current_smoke - prev_smoke
        - reward = delta_smoke * 2.0
        - if min(distances) < 15cm: reward -= 50  (obstacle penalty)
        - if action == Forward and delta_smoke > 0: reward += 5 (progress bonus)
        - Small step penalty: reward -= 0.1
    """

    metadata = {'render_modes': ['human']}

    def __init__(self, udp_comms=None, max_steps=500, render_mode=None):
        super().__init__()

        # Communication
        self.comms = udp_comms

        # Spaces
        self.observation_space = spaces.Box(
            low=0.0,
            high=1.0,
            shape=(6,),
            dtype=np.float32,
        )
        self.action_space = spaces.Discrete(5)

        # Episode state
        self.max_steps = max_steps
        self.current_step = 0
        self.prev_smoke = 0.0
        self.current_obs = np.zeros(6, dtype=np.float32)
        self.episode_reward = 0.0
        self.episode_count = 0

        # History for debugging
        self.smoke_history = deque(maxlen=100)
        self.reward_history = deque(maxlen=100)

        log.info("╔══════════════════════════════════════════════╗")
        log.info("║  PlumeTrackerEnv initialized                  ║")
        log.info(f"║  Observation: {self.observation_space.shape}  ║")
        log.info(f"║  Actions:     {self.action_space.n}               ║")
        log.info(f"║  Max steps:   {self.max_steps}                  ║")
        log.info("╚══════════════════════════════════════════════╝")

    def _get_observation(self):
        """
        Request sensor data from the rover via UDP.
        Returns normalized 6-element observation vector.
        """
        if self.comms is None:
            log.warning("No UDP comms — returning zero observation")
            return np.zeros(6, dtype=np.float32)

        packet = self.comms.receive_packet()
        if packet is None:
            log.warning("No valid packet — reusing last observation")
            return self.current_obs.copy()

        # Normalize distances to [0, 1]
        distances = np.array(packet['distances'], dtype=np.float32)
        distances = np.clip(distances, 0.0, MAX_DISTANCE) / MAX_DISTANCE

        # Handle -1.0 (timeout) readings — treat as max distance
        distances = np.where(distances < 0, 1.0, distances)

        # Normalize smoke to [0, 1]
        smoke_norm = np.clip(float(packet['smoke']), 0.0, MAX_SMOKE) / MAX_SMOKE

        obs = np.concatenate([distances, [smoke_norm]]).astype(np.float32)
        self.current_obs = obs

        log.debug(f"OBS: dist={distances} smoke={smoke_norm:.4f} (raw={packet['smoke']})")
        return obs

    def reset(self, seed=None, options=None):
        """
        Reset the environment for a new episode.
        Sends STOP command to the rover and waits for fresh sensor data.
        """
        super().reset(seed=seed)

        self.current_step = 0
        self.prev_smoke = 0.0
        self.episode_reward = 0.0
        self.episode_count += 1

        # Stop the rover
        if self.comms:
            self.comms.send_command('X')
            time.sleep(0.3)  # Let the rover settle

        # Get initial observation
        obs = self._get_observation()
        self.prev_smoke = obs[5]  # Initial smoke reading

        log.info(f"═══ EPISODE {self.episode_count} RESET ═══")
        log.info(f"Initial smoke: {self.prev_smoke:.4f}")

        return obs, {}

    def step(self, action):
        """
        Execute one environment step:
          1. Send motor command to rover
          2. Wait for sensor response
          3. Compute reward
          4. Check termination conditions
        """
        self.current_step += 1

        # ── 1. SEND ACTION ──
        cmd = ACTION_TO_CMD[action]
        action_name = ACTION_NAMES[action]

        if self.comms:
            self.comms.send_command(cmd)
            # Wait for the rover to act and sensors to update
            time.sleep(0.2)

        # ── 2. GET OBSERVATION ──
        obs = self._get_observation()
        current_smoke = obs[5]
        distances_raw = obs[:5] * MAX_DISTANCE  # De-normalize for reward calc

        # ── 3. COMPUTE REWARD ──
        reward = 0.0

        # Smoke gradient reward
        delta_smoke = current_smoke - self.prev_smoke
        reward += delta_smoke * 2.0

        # Obstacle penalty
        min_dist = np.min(distances_raw)
        if min_dist < MIN_SAFE_DIST:
            reward -= 50.0
            log.warning(f"⚠️  OBSTACLE PENALTY: min_dist={min_dist:.1f}cm")

        # Progress bonus: moving forward toward higher smoke
        if action == 0 and delta_smoke > 0:  # Forward + increasing smoke
            reward += 5.0

        # Small step penalty to encourage efficiency
        reward -= 0.1

        # ── 4. CHECK TERMINATION ──
        truncated = self.current_step >= self.max_steps
        terminated = False

        # Terminate if collision is imminent (any distance < 5cm)
        if min_dist < 5.0:
            terminated = True
            reward -= 100.0
            log.error(f"💥 COLLISION TERMINATION: min_dist={min_dist:.1f}cm")

        # Update state
        self.prev_smoke = current_smoke
        self.episode_reward += reward
        self.smoke_history.append(current_smoke)
        self.reward_history.append(reward)

        # ── 5. LOGGING ──
        log.debug(
            f"Step {self.current_step:>4d} | "
            f"Action: {action_name:>8s} | "
            f"Smoke: {current_smoke:.4f} (Δ{delta_smoke:+.4f}) | "
            f"MinDist: {min_dist:>6.1f}cm | "
            f"Reward: {reward:>7.2f} | "
            f"EpReward: {self.episode_reward:>8.2f}"
        )

        info = {
            'action_name': action_name,
            'delta_smoke': delta_smoke,
            'min_distance': min_dist,
            'step_reward': reward,
            'episode_reward': self.episode_reward,
        }

        return obs, reward, terminated, truncated, info

    def close(self):
        """Clean up resources."""
        if self.comms:
            self.comms.send_command('X')  # Stop rover
            self.comms.close()
        log.info("Environment closed")


# ─────────────────────── CUSTOM CALLBACK ───────────────────────


class PlumeTrackerCallback(BaseCallback):
    """
    Custom callback for logging training progress.
    Logs episode statistics and saves best model.
    """

    def __init__(self, verbose=1):
        super().__init__(verbose)
        self.episode_rewards = []
        self.best_mean_reward = -np.inf

    def _on_step(self) -> bool:
        # Check for episode completion
        if len(self.model.ep_info_buffer) > 0:
            latest = self.model.ep_info_buffer[-1]
            ep_reward = latest.get('r', 0)
            ep_length = latest.get('l', 0)
            self.episode_rewards.append(ep_reward)

            if self.verbose > 0 and len(self.episode_rewards) % 10 == 0:
                mean_reward = np.mean(self.episode_rewards[-100:])
                log.info(
                    f"📊 Episodes: {len(self.episode_rewards)} | "
                    f"Mean Reward (100): {mean_reward:.2f} | "
                    f"Last: {ep_reward:.2f} | "
                    f"Length: {ep_length}"
                )

                # Save best model
                if mean_reward > self.best_mean_reward:
                    self.best_mean_reward = mean_reward
                    self.model.save(f"{MODEL_SAVE_NAME}_best")
                    log.info(f"🏆 New best model! Mean reward: {mean_reward:.2f}")

        return True


# ─────────────────────── TRAINING ───────────────────────


def train(timesteps=DEFAULT_TIMESTEPS, continue_from=None):
    """
    Train the DQN agent on the real hardware environment.

    Args:
        timesteps: Total training timesteps.
        continue_from: Path to existing model to continue training.
    """
    log.info("╔══════════════════════════════════════════════════╗")
    log.info("║  TRAINING MODE — DQN Plume Tracker                ║")
    log.info(f"║  Timesteps: {timesteps:>10d}                       ║")
    log.info("╚══════════════════════════════════════════════════╝")

    # ── Setup UDP ──
    comms = UDPComms()
    if not comms.connect():
        log.error("Failed to connect UDP — aborting training")
        return

    # ── Create Environment ──
    env = PlumeTrackerEnv(udp_comms=comms, max_steps=500)
    env = Monitor(env, LOG_DIR)

    # ── Create or Load Model ──
    os.makedirs(LOG_DIR, exist_ok=True)
    os.makedirs(CHECKPOINT_DIR, exist_ok=True)

    if continue_from and os.path.exists(continue_from):
        log.info(f"Loading existing model: {continue_from}")
        model = DQN.load(continue_from, env=env)
    else:
        log.info("Creating new DQN model")
        model = DQN(
            policy="MlpPolicy",
            env=env,
            learning_rate=1e-4,
            buffer_size=50_000,
            learning_starts=1000,
            batch_size=64,
            tau=0.005,
            gamma=0.99,
            train_freq=4,
            gradient_steps=1,
            target_update_interval=1000,
            exploration_fraction=0.3,
            exploration_initial_eps=1.0,
            exploration_final_eps=0.05,
            verbose=1,
            tensorboard_log=LOG_DIR,
            device="auto",
        )

    # ── Callbacks ──
    checkpoint_cb = CheckpointCallback(
        save_freq=CHECKPOINT_FREQ,
        save_path=CHECKPOINT_DIR,
        name_prefix=MODEL_SAVE_NAME,
    )
    plume_cb = PlumeTrackerCallback(verbose=1)

    # ── Train ──
    log.info("Starting DQN training...")
    start_time = time.time()

    try:
        model.learn(
            total_timesteps=timesteps,
            callback=[checkpoint_cb, plume_cb],
            log_interval=10,
            progress_bar=True,
        )
    except KeyboardInterrupt:
        log.info("Training interrupted by user")
    finally:
        elapsed = time.time() - start_time
        log.info(f"Training complete in {elapsed:.1f}s")

        # Save final model
        model_path = f"{MODEL_SAVE_NAME}.zip"
        model.save(model_path)
        log.info(f"Model saved to: {model_path}")

        env.close()


# ─────────────────────── INFERENCE ───────────────────────


def infer(model_path, max_episodes=10):
    """
    Deploy a trained DQN model on the real hardware.

    Args:
        model_path: Path to the trained model file.
        max_episodes: Number of inference episodes to run.
    """
    log.info("╔══════════════════════════════════════════════════╗")
    log.info("║  INFERENCE MODE — DQN Plume Tracker               ║")
    log.info(f"║  Model: {model_path:<40s}  ║")
    log.info(f"║  Episodes: {max_episodes:>5d}                           ║")
    log.info("╚══════════════════════════════════════════════════╝")

    if not os.path.exists(model_path):
        log.error(f"Model file not found: {model_path}")
        return

    # ── Setup UDP ──
    comms = UDPComms()
    if not comms.connect():
        log.error("Failed to connect UDP — aborting inference")
        return

    # ── Create Environment ──
    env = PlumeTrackerEnv(udp_comms=comms, max_steps=500)

    # ── Load Model ──
    log.info(f"Loading model: {model_path}")
    model = DQN.load(model_path, env=env)

    # ── Run Episodes ──
    for episode in range(1, max_episodes + 1):
        obs, info = env.reset()
        total_reward = 0.0
        step_count = 0
        done = False

        log.info(f"\n{'═' * 50}")
        log.info(f"  EPISODE {episode}/{max_episodes}")
        log.info(f"{'═' * 50}")

        while not done:
            # Get action from trained model (deterministic)
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)

            total_reward += reward
            step_count += 1
            done = terminated or truncated

            # Detailed step logging
            log.info(
                f"  Step {step_count:>3d} | "
                f"{info['action_name']:>8s} | "
                f"Smoke Δ: {info['delta_smoke']:+.4f} | "
                f"MinDist: {info['min_distance']:>6.1f}cm | "
                f"Reward: {info['step_reward']:>7.2f}"
            )

        log.info(f"\n  Episode {episode} done — "
                 f"Steps: {step_count} | "
                 f"Total Reward: {total_reward:.2f}")

    # Cleanup
    env.close()
    log.info("\nInference complete")


# ─────────────────────── DIAGNOSTIC TOOLS ───────────────────────


def test_connection():
    """
    Test the UDP connection to the rover.
    Sends a STOP command and waits for a sensor packet.
    """
    log.info("╔══════════════════════════════════════════════════╗")
    log.info("║  CONNECTION TEST                                  ║")
    log.info("╚══════════════════════════════════════════════════╝")

    comms = UDPComms()
    if not comms.connect():
        log.error("UDP bind failed")
        return

    log.info("Sending STOP command...")
    comms.send_command('X')

    log.info("Waiting for sensor packet (5s timeout)...")
    comms.sock.settimeout(5.0)

    for i in range(10):
        packet = comms.receive_packet()
        if packet:
            log.info(f"✅ Packet {i + 1} received:")
            log.info(f"   Distances: {packet['distances']}")
            log.info(f"   Smoke:     {packet['smoke']}")
            log.info(f"   Battery:   {packet['battery']}")
        else:
            log.warning(f"❌ Packet {i + 1}: No data")
        time.sleep(0.5)

    comms.close()


def manual_control():
    """
    Manual keyboard control for testing motor commands.
    Uses WASDX keys for direct control.
    """
    log.info("╔══════════════════════════════════════════════════╗")
    log.info("║  MANUAL CONTROL MODE                              ║")
    log.info("║  W=Forward  S=Backward  A=Left  D=Right  X=Stop  ║")
    log.info("║  Q=Quit                                           ║")
    log.info("╚══════════════════════════════════════════════════╝")

    comms = UDPComms()
    if not comms.connect():
        log.error("UDP bind failed")
        return

    try:
        if os.name == 'nt':
            import msvcrt
            log.info("Windows detected — using msvcrt for live control")
            while True:
                if msvcrt.kbhit():
                    ch = msvcrt.getch().decode('utf-8').upper()
                    if ch == 'Q':
                        comms.send_command('X')
                        break
                    if ch in ACTION_TO_CMD.values():
                        comms.send_command(ch)
                        packet = comms.receive_packet()
                        if packet:
                            print(f"\r  {ch} → Smoke: {packet['smoke']} | "
                                  f"Dist: {packet['distances']}    ", end='', flush=True)
                time.sleep(0.05)
        else:
            import tty
            import termios
            # Set terminal to raw mode for single-char input
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                while True:
                    ch = sys.stdin.read(1).upper()
                    if ch == 'Q':
                        comms.send_command('X')
                        break
                    if ch in ACTION_TO_CMD.values():
                        comms.send_command(ch)
                        packet = comms.receive_packet()
                        if packet:
                            print(f"\r  {ch} → Smoke: {packet['smoke']} | "
                                  f"Dist: {packet['distances']}    ", end='', flush=True)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    except (ImportError, Exception) as e:
        log.warning(f"Live control error: {e} — using input() fallback")
        while True:
            cmd = input("Command (WASDXQ): ").strip().upper()
            if not cmd: continue
            ch = cmd[0]
            if ch == 'Q':
                comms.send_command('X')
                break
            if ch in ACTION_TO_CMD.values():
                comms.send_command(ch)
    finally:
        comms.close()
        log.info("Manual control ended")


# ─────────────────────── MAIN ENTRY POINT ───────────────────────


def main():
    global ESP32_IP
    
    # ── Dependency Check ──
    try:
        import numpy
        import gymnasium
        import stable_baselines3
    except ImportError as e:
        print(f"\n[!] ERROR: Missing dependency: {e}")
        print("[!] Please run: pip install -r requirements.txt\n")
        sys.exit(1)

    parser = argparse.ArgumentParser(
        description="Plume Tracker DQN — Laptop Brain",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python laptop_brain.py --mode train --timesteps 50000
  python laptop_brain.py --mode infer --model plume_tracker_dqn.zip
  python laptop_brain.py --mode test
  python laptop_brain.py --mode manual
        """
    )
    parser.add_argument(
        "--mode",
        choices=["train", "infer", "test", "manual"],
        default="test",
        help="Operating mode (default: test)",
    )
    parser.add_argument(
        "--timesteps",
        type=int,
        default=DEFAULT_TIMESTEPS,
        help=f"Training timesteps (default: {DEFAULT_TIMESTEPS})",
    )
    parser.add_argument(
        "--model",
        type=str,
        default=f"{MODEL_SAVE_NAME}.zip",
        help=f"Model file for inference (default: {MODEL_SAVE_NAME}.zip)",
    )
    parser.add_argument(
        "--continue-from",
        type=str,
        default=None,
        help="Continue training from existing model",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=10,
        help="Number of inference episodes (default: 10)",
    )
    parser.add_argument(
        "--esp32-ip",
        type=str,
        default=ESP32_IP,
        help=f"ESP32 bridge IP (default: {ESP32_IP})",
    )

    args = parser.parse_args()

    # Update global from args
    ESP32_IP = args.esp32_ip

    log.info(f"Plume Tracker Brain — Mode: {args.mode}")
    log.info(f"ESP32 IP: {ESP32_IP}")

    if args.mode == "train":
        train(timesteps=args.timesteps, continue_from=args.continue_from)
    elif args.mode == "infer":
        infer(model_path=args.model, max_episodes=args.episodes)
    elif args.mode == "test":
        test_connection()
    elif args.mode == "manual":
        manual_control()


if __name__ == "__main__":
    main()
