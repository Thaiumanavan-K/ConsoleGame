"""
╔══════════════════════════════════════════════════════════════════════════╗
║  inference.py — Standalone DQN Inference Script                         ║
║  Project : Autonomous AI-Driven Plume Tracker (DQN + Triple-MCU)       ║
║  Role    : Deploy a trained DQN model on the physical rover             ║
║  Usage   : python inference.py --model plume_tracker_dqn.zip            ║
╚══════════════════════════════════════════════════════════════════════════╝
"""

import argparse
import logging
import time
import sys
import os

import numpy as np
from stable_baselines3 import DQN

# Import from the main brain module
from laptop_brain import (
    UDPComms,
    PlumeTrackerEnv,
    ACTION_NAMES,
    ESP32_IP,
    MODEL_SAVE_NAME,
)

# ─────────────────────── LOGGING ───────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="[INFER %(asctime)s] [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("Inference")


# ─────────────────────── INFERENCE ENGINE ───────────────────────


class PlumeInferenceEngine:
    """
    Standalone inference engine for deploying a trained DQN model.
    Includes safety overrides and detailed telemetry logging.
    """

    def __init__(self, model_path, esp32_ip=ESP32_IP, safety_distance=10.0):
        self.model_path = model_path
        self.esp32_ip = esp32_ip
        self.safety_distance = safety_distance  # cm — emergency stop threshold
        self.model = None
        self.env = None
        self.comms = None

        # Telemetry
        self.total_steps = 0
        self.total_episodes = 0
        self.emergency_stops = 0
        self.max_smoke_seen = 0.0
        self.start_time = None

    def initialize(self):
        """Connect to rover and load the trained model."""
        log.info("═══════════════════════════════════════════════")
        log.info("  PLUME TRACKER — INFERENCE ENGINE v1.0")
        log.info("═══════════════════════════════════════════════")

        # 1. Connect UDP
        self.comms = UDPComms(esp32_ip=self.esp32_ip)
        if not self.comms.connect():
            log.error("❌ Failed to establish UDP connection")
            return False

        # 2. Create environment
        self.env = PlumeTrackerEnv(udp_comms=self.comms, max_steps=1000)

        # 3. Load trained model
        if not os.path.exists(self.model_path):
            log.error(f"❌ Model file not found: {self.model_path}")
            return False

        log.info(f"Loading model: {self.model_path}")
        self.model = DQN.load(self.model_path, env=self.env)
        log.info("✅ Model loaded successfully")

        self.start_time = time.time()
        return True

    def run_episode(self, episode_num=1):
        """
        Run a single inference episode with safety monitoring.

        Returns:
            dict with episode statistics
        """
        log.info(f"\n{'━' * 60}")
        log.info(f"  EPISODE {episode_num}")
        log.info(f"{'━' * 60}")

        obs, info = self.env.reset()
        total_reward = 0.0
        step_count = 0
        done = False
        max_smoke_episode = 0.0
        action_counts = {name: 0 for name in ACTION_NAMES.values()}

        while not done:
            step_count += 1
            self.total_steps += 1

            # ── SAFETY CHECK (before model prediction) ──
            distances_raw = obs[:5] * 400.0  # De-normalize
            min_dist = np.min(distances_raw)

            if min_dist < self.safety_distance:
                # Emergency override: force STOP regardless of model
                self.comms.send_command('X')
                self.emergency_stops += 1
                log.warning(
                    f"🛑 SAFETY OVERRIDE: min_dist={min_dist:.1f}cm < "
                    f"{self.safety_distance}cm — forcing STOP "
                    f"(total overrides: {self.emergency_stops})"
                )
                time.sleep(0.5)  # Pause before resuming

                # Get fresh observation after stop
                obs = self.env._get_observation()
                continue

            # ── MODEL PREDICTION ──
            action, _states = self.model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = self.env.step(action)

            total_reward += reward
            done = terminated or truncated

            # Track statistics
            action_name = info.get('action_name', '?')
            action_counts[action_name] = action_counts.get(action_name, 0) + 1
            current_smoke = obs[5]
            max_smoke_episode = max(max_smoke_episode, current_smoke)
            self.max_smoke_seen = max(self.max_smoke_seen, current_smoke)

            # Periodic telemetry (every 10 steps)
            if step_count % 10 == 0:
                log.info(
                    f"  📡 Step {step_count:>4d} | "
                    f"Action: {action_name:>8s} | "
                    f"Smoke: {current_smoke:.4f} | "
                    f"MinDist: {min_dist:>6.1f}cm | "
                    f"EpReward: {total_reward:>8.2f}"
                )

        # ── Episode Summary ──
        self.total_episodes += 1

        stats = {
            'episode': episode_num,
            'steps': step_count,
            'total_reward': total_reward,
            'max_smoke': max_smoke_episode,
            'action_distribution': action_counts,
            'emergency_stops': self.emergency_stops,
        }

        log.info(f"\n  ┌─── Episode {episode_num} Summary ───┐")
        log.info(f"  │ Steps:        {step_count:>6d}")
        log.info(f"  │ Total Reward: {total_reward:>8.2f}")
        log.info(f"  │ Max Smoke:    {max_smoke_episode:>8.4f}")
        log.info(f"  │ Actions:")
        for act_name, count in sorted(action_counts.items(), key=lambda x: -x[1]):
            pct = (count / max(step_count, 1)) * 100
            bar = '█' * int(pct / 5)
            log.info(f"  │   {act_name:>8s}: {count:>4d} ({pct:>5.1f}%) {bar}")
        log.info(f"  └{'─' * 30}┘")

        return stats

    def run(self, num_episodes=10, continuous=False):
        """
        Run multiple inference episodes.

        Args:
            num_episodes: Number of episodes. Ignored if continuous=True.
            continuous: Run indefinitely until Ctrl+C.
        """
        all_stats = []
        episode = 0

        try:
            while True:
                episode += 1
                if not continuous and episode > num_episodes:
                    break

                stats = self.run_episode(episode_num=episode)
                all_stats.append(stats)

                # Brief pause between episodes
                log.info("  ⏸ Pausing 2s before next episode...")
                time.sleep(2.0)

        except KeyboardInterrupt:
            log.info("\n🛑 Inference stopped by user (Ctrl+C)")
            self.comms.send_command('X')  # Safety stop

        # ── Final Summary ──
        elapsed = time.time() - self.start_time if self.start_time else 0
        log.info(f"\n{'═' * 60}")
        log.info(f"  INFERENCE COMPLETE — FINAL SUMMARY")
        log.info(f"{'═' * 60}")
        log.info(f"  Total Episodes:      {self.total_episodes}")
        log.info(f"  Total Steps:         {self.total_steps}")
        log.info(f"  Total Time:          {elapsed:.1f}s")
        log.info(f"  Emergency Stops:     {self.emergency_stops}")
        log.info(f"  Max Smoke Observed:  {self.max_smoke_seen:.4f}")

        if all_stats:
            rewards = [s['total_reward'] for s in all_stats]
            log.info(f"  Mean Reward:         {np.mean(rewards):.2f}")
            log.info(f"  Best Reward:         {np.max(rewards):.2f}")
            log.info(f"  Worst Reward:        {np.min(rewards):.2f}")

        log.info(f"{'═' * 60}")

        return all_stats

    def shutdown(self):
        """Safely shut down the inference engine."""
        log.info("Shutting down inference engine...")
        if self.comms:
            self.comms.send_command('X')
            time.sleep(0.2)
        if self.env:
            self.env.close()
        log.info("✅ Shutdown complete")


# ─────────────────────── MAIN ───────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="Plume Tracker DQN — Inference Deployment",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python inference.py --model plume_tracker_dqn.zip
  python inference.py --model plume_tracker_dqn_best.zip --episodes 20
  python inference.py --model plume_tracker_dqn.zip --continuous
  python inference.py --model plume_tracker_dqn.zip --safety-dist 20
        """
    )
    parser.add_argument(
        "--model",
        type=str,
        default=f"{MODEL_SAVE_NAME}.zip",
        help=f"Path to trained model (default: {MODEL_SAVE_NAME}.zip)",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=10,
        help="Number of inference episodes (default: 10)",
    )
    parser.add_argument(
        "--continuous",
        action="store_true",
        help="Run continuously until Ctrl+C",
    )
    parser.add_argument(
        "--esp32-ip",
        type=str,
        default=ESP32_IP,
        help=f"ESP32 bridge IP address (default: {ESP32_IP})",
    )
    parser.add_argument(
        "--safety-dist",
        type=float,
        default=10.0,
        help="Emergency stop distance in cm (default: 10.0)",
    )

    args = parser.parse_args()

    # ── Initialize ──
    engine = PlumeInferenceEngine(
        model_path=args.model,
        esp32_ip=args.esp32_ip,
        safety_distance=args.safety_dist,
    )

    if not engine.initialize():
        log.error("Initialization failed — exiting")
        sys.exit(1)

    # ── Run ──
    try:
        engine.run(
            num_episodes=args.episodes,
            continuous=args.continuous,
        )
    finally:
        engine.shutdown()


if __name__ == "__main__":
    main()
