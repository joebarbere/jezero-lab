"""Record a chase-camera video of one episode.

    jezero_env/run.sh python3 -m eval.record --world jezero_delta_rocks_k05 \
        --controller baseline --variant unseen --seed 0 --out runs/videos/x.mp4
    ... --controller runs/ppo_rocks/checkpoints/ppo_2799888_steps.zip --rock-patch

Same spawn as eval/evaluate.py for the same (variant, seed). The camera renders
with Gazebo's ogre2 on the GPU, which needs a display: run the container with
containers/run.sh (it passes DISPLAY, the X socket, and /dev/dri). Frames are in
sim time at 15 fps; --speedup plays them back faster.
"""
import argparse
import os
import pickle
import subprocess
import threading
import time

from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

from eval.evaluate import SEED0, VARIANTS, _vecnormalize_path, obs_options
from jezero_env.baseline import policy as baseline_policy
from jezero_env.env import JezeroEnv

W, H, FPS = 960, 540, 15


def load_policy(path):
    from stable_baselines3 import PPO
    model = PPO.load(path, device='cpu')
    with open(_vecnormalize_path(path), 'rb') as f:
        norm = pickle.load(f)
    norm.training = False
    return lambda obs: model.predict(norm.normalize_obs(obs[None, :]), deterministic=True)[0][0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='jezero_delta_rocks_k05')
    ap.add_argument('--controller', default='baseline', help="'baseline' or a policy .zip")
    ap.add_argument('--rock-patch', action='store_true')
    ap.add_argument('--variant', default='unseen', choices=list(VARIANTS))
    ap.add_argument('--seed', type=int, default=0, help='episode index, as in evaluate.py')
    ap.add_argument('--max-seconds', type=float, default=240.0, help='sim seconds to record at most')
    ap.add_argument('--speedup', type=float, default=4.0)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    obs = (obs_options(args.controller) if args.controller != 'baseline' else None) \
        or {'rock_patch': args.rock_patch}
    env = JezeroEnv(world=args.world, **obs, camera=True,
                    max_episode_seconds=args.max_seconds, **VARIANTS[args.variant])
    act = baseline_policy if args.controller == 'baseline' else load_policy(args.controller)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    label = 'baseline' if args.controller == 'baseline' else 'policy'
    title = f"{label} | {args.world} | {args.variant} {args.seed} | x{args.speedup:g}"
    # Frames stream straight into ffmpeg: holding a 3-minute episode in memory
    # would be ~4 GB of raw RGB.
    ff = subprocess.Popen(
        ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
         '-s', f'{W}x{H}', '-r', f'{FPS * args.speedup:g}', '-i', '-',
         '-vf', f"drawtext=text='{title}':x=12:y=12:fontsize=20:fontcolor=white:box=1:boxcolor=black@0.5",
         '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-r', '30', args.out],
        stdin=subprocess.PIPE)
    lock, state = threading.Lock(), {'recording': False, 'frames': 0}

    def on_image(msg):
        if msg.width != W or msg.height != H:
            return
        with lock:
            if state['recording']:
                ff.stdin.write(msg.data)
                state['frames'] += 1

    node = Node()            # created after the sim, so it shares its GZ_PARTITION
    if not node.subscribe(Image, '/chase_cam', on_image):
        raise SystemExit('could not subscribe to /chase_cam')

    obs, info = env.reset(seed=SEED0 + args.seed)
    time.sleep(0.5)                                        # drain frames from settling
    with lock:
        state['recording'] = True
    t = time.time()
    while True:
        obs, _, term, trunc, step_info = env.step(act(obs))
        if term or trunc:
            break
    env.sim.step(int(1.0 / env.sim.step_size))             # hold the last second
    time.sleep(0.5)                                        # let the last frames arrive
    with lock:
        state['recording'] = False
        n = state['frames']
    ff.stdin.close()
    ff.wait()
    if not n:
        raise SystemExit('no frames received: is rendering working (DISPLAY, /dev/dri)?')
    print(f"{args.out}: {n} frames ({n / FPS:.0f} sim-s), outcome {step_info['event']} "
          f"after {step_info['sim_time']:.0f} sim-s, final distance {step_info['distance']:.1f} m, "
          f"recorded in {time.time() - t:.0f} s")


if __name__ == '__main__':
    main()
