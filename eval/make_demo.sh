#!/usr/bin/env bash
# Side-by-side chase-camera videos, baseline (left) vs policy (right), from
# identical spawns. Needs a display for rendering, so run it on the host (it
# starts the containers itself):
#
#   eval/make_demo.sh runs/ppo_rocks/checkpoints/ppo_2799888_steps.zip \
#       runs/eval_ppo_rocks_k05.json  hard:5 unseen:0 plain:1
#
# Episodes are variant:seed pairs from the evaluation; its JSON supplies the
# outcome captions. Output: runs/videos/<variant>_<seed>.mp4.
set -euo pipefail
cd "$(dirname "$0")/.."
model="$1"; results="$2"; shift 2
world=jezero_delta_rocks_k05
image=jezero-lab:jazzy-harmonic

xauth=()
if [ -n "${XAUTHORITY:-}" ] && [ -f "$XAUTHORITY" ]; then
    xauth=(-e XAUTHORITY=/tmp/.Xauthority -v "$XAUTHORITY:/tmp/.Xauthority:ro")
fi
in_container() {
    podman run --rm --device /dev/dri --group-add keep-groups --security-opt label=disable \
        -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro "${xauth[@]}" --ipc host \
        -v "$PWD:/repo:z" -w /repo "$image" "$@"
}

mkdir -p runs/videos
for ep in "$@"; do
    variant="${ep%%:*}"; seed="${ep##*:}"
    for ctrl in baseline policy; do
        src=baseline; extra=()
        [ "$ctrl" = policy ] && src="$model" && extra=(--rock-patch)
        in_container jezero_env/run.sh python3 -m eval.record --world "$world" \
            --controller "$src" "${extra[@]}" --variant "$variant" --seed "$seed" \
            --out "runs/videos/${variant}_${seed}_${ctrl}.mp4" 2>&1 | grep 'frames'
    done
    # Captions from the evaluation results: outcome and sim time for each side.
    read -r cap_b cap_p < <(python3 - "$results" "$variant" "$seed" <<'EOF'
import json, sys
rows = json.load(open(sys.argv[1]))
v, s = sys.argv[2], 1_000_000 + int(sys.argv[3])
out = []
for c in ('baseline', 'policy'):
    r = next(r for r in rows if r['variant'] == v and r['seed'] == s and r['controller'] == c)
    what = {'goal': 'reached the goal', 'tipped': 'tipped over', 'budget': 'ran out of time',
            'out_of_bounds': 'left the map'}[r['event']]
    out.append(f"{c.upper()}: {what} after {r['sim_time']:.0f} s".replace(' ', '_'))
print(*out)
EOF
)
    in_container ffmpeg -y -loglevel error \
        -i "runs/videos/${variant}_${seed}_baseline.mp4" -i "runs/videos/${variant}_${seed}_policy.mp4" \
        -filter_complex "[0]drawtext=text='${cap_b//_/ }':x=12:y=h-40:fontsize=24:fontcolor=white:box=1:boxcolor=black@0.6[a];[1]drawtext=text='${cap_p//_/ }':x=12:y=h-40:fontsize=24:fontcolor=white:box=1:boxcolor=black@0.6[b];[a][b]hstack=shortest=0" \
        -c:v libx264 -pix_fmt yuv420p -crf 23 "runs/videos/${variant}_${seed}.mp4"
    rm -f "runs/videos/${variant}_${seed}_baseline.mp4" "runs/videos/${variant}_${seed}_policy.mp4"
    echo "wrote runs/videos/${variant}_${seed}.mp4"
done
