#!/usr/bin/env bash
# Side-by-side chase-camera videos, baseline (left) vs policy (right), from
# identical spawns. Needs a display for rendering, so run it on the host (it
# starts the containers itself):
#
#   eval/make_demo.sh runs/ppo_rocks/checkpoints/ppo_2799888_steps.zip \
#       runs/eval_ppo_rocks_k05.json  hard:5 unseen:0 plain:1
#
# Episodes are variant:seed pairs (the evaluation's spawns); the results JSON is
# for picking them. Captions show what happened in each recording. Output: runs/videos/<variant>_<seed>.mp4.
# WORLD=... picks the world (default jezero_delta_rocks_k05).
set -euo pipefail
cd "$(dirname "$0")/.."
model="$1"; results="$2"; shift 2
world=${WORLD:-jezero_delta_rocks_k05}   # override: WORLD=... eval/make_demo.sh ...
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
        line=$(in_container jezero_env/run.sh python3 -m eval.record --world "$world" \
            --controller "$src" "${extra[@]}" --variant "$variant" --seed "$seed" \
            --out "runs/videos/${variant}_${seed}_${ctrl}.mp4" 2>&1 | grep 'frames')
        echo "$line"
        # Caption from what happened in this recording, not the evaluation JSON:
        # resets match only to micrometres across histories, and among boulders
        # that can flip an episode's outcome.
        event=$(sed -E 's/.*outcome ([a-z_]+) after.*/\1/' <<< "$line")
        secs=$(sed -E 's/.*outcome [a-z_]+ after ([0-9]+) sim-s.*/\1/' <<< "$line")
        case $event in
            goal) what="reached the goal" ;; tipped) what="tipped over" ;;
            budget) what="ran out of time" ;; out_of_bounds) what="left the map" ;;
            stuck) what="got stuck" ;; *) what="$event" ;;
        esac
        printf -v "cap_$ctrl" '%s: %s after %s s' "${ctrl^^}" "$what" "$secs"
    done
    in_container ffmpeg -y -loglevel error \
        -i "runs/videos/${variant}_${seed}_baseline.mp4" -i "runs/videos/${variant}_${seed}_policy.mp4" \
        -filter_complex "[0]drawtext=text='${cap_baseline}':x=12:y=h-40:fontsize=24:fontcolor=white:box=1:boxcolor=black@0.6[a];[1]drawtext=text='${cap_policy}':x=12:y=h-40:fontsize=24:fontcolor=white:box=1:boxcolor=black@0.6[b];[a][b]hstack=shortest=0" \
        -c:v libx264 -pix_fmt yuv420p -crf 23 "runs/videos/${variant}_${seed}.mp4"
    rm -f "runs/videos/${variant}_${seed}_baseline.mp4" "runs/videos/${variant}_${seed}_policy.mp4"
    echo "wrote runs/videos/${variant}_${seed}.mp4"
done
