#!/usr/bin/env bash
# Experiment 13 (PLAN.md): six camera-sensing seeds, two at a time, then
# validation (eval.exp13 val) and, if the gate passes, the test (eval.exp13 test).
cd "$(dirname "$0")/.."
SITES="jezero_delta_rocks_k05_full jezero_floor_rocks_k05_full jezero_seitah_rocks_k05_full \
   jezero_upper_rocks_k05_full jezero_margin_rocks_k05_full jezero_rimwest_rocks_k05_full"
IMG=jezero-lab:jazzy-harmonic
train() {  # seed
  local s=$1 name=exp13_cam_s$1
  [ -f "runs/$name/final.zip" ] && return
  podman run --rm --name "jz-$name" -v "$PWD:/repo:z" -w /repo $IMG timeout 28800 \
    jezero_env/run.sh python3 -u -m train.train --name "$name" --seed "$s" --policy cnn \
    --world $SITES --rock-patch --goal-mode map --envs 6 --segments 0 1 2 \
    --random-heading --spawn-jitter 10 --sensing camera --steps 1500000 \
    --n-steps 1024 --batch-size 1024 --lr-schedule linear > "runs/$name.log" 2>&1
}
for pair in "0 1" "2 3" "4 5"; do
  set -- $pair; train $1 & train $2 & wait
done
for s in 0 1 2 3 4 5; do [ -f runs/exp13_cam_s$s/final.zip ] || { echo "seed $s missing"; exit 1; }; done
[ -f runs/exp13_val.json ] || podman run --rm --name jz-exp13-val -v "$PWD:/repo:z" -w /repo $IMG timeout 28800 \
  jezero_env/run.sh python3 -u -m eval.exp13 val --out runs/exp13_val.json > runs/exp13_val.log 2>&1
podman run --rm --name jz-exp13-test -v "$PWD:/repo:z" -w /repo $IMG timeout 43200 \
  jezero_env/run.sh python3 -u -m eval.exp13 test --val runs/exp13_val.json --out runs/exp13_test.json \
  > runs/exp13_test.log 2>&1
echo "exp13 done"
