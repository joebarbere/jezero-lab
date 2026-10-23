# jezero-lab: teaching a rover to drive Jezero Crater

Joe Barbere · October 10, 2026

A reinforcement-learning policy for NASA-JPL's Open Source Rover reaches the goal on 91% of held-out starts among boulders at three Jezero sites, two of which it never trained on, while tipping over in 0–4% of episodes. That beats a hand-written go-to-goal controller that can't see rocks (75%), but **not a hand-written controller given the same rock data**: it reaches 86% on the same starts, and on 600 fresh hard starts it matches the policy (88.7% vs 86.8%; the policy's edge is at most about 2 points). The pipeline learns good driving from rewards alone; it doesn't learn better driving than a simple rock-avoiding rule. With a realistic forward camera instead of a perfect map, the rule is ahead (91.8% vs 87.9% on 1,200 fresh starts, p = 0.002), and neither loses anything measurable from the limited view. Everything runs locally on one desktop (12 CPU cores, no GPU).

*Updated 2026-10-20 and 2026-10-23 with experiments 11–13 (decision log D34–D36); the original version reported only the rock-blind baseline.*

## Results

The chosen policy reaches the goal more often than the rock-blind baseline on all six test sets, by 4 to 38 points. It was picked on a separate validation set, so the test numbers aren't inflated by selection. A second, rock-avoiding baseline (experiment 11, below) closes most of that gap.

| Test set | Baseline goal rate | Policy goal rate | Baseline tip-overs | Policy tip-overs |
| --- | --- | --- | --- | --- |
| Delta, hard starts (among boulders) | 50% | **88%** | 41% | 3% |
| Delta, unseen segment (the scarp) | 69% | **78%** | 19% | 3% |
| Site A (never trained on), hard | 71% | **98%** | 17% | 2% |
| Site A, unseen routes | 79% | **90%** | 6% | 2% |
| Site B (never trained on), hard | 73% | **88%** | 17% | 4% |
| Site B, unseen routes | 96% | **100%** | 0% | 0% |

Delta: 32 fresh starts per test; sites A and B: 48 per test. "Hard" starts face a random direction up to 10 m from the route among Mars-realistic boulder fields. The baseline turns toward the goal and drives. Tiny observation noise flips 12% of individual episodes, but the rates stay within sampling noise (89% vs 91% pooled), so they describe the policy, not one exact run.

**Against a baseline that sees the rocks** (experiment 11). The baseline above ignores the terrain the policy sees. A second hand-written controller uses the same fine terrain patch to pick a rock-free arc toward the goal, tuned only on the validation set and run once on the same test spawns:

| Test set | Rock-blind baseline | Rock-avoiding baseline | Policy |
| --- | --- | --- | --- |
| Delta, hard starts | 50% | 78% | **88%** |
| Delta, unseen segment | 69% | **91%** | 78% |
| Site A, hard | 71% | 77% | **98%** |
| Site A, unseen routes | 79% | 85% | **90%** |
| Site B, hard | 73% | **88%** | **88%** |
| Site B, unseen routes | 96% | 98% | **100%** |
| **All 256 starts** | 75% | 86% | **91%** |

The policy is ahead pooled, but by 5 points rather than 16, and that difference is not statistically established (paired test, p = 0.11). Avoiding rocks also brings tip-overs down from 38 to 6, the same as the policy. Most of the original margin came from using the rock data at all, not from learning.

**Hard starts, re-tested on fresh spawns** (experiment 12). The policy's apparent lead on hard starts was re-measured on 600 new spawns, with the test size fixed in advance:

| Hard starts, fresh spawns | rock-avoiding baseline | policy | tip-overs (avoid / policy) |
|---|---|---|---|
| delta (200) | **91%** | 84% | 10 / 12 |
| site A, held-out upper (200) | 80% | **86%** | 5 / 7 |
| site B, held-out margin (200) | **94%** | 90% | 4 / 6 |
| **pooled (600)** | **88.7%** | 86.8% | 19 / 25 |

No advantage (p = 0.36): the gap is −1.8 points, and at most +1.7 for the policy. The same policy that scored 88–98% on the small test sets scores 84–90% here, so those numbers were partly luck.

**With a realistic camera** (experiment 13). Both controllers see only what a forward depth camera has seen (87° field of view, 4 m range, nothing behind rocks, noise, 0.2 s delay). Six new policies were trained on that view; the best on validation was tested once against the rule, retuned on the same view, on 1,200 fresh starts:

| Fresh spawns (200 each) | rock-avoiding rule | policy | tip-overs (rule / policy) | timeouts (rule / policy) |
|---|---|---|---|---|
| delta hard | 90% | 90% | 13 / 6 | 6 / 15 |
| delta unseen | **93%** | 68% | 2 / 5 | 12 / 59 |
| site A hard | 87% | **88%** | 5 / 10 | 21 / 14 |
| site A unseen | 88% | **94%** | 10 / 5 | 13 / 7 |
| site B hard | **94%** | 90% | 2 / 13 | 9 / 7 |
| site B unseen | 97% | **98%** | 0 / 1 | 6 / 3 |
| **pooled (1,200)** | **91.8%** | 87.9% | 32 / 40 | 67 / 105 |

The rule is ahead by 3.8 points (95% interval 1.5 to 6.2, p = 0.002), mostly on the delta's unseen segment, where the policy often runs out of time. Neither controller loses anything measurable from the camera compared with the perfect map.

## How it works

![Pipeline: HiRISE terrain → Gazebo worlds → Gym environment → PPO + terrain CNN → validation → test](media/pipeline.png)

Real orbital elevation data becomes simulator worlds with Mars-like boulder fields. A Gymnasium environment steps the simulator directly, and PPO trains a small CNN policy across six sites; validation picks the seed, and the test sets only report.

## How we got there

Four changes made the difference: varied training worlds, a decaying learning rate, a CNN that reads the terrain as grids, and training on six sites. Every input added to the observation as raw numbers failed. Each experiment ran three or more random seeds, because one seed can land 20 points away from another.

| # | What changed | Result (delta, hard / unseen goal rate) | Decision |
| --- | --- | --- | --- |
| — | Hand-written baseline: turn to the goal and drive | 50% / 69% | Reference |
| 1 | Rocks everywhere, start and goal anywhere on the map, a held-out region | Large gain over route-only training | Adopted |
| 2 | Body sensors (wheel speeds, suspension angles) | No gain | Dropped |
| 3 | Look-ahead terrain profiles along the goal bearing | Worse | Dropped |
| 4a | Penalty for getting stuck | No gain | Dropped |
| 5 | Learning rate decaying to zero, 1.5M steps | 58% / 66% | Adopted |
| 6 | Look-ahead again, at full length | 47% / 49% | Dropped |
| 7 | Terrain CNN instead of a flat network | 67% / 66% | Adopted |
| 8 | Train on six sites along Perseverance's route, not one | 71% / 76% | Adopted |
| 9 | Six seeds; choose the seed on a validation set | Chosen policy 88% / 78% | Adopted procedure |
| 10 | Batch doubled to 6,144 samples per update | Per-seed average 85% vs 79%; worst seed 74% vs 64% | Adopted |
| 11 | A second baseline that avoids rocks with the same terrain data | 78% / 91%; pooled over all tests 86% vs the policy's 91% (p = 0.11) | Headline revised |
| 12 | Hard starts re-tested on 600 fresh spawns | Rock-avoiding 88.7% vs policy 86.8% (p = 0.36) | No policy advantage |
| 13 | Both controllers see through a realistic camera; policy retrained | Rule 91.8% vs policy 87.9% on 1,200 starts (p = 0.002) | Rule ahead |

Rows 5–8 and 10 are means over seeds on 32 fresh starts; row 9 is the single chosen policy. Earlier rows were short 400k-step screens whose exact numbers aren't comparable, so only the direction is shown. The full reasoning, with numbers and an explainer for each step, is in the [decision log](decision_log.md) (D1–D33).

## What we learned

Most of the progress came from testing honestly, not from clever additions.

- **One seed proves nothing.** Seeds of the same recipe differed by up to 28 points, so every claim rests on three to six seeds.
- **The right information needs a usable form.** Look-ahead and body sensors failed as raw numbers; a CNN made the existing terrain grids pay off.
- **Short screens are biased.** 400k-step tests punished bigger observations; full-length runs settled those questions.
- **One test set is a sample of one.** New held-out sites showed the old "unseen" segment was the hardest place on the map, not a typical one.
- **Keep choosing and reporting separate.** Validation picked the seed; the test sets were only used to report. Validation could pick seeds (big differences) but not checkpoints (small ones).
- **Report rates, not episodes.** Among boulders a fifth-decimal difference flips single episodes; success rates held under perturbation.
- **A baseline is only as good as what it can see.** The first baseline was blind to rocks; a hand-written one given the same rock data matched the policy.
- **Re-test the comparison that caught your eye.** The biggest per-set lead (hard starts) vanished on fresh spawns: it had been picked out because it was large.
- **Steadier updates beat more updates.** Doubling the batch cut seed spread at the same cost.

## Side work

Two pieces reach beyond the RL policy: fixes offered back to the OSR project, and a natural-language demo.

- **OSR upstream.** Building the simulator turned up real bugs in [osr-rover-code](https://github.com/nasa-jpl/osr-rover-code)'s Gazebo setup. [PR #230](https://github.com/nasa-jpl/osr-rover-code/pull/230), which applies wheel friction correctly, was merged on 2026-10-20, and jezero-lab now pins that commit. [Issue #229](https://github.com/nasa-jpl/osr-rover-code/issues/229) reports a wheel-radius mismatch that makes the sim drive about 9% fast. [Issue #228](https://github.com/nasa-jpl/osr-rover-code/issues/228) proposes moving to Gazebo Harmonic; a working port is ready on a branch, held until the maintainer answers. No replies on the issues yet.
- **Talk to the rover.** [ROSA](https://github.com/nasa-jpl/rosa), NASA-JPL's LLM agent for ROS, now drives the simulated rover with Claude: "drive to the nearest waypoint", "which controllers are running?". Its rover tools stop on their own past 25° of tilt. The tools are tested against the simulator; a conversation with the agent hasn't been tried yet (it needs an API key).
- **Not adopted after review:** GRPO and its credit-assignment fixes (built for LLMs; our small critic already does that job), Jev (a classification API, not a controller) and SPOC-Lite (a dormant 2017 terrain classifier for camera images).

## Next steps

The first question now is whether learning can beat the rock-avoiding controller at all; making every seed good (the batch result) matters only if it can.

- [ ] **Measure everything against the rock-avoiding baseline** (D34–D36): a training change only counts if it beats that controller, not the rock-blind one, under the camera view (D36).
- [ ] **The delta's unseen segment** is where the policy loses most (68% vs the rule's 93% with the camera; 59 timeouts in 200): look at what it does there before trying anything bigger.
- [ ] **Push the batch further:** 4× the original (12,288 samples per update), six seeds, compared with the 1× and 2× arms (about 10 h).
- [ ] **Pick from the new seeds too:** score the six bigger-batch seeds on validation; replace the chosen policy if one beats it.
- [ ] **Try the ROSA agent in conversation** ([rosa/](../rosa/README.md)) with an API key, then shape it into a small example for the OSR community.
- [ ] **Follow up upstream:** send the Harmonic port once #228 gets a reply.
- [ ] **Remaining training ideas:** a critic that sees simulation-only facts during training (wheel contacts, slip), and a tilt-rate penalty aimed at the last tip-overs.
- [ ] **Toward the real rover:** add an IMU to the physical OSR (the policy uses roll and pitch) and plan a sim-to-real test.

## Sources

- [jezero-lab repository](https://github.com/joebarbere/jezero-lab): code, [PLAN.md](../PLAN.md) (every measured result) and the [decision log](decision_log.md) (D1–D33 with explainers)
- [NASA-JPL Open Source Rover](https://github.com/nasa-jpl/osr-rover-code)
- [USGS Mars 2020 HiRISE terrain (1 m DTM)](https://planetarymaps.usgs.gov/mosaic/mars2020_trn/HiRISE/) and [Perseverance waypoints (NASA MMGIS)](https://mars.nasa.gov/mmgis-maps/M20/Layers/json/M20_waypoints.json)
- [ROSA](https://github.com/nasa-jpl/rosa)
