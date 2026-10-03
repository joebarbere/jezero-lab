# jezero-lab: training decision log

Every decision that shaped how the rover's driving policy is trained, in the order it
was made, with the numbers behind it and an explainer for anyone new to
reinforcement learning (RL). The goal is to show not just *what* was decided but
*how* evidence led there, including the mistakes.

[PLAN.md](../PLAN.md) has the full engineering detail; this log is the story of the
decisions.

**Contents**

- [How to read this log](#how-to-read-this-log)
- [Glossary](#glossary)
- [Part 1: Building a simulator you can learn in](#part-1-building-a-simulator-you-can-learn-in) (D1–D9)
- [Part 2: First policies, and making the task worth learning](#part-2-first-policies-and-making-the-task-worth-learning) (D10–D16)
- [Part 3: Measuring properly](#part-3-measuring-properly) (D17–D21)
- [Part 4: Experiments](#part-4-experiments) (D22–D28)
- [Lessons in one page](#lessons-in-one-page)

---

## How to read this log

- **Success rate** is the fraction of episodes in which the rover reached the goal.
  "16 spawns" means 16 test episodes; with so few, one episode is 6 percentage points,
  so 50% vs 56% is no real difference.
- **Held-out** spawns are start/goal situations the policy never trained on. *hard*:
  the training route, but starting off-route and facing a random direction.
  *unseen*: a stretch of terrain (segment 3) the policy never trained in.
- **The baseline** is a hand-written controller ("turn toward the goal and drive"). A
  learned policy is only interesting if it beats something this simple.
- **Seeds**: training uses random numbers; the *seed* fixes them. Two runs with
  different seeds are like two students taught the same course. From D19 on, every
  comparison uses 3 seeds and reports the mean and the range.
- **400k / 1.5M** are training lengths in environment steps (one step = 0.2 s of
  simulated driving).

## Glossary

| Term | Meaning |
|---|---|
| **Environment (env)** | The simulated world the policy acts in. Each *step*, the policy sends an action and gets back an observation, a reward, and whether the episode ended. |
| **Episode** | One attempt: from a start position until the goal is reached, the rover tips over, or the time budget runs out. |
| **Observation** | What the policy sees each step (here: where the goal is, tilt, speed, nearby terrain heights...). |
| **Action** | What the policy decides each step (here: speed and turn, each in −1..1). |
| **Reward** | A number scoring each step. The policy learns to maximise the sum of rewards. |
| **Policy** | The learned mapping from observation to action (here a small neural network). |
| **PPO** | Proximal Policy Optimization, the RL algorithm used. It collects a batch of experience with the current policy, then nudges the policy toward actions that turned out better than expected, but only a little per update ("proximal"), which keeps training stable. |
| **Termination vs truncation** | *Terminated*: the episode ended for a reason that's part of the task (goal reached, tipped over). *Truncated*: it was cut off by a time limit. The difference matters for how the algorithm values the final state. |
| **Real-time factor (RTF)** | Simulated seconds per wall-clock second. RTF 16 = the simulator runs 16× faster than reality. |
| **Generalisation** | Doing well in situations not seen in training, the whole point of learning rather than memorising. |
| **Reward shaping** | Adding extra reward terms to guide learning (e.g. a penalty for being stuck). Powerful and easy to get wrong. |

---

## Part 1: Building a simulator you can learn in

### D1. Use simple collision shapes, not the detailed 3D model
**Situation.** NASA-JPL's rover model used full-detail 3D meshes for collision (≈1.4
million triangles; the body alone ≈220k). The simulator ran at **0.25× real time**,
physics-bound on one CPU core.

**Decision.** Collide with cylinders (wheels) and boxes (body); keep the meshes only
for looks.

**Numbers.** Classic Gazebo became several times faster; on Gazebo Harmonic with an
unthrottled world, ~**16× real time** at a 5 ms physics step.

> **Explainer: why speed is the first thing to fix.** RL learns from experience, and
> it needs a lot of it: the runs in this log use 400,000–1,500,000 steps each, i.e.
> 22–83 hours of simulated driving. At 0.25× real time one run would take *two weeks
> of wall-clock time*; at 16× it takes about an hour per simulator. Throughput isn't a
> nicety in RL; it decides which experiments are possible at all. Simplified collision
> geometry is the standard trade: contact physics barely changes, cost drops by orders
> of magnitude.

### D2. Run the physics with a 5 ms step, unthrottled
**Numbers.** Real-time factor scales almost linearly with step size: 1 ms → 3.1×, 2 ms
→ 6.2×, 4 ms → 11.7×, 8 ms → 22.4×. Forward speed was unchanged across all of them;
rotate-in-place got ~7% faster at 5 ms. 5 ms also divides the 100 Hz controller period
evenly.

> **Explainer: fidelity vs speed.** A coarser physics step means fewer calculations per
> simulated second, but the simulation gets less exact. The rule is to coarsen until a
> behaviour you care about changes, then stop. Here, driving speed stayed exact and
> turning changed a little, an acceptable cost for 16× throughput.

### D3. Step the simulator inside the training process, not through ROS
**Situation.** Commands through the robot software stack (ROS) are asynchronous, so at
16× real time a few milliseconds of message latency becomes many simulated steps of
lag between "decide" and "act".

**Decision.** Run Gazebo inside the Python training process; apply each action, then
advance exactly N physics steps.

**Numbers.** Identical driving to the ROS stack (0.300 m/s; −111° vs −112°
rotate-in-place).

> **Explainer: an environment must be a clean function.** RL theory assumes each step
> is "observe, act, the world moves forward a fixed amount, observe again". If the
> action arrives late or the world advances a variable amount, the policy is learning
> from noise it can't see. Deterministic, synchronous stepping makes the environment
> behave like the textbook assumes, and makes bugs reproducible.

### D4. Reset by deleting the rover and spawning a fresh one
**Situation.** Three ways to reset between episodes, tried in order:

| Approach | What happened |
|---|---|
| Gazebo's built-in world reset | The hook that applies commands stopped firing: the rover reset, then ignored every action. |
| Teleport the rover home, zero its joints | Velocity commands persisted forever (the rover hovered, pinned in place); without them it kept its momentum: after a spin, the next episode drifted 8 cm in 2 s. |
| **Delete and respawn** | After three different histories, the next episode was identical: **0.000 mm** difference. |

> **Explainer: why resets must be clean.** Each episode is supposed to be an
> independent trial from a known start. If the previous episode leaks into the next
> (momentum, joint angles), the policy is graded on situations it didn't cause, and the
> reward signal gets noisier. This was the bug in the 2020 version of this project.
> Always test resets explicitly: "after very different histories, does the same
> episode replay the same?"

### D5. Let the rover settle before an episode starts
**Situation.** A soak test showed **27 of 120 episodes** ending in a "tip-over" **on
step 1**, at 40–46° pitch. Given 3 more seconds, 26 of them settled to ~10°, just the
slope. The rover was still rocking from being dropped in when the episode began
(slow swing at Mars gravity, 0.38 g).

**Decision.** Settle until the rover is still (speed and spin near zero), and respawn
with another heading if it never settles.

**Numbers.** Step-1 "tip-overs" dropped to **1 of 120**; in 1,000 random episodes: 0.

> **Explainer: bogus terminations poison learning.** A termination tells the
> algorithm "this state is terrible, avoid what led here". If episodes end for reasons
> unrelated to the policy's actions, it gets punished for nothing and learns
> superstitions. Always check *when* and *why* episodes end before training on them.

### D6. Observe the world from the rover's point of view
**Decision.** The observation gives the goal *relative to the rover* (ahead/left and
distance), the heading error, tilt, the rover's own velocities, the previous action,
and a grid of terrain heights around it, relative to the rover's own height.

> **Explainer: egocentric observations generalise.** If the policy saw raw map
> coordinates ("goal at x = 93"), it would have to learn every location separately.
> Expressed relative to itself ("goal 20 m ahead, slightly left; ground rises in front
> of me"), the same lesson applies anywhere on the map. Choosing what the policy sees is
> one of the biggest levers in RL, often bigger than the algorithm.

### D7. The reward: progress, a goal bonus, a time cost, penalties, a time budget
**Decision** (ported from the 2020 AWS challenge reward and extended):

| Term | Value | Purpose |
|---|---|---|
| Progress | +1 per metre closer to the goal (−1 per metre further) | dense signal every step |
| Goal bonus | +100 × fraction of the time budget left | the actual objective, and "sooner is better" |
| Time cost | −0.01 per step | don't dawdle |
| Tip-over | −50, episode ends | safety |
| Leave the map | −50, episode ends | stay in bounds |
| Budget | episode cut off at 2.5× the straight-line driving time | finite episodes |

> **Explainer: dense vs sparse rewards.** A goal bonus alone is *sparse*: a random
> policy almost never reaches the goal, so it almost never learns anything. The
> progress term is *dense*: every step tells the policy whether it moved the right way.
> The art is making the dense term point at the real goal. "Distance closer" does; a
> reward per step survived would teach survival, not arrival (see D18 for evidence from
> the 2020 contestants).

### D8. Build a hand-written baseline before training anything
**Decision.** "Turn toward the goal, drive, slow down when badly misaligned."

**Numbers.** It reached every goal on Perseverance's 235 m route, within ~1 m, using
~40% of the time budget.

> **Explainer: always have a baseline.** Without one, "the policy reaches the goal 70%
> of the time" means nothing. Here the baseline turned out to be nearly perfect on
> smooth terrain, which told us the task was too easy (D12) *before* weeks went into
> training. A strong simple baseline is the most useful number in an RL project.

### D9. Soak-test the environment before training
**Numbers.** 1,000 random-agent episodes: no hangs, memory flat after warm-up (+33 MB,
then +2 MB over 900 episodes), ~82 env steps/s per process.

> **Explainer.** Training runs for hours across many processes. A memory leak, a rare
> crash, or a hang in the environment will kill a run long after you've stopped
> watching. Testing the environment alone first is cheap insurance.

---

## Part 2: First policies, and making the task worth learning

### D10. PPO with a small network, normalised inputs, parallel simulators
**Decision.** Stable-Baselines3 PPO; a 2×128-unit network; `VecNormalize` (rescale
observations and rewards); 5–6 simulators collecting experience in parallel; a short
**smoke test** (120k steps) before the first long run.

**Numbers.** Smoke test: mean episode reward −15.8 → +20.3, goal rate 0 → 10% in
120k steps: learning was happening, so the long run was worth it.

> **Explainer.**
> - **Why PPO**: robust and forgiving of hyperparameters; the default choice for
>   continuous control. (D18 has evidence: in 2020, only PPO ever reached the goal.)
> - **Why a small network**: 142 numbers in, 2 out. Networks of 64–256 units are
>   standard for low-dimensional control. Bigger isn't free capacity; it's more to
>   learn.
> - **Why normalise**: inputs range from metres to radians to 0–1 flags; rewards from
>   −50 to +100. Neural networks learn far better when inputs and targets are on similar
>   scales.
> - **Why a smoke test**: check the reward goes up at all before burning hours.

### D11. The smooth terrain was too easy: add Mars-like boulders
**Situation.** On the smooth 1 m elevation map, the baseline scored 100% everywhere at
0.30 m/s, flat out in a straight line. The first policy (`ppo_v1`) scored 100% / 92% /
**42%** (plain / hard / unseen) and drove slower (0.267 m/s).

**Decision.** Add rocks sized by the **Golombek–Rapp** model used for Mars landing
sites (k = 0.05 → 2,544 rocks; measured coverage 2.0% vs 1.9% predicted). A steep
crater-rim world was also built, but steepness alone isn't hard for this simulated
rover.

**Numbers.** Rocks behave like rocks (0.25 m: driven over; 0.40 m: climbed at 24°;
0.70 m: blocked; 1.35 m: flipped the rover). The baseline drops to 75% / **42%** / 83%.

> **Explainer: the task has to need learning.** RL is worth using when simple rules
> fail. If a three-line controller is already optimal, no amount of training can show
> an improvement. Making the task realistically hard (here: obstacles a straight-line
> driver hits) is part of the experimental design, not an afterthought.

### D12. Bake the rocks into the terrain instead of adding them as objects
**Numbers.** 2,544 rock objects: **0.40×** real time. One object with 2,544 spheres:
1.6×. Rocks baked into a 12.5 cm terrain grid: **12.8×**, the same as no rocks.

> **Explainer: performance is a design constraint.** Physics engines check every object
> against every nearby object; thousands of objects are expensive. Terrain is checked
> only near the rover. Same physical effect, 30× cheaper, which is the difference
> between an experiment that runs overnight and one that doesn't run.

### D13. The policy memorised its route: train on random goals
**Situation.** `ppo_v1` trained on 3 fixed route segments scored **42%** on a segment
it had never seen (baseline 100%). In boulders, `ppo_rocks` stalled at the *same spot*
in several unseen episodes (39.2 m from the goal): plotting the paths showed it veering
north-east off the goal bearing and stopping at a scarp.

**Decision.** Pick a random start and goal anywhere along the training route each
episode, instead of 3 fixed segments.

> **Explainer: overfitting in RL.** A policy trained on a handful of situations can
> learn "what to do here" instead of "how to drive". The cure is variety in training
> and **held-out evaluation**: test on situations deliberately kept out of training. A
> policy that scores well on its training route but poorly on new terrain has
> memorised, not learned. (Random route goals weren't enough on their own; D22 goes
> further.)

### D14. Make "impossible" bugs impossible: the sampler crash
**Situation.** At 2.7M steps a training run crashed and hung: the random-goal picker
required goals 1.5 m from any rock; only **15%** of route points qualified, so 200 tries
failed about once per 1,000 resets, and over thousands of resets that was certain.

**Decision.** 10× more tries and a fallback that can't fail. (The fix also lowered the
clearance to 1.0 m, which silently made the task harder; see D19.)

> **Explainer: rare is certain at scale.** A training run does tens of thousands of
> resets. Anything that fails "one in a thousand" *will* happen. Code on the reset path
> needs a guaranteed fallback, not just "usually works".

### D15. Profile before optimising: skipping the GPU
**Situation.** The idea: use the AMD GPU (ROCm) to speed up training.

**Numbers.** A PPO iteration was **9.4 s collecting experience and 0.18 s updating the
network: the update is 2%** of training time. An infinitely fast GPU would make
training ≤1.02× faster. More CPU threads for the network made it *slower*.

> **Explainer: find the bottleneck first.** In RL with a CPU physics simulator, the
> simulator is almost always the bottleneck, not the neural network, especially a small
> one. GPUs shine when the network is large or when the *simulator* runs on the GPU
> (e.g. Isaac). Measuring took minutes; the GPU setup would have taken hours for
> nothing.

### D16. Watch policies, not just numbers: path plots and video
**Decision.** Plot each held-out episode's path over the terrain, and record chase-camera
video.

**What it revealed.** The unseen-segment failures weren't random: the policy drove ~10 m
north-east to nearly the same point and stopped at a scarp, every time.

> **Explainer.** A success rate tells you *that* something fails; a picture tells you
> *how*. Many RL bugs (a policy exploiting a reward loophole, a sensor that's always
> zero) are obvious the moment you watch an episode and invisible in the averages.

---

## Part 3: Measuring properly

### D17. Log reward components and evaluate during training
**Decision.** Log each reward term separately per episode; evaluate every checkpoint on
held-out spawns *alongside* training, with the baseline as a reference line.

**What it revealed.** Held-out performance **peaked mid-training, then fell**:
`ppo_v1` unseen 62% at 700k → 38% at 1.0M; `ppo_rocks` best ~1.1M, then unstable
(hard 0% with 75% tip-overs at 2.6M). Evaluating only the final checkpoint had
understated both runs.

> **Explainer.** The last checkpoint isn't necessarily the best. Training keeps
> optimising the *training* reward, which can drift away from what you actually want
> (e.g. generalising). Track the metric you care about over time, then pick checkpoints
> by it, on spawns separate from the ones you report.

### D18. The clock: a theoretically correct fix that hurt
**Situation.** The goal bonus depends on how much time is left, but the policy couldn't
see the clock, so technically the reward isn't a function of what the policy observes.
Fix: add "fraction of time budget left" to the observation.

**Numbers** (training goal rate at 400k, 3 seeds): **no clock 69 / 92 / 75%; with
clock 6 / 13 / 16%.** (A fourth clock run also failed.)

**Decision.** Clock off. A second fix, a constant goal bonus instead of a decaying one,
was also tried (D21): no better.

> **Explainer: theory guides, experiments decide.** The textbook says the policy should
> observe everything the reward depends on (the *Markov property*). Here, showing the
> clock made training much worse, probably because the policy learned that a late finish
> earns almost no bonus, weakening the pull to finish (it matched the symptom: driving
> most of the way, then stalling). The lesson isn't "ignore theory"; it's that every
> change, even a "correct" one, gets measured.

### D19. Seed variance: most of a detective story was luck
**Situation.** Three "control" runs on new code all failed (training goal rate 7–21%)
where the old code had succeeded (60–79%). The investigation:

| Step | Finding |
|---|---|
| Old checkpoints on new code | Still good (hard 50–75%): the environment wasn't broken. |
| Evaluator cross-talk? | Tested directly with a positive control: impossible across containers. |
| Rerun the old commit | Learned again (81–91%). |
| Bisect | One change (a reset rewrite) separated success from failure. |
| But... | That change altered the rover's rest position by **~3 micrometres**. In a chaotic training loop, enough to send a whole run elsewhere. |
| Multiple seeds | Old reset: 69 / 92 / 75%. New reset: 8 / fail / **87%**. The same code learned or failed depending on the seed. |

One confound along the way was a real mistake: the goal clearance had been lowered from
1.5 to 1.0 m during the D14 crash fix, which put goals among rocks.

**Decisions.** Revert to the more reliable reset (4 of 4 runs learned vs 1 of 4).
**From then on: at least 3 seeds per setting**, short screening runs (400k steps, where
learning curves separate), full-length runs only for survivors.

> **Explainer: seed variance is the most common trap in RL.** Training is a long chain
> of random choices; tiny differences compound. The same code can succeed or fail by
> seed alone, and published RL results are often wrong for exactly this reason. One run
> per setting is an anecdote. Use several seeds, report the spread, and be suspicious of
> any comparison where the difference is smaller than the spread between seeds.

### D20. Change one thing at a time, and confirm "off" is untouched
**Practice** (every experiment from here): each new feature is an option; before any
experiment, a short run with the option *off* must reproduce the previous baseline's
episodes **exactly**.

> **Explainer.** In D19, three changes landed in one run and it took a day to untangle.
> If a new feature subtly changes behaviour even when switched off, every comparison
> after it is contaminated. "Reproduces exactly when off" is a cheap, strict check that
> isolates each change.

### D21. A constant goal bonus: no better
**Numbers** (training goal rate, 300–400k, 3 seeds): constant 70 / 92 / **13%** vs
decaying 68 / 90 / 73%. Two seeds matched (and were slightly faster), one failed.

**Decision.** Not adopted: no evidence of improvement.

> **Explainer: "not worse" isn't "better".** A change has to earn its place. With
> 3 seeds and one failure, you can't tell bad luck from a real effect, and without
> evidence of a gain, the simpler status quo wins.

---

## Part 4: Experiments

All experiments: 3 seeds, 400k steps unless noted, evaluated on the same held-out
spawns on the map-wide boulder world (16 per variant per seed). Baseline: **hard 62%,
unseen 81%**.

### D22. Experiment 1, varied worlds: adopted ✓
**Change.** Rocks over the whole map; start and goal **anywhere**, except a **held-out
region** around the unseen segment that training never enters.

| | hard | unseen |
|---|---|---|
| control (route goals), mean (range) | 35% (12–50) | 46% (31–56) |
| **varied worlds**, mean (range) | **52%** (6–81) | **73%** (62–88) |

Every varied-worlds seed beat every control seed on unseen (35/48 vs 22/48 episodes).

> **Explainer: generalisation comes from variety.** Training on start/goal pairs all
> over the map forces the policy to learn "drive to *a* goal", not "drive *this* route".
> The held-out region is what makes the claim testable: if training had seen it, a good
> score there could still be memorisation. This is the same idea as *domain
> randomisation* in robotics: vary everything you can in simulation so the policy can't
> latch onto specifics.

### D23. Experiment 2, proprioception: no measurable gain
**Change.** 16 new inputs: wheel speeds, slip, tilt rates, steering and suspension
angles, time since last progress.

| | hard | unseen |
|---|---|---|
| varied worlds | 52% | 73% |
| + proprioception | 52% (38–75) | 67% (50–75) |

**Decision.** Not adopted at 400k.

> **Explainer.** More information isn't automatically better: every input is something
> the network must learn to use, which costs experience. See D24 for why this result
> may be unfair to it.

### D24. Experiment 3, look-ahead: worse at 400k, and a flaw in the method
**Change.** 40 new inputs: terrain heights along 5 rays toward the goal, out to 12 m.
At the stall point it clearly showed the scarp (+3.5 m in 12 m straight ahead, +2.3 m
on a ray 40° right).

| | hard | unseen | training goal rate at 400k |
|---|---|---|---|
| varied worlds | 52% | 73% | 66–68% |
| + look-ahead | 25% (0–50) | 38% (19–56) | 49–52% |

**Decision.** Not adopted at 400k, and the screening method itself is biased: **a short
screen penalises any change that adds inputs**, because bigger observations learn
slower. Experiments 2 and 3 should be retested at full length.

> **Explainer: know what your test can and can't show.** The training curve said
> "slower", not "wrong": it was still climbing. A fixed short budget measures *learning
> speed*, not final quality. Short screens are fair for changes that keep the
> observation the same (like reward changes) and unfair for changes that make it bigger.

### D25. Experiment 4a, stuck penalty: worse on hard
**Change.** Training only: if the rover goes 30 s without getting 0.25 m closer, end
the episode with −50 (negative on purpose: ending with 0 would let the policy escape the
time cost by getting stuck).

| | hard | unseen |
|---|---|---|
| varied worlds | 52% | 73% |
| + stuck penalty | 25% (6–44) | 62% (25–88) |

The rule fired a lot (up to 38–49% of training episodes), so it probably also punished
legitimate careful manoeuvring among rocks.

**Decision.** Not adopted.

> **Explainer: reward shaping can backfire.** A shaping term encodes your guess about
> *how* to solve the task. If the guess is wrong for some situations (here: slow,
> careful driving among boulders looks like being stuck), the policy is pushed away from
> exactly the behaviour you need. Shape lightly, and measure the outcome you care about,
> not the shaping term.

### D26. Train the best setup longer: no gain, and an unstable policy
**Change.** Varied worlds, resumed from 400k to 1.5M steps.

| Checkpoint | hard (mean) | unseen (mean) |
|---|---|---|
| 400k | 52% | 73% |
| 600k | 31% | 52% |
| 1.0M | 56% | 60% |
| 1.4M | 37% | 69% |

One seed's unseen score went **88% → 12% → 81%** over consecutive checkpoints.

> **Explainer: more training isn't always better.** With a constant learning rate, PPO
> keeps making full-size updates forever; late in training those can knock a good policy
> around as easily as improve it. Swings far larger than the evaluation noise are the
> signature. This pointed at the learning rate, not at more data.

### D27. Experiment 5, a decaying learning rate: adopted ✓
**Change.** Same as D26, but the learning rate decays linearly from 3e-4 to 0 between
400k and 1.5M steps.

| Checkpoint | constant LR (hard / unseen) | **decaying LR (hard / unseen)** |
|---|---|---|
| 1.2M | 40 / 69% | **48 / 75%** |
| 1.4M | 37 / 69% | **48 / 79%** |

Two seeds beat the baseline on unseen (88%, 94% vs 81%); one seed's unseen score
climbed steadily 69 → 81 → 81 → 81 → 88 → 88%.

> **Explainer: learning-rate schedules.** Early in training, big steps explore quickly;
> late in training, small steps refine without breaking what works. Decaying the
> learning rate to zero is a standard PPO setting (it wasn't used at first), a good
> reminder to check defaults against common practice. *Hard* (48% vs the baseline's
> 62%) is still the weak spot.

### D28. A larger, unbiased evaluation of the best policies
**Change.** The final (1.5M) policies of D26 (constant learning rate) and D27 (decaying),
evaluated on **48 spawns per variant** instead of 16, and reported separately on the
**32 spawns never used for any decision**.

**Numbers** (fresh spawns only; 3 seeds pooled = 96 episodes per variant):

| | hard | unseen |
|---|---|---|
| baseline | 50% (16/32) | 69% (22/32) |
| constant LR | 44% (42/96) | 53% (51/96) |
| **decaying LR** | **58%** (56/96) | **66%** (63/96) |
| decaying LR, seed 0 / 1 / 2 | 44 / 66 / 66% | 53 / 75 / 69% |

On *hard* the decaying-LR policies tip over in 17–23% of episodes vs the baseline's 38%.

**Conclusions.**
1. The decaying learning rate is confirmed: clearly better than constant on fresh spawns.
2. The learned policy is now **level with the baseline** (slightly ahead on hard,
   slightly behind on unseen; neither gap is meaningful at this sample size) and
   **drives more safely** among boulders.
3. The D27 headline "two seeds beat the baseline on unseen (88%, 94%)" **did not hold
   up**: on more spawns those seeds score 75% and 69%, level with the baseline.

> **Explainer: selection bias and small samples, caught in the act.** The decaying
> learning rate was chosen because it looked good on the first 16 evaluation spawns,
> so those spawns flatter it; and 16 episodes is a tiny sample, where 88% vs 81% is one
> or two episodes. On 32 fresh spawns, the impressive per-seed numbers shrank back to
> the baseline's level, while the *arm-level* conclusion (decaying beats constant)
> survived. This is why ML keeps a *validation* set (for choosing) separate from a
> *test* set (for reporting), and why a result should be stated only as strongly as
> its sample allows.

### D29. Experiment 6, look-ahead at full length: still worse
**Change.** Look-ahead (D24) given the fair test: its 400k checkpoints continued to 1.5M
with the decaying learning rate, mirroring D27 exactly.

**Numbers** (32 fresh spawns, 3 seeds pooled):

| | hard | unseen |
|---|---|---|
| baseline | 50% | 69% |
| best so far (D27, no look-ahead) | **58%** | **66%** |
| + look-ahead (seed 0 / 1 / 2) | 28 / 56 / 56% | 41 / 31 / 75% |
| + look-ahead, pooled | 47% | 49% |

Its checkpoint curve never caught up either (it started from weaker 400k checkpoints,
25% / 38%, but full training didn't close the gap).

**Decision.** Not adopted. The information is useful (it clearly shows the scarp), but a
flat network fed 40 raw height numbers doesn't learn to use it. Next: give it a better
*form* (a small CNN over terrain grids, or summary features like the steepest slope per
ray).

> **Explainer: information vs representation.** Having the right information in the
> observation isn't enough; the network has to be able to use it. A fully connected
> network sees 40 heights as 40 unrelated numbers and must discover from scratch that
> neighbouring samples belong together. A convolutional layer builds that structure in
> ("nearby cells relate; the same pattern means the same thing anywhere"), which is why
> image-like inputs almost always go through CNNs. When an input that *should* help
> doesn't, ask whether the model can exploit its shape before concluding it's useless.
> And D24's "it just needs longer" was a reasonable hypothesis that this experiment
> tested and rejected.

### D30. Experiment 7, a terrain CNN: the first clear win over the baseline ✓
**Change.** A small convolutional network reads each terrain grid (7×7 coarse, 9×9 fine,
optionally the 5×8 look-ahead) as a 2-D grid instead of loose numbers; same inputs,
same schedule as D27. Two arms, 3 seeds each: **7a** CNN on the existing inputs, **7b**
CNN + look-ahead.

**Numbers** (32 fresh spawns):

| | hard | unseen |
|---|---|---|
| baseline | 50% (16/32) | 69% (22/32) |
| flat network (D27) | 58% (56/96) | 66% (63/96) |
| **7a CNN** (seeds 72 / 72 / 56%; 91 / 47 / 59%) | **67%** (64/96) | 66% (63/96) |
| **7b CNN + look-ahead** (seeds 66 / 75 / 75%; 38 / 72 / 72%) | **72%** (69/96) | 60% (58/96) |

On all 48 spawns, both CNN arms average 66–67% on hard vs the baseline's 54%, and tip
over in 12–25% of hard episodes vs the baseline's 38%.

**Conclusions.**
1. On *hard*, the CNN beats both the flat network and the baseline; every CNN seed is at
   or above the baseline there. 7b's 72% vs 50% is unlikely to be luck (p ≈ 0.02).
   The first learned policy in this project to genuinely beat "turn to the goal and
   drive" among boulders, and more safely.
2. *Unseen* hasn't moved: level with the baseline, with one weak seed per arm.
   Generalising to new terrain is now the gap.
3. In grid form, look-ahead is no longer harmful (better on hard, worse on unseen,
   within noise), but not clearly worth 40 inputs and a depth camera.

**Decision.** Adopt **7a (terrain CNN)** as the best configuration: simplest, no extra
hardware, as good overall. 7b recorded as a close alternative.

> **Explainer: architecture is part of the observation.** D29 concluded that look-ahead's
> information was fine but its form wasn't usable. This experiment tested that directly:
> with convolutions, which build in "neighbouring cells relate and the same pattern means
> the same thing anywhere", the *existing* terrain patches became much more useful, and
> look-ahead stopped hurting. Matching the network to the structure of the input (grids →
> CNNs, sequences → recurrent or attention layers) is often worth more than adding new
> inputs.

---

## Lessons in one page

1. **Make the simulator fast and correct first** (D1–D5): throughput decides what's
   possible; clean resets and real terminations decide whether the signal means
   anything.
2. **Build a baseline before a policy** (D8, D11): it tells you whether the task even
   needs learning.
3. **Test on what you didn't train on** (D13, D22): held-out spawns separate learning
   from memorising.
4. **Measure the bottleneck before optimising** (D15).
5. **Watch episodes, not just averages** (D16).
6. **Use several seeds, always** (D19): single runs are anecdotes; most of a day's
   detective work was luck.
7. **One change at a time; "off" must reproduce the baseline exactly** (D20).
8. **Theory proposes, experiments decide** (D18): a textbook-correct fix made training
   worse.
9. **Know your test's blind spots** (D24): short screens punish bigger observations,
   and then test the excuse (D29): longer training didn't rescue look-ahead.
10. **Shaping can backfire** (D25); **"not worse" isn't "better"** (D21).
11. **Check defaults against standard practice** (D27): the biggest late gain came from
    a standard learning-rate schedule.
12. **Keep choosing and reporting separate** (D28): on fresh spawns, "beats the
    baseline" became "matches the baseline, more safely".
13. **Information needs a usable form** (D29, D30): the right input in the wrong shape
    can make a policy worse; the right architecture made existing inputs pay off.
