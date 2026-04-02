# Pod Racing — Development History

## Starting Point: Silver League Bot (single pod)

We had a working bot that reached Gold League using Silver League input format:
- Single pod, single opponent
- No checkpoint list from input — discovered at runtime
- Velocity/angle derived from position deltas
- 3-deep search + heuristic rollout, 7 frames total
- Shield and boost support

Tagged as `silver-v1` in git.

---

## Step 1: Adapt to Gold League Input Format

Gold League provides:
- **Initialization**: laps count + full checkpoint list
- **Each turn**: 4 pods (2 own, 2 opponent) with full state (x, y, vx, vy, angle, nextCheckPointId)
- **Output**: two lines (one per own pod)

**Removed**: all track detection (`trackCheckpoint`, `initStartPosition`, `gameCheckpoints`), velocity computation from deltas (`computeVelocity`, `computeOpponent`), facing angle derivation (`computeFacingAngle`), constants `TOTAL_LAPS`, `CLOSE_DIST_THRESHOLD`, `MAP_CENTER_X/Y`.

**Changed**: `main()` reads init data + 4 pods per turn. Checkpoints and laps are global. Output two lines.

**Added**: two own pods (`Pod pods[2]`), each with independent search. Collisions with 3 other pods (teammate + 2 opponents) instead of 1.

---

## Step 2: Performance — Reduce SIM_DEPTH

3 pods in collision checks = 3x more work per simulation step. Hit timeout.

- SIM_DEPTH 7 → 6 → 5 → **4** (current)
- Also reduced angle offsets from 14 to 12 (removed ±36)
- Each pod now evaluates ~41k–55k candidates per turn

---

## Step 3: All Angles in Radians

Eliminated degree/radian conversions in hot simulation loop:
- `MAX_TURN_DEG` → `MAX_TURN` (radians)
- `ANGLE_OFFSETS` arrays in radians
- `normalizeAngle` works with ±π
- `angleTo()` returns `atan2()` directly
- `simStep` uses `cos/sin(facingAngle)` without conversion
- Single conversion point: `setState()` converts input degrees → radians

---

## Step 4: Racer/Blocker Roles

**`assignRoles()`**: pod further ahead (by CPs passed, then distance to next CP) = racer. Other = blocker.

**Blocker behavior**:
- Search angles centered on **leading opponent** position (not own checkpoint)
- Higher shield collision bonus: 35000 vs 18000 for racer
- Boost collision bonus: 12500 (only for hitting lead enemy)

**`findLeadOpponent()`**: determines which opponent is ahead by CPs passed / distance.

---

## Step 5: Teammate Awareness

**Problem**: shield and collision bonuses triggered on teammate hits.

**Fix**:
- `closestEnemyDistSq()` / `closestEnemyRelSpeedSq()` skip `others[0]` (teammate)
- Shield eligibility checks only enemies
- Collision tracking separated: `shieldCollided` (enemy + shield), `boostCollided` (enemy + boost), `teammateCollided` (teammate)
- Teammate collision penalty: 25000 + 15000 * currentLap (scales with lap)

---

## Step 6: Sequential Pod Computation

Pod[0] decides first. Pod[1] gets pod[0]'s actual move via `receiveTeammateMove()`. In `precomputeOthers()`, teammate's step 1 uses `simStep()` with real move instead of linear extrapolation.

---

## Step 7: Early Checkpoint Switch

If current velocity will pass through current CP this frame AND enemies are far (> 2*COLLISION_RADIUS): center search angles on **next CP** instead of current. Starts turning toward next CP one frame earlier.

---

## Step 8: Uniform Scoring Across Laps

Removed special final-lap scoring (speed bonus, higher speedToward weight). Same evaluation on all laps — simpler and performed better.

---

## Step 9: Boost Restrictions

**Problem**: boost used immediately on lap 1, often in bad direction.

**Fix**: `BOOST_MIN_LAP = 2` (tried 1, reverted — boost on lap 1 went in wrong directions). Also `BOOST_MIN_DIST = 3500`.

---

## Step 10: Intercept Urgency System

**Problem**: blocker races normally even when losing.

**`calcInterceptUrgency()`**: counts how many enemy pods are ahead of our best pod.
- 0 enemies ahead → urgency = 0
- 1 enemy ahead → urgency = 0.51
- 2 enemies ahead → urgency = 1.02

When urgency > 0, blocker aims at enemy's **next checkpoint** (CP+1 from enemy's current target) to intercept, not chase.

---

## Step 11: Intercept Scoring — Approaches Tried

**Problem**: blocker doesn't effectively intercept on all map sizes.

### Attempt 1: Weighted blend
`(1 - urgency) * raceScore + urgency * interceptScore`

**Result**: blocker paralyzed. When race and intercept pull opposite directions, best compromise is thrust=0. Changing urgency coefficient (0.51 to 0.99) didn't help — any blend causes paralysis.

### Attempt 2: max(raceScore, interceptScore)
**Result**: works on small maps. On large maps, raceScore always wins because `-dist` to own CP is smaller than `-dist` to enemy CP. Blocker never commits to long intercept.

### Attempt 3: Remove -dist entirely (speedToward only)
**Result**: worse overall. Racer needs -dist to properly evaluate trajectories.

### Attempt 4: Different speed weight for intercept
`SCORE_SPEED_TOWARD_FINAL = 10.0` for interceptScore vs `3.0` for raceScore.

**Result**: helps but doesn't overcome -dist gap on large maps.

### Attempt 5 (current): Quadratic distance normalization
`-dist * (avgDist / mapSize)²` instead of `-dist`

Where `avgDist` = average distance between consecutive checkpoints, `mapSize` = 18000.

**Rationale**: on small tracks (avgDist=3000), coeff=0.028 — dist barely matters, speedToward decides. On large tracks (avgDist=9000), coeff=0.25 — dist still matters but doesn't dominate. Intercept wins on most track sizes.

---

## Step 12: Intercept Urgency — Distance Threshold

**Problem**: when cpsPassed are equal, enemy not considered "ahead" even if closer to next CP.

**Fix**: `isEnemyAhead()` — at equal cpsPassed, if enemy is closer to their CP by more than `INTERCEPT_DIST_THRESHOLD` (2000), they count as ahead. Triggers urgency.

---

## Step 13: Path-Crossing Intercept (game-changer)

**Problem**: blocker only intercepts when enemy is explicitly ahead. Misses natural intercept opportunities when blocker's own race path passes near enemy's checkpoint.

**Fix**: in `calcInterceptUrgency()`, check if the segment from blocker to its own CP passes within `INTERCEPT_PATH_RADIUS` (2000) of the enemy's CP using `segmentIntersectsCircle`. If yes, set minimum urgency = 1 enemy ahead (0.51).

**Result**: blocker intercepts "for free" when paths naturally cross. No deviation from own race path needed. Significant improvement — the blocker blocks effectively without sacrificing race performance.

---

## Step 14: Gold-2 Improvements

### Better enemy prediction
**Problem**: enemies predicted with linear extrapolation (pos += vel, vel *= friction). Real enemies steer toward their checkpoints.

**Fix**: `precomputeOthers()` uses `simStep()` with `angleDependentThrust` heuristic for all pods (enemies + teammate without real move). Also tracks checkpoint passing to update target CP. Required adding `facingAngle` and `nextCpId` to `OtherPodState`.

### Segment-based collision detection
**Problem**: `applyCollision` only checks endpoint positions. Fast pods pass through each other in one frame — collision missed. Shield not activated when enemy rams at high speed.

**Fix**: added `findCollisionTime()` — checks if movement segments intersect within `COLLISION_RADIUS`. Fallback in `simCandidate`: if endpoint check misses, segment check detects collision and sets flags (shieldCollided, boostCollided, teammateCollided).

### Blocker passive intercept weight
**Problem**: blocker without urgency had search centered on enemy position but scoring on own CP — inconsistent.

**Fix**: blocker without urgency now centers search on own CP (like racer) + 20% of `interceptScore` added to scoring (`BLOCKER_PASSIVE_INTERCEPT_WEIGHT = 0.2`). Blocker races but slightly steers toward enemy.

### Proximity-based intercept scoring
**Problem**: old `interceptScore` used `approachScore` toward enemy CP. When enemy CP is far, distance penalty dominates → `max(raceScore, interceptScore)` always picks race → blocker never blocks.

**Fix**: `interceptScore` now based on minimum distance to lead enemy's predicted position during 4-step simulation. `SimState` tracks `minEnemyDistSq`. Scoring: `INTERCEPT_PROXIMITY_BONUS (9000) - minEnemyDist * INTERCEPT_PROXIMITY_WEIGHT (1.0)`. Blocker races normally but chooses trajectories that pass close to the enemy.

### Low thrust override
**Problem**: racer stuck near CP with enemy blocker — outputs thrust 0 indefinitely.

**Fix**: if thrust < `LOW_THRUST_THRESHOLD` (20) for > `ZERO_THRUST_MAX_FRAMES` (5) frames → force `LOW_THRUST_OVERRIDE` (100) for 5 more frames. Breaks out of stuck situations.

### Boost restrictions
- `BOOST_MIN_TURN = 20` — no boost in first 20 frames (prevents misfires on early turns).

### findLeadOpponent bugfix
**Problem**: compared `pcpid` (nextCpId) instead of actual CPs passed. Incorrect on multi-lap races.

**Fix**: `findLeadOpponent` now takes `enemyCpsPassed[]` and compares by actual progress.

### Performance optimizations
- `closestEnemyDistSq()` called once and cached (was called twice)
- `distToCp` comparison uses squared distance (removed sqrt)
- `dirFromStep0` / `dirFromStep1` hoisted out of inner loops (saved ~14k `atan2` calls)
- `approachScore` computes dx/dy once (was computed twice via `distTo` + inline)
- `simStep` caches cos/sin in local variables

---

## Step 15: Collision System Rewrite

### Proper movement segments
**Problem**: collision detection used post-friction positions for segment endpoints. Actual movement is with post-thrust velocity (before friction).

**Fix**: split `simStep` into `applyRotationAndThrust` (rotation + thrust only) and movement + friction. `precomputeOthers` stores `moveVx/moveVy` (post-thrust velocity) in `PodPrediction`. Collision segments now use correct post-thrust endpoints.

### Post-collision movement continuation
**Problem**: after segment collision at time `t`, pod stopped at collision point. Remaining `(1-t)` movement lost.

**Fix**: after mutual impulse, both pods continue movement for `(1-t)` fraction. Matches real game engine behavior.

### Mutual collision impulse
**Problem**: `applyCollision` only modified our pod's velocity. Enemy velocity unchanged.

**Fix**: `applyMutualCollision` applies impulse to both pods. Formula: `impulse = max(MIN_IMPULSE, 2*m1*m2*|dvn|/(m1+m2))`. Our velocity change: `impulse/m1`. Enemy: `impulse/m2`. Also fixes shield mass formula — shield pod now correctly receives 1/10th the velocity change.

### Dynamic enemy re-prediction after collision
**Problem**: enemy predictions precomputed once, unchanged after collision. If simulation detects collision with enemy at step N, enemy's predicted positions for steps N+1+ are wrong (don't account for collision impulse).

**Fix**: `EnemyStepState` tracks modified enemy state through search. When collision detected, enemy velocity updated, subsequent steps re-predicted from modified state. Passed through search loop: `findBestMove` → `searchFromStep1` → `simCandidate` → `rollout`.

### Fast path optimization
**Problem**: `simCandidate` did 6 loops per call even without collisions. ~95% of calls have no collision.

**Fix**: when no enemy modified and no collision detected → early return. Reads positions from precomputed predictions. Skips `updateEnemyStates`, `computeEnemyMoves`, endpoint collision checks. Copies of `EnemyStepState[]` between search levels skipped when no enemy modified.

### Shield mass for full cooldown
**Problem**: `getMass` returned SHIELD_MASS only on activation step. Steps 1-2 of cooldown had NORMAL_MASS despite game rules saying mass=10 for all 3 turns.

**Fix**: `getMass` checks `state.shieldTurnsLeft > 0`. `resolveThrust` called before `getMass` so shieldTurnsLeft is current.

---

## Step 16: Removed M_PI from Search Angles

**Problem**: `M_PI` in ANGLE_OFFSETS was nearly always redundant — produced same clamped rotation as ±60°/±45° (due to MAX_TURN=18° clamp). Wasted 1 search slot per level.

**Fix**: removed M_PI from both ANGLE_OFFSETS_1 and ANGLE_OFFSETS_23. Search: 9×5 → 5×4 → 5×4 = 18000 candidates (was 28800 with M_PI). 37% speedup.

---

## Step 17: Next-CP Speed Scoring (reverted)

**Problem**: pod enters checkpoints too fast, overshoots on sharp turns.

**Attempted fix**: added `speedTowardPoint(nextCp) * SCORE_SPEED_TOWARD_NORMAL` to scoring when CP passed. Rewards velocity toward next CP.

**Result**: reverted — needed more tuning, interacted poorly with existing scoring weights.

---

## Step 18: Shield Activation Attempts (reverted)

Multiple approaches tried to make shield activate at the right time:

### Attempt 1: SHIELD_CHECK_DIST gate
Original approach — `canShield` requires `dist < 1200`. Too tight, misses high-speed approaches.

### Attempt 2: Time-to-collision estimate
`estimateFramesToEnemyCollision()` computes closing speed per enemy, estimates frames to contact. Shield considered when `frames <= threshold`.

**Problem**: estimate inaccurate (ignores acceleration), threshold tuning impossible — too high = shield wasted early, too low = shield missed.

### Attempt 3: Linear prediction for close enemies
When enemy within threshold distance, `precomputeOthers` uses linear extrapolation (current velocity) instead of heuristic (steer toward CP) for step 0. More accurate near-collision prediction.

**Problem**: distance threshold still wrong for high-speed approaches.

### Attempt 4: Always-on shield (no gate)
Shield always in search (`canShield = shieldCooldown == 0`). Let simulation decide.

**Problem**: simulation's enemy prediction moves enemy away → collision not detected → shield never scores well. +20% search cost for no benefit. Dropped from 1st to 228th.

**All reverted**. Shield activation remains an open problem. Core issue: simulation predicts enemies steering toward their CP, but enemies near collision don't move as predicted. Shield candidates that "avoid" collision score better than shield candidates that collide, but in reality avoidance isn't possible.

---

## Step 19: Goalie Detection Improvements

### Larger detection radius
**Problem**: `detectGoalie` checked `dist < CP_RADIUS` (595). Enemy standing at 600-1000 from CP blocking the approach wasn't detected.

**Fix**: new constant `GOALIE_DETECT_DIST = 2 * CP_RADIUS` (1190). Enemy within ~1190 of racer's CP counts.

### Speed check
**Problem**: enemy flying through CP zone at full speed triggered goalie counter.

**Fix**: new constant `GOALIE_MAX_SPEED_SQ = 150²`. Enemy must also have low speed (`vx²+vy² < 22500`) to count as goalie.

### Goalie only when leading (commented out)
**Problem**: goalie override changed blocker's target even when losing. Should only prioritize goalie defense when we're ahead.

**Fix**: `applyGoalieOverride` — only applied when `ourBestCps >= enemyBestCps`. Currently **commented out** in `setupTurn`, not active.

### Refactored into setupTurn
`detectGoalie` returns goalie index instead of writing Pod fields directly. `setupTurn` calls `setupLeadEnemy` (default), `setupOtherPods`. Main calls one line.

### Lead enemy overwrite bug (found, not yet fixed in active code)
**Problem**: old `detectGoalie` set blocker's `leadEnemyNextCpId` to racer's CP, but the for loop in main immediately overwrote it with `pcpid[leadOp]`. Goalie targeting was silently broken. Fix exists in `applyGoalieOverride` but is commented out.

---

## Step 20: Gold-1 Tuning & Cleanup

### normalizeAngle → O(1)
While-loops replaced with `fmod` + two `if` statements. Guaranteed O(1) instead of O(n) for large angles.

### CP radius tuning
`CP_RADIUS = 595` (was 590), `CP_RADIUS_REDUCTION = 5` (was 10). Better matches actual game engine behavior.

### Search angle/thrust rebalance
- Step 0: 12×6 → 10×5. Angles: ±60°/±27°/±15°/±7°/0/π (were ±50°/±24°/±18°/±12°/±6°/0/π). Thrusts: 10/42/75/100/0 (were 10/35/55/75/100/0).
- Steps 1-2: angles ±45°/±15°/0/π (were ±24°/±12°/0/π). Thrusts: 10/42/75/100 (were 0/10/45/75).

### Scoring tuning
- `SCORE_DIST_TO_NEXT_WEIGHT = 5.0` (was 2.0) — stronger penalty for distance to next CP after passing current one.
- `INTERCEPT_PROXIMITY_BONUS = 15000` (was 9000) — blocker pursues enemy more aggressively.

### Low thrust override rebalance
- `LOW_THRUST_THRESHOLD = 20` (was 0 — only zero thrust triggered). Now catches low thrust too.
- `ZERO_THRUST_MAX_FRAMES = 4` (was 5/7).
- `LOW_THRUST_OVERRIDE = 100` (was 20). Breaks out of stuck situations at full power.
- Added `overrideFramesLeft` — override lasts `ZERO_THRUST_MAX_FRAMES` turns.

### Boost restrictions
- `BOOST_MIN_TURN = 10` — no boost in first 10 turns (was 20 or absent).

### setupTurn refactoring
Main no longer contains turn setup logic. Extracted into static Pod methods: `setupTurn`, `setupLeadEnemy`, `setupOtherPods`, `detectGoalie`, `applyGoalieOverride`. Main calls one line: `Pod::setupTurn(...)`.

### currentTurn
Added static field `Pod::currentTurn` — used for `BOOST_MIN_TURN` check.

---

## Current Architecture Summary (Gold-1)

- **Two pods**: racer + blocker, roles assigned each frame
- **Blocker with urgency**: intercept scored by proximity to lead enemy (minEnemyDist over simulation)
- **Blocker without urgency**: races own CPs + 20% passive intercept weight
- **normalizeAngle**: O(1) via `fmod`, not while-loops
- **Search**: 3-deep + 1 rollout, 10×5 → 6×4 → 6×4 branching (with M_PI)
- **Scoring**: `max(raceScore, interceptScore)` with quadratic dist normalization
- **Collisions**: mutual impulse, segment-based detection, post-collision movement continuation, dynamic enemy re-prediction
- **Enemy prediction**: simStep with angleDependentThrust heuristic
- **Goalie detection**: enemy near racer's CP + low speed + 4 turns → blocker retargets (only when leading)
- **Shield**: gate by dist + relSpeed, simulation evaluates. Open problem — simulation doesn't reliably detect imminent collisions.
- **All radians**, single deg→rad conversion at input

---

## TODO

- **Blocker friendly-fire penalty**: penalize blocker hitting an enemy when teammate (racer) is behind the enemy along the impact direction. The hit sends the enemy into our racer.
- **Shield activation**: simulation can't detect unavoidable collisions because enemy prediction moves enemy away. Need better near-collision enemy prediction or alternative shield decision mechanism.
- **Next-CP speed scoring**: reward velocity toward next CP when passing current CP, to reduce overshooting on sharp turns.
- **M_PI removal**: tested, caused regression. M_PI critical for post-collision recovery. Do not remove.
