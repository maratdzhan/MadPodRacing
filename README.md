# Pod Racing Bot

Bot for CodinGame Mad Pod Racing (Gold League input format). Controls two pods racing across an arena (16000x9000) from checkpoint to checkpoint, competing against two opponent pods. All checkpoints, velocities, and angles are provided by the game at init and each turn.

## Input Format (Gold League, Two Pods)

**Initialization (once):**
- Line 1: `laps` — number of laps to complete
- Line 2: `checkpointCount` — number of checkpoints
- Next `checkpointCount` lines: `checkpointX checkpointY` — coordinates of each checkpoint

**Output per turn:** two lines (one per own pod): `target_x target_y thrust|BOOST|SHIELD message`

---

## Architecture

### Constants

All tunable parameters are `constexpr` at the top of the file. All angles are stored and computed in **radians** — the only degree-to-radian conversion happens once at input (`angleDeg * DEG_TO_RAD`).

| Group | Constants | Purpose |
|-------|-----------|---------|
| Physics | `FRICTION=0.85`, `MAX_TURN=18*pi/180`, `CP_RADIUS=590`, `CP_RADIUS_REDUCTION=10`, `COLLISION_RADIUS=800`, `MIN_COLLISION_IMPULSE=120`, `DEG_TO_RAD` | Game engine physics |
| Simulation | `SIM_DEPTH=4`, `ZERO_THRUST_MAX_FRAMES=7`, `ZERO_THRUST_OVERRIDE=20`, `TARGET_POINT_DIST=10000` | Search depth, failsafe |
| Scoring | `SCORE_CP_PASSED=50000`, `SCORE_EARLY_ARRIVAL=5000`, `SCORE_SPEED_TOWARD_NORMAL=3.0`, `SCORE_SPEED_TOWARD_FINAL=10.0` (intercept), `SCORE_DIST_TO_NEXT_WEIGHT=2.0` | Evaluation weights. Uniform across all laps. |
| Boost | `BOOST_MIN_DIST`, `BOOST_MIN_LAP=2`, `BOOST_SCORE_THRESHOLD=10000` | Boost eligibility: not on lap 1 |
| Shield | `SHIELD_THRUST_VALUE=-1`, `SHIELD_COOLDOWN=3`, `SHIELD_MASS=10`, `NORMAL_MASS=1`, `SHIELD_CHECK_DIST=1200`, `SHIELD_SCORE_THRESHOLD=0`, `SHIELD_COLLISION_BONUS=18000` (racer) / `SHIELD_COLLISION_BONUS_BLOCKER=35000` (blocker), `SHIELD_MIN_REL_SPEED_SQ=125000` | Shield activation and scoring |
| Blocker | `BOOST_COLLISION_BONUS_BLOCKER=12500`, `TEAMMATE_COLLISION_PENALTY=25000`, `TEAMMATE_COLLISION_PENALTY_PER_LAP=15000` | Collision bonuses/penalties |
| Intercept | `INTERCEPT_URGENCY_PER_ENEMY=0.51`, `INTERCEPT_DIST_THRESHOLD=2000`, `INTERCEPT_PATH_RADIUS=2000` | Intercept urgency triggers |
| Search step 0 | `ANGLE_OFFSETS_1[]` (12 values in radians: -50 to +50 deg, plus pi), `THRUST_OPTIONS_1[]` (6 values) | Branching for first search step |
| Search steps 1-2 | `ANGLE_OFFSETS_23[]` (6 values: -24 to +24 deg, plus pi), `THRUST_OPTIONS_23[]` (4 values) | Reduced branching for deeper steps |

### Global State

- `totalLaps`, `checkpointCount`, `checkpoints[20]` — read at initialization
- `distNormCoeff` — quadratic distance normalization coefficient, computed once: `(avgDist / 18000)²` where `avgDist` = average distance between consecutive checkpoints

### Point

Simple `{x, y}` struct with `==`, `!=`, `+`, `<` operators and `operator<<` for debug output.

### Helper Functions

**`normalizeAngle(angle)`** — wraps angle to [-pi, pi] range. Works in radians.

**`segmentIntersectsCircle(x1,y1, x2,y2, px,py, radius)`** — checks if the line segment passes within `radius` of a point. Solves quadratic for parameter t. Used for: checkpoint detection in simulation, and path-crossing intercept detection.

**`computeDistNormCoeff()`** — computes `distNormCoeff = (avgDist / 18000)²`. Normalizes `-dist` in scoring so it doesn't dominate on large tracks.

### SimState

Simulation-time pod state (all angles in radians):

| Field | Type | Meaning |
|-------|------|---------|
| `x, y` | double | Position |
| `vx, vy` | double | Velocity |
| `facingAngle` | double | Facing direction (radians) |
| `cpsPassed` | int | Checkpoints passed during simulation |
| `shieldTurnsLeft` | int | Shield cooldown remaining |
| `shieldCollided` | bool | Collision with shield active (enemy only) |
| `boostCollided` | bool | Collision while boosting (enemy only, lead enemy only) |
| `teammateCollided` | bool | Collision with teammate |

---

## Pod — Main Structure

### Roles: Racer vs Blocker

Each frame, `assignRoles()` designates one pod as **racer** (further ahead by CPs passed, or closer to next CP if tied) and one as **blocker**.

**Racer** behavior:
- Aims at checkpoints (with early next-CP switch when about to pass current)
- Standard collision bonuses

**Blocker** behavior:
- When `interceptUrgency > 0`: search angles centered on **enemy's next checkpoint**
- When `interceptUrgency == 0`: search angles centered on **leading opponent position** (for close-range encounters)
- Higher shield collision bonus (35000 vs 18000)
- Bonus for boost-ramming lead enemy (12500)
- Shield/boost eligibility checks only consider enemies, not teammate

Both pods penalized for teammate collision: `-(25000 + 15000 * currentLap)`.

### Intercept Urgency System

**`calcInterceptUrgency()`** determines how aggressively the blocker should intercept:

Three triggers (any sets urgency > 0):
1. **Enemy ahead by CPs**: `enemyCpsPassed > ourBestCpsPassed`
2. **Enemy ahead by distance**: at equal cpsPassed, if enemy is closer to next CP by more than `INTERCEPT_DIST_THRESHOLD` (2000)
3. **Path crossing**: blocker's path to own CP passes within `INTERCEPT_PATH_RADIUS` (2000) of enemy's CP — detected via `segmentIntersectsCircle`. This is the most impactful trigger: the blocker intercepts "for free" without deviating from its own race line.

Urgency value: `enemiesAhead * INTERCEPT_URGENCY_PER_ENEMY` (0.51 per enemy).

### Scoring

**`collisionBonus(simPod)`** — calculates all collision bonuses/penalties:
- Shield collision: +18000 (racer) or +35000 (blocker)
- Boost collision with lead enemy (blocker only): +12500
- Teammate collision: -(25000 + 15000 * currentLap)

**`approachScore(simPod, tx, ty, speedWeight)`** — evaluates approach to a target point:
```
-dist * distNormCoeff + speedToward * speedWeight
```
Where `distNormCoeff = (avgDist / 18000)²` normalizes distance penalty across different track sizes.

**`evaluateScore(simPod, arrivalFrame, currentCp, nextCp)`**:

If checkpoint reached (`cpsPassed > 0`):
```
score = cpsPassed * 50000 + (SIM_DEPTH - arrivalFrame) * 5000 - distToNextCP * 2.0 + bonus
```

If not reached:
- Racer: `approachScore(pod, currentCp, 3.0) + bonus`
- Blocker with urgency > 0: `max(raceScore, interceptScore) + bonus`
  - `raceScore` = approachScore toward own CP (weight 3.0)
  - `interceptScore` = approachScore toward enemy's CP (weight 10.0)
  - `max()` ensures blocker always commits to whichever direction is better, no paralysis from blending

### Search Algorithm — findBestMove()

**Setup:**
1. `precomputeOthers()` with teammate's real move if available
2. Early CP switch: if current velocity will pass through current CP this frame and enemies are far (> 2*COLLISION_RADIUS), center search on next CP
3. Blocker with urgency > 0: centers search on enemy's next checkpoint. Without urgency: centers on enemy position.
4. Boost eligibility: `boostAvailable && currentLap >= BOOST_MIN_LAP && distToCp > BOOST_MIN_DIST`
5. Shield eligibility: only checks enemy distances/speeds (not teammate)

**Three-level search:**
- Step 0: 12 angles x 6-8 thrusts = 72-96 branches
- Step 1: 6 x 4 = 24 branches
- Step 2: 6 x 4 = 24 branches
- Step 3: 1 rollout step (SIM_DEPTH=4)

**Total:** ~41k–55k candidates per pod.

Three separate best scores (normal, boost, shield). Boost chosen if +10000 over normal. Shield chosen if +0 over normal.

### Other Pod Prediction

**`precomputeOthers()`** — for each other pod:
- All pods: linear extrapolation (pos += vel, vel *= friction, truncate to int)
- Teammate (p==0) with `hasTeammateMove`: step 1 uses `simStep()` with actual chosen move, then linear

### Collision Physics

**`applyCollision(s, ox, oy, ovx, ovy, myMass)`** — elastic collision with impulse `max(120, -2 * otherMass * dvn / totalMass)`.

Collision tracking in `simCandidate`:
- Hit teammate (p==0): `teammateCollided`
- Hit enemy with shield (mass > normal): `shieldCollided`
- Hit lead enemy with boost (thrust==650, p==leadEnemyIdx): `boostCollided`

**`closestEnemyDistSq()`** / **`closestEnemyRelSpeedSq()`** — skip teammate (others[0]).

### Simulation Engine

**`simStep(s, targetAngle, simThrust)`** — one frame: rotate (max +-18 deg), thrust, move, friction (truncate to int), position truncate.

**`simCandidate(...)`** — full step: resolve thrust, simStep, collisions with all 3 others, dynamic CP radius, segment-circle checkpoint detection.

**`rollout(s, fromStep, ...)`** — heuristic continuation using `angleDependentThrust`: `max(20, 100 - |angleDiff in degrees|)`.

### Output — navigate()

1. Decrement shield cooldown
2. Blocker debug: log urgency, enemy CP, thrust to stderr
3. Stuck detection: 0-thrust for >7 frames overrides to 20
4. Output: `targetX targetY BOOST|SHIELD|thrust podName`

---

## Main Loop

**Initialization:** read laps, checkpoints, `computeDistNormCoeff()`, create `pods[2]`.

**Per frame:**
1. Read 4 pod states, track enemy cpsPassed
2. `setState()` for own pods (converts angle deg->rad, tracks laps)
3. `assignRoles()` — racer/blocker by CP progress
4. `findLeadOpponent()` — leading enemy by CP progress
5. `calcInterceptUrgency()` — urgency from enemy lead + path crossing
6. Set others: teammate (index 0) + 2 opponents (indices 1-2)
7. pod[0]: `findBestMove()` (no teammate info)
8. pod[1]: `receiveTeammateMove(pod[0])` then `findBestMove()`
9. Track scores in `scoreHistory`
10. Both pods: `navigate()`
11. Near finish: output total scores to stderr

---

## Key Design Decisions

- **All angles in radians**: eliminates deg/rad conversions in the hot simulation loop.
- **Quadratic distance normalization**: `distNormCoeff = (avgDist/18000)²` prevents `-dist` from dominating scoring on large tracks. On small tracks coeff is tiny (~0.03), on large tracks moderate (~0.25).
- **max(raceScore, interceptScore)**: blocker commits fully to whichever direction is better. No weighted blending — that caused paralysis (thrust=0 as "compromise").
- **Path-crossing intercept**: `segmentIntersectsCircle` detects when blocker's race path passes near enemy's CP. Enables "free" interceptions without sacrificing race performance. Game-changer improvement.
- **Intercept speed weight 10.0 vs race 3.0**: blocker values speed toward enemy CP more than racer values speed toward own CP, making intercept trajectories competitive in `max()`.
- **Teammate-aware**: shield checks skip teammate. Teammate collision penalty scales with lap (25k + 15k * lap). Boost collision bonus only for hitting lead enemy.
- **Sequential pod computation**: pod[0] decides first, pod[1] gets pod[0]'s actual move for better prediction.
- **No boost on lap 1**: prevents early misfires when positions are suboptimal.
