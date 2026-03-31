# Pod Racing Bot

Bot for CodinGame Mad Pod Racing (Gold League input format). Controls two pods racing across an arena (16000x9000) from checkpoint to checkpoint, competing against two opponent pods. All checkpoints, velocities, and angles are provided by the game at init and each turn.

## Input Format (Gold League, Two Pods)

**Initialization (once):**
- Line 1: `laps` — number of laps to complete
- Line 2: `checkpointCount` — number of checkpoints
- Next `checkpointCount` lines: `checkpointX checkpointY` — coordinates of each checkpoint

**Each turn reads 4 lines** (2 own pods, then 2 opponent pods):
- Each line: `x y vx vy angle nextCheckPointId` — full pod state

**Output per turn:** two lines (one per own pod): `target_x target_y thrust|BOOST|SHIELD`

---

## Architecture

### Constants (lines 7–65)

All tunable parameters are `constexpr` at the top of the file. All angles are stored and computed in **radians** — the only degree-to-radian conversion happens once at input (`angleDeg * DEG_TO_RAD`).

| Group | Constants | Purpose |
|-------|-----------|---------|
| Physics | `FRICTION=0.85`, `MAX_TURN=18*pi/180`, `CP_RADIUS=590`, `CP_RADIUS_REDUCTION=10`, `COLLISION_RADIUS=800`, `MIN_COLLISION_IMPULSE=120`, `DEG_TO_RAD` | Game engine physics |
| Simulation | `SIM_DEPTH=4`, `ZERO_THRUST_MAX_FRAMES=7`, `ZERO_THRUST_OVERRIDE=20`, `TARGET_POINT_DIST=10000` | Search depth, failsafe |
| Scoring | `SCORE_CP_PASSED=50000`, `SCORE_EARLY_ARRIVAL=5000`, `SCORE_SPEED_TOWARD_NORMAL=3.0`, `SCORE_DIST_TO_NEXT_WEIGHT=2.0` | Evaluation weights. Uniform across all laps. |
| Boost | `BOOST_MIN_DIST=5500`, `BOOST_MIN_LAP=2`, `BOOST_SCORE_THRESHOLD=10000` | Boost eligibility: not on lap 1, distance > 5500 |
| Shield | `SHIELD_THRUST_VALUE=-1`, `SHIELD_COOLDOWN=3`, `SHIELD_MASS=10`, `NORMAL_MASS=1`, `SHIELD_CHECK_DIST=1200`, `SHIELD_SCORE_THRESHOLD=0`, `SHIELD_COLLISION_BONUS=18000` (racer) / `SHIELD_COLLISION_BONUS_BLOCKER=35000` (blocker), `SHIELD_MIN_REL_SPEED_SQ=125000` | Shield activation and scoring |
| Blocker | `BOOST_COLLISION_BONUS_BLOCKER=12500`, `TEAMMATE_COLLISION_PENALTY=20000` | Blocker gets bonus for boost-ramming enemy, penalty for hitting teammate |
| Search step 0 | `ANGLE_OFFSETS_1[]` (12 values in radians: -50 to +50 deg, plus pi), `THRUST_OPTIONS_1[]` (6 values) | Branching for first search step |
| Search steps 1-2 | `ANGLE_OFFSETS_23[]` (6 values: -24 to +24 deg, plus pi), `THRUST_OPTIONS_23[]` (4 values) | Reduced branching for deeper steps |

### Global State (lines 67–88)

- `totalLaps` — number of laps, read from initialization input
- `checkpointCount` — number of checkpoints, read from initialization input
- `checkpoints[20]` — fixed array of checkpoint coordinates, populated at startup

### Point (lines 72–86)

Simple `{x, y}` struct with `==`, `!=`, `+`, `<` operators and `operator<<` for debug output `{x;y}`.

### Helper Functions (lines 99–119)

**`normalizeAngle(angle)`** — wraps angle to [-pi, pi] range. Works in radians.

**`segmentIntersectsCircle(x1,y1, x2,y2, px,py, radius)`** — checks if the line segment passes within `radius` of a point. Solves quadratic for parameter t. Needed because at high speed a pod can fly over a checkpoint in one frame.

### SimState (lines 121–137)

Simulation-time pod state (all angles in radians):

| Field | Type | Meaning |
|-------|------|---------|
| `x, y` | double | Position |
| `vx, vy` | double | Velocity |
| `facingAngle` | double | Facing direction (radians) |
| `cpsPassed` | int | Checkpoints passed during simulation |
| `shieldTurnsLeft` | int | Shield cooldown remaining |
| `shieldCollided` | bool | Collision with shield active (enemy only) |
| `boostCollided` | bool | Collision while boosting (enemy only) |
| `teammateCollided` | bool | Collision with teammate |

---

## Pod — Main Structure (lines 139–610)

### Roles: Racer vs Blocker

Each frame, `assignRoles()` designates one pod as **racer** (further ahead by CPs passed, or closer to next CP if tied) and one as **blocker**.

**Racer** behavior:
- Aims at checkpoints (with early next-CP switch when about to pass current)
- Standard collision bonuses

**Blocker** behavior:
- Aims search angles at the **leading opponent** instead of own checkpoint
- Higher shield collision bonus (35000 vs 18000)
- Bonus for boost-ramming enemy (12500)
- Shield/boost eligibility checks only consider enemies, not teammate

Both pods are penalized (-20000) for colliding with teammate.

### State Fields

**Current state (from input each frame via `setState`):**
- `x, y, vx, vy` — position and velocity
- `facingAngle` — facing direction in radians (converted once from input degrees)
- `nextCpId` — index of next checkpoint

**Persistent:**
- `shieldCooldown`, `zeroThrustFrames`, `boostAvailable`
- `currentLap`, `totalCpsPassed`, `prevNextCpId` — lap tracking from checkpoint ID transitions

**Role:**
- `isBlocker` — set each frame by `assignRoles()`
- `leadEnemyX, leadEnemyY` — position of leading opponent

**Other pods:**
- `others[3]` — teammate (index 0) + 2 opponents (indices 1-2), each `{x, y, vx, vy}`
- `otherPredictions[3][SIM_DEPTH+1]` — precomputed predicted positions

**Teammate move (for pod[1] only):**
- `hasTeammateMove`, `teammateAngle`, `teammateThrust`, `teammateFacing` — pod[0]'s chosen move, used for accurate step-1 prediction

### Other Pod Prediction

**`precomputeOthers()`** — for each other pod, fills prediction array:
- All pods: linear extrapolation (pos += vel, vel *= friction, truncate to int)
- Teammate (p==0) with `hasTeammateMove`: step 1 uses `simStep()` with the teammate's actual chosen move (angle + thrust + facing), then linear from step 2 onward

### Collision Physics

**`applyCollision(s, ox, oy, ovx, ovy, myMass)`** — elastic collision:
1. Distance check (skip if >= 800 or < epsilon)
2. Normal vector, relative velocity projection
3. Impulse: `max(120, -2 * otherMass * dvn / totalMass)`
4. Apply velocity change + push-apart

**Collision tracking in `simCandidate`:**
- Hit teammate (p==0): sets `teammateCollided`
- Hit enemy (p>=1) with shield: sets `shieldCollided`
- Hit enemy (p>=1) with boost (thrust==650): sets `boostCollided`

**`closestEnemyDistSq()`** / **`closestEnemyRelSpeedSq()`** — check only opponents (others[1] and others[2]), skip teammate. Used for shield eligibility.

### Simulation Engine

**`simStep(s, targetAngle, simThrust)`** — one frame of physics:
1. Rotate toward target (max +-18 deg/frame, via `normalizeAngle` in radians)
2. Thrust in facing direction: `vx += thrust * cos(facing)`, `vy += thrust * sin(facing)` — no conversion needed, all in radians
3. Move, friction (truncate to int), position truncate

**`simCandidate(...)`** — full step: resolve thrust, simStep, collisions with all 3 others, dynamic CP radius, segment-circle checkpoint detection, advance simNextCpId on pass.

**`angleDependentThrust(aimAngle, facingAngle)`** — rollout heuristic: `max(20, 100 - |angleDiff in degrees|)`. Converts back to degrees only for this threshold calculation.

**`rollout(s, fromStep, ...)`** — heuristic continuation for remaining steps after the 3 searched steps.

### Scoring

**`evaluateScore(simPod, arrivalFrame, currentCp, nextCp)`**:

**Bonuses:**
- Shield collision: +18000 (racer) or +35000 (blocker)
- Boost collision with enemy (blocker only): +12500
- Teammate collision: -20000

**Uniform across all laps** (no special final-lap logic):

If checkpoint reached (`cpsPassed > 0`):
```
score = cpsPassed * 50000 + (SIM_DEPTH - arrivalFrame) * 5000 - distToNextCP * 2.0 + bonus
```

If not reached:
```
score = -distToCP + speedToward * 3.0 + bonus
```

### Search Algorithm — findBestMove()

**Setup:**
1. `precomputeOthers()` with teammate's real move if available
2. Early CP switch: if current velocity will pass through current CP this frame and enemies are far (> 2*COLLISION_RADIUS), center search on next CP
3. Blocker centers search on leading opponent position instead of CP
4. Boost eligibility: `boostAvailable && currentLap >= 2 && distToCp > 5500`
5. Shield eligibility: only checks enemy distances/speeds (not teammate)

**Three-level search:**
- Step 0: 12 angles x 6-8 thrusts = 72-96 branches
- Step 1: 6 x 4 = 24 branches
- Step 2: 6 x 4 = 24 branches
- Step 3: 1 rollout step (SIM_DEPTH=4)

**Total:** 72 x 24 x 24 = **41,472** up to 96 x 24 x 24 = **55,296** candidates per pod.

Three separate best scores (normal, boost, shield). Boost chosen if +10000 over normal. Shield chosen if +0 over normal.

**Output:** best angle converted to target point 10000 units away.

### Output — navigate()

1. Decrement shield cooldown
2. Stuck detection: 0-thrust for >7 frames overrides to 20
3. Output: `targetX targetY BOOST|SHIELD|thrust`

---

## Main Loop (lines 612–658)

**Initialization:** read laps, checkpoints, create `pods[2]`.

**Per frame:**
1. Read 4 pod states
2. `setState()` for own pods (converts angle deg->rad, tracks laps)
3. `assignRoles()` — racer/blocker by CP progress
4. `findLeadOpponent()` — leading enemy by CP progress
5. Set others: teammate (index 0) + 2 opponents (indices 1-2)
6. pod[0]: `findBestMove()` (no teammate info)
7. pod[1]: `receiveTeammateMove(pod[0])` then `findBestMove()` (uses pod[0]'s real move for prediction)
8. Both pods: `navigate()`

---

## Key Design Decisions

- **All angles in radians**: eliminates deg/rad conversions in the hot simulation loop. Only conversion: input degrees -> radians once in `setState()`.
- **Racer/blocker roles**: blocker aims at leading enemy, gets higher collision bonuses, actively seeks ramming opportunities. Racer focuses on checkpoint progress.
- **Teammate-aware**: shield and boost collision checks skip teammate. Teammate collisions are penalized (-20000). Pod[1] uses pod[0]'s actual move for step-1 prediction.
- **Enemies-only shield activation**: `closestEnemyDistSq`/`closestEnemyRelSpeedSq` skip others[0] (teammate), preventing shield activation against own pod.
- **No boost on lap 1**: `BOOST_MIN_LAP=2` — saves boost for later laps when track is known and positions more strategic.
- **Uniform scoring across laps**: no special final-lap logic — same evaluation on all laps.
- **Integer truncation in simulation**: velocity and position truncated to int each step, matching game engine.
- **Segment-circle CP detection**: catches high-speed fly-through cases.
- **Dynamic CP radius**: shrinks by 10 when another pod is within collision range.
