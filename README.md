# Pod Racing Bot

Bot for CodinGame Mad Pod Racing (Silver League input format). Controls a single pod racing across an arena (16000x9000) from checkpoint to checkpoint, competing against one opponent pod. 3 laps. This version reached Gold League promotion. Uses the Silver League input format: no initialization phase, single opponent as position only (no velocity/angle), no explicit checkpoint list — all derived at runtime.

## Input Format (Silver League, Single Pod)

**Each turn reads:**
- Line 1: `x y next_checkpoint_x next_checkpoint_y next_checkpoint_dist next_checkpoint_angle` — pod state + next CP info
- Line 2: `opponent_x opponent_y` — opponent position only

**Output per turn:** one line: `target_x target_y thrust|BOOST|SHIELD`

Key limitation: no explicit velocity, facing angle, or checkpoint list from input — all must be derived.

---

## Architecture

### Constants (lines 8–63)

All tunable parameters are `constexpr` at the top of the file, grouped by purpose:

| Group | Constants | Purpose |
|-------|-----------|---------|
| Physics | `FRICTION=0.85`, `MAX_TURN_DEG=18.0`, `CP_RADIUS=590.0`, `CP_RADIUS_REDUCTION=10.0`, `COLLISION_RADIUS=800.0`, `MIN_COLLISION_IMPULSE=120.0` | Game engine physics parameters |
| Simulation | `SIM_DEPTH=7`, `TOTAL_LAPS=3`, `ZERO_THRUST_MAX_FRAMES=7`, `ZERO_THRUST_OVERRIDE=20`, `TARGET_POINT_DIST=10000.0` | Search depth, failsafe thresholds |
| Search step 0 | `ANGLE_OFFSETS_1[]` (14 values: -50 to +50, plus 180), `THRUST_OPTIONS_1[]` (6 values: 10,35,55,75,100,0), `THRUST_OPTIONS_1_BOOST[]` (adds 650) | Branching factor for first search step |
| Search steps 1-2 | `ANGLE_OFFSETS_23[]` (6 values: -24 to +24, plus 180), `THRUST_OPTIONS_23[]` (4 values: 0,10,45,75) | Reduced branching for deeper steps |
| Scoring | `SCORE_CP_PASSED=50000`, `SCORE_EARLY_ARRIVAL=5000`, `SCORE_SPEED_WEIGHT_FINAL=0.025`, `SCORE_SPEED_TOWARD_NORMAL=3.0`, `SCORE_SPEED_TOWARD_FINAL=10.0`, `SCORE_DIST_TO_NEXT_WEIGHT=1.0` | Evaluation weights |
| Boost | `BOOST_MIN_DIST=4000`, `BOOST_SCORE_THRESHOLD=10000.0` | Boost eligibility and advantage threshold |
| Shield | `SHIELD_THRUST_VALUE=-1`, `SHIELD_COOLDOWN=3`, `SHIELD_MASS=10.0`, `NORMAL_MASS=1.0`, `SHIELD_CHECK_DIST=1200.0`, `SHIELD_SCORE_THRESHOLD=0.0`, `SHIELD_COLLISION_BONUS=18000.0`, `SHIELD_MIN_REL_SPEED_SQ=125000.0` | Shield activation criteria and scoring |
| Track detection | `CLOSE_DIST_THRESHOLD=750.0`, `MAP_CENTER_X=8000.0`, `MAP_CENTER_Y=4500.0` | Checkpoint loop detection |

### Point (lines 74–90)

Simple `{x, y}` struct with:
- `operator==`, `operator!=`, `operator+`, `operator<` for comparisons and arithmetic
- `operator<<` for debug output: `{x;y}`

A `vector<K>` stream operator (lines 65–72) prints elements separated by `;`.

### Helper Functions (lines 92–113)

**`normalizeAngle(angle)`** — wraps angle to [-180, 180] range.

**`segmentIntersectsCircle(x1,y1, x2,y2, px,py, radius)`** — checks if the line segment from (x1,y1) to (x2,y2) passes within `radius` of point (px,py). Solves the quadratic equation for parameter t along the segment. Returns true if any t in [0,1] is within the circle, or if the segment is entirely inside. This is necessary because at high speed a pod can fly over a checkpoint zone in a single frame — a simple distance check at the endpoint would miss it.

### SimState (lines 115–129)

Simulation-time pod state, separate from the persistent `Pod` struct:

| Field | Type | Meaning |
|-------|------|---------|
| `x, y` | double | Position (double precision during sim, truncated at end of each step) |
| `vx, vy` | double | Velocity vector |
| `facingAngle` | double | Absolute facing angle in degrees |
| `cpsPassed` | int | Checkpoints passed during this simulation run |
| `shieldTurnsLeft` | int | Remaining shield cooldown turns |
| `shieldCollided` | bool | Whether a collision occurred while shield was active (for scoring bonus) |

Methods:
- `distTo(px, py)` — Euclidean distance to a point
- `angleTo(px, py)` — angle in degrees to a point (via `atan2`)

---

## Pod — Main Structure (lines 131–585)

### State Fields

**Persistent across turns:**
- `position`, `prevPosition` — current and previous frame position (for velocity calc)
- `vx, vy` — computed velocity (delta of positions between frames)
- `opponentPos`, `prevOpponentPos` — opponent position tracking
- `opVx, opVy` — computed opponent velocity
- `facingAngleDeg` — absolute facing angle, derived each frame
- `targetPosition` — output target point
- `thrust` — output thrust value
- `shieldCooldown` — turns remaining before shield can be used again
- `zeroThrustFrames` — consecutive frames where thrust was 0 (for stuck detection)
- `boostAvailable` — one-time boost, starts `true`
- `currentLap` — current lap number (starts at 1)
- `prevCpIndex` — last checkpoint index passed (for lap counting)
- `firstFrame`, `opFirstFrame` — flags to skip velocity computation on frame 1

**Track detection:**
- `gameCheckpoints` — vector of discovered checkpoint positions
- `trackComplete` — true once the checkpoint loop is detected

### Checkpoint Tracking & Lap Counting

Since the Silver League input does not provide the full checkpoint list, the bot discovers it at runtime:

**`initStartPosition(x, y)`** — called on frame 1. Records the pod's starting position as `gameCheckpoints[0]`. This is not a real checkpoint but a placeholder; it gets overwritten with exact coordinates once the loop is detected.

**`trackCheckpoint(cp)`** — called every frame with the current target checkpoint:
1. If track is not complete:
   - If `gameCheckpoints.size() > 1` and `cp` is close to `gameCheckpoints[0]` (within 750 units, using squared distance — `isCloseTo`), the loop is detected:
     - `gameCheckpoints[0]` is overwritten with the exact checkpoint coordinates (replacing the approximate start position)
     - `trackComplete = true`
   - Otherwise, if `cp` differs from the last known checkpoint, it's appended
2. If track is complete:
   - Finds the index of the current CP via `getCpIndex`
   - Detects lap transitions: when `cpIdx == 0` and `prevCpIndex > 0`, increments `currentLap`

**`getNextCheckpoint(cp)`**:
- Before track completion: returns map center `(8000, 4500)` as a dummy next-CP (used in scoring — since we don't know the real next checkpoint yet, center of map is a reasonable estimate)
- After track completion: returns `gameCheckpoints[(idx+1) % size]`

**`isFinalLap()`** — returns `true` when `currentLap >= TOTAL_LAPS` (3).

### State Computation from Input

**`trackVelocity(newX, newY, outVx, outVy, prevPos, isFirstFrame)`** — generic velocity tracker: on first call, just stores position; on subsequent calls, velocity = new position - previous position. Updates `prevPos` and clears `isFirstFrame`.

**`computeVelocity(x, y)`** — calls `trackVelocity` for the pod's own velocity.

**`computeOpponent(ox, oy)`** — calls `trackVelocity` for the opponent's velocity, also updates `opponentPos`.

**`computeFacingAngle(cpX, cpY)`** — derives absolute facing angle:
```
directionToCheckpoint = atan2(cpY - podY, cpX - podX)
facingAngle = directionToCheckpoint - next_checkpoint_angle
```
The game provides `next_checkpoint_angle` as the relative angle between pod facing direction and the checkpoint. Subtracting it from the absolute direction to the checkpoint yields the absolute facing direction.

### Opponent Prediction

**`precomputeOpponent()`** — called once per frame at the start of `findBestMove()`. Fills `opPredictions[0..SIM_DEPTH]` with predicted opponent positions/velocities:
- Step 0: current position and velocity
- Each subsequent step: linear movement (`pos += vel`), then friction (`vel = int(vel * 0.85)`)

This assumes the opponent continues in a straight line without turning — a simplification that works reasonably well for short prediction horizons.

### Collision Physics

**`applyCollision(s, ox, oy, ovx, ovy, myMass)`** — elastic collision between the simulated pod state `s` and an opponent at (ox,oy) with velocity (ovx,ovy):

1. **Distance check**: if `distSq >= COLLISION_RADIUS^2` (800^2 = 640000) — no collision. Sqrt only computed when collision confirmed.
2. **Normal vector**: unit vector from opponent to pod.
3. **Relative velocity**: `dv = podVel - opVel`. Projects onto normal: `dvn = dot(dv, normal)`.
4. **Separation check**: if `dvn > 0`, pods are already moving apart — skip.
5. **Impulse**: `max(MIN_COLLISION_IMPULSE, -2 * opponentMass * dvn / totalMass)`. Opponent mass is always `NORMAL_MASS` (1.0). Pod mass is either 1.0 (normal) or 10.0 (shield). Minimum impulse is 120.
6. **Apply**: pod velocity adjusted by `impulse * normal`. Position pushed apart by half the overlap distance.
7. Returns `true` if collision occurred.

Note: only the pod's state is modified — opponent state comes from pre-computed predictions and is treated as immutable.

### Simulation Engine

**`simStep(s, targetAngle, simThrust)`** — one frame of game physics:
1. **Rotation**: compute angle difference to `targetAngle`, clamp to `[-18, +18]` degrees, apply to `facingAngle`
2. **Thrust**: acceleration in facing direction: `vx += thrust * cos(facing)`, `vy += thrust * sin(facing)`
3. **Movement**: `x += vx`, `y += vy`
4. **Friction**: `vx = int(vx * 0.85)`, `vy = int(vy * 0.85)` — truncation to int matches game engine
5. **Position truncation**: `x = int(x)`, `y = int(y)`

**`resolveThrust(simThrust, s)`** — handles shield mechanics:
- If `simThrust == SHIELD_THRUST_VALUE` (-1): sets `shieldTurnsLeft = 3`, returns 0 thrust
- If `shieldTurnsLeft > 0`: decrements cooldown, returns 0 thrust (pod can't accelerate during shield recovery)
- Otherwise: returns `simThrust` unchanged

**`getMass(originalThrust)`** — returns `SHIELD_MASS` (10.0) if thrust is shield activation, else `NORMAL_MASS` (1.0).

**`simCandidate(s, aimAngle, simThrust, step, simTarget, arrivalFrame, currentCp, nextCp)`** — full single-step simulation combining all mechanics:
1. Determine mass from thrust (before resolving)
2. Resolve thrust (shield handling)
3. Save previous position
4. Run `simStep` with resolved thrust
5. Apply collision with predicted opponent at `opPredictions[step+1]`
6. If collision happened with heavy mass (shield), set `shieldCollided = true`
7. **Dynamic CP radius**: if opponent is within `COLLISION_RADIUS` of pod, checkpoint detection radius shrinks by `CP_RADIUS_REDUCTION` (590 -> 580). This accounts for potential position displacement from collisions making CP detection harder.
8. Check if the movement segment (prevPos -> newPos) intersects the checkpoint circle via `segmentIntersectsCircle`
9. If checkpoint passed: increment `cpsPassed`, record `arrivalFrame`, advance `simTarget` to next CP

**`angleDependentThrust(aimAngle, facingAngle)`** — heuristic thrust for rollout: `max(20, 100 - |angleDiff|)`. Reduces thrust when facing away from target, ensuring the pod doesn't waste energy accelerating in the wrong direction.

**`rollout(s, fromStep, ...)`** — after the 3 explicitly searched steps, simulates remaining steps (3 through `SIM_DEPTH-1`) using the `angleDependentThrust` heuristic aimed at the current target checkpoint. No branching — single trajectory.

### Scoring

**`evaluateScore(simPod, arrivalFrame, currentCp, nextCp)`**:

Shield collision bonus: `+SHIELD_COLLISION_BONUS` (18000) if `shieldCollided` is true. Applied in all cases below.

**Case 1: Checkpoint(s) reached** (`cpsPassed > 0`):
```
score = cpsPassed * 50000 + (SIM_DEPTH - arrivalFrame) * 5000 + bonus
```
- Each checkpoint passed is worth 50000 — dominates scoring
- Earlier arrival earns up to `SIM_DEPTH * 5000` bonus (35000 max)
- **Final lap or first lap (nextCp == currentCp, i.e., track not yet complete)**:
  - `+= speedSq * 0.025` — reward maintaining speed (no sqrt — uses squared speed)
- **Middle laps**:
  - `-= distToNextCP` — penalize distance to the checkpoint after the one just passed (positioning for next turn)

**Case 2: No checkpoint reached**:
```
score = -distance_to_CP + speedToward * K + bonus
```
- `speedToward` = projection of velocity onto direction to checkpoint (dot product / distance)
- `K = 10.0` on final lap (heavily rewards speed toward goal)
- `K = 3.0` on other laps (balanced between speed and proximity)

### Search Algorithm — findBestMove()

The core decision engine. Called once per frame.

**Setup:**
1. `precomputeOpponent()` — fill opponent prediction array
2. Build initial `SimState` from current pod state
3. Identify `currentCp` and `nextCp`
4. Determine boost eligibility: `boostAvailable && trackComplete && dist > 4000`
5. Determine shield eligibility: `shieldCooldown == 0 && opponentDist < 1200 && relativeSpeedSq > 125000`
6. Build step-0 thrust array dynamically: always includes normal thrusts `[10,35,55,75,100,0]`, conditionally adds 650 (boost) and -1 (shield)

**Three-level exhaustive search:**

- **Step 0**: 14 angle offsets (relative to direction-to-checkpoint) x 6-8 thrust options = 84-112 branches
  - Angles: `[-50, -36, -24, -18, -12, -6, 0, 6, 12, 18, 24, 36, 50, 180]` relative to CP direction
  - 180 is a full reversal — useful for braking or dodging
- **Step 1**: 6 angle offsets x 4 thrust options = 24 branches per step-0 branch
  - Angles: `[-24, -12, 0, 12, 24, 180]` relative to direction from step-0 result to target
- **Step 2**: 6 angle offsets x 4 thrust options = 24 branches per step-1 branch
  - Same structure as step 1

**Total candidates evaluated**: 84 x 24 x 24 = **48,384** (normal) up to 112 x 24 x 24 = **64,512** (with boost + shield).

Each candidate is then extended by a **heuristic rollout** for steps 3-6 (4 more frames), giving a 7-frame lookahead.

**Three separate best scores** are tracked:
- `bestScoreNormal` + best angle/thrust for normal moves
- `bestScoreBoost` + best angle for boost
- `bestScoreShield` + best angle for shield

**Decision logic:**
1. If boost is available AND `bestScoreBoost > bestScoreNormal + 10000`: use BOOST
2. Else if shield is available AND `bestScoreShield > bestScoreNormal + 0`: use SHIELD
3. Else: use the best normal move

**Output encoding**: the chosen angle is converted to a target point 10000 units away from the pod in that direction. This is because the game expects a target coordinate, not an angle.

### Output — navigate()

1. Decrement `shieldCooldown` if active
2. **Stuck detection**: if thrust has been 0 for more than `ZERO_THRUST_MAX_FRAMES` (7) consecutive frames, override thrust to 20. Prevents getting permanently stuck.
3. Output: `targetX targetY BOOST|SHIELD|thrust`
4. Side effects: `boostAvailable = false` after boost, `shieldCooldown = SHIELD_COOLDOWN` after shield

---

## Main Loop (lines 588–619)

**Per frame:**
1. Read pod state: `x, y, next_checkpoint_x, next_checkpoint_y, next_checkpoint_dist, next_checkpoint_angle`
2. Read opponent: `opponent_x, opponent_y`
3. Frame 1 only: `initStartPosition(x, y)`
4. `trackCheckpoint(cp)` — discover checkpoints, detect laps
5. `computeVelocity(x, y)` — derive pod velocity from position delta
6. `computeOpponent(opponent_x, opponent_y)` — derive opponent velocity
7. Set pod state: position, angle, distance, target
8. `computeFacingAngle(cpX, cpY)` — derive absolute facing from relative angle
9. `findBestMove()` — run search
10. `navigate()` — output command

---

## Key Design Decisions

- **No checkpoint list from input**: the bot discovers checkpoints on the fly, completing the track map after one lap. Until then, scoring uses map center as a proxy for "next checkpoint".
- **Integer truncation in simulation**: velocity and position are truncated to int after each step, matching the game engine's behavior. This is critical for accurate multi-frame predictions.
- **Opponent treated as read-only**: collision physics only modifies the pod's state, not the opponent's. Opponent follows pre-computed linear trajectory. This is a simplification — in reality the opponent also gets pushed — but it keeps the search tractable.
- **Three separate best-score buckets**: boost and shield are evaluated independently and only chosen if they provide a significant advantage over normal play. This prevents the bot from using boost/shield when a normal move is nearly as good.
- **Segment-circle intersection for CP detection**: at high speeds (thrust 100 + existing velocity), a pod can move 500+ units per frame. The checkpoint radius is 590. A simple endpoint-distance check would miss cases where the pod flies through the checkpoint zone. The segment intersection catches these.
- **Dynamic CP radius reduction**: when near an opponent (within collision radius), checkpoint detection uses a slightly smaller radius (580 vs 590). This accounts for collision displacement potentially pushing the pod out of the checkpoint zone.
