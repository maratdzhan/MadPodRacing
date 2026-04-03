# Pod Racing Bot

Bot for CodinGame Mad Pod Racing (Gold League input format). Controls two pods racing across an arena (16000x9000) from checkpoint to checkpoint, competing against two opponent pods. All checkpoints, velocities, and angles are provided by the game at init and each turn.

## Input Format (Gold League, Two Pods)

**Initialization (once):**
- Line 1: `laps` — number of laps to complete
- Line 2: `checkpointCount` — number of checkpoints
- Next `checkpointCount` lines: `checkpointX checkpointY` — coordinates of each checkpoint

**Output per turn:** two lines (one per own pod): `target_x target_y thrust|BOOST|SHIELD message`

---

## Type Hierarchy

```
SimpleStateCoords {x, y}          — position only, base for everything
SimpleStateSpeeds {vx, vy}        — velocity only

PodBaseState : SimpleStateCoords, SimpleStateSpeeds {modified}
  — lightweight move data for othersAfterMove/othersFinal in collision sim
  — 5 fields, no methods, trivial construction

PodBase : SimpleStateCoords, SimpleStateSpeeds {facingAngle, nextCpId, modified}
  — enemy tracking in search (enemies0/1/2 arrays)
  — ~56 bytes, update/get methods, no prev-tracking
  — static: boostHoldTurns, currentTurn

SimState : PodBase {prevX, prevY, cpsPassed, shieldTurnsLeft,
                    shieldCollided, boostCollided, teammateCollided, minEnemyDistSq}
  — our pod during simulation, ~80 bytes
  — updateX/Y saves prevX/prevY (for checkpoint detection)
  — setAfterCollisionX/Y writes without saving prev (for post-collision adjustments)
  — getPrevX/Y for checkpoint segment start point

Pod : PodBase {shieldCooldown, boostAvailable, currentLap, totalCpsPassed, ...}
  — full game-time pod with persistent state, search, navigation, scoring
  — contains others[3] (PodBase), otherPredictions, all simulation methods
```

Design principle: heavier types only where needed. Enemies use `PodBase` (~56 bytes), move data uses `PodBaseState` (~40 bytes), only our simulated pod uses `SimState` (~80 bytes).

---

## Constants

All tunable parameters are `constexpr` at the top. All angles in **radians** — single deg→rad conversion at input.

| Group | Key Constants |
|-------|---------------|
| Physics | `FRICTION=0.85`, `CP_RADIUS=585`, `COLLISION_RADIUS=800`, `MAX_TURN=18°`, `MIN_COLLISION_IMPULSE=120` |
| Simulation | `SIM_DEPTH=4`, `TARGET_POINT_DIST=10000` |
| Scoring | `SCORE_CP_PASSED=25000`, `SCORE_EARLY_ARRIVAL=5000`, `SCORE_SPEED_TOWARD_NORMAL=3.0`, `SCORE_SPEED_TOWARD_FINAL=15.0`, `SCORE_DIST_TO_NEXT_WEIGHT=5.0` |
| Boost | `BOOST_MIN_DIST=4500`, `BOOST_MIN_LAP=1`, `BOOST_MIN_TURN=10`, `BOOST_SCORE_THRESHOLD=10000` |
| Shield | `SHIELD_COOLDOWN=3`, `SHIELD_MASS=10`, `SHIELD_COLLISION_BONUS=18000` (racer) / `35000` (blocker) |
| Intercept | `INTERCEPT_URGENCY_PER_ENEMY=0.51`, `INTERCEPT_DIST_THRESHOLD=2000`, `INTERCEPT_PATH_RADIUS=2000`, `BLOCKER_PASSIVE_INTERCEPT_WEIGHT=0.25` |
| Search step 0 | `ANGLE_OFFSETS_1[]`: -27°, -15°, -7°, 0, +7°, +15°, +27°, π (8 angles), `THRUST_OPTIONS_1[]`: 10, 42, 75, 100, 0 (5 thrusts) |
| Search steps 1-2 | `ANGLE_OFFSETS_23[]`: -45°, -15°, 0, +15°, +45°, π (6 angles), `THRUST_OPTIONS_23[]`: 10, 42, 75, 100 (4 thrusts) |

---

## Global State

- `totalLaps`, `checkpointCount`, `checkpoints[20]` — from initialization input
- `distNormCoeff` — `(avgDist / 18000)²`, normalizes `-dist` in scoring across track sizes

---

## Helper Functions

**`normalizeAngle(angle)`** — wraps to [-π, π] via `fmod` + two `if`. O(1).

**`segmentIntersectsCircle(x1,y1, x2,y2, circleX,circleY, radius)`** — quadratic solve for segment-circle intersection. Used for checkpoint detection and path-crossing intercept.

**`computeDistNormCoeff()`** — computes `distNormCoeff = (avgDist / 18000)²`.

---

## Pod Methods — Simulation Engine

### applyRotationAndThrust(podState, targetAngle, simThrust)

One-step rotation + thrust on a `PodBase&`. Clamps rotation to `MAX_TURN` (18°), applies thrust in facing direction. Modifies `facingAngle`, `vx`, `vy`.

### findCollisionTime(startA, endA, startB, endB, radius)

Static. Takes 4 `SimpleStateCoords` (start/end positions of two objects) + collision radius. Solves quadratic for time t ∈ [0,1] when objects are within radius. Returns t or -1.

### getNextCpIdIfIntersects(from, to, circle, radius, nextCpId)

Checks if segment from→to passes through checkpoint circle. Returns `(nextCpId + 1) % checkpointCount` if yes, else `nextCpId`. Takes `SimpleStateCoords` refs.

### predictPodStep(state, target, podIdx, step)

Predicts one step for another pod (`PodBase& state`). Aims at `target`, computes `angleDependentThrust`, applies rotation+thrust via temp `PodBase`, saves prediction in `otherPredictions[podIdx][step]`, advances position with friction and int truncation. For teammate at step 0 (`hasTeammateMove && step == 0`): uses actual teammate move instead of heuristic.

### precomputeOthers()

Runs `predictPodStep` for all 3 other pods over `SIM_DEPTH` steps. Fills `otherPredictions[3][5]` array. Teammate step 0 uses real move if available.

### computeEnemyMoves(step, others[], othersAfterMove[])

Fills `PodBaseState othersAfterMove[]` with each pod's start position and post-thrust velocity for this step. For modified enemies: recomputes aim + `applyRotationAndThrust` from current state, updates `others[].facingAngle`. For unmodified: reads from `otherPredictions`.

### applyMutualCollision(simState, enemy, myMass, otherMass)

Elastic collision between our pod (`SimState&`) and enemy (`PodBaseState&`). Checks distance < `COLLISION_RADIUS` and closing velocity. Applies impulse `max(120, 2*m1*m2*|dvn|/(m1+m2))`. Overlap push uses `setAfterCollisionX/Y` on simState (preserves prevX) and direct writes on enemy.

### resolveThrust(simThrust, simState)

Handles shield activation (`shieldTurnsLeft = 3`, return 0) and cooldown (return 0 while cooling). Otherwise returns simThrust.

### fastPath(simState, step, simNextCpId, arrivalFrame)

No-collision, no-modified-enemies path. Moves position, applies friction+truncation in one `updateX` call (preserves prevX). Tracks min enemy distance, detects checkpoint crossing via `segmentIntersectsCircle(getPrevX, getPrevY, getX, getY, ...)`.

### slowPath(simState, simThrust, step, simNextCpId, arrivalFrame, earliestCollisionPod, earliestCollisionTime, othersAfterMove[], others[])

Full collision path. Creates `PodBaseState othersFinal[]` (end-of-step positions). If collision detected: interpolates to collision point (`updateX` — saves prevX), resolves collision (`applyMutualCollision`), continues remaining movement (`setAfterCollisionX` — preserves prevX). Checks secondary collisions with other pods. Calls `updateEnemyStates` to propagate changes. Applies friction. Detects checkpoint crossing.

### simCandidate(simState, aimAngle, simThrust, step, simNextCpId&, arrivalFrame&, others[])

Main simulation step. Resolves thrust, applies rotation+thrust. Calls `computeEnemyMoves` once. Runs collision check loop using `othersAfterMove` data + `findCollisionTime`. If no collision and no modified enemies → `fastPath` + early return. Otherwise → `slowPath`. Returns modified `SimState`. `simNextCpId` and `arrivalFrame` passed by reference — checkpoint progress propagates to caller.

### updateEnemyStates(step, others[], otherAfter[])

Updates `PodBase others[]` after simulation step. For collided/modified pods: saves `SimpleStateCoords enemyPositionBeforeMove`, writes new position+velocity with friction, checks checkpoint crossing via `getNextCpIdIfIntersects(enemyPositionBeforeMove, otherAfter, ...)`. For unmodified: copies from `otherPredictions[step+1]`.

### rollout(simState, fromStep, simNextCpId&, arrivalFrame&, others[])

Greedy continuation: aims at current checkpoint, uses `angleDependentThrust`, calls `simCandidate` for remaining steps.

---

## Pod Methods — Scoring

### collisionBonus(simPod)

Shield collision: +18000 (racer) / +35000 (blocker). Boost collision with lead enemy (blocker): +12500. Teammate collision: -(25000 + 15000 * currentLap).

### approachScore(simPod, tx, ty, speedWeight)

`-dist * distNormCoeff + speedToward * speedWeight`. `distNormCoeff = (avgDist/18000)²` prevents distance from dominating on large tracks.

### evaluateScore(simPod, arrivalFrame, currentCp, nextCp)

If checkpoint passed: `cpsPassed * 25000 + (SIM_DEPTH - arrivalFrame) * 5000 - distToNextCP * 5.0 + bonus`.
If not passed, racer: `approachScore(currentCp, weight=3.0) + bonus`.
If not passed, blocker with urgency: `max(raceScore, interceptScore) + bonus` where interceptScore uses `minEnemyDistSq`.
If not passed, blocker without urgency: `raceScore + interceptScore(enemyCp, weight=15.0) * 0.25 + bonus`.

---

## Pod Methods — Search

### searchForBestThrust(initial, dirToCp, currentCp, nextCp, thrusts[], ...)

Step 0 search loop. For each angle × thrust combination: creates fresh `PodBase enemies0[3]`, calls `simCandidate` for step 0, then `searchFromStep1` for deeper search. Tracks best scores for normal/boost/shield separately.

### searchFromStep1(afterStep0, simCpId0, arrival0, currentCp, nextCp, enemies0[])

Steps 1-2 nested loops. For each angle × thrust at step 1: `simCandidate`, then for each angle × thrust at step 2: `simCandidate` + `rollout` for step 3. Returns best score across all step 1-2-3 combinations.

### selectBestAction(canBoost, canShield, scores...)

Picks best action: boost if score > normal + 10000, shield if score > normal + 0, otherwise normal thrust.

### findBestMove()

Entry point. Calls `precomputeOthers`, determines aim direction (early CP switch for racer, enemy CP for blocker with urgency), builds thrust options (adding boost/shield if eligible), calls `searchForBestThrust`, calls `selectBestAction`.

---

## Pod Methods — Roles & Setup

### assignRoles(a, b)

Pod further ahead (by `totalCpsPassed`, then `distSqToNextCp`) = racer. Other = blocker.

### calcInterceptUrgency(a, b, enemyCps[], ...)

Counts enemies ahead. Triggers: enemy ahead by CPs, ahead by distance (> 2000 closer), or blocker path crosses enemy CP (`segmentIntersectsCircle`). Urgency = `enemiesAhead * 0.51`.

### findLeadOpponent(podX[], podY[], podCpId[], enemyCps[])

Returns index (2 or 3) of leading enemy by `enemyCpsPassed`, then distance to next CP.

### setupLeadEnemy(pods[], leadOp, podX[], podY[], podCpId[])

Sets `leadEnemyX/Y/NextCpId/Idx` on both pods from `podX/Y/CpId[leadOp]`.

### setupOtherPods(pods[], podX[], podY[], podVx[], podVy[], podAngle[], podCpId[])

Creates `PodBase others[3]` (teammate + 2 enemies) for each pod via aggregate initialization.

### setupTurn(pods[], podX[], ...)

Orchestrates per-turn setup: `assignRoles` → `findLeadOpponent` → `calcInterceptUrgency` → `setupLeadEnemy` → `setupOtherPods`.

---

## Pod Methods — Output

### receiveTeammateMove(teammate)

Extracts teammate's chosen angle, thrust, facing for `precomputeOthers` step 0 prediction.

### navigate()

Decrements shield cooldown. Low-thrust override: if thrust < 20 for > 4 frames → force 100 for 4 frames. Outputs `targetX targetY BOOST|SHIELD|thrust message`.

---

## Main Loop

1. Read laps, checkpoints, `computeDistNormCoeff()`, create `pods[2]`
2. Per turn:
   - Read 4 pod states, track enemy `cpsPassed`
   - `setState()` for own pods (deg→rad, lap tracking)
   - `Pod::setupTurn()` — roles, intercept urgency, lead enemy, other pods
   - Racer: `findBestMove()` (no teammate info)
   - Blocker: `receiveTeammateMove(racer)` then `findBestMove()`
   - `chrono` timing: `T{turn} R={μs} B={μs}` to stderr
   - Both pods: `navigate()`
   - Near finish: output total scores to stderr
