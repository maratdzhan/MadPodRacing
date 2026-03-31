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

## Current Architecture Summary

- **Two pods**: racer + blocker, roles assigned each frame
- **Blocker**: aims at enemy's next CP, intercept urgency based on enemy lead
- **Search**: 3-deep + 1 rollout, 12×6 → 6×4 → 6×4 branching
- **Scoring**: `max(raceScore, interceptScore)` with quadratic dist normalization
- **Collisions**: 3 other pods, separate tracking (shield/boost/teammate)
- **All radians**, single deg→rad conversion at input
