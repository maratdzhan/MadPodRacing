#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

// ===== Constants =====

// Physics
constexpr double FRICTION = 0.85;
constexpr double MAX_TURN = 18.0 * M_PI / 180.0;
constexpr double CP_RADIUS = 590.0;
constexpr double CP_RADIUS_REDUCTION = 10.0;
constexpr double COLLISION_RADIUS = 800.0;
constexpr double MIN_COLLISION_IMPULSE = 120.0;
constexpr double DEG_TO_RAD = M_PI / 180.0;

// Simulation
constexpr int SIM_DEPTH = 4;
constexpr int ZERO_THRUST_MAX_FRAMES = 7;
constexpr int ZERO_THRUST_OVERRIDE = 20;
constexpr double TARGET_POINT_DIST = 10000.0;

// Scoring
constexpr double SCORE_CP_PASSED = 50000.0;
constexpr double SCORE_EARLY_ARRIVAL = 5000.0;
constexpr double SCORE_SPEED_WEIGHT_FINAL = 0.025;
constexpr double SCORE_SPEED_TOWARD_NORMAL = 3.0;
constexpr double SCORE_SPEED_TOWARD_FINAL = 15.0;
constexpr double SCORE_DIST_TO_NEXT_WEIGHT = 2.0;

// Boost
constexpr int BOOST_MIN_DIST = 3500;
constexpr int BOOST_MIN_LAP = 1;
constexpr double BOOST_SCORE_THRESHOLD = 10000.0;

// Shield
constexpr int SHIELD_THRUST_VALUE = -1;
constexpr int SHIELD_COOLDOWN = 3;
constexpr double SHIELD_MASS = 10.0;
constexpr double NORMAL_MASS = 1.0;
constexpr double SHIELD_CHECK_DIST = 1200.0;
constexpr double SHIELD_SCORE_THRESHOLD = 0.0;
constexpr double SHIELD_COLLISION_BONUS = 18000.0;
constexpr double SHIELD_COLLISION_BONUS_BLOCKER = 35000.0;
constexpr double SHIELD_MIN_REL_SPEED_SQ = 125000.0;
constexpr double BOOST_COLLISION_BONUS_BLOCKER = 12500.0;
constexpr double TEAMMATE_COLLISION_PENALTY = 25000.0;
constexpr double TEAMMATE_COLLISION_PENALTY_PER_LAP = 15000.0;
constexpr double INTERCEPT_URGENCY_PER_ENEMY = 0.51;
constexpr double INTERCEPT_DIST_THRESHOLD = 2000.0;
constexpr double INTERCEPT_PATH_RADIUS = 2000.0;

// Search: step 0
constexpr double ANGLE_OFFSETS_1[] = {
    -50*DEG_TO_RAD, -24*DEG_TO_RAD, -18*DEG_TO_RAD, -12*DEG_TO_RAD, -6*DEG_TO_RAD, 0,
    6*DEG_TO_RAD, 12*DEG_TO_RAD, 18*DEG_TO_RAD, 24*DEG_TO_RAD, 50*DEG_TO_RAD, M_PI
};
constexpr int THRUST_OPTIONS_1[] = {10, 35, 55, 75, 100, 0};
constexpr int ANGLE_COUNT_1 = sizeof(ANGLE_OFFSETS_1) / sizeof(ANGLE_OFFSETS_1[0]);
constexpr int THRUST_COUNT_1 = sizeof(THRUST_OPTIONS_1) / sizeof(THRUST_OPTIONS_1[0]);

// Search: steps 1-2
constexpr double ANGLE_OFFSETS_23[] = {
    -24*DEG_TO_RAD, -12*DEG_TO_RAD, 0, 12*DEG_TO_RAD, 24*DEG_TO_RAD, M_PI
};
constexpr int THRUST_OPTIONS_23[] = {0, 10, 45, 75};
constexpr int ANGLE_COUNT_23 = sizeof(ANGLE_OFFSETS_23) / sizeof(ANGLE_OFFSETS_23[0]);
constexpr int THRUST_COUNT_23 = sizeof(THRUST_OPTIONS_23) / sizeof(THRUST_OPTIONS_23[0]);

// ===== Global game state =====

int totalLaps;
int checkpointCount;

struct Point {
    int x, y;
    Point(int x=0, int y=0) : x(x), y(y) {}
    bool operator==(const Point& p) const { return x == p.x && y == p.y; }
    bool operator!=(const Point& p) const { return !(*this == p); }
    Point operator+(const Point& p) const { return Point(x + p.x, y + p.y); }
    bool operator<(const Point& p) const {
        if (x != p.x) return x < p.x;
        return y < p.y;
    }
    friend ostream& operator<<(ostream& os, const Point& p) {
        os << "{" << p.x << ";" << p.y << "}";
        return os;
    }
};

Point checkpoints[20];
double distNormCoeff = 1.0;

template<typename K>
ostream& operator<<(ostream& os, const vector<K>& v) {
    for (auto it = v.begin(); it != v.end(); ++it) {
        if (it != v.begin()) { os << ";"; }
        os << *it;
    }
    return os;
}

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

bool segmentIntersectsCircle(double x1, double y1, double x2, double y2,
                              double px, double py, double radius) {
    double dx = x2 - x1, dy = y2 - y1;
    double fx = x1 - px, fy = y1 - py;
    double a = dx*dx + dy*dy;
    if (a < 1e-9) return (fx*fx + fy*fy <= radius*radius);
    double b = 2*(fx*dx + fy*dy);
    double c = fx*fx + fy*fy - radius*radius;
    double disc = b*b - 4*a*c;
    if (disc < 0) return false;
    double sqrtD = sqrt(disc);
    double t1 = (-b - sqrtD) / (2*a);
    double t2 = (-b + sqrtD) / (2*a);
    return (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1) || (t1 < 0 && t2 > 1);
}

struct SimState {
    double x, y, vx, vy, facingAngle; // facingAngle in radians
    int cpsPassed;
    int shieldTurnsLeft;
    bool shieldCollided;
    bool boostCollided;
    bool teammateCollided;

    double distTo(double px, double py) const {
        double dx = px - x, dy = py - y;
        return sqrt(dx*dx + dy*dy);
    }

    double angleTo(double px, double py) const {
        return atan2(py - y, px - x);
    }
};

struct Pod {
    Pod() : shieldCooldown(0), zeroThrustFrames(0), boostAvailable(true),
            currentLap(1), totalCpsPassed(0), prevNextCpId(-1) {}

    // Current state (from input)
    double x, y, vx, vy;
    double facingAngle; // radians
    int nextCpId;

    // Persistent state
    int shieldCooldown;
    int zeroThrustFrames;
    bool boostAvailable;
    int currentLap;
    int totalCpsPassed;
    int prevNextCpId;

    // Role
    bool isBlocker = false;
    double interceptUrgency = 0.0; // 0.0 = race, 0.5 = one enemy ahead, 1.0 = both enemies ahead
    double leadEnemyX, leadEnemyY;
    int leadEnemyIdx = -1; // index in others[] array (1 or 2)
    int leadEnemyNextCpId = 0; // enemy's next checkpoint

    // Output
    Point targetPosition;
    int thrust;
    double chosenScore;
    int podIndex = 0;
    vector<int> scoreHistory = vector<int>(500);

    // Other pods for collision prediction
    static constexpr int MAX_OTHERS = 3;
    struct OtherPodState {
        double x, y, vx, vy;
    };
    OtherPodState others[MAX_OTHERS];
    int otherCount;

    struct PodPrediction {
        double x, y, vx, vy;
    };
    PodPrediction otherPredictions[MAX_OTHERS][SIM_DEPTH + 1];

    void setState(int px, int py, int pvx, int pvy, int angleDeg, int cpId) {
        x = px; y = py;
        vx = pvx; vy = pvy;
        facingAngle = angleDeg * DEG_TO_RAD;
        nextCpId = cpId;
        targetPosition = checkpoints[cpId];

        if (prevNextCpId >= 0 && cpId != prevNextCpId) {
            totalCpsPassed++;
            currentLap = 1 + totalCpsPassed / checkpointCount;
        }
        prevNextCpId = cpId;
    }

    void setOthers(const OtherPodState* o, int count) {
        otherCount = count;
        for (int i = 0; i < count; i++) others[i] = o[i];
    }

    bool isFinalLap() const {
        return currentLap >= totalLaps;
    }

    Point getNextCp() const {
        return checkpoints[nextCpId];
    }

    Point getAfterNextCp() const {
        return checkpoints[(nextCpId + 1) % checkpointCount];
    }

    // Teammate's chosen move (set externally before findBestMove for pod[1])
    bool hasTeammateMove = false;
    double teammateAngle; // radians
    int teammateThrust;
    double teammateFacing; // radians

    void precomputeOthers() {
        for (int p = 0; p < otherCount; p++) {
            double ox = others[p].x, oy = others[p].y;
            double ovx = others[p].vx, ovy = others[p].vy;
            otherPredictions[p][0] = {ox, oy, ovx, ovy};

            // For teammate (p==0): use real move for step 1
            if (p == 0 && hasTeammateMove) {
                SimState ts = {ox, oy, ovx, ovy, teammateFacing, 0, 0, false};
                ts = simStep(ts, teammateAngle, teammateThrust);
                ox = ts.x; oy = ts.y;
                ovx = ts.vx; ovy = ts.vy;
                otherPredictions[p][1] = {ox, oy, ovx, ovy};
                for (int i = 2; i <= SIM_DEPTH; i++) {
                    ox += ovx; oy += ovy;
                    ovx = (int)(ovx * FRICTION);
                    ovy = (int)(ovy * FRICTION);
                    otherPredictions[p][i] = {ox, oy, ovx, ovy};
                }
            } else {
                for (int i = 1; i <= SIM_DEPTH; i++) {
                    ox += ovx; oy += ovy;
                    ovx = (int)(ovx * FRICTION);
                    ovy = (int)(ovy * FRICTION);
                    otherPredictions[p][i] = {ox, oy, ovx, ovy};
                }
            }
        }
    }

    bool applyCollision(SimState& s, double ox, double oy, double ovx, double ovy, double myMass = NORMAL_MASS) {
        double dx = s.x - ox;
        double dy = s.y - oy;
        double distSq = dx*dx + dy*dy;
        if (distSq >= COLLISION_RADIUS * COLLISION_RADIUS || distSq < 1e-9) return false;
        double dist = sqrt(distSq);
        double nx = dx / dist;
        double ny = dy / dist;
        double dvx = s.vx - ovx;
        double dvy = s.vy - ovy;
        double dvn = dvx * nx + dvy * ny;
        if (dvn > 0) return false;
        double totalMass = myMass + NORMAL_MASS;
        double impulse = max(MIN_COLLISION_IMPULSE, -2.0 * NORMAL_MASS * dvn / totalMass);
        s.vx += impulse * nx;
        s.vy += impulse * ny;
        double overlap = COLLISION_RADIUS - dist;
        s.x += overlap * nx * 0.5;
        s.y += overlap * ny * 0.5;
        return true;
    }

    SimState simStep(SimState s, double targetAngle, int simThrust) {
        double diff = normalizeAngle(targetAngle - s.facingAngle);
        if (diff > MAX_TURN) diff = MAX_TURN;
        if (diff < -MAX_TURN) diff = -MAX_TURN;
        s.facingAngle += diff;
        s.vx += simThrust * cos(s.facingAngle);
        s.vy += simThrust * sin(s.facingAngle);
        s.x += s.vx;
        s.y += s.vy;
        s.vx = (int)(s.vx * FRICTION);
        s.vy = (int)(s.vy * FRICTION);
        s.x = (int)s.x;
        s.y = (int)s.y;
        return s;
    }

    int resolveThrust(int simThrust, SimState& s) const {
        if (simThrust == SHIELD_THRUST_VALUE) {
            s.shieldTurnsLeft = SHIELD_COOLDOWN;
            return 0;
        }
        if (s.shieldTurnsLeft > 0) {
            s.shieldTurnsLeft--;
            return 0;
        }
        return simThrust;
    }

    double getMass(int originalThrust) const {
        return (originalThrust == SHIELD_THRUST_VALUE) ? SHIELD_MASS : NORMAL_MASS;
    }

    bool anyOtherNear(const SimState& s, int step) const {
        for (int p = 0; p < otherCount; p++) {
            const auto& op = otherPredictions[p][step];
            double dx = s.x - op.x, dy = s.y - op.y;
            if (dx*dx + dy*dy < COLLISION_RADIUS * COLLISION_RADIUS) return true;
        }
        return false;
    }

    SimState simCandidate(SimState s, double aimAngle, int simThrust, int step,
                          int& simNextCpId, int& arrivalFrame,
                          const Point& currentCp, const Point& nextCp) {
        double myMass = getMass(simThrust);
        int actualThrust = resolveThrust(simThrust, s);
        double prevX = s.x, prevY = s.y;
        s = simStep(s, aimAngle, actualThrust);

        for (int p = 0; p < otherCount; p++) {
            const auto& op = otherPredictions[p][step+1];
            bool hit = applyCollision(s, op.x, op.y, op.vx, op.vy, myMass);
            if (hit) {
                if (p == 0) {
                    s.teammateCollided = true;
                } else {
                    if (myMass > NORMAL_MASS) s.shieldCollided = true;
                    if (simThrust == 650 && p == leadEnemyIdx) s.boostCollided = true;
                }
            }
        }

        double cpRadius = anyOtherNear(s, step+1) ? CP_RADIUS - CP_RADIUS_REDUCTION : CP_RADIUS;

        Point simTarget = checkpoints[simNextCpId];
        if (segmentIntersectsCircle(prevX, prevY, s.x, s.y,
                                     simTarget.x, simTarget.y, cpRadius)) {
            s.cpsPassed++;
            if (arrivalFrame < 0) arrivalFrame = step + 1;
            simNextCpId = (simNextCpId + 1) % checkpointCount;
        }
        return s;
    }

    int angleDependentThrust(double aimAngle, double facingAngle) const {
        return max(20, (int)(100 - abs(normalizeAngle(aimAngle - facingAngle)) / DEG_TO_RAD));
    }

    SimState rollout(SimState s, int fromStep, int& simNextCpId, int& arrivalFrame,
                     const Point& currentCp, const Point& nextCp) {
        for (int step = fromStep; step < SIM_DEPTH; step++) {
            Point target = checkpoints[simNextCpId];
            double aimAngle = s.angleTo(target.x, target.y);
            int rollThrust = angleDependentThrust(aimAngle, s.facingAngle);
            s = simCandidate(s, aimAngle, rollThrust, step, simNextCpId, arrivalFrame, currentCp, nextCp);
        }
        return s;
    }

    double collisionBonus(const SimState& simPod) const {
        double bonus = 0.0;
        if (simPod.shieldCollided) {
            bonus = isBlocker ? SHIELD_COLLISION_BONUS_BLOCKER : SHIELD_COLLISION_BONUS;
        } else if (simPod.boostCollided && isBlocker) {
            bonus = BOOST_COLLISION_BONUS_BLOCKER;
        }
        if (simPod.teammateCollided) {
            bonus -= TEAMMATE_COLLISION_PENALTY + TEAMMATE_COLLISION_PENALTY_PER_LAP * currentLap;
        }
        return bonus;
    }

    double approachScore(const SimState& simPod, double tx, double ty, double speedWeight = SCORE_SPEED_TOWARD_NORMAL) const {
        double dist = simPod.distTo(tx, ty);
        double dx = tx - simPod.x;
        double dy = ty - simPod.y;
        double norm = max(1.0, dist);
        double speedToward = (simPod.vx * dx + simPod.vy * dy) / norm;
        return -dist * distNormCoeff + speedToward * speedWeight;
    }

    double evaluateScore(const SimState& simPod, int arrivalFrame,
                         const Point& currentCp, const Point& nextCp) {
        double bonus = collisionBonus(simPod);

        if (simPod.cpsPassed > 0) {
            double score = simPod.cpsPassed * SCORE_CP_PASSED
                         + (SIM_DEPTH - arrivalFrame) * SCORE_EARLY_ARRIVAL + bonus;
            score -= simPod.distTo(nextCp.x, nextCp.y) * SCORE_DIST_TO_NEXT_WEIGHT;
            return score;
        }

        double raceScore = approachScore(simPod, currentCp.x, currentCp.y);

        if (isBlocker && interceptUrgency > 0.0) {
            Point enemyCp = checkpoints[leadEnemyNextCpId];
            double interceptScore = approachScore(simPod, enemyCp.x, enemyCp.y, SCORE_SPEED_TOWARD_FINAL);
            return max(raceScore, interceptScore) + bonus;
        }
        return raceScore + bonus;
    }

    double closestEnemyDistSq() const {
        double minDistSq = 1e18;
        for (int i = 1; i < otherCount; i++) {
            double dx = x - others[i].x, dy = y - others[i].y;
            double d = dx*dx + dy*dy;
            if (d < minDistSq) minDistSq = d;
        }
        return minDistSq;
    }

    double closestEnemyRelSpeedSq() const {
        double maxRelSq = 0;
        for (int i = 1; i < otherCount; i++) {
            double dx = x - others[i].x, dy = y - others[i].y;
            double distSq = dx*dx + dy*dy;
            if (distSq < SHIELD_CHECK_DIST * SHIELD_CHECK_DIST) {
                double rvx = vx - others[i].vx, rvy = vy - others[i].vy;
                double relSq = rvx*rvx + rvy*rvy;
                if (relSq > maxRelSq) maxRelSq = relSq;
            }
        }
        return maxRelSq;
    }

    void findBestMove() {
        precomputeOthers();

        SimState initial = {
            x, y, vx, vy, facingAngle, 0, shieldCooldown, false, false, false
        };

        Point currentCp = getNextCp();
        Point nextCp = getAfterNextCp();

        double bestScoreNormal = -1e18, bestScoreBoost = -1e18, bestScoreShield = -1e18;
        int bestThrustNormal = 100;
        double bestAngleNormal = 0, bestAngleBoost = 0, bestAngleShield = 0;

        // If we'll pass through current CP this frame and no enemy nearby, aim at next CP
        bool willReachCp = segmentIntersectsCircle(x, y, x + vx, y + vy,
                                                    currentCp.x, currentCp.y, CP_RADIUS);
        bool enemyFar = closestEnemyDistSq() > 4 * COLLISION_RADIUS * COLLISION_RADIUS;
        Point aimCp = (willReachCp && enemyFar) ? nextCp : currentCp;

        double dirToCp;
        if (isBlocker && interceptUrgency > 0.0) {
            Point enemyCp = checkpoints[leadEnemyNextCpId];
            dirToCp = atan2(enemyCp.y - y, enemyCp.x - x);
        } else if (isBlocker) {
            dirToCp = atan2(leadEnemyY - y, leadEnemyX - x);
        } else {
            dirToCp = atan2(aimCp.y - y, aimCp.x - x);
        }
        double distToCp = sqrt((currentCp.x - x) * (currentCp.x - x) +
                               (currentCp.y - y) * (currentCp.y - y));

        bool canBoost = boostAvailable && currentLap >= BOOST_MIN_LAP && distToCp > BOOST_MIN_DIST;

        double minOtherDistSq = closestEnemyDistSq();
        double maxRelSpeedSq = closestEnemyRelSpeedSq();
        bool canShield = shieldCooldown == 0
            && minOtherDistSq < SHIELD_CHECK_DIST * SHIELD_CHECK_DIST
            && maxRelSpeedSq > SHIELD_MIN_REL_SPEED_SQ;

        if (minOtherDistSq < SHIELD_CHECK_DIST * SHIELD_CHECK_DIST) {
            cerr << "Collision: relV²=" << maxRelSpeedSq
                 << " dist=" << sqrt(minOtherDistSq)
                 << " shield=" << (canShield ? "available" : "no") << endl;
        }

        // Build step 0 thrust options
        int step0Thrusts[THRUST_COUNT_1 + 2];
        int step0ThrustCount = THRUST_COUNT_1;
        for (int i = 0; i < THRUST_COUNT_1; i++) step0Thrusts[i] = THRUST_OPTIONS_1[i];
        if (canBoost) step0Thrusts[step0ThrustCount++] = 650;
        if (canShield) step0Thrusts[step0ThrustCount++] = SHIELD_THRUST_VALUE;

        for (int ai1 = 0; ai1 < ANGLE_COUNT_1; ai1++) {
            for (int ti1 = 0; ti1 < step0ThrustCount; ti1++) {
                double moveAngle1 = dirToCp + ANGLE_OFFSETS_1[ai1];
                int simCpId0 = nextCpId;
                int arrival0 = -1;
                SimState afterStep0 = simCandidate(initial, moveAngle1, step0Thrusts[ti1],
                                                    0, simCpId0, arrival0, currentCp, nextCp);

                for (int ai2 = 0; ai2 < ANGLE_COUNT_23; ai2++) {
                    for (int ti2 = 0; ti2 < THRUST_COUNT_23; ti2++) {
                        Point target1 = checkpoints[simCpId0];
                        double dirFromStep0 = afterStep0.angleTo(target1.x, target1.y);
                        int simCpId1 = simCpId0;
                        int arrival1 = arrival0;
                        SimState afterStep1 = simCandidate(afterStep0, dirFromStep0 + ANGLE_OFFSETS_23[ai2],
                                                            THRUST_OPTIONS_23[ti2], 1, simCpId1, arrival1,
                                                            currentCp, nextCp);

                        for (int ai3 = 0; ai3 < ANGLE_COUNT_23; ai3++) {
                            for (int ti3 = 0; ti3 < THRUST_COUNT_23; ti3++) {
                                Point target2 = checkpoints[simCpId1];
                                double dirFromStep1 = afterStep1.angleTo(target2.x, target2.y);
                                int simCpId = simCpId1;
                                int arrivalFrame = arrival1;
                                SimState simPod = simCandidate(afterStep1, dirFromStep1 + ANGLE_OFFSETS_23[ai3],
                                                               THRUST_OPTIONS_23[ti3], 2, simCpId, arrivalFrame,
                                                               currentCp, nextCp);

                                simPod = rollout(simPod, 3, simCpId, arrivalFrame, currentCp, nextCp);

                                double score = evaluateScore(simPod, arrivalFrame, currentCp, nextCp);
                                int t1 = step0Thrusts[ti1];
                                if (t1 == 650) {
                                    if (score > bestScoreBoost) {
                                        bestScoreBoost = score;
                                        bestAngleBoost = moveAngle1;
                                    }
                                } else if (t1 == SHIELD_THRUST_VALUE) {
                                    if (score > bestScoreShield) {
                                        bestScoreShield = score;
                                        bestAngleShield = moveAngle1;
                                    }
                                } else {
                                    if (score > bestScoreNormal) {
                                        bestScoreNormal = score;
                                        bestThrustNormal = t1;
                                        bestAngleNormal = moveAngle1;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // Use boost only if it gives significant advantage
        if (canBoost && bestScoreBoost > bestScoreNormal + BOOST_SCORE_THRESHOLD) {
            thrust = 650;
            chosenScore = bestScoreBoost;
            targetPosition = Point(
                (int)(x + TARGET_POINT_DIST * cos(bestAngleBoost)),
                (int)(y + TARGET_POINT_DIST * sin(bestAngleBoost))
            );
            return;
        }

        // Use shield only if it gives significant advantage
        if (canShield) {
            cerr << "Shield scores: shield=" << bestScoreShield
                 << " normal=" << bestScoreNormal
                 << " diff=" << bestScoreShield - bestScoreNormal << endl;
        }
        if (canShield && bestScoreShield > bestScoreNormal + SHIELD_SCORE_THRESHOLD) {
            thrust = SHIELD_THRUST_VALUE;
            chosenScore = bestScoreShield;
            targetPosition = Point(
                (int)(x + TARGET_POINT_DIST * cos(bestAngleShield)),
                (int)(y + TARGET_POINT_DIST * sin(bestAngleShield))
            );
            return;
        }

        thrust = bestThrustNormal;
        chosenScore = bestScoreNormal;
        targetPosition = Point(
            (int)(x + TARGET_POINT_DIST * cos(bestAngleNormal)),
            (int)(y + TARGET_POINT_DIST * sin(bestAngleNormal))
        );
    }

    void navigate() {
        if (shieldCooldown > 0) shieldCooldown--;

        if (thrust == 0) {
            zeroThrustFrames++;
            if (zeroThrustFrames > ZERO_THRUST_MAX_FRAMES) {
                thrust = ZERO_THRUST_OVERRIDE;
                cerr << "Zero thrust override!" << endl;
            }
        } else {
            zeroThrustFrames = 0;
        }

        if (isBlocker) {
            cerr << "P" << podIndex << " blocker urg=" << interceptUrgency
                 << " eCpId=" << leadEnemyNextCpId
                 << " thrust=" << thrust << endl;
        }

        string msg = (podIndex == 0) ? "raka maka foo" : "fristaylo";
        cout << targetPosition.x << " " << targetPosition.y << " ";
        if (thrust == 650) {
            cout << "BOOST " << msg << endl;
            boostAvailable = false;
            cerr << "BOOST activated!" << endl;
        } else if (thrust == SHIELD_THRUST_VALUE) {
            cout << "SHIELD " << msg << endl;
            shieldCooldown = SHIELD_COOLDOWN;
            cerr << "SHIELD activated!" << endl;
        } else {
            cout << thrust << " " << msg << endl;
        }
    }

    static void assignRoles(Pod& a, Pod& b) {
        if (a.totalCpsPassed > b.totalCpsPassed) {
            a.isBlocker = false;
            b.isBlocker = true;
        } else if (b.totalCpsPassed > a.totalCpsPassed) {
            a.isBlocker = true;
            b.isBlocker = false;
        } else {
            double da = a.distSqToNextCp();
            double db = b.distSqToNextCp();
            a.isBlocker = da > db;
            b.isBlocker = db >= da;
        }
    }

    double distSqToNextCp() const {
        double dx = checkpoints[nextCpId].x - x;
        double dy = checkpoints[nextCpId].y - y;
        return dx*dx + dy*dy;
    }

    static bool isEnemyAhead(int enemyCps, int ourBestCps, double enemyDistSq, double ourBestDistSq) {
        if (enemyCps > ourBestCps) return true;
        if (enemyCps == ourBestCps && ourBestDistSq - enemyDistSq > INTERCEPT_DIST_THRESHOLD * INTERCEPT_DIST_THRESHOLD) return true;
        return false;
    }

    static void calcInterceptUrgency(Pod& a, Pod& b, int enemyCps[], int px[], int py[], int pcpid[]) {
        int ourBestCpsPassed = max(a.totalCpsPassed, b.totalCpsPassed);
        double ourBestDistSq = min(a.distSqToNextCp(), b.distSqToNextCp());

        int enemiesAhead = 0;
        for (int e = 0; e < 2; e++) {
            double edx = checkpoints[pcpid[2+e]].x - px[2+e];
            double edy = checkpoints[pcpid[2+e]].y - py[2+e];
            double enemyDistSq = edx*edx + edy*edy;
            if (isEnemyAhead(enemyCps[e], ourBestCpsPassed, enemyDistSq, ourBestDistSq)) enemiesAhead++;
        }

        // Check if blocker's path to own CP passes near enemy's CP
        Pod& blocker = a.isBlocker ? a : b;
        Point blockerCp = blocker.getNextCp();
        int leadOp = findLeadOpponent(px, py, pcpid);
        Point enemyCp = checkpoints[pcpid[leadOp]];
        bool pathCrossesEnemy = segmentIntersectsCircle(
            blocker.x, blocker.y, blockerCp.x, blockerCp.y,
            enemyCp.x, enemyCp.y, INTERCEPT_PATH_RADIUS);
        if (pathCrossesEnemy && enemiesAhead == 0) enemiesAhead = 1;

        double urgency = enemiesAhead * INTERCEPT_URGENCY_PER_ENEMY;
        a.interceptUrgency = a.isBlocker ? urgency : 0.0;
        b.interceptUrgency = b.isBlocker ? urgency : 0.0;
    }

    static int findLeadOpponent(int px[], int py[], int pcpid[]) {
        if (pcpid[3] > pcpid[2]) return 3;
        if (pcpid[2] > pcpid[3]) return 2;
        double d2 = (checkpoints[pcpid[2]].x - px[2]) * (checkpoints[pcpid[2]].x - px[2])
                   + (checkpoints[pcpid[2]].y - py[2]) * (checkpoints[pcpid[2]].y - py[2]);
        double d3 = (checkpoints[pcpid[3]].x - px[3]) * (checkpoints[pcpid[3]].x - px[3])
                   + (checkpoints[pcpid[3]].y - py[3]) * (checkpoints[pcpid[3]].y - py[3]);
        return (d3 < d2) ? 3 : 2;
    }

    void receiveTeammateMove(const Pod& teammate) {
        hasTeammateMove = true;
        teammateAngle = atan2(teammate.targetPosition.y - teammate.y,
                              teammate.targetPosition.x - teammate.x);
        teammateThrust = (teammate.thrust == 650) ? 650 :
                          (teammate.thrust == SHIELD_THRUST_VALUE ? 0 : teammate.thrust);
        teammateFacing = teammate.facingAngle;
    }
};

void computeDistNormCoeff() {
    double totalDist = 0;
    for (int i = 0; i < checkpointCount; i++) {
        int ni = (i + 1) % checkpointCount;
        double dx = checkpoints[ni].x - checkpoints[i].x;
        double dy = checkpoints[ni].y - checkpoints[i].y;
        totalDist += sqrt(dx*dx + dy*dy);
    }
    double avgDist = totalDist / checkpointCount;
    double ratio = avgDist / 18000.0;
    distNormCoeff = ratio * ratio;
}

int main() {
    cin >> totalLaps; cin.ignore();
    cin >> checkpointCount; cin.ignore();
    for (int i = 0; i < checkpointCount; i++) {
        cin >> checkpoints[i].x >> checkpoints[i].y; cin.ignore();
    }

    computeDistNormCoeff();

    Pod pods[2];
    pods[0].podIndex = 0;
    pods[1].podIndex = 1;
    int turn = 0;
    bool showResults = true;
    int enemyCpsPassed[2] = {0, 0};
    int enemyPrevCpId[2] = {-1, -1};

    while (true) {
        turn++;
        int px[4], py[4], pvx[4], pvy[4], pangle[4], pcpid[4];
        for (int i = 0; i < 4; i++) {
            cin >> px[i] >> py[i] >> pvx[i] >> pvy[i] >> pangle[i] >> pcpid[i]; cin.ignore();
        }

        for (int i = 0; i < 2; i++) {
            pods[i].setState(px[i], py[i], pvx[i], pvy[i], pangle[i], pcpid[i]);
        }

        for (int e = 0; e < 2; e++) {
            int cpId = pcpid[2 + e];
            if (enemyPrevCpId[e] >= 0 && cpId != enemyPrevCpId[e]) enemyCpsPassed[e]++;
            enemyPrevCpId[e] = cpId;
        }

        Pod::assignRoles(pods[0], pods[1]);

        int leadOp = Pod::findLeadOpponent(px, py, pcpid);
        Pod::calcInterceptUrgency(pods[0], pods[1], enemyCpsPassed, px, py, pcpid);
        for (int i = 0; i < 2; i++) {
            pods[i].leadEnemyX = px[leadOp];
            pods[i].leadEnemyY = py[leadOp];
            pods[i].leadEnemyNextCpId = pcpid[leadOp];
            pods[i].leadEnemyIdx = (leadOp == 2) ? 1 : 2; // others[1]=px[2], others[2]=px[3]

            Pod::OtherPodState others[3];
            int other = 1 - i;
            others[0] = {(double)px[other], (double)py[other], (double)pvx[other], (double)pvy[other]};
            others[1] = {(double)px[2], (double)py[2], (double)pvx[2], (double)pvy[2]};
            others[2] = {(double)px[3], (double)py[3], (double)pvx[3], (double)pvy[3]};
            pods[i].setOthers(others, 3);
        }

        pods[0].hasTeammateMove = false;
        pods[0].findBestMove();

        pods[1].receiveTeammateMove(pods[0]);
        pods[1].findBestMove();

        for (int i = 0; i < 2; i++) {
            if (turn <= 500) pods[i].scoreHistory[turn - 1] = (int)pods[i].chosenScore;
        }

        pods[0].navigate();
        pods[1].navigate();

        if (showResults) {
            bool anyReady = false;
            for (int i = 0; i < 2; i++) {
                if (pods[i].currentLap == totalLaps
                    && pods[i].nextCpId == 0
                    && pods[i].distSqToNextCp() < 3000.0 * 3000.0) {
                    anyReady = true;
                }
            }
            if (anyReady) {
                int count = min(turn, 500);
                for (int i = 0; i < 2; i++) {
                    int sum = 0;
                    for (int t = 0; t < count; t++) sum += pods[i].scoreHistory[t];
                    cerr << "Pod" << i << " score: " << sum
                         << " turns: " << count << endl;
                }
                showResults = false;
            }
        }
    }
}
