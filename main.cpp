#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

// ===== Constants =====

// Physics
constexpr double FRICTION = 0.85;
constexpr double MAX_TURN_DEG = 18.0;
constexpr double CP_RADIUS = 590.0;
constexpr double CP_RADIUS_REDUCTION = 10.0;
constexpr double COLLISION_RADIUS = 800.0;
constexpr double MIN_COLLISION_IMPULSE = 120.0;

// Simulation
constexpr int SIM_DEPTH = 7;
constexpr int TOTAL_LAPS = 3;
constexpr int ZERO_THRUST_MAX_FRAMES = 7;
constexpr int ZERO_THRUST_OVERRIDE = 20;
constexpr double TARGET_POINT_DIST = 10000.0;

// Search: step 0
constexpr double ANGLE_OFFSETS_1[] = {-50, -36, -24, -18, -12, -6, 0, 6, 12, 18, 24, 36, 50, 180};
constexpr int THRUST_OPTIONS_1[] = {10, 35, 55, 75, 100, 0};
constexpr int THRUST_OPTIONS_1_BOOST[] = {10, 35, 55, 75, 100, 0, 650};
constexpr int ANGLE_COUNT_1 = sizeof(ANGLE_OFFSETS_1) / sizeof(ANGLE_OFFSETS_1[0]);
constexpr int THRUST_COUNT_1 = sizeof(THRUST_OPTIONS_1) / sizeof(THRUST_OPTIONS_1[0]);
constexpr int THRUST_COUNT_1_BOOST = sizeof(THRUST_OPTIONS_1_BOOST) / sizeof(THRUST_OPTIONS_1_BOOST[0]);

// Search: steps 1-2
constexpr double ANGLE_OFFSETS_23[] = {-24, -12, 0, 12, 24, 180};
constexpr int THRUST_OPTIONS_23[] = {0, 10, 45, 75};
constexpr int ANGLE_COUNT_23 = sizeof(ANGLE_OFFSETS_23) / sizeof(ANGLE_OFFSETS_23[0]);
constexpr int THRUST_COUNT_23 = sizeof(THRUST_OPTIONS_23) / sizeof(THRUST_OPTIONS_23[0]);

// Scoring
constexpr double SCORE_CP_PASSED = 50000.0;
constexpr double SCORE_EARLY_ARRIVAL = 5000.0;
constexpr double SCORE_SPEED_WEIGHT_FINAL = 0.025;
constexpr double SCORE_SPEED_TOWARD_NORMAL = 3.0;
constexpr double SCORE_SPEED_TOWARD_FINAL = 10.0;
constexpr double SCORE_DIST_TO_NEXT_WEIGHT = 1.0;

// Boost
constexpr int BOOST_MIN_DIST = 4000;
constexpr double BOOST_SCORE_THRESHOLD = 10000.0;

// Shield
constexpr int SHIELD_THRUST_VALUE = -1;
constexpr int SHIELD_COOLDOWN = 3;
constexpr double SHIELD_MASS = 10.0;
constexpr double NORMAL_MASS = 1.0;
constexpr double SHIELD_CHECK_DIST = 1200.0;
constexpr double SHIELD_SCORE_THRESHOLD = 0.0;
constexpr double SHIELD_COLLISION_BONUS = 18000.0;
constexpr double SHIELD_MIN_REL_SPEED_SQ = 125000.0;

// Track detection
constexpr double CLOSE_DIST_THRESHOLD = 750.0;
constexpr double MAP_CENTER_X = 8000.0;
constexpr double MAP_CENTER_Y = 4500.0;

template<typename K>
ostream& operator<<(ostream& os, const vector<K>& v) {
    for (auto it = v.begin(); it != v.end(); ++it) {
        if (it != v.begin()) { os << ";"; }
        os << *it;
    }
    return os;
}

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

double normalizeAngle(double angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

// Check if line segment (x1,y1)→(x2,y2) passes within radius of (px,py)
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
    double x, y, vx, vy, facingAngle; // facingAngle in degrees
    int cpsPassed;
    int shieldTurnsLeft;
    bool shieldCollided;

    double distTo(double px, double py) const {
        double dx = px - x, dy = py - y;
        return sqrt(dx*dx + dy*dy);
    }

    double angleTo(double px, double py) const {
        return atan2(py - y, px - x) * 180.0 / M_PI;
    }
};

struct Pod {

    Pod() : vx(0), vy(0), opVx(0), opVy(0), opFirstFrame(true),
            facingAngleDeg(0), firstFrame(true), shieldCooldown(0),
            zeroThrustFrames(0), trackComplete(false), boostAvailable(true),
            currentLap(1), prevCpIndex(-1) {}


    // Track original game checkpoints (without midpoints)
    vector<Point> gameCheckpoints;
    bool trackComplete;
    bool boostAvailable;
    int currentLap;
    int prevCpIndex;

    // Call once on first frame with pod's starting position
    void initStartPosition(int x, int y) {
        gameCheckpoints.push_back(Point(x, y));
    }

    bool isCloseTo(const Point& a, const Point& b) const {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return dx*dx + dy*dy < CLOSE_DIST_THRESHOLD * CLOSE_DIST_THRESHOLD;
    }

    void trackCheckpoint(const Point& cp) {
        if (!trackComplete) {
            // Check if this CP is close to start position (loop detected)
            if (gameCheckpoints.size() > 1 && isCloseTo(cp, gameCheckpoints[0])) {
                cerr << "Start pos replaced: " << gameCheckpoints[0] << " -> " << cp << endl;
                gameCheckpoints[0] = cp; // overwrite start pos with exact CP coords
                trackComplete = true;
                cerr << "Track complete: " << gameCheckpoints << endl;
            } else if (cp != gameCheckpoints.back()) {
                gameCheckpoints.push_back(cp);
            }
        }
        if (trackComplete) {
            int cpIdx = getCpIndex(cp);
            if (cpIdx >= 0 && cpIdx != prevCpIndex) {
                if (cpIdx == 0 && prevCpIndex > 0) {
                    currentLap++;
                    cerr << "Lap " << currentLap << endl;
                }
                prevCpIndex = cpIdx;
            }
        }
    }

    int getCpIndex(const Point& cp) const {
        for (int i = 0; i < (int)gameCheckpoints.size(); i++) {
            if (gameCheckpoints[i] == cp) return i;
        }
        return -1;
    }

    Point getNextCheckpoint(const Point& currentCp) const {
        if (!trackComplete) return Point((int)MAP_CENTER_X, (int)MAP_CENTER_Y);
        int idx = getCpIndex(currentCp);
        if (idx >= 0) return gameCheckpoints[(idx + 1) % gameCheckpoints.size()];
        return currentCp;
    }

    bool isFinalLap() const {
        return currentLap >= TOTAL_LAPS;
    }

    void setPos(int x, int y) {
        position.x = x;
        position.y = y;
    }

    void setDistance(int dist) {
        next_checkpoint_dist = dist;
    }

    void setNextCheckpointAngle(int a) {
        next_checkpoint_angle = a;
    }

    void setTargetPosition(const Point& t) {
        targetPosition = t;
    }

    void trackVelocity(int newX, int newY, double& outVx, double& outVy,
                        Point& prevPos, bool& isFirstFrame) {
        if (!isFirstFrame) {
            outVx = newX - prevPos.x;
            outVy = newY - prevPos.y;
        }
        prevPos = Point(newX, newY);
        isFirstFrame = false;
    }

    void computeVelocity(int newX, int newY) {
        trackVelocity(newX, newY, vx, vy, prevPosition, firstFrame);
    }

    void computeOpponent(int ox, int oy) {
        trackVelocity(ox, oy, opVx, opVy, prevOpponentPos, opFirstFrame);
        opponentPos = prevOpponentPos;
    }

    // Derive absolute facing angle from game's relative angle
    void computeFacingAngle(int cpX, int cpY) {
        double dirToCp = atan2(cpY - position.y, cpX - position.x) * 180.0 / M_PI;
        facingAngleDeg = dirToCp - next_checkpoint_angle;
    }

    // Precomputed opponent predictions for each step
    struct OpponentPrediction {
        double x, y, vx, vy;
    };
    OpponentPrediction opPredictions[SIM_DEPTH + 1];

    void precomputeOpponent() {
        double ox = opponentPos.x, oy = opponentPos.y;
        double ovx = opVx, ovy = opVy;
        opPredictions[0] = {ox, oy, ovx, ovy};
        for (int i = 1; i <= SIM_DEPTH; i++) {
            ox += ovx;
            oy += ovy;
            ovx = (int)(ovx * FRICTION);
            ovy = (int)(ovy * FRICTION);
            opPredictions[i] = {ox, oy, ovx, ovy};
        }
    }

    // Apply elastic collision between simPod and opponent
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

        // Impulse with mass: opponent mass is always 1
        double totalMass = myMass + NORMAL_MASS;
        double impulse = max(MIN_COLLISION_IMPULSE, -2.0 * NORMAL_MASS * dvn / totalMass);

        s.vx += impulse * nx;
        s.vy += impulse * ny;

        double overlap = COLLISION_RADIUS - dist;
        s.x += overlap * nx * 0.5;
        s.y += overlap * ny * 0.5;
        return true;
    }

    // Simulate one frame of game physics
    SimState simStep(SimState s, double targetAngle, int simThrust) {
        // 1. Rotation: max 18 deg per frame
        double diff = normalizeAngle(targetAngle - s.facingAngle);
        if (diff > MAX_TURN_DEG) diff = MAX_TURN_DEG;
        if (diff < -MAX_TURN_DEG) diff = -MAX_TURN_DEG;
        s.facingAngle += diff;

        // 2. Thrust applied in facing direction
        double rad = s.facingAngle * M_PI / 180.0;
        s.vx += simThrust * cos(rad);
        s.vy += simThrust * sin(rad);

        // 3. Movement
        s.x += s.vx;
        s.y += s.vy;

        // 4. Friction (truncate to int, as the game does)
        s.vx = (int)(s.vx * FRICTION);
        s.vy = (int)(s.vy * FRICTION);

        // 5. Truncate position
        s.x = (int)s.x;
        s.y = (int)s.y;

        return s;
    }

    // Resolve thrust considering shield state
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

    // Get mass for collision (shield active = heavy)
    double getMass(int originalThrust) const {
        return (originalThrust == SHIELD_THRUST_VALUE) ? SHIELD_MASS : NORMAL_MASS;
    }

    // Simulate one candidate step: apply move, collision, checkpoint detection
    SimState simCandidate(SimState s, double aimAngle, int simThrust, int step,
                          Point& simTarget, int& arrivalFrame,
                          const Point& currentCp, const Point& nextCp) {
        double myMass = getMass(simThrust);
        int actualThrust = resolveThrust(simThrust, s);

        double prevX = s.x, prevY = s.y;
        s = simStep(s, aimAngle, actualThrust);
        const auto& op = opPredictions[step+1];
        bool collided = applyCollision(s, op.x, op.y, op.vx, op.vy, myMass);
        if (collided && myMass > NORMAL_MASS) s.shieldCollided = true;
        double opDistSq = (s.x - op.x)*(s.x - op.x) + (s.y - op.y)*(s.y - op.y);
        double cpRadius = (opDistSq < COLLISION_RADIUS * COLLISION_RADIUS) ? CP_RADIUS - CP_RADIUS_REDUCTION : CP_RADIUS;
        if (segmentIntersectsCircle(prevX, prevY, s.x, s.y,
                                     simTarget.x, simTarget.y, cpRadius)) {
            s.cpsPassed++;
            if (arrivalFrame < 0) arrivalFrame = step + 1;
            if (simTarget == currentCp) simTarget = nextCp;
        }
        return s;
    }

    int angleDependentThrust(double aimAngle, double facingAngle) const {
        return max(20, (int)(100 - abs(normalizeAngle(aimAngle - facingAngle))));
    }

    // Run heuristic rollout for remaining steps
    SimState rollout(SimState s, int fromStep, Point& simTarget, int& arrivalFrame,
                     const Point& currentCp, const Point& nextCp) {
        for (int step = fromStep; step < SIM_DEPTH; step++) {
            double aimAngle = s.angleTo(simTarget.x, simTarget.y);
            // Shield cooldown handled inside simCandidate via resolveThrust
            int rollThrust = angleDependentThrust(aimAngle, s.facingAngle);
            s = simCandidate(s, aimAngle, rollThrust, step, simTarget, arrivalFrame, currentCp, nextCp);
        }
        return s;
    }

    // Evaluate score for a completed simulation
    double evaluateScore(const SimState& simPod, int arrivalFrame,
                         const Point& currentCp, const Point& nextCp) {
        double bonus = simPod.shieldCollided ? SHIELD_COLLISION_BONUS : 0.0;

        if (simPod.cpsPassed > 0) {
            double score = simPod.cpsPassed * SCORE_CP_PASSED
                         + (SIM_DEPTH - arrivalFrame) * SCORE_EARLY_ARRIVAL + bonus;
            if (isFinalLap() || nextCp == currentCp) {
                double speedSq = simPod.vx * simPod.vx + simPod.vy * simPod.vy;
                score += speedSq * SCORE_SPEED_WEIGHT_FINAL;
            } else {
                double distToNext = simPod.distTo(nextCp.x, nextCp.y);
                score -= distToNext * SCORE_DIST_TO_NEXT_WEIGHT;
            }
            return score;
        }
        double dist = simPod.distTo(currentCp.x, currentCp.y);
        double dx = currentCp.x - simPod.x;
        double dy = currentCp.y - simPod.y;
        double norm = max(1.0, dist);
        double speedToward = (simPod.vx * dx + simPod.vy * dy) / norm;
        if (isFinalLap()) {
            return -dist + speedToward * SCORE_SPEED_TOWARD_FINAL + bonus;
        }
        return -dist + speedToward * SCORE_SPEED_TOWARD_NORMAL + bonus;
    }

    // Try different (angle, thrust) for first 3 moves, simulate N frames, pick best
    void findBestMove() {
        precomputeOpponent();

        SimState initial = {
            (double)position.x, (double)position.y,
            vx, vy, facingAngleDeg, 0, shieldCooldown, false
        };

        Point currentCp = targetPosition;
        currentCpIndex = getCpIndex(currentCp);
        Point nextCp = getNextCheckpoint(currentCp);

        double bestScoreNormal = -1e18, bestScoreBoost = -1e18, bestScoreShield = -1e18;
        int bestThrustNormal = 100, bestThrustBoost = 650, bestThrustShield = SHIELD_THRUST_VALUE;
        double bestAngleNormal = 0, bestAngleBoost = 0, bestAngleShield = 0;

        double dirToCp = atan2(currentCp.y - position.y,
                               currentCp.x - position.x) * 180.0 / M_PI;

        bool canBoost = boostAvailable && trackComplete && next_checkpoint_dist > BOOST_MIN_DIST;
        double opDistSq = (position.x - opponentPos.x) * (position.x - opponentPos.x)
                        + (position.y - opponentPos.y) * (position.y - opponentPos.y);
        double relVxSq = (vx - opVx) * (vx - opVx) + (vy - opVy) * (vy - opVy);
        bool canShield = shieldCooldown == 0
            && opDistSq < SHIELD_CHECK_DIST * SHIELD_CHECK_DIST
            && relVxSq > SHIELD_MIN_REL_SPEED_SQ;
        if (opDistSq < SHIELD_CHECK_DIST * SHIELD_CHECK_DIST) {
            cerr << "Collision: relV²=" << relVxSq
                 << " vx=" << vx << " vy=" << vy
                 << " opVx=" << opVx << " opVy=" << opVy
                 << " dist=" << sqrt(opDistSq)
                 << " shield=" << (canShield ? "available" : "no") << endl;
        }

        // Build step 0 thrust options dynamically
        int step0Thrusts[THRUST_COUNT_1 + 2]; // +boost +shield max
        int step0ThrustCount = THRUST_COUNT_1;
        for (int i = 0; i < THRUST_COUNT_1; i++) step0Thrusts[i] = THRUST_OPTIONS_1[i];
        if (canBoost) step0Thrusts[step0ThrustCount++] = 650;
        if (canShield) step0Thrusts[step0ThrustCount++] = SHIELD_THRUST_VALUE;

        for (int ai1 = 0; ai1 < ANGLE_COUNT_1; ai1++) {
            for (int ti1 = 0; ti1 < step0ThrustCount; ti1++) {
                // Step 0
                double moveAngle1 = dirToCp + ANGLE_OFFSETS_1[ai1];
                Point simTarget0 = currentCp;
                int arrival0 = -1;
                SimState afterStep0 = simCandidate(initial, moveAngle1, step0Thrusts[ti1],
                                                    0, simTarget0, arrival0, currentCp, nextCp);

                for (int ai2 = 0; ai2 < ANGLE_COUNT_23; ai2++) {
                    for (int ti2 = 0; ti2 < THRUST_COUNT_23; ti2++) {
                        // Step 1
                        double dirFromStep0 = afterStep0.angleTo(simTarget0.x, simTarget0.y);
                        Point simTarget1 = simTarget0;
                        int arrival1 = arrival0;
                        SimState afterStep1 = simCandidate(afterStep0, dirFromStep0 + ANGLE_OFFSETS_23[ai2],
                                                            THRUST_OPTIONS_23[ti2], 1, simTarget1, arrival1,
                                                            currentCp, nextCp);

                        for (int ai3 = 0; ai3 < ANGLE_COUNT_23; ai3++) {
                            for (int ti3 = 0; ti3 < THRUST_COUNT_23; ti3++) {
                                // Step 2
                                double dirFromStep1 = afterStep1.angleTo(simTarget1.x, simTarget1.y);
                                Point simTarget = simTarget1;
                                int arrivalFrame = arrival1;
                                SimState simPod = simCandidate(afterStep1, dirFromStep1 + ANGLE_OFFSETS_23[ai3],
                                                               THRUST_OPTIONS_23[ti3], 2, simTarget, arrivalFrame,
                                                               currentCp, nextCp);

                                // Heuristic rollout for remaining steps
                                simPod = rollout(simPod, 3, simTarget, arrivalFrame, currentCp, nextCp);

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
            double rad = bestAngleBoost * M_PI / 180.0;
            targetPosition = Point(
                position.x + (int)(TARGET_POINT_DIST * cos(rad)),
                position.y + (int)(TARGET_POINT_DIST * sin(rad))
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
            double rad = bestAngleShield * M_PI / 180.0;
            targetPosition = Point(
                position.x + (int)(TARGET_POINT_DIST * cos(rad)),
                position.y + (int)(TARGET_POINT_DIST * sin(rad))
            );
            return;
        }

        thrust = bestThrustNormal;
        double rad = bestAngleNormal * M_PI / 180.0;
        targetPosition = Point(
            position.x + (int)(TARGET_POINT_DIST * cos(rad)),
            position.y + (int)(TARGET_POINT_DIST * sin(rad))
        );

    }


    void navigate() {
        // Decrement shield cooldown each frame
        if (shieldCooldown > 0) shieldCooldown--;

        // Failsafe: if stuck at thrust=0 too long, force movement
        if (thrust == 0) {
            zeroThrustFrames++;
            if (zeroThrustFrames > ZERO_THRUST_MAX_FRAMES) {
                thrust = ZERO_THRUST_OVERRIDE;
                cerr << "Zero thrust override!" << endl;
            }
        } else {
            zeroThrustFrames = 0;
        }

        cout << targetPosition.x << " " << targetPosition.y << " ";
        if (thrust == 650) {
            cout << "BOOST" << endl;
            boostAvailable = false;
            cerr << "BOOST activated!" << endl;
        } else if (thrust == SHIELD_THRUST_VALUE) {
            cout << "SHIELD" << endl;
            shieldCooldown = SHIELD_COOLDOWN;
            cerr << "SHIELD activated!" << endl;
        } else {
            cout << thrust << endl;
        }
    }

    bool firstFrame;
    bool opFirstFrame;
    Point position;
    Point prevPosition;
    Point targetPosition;
    Point opponentPos;
    Point prevOpponentPos;
    int next_checkpoint_dist;
    int next_checkpoint_angle;
    double facingAngleDeg;
    int thrust;
    int currentCpIndex;
    int shieldCooldown;
    int zeroThrustFrames;
    double vx, vy;
    double opVx, opVy;
};


int main() {
    Pod pod;

    int i=0;
    while (++i) {
        int x;
        int y;
        int next_checkpoint_x; // x position of the next check point
        int next_checkpoint_y; // y position of the next check point
        int next_checkpoint_dist; // distance to the next checkpoint
        int next_checkpoint_angle; // angle between your pod orientation and the direction of the next checkpoint
        cin >> x >> y >> next_checkpoint_x >> next_checkpoint_y >> next_checkpoint_dist >> next_checkpoint_angle; cin.ignore();
        int opponent_x;
        int opponent_y;
        cin >> opponent_x >> opponent_y; cin.ignore();

        const Point cp(next_checkpoint_x, next_checkpoint_y);
        if (i == 1) pod.initStartPosition(x, y);
        pod.trackCheckpoint(cp);
        pod.computeVelocity(x, y);
        pod.computeOpponent(opponent_x, opponent_y);
        pod.setPos(x, y);
        pod.setNextCheckpointAngle(next_checkpoint_angle);
        pod.setDistance(next_checkpoint_dist);
        pod.setTargetPosition(cp);
        pod.computeFacingAngle(next_checkpoint_x, next_checkpoint_y);

        pod.findBestMove();
        pod.navigate();

    }
}