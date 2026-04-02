#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

// ===== Constants =====

// Physics
constexpr double FRICTION = 0.85;
constexpr double CP_RADIUS = 595.0;
constexpr double CP_RADIUS_REDUCTION = 5.0;
constexpr double COLLISION_RADIUS = 800.0;
constexpr double MIN_COLLISION_IMPULSE = 120.0;
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double MAX_TURN = 18.0 * DEG_TO_RAD;
constexpr int GOALIE_STALL_THRESHOLD = 4;
constexpr double GOALIE_DETECT_DIST = 2.0 * CP_RADIUS;
constexpr double GOALIE_MAX_SPEED_SQ = 150.0 * 150.0;

// Simulation
constexpr int SIM_DEPTH = 4;
constexpr int ZERO_THRUST_MAX_FRAMES = 4;
constexpr int LOW_THRUST_THRESHOLD = 20;
constexpr int LOW_THRUST_OVERRIDE = 100;
constexpr double TARGET_POINT_DIST = 10000.0;

// Scoring
constexpr double SCORE_CP_PASSED = 50000.0;
constexpr double SCORE_EARLY_ARRIVAL = 5000.0;
constexpr double SCORE_SPEED_WEIGHT_FINAL = 0.025;
constexpr double SCORE_SPEED_TOWARD_NORMAL = 3.0;
constexpr double SCORE_SPEED_TOWARD_FINAL = 15.0;
constexpr double SCORE_DIST_TO_NEXT_WEIGHT = 5.0;

// Boost
constexpr int BOOST_MIN_DIST = 3500;
constexpr int BOOST_MIN_LAP = 1;
constexpr int BOOST_MIN_TURN = 10;
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
constexpr double BLOCKER_PASSIVE_INTERCEPT_WEIGHT = 0.2;
constexpr double INTERCEPT_PROXIMITY_BONUS = 15000.0;
constexpr double INTERCEPT_PROXIMITY_WEIGHT = 1.0;

// Search: step 0
constexpr double ANGLE_OFFSETS_1[] = {
    -60*DEG_TO_RAD, -27*DEG_TO_RAD, -15*DEG_TO_RAD, -7*DEG_TO_RAD, 0,
    7*DEG_TO_RAD, 15*DEG_TO_RAD, 27*DEG_TO_RAD, 60*DEG_TO_RAD, M_PI
};
constexpr int THRUST_OPTIONS_1[] = {10, 42, 75, 100, 0};
constexpr int ANGLE_COUNT_1 = sizeof(ANGLE_OFFSETS_1) / sizeof(ANGLE_OFFSETS_1[0]);
constexpr int THRUST_COUNT_1 = sizeof(THRUST_OPTIONS_1) / sizeof(THRUST_OPTIONS_1[0]);

// Search: steps 1-2
constexpr double ANGLE_OFFSETS_23[] = {
    -45*DEG_TO_RAD, -15*DEG_TO_RAD, 0, 15*DEG_TO_RAD, 45*DEG_TO_RAD, M_PI
};
constexpr int THRUST_OPTIONS_23[] = {10, 42, 75, 100};
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
    angle = fmod(angle, 2 * M_PI);
    if (angle > M_PI) angle -= 2 * M_PI;
    if (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

double findCollisionTime(double ax0, double ay0, double ax1, double ay1,
                         double bx0, double by0, double bx1, double by1,
                         double radius) {
    double rx = ax0 - bx0, ry = ay0 - by0;
    double drx = (ax1 - ax0) - (bx1 - bx0);
    double dry = (ay1 - ay0) - (by1 - by0);
    double a = drx*drx + dry*dry;
    double b = 2*(rx*drx + ry*dry);
    double c = rx*rx + ry*ry - radius*radius;
    if (c <= 0) return 0;
    if (a < 1e-9) return -1;
    double disc = b*b - 4*a*c;
    if (disc < 0) return -1;
    double sqrtD = sqrt(disc);
    double t = (-b - sqrtD) / (2*a);
    if (t >= 0 && t <= 1) return t;
    return -1;
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
    double minEnemyDistSq;

    double distTo(double px, double py) const {
        double dx = px - x, dy = py - y;
        return sqrt(dx*dx + dy*dy);
    }

    double angleTo(double px, double py) const {
        return atan2(py - y, px - x);
    }
};

struct EnemyStepState {
    double x, y, vx, vy;
    double facingAngle;
    int nextCpId;
    bool modified;
};

struct Pod {
    Pod() : shieldCooldown(0), lowThrustFrames(0), overrideFramesLeft(0),
            boostAvailable(true), currentLap(1), totalCpsPassed(0), prevNextCpId(-1) {}

    // Current state (from input)
    double x, y, vx, vy;
    double facingAngle; // radians
    int nextCpId;

    // Persistent state
    int shieldCooldown;
    int lowThrustFrames;
    int overrideFramesLeft;
    bool boostAvailable;
    int currentLap;
    int totalCpsPassed;
    int prevNextCpId;

    // Role
    bool isBlocker = false;
    double interceptUrgency = 0.0;
    double leadEnemyX, leadEnemyY;
    int leadEnemyIdx = -1;
    int leadEnemyNextCpId = 0;

    // Output
    Point targetPosition;
    int thrust;
    double chosenScore;
    int podIndex = 0;
    static int currentTurn;
    vector<int> scoreHistory = vector<int>(500);

    // Other pods for collision prediction
    static constexpr int MAX_OTHERS = 3;
    struct OtherPodState {
        double x, y, vx, vy;
        double facingAngle;
        int nextCpId;
    };
    OtherPodState others[MAX_OTHERS];
    int otherCount;

    struct PodPrediction {
        double x, y, vx, vy;
        double moveVx, moveVy;
        double facingAfterStep;
        int nextCpId;
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
    double teammateAngle;
    int teammateThrust;
    double teammateFacing;

    // ===== Simulation methods =====

    SimState applyRotationAndThrust(SimState s, double targetAngle, int simThrust) const {
        double diff = normalizeAngle(targetAngle - s.facingAngle);
        if (diff > MAX_TURN) diff = MAX_TURN;
        if (diff < -MAX_TURN) diff = -MAX_TURN;
        s.facingAngle += diff;
        double cosAngle = cos(s.facingAngle);
        double sinAngle = sin(s.facingAngle);
        s.vx += simThrust * cosAngle;
        s.vy += simThrust * sinAngle;
        return s;
    }

    SimState simStep(SimState s, double targetAngle, int simThrust) const {
        s = applyRotationAndThrust(s, targetAngle, simThrust);
        s.x += s.vx;
        s.y += s.vy;
        s.vx = (int)(s.vx * FRICTION);
        s.vy = (int)(s.vy * FRICTION);
        s.x = (int)s.x;
        s.y = (int)s.y;
        return s;
    }

    void precomputeOthers() {
        for (int p = 0; p < otherCount; p++) {
            double ox = others[p].x, oy = others[p].y;
            double ovx = others[p].vx, ovy = others[p].vy;
            double facing = others[p].facingAngle;
            int cpId = others[p].nextCpId;

            int startStep = 0;

            if (p == 0 && hasTeammateMove) {
                SimState ts = {ox, oy, ovx, ovy, teammateFacing, 0, 0, false, false, false, 1e18};
                ts = applyRotationAndThrust(ts, teammateAngle, teammateThrust);
                otherPredictions[p][0] = {ox, oy, ovx, ovy, ts.vx, ts.vy, ts.facingAngle, cpId};

                double prevX = ox, prevY = oy;
                ox = (int)(ox + ts.vx);
                oy = (int)(oy + ts.vy);
                ovx = (int)(ts.vx * FRICTION);
                ovy = (int)(ts.vy * FRICTION);
                facing = ts.facingAngle;

                if (segmentIntersectsCircle(prevX, prevY, ox, oy,
                                            checkpoints[cpId].x, checkpoints[cpId].y, CP_RADIUS)) {
                    cpId = (cpId + 1) % checkpointCount;
                }
                startStep = 1;
            }

            for (int i = startStep; i < SIM_DEPTH; i++) {
                Point target = checkpoints[cpId];
                double aimAngle = atan2(target.y - oy, target.x - ox);
                int rollThrust = angleDependentThrust(aimAngle, facing);

                SimState ts = {ox, oy, ovx, ovy, facing, 0, 0, false, false, false, 1e18};
                ts = applyRotationAndThrust(ts, aimAngle, rollThrust);
                otherPredictions[p][i] = {ox, oy, ovx, ovy, ts.vx, ts.vy, ts.facingAngle, cpId};

                double prevX = ox, prevY = oy;
                ox = (int)(ox + ts.vx);
                oy = (int)(oy + ts.vy);
                ovx = (int)(ts.vx * FRICTION);
                ovy = (int)(ts.vy * FRICTION);
                facing = ts.facingAngle;

                if (segmentIntersectsCircle(prevX, prevY, ox, oy,
                                            target.x, target.y, CP_RADIUS)) {
                    cpId = (cpId + 1) % checkpointCount;
                }
            }

            otherPredictions[p][SIM_DEPTH] = {ox, oy, ovx, ovy, 0, 0, facing, cpId};
        }
    }

    bool applyMutualCollision(SimState& s, double& ox, double& oy,
                              double& ovx, double& ovy,
                              double myMass, double otherMass) {
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
        double totalMass = myMass + otherMass;
        double impulse = max(MIN_COLLISION_IMPULSE, -2.0 * myMass * otherMass * dvn / totalMass);
        s.vx += (impulse / myMass) * nx;
        s.vy += (impulse / myMass) * ny;
        ovx -= (impulse / otherMass) * nx;
        ovy -= (impulse / otherMass) * ny;
        double overlap = COLLISION_RADIUS - dist;
        if (overlap > 0) {
            double pushUs = otherMass / totalMass;
            double pushThem = myMass / totalMass;
            s.x += overlap * nx * pushUs;
            s.y += overlap * ny * pushUs;
            ox -= overlap * nx * pushThem;
            oy -= overlap * ny * pushThem;
        }
        return true;
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

    int angleDependentThrust(double aimAngle, double facingAngle) const {
        return max(20, (int)(100 - abs(normalizeAngle(aimAngle - facingAngle)) / DEG_TO_RAD));
    }

    void computeEnemyMoves(int step, EnemyStepState enemies[MAX_OTHERS],
                           double eStartX[], double eStartY[],
                           double eMoveVx[], double eMoveVy[]) {
        for (int p = 0; p < otherCount; p++) {
            if (enemies[p].modified) {
                eStartX[p] = enemies[p].x;
                eStartY[p] = enemies[p].y;
                Point target = checkpoints[enemies[p].nextCpId];
                double eAim = atan2(target.y - enemies[p].y, target.x - enemies[p].x);
                int eThrust = angleDependentThrust(eAim, enemies[p].facingAngle);
                SimState es = {enemies[p].x, enemies[p].y, enemies[p].vx, enemies[p].vy,
                              enemies[p].facingAngle, 0, 0, false, false, false, 1e18};
                es = applyRotationAndThrust(es, eAim, eThrust);
                eMoveVx[p] = es.vx;
                eMoveVy[p] = es.vy;
                enemies[p].facingAngle = es.facingAngle;
            } else {
                const auto& pred = otherPredictions[p][step];
                eStartX[p] = pred.x;
                eStartY[p] = pred.y;
                eMoveVx[p] = pred.moveVx;
                eMoveVy[p] = pred.moveVy;
            }
        }
    }

    void setCollisionFlags(SimState& s, int p, double myMass, int simThrust) {
        if (p == 0) {
            s.teammateCollided = true;
        } else {
            if (myMass > NORMAL_MASS) s.shieldCollided = true;
            if (simThrust == 650 && p == leadEnemyIdx) s.boostCollided = true;
        }
    }

    bool isAnyEnemyNear(const SimState& s, const EnemyStepState enemies[MAX_OTHERS]) const {
        for (int p = 0; p < otherCount; p++) {
            double dx = s.x - enemies[p].x, dy = s.y - enemies[p].y;
            if (dx*dx + dy*dy < COLLISION_RADIUS * COLLISION_RADIUS) return true;
        }
        return false;
    }

    void updateEnemyStates(int step, EnemyStepState enemies[MAX_OTHERS],
                           const double eStartX[], const double eStartY[],
                           const double eFinalX[], const double eFinalY[],
                           const double eFinalVx[], const double eFinalVy[],
                           const bool eCollided[]) {
        for (int p = 0; p < otherCount; p++) {
            if (eCollided[p] || enemies[p].modified) {
                if (!enemies[p].modified && eCollided[p]) {
                    enemies[p].facingAngle = otherPredictions[p][step].facingAfterStep;
                    enemies[p].nextCpId = otherPredictions[p][step].nextCpId;
                }
                enemies[p].x = (int)eFinalX[p];
                enemies[p].y = (int)eFinalY[p];
                enemies[p].vx = (int)(eFinalVx[p] * FRICTION);
                enemies[p].vy = (int)(eFinalVy[p] * FRICTION);
                enemies[p].modified = true;

                Point eTarget = checkpoints[enemies[p].nextCpId];
                if (segmentIntersectsCircle(eStartX[p], eStartY[p],
                                            eFinalX[p], eFinalY[p],
                                            eTarget.x, eTarget.y, CP_RADIUS)) {
                    enemies[p].nextCpId = (enemies[p].nextCpId + 1) % checkpointCount;
                }
            } else {
                const auto& nextPred = otherPredictions[p][step + 1];
                enemies[p].x = nextPred.x;
                enemies[p].y = nextPred.y;
                enemies[p].vx = nextPred.vx;
                enemies[p].vy = nextPred.vy;
                enemies[p].facingAngle = otherPredictions[p][step].facingAfterStep;
                enemies[p].nextCpId = nextPred.nextCpId;
            }
        }
    }

    SimState simCandidate(SimState s, double aimAngle, int simThrust, int step,
                          int& simNextCpId, int& arrivalFrame,
                          EnemyStepState enemies[MAX_OTHERS]) {
        double myMass = getMass(simThrust);
        int actualThrust = resolveThrust(simThrust, s);
        s = applyRotationAndThrust(s, aimAngle, actualThrust);
        double startX = s.x, startY = s.y;

        bool anyModified = enemies[0].modified || enemies[1].modified ||
                           (otherCount > 2 && enemies[2].modified);

        double eStartX[MAX_OTHERS], eStartY[MAX_OTHERS];
        double eMoveVx[MAX_OTHERS], eMoveVy[MAX_OTHERS];
        if (anyModified) {
            computeEnemyMoves(step, enemies, eStartX, eStartY, eMoveVx, eMoveVy);
        } else {
            for (int p = 0; p < otherCount; p++) {
                const auto& pred = otherPredictions[p][step];
                eStartX[p] = pred.x; eStartY[p] = pred.y;
                eMoveVx[p] = pred.moveVx; eMoveVy[p] = pred.moveVy;
            }
        }

        // Find earliest segment collision
        double firstT = 1.1;
        int firstP = -1;
        for (int p = 0; p < otherCount; p++) {
            double t = findCollisionTime(startX, startY, startX + s.vx, startY + s.vy,
                                         eStartX[p], eStartY[p],
                                         eStartX[p] + eMoveVx[p], eStartY[p] + eMoveVy[p],
                                         COLLISION_RADIUS);
            if (t >= 0 && t < firstT) { firstT = t; firstP = p; }
        }

        // Fast path: no collision, no modified enemies
        if (firstP < 0 && !anyModified) {
            s.x = startX + s.vx;
            s.y = startY + s.vy;
            s.vx = (int)(s.vx * FRICTION);
            s.vy = (int)(s.vy * FRICTION);
            s.x = (int)s.x;
            s.y = (int)s.y;

            if (leadEnemyIdx >= 0) {
                const auto& pred = otherPredictions[leadEnemyIdx][step + 1];
                double edx = s.x - pred.x, edy = s.y - pred.y;
                double enemyDistSq = edx*edx + edy*edy;
                if (enemyDistSq < s.minEnemyDistSq) s.minEnemyDistSq = enemyDistSq;
            }

            bool anyNear = false;
            for (int p = 0; p < otherCount; p++) {
                const auto& np = otherPredictions[p][step + 1];
                double dx = s.x - np.x, dy = s.y - np.y;
                if (dx*dx + dy*dy < COLLISION_RADIUS * COLLISION_RADIUS) { anyNear = true; break; }
            }
            double cpRadius = anyNear ? CP_RADIUS - CP_RADIUS_REDUCTION : CP_RADIUS;
            Point simTarget = checkpoints[simNextCpId];
            if (segmentIntersectsCircle(startX, startY, s.x, s.y,
                                         simTarget.x, simTarget.y, cpRadius)) {
                s.cpsPassed++;
                if (arrivalFrame < 0) arrivalFrame = step + 1;
                simNextCpId = (simNextCpId + 1) % checkpointCount;
            }
            return s;
        }

        // Slow path: collision or modified enemies
        double eFinalX[MAX_OTHERS], eFinalY[MAX_OTHERS];
        double eFinalVx[MAX_OTHERS], eFinalVy[MAX_OTHERS];
        bool eCollided[MAX_OTHERS] = {};
        for (int p = 0; p < otherCount; p++) {
            eFinalX[p] = eStartX[p] + eMoveVx[p];
            eFinalY[p] = eStartY[p] + eMoveVy[p];
            eFinalVx[p] = eMoveVx[p];
            eFinalVy[p] = eMoveVy[p];
        }

        if (firstP >= 0) {
            double t = firstT;
            s.x = startX + t * s.vx;
            s.y = startY + t * s.vy;
            double eCollX = eStartX[firstP] + t * eMoveVx[firstP];
            double eCollY = eStartY[firstP] + t * eMoveVy[firstP];
            double postEVx = eMoveVx[firstP], postEVy = eMoveVy[firstP];
            applyMutualCollision(s, eCollX, eCollY, postEVx, postEVy, myMass, NORMAL_MASS);
            double remaining = 1.0 - t;
            s.x += s.vx * remaining;
            s.y += s.vy * remaining;
            eFinalX[firstP] = eCollX + postEVx * remaining;
            eFinalY[firstP] = eCollY + postEVy * remaining;
            eFinalVx[firstP] = postEVx;
            eFinalVy[firstP] = postEVy;
            eCollided[firstP] = true;
            setCollisionFlags(s, firstP, myMass, simThrust);
        } else {
            s.x = startX + s.vx;
            s.y = startY + s.vy;
        }

        for (int p = 0; p < otherCount; p++) {
            if (p == firstP) continue;
            if (applyMutualCollision(s, eFinalX[p], eFinalY[p],
                                      eFinalVx[p], eFinalVy[p], myMass, NORMAL_MASS)) {
                eCollided[p] = true;
                setCollisionFlags(s, p, myMass, simThrust);
            }
        }

        updateEnemyStates(step, enemies, eStartX, eStartY,
                          eFinalX, eFinalY, eFinalVx, eFinalVy, eCollided);

        s.vx = (int)(s.vx * FRICTION);
        s.vy = (int)(s.vy * FRICTION);
        s.x = (int)s.x;
        s.y = (int)s.y;

        if (leadEnemyIdx >= 0) {
            double edx = s.x - enemies[leadEnemyIdx].x;
            double edy = s.y - enemies[leadEnemyIdx].y;
            double enemyDistSq = edx*edx + edy*edy;
            if (enemyDistSq < s.minEnemyDistSq) s.minEnemyDistSq = enemyDistSq;
        }

        bool anyNear = isAnyEnemyNear(s, enemies);
        double cpRadius = anyNear ? CP_RADIUS - CP_RADIUS_REDUCTION : CP_RADIUS;
        Point simTarget = checkpoints[simNextCpId];
        if (segmentIntersectsCircle(startX, startY, s.x, s.y,
                                     simTarget.x, simTarget.y, cpRadius)) {
            s.cpsPassed++;
            if (arrivalFrame < 0) arrivalFrame = step + 1;
            simNextCpId = (simNextCpId + 1) % checkpointCount;
        }
        return s;
    }

    SimState rollout(SimState s, int fromStep, int& simNextCpId, int& arrivalFrame,
                     EnemyStepState enemies[MAX_OTHERS]) {
        for (int step = fromStep; step < SIM_DEPTH; step++) {
            Point target = checkpoints[simNextCpId];
            double aimAngle = s.angleTo(target.x, target.y);
            int rollThrust = angleDependentThrust(aimAngle, s.facingAngle);
            s = simCandidate(s, aimAngle, rollThrust, step, simNextCpId, arrivalFrame,
                             enemies);
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
        double dx = tx - simPod.x;
        double dy = ty - simPod.y;
        double distSq = dx*dx + dy*dy;
        double dist = sqrt(distSq);
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

        if (isBlocker) {
            if (interceptUrgency > 0.0) {
                double minEnemyDist = sqrt(simPod.minEnemyDistSq);
                double interceptScore = INTERCEPT_PROXIMITY_BONUS - minEnemyDist * INTERCEPT_PROXIMITY_WEIGHT;
                return max(raceScore, interceptScore) + bonus;
            }
            Point enemyCp = checkpoints[leadEnemyNextCpId];
            double interceptScore = approachScore(simPod, enemyCp.x, enemyCp.y, SCORE_SPEED_TOWARD_FINAL);
            return raceScore + interceptScore * BLOCKER_PASSIVE_INTERCEPT_WEIGHT + bonus;
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

    static bool anyEnemyModified(const EnemyStepState enemies[], int count) {
        for (int p = 0; p < count; p++) {
            if (enemies[p].modified) return true;
        }
        return false;
    }

    static void initUnmodifiedEnemies(EnemyStepState enemies[], int count) {
        for (int p = 0; p < count; p++) enemies[p].modified = false;
    }

    double searchFromStep1(const SimState& afterStep0, int simCpId0, int arrival0,
                           const Point& currentCp, const Point& nextCp,
                           EnemyStepState enemies0[MAX_OTHERS]) {
        Point target1 = checkpoints[simCpId0];
        double dirFromStep0 = afterStep0.angleTo(target1.x, target1.y);
        double bestScore = -1e18;
        bool e0Modified = anyEnemyModified(enemies0, otherCount);

        for (int ai2 = 0; ai2 < ANGLE_COUNT_23; ai2++) {
            for (int ti2 = 0; ti2 < THRUST_COUNT_23; ti2++) {
                int simCpId1 = simCpId0;
                int arrival1 = arrival0;
                EnemyStepState enemies1[MAX_OTHERS];
                if (e0Modified) {
                    for (int p = 0; p < otherCount; p++) enemies1[p] = enemies0[p];
                } else {
                    initUnmodifiedEnemies(enemies1, otherCount);
                }
                SimState afterStep1 = simCandidate(afterStep0, dirFromStep0 + ANGLE_OFFSETS_23[ai2],
                                                    THRUST_OPTIONS_23[ti2], 1, simCpId1, arrival1,
                                                    enemies1);

                Point target2 = checkpoints[simCpId1];
                double dirFromStep1 = afterStep1.angleTo(target2.x, target2.y);
                bool e1Modified = anyEnemyModified(enemies1, otherCount);

                for (int ai3 = 0; ai3 < ANGLE_COUNT_23; ai3++) {
                    for (int ti3 = 0; ti3 < THRUST_COUNT_23; ti3++) {
                        int simCpId = simCpId1;
                        int arrivalFrame = arrival1;
                        EnemyStepState enemies2[MAX_OTHERS];
                        if (e1Modified) {
                            for (int p = 0; p < otherCount; p++) enemies2[p] = enemies1[p];
                        } else {
                            initUnmodifiedEnemies(enemies2, otherCount);
                        }
                        SimState simPod = simCandidate(afterStep1, dirFromStep1 + ANGLE_OFFSETS_23[ai3],
                                                       THRUST_OPTIONS_23[ti3], 2, simCpId, arrivalFrame,
                                                       enemies2);

                        simPod = rollout(simPod, 3, simCpId, arrivalFrame, enemies2);

                        double score = evaluateScore(simPod, arrivalFrame, currentCp, nextCp);
                        if (score > bestScore) bestScore = score;
                    }
                }
            }
        }
        return bestScore;
    }

    void selectBestAction(bool canBoost, bool canShield,
                          double bestScoreNormal, int bestThrustNormal, double bestAngleNormal,
                          double bestScoreBoost, double bestAngleBoost,
                          double bestScoreShield, double bestAngleShield) {
        if (canBoost && bestScoreBoost > bestScoreNormal + BOOST_SCORE_THRESHOLD) {
            thrust = 650;
            chosenScore = bestScoreBoost;
            targetPosition = Point(
                (int)(x + TARGET_POINT_DIST * cos(bestAngleBoost)),
                (int)(y + TARGET_POINT_DIST * sin(bestAngleBoost))
            );
            return;
        }

        // if (canShield) {
        //     cerr << "Shield scores: shield=" << bestScoreShield
        //          << " normal=" << bestScoreNormal
        //          << " diff=" << bestScoreShield - bestScoreNormal << endl;
        // }
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

    void findBestMove() {
        precomputeOthers();

        SimState initial = {
            x, y, vx, vy, facingAngle, 0, shieldCooldown, false, false, false, 1e18
        };

        Point currentCp = getNextCp();
        Point nextCp = getAfterNextCp();

        double bestScoreNormal = -1e18, bestScoreBoost = -1e18, bestScoreShield = -1e18;
        int bestThrustNormal = 100;
        double bestAngleNormal = 0, bestAngleBoost = 0, bestAngleShield = 0;

        bool willReachCp = segmentIntersectsCircle(x, y, x + vx, y + vy,
                                                    currentCp.x, currentCp.y, CP_RADIUS);
        double minEnemyDistSqVal = closestEnemyDistSq();
        bool enemyFar = minEnemyDistSqVal > 4 * COLLISION_RADIUS * COLLISION_RADIUS;
        Point aimCp = (willReachCp && enemyFar) ? nextCp : currentCp;

        double dirToCp;
        if (isBlocker && interceptUrgency > 0.0) {
            Point enemyCp = checkpoints[leadEnemyNextCpId];
            dirToCp = atan2(enemyCp.y - y, enemyCp.x - x);
        } else {
            dirToCp = atan2(aimCp.y - y, aimCp.x - x);
        }
        double distToCpSq = (currentCp.x - x) * (double)(currentCp.x - x) +
                             (currentCp.y - y) * (double)(currentCp.y - y);

        bool canBoost = boostAvailable && currentLap >= BOOST_MIN_LAP
                        && currentTurn >= BOOST_MIN_TURN
                        && distToCpSq > (double)BOOST_MIN_DIST * BOOST_MIN_DIST;
        double maxRelSpeedSq = closestEnemyRelSpeedSq();
        bool canShield = shieldCooldown == 0
            && minEnemyDistSqVal < SHIELD_CHECK_DIST * SHIELD_CHECK_DIST
            && maxRelSpeedSq > SHIELD_MIN_REL_SPEED_SQ;

        // if (minEnemyDistSqVal < SHIELD_CHECK_DIST * SHIELD_CHECK_DIST) {
        //     cerr << "Collision: relV²=" << maxRelSpeedSq
        //          << " dist=" << sqrt(minEnemyDistSqVal)
        //          << " shield=" << (canShield ? "available" : "no") << endl;
        // }

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
                EnemyStepState enemies0[MAX_OTHERS];
                initUnmodifiedEnemies(enemies0, otherCount);
                SimState afterStep0 = simCandidate(initial, moveAngle1, step0Thrusts[ti1],
                                                    0, simCpId0, arrival0, enemies0);

                double score = searchFromStep1(afterStep0, simCpId0, arrival0,
                                               currentCp, nextCp, enemies0);

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

        selectBestAction(canBoost, canShield, bestScoreNormal, bestThrustNormal, bestAngleNormal,
                         bestScoreBoost, bestAngleBoost, bestScoreShield, bestAngleShield);
    }

    void navigate() {
        if (shieldCooldown > 0) shieldCooldown--;

        if (overrideFramesLeft > 0) {
            thrust = LOW_THRUST_OVERRIDE;
            overrideFramesLeft--;
            // cerr << "Low thrust override! (" << overrideFramesLeft << " left)" << endl;
        } else if (thrust < LOW_THRUST_THRESHOLD) {
            lowThrustFrames++;
            if (lowThrustFrames > ZERO_THRUST_MAX_FRAMES) {
                thrust = LOW_THRUST_OVERRIDE;
                overrideFramesLeft = ZERO_THRUST_MAX_FRAMES;
                lowThrustFrames = 0;
                // cerr << "Low thrust override triggered!" << endl;
            }
        } else {
            lowThrustFrames = 0;
        }

        if (isBlocker) {
            // cerr << "P" << podIndex << " blocker urg=" << interceptUrgency
            //      << " eCpId=" << leadEnemyNextCpId
            //      << " thrust=" << thrust << endl;
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

        Pod& blocker = a.isBlocker ? a : b;
        Point blockerCp = blocker.getNextCp();
        int leadOp = findLeadOpponent(px, py, pcpid, enemyCps);
        Point enemyCp = checkpoints[pcpid[leadOp]];
        bool pathCrossesEnemy = segmentIntersectsCircle(
            blocker.x, blocker.y, blockerCp.x, blockerCp.y,
            enemyCp.x, enemyCp.y, INTERCEPT_PATH_RADIUS);
        if (pathCrossesEnemy && enemiesAhead == 0) enemiesAhead = 1;

        double urgency = enemiesAhead * INTERCEPT_URGENCY_PER_ENEMY;
        a.interceptUrgency = a.isBlocker ? urgency : 0.0;
        b.interceptUrgency = b.isBlocker ? urgency : 0.0;
    }

    static int findLeadOpponent(int px[], int py[], int pcpid[], int enemyCps[]) {
        if (enemyCps[1] > enemyCps[0]) return 3;
        if (enemyCps[0] > enemyCps[1]) return 2;
        double d2 = (checkpoints[pcpid[2]].x - px[2]) * (double)(checkpoints[pcpid[2]].x - px[2])
                   + (checkpoints[pcpid[2]].y - py[2]) * (double)(checkpoints[pcpid[2]].y - py[2]);
        double d3 = (checkpoints[pcpid[3]].x - px[3]) * (double)(checkpoints[pcpid[3]].x - px[3])
                   + (checkpoints[pcpid[3]].y - py[3]) * (double)(checkpoints[pcpid[3]].y - py[3]);
        return (d3 < d2) ? 3 : 2;
    }

    static int enemyStallCounter[2];

    static int detectGoalie(Pod& a, Pod& b, int px[], int py[], int pvx[], int pvy[]) {
        Pod& racer = a.isBlocker ? b : a;
        Point racerCp = racer.getNextCp();

        int goalieEnemy = -1;
        for (int e = 0; e < 2; e++) {
            double dx = px[2 + e] - racerCp.x;
            double dy = py[2 + e] - racerCp.y;
            double distSq = dx * dx + dy * dy;
            double speedSq = pvx[2+e]*(double)pvx[2+e] + pvy[2+e]*(double)pvy[2+e];
            if (distSq < GOALIE_DETECT_DIST * GOALIE_DETECT_DIST && speedSq < GOALIE_MAX_SPEED_SQ) {
                enemyStallCounter[e]++;
            } else {
                enemyStallCounter[e] = 0;
            }
            if (enemyStallCounter[e] >= GOALIE_STALL_THRESHOLD) {
                goalieEnemy = e;
            }
        }

        if (goalieEnemy >= 0) {
            cerr << "GOALIE e" << goalieEnemy << " on CP" << racer.nextCpId
                 << " stall=" << enemyStallCounter[goalieEnemy] << endl;
        }
        return goalieEnemy;
    }

    static void setupLeadEnemy(Pod pods[], int leadOp, int px[], int py[], int pcpid[]) {
        for (int i = 0; i < 2; i++) {
            pods[i].leadEnemyX = px[leadOp];
            pods[i].leadEnemyY = py[leadOp];
            pods[i].leadEnemyNextCpId = pcpid[leadOp];
            pods[i].leadEnemyIdx = (leadOp == 2) ? 1 : 2;
        }
    }

    static void setupOtherPods(Pod pods[], int px[], int py[], int pvx[], int pvy[],
                                int pangle[], int pcpid[]) {
        for (int i = 0; i < 2; i++) {
            OtherPodState others[3];
            int teammate = 1 - i;
            others[0] = {(double)px[teammate], (double)py[teammate], (double)pvx[teammate], (double)pvy[teammate], pangle[teammate] * DEG_TO_RAD, pcpid[teammate]};
            others[1] = {(double)px[2], (double)py[2], (double)pvx[2], (double)pvy[2], pangle[2] * DEG_TO_RAD, pcpid[2]};
            others[2] = {(double)px[3], (double)py[3], (double)pvx[3], (double)pvy[3], pangle[3] * DEG_TO_RAD, pcpid[3]};
            pods[i].setOthers(others, 3);
        }
    }

    static void applyGoalieOverride(Pod pods[], int px[], int py[], int pvx[], int pvy[],
                                     int enemyCpsPassed[]) {
        int ourBestCps = max(pods[0].totalCpsPassed, pods[1].totalCpsPassed);
        int enemyBestCps = max(enemyCpsPassed[0], enemyCpsPassed[1]);
        if (ourBestCps < enemyBestCps) return;

        Pod& racer = pods[0].isBlocker ? pods[1] : pods[0];
        Pod& blocker = pods[0].isBlocker ? pods[0] : pods[1];
        int goalieEnemy = detectGoalie(pods[0], pods[1], px, py, pvx, pvy);
        if (goalieEnemy < 0) return;

        int idx = 2 + goalieEnemy;
        blocker.interceptUrgency = 1.0;
        blocker.leadEnemyX = px[idx];
        blocker.leadEnemyY = py[idx];
        blocker.leadEnemyNextCpId = racer.nextCpId;
        blocker.leadEnemyIdx = (idx == 2) ? 1 : 2;
    }

    static void setupTurn(Pod pods[], int px[], int py[], int pvx[], int pvy[],
                          int pangle[], int pcpid[], int enemyCpsPassed[]) {
        assignRoles(pods[0], pods[1]);
        int leadOp = findLeadOpponent(px, py, pcpid, enemyCpsPassed);
        calcInterceptUrgency(pods[0], pods[1], enemyCpsPassed, px, py, pcpid);
        setupLeadEnemy(pods, leadOp, px, py, pcpid);
        setupOtherPods(pods, px, py, pvx, pvy, pangle, pcpid);
        // applyGoalieOverride(pods, px, py, pvx, pvy, enemyCpsPassed);
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

int Pod::currentTurn = 0;
int Pod::enemyStallCounter[2] = {0, 0};

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

        Pod::setupTurn(pods, px, py, pvx, pvy, pangle, pcpid, enemyCpsPassed);

        Pod::currentTurn = turn;

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
