#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>
#include <chrono>

using namespace std;

// ===== Constants =====
// Physics
constexpr double FRICTION = 0.85;
constexpr double CP_RADIUS = 585.0;
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
constexpr double SCORE_CP_PASSED = 25000.0;

constexpr double SCORE_EARLY_ARRIVAL = 5000.0;
constexpr double SCORE_SPEED_WEIGHT_FINAL = 0.025;
constexpr double SCORE_SPEED_TOWARD_NORMAL = 3.0;
constexpr double SCORE_SPEED_TOWARD_FINAL = 15.0;
constexpr double SCORE_DIST_TO_NEXT_WEIGHT = 5.0;

// Boost
constexpr int BOOST_MIN_DIST = 4500;
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
// need bonus for hitting leader
constexpr double TEAMMATE_COLLISION_PENALTY = 25000.0;
constexpr double TEAMMATE_COLLISION_PENALTY_PER_LAP = 15000.0;
constexpr double INTERCEPT_URGENCY_PER_ENEMY = 0.51;
constexpr double INTERCEPT_DIST_THRESHOLD = 2000.0;
constexpr double INTERCEPT_PATH_RADIUS = 2000.0;
constexpr double BLOCKER_PASSIVE_INTERCEPT_WEIGHT = 0.25;
constexpr double INTERCEPT_PROXIMITY_BONUS = 15000.0;
constexpr double INTERCEPT_PROXIMITY_WEIGHT = 1.0;

// Search: step 0
constexpr double ANGLE_OFFSETS_1[] = {
    -27*DEG_TO_RAD, -15*DEG_TO_RAD, -7*DEG_TO_RAD, 0,
    7*DEG_TO_RAD, 15*DEG_TO_RAD, 27*DEG_TO_RAD, M_PI
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

enum OtherPodIndex {
    TEAMMATE = 0,
    ENEMY_FIRST = 1,
    ENEMY_SECOND = 2
};

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

struct SimpleStateCoords {
    double x, y;
};
struct SimpleStateSpeeds {
    double vx, vy;
};

struct PodBaseState : SimpleStateCoords, SimpleStateSpeeds {
    bool modified = false;
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

bool segmentIntersectsCircle(double x1, double y1, double x2, double y2,
                              double circleX, double circleY, double radius) {
    double dx = x2 - x1, dy = y2 - y1;
    double fx = x1 - circleX, fy = y1 - circleY;
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

struct PodBase : SimpleStateCoords, SimpleStateSpeeds {
    static int boostHoldTurns;
    static int currentTurn;

    double facingAngle;
    int nextCpId;
    bool modified = false;

    void updateX(double _x) { x = _x; }
    void updateY(double _y) { y = _y; }
    void updateVx(double _vx) { vx = _vx; }
    void updateVy(double _vy) { vy = _vy; }
    void updateFacingAngle(double _fa) { facingAngle = _fa; }

    double getX() const { return x; }
    double getY() const { return y; }
    double getVx() const { return vx; }
    double getVy() const { return vy; }
    double getFacingAngle() const { return facingAngle; }

    double distTo(double tx, double ty) const { double dx = tx - x, dy = ty - y; return sqrt(dx*dx + dy*dy); }
    double sqDistTo(double tx, double ty) const { double dx = tx - x, dy = ty - y; return dx*dx + dy*dy; }
    double angleTo(double tx, double ty) const { return atan2(ty - y, tx - x); }
};

struct SimState : PodBase {
    double prevX, prevY;
    int cpsPassed;
    int shieldTurnsLeft;
    bool shieldCollided;
    bool boostCollided;
    bool teammateCollided;
    double minEnemyDistSq;

    void updateX(double _x) { prevX = x; x = _x; }
    void updateY(double _y) { prevY = y; y = _y; }
    void setAfterCollisionX(double _x) { x = _x; }
    void setAfterCollisionY(double _y) { y = _y; }
    double getPrevX() const { return prevX; }
    double getPrevY() const { return prevY; }

    SimState() : PodBase(), prevX(0), prevY(0), cpsPassed(0), shieldTurnsLeft(0),
                 shieldCollided(false), boostCollided(false), teammateCollided(false),
                 minEnemyDistSq(1e18) {}

    SimState(double x, double y, double vx, double vy, double facingAngle, int cpsPassed, int shieldCooldown,
             bool shieldCollided, bool boostCollided, bool teammateCollided, double minEnemyDistSq, int nextCpId=0) :
    prevX(x), prevY(y), cpsPassed(cpsPassed), shieldTurnsLeft(shieldCooldown),
    shieldCollided(shieldCollided), boostCollided(boostCollided),
    teammateCollided(teammateCollided), minEnemyDistSq(minEnemyDistSq)
    {
        this->x = x; this->y = y; this->vx = vx; this->vy = vy;
        this->facingAngle = facingAngle;
        this->nextCpId = nextCpId; this->modified = false;
    }
};

struct Pod : PodBase {
    Pod() : shieldCooldown(0), lowThrustFrames(0), overrideFramesLeft(0),
            boostAvailable(true), currentLap(1), totalCpsPassed(0), prevNextCpId(-1) {}

    // Current state (from input)
    double x, y, vx, vy, prevX, prevY, prevVx, prevVy;
    double facingAngle, prevFacingAngle; // radians
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
    vector<int> scoreHistory = vector<int>(500);

    // Other pods for collision prediction
    static constexpr int MAX_OTHERS = 3;
    PodBase others[MAX_OTHERS];
    int otherCount;

    struct PodPrediction {
        double x, y, vx, vy;
        double moveVx, moveVy;
        double facingAfterStep;
        int nextCpId;
    };
    PodPrediction otherPredictions[MAX_OTHERS][SIM_DEPTH + 1];

    void setState(int podX, int podY, int podVx, int podVy, int angleDeg, int cpId) {
        x = podX; y = podY;
        vx = podVx; vy = podVy;
        facingAngle = angleDeg * DEG_TO_RAD;
        nextCpId = cpId;
        targetPosition = checkpoints[cpId];

        if (prevNextCpId >= 0 && cpId != prevNextCpId) {
            totalCpsPassed++;
            currentLap = 1 + totalCpsPassed / checkpointCount;
        }
        prevNextCpId = cpId;
    }

    void setOthers(const PodBase* o, int count) {
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

    void applyRotationAndThrust(PodBase& podState, double targetAngle, int simThrust) const {
        double diff = normalizeAngle(targetAngle - podState.getFacingAngle());
        if (diff > MAX_TURN) diff = MAX_TURN;
        if (diff < -MAX_TURN) diff = -MAX_TURN;
        podState.updateFacingAngle(podState.getFacingAngle() + diff);
        double cosAngle = cos(podState.getFacingAngle());
        double sinAngle = sin(podState.getFacingAngle());
        podState.updateVx(podState.getVx() + simThrust * cosAngle);
        podState.updateVy(podState.getVy() + simThrust * sinAngle);
    }
    

    static double findCollisionTime(const SimpleStateCoords& startA, const SimpleStateCoords& endA,
                                    const SimpleStateCoords& startB, const SimpleStateCoords& endB,
                                    double radius) {
        double rx = startA.x - startB.x, ry = startA.y - startB.y;
        double drx = (endA.x - startA.x) - (endB.x - startB.x);
        double dry = (endA.y - startA.y) - (endB.y - startB.y);
        double quadA = drx*drx + dry*dry;
        double quadB = 2*(rx*drx + ry*dry);
        double quadC = rx*rx + ry*ry - radius*radius;
        if (quadC <= 0) return 0;
        if (quadA < 1e-9) return -1;
        double discriminant = quadB*quadB - 4*quadA*quadC;
        if (discriminant < 0) return -1;
        double sqrtDiscriminant = sqrt(discriminant);
        double collisionTime = (-quadB - sqrtDiscriminant) / (2*quadA);
        if (collisionTime >= 0 && collisionTime <= 1) return collisionTime;
        return -1;
    }


    int getNextCpIdIfIntersects(const SimpleStateCoords& from, const SimpleStateCoords& to,
                              const Point& circle, double radius, int nextCpId) {
        if (segmentIntersectsCircle(from.x, from.y, to.x, to.y, circle.x, circle.y, CP_RADIUS)) {
            return (nextCpId + 1) % checkpointCount;
        } else {
            return nextCpId;
        }
    }

    void predictPodStep(PodBase& state, const Point& target, int podIdx, int step) {

        double aimAngle = atan2(target.y - state.getY(), target.x - state.getX());
        int rollThrust = angleDependentThrust(aimAngle, state.getFacingAngle());
        double facingAngle = state.getFacingAngle();
        if (podIdx == TEAMMATE && hasTeammateMove && step == 0) {
            aimAngle = teammateAngle;
            rollThrust = teammateThrust;
            facingAngle = teammateFacing;
        }

        PodBase tempState = {state.getX(), state.getY(), state.getVx(), state.getVy(), facingAngle};
        applyRotationAndThrust(tempState, aimAngle, rollThrust);
        otherPredictions[podIdx][step] = {state.getX(), state.getY(), state.getVx(), state.getVy(),
                                          tempState.vx, tempState.vy, tempState.facingAngle, state.nextCpId};

        SimpleStateCoords positionBeforeMove = {state.x, state.y};
        state.updateX((int)(state.getX() + tempState.vx));
        state.updateY((int)(state.getY() + tempState.vy));
        state.updateVx((int)(tempState.vx * FRICTION));
        state.updateVy((int)(tempState.vy * FRICTION));
        state.updateFacingAngle(tempState.facingAngle);

        state.nextCpId = getNextCpIdIfIntersects(positionBeforeMove, state, target, CP_RADIUS, state.nextCpId);
    }

    void precomputeOthers() {
        for (int pId = 0; pId < otherCount; pId++) {
            PodBase state = others[pId];
            int startStep = 0;

            for (int i = 0; i < SIM_DEPTH; i++) {
                predictPodStep(state, checkpoints[state.nextCpId], pId, i);
            }

            otherPredictions[pId][SIM_DEPTH] = {state.getX(), state.getY(), state.getVx(), state.getVy(),
                                               0, 0, state.getFacingAngle(), state.nextCpId};
        }
    }

    bool applyMutualCollision(SimState& simState, PodBaseState& enemy,
                              double myMass, double otherMass) {
        double dx = simState.getX() - enemy.x;
        double dy = simState.getY() - enemy.y;
        double distSq = dx*dx + dy*dy;
        if (distSq >= COLLISION_RADIUS * COLLISION_RADIUS || distSq < 1e-9) return false;
        double dist = sqrt(distSq);
        double nx = dx / dist;
        double ny = dy / dist;
        double dvx = simState.getVx() - enemy.vx;
        double dvy = simState.getVy() - enemy.vy;
        double dvn = dvx * nx + dvy * ny;
        if (dvn > 0) return false;
        double totalMass = myMass + otherMass;
        double impulse = max(MIN_COLLISION_IMPULSE, -2.0 * myMass * otherMass * dvn / totalMass);
        simState.updateVx(simState.getVx() + (impulse / myMass) * nx);
        simState.updateVy(simState.getVy() + (impulse / myMass) * ny);
        enemy.vx -= (impulse / otherMass) * nx;
        enemy.vy -= (impulse / otherMass) * ny;
        double overlap = COLLISION_RADIUS - dist;
        if (overlap > 0) {
            double pushUs = otherMass / totalMass;
            double pushThem = myMass / totalMass;
            simState.setAfterCollisionX(simState.getX() + overlap * nx * pushUs);
            simState.setAfterCollisionY(simState.getY() + overlap * ny * pushUs);
            enemy.x -= overlap * nx * pushThem;
            enemy.y -= overlap * ny * pushThem;
        }
        return true;
    }

    int resolveThrust(int simThrust, SimState& simState) const {
        if (simThrust == SHIELD_THRUST_VALUE) {
            simState.shieldTurnsLeft = SHIELD_COOLDOWN;
            return 0;
        }
        if (simState.shieldTurnsLeft > 0) {
            simState.shieldTurnsLeft--;
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

    void computeEnemyMoves(int step, PodBase others[MAX_OTHERS],
                           PodBaseState othersAfterMove[]) {
        for (int podId = 0; podId < otherCount; ++podId) {
            if (others[podId].modified) {
                othersAfterMove[podId].x = others[podId].getX();
                othersAfterMove[podId].y = others[podId].getY();
                Point target = checkpoints[others[podId].nextCpId];
                double eAim = atan2(target.y - others[podId].getY(), target.x - others[podId].getX());
                int eThrust = angleDependentThrust(eAim, others[podId].getFacingAngle());
                PodBase enemySimState = {others[podId].getX(), others[podId].getY(), others[podId].getVx(),
                                        others[podId].getVy(), others[podId].getFacingAngle()};
                applyRotationAndThrust(enemySimState, eAim, eThrust);
                othersAfterMove[podId].vx = enemySimState.vx;
                othersAfterMove[podId].vy = enemySimState.vy;
                others[podId].updateFacingAngle(enemySimState.facingAngle);
            } else {
                const auto& pred = otherPredictions[podId][step];
                othersAfterMove[podId].x = pred.x;
                othersAfterMove[podId].y = pred.y;
                othersAfterMove[podId].vx = pred.moveVx;
                othersAfterMove[podId].vy = pred.moveVy;
            }
        }
    }

    void setCollisionFlags(SimState& simState, int p, double myMass, int simThrust) {
        if (p == TEAMMATE) {
            simState.teammateCollided = true;
        } else {
            enemyCollisionDetected = true;
            if (myMass > NORMAL_MASS) simState.shieldCollided = true;
            if (simThrust == 650 && p == leadEnemyIdx) simState.boostCollided = true;
        }
    }

    bool isAnyEnemyNear(const SimState& simState, const PodBase others[MAX_OTHERS]) const {
        for (int podId = 0; podId < otherCount; ++podId) {
            double dx = simState.getX() - others[podId].getX(), dy = simState.getY() - others[podId].getY();
            if (dx*dx + dy*dy < COLLISION_RADIUS * COLLISION_RADIUS) return true;
        }
        return false;
    }

    void updateEnemyStates(int step, PodBase others[MAX_OTHERS],
                           const PodBaseState otherAfter[MAX_OTHERS]) {

        for (int podId = 0; podId < otherCount; podId++) {
            if (otherAfter[podId].modified || others[podId].modified) {
                if (!others[podId].modified && otherAfter[podId].modified) {
                    others[podId].updateFacingAngle(otherPredictions[podId][step].facingAfterStep);
                    others[podId].nextCpId = otherPredictions[podId][step].nextCpId;
                }
                SimpleStateCoords enemyPositionBeforeMove = {others[podId].x, others[podId].y};
                others[podId].updateX((int)otherAfter[podId].x);
                others[podId].updateY((int)otherAfter[podId].y);
                others[podId].updateVx((int)(otherAfter[podId].vx * FRICTION));
                others[podId].updateVy((int)(otherAfter[podId].vy * FRICTION));
                others[podId].modified = true;

                Point enemyCheckpoint = checkpoints[others[podId].nextCpId];
                others[podId].nextCpId = getNextCpIdIfIntersects(enemyPositionBeforeMove, otherAfter[podId],
                                                                 enemyCheckpoint, CP_RADIUS, others[podId].nextCpId);
                
            } else {
                const auto& nextPred = otherPredictions[podId][step + 1];
                others[podId].updateX( nextPred.x);
                others[podId].updateY( nextPred.y);
                others[podId].updateVx( nextPred.vx);
                others[podId].updateVy( nextPred.vy);
                others[podId].updateFacingAngle(otherPredictions[podId][step].facingAfterStep);
                others[podId].nextCpId = nextPred.nextCpId;
            }
        }
    }


    void fastPath(SimState& simState, int step, int& simNextCpId, int& arrivalFrame) {
        fastPathCount++;
        simState.updateX((int)(simState.getX() + simState.getVx()));
        simState.updateY((int)(simState.getY() + simState.getVy()));
        simState.updateVx((int)(simState.getVx() * FRICTION));
        simState.updateVy((int)(simState.getVy() * FRICTION));

        if (leadEnemyIdx >= 0) {
            const auto& pred = otherPredictions[leadEnemyIdx][step + 1];
            double edx = simState.getX() - pred.x,
                   edy = simState.getY() - pred.y;
            double enemyDistSq = edx*edx + edy*edy;
            if (enemyDistSq < simState.minEnemyDistSq) simState.minEnemyDistSq = enemyDistSq;
        }

        bool anyNear = false;
        for (int podId = 0; podId < otherCount; podId++) {
            const auto& np = otherPredictions[podId][step + 1];
            double dx = simState.getX() - np.x, dy = simState.getY() - np.y;
            if (dx*dx + dy*dy < COLLISION_RADIUS * COLLISION_RADIUS) { anyNear = true; break; }
        }
        double cpRadius = anyNear ? CP_RADIUS - CP_RADIUS_REDUCTION : CP_RADIUS;
        Point simTarget = checkpoints[simNextCpId];
        if (segmentIntersectsCircle(simState.getPrevX(), simState.getPrevY(), simState.getX(), simState.getY(),
                                    simTarget.x, simTarget.y, cpRadius)) {
            simState.cpsPassed++;
            if (arrivalFrame < 0) arrivalFrame = step + 1;
            simNextCpId = (simNextCpId + 1) % checkpointCount;
        }
    }

    void slowPath(SimState& simState, int simThrust, int step, int& simNextCpId, int& arrivalFrame,
                  int earliestCollisionPod, double earliestCollisionTime,
                  PodBaseState othersAfterMove[MAX_OTHERS], PodBase others[MAX_OTHERS]) {
        slowPathCount++;

        PodBaseState othersFinal[MAX_OTHERS];

        for (int podId = 0; podId < otherCount; ++podId) {
            othersFinal[podId].x = othersAfterMove[podId].x + othersAfterMove[podId].vx;
            othersFinal[podId].y = othersAfterMove[podId].y + othersAfterMove[podId].vy;
            othersFinal[podId].vx = othersAfterMove[podId].vx;
            othersFinal[podId].vy = othersAfterMove[podId].vy;
        }

        if (earliestCollisionPod >= 0) {
            double t = earliestCollisionTime;

            simState.updateX(simState.getX() + t * simState.getVx());
            simState.updateY(simState.getY() + t * simState.getVy());

            PodBaseState enemyAtCollision;
            enemyAtCollision.x = othersAfterMove[earliestCollisionPod].x + t * othersAfterMove[earliestCollisionPod].vx;
            enemyAtCollision.y = othersAfterMove[earliestCollisionPod].y + t * othersAfterMove[earliestCollisionPod].vy;
            enemyAtCollision.vx = othersFinal[earliestCollisionPod].vx;
            enemyAtCollision.vy = othersFinal[earliestCollisionPod].vy;

            applyMutualCollision(simState, enemyAtCollision, getMass(simThrust), NORMAL_MASS);
            if (step == 0 && earliestCollisionPod >= ENEMY_FIRST) {
                cerr << "HIT p=" << earliestCollisionPod << " m=" << getMass(simThrust)
                     << " our=(" << (int)simState.getVx() << "," << (int)simState.getVy() << ")"
                     << " enemy=(" << (int)enemyAtCollision.vx << "," << (int)enemyAtCollision.vy << ")" << endl;
            }
            double remaining = 1.0 - t;
            simState.setAfterCollisionX((int)(simState.getX() + simState.getVx() * remaining));
            simState.setAfterCollisionY((int)(simState.getY() + simState.getVy() * remaining));
            othersFinal[earliestCollisionPod].x = enemyAtCollision.x + enemyAtCollision.vx * remaining;
            othersFinal[earliestCollisionPod].y = enemyAtCollision.y + enemyAtCollision.vy * remaining;
            othersFinal[earliestCollisionPod].vx = enemyAtCollision.vx;
            othersFinal[earliestCollisionPod].vy = enemyAtCollision.vy;
            othersFinal[earliestCollisionPod].modified = true;

            setCollisionFlags(simState, earliestCollisionPod, getMass(simThrust), simThrust);
        } else {
            simState.updateX((int)(simState.getX() + simState.getVx()));
            simState.updateY((int)(simState.getY() + simState.getVy()));
        }

        for (int podId = 0; podId < otherCount; podId++) {
            if (podId == earliestCollisionPod) continue;
            if (applyMutualCollision(simState, othersFinal[podId], getMass(simThrust), NORMAL_MASS)) {
                othersFinal[podId].modified = true;
                setCollisionFlags(simState, podId, getMass(simThrust), simThrust);
            }
        }

        updateEnemyStates(step, others, othersFinal);

        simState.updateVx((int)(simState.getVx() * FRICTION));
        simState.updateVy((int)(simState.getVy() * FRICTION));

        if (leadEnemyIdx >= 0) {
            simState.minEnemyDistSq = min(simState.sqDistTo(others[leadEnemyIdx].getX(), others[leadEnemyIdx].getY()), simState.minEnemyDistSq);
        }

        Point simTarget = checkpoints[simNextCpId];
        if (segmentIntersectsCircle(simState.getPrevX(), simState.getPrevY(), simState.getX(), simState.getY(),
                                     simTarget.x, simTarget.y,
                                     (isAnyEnemyNear(simState, others) ? CP_RADIUS - CP_RADIUS_REDUCTION : CP_RADIUS))) {
            simState.cpsPassed++;
            if (arrivalFrame < 0) arrivalFrame = step + 1;
            simNextCpId = (simNextCpId + 1) % checkpointCount;
        }
    }

    SimState simCandidate(SimState simState, double aimAngle, int simThrust, int step,
                          int& simNextCpId, int& arrivalFrame,
                          PodBase others[MAX_OTHERS]) {
        int actualThrust = resolveThrust(simThrust, simState);
        applyRotationAndThrust(simState, aimAngle, actualThrust);

        bool anyModified = others[0].modified || others[1].modified ||
                           (otherCount > 2 && others[2].modified);

        PodBaseState othersAfterMove[MAX_OTHERS];
        computeEnemyMoves(step, others, othersAfterMove);

        SimpleStateCoords myEndPosition = {simState.getX() + simState.getVx(),
                                           simState.getY() + simState.getVy()};

        double earliestCollisionTime = 1.1;
        int earliestCollisionPod = -1;
        for (int podId = 0; podId < otherCount; ++podId) {
            SimpleStateCoords enemyEndPosition = {othersAfterMove[podId].x + othersAfterMove[podId].vx,
                                                  othersAfterMove[podId].y + othersAfterMove[podId].vy};
            double collisionTime = findCollisionTime(othersAfterMove[podId], enemyEndPosition,
                                                     simState, myEndPosition, COLLISION_RADIUS);
            if (collisionTime >= 0 && collisionTime < earliestCollisionTime) {
                earliestCollisionTime = collisionTime;
                earliestCollisionPod = podId;
            }
        }

        if (earliestCollisionPod < 0 && !anyModified) {
            fastPath(simState, step, simNextCpId, arrivalFrame);
            return simState;
        }

        slowPath(simState, simThrust, step, simNextCpId, arrivalFrame, earliestCollisionPod, earliestCollisionTime, othersAfterMove, others);
        return simState;
    }

    void rollout(SimState& simState, int fromStep, int& simNextCpId, int& arrivalFrame,
                     PodBase others[MAX_OTHERS]) {
        for (int step = fromStep; step < SIM_DEPTH; step++) {
            Point target = checkpoints[simNextCpId];
            double aimAngle = simState.angleTo(target.x, target.y);
            int rollThrust = angleDependentThrust(aimAngle, simState.getFacingAngle());
            simState = simCandidate(simState, aimAngle, rollThrust, step, simNextCpId, arrivalFrame,
                             others);
        }
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
        double dx = tx - simPod.getX();
        double dy = ty - simPod.getY();
        double distSq = dx*dx + dy*dy;
        double dist = sqrt(distSq);
        double norm = max(1.0, dist);
        double speedToward = (simPod.getVx() * dx + simPod.getVy() * dy) / norm;
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
        for (int i = ENEMY_FIRST; i < otherCount; i++) {
            double dx = x - others[i].getX(), dy = y - others[i].getY();
            double d = dx*dx + dy*dy;
            if (d < minDistSq) minDistSq = d;
        }
        return minDistSq;
    }

    double closestEnemyRelSpeedSq() const {
        double maxRelSq = 0;
        for (int i = ENEMY_FIRST; i < otherCount; i++) {
            double dx = x - others[i].getX(), dy = y - others[i].getY();
            double distSq = dx*dx + dy*dy;
            if (distSq < SHIELD_CHECK_DIST * SHIELD_CHECK_DIST) {
                double rvx = vx - others[i].getVx(), rvy = vy - others[i].getVy();
                double relSq = rvx*rvx + rvy*rvy;
                if (relSq > maxRelSq) maxRelSq = relSq;
            }
        }
        return maxRelSq;
    }

    static bool anyEnemyModified(const PodBase others[], int count) {
        for (int podId = 0; podId < count; ++podId) {
            if (others[podId].modified) return true;
        }
        return false;
    }

    static void initUnmodifiedEnemies(PodBase others[], int count) {
        for (int podId = 0; podId < count; ++podId) others[podId].modified = false;
    }

    double searchFromStep1(const SimState& afterStep0, int simCpId0, int arrival0,
                           const Point& currentCp, const Point& nextCp,
                           PodBase enemies0[MAX_OTHERS]) {
        Point target1 = checkpoints[simCpId0];
        double dirFromStep0 = afterStep0.angleTo(target1.x, target1.y);
        double bestScore = -1e18;
        bool enemiesModified = anyEnemyModified(enemies0, otherCount);

        for (int angleIdx = 0; angleIdx < ANGLE_COUNT_23; angleIdx++) {
            for (int thrustIdx = 0; thrustIdx < THRUST_COUNT_23; thrustIdx++) {
                int simCpId1 = simCpId0;
                int arrival1 = arrival0;
                PodBase enemies1[MAX_OTHERS];
                if (enemiesModified) {
                    for (int podId = 0; podId < otherCount; ++podId) enemies1[podId] = enemies0[podId];
                } else {
                    initUnmodifiedEnemies(enemies1, otherCount);
                }
                SimState afterStep1 = simCandidate(afterStep0, dirFromStep0 + ANGLE_OFFSETS_23[angleIdx],
                                                    THRUST_OPTIONS_23[thrustIdx], 1, simCpId1, arrival1,
                                                    enemies1);

                Point target2 = checkpoints[simCpId1];
                double dirFromStep1 = afterStep1.angleTo(target2.x, target2.y);
                bool enemiesModifiedStep2 = anyEnemyModified(enemies1, otherCount);

                for (int angleIdx2 = 0; angleIdx2 < ANGLE_COUNT_23; angleIdx2++) {
                    for (int thrustIdx2 = 0; thrustIdx2 < THRUST_COUNT_23; thrustIdx2++) {
                        int simCpId = simCpId1;
                        int arrivalFrame = arrival1;
                        PodBase enemies2[MAX_OTHERS];
                        if (enemiesModifiedStep2) {
                            for (int podId = 0; podId < otherCount; ++podId) enemies2[podId] = enemies1[podId];
                        } else {
                            initUnmodifiedEnemies(enemies2, otherCount);
                        }
                        SimState simPod = simCandidate(afterStep1, dirFromStep1 + ANGLE_OFFSETS_23[angleIdx2],
                                                       THRUST_OPTIONS_23[thrustIdx2], 2, simCpId, arrivalFrame,
                                                       enemies2);

                        rollout(simPod, 3, simCpId, arrivalFrame, enemies2);

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

    void searchForBestThrust(const SimState& initial, double dirToCp,
                   const Point& currentCp, const Point& nextCp,
                   const int step0Thrusts[], int step0ThrustCount,
                   double& bestScoreNormal, int& bestThrustNormal, double& bestAngleNormal,
                   double& bestScoreBoost, double& bestAngleBoost,
                   double& bestScoreShield, double& bestAngleShield) {
        for (int angleIdx = 0; angleIdx < ANGLE_COUNT_1; angleIdx++) {
            for (int thrustIdx = 0; thrustIdx < step0ThrustCount; thrustIdx++) {
                double moveAngle = dirToCp + ANGLE_OFFSETS_1[angleIdx];
                int simCpId0 = nextCpId;
                int arrival0 = -1;
                PodBase enemies0[MAX_OTHERS];
                initUnmodifiedEnemies(enemies0, otherCount);
                SimState afterStep0 = simCandidate(initial, moveAngle, step0Thrusts[thrustIdx],
                                                    0, simCpId0, arrival0, enemies0);

                double score = searchFromStep1(afterStep0, simCpId0, arrival0,
                                               currentCp, nextCp, enemies0);

                int currentThrust = step0Thrusts[thrustIdx];
                if (currentThrust == 650) {
                    if (score > bestScoreBoost) {
                        bestScoreBoost = score;
                        bestAngleBoost = moveAngle;
                    }
                } else if (currentThrust == SHIELD_THRUST_VALUE) {
                    if (score > bestScoreShield) {
                        bestScoreShield = score;
                        bestAngleShield = moveAngle;
                    }
                } else {
                    if (score > bestScoreNormal) {
                        bestScoreNormal = score;
                        bestThrustNormal = currentThrust;
                        bestAngleNormal = moveAngle;
                    }
                }
            }
        }
    }

    void findBestMove() {
        fastPathCount = 0; slowPathCount = 0; enemyCollisionDetected = false;
        cerr<<"m4";
        precomputeOthers();

        SimState initial = SimState{
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
                        && boostHoldTurns == 0
                        && distToCpSq > (double)BOOST_MIN_DIST * BOOST_MIN_DIST;
        double maxRelSpeedSq = closestEnemyRelSpeedSq();
        bool canShield = shieldCooldown == 0;

        if (minEnemyDistSqVal < SHIELD_CHECK_DIST * SHIELD_CHECK_DIST) {
            cerr << "Collision: relV²=" << maxRelSpeedSq
                 << " dist=" << sqrt(minEnemyDistSqVal)
                 << " shield=" << (canShield ? "available" : "no") << endl;
        }

        int step0Thrusts[THRUST_COUNT_1 + 2];
        int step0ThrustCount = THRUST_COUNT_1;
        for (int i = 0; i < THRUST_COUNT_1; i++) step0Thrusts[i] = THRUST_OPTIONS_1[i];
        if (canBoost) step0Thrusts[step0ThrustCount++] = 650;
        if (canShield) step0Thrusts[step0ThrustCount++] = SHIELD_THRUST_VALUE;

        cerr<<"m5";
        searchForBestThrust(initial, dirToCp, currentCp, nextCp,
                  step0Thrusts, step0ThrustCount,
                  bestScoreNormal, bestThrustNormal, bestAngleNormal,
                  bestScoreBoost, bestAngleBoost,
                  bestScoreShield, bestAngleShield);

        cerr<<"m6 F="<<fastPathCount<<" S="<<slowPathCount;
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
            boostHoldTurns = BOOST_MIN_TURN / 2;
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

    static void calcInterceptUrgency(Pod& a, Pod& b, int enemyCps[], int podX[], int podY[], int podCpId[]) {
        int ourBestCpsPassed = max(a.totalCpsPassed, b.totalCpsPassed);
        double ourBestDistSq = min(a.distSqToNextCp(), b.distSqToNextCp());

        int enemiesAhead = 0;
        for (int e = 0; e < 2; e++) {
            double edx = checkpoints[podCpId[2+e]].x - podX[2+e];
            double edy = checkpoints[podCpId[2+e]].y - podY[2+e];
            double enemyDistSq = edx*edx + edy*edy;
            if (isEnemyAhead(enemyCps[e], ourBestCpsPassed, enemyDistSq, ourBestDistSq)) enemiesAhead++;
        }

        Pod& blocker = a.isBlocker ? a : b;
        Point blockerCp = blocker.getNextCp();
        int leadOp = findLeadOpponent(podX, podY, podCpId, enemyCps);
        Point enemyCp = checkpoints[podCpId[leadOp]];
        bool pathCrossesEnemy = segmentIntersectsCircle(
            blocker.x, blocker.y, blockerCp.x, blockerCp.y,
            enemyCp.x, enemyCp.y, INTERCEPT_PATH_RADIUS);
        if (pathCrossesEnemy && enemiesAhead == 0) enemiesAhead = 1;

        double urgency = enemiesAhead * INTERCEPT_URGENCY_PER_ENEMY;
        a.interceptUrgency = a.isBlocker ? urgency : 0.0;
        b.interceptUrgency = b.isBlocker ? urgency : 0.0;
    }

    static int findLeadOpponent(int podX[], int podY[], int podCpId[], int enemyCps[]) {
        if (enemyCps[1] > enemyCps[0]) return 3;
        if (enemyCps[0] > enemyCps[1]) return 2;
        double d2 = (checkpoints[podCpId[2]].x - podX[2]) * (double)(checkpoints[podCpId[2]].x - podX[2])
                   + (checkpoints[podCpId[2]].y - podY[2]) * (double)(checkpoints[podCpId[2]].y - podY[2]);
        double d3 = (checkpoints[podCpId[3]].x - podX[3]) * (double)(checkpoints[podCpId[3]].x - podX[3])
                   + (checkpoints[podCpId[3]].y - podY[3]) * (double)(checkpoints[podCpId[3]].y - podY[3]);
        return (d3 < d2) ? 3 : 2;
    }

    static int enemyStallCounter[2];
    static int fastPathCount, slowPathCount;
    static bool enemyCollisionDetected;

    static void setupLeadEnemy(Pod pods[], int leadOp, int podX[], int podY[], int podCpId[]) {
        for (int i = 0; i < 2; i++) {
            pods[i].leadEnemyX = podX[leadOp];
            pods[i].leadEnemyY = podY[leadOp];
            pods[i].leadEnemyNextCpId = podCpId[leadOp];
            pods[i].leadEnemyIdx = (leadOp == 2) ? 1 : 2;
        }
    }

    static void setupOtherPods(Pod pods[], int podX[], int podY[], int podVx[], int podVy[],
                                int podAngle[], int podCpId[]) {
        for (int i = 0; i < 2; i++) {
            PodBase others[3];
            int teammate = 1 - i;
            others[0] = {{(double)podX[teammate], (double)podY[teammate]}, {(double)podVx[teammate], (double)podVy[teammate]}, podAngle[teammate] * DEG_TO_RAD, podCpId[teammate], false};
            others[1] = {{(double)podX[2], (double)podY[2]}, {(double)podVx[2], (double)podVy[2]}, podAngle[2] * DEG_TO_RAD, podCpId[2], false};
            others[2] = {{(double)podX[3], (double)podY[3]}, {(double)podVx[3], (double)podVy[3]}, podAngle[3] * DEG_TO_RAD, podCpId[3], false};
            pods[i].setOthers(others, 3);
        }
    }

    static void setupTurn(Pod pods[], int podX[], int podY[], int podVx[], int podVy[],
                          int podAngle[], int podCpId[], int enemyCpsPassed[]) {
        cerr<<"m7";
        assignRoles(pods[0], pods[1]);
        int leadOp = findLeadOpponent(podX, podY, podCpId, enemyCpsPassed);
        cerr<<"m8";
        calcInterceptUrgency(pods[0], pods[1], enemyCpsPassed, podX, podY, podCpId);
        setupLeadEnemy(pods, leadOp, podX, podY, podCpId);
        cerr<<"m9";
        setupOtherPods(pods, podX, podY, podVx, podVy, podAngle, podCpId);
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

int PodBase::boostHoldTurns = BOOST_MIN_TURN;
int PodBase::currentTurn = 0;
int Pod::enemyStallCounter[2] = {0, 0};
int Pod::fastPathCount = 0;
int Pod::slowPathCount = 0;
bool Pod::enemyCollisionDetected = false;

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
        cerr<<"st";
        turn++;
        int podX[4], podY[4], podVx[4], podVy[4], podAngle[4], podCpId[4];
        for (int i = 0; i < 4; i++) {
            cin >> podX[i] >> podY[i] >> podVx[i] >> podVy[i] >> podAngle[i] >> podCpId[i]; cin.ignore();
        }

        for (int i = 0; i < 2; i++) {
            pods[i].setState(podX[i], podY[i], podVx[i], podVy[i], podAngle[i], podCpId[i]);
        }

        for (int e = 0; e < 2; e++) {
            int cpId = podCpId[2 + e];
            if (enemyPrevCpId[e] >= 0 && cpId != enemyPrevCpId[e]) enemyCpsPassed[e]++;
            enemyPrevCpId[e] = cpId;
        }

        cerr << "A:us=" << podAngle[0] << "," << podAngle[1] << " en=" << podAngle[2] << "," << podAngle[3];
        cerr<<"m0";
        Pod::setupTurn(pods, podX, podY, podVx, podVy, podAngle, podCpId, enemyCpsPassed);

        Pod::currentTurn = turn;
        if (PodBase::boostHoldTurns > 0) PodBase::boostHoldTurns--;

        int racerIdx  = pods[0].isBlocker ? 1 : 0;
        int blockerIdx = 1 - racerIdx;

        pods[racerIdx].hasTeammateMove = false;
        cerr<<"m1";
        auto t0 = chrono::steady_clock::now();
        pods[racerIdx].findBestMove();
        auto t1 = chrono::steady_clock::now();
        cerr<<"m2";
        pods[blockerIdx].receiveTeammateMove(pods[racerIdx]);
        pods[blockerIdx].findBestMove();
        auto t2 = chrono::steady_clock::now();
        cerr << "T" << turn
             << " R=" << chrono::duration_cast<chrono::microseconds>(t1-t0).count()
             << " B=" << chrono::duration_cast<chrono::microseconds>(t2-t1).count() << endl;

        for (int i = 0; i < 2; i++) {
            if (turn <= 500) pods[i].scoreHistory[turn - 1] = (int)pods[i].chosenScore;
        }

        cerr<<"m3";
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
                    cerr << "Pod" << i << " score: " 
                         << std::accumulate(pods[i].scoreHistory.begin(), pods[i].scoreHistory.end(), 0)
                         << " turns: " << count << endl;
                }
                showResults = false;
            }
        }
    }
}
