#include <eigen3/Eigen/Dense>

constexpr float PI = 3.1415927f;

struct ObjectState
{
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    Eigen::Vector3f orientation;
};

struct Sphere
{
    ObjectState objectState;
    float radius;
};

struct Box
{
    ObjectState objectState;
    Eigen::Vector3f size;
};

struct Action
{
    float throttle;
    float steering;
};

struct Player
{
    ObjectState carState;
    Action action;
    uint8_t score;
};

const std::vector<Player> initialPlayers = {
    Player{.carState = {.position = {0.0f, 0.375f, -40.0f},
                        .velocity = {0.0f, 0.0f, 0.0f},
                        .orientation = {0.0f, 0.0f, 0.0f}},
           .action = {.throttle = 0.0f, .steering = 0.0f},
           .score = 0},
    Player{.carState = {.position = {0.0f, 0.375f, 40.0f},
                        .velocity = {0.0f, 0.0f, 0.0f},
                        .orientation = {0.0f, PI, 0.0f}},
           .action = {.throttle = 0.0f, .steering = 0.0f},
           .score = 0},
};
const Sphere initialBall = {.objectState = {.position = {0.0f, 1.0f, 0.0f},
                                            .velocity = {0.0f, 0.0f, 0.0f},
                                            .orientation = {0.0f, 0.0f, 0.0f}},
                            .radius = 1.0f};
const Eigen::Vector2f goal = {20.0, 8.0};
const Eigen::Vector3f carSize = {1.25f, 0.75f, 2.f};
const Eigen::Vector3f arenaSize = {100.0f, 20.0f, 200.0f};

void physicsStep(const Eigen::Vector3f &arenaSize, const Eigen::Vector2f &goal, Sphere &ball, const Eigen::Vector3f &carSize, std::vector<Player> &players, const bool detectGoals)
{
    constexpr uint FREQUENCY = 60;
    constexpr float PERIOD = 1.f / FREQUENCY;
    constexpr float MAX_ACCELERATION = 30;
    constexpr float CAR_FRICTION = 10;
    constexpr float BALL_FRICTION = 10;
    constexpr float TURN_RADIUS_MIN = 0.3;
    constexpr float TURN_RADIUS_RATE = 0.5;
    constexpr float MAX_SPEED = 40;
    constexpr float GRAVITY = 4;
    constexpr float BALL_RESTITUTION = 0.6;
    constexpr float MAX_DELTA_SPEED = MAX_ACCELERATION * PERIOD;
    constexpr float DELTA_CAR_FRICTION = CAR_FRICTION * PERIOD;
    constexpr float DELTA_BALL_FRICTION = BALL_FRICTION * PERIOD;
    constexpr float DELTA_GRAVITY = GRAVITY * PERIOD;
    // car
    // acceleration
    Eigen::Vector3f halfArenaSize = arenaSize / 2.f;
    Eigen::Vector3f halfCarSize = carSize / 2.f;
    std::vector<Eigen::Vector2f> localCorners = {
        {-halfCarSize.x(), -halfCarSize.z()},
        {halfCarSize.x(), -halfCarSize.z()},
        {-halfCarSize.x(), halfCarSize.z()},
        {halfCarSize.x(), halfCarSize.z()}};
    for (auto &[carState, action, _] : players)
    {
        Eigen::Vector3f orientationVector = {std::sin(carState.orientation.y()), 0.0f, std::cos(carState.orientation.y())};
        carState.velocity += orientationVector * MAX_DELTA_SPEED * action.throttle;
        float speed = carState.velocity.norm();
        if (speed > DELTA_CAR_FRICTION)
        {
            carState.velocity -= carState.velocity.normalized() * DELTA_CAR_FRICTION;
            speed -= DELTA_CAR_FRICTION;
            if (speed > MAX_SPEED)
            {
                carState.velocity *= MAX_SPEED / speed;
                speed = MAX_SPEED;
            }
        }
        else
        {
            carState.velocity.setZero();
            speed = 0;
        }
        // steering
        int backwards = carState.velocity.dot(orientationVector) < 0 ? -1 : 1;
        carState.orientation.y() -= backwards * action.steering * speed / (speed * TURN_RADIUS_RATE + TURN_RADIUS_MIN) * PERIOD;
        carState.velocity = backwards * Eigen::Vector3f(std::sin(carState.orientation.y()), 0.0f, std::cos(carState.orientation.y())) * speed;
        // wall collision
        // TODO: make more efficient
        for (const auto &localCorner : localCorners)
        {
            Eigen::Vector2f globalCorner = Eigen::Rotation2Df(carState.orientation.y()).toRotationMatrix() * localCorner + Eigen::Vector2f(carState.position.x(), carState.position.z());
            float xDistance = std::abs(globalCorner.x()) - halfArenaSize.x();
            float zDistance = std::abs(globalCorner.y()) - halfArenaSize.z();
            if (xDistance > 0)
            {
                int leftRight = globalCorner.x() > 0 ? 1 : -1;
                carState.position.x() -= leftRight * (xDistance + 0.001f);
                if (leftRight * carState.velocity.x() > 0)
                {
                    carState.velocity.x() = 0;
                }
            }
            if (zDistance > 0)
            {
                int backFront = globalCorner.y() > 0 ? 1 : -1;
                carState.position.z() -= backFront * (zDistance + 0.001f);
                if (backFront * carState.velocity.z() > 0)
                {
                    carState.velocity.z() = 0;
                }
            }
        }

        // car ball collision
        Eigen::AngleAxisf::Matrix3 rotation = Eigen::AngleAxisf(carState.orientation.y(), Eigen::Vector3f::UnitY()).toRotationMatrix();
        Eigen::Vector3f localBallPosition = rotation.transpose() * (ball.objectState.position - carState.position);
        Eigen::Vector3f difference = localBallPosition.cwiseMax(-halfCarSize).cwiseMin(halfCarSize) - localBallPosition;
        float distance = difference.norm();
        if (distance < ball.radius)
        {
            ball.objectState.position -= rotation * difference * (ball.radius / distance - 1);
            Eigen::Vector3f collisionNormal = (ball.objectState.position - carState.position).normalized();
            float velocityAlongNormal = (ball.objectState.velocity - carState.velocity).dot(collisionNormal);
            if (velocityAlongNormal < 0)
            {
                ball.objectState.velocity -= 1 * velocityAlongNormal * collisionNormal;
            }
        }
    }
    // ball
    // vertical
    if (ball.objectState.position.y() < ball.radius)
    {
        ball.objectState.position.y() = ball.radius;
        ball.objectState.velocity.y() *= ball.objectState.velocity.y() < -0.1 ? -BALL_RESTITUTION : 0;
    }
    else if (ball.objectState.position.y() > ball.radius)
    {
        ball.objectState.velocity.y() -= DELTA_GRAVITY;
        if (ball.objectState.position.y() > arenaSize.y() - ball.radius)
        {
            ball.objectState.position.y() = arenaSize.y() - ball.radius;
            ball.objectState.velocity.y() *= -1;
        }
    }
    // friction
    if (ball.objectState.position.y() == ball.radius)
    {
        Eigen::Vector3f &velocity = ball.objectState.velocity;
        float scale = std::max(1 - DELTA_BALL_FRICTION / std::hypot(velocity.x(), velocity.z()), 0.f);
        velocity.x() *= scale;
        velocity.z() *= scale;
    }
    // side walls
    if (halfArenaSize.x() - abs(ball.objectState.position.x()) < ball.radius)
    {
        ball.objectState.position.x() = (ball.objectState.position.x() < 0 ? -1 : 1) * (halfArenaSize.x() - ball.radius);
        ball.objectState.velocity.x() *= -BALL_RESTITUTION;
    }
    // front + back wall
    if (halfArenaSize.z() - abs(ball.objectState.position.z()) < ball.radius && (ball.objectState.position.y() > goal.y() || abs(ball.objectState.position.x()) > goal.x() / 2))
    {
        ball.objectState.position.z() = (ball.objectState.position.z() < 0 ? -1 : 1) * (halfArenaSize.z() - ball.radius);
        ball.objectState.velocity.z() *= -BALL_RESTITUTION;
    }
    // goal
    if (detectGoals && abs(ball.objectState.position.z()) > arenaSize.z() / 2 + ball.radius)
    {
        players[ball.objectState.position.z() < 0 ? 1 : 0].score += 1;
        ball.objectState.position.setZero();
        ball.objectState.velocity.setZero();
    }
    // goal posts + crossbar
    for (int i = 0; i < 2; ++i)
    {
        if ((i == 0) ? ball.objectState.position.y() < goal.y() - ball.radius : abs(ball.objectState.position.x()) < goal.x() / 2 - ball.radius)
        {
            Eigen::Vector3f difference = (i == 0) ? Eigen::Vector3f(goal.x() / 2 - abs(ball.objectState.position.x()), 0.f, arenaSize.z() / 2 - abs(ball.objectState.position.z())) : Eigen::Vector3f(0.f, goal.y() - ball.objectState.position.y(), arenaSize.z() / 2 - abs(ball.objectState.position.z()));
            float distance = difference.norm();
            if (distance < ball.radius)
            {
                Eigen::Vector3f adjustedDifference = difference.cwiseProduct(ball.objectState.position.cwiseSign());
                ball.objectState.position -= adjustedDifference * (ball.radius / distance - 1);
                Eigen::Vector3f normal = adjustedDifference / distance;
                ball.objectState.velocity -= 2 * ball.objectState.velocity.dot(normal) * normal;
            }
        }
    }

    for (auto &[carState, _0, _1] : players)
        carState.position += carState.velocity * PERIOD;
    ball.objectState.position += ball.objectState.velocity * PERIOD;
}
