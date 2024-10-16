#include <eigen3/Eigen/Dense>

#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#else
#define ZoneScopedN(x)
#endif

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
};

const std::vector<Player> initialPlayers = {
    Player{.carState = {.position = {0.0f, 0.375f, -40.0f},
                        .velocity = {0.0f, 0.0f, 0.0f},
                        .orientation = {0.0f, 0.0f, 0.0f}},
           .action = {.throttle = 0.0f, .steering = 0.0f}},
    Player{.carState = {.position = {0.0f, 0.375f, 40.0f},
                        .velocity = {0.0f, 0.0f, 0.0f},
                        .orientation = {0.0f, PI, 0.0f}},
           .action = {.throttle = 0.0f, .steering = 0.0f}},
};
const Sphere initialBall = {.objectState = {.position = {0.0f, 1.0f, 0.0f},
                                            .velocity = {0.0f, 0.0f, 0.0f},
                                            .orientation = {0.0f, 0.0f, 0.0f}},
                            .radius = 1.0f};
const Eigen::Vector2f goal = {20.0, 8.0};
const Eigen::Vector3f carSize = {1.25f, 0.75f, 2.f};
const Eigen::Vector3f arenaSize = {100.0f, 20.0f, 200.0f};

void physicsStep(const Eigen::Vector3f &arenaSize, const Eigen::Vector2f &goal, Sphere &ball, const Eigen::Vector3f &carSize, std::vector<Player> &players, const bool detectGoals, uint8_t scores[2] = nullptr)
{
    ZoneScopedN("physics step");
    constexpr uint FREQUENCY = 60;
    constexpr float PERIOD = 1.f / FREQUENCY;
    constexpr float MAX_ACCELERATION = 30;
    constexpr float CAR_FRICTION = 5;
    constexpr float BALL_FRICTION = 10;
    constexpr float STEERING_SCALE = 2.1;
    constexpr float MAX_SPEED = 40;
    constexpr float GRAVITY = 4;
    constexpr float BALL_RESTITUTION = 0.6;
    constexpr float MAX_DELTA_SPEED = MAX_ACCELERATION * PERIOD;
    constexpr float DELTA_CAR_FRICTION = CAR_FRICTION * PERIOD;
    constexpr float DELTA_BALL_FRICTION = BALL_FRICTION * PERIOD;
    constexpr float DELTA_GRAVITY = GRAVITY * PERIOD;

    Eigen::Vector3f halfArenaSize = arenaSize / 2.f;
    Eigen::Vector3f halfCarSize = carSize / 2.f;
    std::vector<Eigen::Vector2f> localCorners = {
        {-halfCarSize.x(), -halfCarSize.z()},
        {halfCarSize.x(), -halfCarSize.z()},
        {-halfCarSize.x(), halfCarSize.z()},
        {halfCarSize.x(), halfCarSize.z()}};
    
    // cars
    for (auto &[carState, action] : players)
    {
        ZoneScopedN("car acceleration and collision");
        // steering
        const int backwards = carState.velocity.dot(Eigen::Vector3f{std::sin(carState.orientation.y()), 0.0f, std::cos(carState.orientation.y())}) < 0 ? -1 : 1;
        carState.orientation.y() -= backwards * action.steering * STEERING_SCALE * PERIOD;
        // acceleration
        float speed = carState.velocity.norm();
        speed += backwards * action.throttle * MAX_DELTA_SPEED - std::min(DELTA_CAR_FRICTION, speed);
        carState.velocity = backwards * speed * Eigen::Vector3f{std::sin(carState.orientation.y()), 0.0f, std::cos(carState.orientation.y())};
        if (speed > MAX_SPEED)
        {
            carState.velocity *= MAX_SPEED / speed;
            speed = MAX_SPEED;
        }
        // wall collision
        if (abs(carState.position.x()) > halfArenaSize.x() - carSize.z() || abs(carState.position.z()) > halfArenaSize.z() - carSize.z())
        {
            Eigen::Rotation2Df carRotation(carState.orientation.y());
            for (const auto &localCorner : localCorners)
            {
                Eigen::Vector2f globalCorner = Eigen::Vector2f(carState.position.x(), carState.position.z()) + carRotation * localCorner;
                struct Dimension
                {
                    float globalCorner;
                    float halfArenaSize;
                    float &position;
                    float &velocity;
                };
                const Dimension dimensions[] = {
                    {globalCorner.x(), halfArenaSize.x(), carState.position.x(), carState.velocity.x()},
                    {globalCorner.y(), halfArenaSize.z(), carState.position.z(), carState.velocity.z()}};

                for (const auto &[globalCorner, halfArenaSize, position, velocity] : dimensions)
                {
                    const float distance = std::abs(globalCorner) - halfArenaSize;
                    if (distance > 0)
                    {
                        const int direction = globalCorner < 0 ? -1 : 1;
                        position -= direction * distance;
                        if (direction * velocity > 0)
                        {
                            velocity = 0;
                        }
                    }
                }
            }
        }
        // car ball collision
        if ((ball.objectState.position - carState.position).norm() < ball.radius + localCorners[0].norm())
        {
            const Eigen::AngleAxisf carRotation(carState.orientation.y(), Eigen::Vector3f::UnitY());
            const Eigen::Vector3f localBallPosition = carRotation.inverse() * (ball.objectState.position - carState.position);
            const Eigen::Vector3f difference = localBallPosition.cwiseMax(-halfCarSize).cwiseMin(halfCarSize) - localBallPosition;
            const float distance = difference.norm();
            if (distance < ball.radius)
            {
                ball.objectState.position -= carRotation * difference * (ball.radius / distance - 1);
                const Eigen::Vector3f collisionNormal = (ball.objectState.position - carState.position).normalized();
                float velocityAlongNormal = (ball.objectState.velocity - carState.velocity).dot(collisionNormal);
                if (velocityAlongNormal < 0)
                {
                    ball.objectState.velocity -= velocityAlongNormal * collisionNormal;
                }
            }
        }
    }
    // ball
    // vertical collision and gravity
    {
        ZoneScopedN("ball vertical collision and gravity");
        if (ball.objectState.position.y() < ball.radius)
        {
            ball.objectState.position.y() = ball.radius;
            ball.objectState.velocity.y() *= ball.objectState.velocity.y() < -0.1 ? -BALL_RESTITUTION : 0;
        }
        else if (ball.objectState.position.y() > ball.radius)
        {
            ball.objectState.velocity.y() -= DELTA_GRAVITY;
            const float maxY = arenaSize.y() - ball.radius;
            if (ball.objectState.position.y() > maxY)
            {
                ball.objectState.position.y() = maxY;
                ball.objectState.velocity.y() *= -1;
            }
        }
    }
    // side-walls
    {
        ZoneScopedN("ball side-walls");
        const float maxX = halfArenaSize.x() - ball.radius;
        if (abs(ball.objectState.position.x()) > maxX)
        {
            ball.objectState.position.x() = (ball.objectState.position.x() < 0 ? -1 : 1) * maxX;
            ball.objectState.velocity.x() *= -BALL_RESTITUTION;
        }
    }
    // goal-walls
    {
        ZoneScopedN("ball goal-walls");
        const float maxZ = halfArenaSize.z() - ball.radius;
        if (abs(ball.objectState.position.z()) > maxZ)
        {
            if (ball.objectState.position.y() > goal.y() || abs(ball.objectState.position.x()) > goal.x() / 2)
            {
                ball.objectState.position.z() = (ball.objectState.position.z() < 0 ? -1 : 1) * maxZ;
                ball.objectState.velocity.z() *= -BALL_RESTITUTION;
            }
            else
            {
                for (int i = 0; i < 2; ++i)
                {
                    if (i == 0 ? abs(ball.objectState.position.x()) > goal.x() / 2 - ball.radius : ball.objectState.position.y() > goal.y() - ball.radius)
                    {
                        const Eigen::Vector3f difference = i == 0 ? Eigen::Vector3f(goal.x() / 2 - abs(ball.objectState.position.x()), 0.f, halfArenaSize.z() - abs(ball.objectState.position.z())) : Eigen::Vector3f(0.f, goal.y() - ball.objectState.position.y(), halfArenaSize.z() - abs(ball.objectState.position.z()));
                        const float distance = difference.norm();
                        const Eigen::Vector3f adjustedDifference = difference.cwiseProduct(ball.objectState.position.cwiseSign());
                        ball.objectState.position -= adjustedDifference * (ball.radius / distance - 1);
                        const Eigen::Vector3f normal = adjustedDifference / distance;
                        ball.objectState.velocity -= 2 * ball.objectState.velocity.dot(normal) * normal;
                    }
                }
            }
            if (detectGoals && abs(ball.objectState.position.z()) > halfArenaSize.z() + ball.radius)
            {
                if (scores != nullptr)
                {
                    scores[ball.objectState.position.z() < 0 ? 1 : 0] += 1;
                }
                ball.objectState.position.setZero();
                ball.objectState.velocity.setZero();
            }
        }
    }
    // friction
    {
        ZoneScopedN("ball friction");
        if (ball.objectState.position.y() == ball.radius)
        {
            Eigen::Vector3f &velocity = ball.objectState.velocity;
            const float scale = std::max(1 - DELTA_BALL_FRICTION / std::hypot(velocity.x(), velocity.z()), 0.f);
            velocity.x() *= scale;
            velocity.z() *= scale;
        }
    }
    // change position of car and ball
    {
        ZoneScopedN("change position of car and ball");
        for (auto &[carState, _] : players)
            carState.position += carState.velocity * PERIOD;
        ball.objectState.position += ball.objectState.velocity * PERIOD;
    }
}
