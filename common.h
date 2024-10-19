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

struct Action
{
    float throttle;
    float steering;
};

struct Player
{
    ObjectState state;
    Action action;
};

const std::vector<Player> initialPlayers = {
    Player{.state = {.position = {0.0f, 0.375f, -40.0f},
                        .velocity = {0.0f, 0.0f, 0.0f},
                        .orientation = {0.0f, 0.0f, 0.0f}},
           .action = {.throttle = 0.0f, .steering = 0.0f}},
    Player{.state = {.position = {0.0f, 0.375f, 40.0f},
                        .velocity = {0.0f, 0.0f, 0.0f},
                        .orientation = {0.0f, PI, 0.0f}},
           .action = {.throttle = 0.0f, .steering = 0.0f}},
};
const ObjectState initialBall = {.position = {0.0f, 1.0f, 0.0f},
                                 .velocity = {0.0f, 0.0f, 0.0f},
                                 .orientation = {0.0f, 0.0f, 0.0f}};

const Eigen::Vector3f arenaSize = {100.0f, 20.0f, 200.0f};
const Eigen::Vector3f carSize = {1.25f, 0.75f, 2.f};
const Eigen::Vector2f goalSize = {20.0, 8.0};
constexpr float ballRadius = 1.0f;

const Eigen::Vector3f halfArenaSize = arenaSize / 2.f;
const Eigen::Vector3f halfCarSize = carSize / 2.f;
const std::vector<Eigen::Vector2f> carCorners = {
    {-halfCarSize.x(), -halfCarSize.z()},
    {halfCarSize.x(), -halfCarSize.z()},
    {-halfCarSize.x(), halfCarSize.z()},
    {halfCarSize.x(), halfCarSize.z()}};

void physicsStep(ObjectState &ball, std::vector<Player> &players, const bool detectGoals, const float duration, uint8_t scores[2] = nullptr)
{
    ZoneScopedN("physics step");
    constexpr float MAX_ACCELERATION = 30;
    constexpr float CAR_FRICTION = 5;
    constexpr float BALL_FRICTION = 10;
    constexpr float STEERING_SCALE = 2.1;
    constexpr float MAX_SPEED = 40;
    constexpr float GRAVITY = 4;
    constexpr float BALL_RESTITUTION = 0.6;
    const float MAX_DELTA_SPEED = MAX_ACCELERATION * duration;
    const float DELTA_CAR_FRICTION = CAR_FRICTION * duration;
    const float DELTA_BALL_FRICTION = BALL_FRICTION * duration;
    const float DELTA_GRAVITY = GRAVITY * duration;

    // cars
    for (auto &[playerState, action] : players)
    {
        ZoneScopedN("car acceleration and collision");
        // steering
        const int backwards = playerState.velocity.dot(Eigen::Vector3f{std::sin(playerState.orientation.y()), 0.0f, std::cos(playerState.orientation.y())}) < 0 ? -1 : 1;
        playerState.orientation.y() -= backwards * action.steering * STEERING_SCALE * duration;
        // acceleration
        float speed = playerState.velocity.norm();
        speed += backwards * action.throttle * MAX_DELTA_SPEED - std::min(DELTA_CAR_FRICTION, speed);
        playerState.velocity = backwards * speed * Eigen::Vector3f{std::sin(playerState.orientation.y()), 0.0f, std::cos(playerState.orientation.y())};
        if (speed > MAX_SPEED)
        {
            playerState.velocity *= MAX_SPEED / speed;
            speed = MAX_SPEED;
        }
        // wall collision
        if (abs(playerState.position.x()) > halfArenaSize.x() - carSize.z() || abs(playerState.position.z()) > halfArenaSize.z() - carSize.z())
        {
            Eigen::Rotation2Df carRotation(playerState.orientation.y());
            for (const auto &localCorner : carCorners)
            {
                Eigen::Vector2f globalCorner = Eigen::Vector2f(playerState.position.x(), playerState.position.z()) + carRotation * localCorner;
                struct Dimension
                {
                    float globalCorner;
                    float halfArenaSize;
                    float &position;
                    float &velocity;
                };
                const Dimension dimensions[] = {
                    {globalCorner.x(), halfArenaSize.x(), playerState.position.x(), playerState.velocity.x()},
                    {globalCorner.y(), halfArenaSize.z(), playerState.position.z(), playerState.velocity.z()}};

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
        if ((ball.position - playerState.position).norm() < ballRadius + carCorners[0].norm())
        {
            const Eigen::AngleAxisf carRotation(playerState.orientation.y(), Eigen::Vector3f::UnitY());
            const Eigen::Vector3f localBallPosition = carRotation.inverse() * (ball.position - playerState.position);
            const Eigen::Vector3f difference = localBallPosition.cwiseMax(-halfCarSize).cwiseMin(halfCarSize) - localBallPosition;
            const float distance = difference.norm();
            if (distance < ballRadius)
            {
                ball.position -= carRotation * difference * (ballRadius / distance - 1);
                const Eigen::Vector3f collisionNormal = (ball.position - playerState.position).normalized();
                float velocityAlongNormal = (ball.velocity - playerState.velocity).dot(collisionNormal);
                if (velocityAlongNormal < 0)
                {
                    ball.velocity -= velocityAlongNormal * collisionNormal;
                }
            }
        }
    }
    // ball
    // vertical collision and gravity
    {
        ZoneScopedN("ball vertical collision and gravity");
        if (ball.position.y() < ballRadius)
        {
            ball.position.y() = ballRadius;
            ball.velocity.y() *= ball.velocity.y() < -0.1 ? -BALL_RESTITUTION : 0;
        }
        else if (ball.position.y() > ballRadius)
        {
            ball.velocity.y() -= DELTA_GRAVITY;
            const float maxY = arenaSize.y() - ballRadius;
            if (ball.position.y() > maxY)
            {
                ball.position.y() = maxY;
                ball.velocity.y() *= -1;
            }
        }
    }
    // side-walls
    {
        ZoneScopedN("ball side-walls");
        const float maxX = halfArenaSize.x() - ballRadius;
        if (abs(ball.position.x()) > maxX)
        {
            ball.position.x() = (ball.position.x() < 0 ? -1 : 1) * maxX;
            ball.velocity.x() *= -BALL_RESTITUTION;
        }
    }
    // goal-walls
    {
        ZoneScopedN("ball goal-walls");
        const float maxZ = halfArenaSize.z() - ballRadius;
        if (abs(ball.position.z()) > maxZ)
        {
            if (ball.position.y() > goalSize.y() || abs(ball.position.x()) > goalSize.x() / 2)
            {
                ball.position.z() = (ball.position.z() < 0 ? -1 : 1) * maxZ;
                ball.velocity.z() *= -BALL_RESTITUTION;
            }
            else
            {
                for (int i = 0; i < 2; ++i)
                {
                    if (i == 0 ? abs(ball.position.x()) > goalSize.x() / 2 - ballRadius : ball.position.y() > goalSize.y() - ballRadius)
                    {
                        const Eigen::Vector3f difference = i == 0 ? Eigen::Vector3f(goalSize.x() / 2 - abs(ball.position.x()), 0.f, halfArenaSize.z() - abs(ball.position.z())) : Eigen::Vector3f(0.f, goalSize.y() - ball.position.y(), halfArenaSize.z() - abs(ball.position.z()));
                        const float distance = difference.norm();
                        const Eigen::Vector3f adjustedDifference = difference.cwiseProduct(ball.position.cwiseSign());
                        ball.position -= adjustedDifference * (ballRadius / distance - 1);
                        const Eigen::Vector3f normal = adjustedDifference / distance;
                        ball.velocity -= 2 * ball.velocity.dot(normal) * normal;
                    }
                }
            }
            if (detectGoals && abs(ball.position.z()) > halfArenaSize.z() + ballRadius)
            {
                if (scores != nullptr)
                {
                    scores[ball.position.z() < 0 ? 1 : 0] += 1;
                }
                ball.position.setZero();
                ball.velocity.setZero();
            }
        }
    }
    // friction
    {
        ZoneScopedN("ball friction");
        if (ball.position.y() == ballRadius)
        {
            Eigen::Vector3f &velocity = ball.velocity;
            const float scale = std::max(1 - DELTA_BALL_FRICTION / std::hypot(velocity.x(), velocity.z()), 0.f);
            velocity.x() *= scale;
            velocity.z() *= scale;
        }
    }
    // change position of car and ball
    {
        ZoneScopedN("change position of car and ball");
        for (auto &[playerState, _] : players) {
            playerState.position += playerState.velocity * duration;
        }
        ball.position += ball.velocity * duration;
    }
}
