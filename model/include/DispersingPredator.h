#pragma once
#include <vector>
#include "glm.hpp"

#if(PREDATOR==DISPERSING)

class Prey;
class Predator
{
  public:
    typedef enum { DISPERSE, PERIPHERAL } tactic;

    Predator();
    Predator(int animatID);
    Predator(int animatID, float parentLockOnDistance, float parentlockOnRadius);
    float calculatePreyRisk(Prey& prey, std::vector<Prey>& preyAnimats) const;
    void calculate(std::vector<Prey>& preyAnimats);
    void update();
    void reset();
    void draw();
    bool isOnFrontSideOfSchool(Prey const& prey) const;
//    tactic selectTactic();
//    int selectTarget(std::vector<Prey> &preyAnimats);
    glm::vec2 getVelocity() const { return speed * heading; }
    std::string paramsStr();
    std::string currentTacticStr();
    std::string tacticProportionStr();

    int id;
    glm::vec2 position;
    glm::vec2 heading;
    float speed;
    glm::vec2 acceleration;
    std::vector<ci::Vec2f> h;

    bool handling;
    int handlingTimer;
    int central;
    int target;
    int huntCount;
    float confusability;
    int confusedCount;
    tactic currentTactic;

    float maxPreyDist;
    float minPreyDist;
    int nearestPrey;
    float minPreyRisk;
    float maxPreyRisk;

    bool lockOn;
    float lockOnDistance;
    float lockOnRadius;
};

#endif