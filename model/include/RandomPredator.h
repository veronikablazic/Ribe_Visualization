#pragma once
#include <vector>
#include "glm.hpp"

#if(PREDATOR==RANDOM)

class Prey;
class Predator
{
  public:  
    Predator();
    Predator(int animatID);
    void calculate(std::vector<Prey>& preyAnimats);
    void update();
    void draw();
    bool isOnFrontSideOfSchool(Prey const& prey) const;
    int selectTarget(std::vector<Prey> &preyAnimats);
    glm::vec2 getVelocity() const { return speed * heading; }

    int id;
    glm::vec2 position;
    glm::vec2 heading;
    float speed;
    glm::vec2 acceleration;
    std::vector<ci::Vec2f> h;

    bool handling;
    int handlingTimer;
    int target;
    int huntCount;
    float confusability;
    int confusedCount;

    float maxPreyDist;
    float minPreyDist;
    int nearestPrey;
    float minPreyRisk;
    float maxPreyRisk;

    long attackCount;
};

#endif