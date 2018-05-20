#pragma once
#include <vector>
#include "AppSettings.h"
#include "BasicPredator.h"
#include "DispersingPredator.h"
#include "RandomPredator.h"
#include "glm.hpp"

struct neighbour_info
{
  neighbour_info(glm::vec2 const& Ofs, glm::vec2 const& Dir, glm::vec2 const& Vel, float Dist)
    : ofs(Ofs), dir(Dir), vel(Vel), dist(Dist) {}

  glm::vec2 ofs;
  glm::vec2 vel;
  glm::vec2 dir;
  float dist;
};


class Prey 
{
  public:
    Prey(int animatID);
    void calculate(Predator const& predator, std::vector<Prey> &preyAnimats);
    void update(Predator const& predator);
    void draw();
    void drawTarget(Predator const& predator, bool disperse = false);
    void drawDomainOfDanger();
    bool isInBlindSpot(glm::vec2 const& animatDirection);
    glm::vec2 getVelocity() const { return speed * heading; }

    bool isDead;
    int id;
    glm::vec2 position;
    glm::vec2 heading;
    float speed;
    float nnd;
    float predatorDist;
    glm::vec2 acceleration;

    glm::vec2 peripheralityDir;
    float peripherality;
    float confusabilty;
    float risk;
    ci::ColorA mColor;
    std::vector<ci::Vec2f> h;

    std::vector<neighbour_info> neighbours;

	float energy;
	bool isExhausted = false;
	float regenerationGain = 30.0f / 10000.0f; //regeneration in 30'
	float prevAngle_t;
	float angle_t;
	double ncoll = 0;

};