#include "cinder/gl/gl.h"
#include "boost/math/constants/constants.hpp"
#include "AppSettings.h"
#include "RandomPredator.h"
#include "Prey.h"

#if(PREDATOR==RANDOM)

using namespace ci;

Predator::Predator() {}

Predator::Predator(int animatID)
{
  // acceleration
  acceleration = glm::vec2(.0f, .0f);

  // position
  position = glm::vec2(AppSettings::worldCentre.x, AppSettings::worldCentre.y + (AppSettings::preySize * 2 * 200.0f));

  // speed and heading
  heading = glm::vec2(.0f, -1.0f);
  speed = randomFloat(AppSettings::minPredatorVelocity, AppSettings::maxPredatorVelocity);

  id = animatID;
  target = -1;
  huntCount = 0;
  confusedCount = 0;
  confusability = 0;
  handling = false;
  handlingTimer = 0;

  minPreyRisk = std::numeric_limits<float>::max();
  maxPreyRisk = std::numeric_limits<float>::lowest();
}

void Predator::calculate(std::vector<Prey>& preyAnimats)
{
  // reset accelertion to 0 each cycle
  acceleration = glm::vec2(.0f, .0f);

  // used for visualization of prey's Risk
  minPreyRisk = std::numeric_limits<float>::max();
  maxPreyRisk = std::numeric_limits<float>::lowest();

  // confusability
  int noOfConfusors = 0;
  for (Prey p : preyAnimats)
  {
    // used for visualization of prey's Risk
    if ((!p.isDead) && (p.risk >= 0.f))
    {
      minPreyRisk = std::min(minPreyRisk, p.risk);
      maxPreyRisk = std::max(maxPreyRisk, p.risk);
    }

    if (!p.isDead)
    {
      // what's correct now (confusors in target prey's or predator's confusability ZONE????)
      float dist = glm::distance(p.position, position) - AppSettings::preySize - AppSettings::predatorSize;
      if (dist < .0f)
        dist = .0f;
      if (dist < AppSettings::confusabilitySize)
        noOfConfusors++;
    }
  }

  confusability = 1.f - 1.f / std::max(noOfConfusors, 1);

  // only update stuff if not handling
  if (!handling)
  {
    // neighbour data
    glm::vec2 huntVector = glm::vec2(.0f, .0f);

    // if has target and target no dead
    if ((target != -1) && (!preyAnimats[target].isDead))
    {
      // check if caught anything or target out of sight
      Prey& targetPrey = preyAnimats[target];
      float distFromTarget = glm::distance(targetPrey.position, position) - AppSettings::preySize - AppSettings::predatorSize;
      if (distFromTarget < .0f)
        distFromTarget = .0f;

      // catch attempt
      if (distFromTarget < AppSettings::catchDistance)
      {
				if (AppSettings::confusablePredator)
				{
					float random = randomFloat(.0f, 1.0f);

					// not confused
					if (random < 1.f - confusability)
					{
						targetPrey.isDead = true;
						huntCount++;
					}
					// confused
					else
						confusedCount++;
				}
				else
				{
					targetPrey.isDead = true;
					huntCount++;
				}

        target = -1;

        handling = true;
      }

      // target out of range
      if (distFromTarget > AppSettings::huntSize)
      {
        target = -1;
      }
    }
    // find target
    else
    {
      target = selectTarget(preyAnimats);
    }

    // normalize and multiply with weight
    if (target != -1)
    {
      huntVector = glm::normalize(preyAnimats[target].position - position);
      acceleration = huntVector * AppSettings::maxPredatorForce;
    }
  }
  // handling
  else
  {
    handlingTimer++;
    if (handlingTimer > AppSettings::handlingTime)
    {
      handling = false;
      handlingTimer = 0;
    }
  }
}

void Predator::update()
{
  // update speed and heading
  glm::vec2 velocity = getVelocity();
  velocity += acceleration;

  speed = .0f;
  float speed2 = glm::length2(velocity);
  if (speed2 > .0f)
  {
    speed = std::sqrt(speed2);
    heading = velocity / speed;
  }

  // limit speed
  speed = glm::clamp(speed, AppSettings::minPredatorVelocity, AppSettings::maxPredatorVelocity);

  position += getVelocity();

  do
    h.push_back(Vec2f(position.x, position.y) + Vec2f(heading.x, heading.y) / 2.f);
  while (h.size() < 4);
  if (h.size() > 4)
    h.erase(h.begin());
}

bool Predator::isOnFrontSideOfSchool(Prey const& prey) const
{
  if (prey.peripherality == std::numeric_limits<float>::max()) return true;
  glm::vec2 a = glm::normalize(prey.position - position);
  return glm::dot(prey.peripheralityDir, a) > AppSettings::oppositeThreshold;
}

int Predator::selectTarget(std::vector<Prey> &preyAnimats)
{
  int newTarget = -1;

  // randomTarget
	int random = randomInt(0, preyAnimats.size() - 1);
	newTarget = preyAnimats.at(random).id;
  return newTarget;
}

void Predator::draw()
{
	if (h.size() < 4)
		return;

  gl::color(255, 0, 0);
  Vec2f cinderPos = Vec2f(position.x, position.y);

#if 0
  gl::drawSolidCircle(cinderPos, AppSettings::predatorSize);
  gl::drawLine(cinderPos, cinderPos - Vec2f(getVelocity().x, getVelocity().y));
#else
  Vec2f ex = Vec2f(heading.x, heading.y) * AppSettings::predatorSize;
  Vec2f ey = Vec2f(-heading.y, heading.x) * AppSettings::predatorSize;
  Shape2d s;
//  s.moveTo(cinderPos - ex * speed);
//  s.lineTo(cinderPos + ey);
//  s.lineTo(cinderPos + ex);
//  s.lineTo(cinderPos - ey);
//  s.close();
//  gl::draw(s);
  if (AppSettings::renderShape) 
  {
    gl::color(Color::black());
    std::vector<Vec2f> p = h;
    std::reverse(p.begin(), p.end());
    s.moveTo(p[0]);
    ex = (p[1] - p[0]).normalized()*AppSettings::predatorSize;
    ey = Vec2f(-ex.y, ex.x);
    s.lineTo(p[0] + ex / 2.f + ey*1.f);
    ex = (p[2] - p[0]).normalized()*AppSettings::predatorSize;
    ey = Vec2f(-ex.y, ex.x);
    s.lineTo(p[1] + ey*0.8f);
    ex = (p[3] - p[1]).normalized()*AppSettings::predatorSize;
    ey = Vec2f(-ex.y, ex.x);
    s.lineTo(p[2] + ey*0.25f);
    s.lineTo(p[3]);
    s.lineTo(p[2] - ey*0.25f);
    ex = (p[2] - p[0]).normalized()*AppSettings::predatorSize;
    ey = Vec2f(-ex.y, ex.x);
    s.lineTo(p[1] - ey*0.8f);
    ex = (p[1] - p[0]).normalized()*AppSettings::predatorSize;
    ey = Vec2f(-ex.y, ex.x);
    s.lineTo(p[0] + ex / 2.f - ey*1.f);
    s.close();
    if ((p[3] - p[1]).length() != 0)
      gl::drawSolid(s);
    else
      gl::draw(s);
//    gl::drawStrokedCircle(cinderPos, AppSettings::predatorSize);
  }
//  gl::drawSolidCircle(cinderPos, AppSettings::predatorSize);
#endif

  if (handling)
  {
    gl::color(Color::gray(.5));
    const float alpha = 1.f - float(handlingTimer) / float(AppSettings::handlingTime);
    Shape2d arc;
    arc.arc(cinderPos, AppSettings::confusabilitySize, 0.f, 2.f * boost::math::float_constants::pi * alpha);
    gl::lineWidth(3.f);
    gl::draw(arc);
    gl::lineWidth(1.0f);
  }

  gl::color(Color::gray(.4f));
  gl::lineWidth(2.0f);
  gl::drawStrokedCircle(cinderPos, AppSettings::huntSize);
  gl::lineWidth(1.0f);
}

#endif