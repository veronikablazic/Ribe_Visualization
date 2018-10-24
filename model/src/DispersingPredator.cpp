#include "cinder/gl/gl.h"
#include "boost/math/constants/constants.hpp"
#include "AppSettings.h"
#include "DispersingPredator.h"
#include "Prey.h"
#include <algorithm>

#if(PREDATOR==DISPERSING)

using namespace ci;

Predator::Predator() {}

/*Predator::Predator(int animatID)
{
  // acceleration
  acceleration = glm::vec2(.0f, .0f);

  // position
  position = glm::vec2(AppSettings::worldCentre.x, AppSettings::worldCentre.y + (AppSettings::preySize * 2.0f * 200.0f));

  // speed and heading
  heading = glm::vec2(.0f, -1.0f);
  speed = randomFloat(AppSettings::minPredatorVelocity, AppSettings::maxPredatorVelocity);

  id = animatID;
  target = -1;
  central = -1;
  huntCount = 0;
  confusedCount = 0;
  confusability = 0;
  handling = false;
  handlingTimer = 0;

  lockOn = false;
  // random lock-on distances
  lockOnDistance = randomFloat(1.0f, AppSettings::huntSize);
  lockOnRadius = randomFloat(1.0f, AppSettings::huntSize);

  minPreyRisk = std::numeric_limits<float>::max();
  maxPreyRisk = std::numeric_limits<float>::lowest();

//  currentTactic = selectTactic();
}*/

Predator::Predator(int animatID,  float parentLockOnDistance, float parentlockOnRadius)
{
  // acceleration
  acceleration = glm::vec2(.0f, .0f);

  // position
  position = glm::vec2(AppSettings::worldCentre.x, AppSettings::worldCentre.y + (AppSettings::preySize * 2.0f * 200.0f));

  // speed and heading
  heading = glm::vec2(.0f, -1.0f);
  speed = randomFloat(AppSettings::minPredatorVelocity, AppSettings::maxPredatorVelocity);

  id = animatID;
  target = -1;
  central = -1;
  huntCount = 0;
  confusedCount = 0;
  confusability = 0;
  handling = false;
  handlingTimer = 0;

  lockOn = false;
  // lock-on distances from parents
  lockOnDistance = parentLockOnDistance * 2.0f * AppSettings::preySize;
	lockOnRadius = parentlockOnRadius * 2.0f * AppSettings::preySize;

  minPreyRisk = std::numeric_limits<float>::max();
  maxPreyRisk = std::numeric_limits<float>::lowest();

  //  currentTactic = selectTactic();

  bodyLength = AppSettings::predatorSize; //every fish has different size
  energy = 1;// 1; //normalised fish energy
  regenerationGain = 30.0f / 100000.0f; //regeneration in 30'

  sustainedSpeed = AppSettings::predatorSize * 0.75;
  prolongedSpeed = AppSettings::predatorSize * 2;
  burstSpeed = AppSettings::predatorSize * 3.75;

  speed = randomFloat(sustainedSpeed, prolongedSpeed);
  attackZoneAngle = (std::_Pi / 6);
  angle_t = 0;

  distFromTarget = 1000000;

  distanceForAcceleration = 150.0f;
  velocityMultiplier = 2.0f;
  attackPeriod = 2000;
  currentAttackTime = 0;

  predator_speed = {};
  //predatorSpeeds = std::ofstream("Predator");

}

void Predator::reset()
{
  // acceleration
  acceleration = glm::vec2(.0f, .0f);

  // position
  position = glm::vec2(AppSettings::worldCentre.x, AppSettings::worldCentre.y + (AppSettings::preySize * 2.0f * 200.0f));

  // speed and heading
  heading = glm::vec2(.0f, -1.0f);
  speed = randomFloat(AppSettings::minPredatorVelocity, AppSettings::maxPredatorVelocity);

  target = -1;
  central = -1;
  lockOn = false;
  handling = false;
  handlingTimer = 0;
}

std::string Predator::paramsStr()
{
  std::stringstream s;

  //char buff[512];
  //float lodBL = lockOnDistance / (AppSettings::preySize*2.f);
  //sprintf_s<512>(buff, "%.2f", lodBL);
  //s << buff;
  //s << "/";
  //float loRBL = lockOnRadius / (AppSettings::preySize*2.f);
  //sprintf_s<512>(buff, "%.2f", loRBL);
  //s << buff;

  s << std::setprecision(4);
  float lodBL = lockOnDistance / (AppSettings::preySize*2.f);
  float loRBL = lockOnRadius / (AppSettings::preySize*2.f);
  s << lodBL << "/" << loRBL;

  return s.str();
}
std::string Predator::currentTacticStr()
{
  std::string tacticStr[] {"DISPERSE", "PERIPHERAL"};

  if (target == -1)
    currentTactic = Predator::DISPERSE;
  if (target != -1)
    currentTactic = Predator::PERIPHERAL;

  return tacticStr[currentTactic];
}
std::string Predator::tacticProportionStr()
{
  std::stringstream s;

  s << std::setprecision(2);
  //s << float(centreCount) / float(attackCount) << "/"
  //  << float(nearestCount) / float(attackCount) << "/"
  //  << float(peripheralCount) / float(attackCount);

  return s.str();
}

float Predator::calculatePreyRisk(Prey &p, std::vector<Prey>& preyAnimats) const
{
  float risk = 0.f;

  float dist = 0.f;
  dist = glm::distance(p.position, position) - AppSettings::preySize - AppSettings::predatorSize;

  if (handling)
  {
    if (nearestPrey >= 0)
    {
      float distFromNearest = glm::distance(p.position, preyAnimats[nearestPrey].position) - AppSettings::preySize - AppSettings::preySize;
      if (distFromNearest < .0f)
        distFromNearest = .0f;
      risk = ((AppSettings::cohesionSize - distFromNearest) / AppSettings::cohesionSize);
      risk *= ((AppSettings::huntSize - dist) / AppSettings::huntSize);
      risk *= (1.f - p.peripherality) * isOnFrontSideOfSchool(p);
    }
    else
      risk = -1;
  }
  else {
    if (target == -1)
    {
      if (dist < lockOnRadius)
        risk = p.peripherality * (lockOnRadius - dist) / lockOnRadius;
      else
        risk = -1.f;
    }

    if (target != -1)
    {
      if (target == p.id)
        risk = p.confusabilty;
      else
        risk = -1.f;
    }
  }

  if ((p.isDead) || (dist > AppSettings::huntSize))
    risk = -1.f;

  return risk;
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
	
	if (CONFUSABILITY == 1) {
		// gleda razdaljo od predatorja do rib, ce je manjsa od confusability siza, se poveca st. confusorjev
		if (!p.isDead) {
			// what's correct now (confusors in target prey's or predator's confusability ZONE????)
			float dist = glm::distance(p.position, position) - AppSettings::preySize - AppSettings::predatorSize;
			if (dist < .0f) dist = .0f;
			if (dist < AppSettings::confusabilitySize) noOfConfusors++;
		}
	}
  }

  confusability = 1.f - 1.f / std::max(noOfConfusors, 1);

  // only update stuff if not handling
  if (!handling)
  {
    // neighbour data
    glm::vec2 huntVector = glm::vec2(.0f, .0f);
	currentAttackTime++;

	if (EVOL_PARAMETERS == 1) {
		if (currentAttackTime > attackPeriod) {
			target = -1;
			handling = true;
			currentAttackTime = 0;
		}

		if ((target != -1) && (!preyAnimats[target].isDead)) {
			Prey& targetPrey = preyAnimats[target];
			float distFromTarget = glm::distance(targetPrey.position, position) - AppSettings::preySize - AppSettings::predatorSize;

			if (distFromTarget < distanceForAcceleration) isNearCatch = true;
			else isNearCatch = false;
		}
	}
    
    // if has target
    if ((target != -1) && (!preyAnimats[target].isDead))
    {
      // check if caught anything or target out of sight
      Prey& targetPrey = preyAnimats[target];
      float distFromTarget = glm::distance(targetPrey.position, position) - AppSettings::preySize - AppSettings::predatorSize;
      if (distFromTarget < .0f) distFromTarget = .0f;

      // catch attempt
      if (distFromTarget < AppSettings::catchDistance) {
		  if (CONFUSABILITY == 1) {
			  float random = randomFloat(.0f, 1.0f);
			  // not confused
			  if (random < 1.f - confusability) {
				  targetPrey.isDead = true;
				  huntCount++;
			  }
			  // confused
			  else confusedCount++;
		  }
		  // non confused, samo ujame ribo
		  else {
			  targetPrey.isDead = true;
			  huntCount++;
		  }

        central = -1;
        target = -1;
        handling = true;
      }
			
      // target out of range
      else if (distFromTarget > AppSettings::huntSize){
        target = -1;
        central = -1;
      }

	  else if (CONFUSABILITY == 2) {

		  huntVector = glm::normalize(preyAnimats[target].position - position);
		  std::vector<int> targetsInAttackZone;
		  std::vector<float> targetsInAttackZoneDistance;
		  float p_dist = 0.0f;

		  int index = 0;
		  float min_distance = 100000.0f;
		  for (Prey p : preyAnimats) {
			  if (!p.isDead) {
				  glm::vec2 p_huntVector = glm::normalize(p.position - position);
				  // angle between current target direction and animat p
				  float p_alpha = std::acos((p_huntVector.x * huntVector.x + p_huntVector.y * huntVector.y));
				  if (p_alpha < 0) p_alpha += 2 * std::_Pi;
				  if (p_alpha < attackZoneAngle / 2) {
					  // finding the nearest fish in the attack zone
					  p_dist = glm::distance(p.position, position) - AppSettings::preySize - AppSettings::predatorSize;
					  if (p_dist < min_distance) min_distance = p_dist;
					  targetsInAttackZone.push_back(index);
					  targetsInAttackZoneDistance.push_back(p_dist);
				  }
			  }
			  index++;
		  }

		  int n = targetsInAttackZone.size();
		  std::vector<int> targetsInAttackZone2;
		  if (n > 0) {
			  for (int i = 0; i < n; i++){
				  // izberemo samo animate, ki so znotraj tega polja
				  if ((min_distance + 50.0f) > targetsInAttackZoneDistance[i]) {
					  targetsInAttackZone2.push_back(targetsInAttackZone[i]);
				  }
			  }
		  }
		  else target = -1;

		  n = targetsInAttackZone2.size();
		  if (n > 0) target = targetsInAttackZone2[std::rand() % (n)];
	  }
    }
    // else find target
    else
    {
      // find centre of nearest flock
      if (central == -1)
      {
        // copy and sort the prey by distance from predator
        std::vector<const Prey*> preyView(preyAnimats.size());
        std::transform(preyAnimats.begin(), preyAnimats.end(), preyView.begin(), [](Prey const& p) { return &p; });
        std::sort(preyView.begin(), preyView.end(), [](Prey const* a, Prey const* b)
        { return a->predatorDist < b->predatorDist; }
        );

        // get nearest and then the most central in his vicinity
        glm::vec2 positionOfNearest;
        bool first = false;
        float minPeripherality;
        for(auto pp = preyView.begin(); pp != preyView.end(); ++pp)
        {
          Prey const& prey = **pp;
          float distFromPred = glm::distance(prey.position, position) - AppSettings::preySize - AppSettings::predatorSize;
          if (distFromPred < .0f)
            distFromPred = .0f;
          // find nearest
          if (!first)
          {
            if (distFromPred < AppSettings::huntSize && !prey.isDead && isOnFrontSideOfSchool(prey))
            {
              first = true;
              positionOfNearest = prey.position;
              central = prey.id;
              minPeripherality = prey.peripherality;
            }
          }
          // then find most central in its vicinity
          else
          {
            float distFromNearest = glm::distance(positionOfNearest, prey.position) - AppSettings::preySize - AppSettings::predatorSize;
            if (distFromNearest < .0f)
              distFromNearest = .0f;
            // if closer than hunting zone and in nearest vicinity and smaller peripherality
            if (distFromNearest < AppSettings::cohesionSize && distFromPred < AppSettings::huntSize && 
              !prey.isDead && prey.peripherality < minPeripherality && isOnFrontSideOfSchool(prey))
            {
              minPeripherality = prey.peripherality;
              central = prey.id;
            }
          }
        }
      }

      // arppoach centre of nearest flock
      if (central != -1)
      {
        Prey& prey = preyAnimats[central];
        float distFromPrey = glm::distance(prey.position, position) - AppSettings::preySize - AppSettings::predatorSize;
        if (distFromPrey < .0f)
          distFromPrey = .0f;
        // if close enough lockOn else approach
        if (distFromPrey < lockOnDistance)
        {
          lockOn = true;
        }
        else
        {
          huntVector = glm::normalize(prey.position - position);
          acceleration = huntVector * AppSettings::maxPredatorForce;
        }
      }

      // lock on
      if (lockOn)
      {
        // select most peripheral in lockOnRadius
        bool targetFound = false;
        float maxPer = std::numeric_limits<float>::lowest();
        for(auto p = preyAnimats.begin(); p != preyAnimats.end(); ++p)
        {
          float distFromPred = glm::distance(p->position, position) - AppSettings::preySize - AppSettings::predatorSize;
          if (distFromPred < .0f)
            distFromPred = .0f;
          if (!p->isDead && p->peripherality > maxPer && distFromPred < lockOnRadius)
          {
            maxPer = p->peripherality;
            target = p->id;
            targetFound = true;
          }
        }
        // if no possible target in lockOnRadius, cooldown (handling) and create a new attempt
        if (!targetFound)
          handling = true;
      }
    }

    // normalize and multiply with weight
    if (target != -1)
    {
      Prey &targetPrey = preyAnimats[target];
      huntVector = glm::normalize(targetPrey.position - position);
      if(!isExhausted) acceleration = huntVector * AppSettings::maxPredatorForce;
	  else acceleration = huntVector * 0.08f;
    }
  }

  // handling
  if (handling)
  {
    handlingTimer++;
    if (handlingTimer > AppSettings::handlingTime)
    {
      handling = false;
      lockOn = false;
      central = -1;
      target = -1;
      handlingTimer = 0;
    }
  }
}

void Predator::update(std::vector<Prey>& preyAnimats)
{
  // update speed and heading
  glm::vec2 velocity = getVelocity();

  if (ENERGY > 0) {

	  // added acceleration if fish still has energy
	  if (!(energy > 0)) {
		  isExhausted = true;
		  speed = AppSettings::minPredatorVelocity; // minimum speed
		  velocity = getVelocity();
	  }

	  else if (!isNearCatch && EVOL_PARAMETERS == 1) {
		  speed = AppSettings::minPredatorVelocity * velocityMultiplier;
		  velocity = getVelocity();
	  }

	  if (isExhausted) speed = AppSettings::minPredatorVelocity;
  }

  velocity += acceleration;

  speed = 0.0f;
  float speed2 = glm::length2(velocity);
  if (speed2 > 0.0f)
  {
    speed = std::sqrt(speed2);
    heading = velocity / speed;
  }

  // limit speed
  speed = glm::clamp(speed, AppSettings::minPredatorVelocity, AppSettings::maxPredatorVelocity);

  if (ENERGY == 1) {
	  oxygenConsumption = pow(62.9, (0.21 * (speed / bodyLength))) / 36; //mgO2 / kg h
	  oxygenConsumption = oxygenConsumption / 360;

	  if (speed > AppSettings::minPredatorVelocity + 0.1f) {
		  energy -= oxygenConsumption;
		  if (energy < 0) energy = 0;
	  }
	  else {
		  energy += regenerationGain;
	  }
  }
  // if we're using Zheng's method
  else if (ENERGY == 2) {
	  float pi = 3.14159265358979f;
	  prevAngle_t = angle_t;
	  angle_t = atan(heading.y / heading.x);
	  float average_speed = (AppSettings::minPredatorVelocity + AppSettings::maxPredatorVelocity) / 2;
	  float energyChange = 0.001f * (1 / average_speed) * (AppSettings::minPredatorVelocity - speed) - 0.002f * (1 / pow(pi, 2)) * pow((angle_t - prevAngle_t), 2);

	  if (speed > AppSettings::minPredatorVelocity + 0.1f) {
		  energy += energyChange;
	  }
	  else {
		  energy += regenerationGain;
	  }
  }

  // half regenerated
  if (isExhausted && (energy >= 0.5)) {
	  isExhausted = false;
  }

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

  //gl::color(0.f,1.f,0.f);
  //gl::drawStrokedCircle(cinderPos, lockOnRadius);
  //gl::color(0.f, 1.f, 1.f);
  //gl::drawStrokedCircle(cinderPos, lockOnDistance);
}

#endif