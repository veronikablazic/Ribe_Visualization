#include "cinder/gl/gl.h"
#include "boost/math/constants/constants.hpp"
#include "AppSettings.h"
#include "BasicPredator.h"
#include "Prey.h"

#if(PREDATOR==BASIC || PREDATOR==BASICNEAREST || PREDATOR==BASICCENTRAL || PREDATOR==BASICPERIPHERAL)

using namespace ci;

Predator::Predator() {}

/*Predator::Predator(int animatID)
{
  // acceleration
  acceleration = glm::vec2(.0f, .0f);

  // position
  position = glm::vec2(AppSettings::worldCentre.x, AppSettings::worldCentre.y + (AppSettings::preySize * 2 * 200.0f));

  // speed and heading
  heading = glm::vec2(.0f, -1.0f);
  // speed = randomFloat(AppSettings::minPredatorVelocity, AppSettings::maxPredatorVelocity);

  id = animatID;
  target = -1;
  huntCount = 0;
  confusedCount = 0;
  confusability = 0;
  handling = false;
  handlingTimer = 0;

  //random weights
  float sum = .0f;
  nearestWeight = randomFloat(.0f, 1.0f);
  sum += nearestWeight;
  peripheralWeight = randomFloat(.0f, 1.0f);
  sum += peripheralWeight;
  centreWeight = randomFloat(.0f, 1.0f);
  sum += centreWeight;
  isExhausted = false;

  nearestWeight /= sum;
  peripheralWeight /= sum;
  centreWeight /= sum;
  minPreyRisk = std::numeric_limits<float>::max();
  maxPreyRisk = std::numeric_limits<float>::lowest();

  currentTactic = selectTactic();

  bodyLength = AppSettings::predatorSize;
  energy = 360; 
  regenerationGain = 30.0f / 100000.0f;

  sustainedSpeed = bodyLength * 0.75;
  prolongedSpeed = bodyLength * 2;
  burstSpeed = bodyLength * 3.75;

  speed = randomFloat(sustainedSpeed, prolongedSpeed); // nov nacin dolocanja hitrosti

  distFromTarget = 100000;
  catchDistance = randomFloat(5, 30);

}*/

Predator::Predator(int animatID, float nW, float pW, float cW)
{
  // acceleration
  acceleration = glm::vec2(.0f, .0f);

  // position
  position = glm::vec2(AppSettings::worldCentre.x, AppSettings::worldCentre.y + (AppSettings::preySize * 2 * 200.0f));

  // speed and heading
  heading = glm::vec2(.0f, -1.0f);
  // speed = randomFloat(AppSettings::minPredatorVelocity, AppSettings::maxPredatorVelocity);

  id = animatID;
  target = -1;
  huntCount = 0;
  confusedCount = 0;
  confusability = 0;
  handling = false;
  handlingTimer = 0;

  nearestWeight = nW;
  peripheralWeight = pW;
  centreWeight = cW;

  attackCount = 0;
  nearestCount = 0;
  peripheralCount = 0;
  centreCount = 0;

  minPreyRisk = std::numeric_limits<float>::max();
  maxPreyRisk = std::numeric_limits<float>::lowest();

  currentTactic = selectTactic();

  bodyLength = AppSettings::predatorSize; //every fish has different size
  energy = 1;// 1; //normalised fish energy
  regenerationGain = 30.0f / 100000.0f; //regeneration in 30'

  sustainedSpeed = AppSettings::predatorSize * 0.75;
  prolongedSpeed = AppSettings::predatorSize * 2;
  burstSpeed = AppSettings::predatorSize * 3.75;

  speed = randomFloat(sustainedSpeed, prolongedSpeed);
  attackZoneAngle = (std::_Pi / 6);

  distFromTarget = 1000000;

}

std::string Predator::paramsStr()
{
  std::stringstream s;

  s << std::setprecision(2);
  s << centreWeight << "/" << nearestWeight << "/" << peripheralWeight;

  return s.str();
}
std::string Predator::currentTacticStr()
{
  std::string tacticStr[] {"NEAREST", "PERIPHERAL", "CENTRE"};

  if (currentTactic < 0)
    return "";

  return tacticStr[currentTactic];
}
std::string Predator::tacticProportionStr()
{
  std::stringstream s;

  s << std::setprecision(2);
  s << float(centreCount) / float(attackCount) << "/"
    << float(nearestCount) / float(attackCount) << "/"
    << float(peripheralCount) / float(attackCount);

  return s.str();
}

float Predator::calculatePreyRisk(Prey &p, std::vector<Prey>& preyAnimats) const
{
  float risk = 0.f;

  float dist = 0.f;
  switch (currentTactic)
  {
  case NEAREST:
    dist = glm::distance(p.position, position) - AppSettings::preySize - AppSettings::predatorSize;
    if (dist < .0f)
      dist = .0f;
    risk = (AppSettings::huntSize - dist);
    break;
  case PERIPHERAL:
    if (isOnFrontSideOfSchool(p))
      risk = p.peripherality;
    else
      risk = -1.f;
    break;
  case CENTRE:
    risk = 1.f - p.peripherality;
    break;
  }

  if (target != -1)
  {
    if (target == p.id)
      risk = p.confusabilty;
    else
      risk = -1.f;
  }

  if (p.isDead)
    risk = -1.f;

  return risk;
}

void Predator::calculate(std::vector<Prey>& preyAnimats) {
	// reset accelertion to 0 each cycle
	acceleration = glm::vec2(.0f, .0f);

	// used for visualization of prey's Risk
	minPreyRisk = std::numeric_limits<float>::max();
	maxPreyRisk = std::numeric_limits<float>::lowest();

	// confusability
	int noOfConfusors = 0;
	for (Prey p : preyAnimats) {

		// used for visualization of prey's Risk
		if ((!p.isDead) && (p.risk >= 0.f)) {
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
	if (!handling) {
		// neighbour data
		glm::vec2 huntVector = glm::vec2(.0f, .0f);

		// if has target and target no dead
		if ((target != -1) && (!preyAnimats[target].isDead)) {
      
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

				// isce novo tarco, ker se zmede ali je ujel ribo
				target = -1;
				handling = true;
				currentTactic = selectTactic();
			}

			// target out of range
			else if (distFromTarget > AppSettings::huntSize) {
				target = -1;
				currentTactic = selectTactic();
			}
			// zheng confusability
			else if (CONFUSABILITY == 2) {

				huntVector = glm::normalize(preyAnimats[target].position - position);
				std::vector<int> targetsInAttackZone;
				std::vector<float> targetsInAttackZoneDistance;

				int index = 0;
				float min_distance = 100000.0f;
				for (Prey p : preyAnimats) {
					if (!p.isDead) {
						glm::vec2 p_huntVector = glm::normalize(p.position - position);
						// angle between current target direction and animat p
						float p_alpha = std::acos((p_huntVector.x * huntVector.x + p_huntVector.y * huntVector.y));
						if (p_alpha < 0) p_alpha += 2 * std::_Pi;
						if (p_alpha < attackZoneAngle / 2) {
							float p_dist = glm::distance(p.position, position) - AppSettings::preySize - AppSettings::predatorSize;
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
						if ((min_distance + 2.0f) < targetsInAttackZoneDistance[i]) {
							targetsInAttackZone2.push_back(targetsInAttackZone[i]);
						}
					}
				}
				else target = -1;

				n = targetsInAttackZone2.size();
				if(n > 0) target = targetsInAttackZone2[std::rand() % (n)];
			}
		}

		// find target
		else {
		  target = selectTarget(preyAnimats);
		}

		// normalize and multiply with weight
		if (target != -1) {
		  huntVector = glm::normalize(preyAnimats[target].position - position);
		  if(!isExhausted) acceleration = huntVector * AppSettings::maxPredatorForce;
		  // still changes direction to stay within reach
		  else acceleration = huntVector * 0.08f;
		}
	}
	// handling
	else {
		handlingTimer++;
		if (handlingTimer > AppSettings::handlingTime) {
			handling = false;
			handlingTimer = 0;
		}
	}
}

void Predator::update() {

	// update speed and heading
	glm::vec2 velocity = getVelocity();

	// if fish has energy and we're using energy mode
	if (ENERGY > 0) {

		// added acceleration if fish still has energy
		if (!((energy > 0) && (!isExhausted))) {
			isExhausted = true;
			speed = AppSettings::minPredatorVelocity; // minimum speed
			velocity = getVelocity();
		}
	}
	
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
	
	// if we're using oxygen consimption
	if (ENERGY == 1) {
		oxygenConsumption = pow(62.9, (0.21 * (speed / bodyLength))) / 36; //mgO2 / kg h
		oxygenConsumption = oxygenConsumption / 360;

		if (speed > AppSettings::minPredatorVelocity) {
			energy -= oxygenConsumption;
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

		if (speed > AppSettings::minPredatorVelocity) {
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

Predator::tactic Predator::selectTactic()
{
  float random = randomFloat(.0f, 1.0f);

  attackCount++;

  float threshold = nearestWeight;
  if (random < threshold)
  {
    nearestCount++;
    return NEAREST;
  }

  threshold += peripheralWeight;
  if (random < threshold)
  {
    peripheralCount++;
    return PERIPHERAL;
  }

  centreCount++;
  return CENTRE;
}

int Predator::selectTarget(std::vector<Prey> &preyAnimats)
{
  int newTarget = -1;

  // nearest by distance
  if (currentTactic == NEAREST)
  {
    float minDist = std::numeric_limits<float>::max();
    for (Prey p : preyAnimats)
    {
      float distFromPrey = glm::distance(p.position, position) - AppSettings::preySize - AppSettings::predatorSize;
      if (distFromPrey < .0f)
        distFromPrey = .0f;
      if (!p.isDead && distFromPrey < minDist && distFromPrey < AppSettings::huntSize)
      {
        minDist = distFromPrey;
        newTarget = p.id;
      }
    }
  }
  // peripheral is the one with the largest peripheriality
  else if (currentTactic == PERIPHERAL)
  {
    float maxPer = std::numeric_limits<float>::lowest();
    for (Prey p : preyAnimats)
    {
      float distFromPrey = glm::distance(p.position, position) - AppSettings::preySize - AppSettings::predatorSize;
      if (distFromPrey < .0f)
        distFromPrey = .0f;
      if (!p.isDead && p.peripherality > maxPer && distFromPrey < AppSettings::huntSize && isOnFrontSideOfSchool(p))
      {
        maxPer = p.peripherality;
        newTarget = p.id;
      }
    }
  }
  // centre is the one with the smallest peripheriality
  else if (currentTactic == CENTRE)
  {
    float minPer = std::numeric_limits<float>::max();
    for (Prey p : preyAnimats)
    {
      float distFromPrey = glm::distance(p.position, position) - AppSettings::preySize - AppSettings::predatorSize;
      if (distFromPrey < .0f)
        distFromPrey = .0f;
      if (!p.isDead && p.peripherality < minPer && distFromPrey < AppSettings::huntSize)
      {
        minPer = p.peripherality;
        newTarget = p.id;
      }
    }
  }

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