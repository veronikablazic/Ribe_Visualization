#include "cinder/gl/gl.h"
#include <cmath>
#include <limits>
#include "Prey.h"

using namespace ci;

Prey::Prey(int animatID)
{
	// position
	float x = randomFloat(AppSettings::screenWidth/2 - AppSettings::worldSize, AppSettings::screenWidth/2 + AppSettings::worldSize);
	float y = randomFloat(AppSettings::screenHeight/2 - AppSettings::worldSize, AppSettings::screenHeight/2 + AppSettings::worldSize);
	position = glm::vec2(x, y);

	// speed and heading
	x = randomFloat(-.1f * AppSettings::minPreyVelocity, .1f * AppSettings::minPreyVelocity);
	heading = glm::normalize(glm::vec2(x, -1.0f));
	speed = randomFloat(AppSettings::minPreyVelocity, AppSettings::maxPreyVelocity);

	id = animatID;
	isDead = false;

	acceleration = glm::vec2(.0f, .0f);
	peripheralityDir = glm::vec2(.0f, .0f);
	peripherality = std::numeric_limits<float>::max();

	nnd = std::numeric_limits<float>::max();
	energy = 1;
	angle_t = 0;
}

bool closer(const neighbour_info &ni, const neighbour_info &nj) 
{ 
	return (ni.dist<nj.dist); 
}

void Prey::calculate(Predator const& predator, std::vector<Prey> &preyAnimats)
{
  // reset accelertion to 0 each cycle
  acceleration = glm::vec2(.0f, .0f);

  // neighbour data
  int escapeCount = 0;
  glm::vec2 escapeVector = glm::vec2(.0f, .0f);
  int separationCount = 0;
  glm::vec2 separationVector = glm::vec2(.0f, .0f);
  int alignmentCount = 0;
  glm::vec2 alignmentVector = glm::vec2(.0f, .0f);
  int cohesionCount = 0;
  glm::vec2 cohesionVector = glm::vec2(.0f, .0f);
  int confusorCount = 0;

  // reset peripherality
  peripheralityDir = glm::vec2(.0f, .0f);
  peripherality = std::numeric_limits<float>::max();
  int peripheralityCount = 0;

  // reset nnd
  nnd = std::numeric_limits<float>::max();

  // escape drive
  glm::vec2 ofs = position - predator.position;
  const float dist2 = glm::length2(ofs);
  if ((dist2 > 0.0) && (dist2 < AppSettings::escapeSize * AppSettings::escapeSize))
  {
    float dist = std::sqrt(dist2) - AppSettings::preySize - AppSettings::predatorSize;
    if (dist < .0f)
      dist = .0f;
    if (dist < AppSettings::escapeSize)
    {
      glm::vec2 dir = ofs / sqrt(dist2);
      if (!isInBlindSpot(dir))
      {
        escapeCount++;
        escapeVector = ((AppSettings::escapeSize - dist) / AppSettings::escapeSize) * dir;
      }
    }
  }

//  std::sort(neighbours.begin(), neighbours.end(), closer);

  // if no neighbours set nnd to -1
  if (neighbours.size() == 0)
    nnd = -1;

  long nn = 0;
  // other drives
  for (auto ni = neighbours.begin(); ni != neighbours.end(); ++ni, nn++)
  {
    if ((ni->dist/*- AppSettings::preySize*/) < nnd) // BUG:: dist is already dist -2*preySize (see ni->dist calculation)
    {
      nnd = ni->dist/* - AppSettings::preySize*/;
      if (nnd < .0f)
        nnd = .0f;
    }
    if (!isInBlindSpot(ni->dir))
    {
      if (ni->dist < AppSettings::separationSize)
      {
        separationCount++;
        separationVector += ((AppSettings::separationSize - ni->dist) / AppSettings::separationSize) * -ni->dir;
      }
      else if (ni->dist < AppSettings::alignmentSize)
      {
        alignmentCount++;
        alignmentVector += ni->vel;
      }
      else if (ni->dist < AppSettings::cohesionSize)
      {
        cohesionCount++;
        cohesionVector += ni->ofs;
      }
    }

    if (ni->dist < AppSettings::peripheralitySize)
    {
      peripheralityDir += ni->dir;
      peripheralityCount++;
    }

    if (ni->dist < AppSettings::confusabilitySize)
      confusorCount++;
  }

  confusabilty = 1 / (float)(confusorCount + 1);

  if (peripheralityCount != 0)
  {
    peripheralityDir /= (float)peripheralityCount;
    peripherality = glm::length(peripheralityDir);
    peripheralityDir /= peripherality;
  }

  // calculate drives
  if (escapeCount > 0)
    acceleration += (escapeVector / (float)escapeCount) * AppSettings::escapeWeight;
  if (separationCount > 0 && !selfishEscape)
    acceleration += (separationVector / (float)separationCount) * AppSettings::separationWeight;
  if (alignmentCount > 0 && !selfishEscape)
    acceleration += ((alignmentVector / (float)alignmentCount) - getVelocity()) * AppSettings::alignmentWeight;
  if (cohesionCount > 0) 
    acceleration += ((cohesionVector / (float)cohesionCount)) * AppSettings::cohesionWeight;
#if(PREDATOR!=RANDOM)
  // update prey's risk for color coding visualization
  risk = predator.calculatePreyRisk(*this, preyAnimats);
#endif
  // clamp
  acceleration = limit(acceleration, AppSettings::maxPreyForce);

  neighbours.clear();
}

void Prey::update(Predator const& predator, std::vector<Prey> &preyAnimats)
{

	long rC = 0;
	float r = (std::max(risk, 0.f) - predator.minPreyRisk) / (predator.maxPreyRisk - predator.minPreyRisk);

	if (predator.target != -1) {
		if (predator.target == id)
			r = risk;
		else
			r = 0.f;
	}
	for (auto rL : AppSettings::riskLevels) {
		if (r <= rL)
			break;
		rC++;
	}

	mColor = AppSettings::riskColor[rC];
	if (SELFISH_ESCAPE == 1 && predator.target == id) {
		float dist = sqrt(pow(position.x - predator.position.x, 2) + pow(position.y - predator.position.y, 2));
		if (dist < 100) selfishEscape = true;
		else selfishEscape = false;
	}
	

	// update speed and heading
	glm::vec2 velocity = getVelocity();

	if (PREY_ENERGY > 0) {
		if ((energy > 0) && (!isExhausted)) {
			velocity += acceleration;
		}
		else {
			isExhausted = true;
			speed = AppSettings::minPreyVelocity;
			velocity = getVelocity();
		}
	}
	else {
		velocity += acceleration;
	}

	speed = .0f;
	float speed2 = glm::length2(velocity);

	if (!selfishEscape) {
		if (speed2 > .0f) {
			speed = std::sqrt(speed2);
			heading = velocity / speed;
		}

		// limit speed
		speed = glm::clamp(speed, AppSettings::minPreyVelocity, AppSettings::maxPreyVelocity);
	}
	else {
		// selfish escape
		speed = AppSettings::maxPreyVelocity;
		heading = glm::vec2(predator.heading.y, -predator.heading.x);
	}

	speed = glm::clamp(speed, AppSettings::minPreyVelocity, AppSettings::maxPreyVelocity + glm::length(Ui));

	if (PREY_ENERGY == 1) {
		
		float oxygenConsumption = pow(62.9, (0.21 * ((speed / 1.5) / AppSettings::preySize))) / 36; //mgO2 / kg h
		oxygenConsumption = oxygenConsumption / 360;

		if (speed - glm::length(Ui) > AppSettings::minPreyVelocity) {
			energy -= oxygenConsumption;
			if (energy > 1) energy = 1.0f;
		}
		else {
			energy += regenerationGain;
			if (energy > 1) energy = 1.0f;
		}
	}

  // if we're using Zheng's method
  else if (PREY_ENERGY == 2) {

	  prevAngle_t = angle_t;
	  angle_t = atan(heading.y / heading.x);

	  ncoll = 0;
	  for (int i = 0; i < preyAnimats.size(); i++) {
		  if (id != preyAnimats[i].id && !preyAnimats[i].isDead){
			  float distance = sqrt(pow(position.x - preyAnimats[i].position.x, 2) + pow(position.y - preyAnimats[i].position.y, 2));
			  if (distance < AppSettings::preySize) ncoll += 1;
		  }  
	  }
	  
	  float average_speed = (AppSettings::minPreyVelocity + AppSettings::maxPreyVelocity) / 2;
	  float energyChange = 0.001f * (1 / average_speed) * (AppSettings::minPreyVelocity - speed) 
		  - 0.002f * (1 / pow(std::_Pi, 2)) * pow((angle_t - prevAngle_t), 2)
		  - 0.002f * 1 * ncoll * std::exp(ncoll / 2);

	  if (speed - glm::length(Ui) > AppSettings::minPreyVelocity) {
		  energy += energyChange;
	  }
	  else {
		  energy += regenerationGain;
		  if (energy > 1) energy = 1.0f;
	  }
  }

  // half regenerated
  if (isExhausted && (energy >= 0.5)) {
	  isExhausted = false;
  }

  position += getVelocity();

  if (HYDRO == 1) {
	  //glm::vec2 Ui = glm::vec2(.0f, .0f);
	  Ui = glm::vec2(.0f, .0f);
	  glm::vec2 e_jp, e_jp_polar;
	  glm::vec2 e_j0, e_j0_polar;
	  glm::vec2 u_ji;
	  float p, theta, I;

	  for (int i = 0; i < preyAnimats.size(); i++) {

		  if (id != preyAnimats[i].id && !preyAnimats[i].isDead){
			  // cartesic
			  e_jp = glm::normalize(glm::vec2(position.x - preyAnimats[i].position.x, position.y - preyAnimats[i].position.y));
			  e_j0 = glm::normalize(glm::vec2(e_jp.y, -e_jp.x));
			  p = glm::distance(position, preyAnimats[i].position); 
			  theta = atan2(heading.y, heading.x) - atan2(preyAnimats[i].heading.y, preyAnimats[i].heading.x);

			  // polar
			  e_jp_polar = glm::vec2(1, atan2(e_jp.y, e_jp.x)) * cos(theta);
			  e_j0_polar = glm::vec2(1, atan2(e_j0.y, e_j0.x)) * sin(theta);

			  e_jp = glm::vec2(e_jp_polar.x * cos(e_jp_polar.y), e_jp_polar.x * sin(e_jp_polar.y));
			  e_j0 = glm::vec2(e_j0_polar.x * cos(e_j0_polar.y), e_j0_polar.x * sin(e_j0_polar.y));

			  I = (std::_Pi * AppSettings::preySize * 0.03 * 0.03 * AppSettings::cohesionWeight) / speed;

			  u_ji = I * (e_j0 + e_jp) / (float)(std::_Pi * pow(p, 2));
			  //Ui += u_ji;
			  Ui += u_ji;
		  }
	  }

	  //plenilec
	  e_jp = glm::normalize(glm::vec2(position.x - predator.position.x, position.y - predator.position.y));
	  e_j0 = glm::normalize(glm::vec2(e_jp.y, -e_jp.x));
	  p = glm::distance(position, predator.position);
	  theta = atan2(heading.y, heading.x) - atan2(predator.heading.y, predator.heading.x);
	  e_jp_polar = glm::vec2(1, atan2(e_jp.y, e_jp.x)) * cos(theta);
	  e_j0_polar = glm::vec2(1, atan2(e_j0.y, e_j0.x)) * sin(theta);

	  //cathesic
	  e_jp = glm::vec2(e_jp_polar.x * cos(e_jp_polar.y), e_jp_polar.x * sin(e_jp_polar.y));
	  e_j0 = glm::vec2(e_j0_polar.x * cos(e_j0_polar.y), e_j0_polar.x * sin(e_j0_polar.y));

	  I = (std::_Pi * AppSettings::predatorSize * 0.03 * 0.03 * AppSettings::cohesionWeight) / speed;
	  u_ji = I * (e_j0 + e_jp) / (float)(std::_Pi * pow(p, 2));
	  Ui += u_ji;


	  position += Ui;
	  float a = glm::length(Ui);
	  float b = a;

  }

  do
    h.push_back(Vec2f(position.x, position.y) +Vec2f(heading.x, heading.y) / 2.f);
  while (h.size() < 4);
  if (h.size() > 4)
    h.erase(h.begin());

  neighbours.clear();
}

bool Prey::isInBlindSpot(glm::vec2 const& animatDirection)
{
  return glm::dot(heading, animatDirection) < AppSettings::cosPreyBlindThreshold;
}

void Prey::drawDomainOfDanger()
{
  if (!isDead)
  {
    Vec2f cinderPos = Vec2f(position.x, position.y);

    gl::enableAlphaBlending();
    gl::color(ColorA(mColor, .3f));
    if (nnd >= 0)
    {
      float r = nnd + 2 * AppSettings::preySize;
      gl::drawSolidCircle(cinderPos, std::min(AppSettings::confusabilitySize, r));
    }
    else
      gl::drawSolidCircle(cinderPos, AppSettings::confusabilitySize);
    gl::disableAlphaBlending();
  }
}

void Prey::drawTarget(Predator const& predator, bool disperse)
{
  if (!isDead)
  {
    Vec2f cinderPos = Vec2f(position.x, position.y);
    gl::color(255, 0, 0);
    gl::pushMatrices();
    gl::translate(cinderPos);
    if (disperse)
    {
      gl::scale(AppSettings::catchDistance, AppSettings::catchDistance);
      gl::color(AppSettings::riskColor[AppSettings::riskColor.size() - 1]);
      gl::lineWidth(4.f);
      glm::vec2 dir = glm::normalize(position - predator.position);
      Vec2f ex = Vec2f(dir.x, dir.y);
      Vec2f ey = Vec2f(-dir.y, dir.x);
      gl::drawStrokedTriangle(ex,-ex-ey,-ex+ey);
      gl::lineWidth(1.0f);
    }
    else
    { 
      gl::rotate(45.f);
      gl::scale(AppSettings::catchDistance, AppSettings::catchDistance);
      gl::color(AppSettings::riskColor[AppSettings::riskColor.size() - 1]);
      gl::lineWidth(4.f);
      gl::drawStrokedRect(Rectf(-1.f,-1.f,1.f,1.f));
      gl::lineWidth(1.0f);
    }
    gl::popMatrices();
  }
}

void Prey::draw()
{
  if (h.size() < 4)
    return;
  
  if (!isDead)
  {
    Vec2f cinderPos = Vec2f(position.x, position.y);

    gl::color(Color(0.5f,0.5f,0.5f));
#if 0
    gl::drawSolidCircle(cinderPos, AppSettings::preySize);
    gl::drawLine(cinderPos, cinderPos - Vec2f(getVelocity().x, getVelocity().y));
#else

    Vec2f ex = Vec2f(heading.x, heading.y) * AppSettings::preySize;
    Vec2f ey = Vec2f(-heading.y, heading.x) * AppSettings::preySize;
    Shape2d s;
    //    s.moveTo(cinderPos - ex * speed);
    //    s.lineTo(cinderPos + ey);
    //    s.lineTo(cinderPos + ex);
    //    s.lineTo(cinderPos - ey);
    //    s.close();

//    gl::enableAlphaBlending();
#if(PREDATOR!=RANDOM)
    gl::color(mColor);
#endif
#if(PREDATOR==RANDOM)
	gl::color(AppSettings::riskColor[0]);
#endif
    if (AppSettings::renderShape)
    {
      std::vector<Vec2f> p = h;
      std::reverse(p.begin(), p.end());
      if (p.size() > 0)
      {
        s.moveTo(p[0]);
        ex = (p[1] - p[0]).normalized();
        ey = Vec2f(-ex.y, ex.x);
        s.lineTo(p[0] + ex / 2.f + ey*1.f);
        ex = (p[2] - p[0]).normalized();
        ey = Vec2f(-ex.y, ex.x);
        s.lineTo(p[1] + ey*0.8f);
        ex = (p[3] - p[1]).normalized();
        ey = Vec2f(-ex.y, ex.x);
        s.lineTo(p[2] + ey*0.25f);
        s.lineTo(p[3]);
        s.lineTo(p[2] - ey*0.25f);
        ex = (p[2] - p[0]).normalized();
        ey = Vec2f(-ex.y, ex.x);
        s.lineTo(p[1] - ey*0.8f);
        ex = (p[1] - p[0]).normalized();
        ey = Vec2f(-ex.y, ex.x);
        s.lineTo(p[0] + ex / 2.f - ey*1.f);
        s.close();
        if ((p[3] - p[1]).length() != 0)
          gl::drawSolid(s);
        else
          gl::draw(s);
      }
    }
//    gl::lineWidth(0.5f);
//    gl::drawStrokedCircle(cinderPos, AppSettings::cohesionSize);
//    gl::lineWidth(1.f);
    gl::drawSolidCircle(cinderPos, AppSettings::preySize);
//    gl::disableAlphaBlending();
#endif
  } else {
    gl::color(1.0, 0.0, 0.0);
    gl::lineWidth(4.f);
    gl::pushMatrices();
    gl::translate(position.x,position.y);
    gl::scale(Vec2f(AppSettings::predatorSize, AppSettings::predatorSize));
    gl::drawLine(Vec2f(-1.f, -1.f), Vec2f(1.f, 1.f));
    gl::drawLine(Vec2f(-1.f, 1.f), Vec2f(1.f, -1.f));
    gl::popMatrices();
    gl::lineWidth(1.f);
  }
}