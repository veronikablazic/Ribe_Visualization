#pragma once
#include <vector>
#include "glm.hpp"

#if(PREDATOR==BASIC || PREDATOR==BASICNEAREST || PREDATOR==BASICCENTRAL || PREDATOR==BASICPERIPHERAL)

class Prey;
class Predator
{
  public:
    typedef enum { NEAREST, PERIPHERAL, CENTRE } tactic;
    
    Predator();
    Predator(int animatID);
    Predator(int animatID, float nW, float iW, float cW);
    float calculatePreyRisk(Prey& prey, std::vector<Prey>& preyAnimats) const;
    void calculate(std::vector<Prey>& preyAnimats);
    void update();
    void draw();
    bool isOnFrontSideOfSchool(Prey const& prey) const;
    tactic selectTactic();
    int selectTarget(std::vector<Prey> &preyAnimats);
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

    float nearestWeight;
    long nearestCount;
    float peripheralWeight;
    long peripheralCount;
    float centreWeight;
    long centreCount;
    long attackCount;

	// new energy variables from here on

	float distFromTarget;
	float catchDistance;

	float bodyLength;
	float regenerationGain;				// weight that is added to energyTimer at cruisingSpeed
	float energy;
	float angle_t;						// moving direction arctan(vy/vx)
	float prevAngle_t;					// previous moving direction arctan(vy/vx)
	float Ncoll;						// number of collisions
	float rFish;						// distance to other fish
	float averageSpeed;
	int   averageCounter;

	float iterationsToGetRecovered;		// how many frames to get fully recovered
	float sustainedSpeed;				// sustained speed, at this speed fish can swim infinitely and can regenerate 
	float prolongedSpeed;				// a bit faster than sustained, fish will eventualy get fatigue
	float burstSpeed;					// burst speed is maximum swimming speed, fish can't swim long time at this speed
	float oxygenConsumption;			// average oxygen consumption
	float predatorDistnce;				// predator distance when fish stars to accelerating
	bool  isPeripheral;					// peripheral fish consume 10% more oxygen
	bool  isExhausted;					// if fish was exhausted it has to rest 
	float perifcount;

	double burstSpeedStart;				//when fish started to swim at burst speed

	//prey selection parameters
	double angleAttack;
	double depthAttackZone;
	double timePeriod;					// in seconds
	double prevPeriod;

	float attackZoneAngle;
};

#endif