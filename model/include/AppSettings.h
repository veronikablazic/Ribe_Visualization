#pragma once
#include "glm.hpp"
#include "random.hpp"

namespace AppSettings {

	#define BASIC 0
	#define DISPERSING 1
	#define RANDOM 2
	#define BASICNEAREST 3
	#define BASICCENTRAL 4
	#define BASICPERIPHERAL 5
	#define PREDATOR BASIC

	#define DEFAULT 0
	#define DELAYEDRESPONSE 1
	#define NONCONFUSING 2
	#define PREY DEFAULT

	// 0 - no energy, 1 - oxygen consumption, 2 - zheng
	#define ENERGY 2
	#define PREY_ENERGY 2

	// 0 - predator non confusable, 1 - predator confusable, 2 - predator zheng
	#define CONFUSABILITY 2

	// 0 - no selfish prey escape, 1 - selfish prey escape
	#define SELFISH_ESCAPE 0

	//model
	const int screenWidth = 1280;
	const int screenHeight = 720;

	const int noOfSteps = 2000;

	const float worldSize = 100;
	const glm::vec2 worldCentre = glm::vec2((float)screenWidth / 2, (float)screenHeight / 2);

	// prey settings
	const int noOfPreyAnimats = 0 + 1 * 100;
	// NOTE: this is 1/2 BL - r of circle
	const float preySize = 1.0f;

	const float maxPreyForce = 2.0f;
	/*const float maxPreyVelocity = 8.0f;
	const float minPreyVelocity = 4.0f;*/

	const float maxPreyVelocity = preySize * 8.0f;
	const float minPreyVelocity = preySize * 4.0f;

	const float separationSize = 10.0f;
	const float alignmentSize = 50.0f;
	const float cohesionSize = 200.0f;
	const float peripheralitySize = 200.0f;

#if(PREY == DELAYEDRESPONSE)
	const float escapeSize = 100.0f;
	const float escapeWeight = 12.0f;
#else
	const float escapeSize = 200.0f;
	const float escapeWeight = 5.0f;
#endif

	const float separationWeight = 5.0f;
	const float alignmentWeight = .3f;
	const float cohesionWeight = 0.01f;

	// full blind angle (deg)
	const float preyBlindAngle = 60.0f;
	const float cosPreyBlindThreshold = glm::cos(glm::radians(180.0f - preyBlindAngle / 2.0f));

	// predator settings
	const float predatorSize = 3.0f;
	const int handlingTime = 30;

	const float maxPredatorForce = 2.5f;
	/*const float maxPredatorVelocity = 12.0f;
	const float minPredatorVelocity = 6.0f;*/

	const float maxPredatorVelocity = predatorSize * 4.0f;
	const float minPredatorVelocity = predatorSize * 1.0f;

	const float huntSize = 800.0f;
	const float confusabilitySize = 50.0f;

#if(PREY == NONCONFUSING)
	const bool confusablePredator = false;
#else
	const bool confusablePredator = true;
#endif

	const float catchDistance = 12.0f;

	const float oppositeThreshold = glm::cos(glm::half_pi<float>());

	// visualization settings
	const bool showGUI = true;
	const bool usePredator = true;
	const bool renderShape = true;
	const bool showTacticProportion = false;
	const std::vector<ci::Color> riskColor = {
		ci::Color(19.f / 255.f, 74.f / 255.f, 152.f / 255.f), ci::Color(52.f / 255.f, 72.f / 255.f, 174.f / 255.f), ci::Color(94.f / 255.f, 103.f / 255.f, 199.f / 255.f),
		ci::Color(190.f / 255.f, 190.f / 255.f, 190.f / 255.f),
		ci::Color(255.f / 255.f, 74.f / 255.f, 71.f / 255.f), ci::Color(221.f / 255.f, 38.f / 255.f, 42.f / 255.f), ci::Color(185.f / 255.f, 14.f / 255.f, 21.f / 255.f)
	};
	const std::vector<float> riskLevels = { .05f, .25f, .45f, .55f, .75f, .95f };
}