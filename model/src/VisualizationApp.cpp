#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/ImageIo.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/TextureFont.h"
#include "AppSettings.h"
#include "BasicPredator.h"
#include "DispersingPredator.h"
#include "Prey.h"
#include "cinder/Camera.h"
#include "resource.h"
#include "cinder/params/Params.h"

#include "cinder/qtime/MovieWriter.h"
#include "cinder/Utilities.h"

#define SAVE_MOVIE
//#define SAVE_SS

using namespace ci;
using namespace ci::app;
using namespace std;

class VisualizationApp : public AppNative
{
  public:
    void setup();
    void update();
    void draw();
    void searchNeighbours(Predator& predator, vector<Prey>& prey);
    void prepareSettings(Settings *settings);
    void keyDown(KeyEvent event)
    {
      if (event.getChar() == ' '){
        pause = !pause;
      }
    }

    Predator predator;
    vector<Prey> prey;
    int step;
    bool pause;
    float avgNND;
    float sdNND;
    int predatorVisiblePrey;
    int livePrey;

    CameraOrtho cam;
    gl::Texture bgTexture;
    Area srcArea;
    Rectf destRect;

    gl::TextureFontRef  mFont,mFontB;

    params::InterfaceGlRef params;
#ifdef SAVE_MOVIE
    qtime::MovieWriterRef  mMovieWriter;
#endif
};

void VisualizationApp::prepareSettings(Settings *settings)
{
//  settings->setFullScreen(true);
  settings->setWindowSize(AppSettings::screenWidth, AppSettings::screenHeight);
  settings->setFrameRate(10000.0f);
}

void VisualizationApp::setup()
{
  // random seed
  rnd_seed((unsigned)time(NULL));

  pause = false;
  step = 0;

  mFont = gl::TextureFont::create(Font("Source Sans Pro Light", 20 * 2), gl::TextureFont::Format().enableMipmapping());
  mFontB = gl::TextureFont::create(Font("Source Sans Pro Semibold", 20 * 2), gl::TextureFont::Format().enableMipmapping());

  if (AppSettings::usePredator)
  {
    // predator constructor
#if(PREDATOR==BASIC)
#if(PREY == DEFAULT)
	predator = Predator(0, .04f, .95f, .01f);
#endif
#if(PREY == DELAYEDRESPONSE)
    predator = Predator(0, .05f, .91f, .04f);
#endif
#if(PREY == NONCONFUSING)
	predator = Predator(0, .40f, .05f, .55f);
#endif
#endif
#if(PREDATOR==BASICNEAREST)
	predator = Predator(0, 1.0f, .0f, .0f);
#endif
#if(PREDATOR==BASICCENTRAL)
	predator = Predator(0, .0f, .0f, 1.0f);
#endif
#if(PREDATOR==BASICPERIPHERAL)
	predator = Predator(0, .0f, 1.0f, .0f);
#endif
#if(PREDATOR==DISPERSING)
#if(PREY == DEFAULT)
	predator = Predator(0, 17.5f, 111.8f);
#endif
#if(PREY == DELAYEDRESPONSE)
	predator = Predator(0, 12.4f, 113.7f);
#endif
#if(PREY == NONCONFUSING)
	predator = Predator(0, 157.6f, 157.2f);
#endif
#endif
#if(PREDATOR==RANDOM)
		predator = Predator(0);
#endif
  }

  // update
  ci::RectT<float> area(numeric_limits<float>::max(), numeric_limits<float>::max(),
                        numeric_limits<float>::lowest(), numeric_limits<float>::lowest());

  for (int j = 0; j < AppSettings::noOfPreyAnimats; j++)
  {
    Prey p = Prey(j);
    prey.push_back(p);

    area.include(Vec2f(p.position.x, p.position.y));
  }

  if (AppSettings::usePredator)
    area.include(Vec2f(predator.position.x, predator.position.y));

  if (AppSettings::usePredator)
  {
    area.x1 = math<float>::max(math<float>::min(area.x1, predator.position.x - AppSettings::huntSize*.2f), predator.position.x - AppSettings::huntSize*.6f);
    area.x2 = math<float>::min(math<float>::max(area.x2, predator.position.x + AppSettings::huntSize*.2f), predator.position.x + AppSettings::huntSize*.6f);
    area.y1 = math<float>::max(math<float>::min(area.y1, predator.position.y - AppSettings::huntSize*.2f), predator.position.y - AppSettings::huntSize*.6f);
    area.y2 = math<float>::min(math<float>::max(area.y2, predator.position.y + AppSettings::huntSize*.2f), predator.position.y + AppSettings::huntSize*.6f);
  }

  const float aR = getWindowAspectRatio();
  if (area.getAspectRatio() < aR)
    area.inflate(Vec2f((area.getHeight()*aR - area.getWidth()) / 2.f, 0.f));
  if (area.getAspectRatio() > aR)
    area.inflate(Vec2f(0.f, (area.getWidth() / aR - area.getHeight()) / 2.f));
  area.scaleCentered(1.1f);

  cam.setOrtho(area.getX1(), area.getX2(), area.getY2(), area.getY1(), 10.f, -10.f);

  // background texture
  gl::Texture::Format fmt;
  fmt.enableMipmapping(true);
  fmt.setMinFilter(GL_LINEAR_MIPMAP_LINEAR);
  fmt.setMagFilter(GL_LINEAR_MIPMAP_LINEAR);
  fmt.setWrap(GL_REPEAT, GL_REPEAT);
  ImageSourceRef isr = loadImage(loadResource(BG_PNG4, "PNG"));
  bgTexture = gl::Texture(isr, fmt);

  srcArea = Area(Vec2i(-20000, -20000), Vec2i(20000, 20000));
  destRect = Rectf(-10000.0f, -10000.0f, 10000.0f, 10000.0f);

#ifdef SAVE_MOVIE
  fs::path path = getSaveFilePath();
  qtime::MovieWriter::Format format;

  if (qtime::MovieWriter::getUserCompressionSettings(&format, loadImage(loadResource(RES_PREVIEW_IMAGE, "PNG")))) {
    mMovieWriter = qtime::MovieWriter::create(path, AppSettings::screenWidth, AppSettings::screenHeight, format);
  }
#endif
}

// search pairwise neighbours
void VisualizationApp::searchNeighbours(Predator& predator, vector<Prey>& prey)
{
  const float maxSize = math<float>::max(AppSettings::cohesionSize, AppSettings::peripheralitySize);
  const float maxSize2 = maxSize * maxSize;
  auto end = prey.end();

  // reset NND & SD
  avgNND = 0;
  sdNND = 0;
  predatorVisiblePrey = 0;
  livePrey = 0;

  if (AppSettings::usePredator)
  {
    predator.maxPreyDist = 0.f;
    predator.nearestPrey = -1;
    predator.minPreyDist = AppSettings::huntSize;
  }
  for (auto a = prey.begin(); a != end; ++a)
  {
    if (!a->isDead)
    {
      livePrey++;
      if (a->nnd != -1)
        avgNND += a->nnd;
      
      float preyDist = glm::distance(predator.position, a->position) - AppSettings::preySize - AppSettings::predatorSize;
      if (preyDist < 0)
        preyDist = 0;
      a->predatorDist = preyDist;

      if (AppSettings::usePredator && preyDist < AppSettings::huntSize)
      {
        predatorVisiblePrey++;
        predator.maxPreyDist = math<float>::max(predator.maxPreyDist, preyDist);
        if ((preyDist < predator.minPreyDist) && (predator.isOnFrontSideOfSchool(*a)))
          predator.nearestPrey = a->id;
        predator.minPreyDist = math<float>::min(predator.minPreyDist, preyDist);
      }
      for (auto b = a + 1; b != end; ++b)
      {
        if (!b->isDead)
        {
          const glm::vec2 ofs = b->position - a->position;
          float const dist2 = glm::length2(ofs);
          if ((dist2 > 0.0f) && (dist2 < maxSize2))
          {
            float dist = sqrt(dist2);
            const glm::vec2 dir = ofs / dist;
            dist -= (2 * AppSettings::preySize);
            if (dist < 0)
              dist = .0f;
            if (dist < maxSize)
            {
              a->neighbours.emplace_back(ofs, dir, b->getVelocity(), dist);
              b->neighbours.emplace_back(-ofs, -dir, a->getVelocity(), dist);
            }
          }
        }
      }
    }
  }
  avgNND /= livePrey;
  avgNND /= AppSettings::preySize * 2;
}

void VisualizationApp::update()
{
  if (pause)
    return;

  // calculate
  // prey
  searchNeighbours(predator, prey);
  for(vector<Prey>::iterator p = prey.begin(); p != prey.end(); ++p)
  {
    if (!p->isDead)
      p->calculate(predator, prey);
  }
  // predator
  if (AppSettings::usePredator)
  {
    predator.calculate(prey);
  }
  // update
  ci::RectT<float> area(numeric_limits<float>::max(), numeric_limits<float>::max(), 
                        numeric_limits<float>::lowest(), numeric_limits<float>::lowest());
  glm::vec2 center(0.f,0.f);
  // prey
  searchNeighbours(predator, prey);
  for (Prey &p : prey)
  {
    if (!p.isDead)
    {
      if (p.nnd != -1)
        sdNND += (p.nnd / AppSettings::preySize*2.f - avgNND)*(p.nnd / AppSettings::preySize*2.f - avgNND);

      p.update(predator);

      center += p.position;
      area.include(Vec2f(p.position.x, p.position.y));
    }
  }
  sdNND /= livePrey;
  sdNND = sqrt(sdNND);
  center /= std::max(livePrey, 1);

  // predator
  if (AppSettings::usePredator)
  {
    predator.update();
    area.include(Vec2f(predator.position.x, predator.position.y));
  }

#if 1
  if (AppSettings::usePredator)
  {
    area.x1 = math<float>::max(math<float>::min(area.x1, predator.position.x - AppSettings::huntSize*.2f), predator.position.x - AppSettings::huntSize*.6f);
    area.x2 = math<float>::min(math<float>::max(area.x2, predator.position.x + AppSettings::huntSize*.2f), predator.position.x + AppSettings::huntSize*.6f);
    area.y1 = math<float>::max(math<float>::min(area.y1, predator.position.y - AppSettings::huntSize*.2f), predator.position.y - AppSettings::huntSize*.6f);
    area.y2 = math<float>::min(math<float>::max(area.y2, predator.position.y + AppSettings::huntSize*.2f), predator.position.y + AppSettings::huntSize*.6f);
  }
#else
  area.x1 = math<float>::max(math<float>::min(area.x1, center.x - AppSettings::huntSize*.2f), center.x - AppSettings::huntSize*.6f);
  area.x2 = math<float>::min(math<float>::max(area.x2, center.x + AppSettings::huntSize*.2f), center.x + AppSettings::huntSize*.6f);
  area.y1 = math<float>::max(math<float>::min(area.y1, center.y - AppSettings::huntSize*.2f), center.y - AppSettings::huntSize*.6f);
  area.y2 = math<float>::min(math<float>::max(area.y2, center.y + AppSettings::huntSize*.2f), center.y + AppSettings::huntSize*.6f);
#endif

  const float aR = getWindowAspectRatio();
  if (area.getAspectRatio() < aR)
    area.inflate(Vec2f((area.getHeight()*aR - area.getWidth()) / 2.f, 0.f));
  if (area.getAspectRatio() > aR)
    area.inflate(Vec2f(0.f, (area.getWidth() / aR - area.getHeight()) / 2.f));
  area.scaleCentered(1.1f);

  cam.setOrtho(area.getX1(), area.getX2(), area.getY2(), area.getY1(), 10.f, -10.f);

  step++;
}

void VisualizationApp::draw()
{
  gl::setMatrices(cam);

  if (step <= AppSettings::noOfSteps)
  {
#if 0 // set to 1 for hand-drawn grid (10 unit grid)
    // clear the window
    gl::clear(Color::gray(.90f), true);

    gl::lineWidth(.5f);
    gl::color(Color::gray(.75f));
    for (int i = 0; i < 1000; i++)
    {
      gl::drawLine(Vec2f(-i * 10.f, -20000.f), Vec2f(-i * 10.f, 20000.f));
      gl::drawLine(Vec2f(i * 10.f, -20000.f), Vec2f(i * 10.f, 20000.f));
      gl::drawLine(Vec2f(-20000.f, -i * 10.f), Vec2f(20000.f, -i * 10.f));
      gl::drawLine(Vec2f(-20000.f, i * 10.f), Vec2f(20000.f, i * 10.f));
    }
    gl::lineWidth(2.0f);
    gl::color(Color::gray(.6f));
    for (int i = 0; i < 100; i++)
    {
      gl::drawLine(Vec2f(-i * 100.f, -20000.f), Vec2f(-i * 100.f, 20000.f));
      gl::drawLine(Vec2f(i * 100.f, -20000.f), Vec2f(i * 100.f, 20000.f));
      gl::drawLine(Vec2f(-20000.f, -i * 100.f), Vec2f(20000.f, -i * 100.f));
      gl::drawLine(Vec2f(-20000.f, i * 100.f), Vec2f(20000.f, i * 100.f));
    }
    gl::color(Color::white());
    gl::lineWidth(1.0f);
#else
    gl::color(Color::white());
    //    gl::draw(bgTexture, srcArea, destRect);
    gl::clear(Color::gray(.9f), true);
    gl::lineWidth(.5f);
    gl::color(Color::gray(.75f));
		/*
    for (int i = 0; i < 10; i++)
    {
      Vec3f tL, tR, bL, bR;
      cam.getNearClipCoordinates(&tL, &tR, &bL, &bR);
      Vec3f x = bR - bL;
      gl::drawLine(Vec2f(-i * 10.f, -20000.f), Vec2f(-i * 10.f, 20000.f));
      gl::drawLine(Vec2f(i * 10.f, -20000.f), Vec2f(i * 10.f, 20000.f));
      gl::drawLine(Vec2f(-20000.f, -i * 10.f), Vec2f(20000.f, -i * 10.f));
      gl::drawLine(Vec2f(-20000.f, i * 10.f), Vec2f(20000.f, i * 10.f));
    }
		*/
#endif
    if (AppSettings::usePredator)
    {
      if (predator.target != -1)
        prey[predator.target].drawDomainOfDanger();
    }

    // prey
    for (Prey &p : prey)
    {
      p.draw();
      //if (predator.isOnFrontSideOfSchool(p))
      //  gl::color(0.f, 0.f, 0.f);
      //else
      //  gl::color(1.f, 0.f, 0.f);
      //gl::drawLine(Vec2f(p.position.x, p.position.y), Vec2f(p.position.x, p.position.y) + Vec2f(p.peripheralityDir.x, p.peripheralityDir.y)*10.f);
    }

    // predator
    if (AppSettings::usePredator) 
    {
      predator.draw();

      if (predator.target != -1)
        prey[predator.target].drawTarget(predator);
#if(PREDATOR==DISPERSING)
      else
        if (predator.central != -1)
          prey[predator.central].drawTarget(predator, true);
#endif
    }

//      params->draw();

#if 1
    if (AppSettings::showGUI)
    {
      gl::enableAlphaBlending();
      gl::pushMatrices();
      gl::setProjection(CameraOrtho(0.f, (float)AppSettings::screenWidth, (float)AppSettings::screenHeight, 0.f, 10.f, -10.f));
      const float lineHeight = 20.f;
      float stringWidth = 0.f;
      float lineID = 0;
      std::stringstream s;

      s << setprecision(2);

      gl::color(Color::gray(.4f));

      s.str("");
#if(PREDATOR==BASIC)
      s.str("Attack using a mixture of simple tactics");
#endif
#if(PREDATOR==BASICNEAREST)
	  s.str("Attack exclusively the nearest prey");
#endif
#if(PREDATOR==BASICCENTRAL)
	  s.str("Attack exclusively the most central prey");
#endif
#if(PREDATOR==BASICPERIPHERAL)
	  s.str("Attack exclusively the most peripheral prey");
#endif
#if(PREDATOR==DISPERSING)
      s.str("Dispersing tactic");
#endif
#if(PREDATOR==RANDOM)
	s.str("Attack random prey");
#endif
      mFontB->drawString(s.str(), Vec2f(12, 12 + mFontB->measureString(s.str()).y * .5f),
        gl::TextureFont::DrawOptions().scale(.5f).pixelSnap(false));
#if(PREDATOR!=RANDOM)
      s.str("");
      s << (predator.handling ? "HANDLING" : "HUNTING");
      if (AppSettings::showTacticProportion)
        s << " " << predator.currentTacticStr();
      stringWidth = mFont->measureString(s.str()).x * 0.5f;
      mFont->drawString(s.str(), Vec2f(AppSettings::screenWidth - stringWidth - 12, AppSettings::screenHeight - 12 - lineID*lineHeight),
        gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
      s.str("Mode  ");
      stringWidth += mFontB->measureString(s.str()).x * 0.5f;
      mFontB->drawString(s.str(), Vec2f(AppSettings::screenWidth - stringWidth - 12, AppSettings::screenHeight - 12 - lineID*lineHeight),
        gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
      lineID++;
#endif
      s.str("");
      s << float(predator.huntCount) << "/" << float(predator.huntCount + predator.confusedCount);
      stringWidth = mFont->measureString(s.str()).x * 0.5f;
      mFont->drawString(s.str(), Vec2f(AppSettings::screenWidth - stringWidth - 12, AppSettings::screenHeight - 12 - lineID*lineHeight),
        gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
      s.str("Attacks  ");
      stringWidth += mFontB->measureString(s.str()).x * 0.5f;
      mFontB->drawString(s.str(), Vec2f(AppSettings::screenWidth - stringWidth - 12, AppSettings::screenHeight - 12 - lineID*lineHeight),
        gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
      lineID++;
#if(PREDATOR!=RANDOM)
      s.str("");
      if (AppSettings::showTacticProportion)
      {
        s << predator.tacticProportionStr();
        stringWidth = mFont->measureString(s.str()).x * 0.5f;
        mFont->drawString(s.str(), Vec2f(AppSettings::screenWidth - stringWidth - 12, AppSettings::screenHeight - 12 - lineID*lineHeight),
          gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
        s.str("Tactic  ");
        stringWidth += mFontB->measureString(s.str()).x * 0.5f;
        mFontB->drawString(s.str(), Vec2f(AppSettings::screenWidth - stringWidth - 12, AppSettings::screenHeight - 12 - lineID*lineHeight),
          gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
      }
      else
      {
        s << predator.currentTacticStr();
        stringWidth = mFont->measureString(s.str()).x * 0.5f;
        mFont->drawString(s.str(), Vec2f(AppSettings::screenWidth - stringWidth - 12, AppSettings::screenHeight - 12 - lineID*lineHeight),
          gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
        s.str("Tactic  ");
        stringWidth += mFontB->measureString(s.str()).x * 0.5f;
        mFontB->drawString(s.str(), Vec2f(AppSettings::screenWidth - stringWidth - 12, AppSettings::screenHeight - 12 - lineID*lineHeight),
          gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
        lineID++;
      }
#if(PREDATOR!=BASICNEAREST && PREDATOR!=BASICCENTRAL && PREDATOR!=BASICPERIPHERAL) 
      s.str("");
      s << predator.paramsStr();
      stringWidth = mFont->measureString(s.str()).x * 0.5f;
      mFont->drawString(s.str(), Vec2f(AppSettings::screenWidth - stringWidth - 12, AppSettings::screenHeight - 12 - lineID*lineHeight),
        gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
      s.str("Predator  ");
      stringWidth += mFontB->measureString(s.str()).x * 0.5f;
      mFontB->drawString(s.str(), Vec2f(AppSettings::screenWidth - stringWidth - 12, AppSettings::screenHeight - 12 - lineID*lineHeight),
        gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
#endif
#endif
      lineID = 0.f;
      s.str("Frame  ");
      stringWidth = mFontB->measureString(s.str()).x * 0.5f;
      mFontB->drawString(s.str(), Vec2f(12, AppSettings::screenHeight - 12 - lineID*lineHeight),
        gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
      s.str("");
      s << step << "/" << AppSettings::noOfSteps;
      mFont->drawString(s.str(), Vec2f(12 + stringWidth, AppSettings::screenHeight - 12 - lineID*lineHeight),
        gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
      lineID += 2.f;

	  s.str("");
	  s.str("Prey velocity ");
	  stringWidth = mFontB->measureString(s.str()).x * 0.5f;
	  mFontB->drawString(s.str(), Vec2f(12, AppSettings::screenHeight - 12 - lineID*lineHeight),
		  gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
	  s.str("");
	  s << setprecision(4) << std::sqrt(glm::length2(prey[0].getVelocity()));
	  mFont->drawString(s.str(), Vec2f(12 + stringWidth, AppSettings::screenHeight - 12 - lineID*lineHeight),
		  gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
	  lineID++;

	  s.str("");
	  s.str("Prey collisions ");
	  stringWidth = mFontB->measureString(s.str()).x * 0.5f;
	  mFontB->drawString(s.str(), Vec2f(12, AppSettings::screenHeight - 12 - lineID*lineHeight),
		  gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
	  s.str("");
	  s << setprecision(4) << prey[0].neighbours.size();
	  mFont->drawString(s.str(), Vec2f(12 + stringWidth, AppSettings::screenHeight - 12 - lineID*lineHeight),
		  gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
	  lineID++;

	  s.str("");
	  s.str("Prey energy ");
	  stringWidth = mFontB->measureString(s.str()).x * 0.5f;
	  mFontB->drawString(s.str(), Vec2f(12, AppSettings::screenHeight - 12 - lineID*lineHeight),
		  gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
	  s.str("");
	  s << setprecision(4) << prey[0].energy;
	  mFont->drawString(s.str(), Vec2f(12 + stringWidth, AppSettings::screenHeight - 12 - lineID*lineHeight),
		  gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
	  lineID++;

	  s.str("");
	  s.str("Predator energy ");
	  stringWidth = mFontB->measureString(s.str()).x * 0.5f;
	  mFontB->drawString(s.str(), Vec2f(12, AppSettings::screenHeight - 12 - lineID*lineHeight),
		  gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
	  s.str("");
	  s << setprecision(4) << predator.energy; 
	  mFont->drawString(s.str(), Vec2f(12 + stringWidth, AppSettings::screenHeight - 12 - lineID*lineHeight),
		  gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
	  lineID++;

	  s.str("");
	  s.str("Predator angle ");
	  stringWidth = mFontB->measureString(s.str()).x * 0.5f;
	  mFontB->drawString(s.str(), Vec2f(12, AppSettings::screenHeight - 12 - lineID*lineHeight),
		  gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
	  s.str("");
	  s << predator.angle_t << " " << predator.prevAngle_t;
	  mFont->drawString(s.str(), Vec2f(12 + stringWidth, AppSettings::screenHeight - 12 - lineID*lineHeight),
		  gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
	  lineID++;

      s.str("");
      s.str("NND  ");
      stringWidth = mFontB->measureString(s.str()).x * 0.5f;
      mFontB->drawString(s.str(), Vec2f(12, AppSettings::screenHeight - 12 - lineID*lineHeight),
        gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
      s.str("");
      s << avgNND << " (sd=" << sdNND << ")"; // to do show the ± symbol ;)
      mFont->drawString(s.str(), Vec2f(12 + stringWidth, AppSettings::screenHeight - 12 - lineID*lineHeight),
        gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
      lineID++;

      s.str("");
      s.str("Prey  ");
      stringWidth = mFontB->measureString(s.str()).x * 0.5f;
      mFontB->drawString(s.str(), Vec2f(12, AppSettings::screenHeight - 12 - lineID*lineHeight),
        gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
      s.str("");
      s << predatorVisiblePrey << "/" << livePrey;
      mFont->drawString(s.str(), Vec2f(12 + stringWidth, AppSettings::screenHeight - 12 - lineID*lineHeight),
        gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
      lineID++;

      lineID++;
#if(PREDATOR!=RANDOM)
      float rW = mFontB->measureString("Risk").x * 0.5f;
      Rectf r = Rectf(12, AppSettings::screenHeight - 12 - lineID*lineHeight, 12 + rW, AppSettings::screenHeight - 12 - lineID*lineHeight - rW);
      for (auto i = 0; i < (int)AppSettings::riskColor.size(); i++)
      {
        gl::color(AppSettings::riskColor[i]);
        gl::drawSolidRect(r);
        r.offset(Vec2f(0.f, -rW));
        if (i < (int)AppSettings::riskLevels.size())
        {
          gl::color(Color::gray(.4f));
          s.str("");
          s << AppSettings::riskLevels[i];
          mFont->drawString(s.str(), Vec2f(12 + 6 + rW, AppSettings::screenHeight - 12 - lineID*lineHeight - 18 - rW*i),
            gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
        }
      }
      gl::color(Color::gray(.4f));
      s.str("Risk");
      mFontB->drawString(s.str(), Vec2f(12, AppSettings::screenHeight - 12 - lineID*lineHeight - rW * AppSettings::riskColor.size() - 6),
        gl::TextureFont::DrawOptions().scale(0.5f).pixelSnap(false));
#endif
      gl::popMatrices();
      gl::disableAlphaBlending();
    }
#endif

#ifdef SAVE_MOVIE
    if (mMovieWriter)
      mMovieWriter->addFrame(copyWindowSurface());
#endif

#ifdef SAVE_SS
	writeImage(fs::initial_path() / "images" / (toString(step) + ".png"), copyWindowSurface());
	//writeImage(getHomeDirectory() / "images" / "saveImage_" / (toString(step) + ".png"), copyWindowSurface());
#endif
  }
  else {
    VisualizationApp::quit();
  }
}

CINDER_APP_NATIVE(VisualizationApp, RendererGl)