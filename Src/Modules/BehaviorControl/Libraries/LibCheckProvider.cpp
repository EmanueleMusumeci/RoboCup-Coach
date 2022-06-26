/**
 * @file LibCheckProvider.cpp
 *
 * Performs some checks of the behavior control module such as
 * - How often is a MotionRequest set?
 * - How often is a HeadMotionRequest set?
 *
 * @author Tim Laue
 */
#include "Platform/Nao/SoundPlayer.h"
#include "LibCheckProvider.h"
#include <math.h>
#include <iostream>
#include <algorithm>
#define SQ(x) x*x

MAKE_MODULE(LibCheckProvider, behaviorControl);

int sign(float n){
  if(n > 0){
    return 1;
  }else if(n == 0){
    return 0;
  }else{
    return -1;
  }
}

LibCheckProvider::LibCheckProvider() {}

void LibCheckProvider::update(LibCheck& libCheck)
{
  reset();
  libCheck.timeSinceBallWasSeen = theFrameInfo.getTimeSince(theTeamBallModel.timeWhenLastSeen);
  libCheck.angleToBall = (theRobotPose.inversePose * theFieldBall.positionOnField).angle();
  
  libCheck.angleBetweenPoints = [&](Vector2f p1, Vector2f p2) -> float 
  {
    double deltaY = p1.y() - p2.y();
    double deltaX = p2.x() - p1.x();
    return atan2(deltaY, deltaX);
  };

  libCheck.norm = [&](float x, float y) -> float
  {
    return (float)(sqrt((x*x) + (y*y)));
  };

  libCheck.inc = [this](LibCheck::CheckedOutput outputToCheck) {inc(outputToCheck);};

  libCheck.setArm = [this](Arms::Arm arm)
  {
    setArmsInThisFrame[arm] = true;
  };

  libCheck.wasSetArm = [this](Arms::Arm arm) -> bool
  {
    return setArmsInThisFrame[arm];
  };

  libCheck.performCheck = [this](const MotionRequest& theMotionRequest)
  {
    checkOutputs(theActivationGraph, static_cast<LibCheck::CheckedOutput>(0), LibCheck::firstTeamCheckedOutput);
    checkMotionRequest(theActivationGraph, theMotionRequest);
  };

  libCheck.performTeamCheck = [this]()
  {
    checkOutputs(theTeamActivationGraph, LibCheck::firstTeamCheckedOutput, LibCheck::numOfCheckedOutputs);
  };

  libCheck.myReadyPosition = [this]() -> Pose2f
  {

    Pose2f strikerPose = Pose2f(0.f, -1000.f, 0.f);
    if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber){
      strikerPose = Pose2f(0.f, -500.f, 0.f);
    }else{
      strikerPose = Pose2f(0.f, -1000.f, 0.f);
    }

    Pose2f goaliePose = Pose2f(0.f, theFieldDimensions.xPosOwnGroundline + 200.f, 0.f);
    Pose2f defenderPose = Pose2f(0.f, theFieldDimensions.xPosOwnGroundline + 1000.f, -1000.f);
    Pose2f jollyPose = Pose2f(0.f, -500.f, -1500.f);
    Pose2f supporterPose = Pose2f(0.f, theFieldDimensions.xPosOwnGroundline + 1700.f, 800.f);
    int activeTeam = 0;
    int lowerNumbers = 0;
    if(theRobotInfo.penalty == PENALTY_NONE){
      activeTeam++;
    }
    if(theRobotInfo.number == 1){
        return goaliePose;
    }
    for( auto teammate : theTeamData.teammates ){
      if(teammate.status != Teammate::PENALIZED && teammate.number != 1){
        activeTeam ++;
        if(teammate.number < theRobotInfo.number){
          lowerNumbers ++;
        }
      }
    }
    switch(activeTeam){
      case 0: return Pose2f(0.f,0.f,0.f); break;

      case 1: return strikerPose;
              break;

      case 2: if(lowerNumbers <= 0){
                return defenderPose;
              }else{
                return strikerPose;
              }
              break;

      case 3: if(lowerNumbers == 2){
                  return strikerPose;
                }else if(lowerNumbers == 1){
                  return defenderPose;
                }else if(lowerNumbers == 0){
                  return supporterPose;
                }
                break;

      case 4: if(theRobotInfo.number == 3){
                return strikerPose;
              }else if(theRobotInfo.number == 2){
                return supporterPose;
              }else if (theRobotInfo.number == 5){
                return defenderPose;
              }else if(theRobotInfo.number == 4){
                return jollyPose;
              }
    }

    return strikerPose;
  };

  libCheck.nearestTemmate = [&]() -> Pose2f
  {
      Pose2f nearest = Pose2f(4500.f,0.f);
      for(const auto& mate : theTeamData.teammates){
              if( (mate.theRobotPose.translation- theRobotPose.translation).norm() < 500.f) {
                  nearest = mate.theRobotPose.translation;
              }
          }
      return nearest;
  };

  libCheck.distance = [this](const Pose2f p1, const Pose2f p2) -> float{
    return static_cast<float>( std::sqrt( std::pow(p2.translation.x() - p1.translation.x(), 2) +
      std::pow(p2.translation.y() - p1.translation.y(), 2) ) );
  };

  /*
  @author Emanuele Musumeci
  Dummy goal target
  */
  libCheck.goalTarget = [this]() -> Vector2f {
    return Vector2f(theFieldDimensions.xPosOpponentGoal, 0);
  };
  

  libCheck.glob2RelWithAngle = [&](float theta, float x, float y) -> Pose2f
  {
      Vector2f result;
      float tempX = x - theRobotPose.translation.x();
      float tempY = y - theRobotPose.translation.y();

      result.x() = (float)(tempX * cos(theRobotPose.rotation) + tempY * sin(theRobotPose.rotation));
      result.y() = (float)(-tempX * sin(theRobotPose.rotation) + tempY * cos(theRobotPose.rotation));

      return Pose2f(theta , result.x(),result.y());
  };

  libCheck.glob2Rel = [&](float x, float y) -> Pose2f
  {
      Vector2f result;
      float tempX = x - theRobotPose.translation.x();
      float tempY = y - theRobotPose.translation.y();

      result.x() = (float)(tempX * cos(theRobotPose.rotation) + tempY * sin(theRobotPose.rotation));
      result.y() = (float)(-tempX * sin(theRobotPose.rotation) + tempY * cos(theRobotPose.rotation));

      return Pose2f(result.x(),result.y());
  };


  libCheck.rel2Glob = [&](float x, float y) -> Pose2f
  {
      Vector2f result;
      float rho = (float)(sqrt((x * x) + (y * y)));

      result.x() = (float)(theRobotPose.translation.x() + (rho * cos(theRobotPose.rotation + atan2(y,x))));
      result.y() = (float)(theRobotPose.translation.y() + (rho * sin(theRobotPose.rotation + atan2(y,x))));

      return Pose2f(result.x(),result.y());
  };


  libCheck.angleToTarget = [&](float x, float y) -> float
  {
    //gets the relative position of the point to go for the robot
    Pose2f relativePosition = glob2Rel(x,y);
    return (atan2f(relativePosition.translation.y(), relativePosition.translation.x()));
  };


  //DUMMY POSITION
  libCheck.getSupporterPosition = [&] () -> Vector2f
  {
    return Vector2f(-3000, 3000);
  };

  //DUMMY POSITION
  libCheck.getDefenderPosition = [&] () -> Vector2f
  {
    return Vector2f(-3000, 0);
  };

  //DUMMY jolly position
  libCheck.getJollyPosition = [&] () -> Vector2f
  {
    // to return
    return Vector2f(-3000,-3000);
  };

  libCheck.radiansToDegree = [&](float x) -> float
  {
    return (float)((x*180)/3.14159265358979323846);
  };


  libCheck.distanceToLine = [&] (Vector2f objectToCheck, Vector2f linePoint1, Vector2f linePoint2) -> float
  {
    return std::abs(((linePoint2.y()-linePoint1.y())*objectToCheck.x()) - ((linePoint2.x() - linePoint1.x()) * objectToCheck.y())
    + (linePoint2.x() * linePoint1.y()) - (linePoint2.y() * linePoint1.x())) / ((linePoint1-linePoint2).norm());
  };
}


void LibCheckProvider::reset()
{
  FOREACH_ENUM(LibCheck::CheckedOutput, i)
    callCounters[i] = 0;

  for(int i = 0; i < Arms::numOfArms; ++i)
    setArmsInThisFrame[i] = false;
}

void LibCheckProvider::checkOutputs(const ActivationGraph& activationGraph, LibCheck::CheckedOutput start, LibCheck::CheckedOutput end) const
{
  const std::string options = getActivationGraphString(activationGraph);

  // Output counting checks:
  for(LibCheck::CheckedOutput i = start; i < end; i = static_cast<LibCheck::CheckedOutput>(static_cast<unsigned>(i) + 1))
  {
    // Check, if output has been set at least once:
    if(callCounters[i] == 0 && notSetCheck[i] == 1)
    {
      OUTPUT_TEXT("Meeek! " << TypeRegistry::getEnumName(i) << " has not been set in this cycle (Robot " << theRobotInfo.number
                  << (!callCounters[LibCheck::role] ? "" : ", Role: " + theTeamBehaviorStatus.role.getName())
                  << (options == "" ? "" : ", Options: " + options) << ") !");
    }
    else if(notSetCheck[i] == 2)
    {
      ASSERT(callCounters[i] > 0);
    }
  }
}


void LibCheckProvider::checkMotionRequest(const ActivationGraph& activationGraph, const MotionRequest& theMotionRequest) const
{
  // Check for invalid motion request:
  if(assertValidWalkRequest &&
     theMotionRequest.motion == MotionRequest::walk &&
     !theMotionRequest.walkRequest.isValid())
  {
#ifndef NDEBUG
    {
      std::string logDir = "";
#ifdef TARGET_ROBOT
      logDir = "../logs/";
#endif
      OutMapFile stream(logDir + "walkRequest.log");
      stream << theMotionRequest.walkRequest;
      stream << getActivationGraphString(activationGraph);
    }
#endif
    FAIL("Motion request is not valid (see walkRequest.log).");
  }
}

void LibCheckProvider::inc(LibCheck::CheckedOutput outputToCheck)
{
  const int index = static_cast<int>(outputToCheck);
  if(index >= 0 && index < LibCheck::numOfCheckedOutputs)
  {
    ++callCounters[index];

    // Check, if output has not been set more than once:
    if(callCounters[index] > 1)
    {
      if(multipleSetCheck[index] == 1)
      {
        const std::string options = getActivationGraphString(index >= LibCheck::firstTeamCheckedOutput ? theTeamActivationGraph : theActivationGraph);

        OUTPUT_TEXT("Meeek! " << TypeRegistry::getEnumName(static_cast<LibCheck::CheckedOutput>(index)) << " has been set more than once in this cycle (Robot "
                    << theRobotInfo.number
                    << (!callCounters[LibCheck::role] ? "" : ", Role: " + theTeamBehaviorStatus.role.getName())
                    << (options == "" ? "" : ", Options: " + options) << ") !");
      }
      else if(multipleSetCheck[index] == 2)
      {
        ASSERT(callCounters[index] <= 1);
      }
    }
  }
}

Pose2f LibCheckProvider::myReadyPosition() const{
  Pose2f strikerPose = Pose2f(0.f, -1000.f, 0.f);
  if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber){
    strikerPose = Pose2f(0.f, -500.f, 0.f);
  }else{
    strikerPose = Pose2f(0.f, -1000.f, 0.f);
  }

  Pose2f goaliePose = Pose2f(0.f, theFieldDimensions.xPosOwnGroundline + 200.f, 0.f);
  Pose2f defenderPose = Pose2f(0.f, theFieldDimensions.xPosOwnGroundline + 1000.f, -1000.f);
  Pose2f jollyPose = Pose2f(0.f, -500.f, -1500.f);
  Pose2f supporterPose = Pose2f(0.f, theFieldDimensions.xPosOwnGroundline + 1700.f, 800.f);
  int activeTeam = 0;
  int lowerNumbers = 0;
  if(theRobotInfo.penalty == PENALTY_NONE){
    activeTeam++;
  }
  if(theRobotInfo.number == 1){
      return goaliePose;
  }
  for( auto teammate : theTeamData.teammates ){
    if(teammate.status != Teammate::PENALIZED && teammate.number != 1){
      activeTeam ++;
      if(teammate.number < theRobotInfo.number){
        lowerNumbers ++;
      }
    }
  }
  switch(activeTeam){
    case 0: return Pose2f(0.f,0.f,0.f); break;

    case 1: return strikerPose;
            break;

    case 2: if(lowerNumbers <= 0){
              return defenderPose;
            }else{
              return strikerPose;
            }
            break;

    case 3: if(lowerNumbers == 2){
                return strikerPose;
              }else if(lowerNumbers == 1){
                return defenderPose;
              }else if(lowerNumbers == 0){
                return supporterPose;
              }
              break;

    case 4: if(theRobotInfo.number == 3){
              return strikerPose;
            }else if(theRobotInfo.number == 2){
              return supporterPose;
            }else if (theRobotInfo.number == 5){
              return defenderPose;
            }else if(theRobotInfo.number == 4){
              return jollyPose;
            }
  }

  return strikerPose;

}

float LibCheckProvider::distance(float x1, float y1, float x2, float y2) const{

  return sqrt(pow(x1-x2,2)+pow(y1-y2,2));

}

float LibCheckProvider::distance(Pose2f p1, Pose2f p2) const{

  return static_cast<float>( std::sqrt( std::pow(p2.translation.x() - p1.translation.x(), 2) +
    std::pow(p2.translation.y() - p1.translation.y(), 2) ) );

}

std::string LibCheckProvider::getActivationGraphString(const ActivationGraph& activationGraph) const
{
  std::string options = "";
  for(const auto& node : activationGraph.graph)
    options += (options == "" ? "" : ", ") + node.option + (node.state == "" ? "" : "/" + node.state);
  return options;
}

float LibCheckProvider::mapToInterval(float value, float fromIntervalMin, float fromIntervalMax, float toIntervalMin, float toIntervalMax) {
  float fromIntervalSize = fromIntervalMax - fromIntervalMin;
  float toIntervalSize = toIntervalMax - toIntervalMin;
  if(value >= fromIntervalMax)
  {
    //std::cout<<"value ("<<value<<") >= fromIntervalMax ("<<fromIntervalMax<<")"<<std::endl;
    return toIntervalMax;
  }
  else if (value <= fromIntervalMin)
  {
    //std::cout<<"value ("<<value<<") <= fromIntervalMin ("<<fromIntervalMin<<")"<<std::endl;
    return toIntervalMin;
  }
  else
  {
    //std::cout<<"toIntervalMin + (value - fromIntervalMin) * toIntervalSize / fromIntervalSize: " << toIntervalMin + (value - fromIntervalMin) * toIntervalSize / fromIntervalSize<<std::endl;
    return toIntervalMin + (value - fromIntervalMin) * toIntervalSize / fromIntervalSize;
  }
}


Pose2f LibCheckProvider::glob2Rel(float x, float y)
{
    Vector2f result;
    float theta = 0;
    float tempX = x - theRobotPose.translation.x();
    float tempY = y - theRobotPose.translation.y();

    result.x() = (float)(tempX * cos(theRobotPose.rotation) + tempY * sin(theRobotPose.rotation));
    result.y() = (float)(-tempX * sin(theRobotPose.rotation) + tempY * cos(theRobotPose.rotation));

    return Pose2f(theta /*deg*/, result.x(),result.y());
}

Pose2f LibCheckProvider::rel2Glob(float x, float y)
{
    Vector2f result;
    float rho = (float)(sqrt((x * x) + (y * y)));

    result.x() = (float)(theRobotPose.translation.x() + (rho * cos(theRobotPose.rotation + atan2(y,x))));
    result.y() = (float)(theRobotPose.translation.y() + (rho * sin(theRobotPose.rotation + atan2(y,x))));

    return Pose2f(result.x(),result.y());
}
float LibCheckProvider::radiansToDegree(float x)
{
  return (float)((x*180)/3.14159265358979323846);
}