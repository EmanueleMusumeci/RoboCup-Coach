/**
 * @file ApproachAndCarryWithTwoStepRealignmentCard.cpp
 *
 * This file implements a behavior to carry the ball forward to a certain target in the field, avoiding obstacles.
 *
 * @author Emanuele Musumeci
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/BallCarrierModel/BallCarrierModel.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"

#include "Representations/BehaviorControl/TasksProvider/TaskController.h"

#include "Representations/Communication/RobotInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"


#include "Tools/Math/BHMath.h"
#include "Platform/SystemCall.h"
#include <string>


#define __STRINGIFY_I(arg) #arg
#define __STRINGIFY(arg) __STRINGIFY_I(arg)
#define DEBUG_CODE(code) \
  std::cout<<"[Robot #"<<std::to_string(theRobotInfo.number)<<"] "<<__STRINGIFY(code)<<": "<<std::to_string(code)<<std::endl;


CARD(ApproachAndCarryWithTwoStepRealignmentCard,
{,
  CALLS(Stand),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTargetPathPlanner),
  CALLS(WalkToApproach),
  CALLS(Kick),
  CALLS(InWalkKick),
  CALLS(Approacher2021),
  CALLS(Approacher2021WithRanges),
  CALLS(Activity),

  REQUIRES(RobotPose),
  REQUIRES(RobotInfo),
  REQUIRES(BallModel),
  REQUIRES(LibCheck),
  REQUIRES(BallCarrierModel),
  REQUIRES(FrameInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(TaskController),
  REQUIRES(FieldDimensions),

  DEFINES_PARAMETERS(
  {,
    (int)(50) initialWaitTime,
    (int)(2500) ballNotSeenTimeout,
    (float)(300.f) posStartApprX,         // Offset on x axis respect to ball to start approach
    (Rangef)({-28.f, 28.f}) reachedRange, // Range to test if a position is reached
    (float)(360.f) tooFarValueX,                // Value on x axis to detect if the ball is too far to approach
    (float)(100.f) tooFarValueY,                // Value on y axis to detect if the ball is too far to approach
    (float)(10) minKickWaitTime,
    (float)(3000) maxKickWaitTime,

    (float)(145.f) approachOffsetX,
    (float)(-55.f) approachOffsetY,

    (bool)(true) USE_DEBUG_MESSAGES,
  }),
});

class ApproachAndCarryWithTwoStepRealignmentCard : public ApproachAndCarryWithTwoStepRealignmentCardBase
{

  bool targetChosen = false;

  //To avoid oscillation in front of the goal
  bool footChosen = false;
  bool kickWithRightFoot = true;


  Vector2f chosenTarget;
  Vector2f goalTarget;
  Vector2f previousBallPosition;
  int previousNearbyObstaclesCount = 0;
  Vector2f nearestObstacleGlobalPositionWhenPathWasComputed = getNearestObstacle(true);
  

  //Ball searcher
  bool isSearchingToggle = false;
  float rotationVel;
  Angle startingRobotAngleWhenSearching;
  bool fullLoopRevolution = false;

  bool preconditions() const override
  {
    //std::cout<<"theTaskController.getCurrentActionType(): "<<TypeRegistry::getEnumName(theTaskController.getCurrentActionType())<<std::endl;
    return theTaskController.getCurrentActionType() == HRI::ActionType::CarryBall
          &&
          theLibCheck.distance(theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()), theTaskController.currentBallDestination) > theTaskController.ballCarrierDistanceThreshold;
  }

  bool postconditions() const override
  {
    DEBUG_CODE(theTaskController.ballCarrierDistanceThreshold)
    DEBUG_CODE(theTaskController.currentBallDestination.x())
    DEBUG_CODE(theTaskController.currentBallDestination.y())
    DEBUG_CODE(theLibCheck.distance(theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()), theTaskController.currentBallDestination))
    return theTaskController.getCurrentActionType() != HRI::ActionType::CarryBall
          ||
          theLibCheck.distance(theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()), theTaskController.currentBallDestination) <= theTaskController.ballCarrierDistanceThreshold;
  }

  option
  {
    theActivitySkill(BehaviorStatus::approachAndCarry);
    bool first_time = true;
    Vector2f chosenTarget = Vector2f(4500.f, 0.f);
    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
          goto walkToBall;
      }

      action
      {
        first_time = true;
        theLookForwardSkill();
        theStandSkill();
      }
    }


    state(walkToBall)
    {
      transition
      {
        //if (theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > ballNotSeenTimeout)
        //  goto searchForBall;

        
      }

      action
      {
        /*theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(),
                                     theFieldBall.positionRelative.y(),
                                       0.f));*/
        if(first_time){
          chosenTarget = theBallCarrierModel.dynamicTarget.translation;
          first_time = false;
        }
        if(theRobotPose.translation.x() > 3000.f){
          chosenTarget = Vector2f(5000.f, std::max(-700.f,std::min(700.f, theRobotPose.translation.y())));
          
        }
        theApproacher2021WithRangesSkill(chosenTarget, 350.f, 180.f, 55.f, true, 0.8f, 0.7f, 0.5f, Rangef(100, 450), Rangef(-70, 70), Rangef(-10, 10));
        }
    }


  

    state(searchForBall)
    {
      transition
      {
        if (theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 500)
          goto walkToBall;
      }

      action
      {
        first_time = true;
        theLookForwardSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(0.8f, 0.f, 0.f));
      }
    }

  }

  Vector2f getNearestObstacle(bool global = false) const
  {
    Vector2f nearestObstacle;
    float nearestObstacleDistance = -1;
    for(auto obs : theObstacleModel.obstacles)
    {
      float currentObsDistance = obs.center.norm();
      if(nearestObstacleDistance == -1 || obs.center.norm() < nearestObstacleDistance)
      {
          nearestObstacle = Vector2f(obs.center.x(), obs.center.y());
          nearestObstacleDistance = currentObsDistance;     
      }
    }

    if(global) 
      return theLibCheck.rel2Glob(nearestObstacle.x(), nearestObstacle.y()).translation;
    else 
      return nearestObstacle;
  }
  
  float decideWalkSpeed(float minObstacleRadius, float maxObstacleRadius, float minWalkSpeed, float maxWalkSpeed)
  {
    float distanceFromNearestObs = theLibCheck.distance(theRobotPose.translation, getNearestObstacle(true));
    if(distanceFromNearestObs > maxObstacleRadius)
    {
      return maxWalkSpeed;
    }
    else if(distanceFromNearestObs < maxObstacleRadius && distanceFromNearestObs > minObstacleRadius)
    {
      return theLibCheck.mapToInterval(distanceFromNearestObs - minObstacleRadius, minObstacleRadius, maxObstacleRadius, minWalkSpeed, maxWalkSpeed);
    }
    else
    {
      return minWalkSpeed;
    }
  }


  bool isRightFootUsedToKick()
  {
    if(getNearestObstacle().y() < -300.f && kickWithRightFoot)
    {
      return false;
    }
    else if(getNearestObstacle().y() > 300.f && !kickWithRightFoot)
    {
      return true;
    }
    else
    {
      return kickWithRightFoot;
    }
  }

  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }

  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }

  int countNearbyObstacles(float distanceRadius) const
  {
    int obstacleCount = 0;
    for(auto obs : theObstacleModel.obstacles)
    {
      if(theLibCheck.distance(obs.center, theRobotPose) < distanceRadius)
      {
        obstacleCount++;
      }
    }  

    return obstacleCount;
  }

  Vector2f chooseGoalTarget()
  {
    bool useHeuristic = true;
    
    Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

    if(globalBall.x() > theFieldDimensions.xPosOpponentGroundline - theBallCarrierModel.xRangeDistanceFromGoalToUseKicks.max)
    {
      useHeuristic = false;
    }

    return theLibCheck.goalTarget();
  }

  bool isRobotInCenterCircle()
  {
    return theRobotPose.translation.norm() < theFieldDimensions.centerCircleRadius;
  }

  /*bool useLongKick()
  {
    return false;
  }*/

};

MAKE_CARD(ApproachAndCarryWithTwoStepRealignmentCard);
