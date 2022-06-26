/**
 * @file ReachPosition.cpp
 *
 * This file implements a basic behavior for the robot to reach a certain position on the field
 *
 * @author Emanuele Musumeci
 */

#include "Platform/Linux/SoundPlayer.h"

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"

#include "Representations/BehaviorControl/TasksProvider/TaskController.h"

#include "Tools/Math/BHMath.h"
#include "Platform/SystemCall.h"
#include <string>

CARD(ReachPositionCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(Stand),
  CALLS(WalkToTargetPathPlanner),
  CALLS(WalkToTarget),
  CALLS(GoalTarget),
  CALLS(SetTarget),
  CALLS(KeyFrameArms),
  CALLS(LookLeftAndRight),
  
  CALLS(TurnToTargetThenTurnToUserThenPointAndSaySomething),

  REQUIRES(LibCheck),
  REQUIRES(RobotPose),

  REQUIRES(TaskController),

  USES(BehaviorStatus),

  LOADS_PARAMETERS(
  {,
    (float) walkSpeed,
    
    (int) initialWaitTime,
    (int) realignmentTimeout,
    (int) waitForSoundToStartPlaying,
    (int) waitBeforePlayingSound,

    (bool) DEBUG_MODE,
  }),
});

class ReachPositionCard : public ReachPositionCardBase
{

  bool soundPlaying = false;

  bool preconditions() const override
  {
    //std::cout<<"theTaskController.getCurrentActionType(): "<<TypeRegistry::getEnumName(theTaskController.getCurrentActionType())<<std::endl;
    return (theTaskController.getCurrentActionType() == HRI::ActionType::ReachPosition || theTaskController.getCurrentActionType() == HRI::ActionType::ReachBall)
          &&
          theLibCheck.distance(theRobotPose, theTaskController.currentRobotDestination) > theTaskController.reachPositionDistanceThreshold;
  }

  bool postconditions() const override
  {
    return (theTaskController.getCurrentActionType() != HRI::ActionType::ReachPosition && theTaskController.getCurrentActionType() != HRI::ActionType::ReachBall)
          ||
          theLibCheck.distance(theRobotPose, theTaskController.currentRobotDestination) <= theTaskController.reachPositionDistanceThreshold;
  }

  option
  {

    initial_state(start)
    {
      std::cout<<"REACH_POSITION: start"<<std::endl;
      transition
      {
        if(state_time > initialWaitTime)
        {
          if(theTaskController.interactWithUser)
          {
            std::cout<<"start -> interactWithHuman: TIMEOUT"<<std::endl;
            goto interactWithHuman;
          }
          else
          {
            goto walkToPosition;
          }
        }
      }

      action
      {
        theActivitySkill(BehaviorStatus::reaching_position);
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(debug_state)
    {
      transition
      {}

      action
      {
        theActivitySkill(BehaviorStatus::debug_standing);
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(interactWithHuman)
    {
      transition
      {
        if(theTurnToTargetThenTurnToUserThenPointAndSaySomethingSkill.isDone())
        {
          std::cout<<"speakToHuman -> stand: sound finished playing"<<std::endl;
          goto walkToPosition;
        }
      }

      action
      {
        if(theTaskController.getCurrentActionType() == HRI::ActionType::ReachPosition)
        {
          theTurnToTargetThenTurnToUserThenPointAndSaySomethingSkill(theTaskController.currentRobotDestination.translation, 
                                                                    Vector3f(theTaskController.userPosition.x(), theTaskController.userPosition.y(), theTaskController.userHeight),
                                                                    Vector3f(theTaskController.currentRobotDestination.translation.x(), theTaskController.currentRobotDestination.translation.y(), 0.f),
                                                                    std::string("ReachingPosition.wav"));
        }
        else if(theTaskController.getCurrentActionType() == HRI::ActionType::ReachBall)
        {
          theTurnToTargetThenTurnToUserThenPointAndSaySomethingSkill(theTaskController.currentRobotDestination.translation, 
                                                                    Vector3f(theTaskController.userPosition.x(), theTaskController.userPosition.y(), theTaskController.userHeight),
                                                                    Vector3f(theTaskController.currentRobotDestination.translation.x(), theTaskController.currentRobotDestination.translation.y(), 0.f),
                                                                    std::string("ReachingBall.wav"));
        }
      }
    }

    state(walkToPosition)
    {
      transition
      {
           if(theLibCheck.distance(theRobotPose, theTaskController.currentRobotDestination) <= theTaskController.reachPositionDistanceThreshold)
            goto stand;
      }

      action
      {
        theActivitySkill(BehaviorStatus::reaching_position);
        Vector2f localTarget = theLibCheck.glob2Rel(theTaskController.currentRobotDestination.translation.x(), theTaskController.currentRobotDestination.translation.y()).translation;
        //theLookAtPointSkill(Vector3f(localTarget.x(), localTarget.y(), 10.f));
        theLookLeftAndRightSkill();
        theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f), theTaskController.currentRobotDestination);
      }
    }

    state(stand)
    {
      transition
      {}

      action
      {
        theActivitySkill(BehaviorStatus::idle);
        theLookForwardSkill();
        theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f), theTaskController.currentRobotDestination);
      }
    }

  }

  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }
};

MAKE_CARD(ReachPositionCard);
