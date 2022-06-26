/**
 * @file C1ApproachAndCarryWithTwoStepRealignmentCard.cpp
 *
 * This file implements a behavior to carry the ball forward in the field, avoiding obstacles.
 *
 * @author Emanuele Musumeci (based on Emanuele Antonioni's basic approacher behavior structure)
 */

#include "Platform/Linux/SoundPlayer.h"

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Communication/RobotInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"

#include "Representations/BehaviorControl/TasksProvider/TaskController.h"
#include "Tools/BehaviorControl/Framework/BehaviorContext.h"

#include "Tools/Math/BHMath.h"
#include "Platform/SystemCall.h"
#include <string>
CARD(PerformInitialSpeechCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(LookLeftAndRight),
  CALLS(Stand),
  CALLS(KeyFrameArms),
  CALLS(WalkToTarget),
  CALLS(WalkToTargetPathPlanner),

  REQUIRES(LibCheck),
  REQUIRES(RobotPose),
  REQUIRES(RobotInfo),

  REQUIRES(TaskController),

  USES(BehaviorStatus),

#define __STRINGIFY_I(arg) #arg
#define __STRINGIFY(arg) __STRINGIFY_I(arg) 
//#define GOTO(to) \
//    std::cout << std::string(this->_context.stateName) << std::string(" -> ") << __STRINGIFY(to) <<std::endl; \
//    goto to;
    
#define GOTO(condition, to, reason) \
    if(condition) \
    { \
        std::cout << std::string(this->_context.stateName) << std::string(" -> ") << __STRINGIFY(to) << std::string(": ") << __STRINGIFY(reason) <<std::endl; \
        goto to; \
    };

#define DEBUG_CODE(code) \
  std::cout<<"[Robot #"<<std::to_string(theRobotInfo.number)<<"] "<<__STRINGIFY(code)<<": "<<std::to_string(code)<<std::endl;


  LOADS_PARAMETERS(
  {,
    
    (int) initialWaitTime,
    (int) waitForSoundToStartPlaying,

    (float) walkSpeed,

    (bool) DEBUG_MODE,

    (Rangef) smallAlignmentRange,

    (Vector2f) initialPosition,

    (float) distanceThreshold,
  }),
});

class PerformInitialSpeechCard : public PerformInitialSpeechCardBase
{

  bool speechStarted = false;
  bool preconditions() const override
  {
    return theTaskController.getCurrentActionType() == HRI::ActionType::PerformInitialSpeech || theTaskController.getCurrentActionType() == HRI::ActionType::PerformInstructionsSpeech;
  }

  bool postconditions() const override
  {
    return theTaskController.getCurrentActionType() != HRI::ActionType::PerformInitialSpeech && theTaskController.getCurrentActionType() != HRI::ActionType::PerformInstructionsSpeech;
  }


  option
  {

    initial_state(start)
    {
      std::cout<<"PERFORM_INITIAL_SPEECH: start"<<std::endl;
      transition
      {
        GOTO(DEBUG_MODE, debug_state, "Debug mode");
        if(state_time > initialWaitTime)
        {
          if(theTaskController.getCurrentActionType() == HRI::ActionType::PerformInstructionsSpeech)
            goto turnToUser;
        }
        else
        {
          if(theTaskController.getCurrentActionType() == HRI::ActionType::PerformInitialSpeech)
            goto reachInitialPose;
        }

      }

      action
      {
        theActivitySkill(BehaviorStatus::walking_to_initial_position);
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(debug_state)
    {
      action
      {
        theStandSkill();
      }
    }

    state(reachInitialPose)
    {
      transition
      {
        GOTO(theLibCheck.distance(theRobotPose, initialPosition) < distanceThreshold, turnToUser, "Reached initial position");
      }

      action
      {
        theActivitySkill(BehaviorStatus::walking_to_initial_position);
        theWalkToTargetPathPlannerSkill(Pose2f(1.0f, walkSpeed, walkSpeed), Pose2f(0.f, initialPosition.x(), initialPosition.y()));
                
        theLookLeftAndRightSkill();
      }

    }

    state(turnToUser)
    {
      transition
      {
        GOTO(smallAlignmentRange.isInside(calcAngleToTarget(theTaskController.userPosition).toDegrees()), startSpeech, "Reached initial position");
      }

      action
      {
        theActivitySkill(BehaviorStatus::walking_to_initial_position);
        theWalkToTargetSkill(Pose2f(1.0f, walkSpeed, walkSpeed), Pose2f(calcAngleToTarget(theTaskController.userPosition), 0.f, 0.f));
                
        Vector2f localLookTarget = theLibCheck.glob2Rel(theTaskController.userPosition.x(), theTaskController.userPosition.y()).translation;
        theLookAtPointSkill(Vector3f(localLookTarget.x(), localLookTarget.y(), theTaskController.userHeight));
      }
    }

    state(startSpeech)
    {
        transition
        {
          GOTO(speechStarted, animateSpeech, "speech started")
        }

        action
        {
            if(!speechStarted)
            {
              if(theTaskController.getCurrentActionType() == HRI::ActionType::PerformInitialSpeech)
              {
                SoundPlayer::play("InitialSpeech.wav");
              }
              else if(theTaskController.getCurrentActionType() == HRI::ActionType::PerformInstructionsSpeech)
              {
                SoundPlayer::play("InstructionsSpeech.wav");                
              }
              speechStarted = true;
              DEBUG_CODE(speechStarted);
            }
            theActivitySkill(BehaviorStatus::givingInitialSpeech);
            theStandSkill();

            Vector2f localLookTarget = theLibCheck.glob2Rel(theTaskController.userPosition.x(), theTaskController.userPosition.y()).translation;
            theLookAtPointSkill(Vector3f(localLookTarget.x(), localLookTarget.y(), theTaskController.userHeight));
        }
    }

    state(animateSpeech)
    {
        transition
        {
            if(!SoundPlayer::isPlaying() && state_time > waitForSoundToStartPlaying)
            {
              std::cout<<"animateSpeech -> speechEnd: speech finished"<<std::endl;
              speechStarted = false;
              goto speechEnd;
            }
        }

        action
        {
          theStandSkill();
          theActivitySkill(BehaviorStatus::givingInitialSpeech);
          
          Vector2f localLookTarget = theLibCheck.glob2Rel(theTaskController.userPosition.x(), theTaskController.userPosition.y()).translation;
          theLookAtPointSkill(Vector3f(localLookTarget.x(), localLookTarget.y(), theTaskController.userHeight));
        }
    }

    state(speechEnd)
    {
        action
        {
          theStandSkill();
          theActivitySkill(BehaviorStatus::givingInitialSpeech);
          
          Vector2f localLookTarget = theLibCheck.glob2Rel(theTaskController.userPosition.x(), theTaskController.userPosition.y()).translation;
          theLookAtPointSkill(Vector3f(localLookTarget.x(), localLookTarget.y(), theTaskController.userHeight));

          theTaskController.signalTaskCompleted(false);
        }
    }
  }

  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }
};

MAKE_CARD(PerformInitialSpeechCard);
