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
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"

#include "Representations/BehaviorControl/TasksProvider/TaskController.h"

#include "Tools/Math/BHMath.h"
#include "Platform/SystemCall.h"
#include <string>
#include <stdlib.h>
#include <time.h>

CARD(IdleCard,
{,
  CALLS(Activity),

  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(LookLeftAndRight),

  CALLS(Stand),
  CALLS(Kick),
  CALLS(WalkToTarget),

  REQUIRES(TaskController),
  REQUIRES(RobotPose),
  REQUIRES(LibCheck),

  USES(BehaviorStatus),

  LOADS_PARAMETERS(
  {,
    (int) initialWaitTime,
    (float) walkSpeed,

    (float) saySomethingTimeout,
    (float) turnToUserTimeout,
    (float) waitForSoundToStartPlaying,

    (bool) turnToUserInIdleState,

    (Rangef) smallAlignmentRange,
  }),
  
});

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

class IdleCard : public IdleCardBase
{

  bool soundPlaying = false;


  bool preconditions() const override
  {
    //std::cout<<"theTaskController.getCurrentActionType(): "<<TypeRegistry::getEnumName(theTaskController.getCurrentActionType())<<std::endl;
    //return theTaskController.getCurrentActionType() == HRI::ActionType::Idle;
    return true;
  }

  bool postconditions() const override
  {
    return theTaskController.getCurrentActionType() != HRI::ActionType::Idle;
  }

  option
  {

    initial_state(start)
    {
      srand(time(NULL));

      std::cout<<"REACH_POSITION: start"<<std::endl;
      transition
      {
        if(state_time > initialWaitTime)
          goto idle_state;
      }

      action
      {
        theActivitySkill(BehaviorStatus::idle);
        theLookForwardSkill();
        theStandSkill();
      }
    }
    
    state(idle_state)
    {
      transition
      {
        if(!smallAlignmentRange.isInside(calcAngleToTarget(theTaskController.userPosition).toDegrees()))
        {
          if(state_time > turnToUserTimeout && turnToUserInIdleState)
          {
            std::cout << "idle_state -> turnToUser: turn to user TIMEOUT" << std::endl;
            goto turnToUser;
          }
        }
        else
        {
          if(state_time > saySomethingTimeout && theTaskController.interactWithUser)
          {
            std::cout << "idle_state -> saySomething: say something TIMEOUT" << std::endl;
            goto saySomething;
          }
        }
      }

      action
      {
        theActivitySkill(BehaviorStatus::idle);
        theLookLeftAndRightSkill();
        theStandSkill();
      }
    }

    state(saySomething)
    {
      transition
      {
        if(soundPlaying)
        {
          std::cout<<"saySomething -> waitForSoundToPlay: sound started playing"<<std::endl;
          goto waitForSoundToPlay;
        }
      }

      action
      {
        Vector2f localLookTarget = theLibCheck.glob2Rel(theTaskController.userPosition.x(), theTaskController.userPosition.y()).translation;
        theLookAtPointSkill(Vector3f(localLookTarget.x(), localLookTarget.y(), theTaskController.userHeight));
        theActivitySkill(BehaviorStatus::idle);
        theStandSkill();
        if(!soundPlaying)
        {
          playRandomIdleSound();
          soundPlaying = true;
        }
      }
    }

    state(waitForSoundToPlay)
    {
      transition
      {
        if(state_time > waitForSoundToStartPlaying && !SoundPlayer::isPlaying())
        {
          std::cout << "saySomething -> idle_state: sound played" << std::endl;
          soundPlaying = false;
          goto idle_state;
        }
      }

      action
      {
        theActivitySkill(BehaviorStatus::idle);

        theStandSkill();

        Vector2f localLookTarget = theLibCheck.glob2Rel(theTaskController.userPosition.x(), theTaskController.userPosition.y()).translation;
        theLookAtPointSkill(Vector3f(localLookTarget.x(), localLookTarget.y(), theTaskController.userHeight));
      }
    }

    state(turnToUser)
    {
      transition
      {
        if(smallAlignmentRange.isInside(calcAngleToTarget(theTaskController.userPosition).toDegrees()))
        {
          std::cout << "turnToUser -> idle_state: turned to user" << std::endl;
          goto idle_state;
        }
      }

      action
      {
        theActivitySkill(BehaviorStatus::walking_to_initial_position);
       
        theWalkToTargetSkill(Pose2f(1.0f, walkSpeed, walkSpeed), Pose2f(calcAngleToTarget(theTaskController.userPosition), 0.f, 0.f));      

        Vector2f localLookTarget = theLibCheck.glob2Rel(theTaskController.userPosition.x(), theTaskController.userPosition.y()).translation;
        theLookAtPointSkill(Vector3f(localLookTarget.x(), localLookTarget.y(), theTaskController.userHeight));
      }
    }
  }

  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }

  void playRandomIdleSound() const
  {
    int random = rand() % 7 + 1;

    switch(random)
    {
      case 1:
      {
        std::cout<<"Playing sound: Joke1.wav"<<std::endl;
        SoundPlayer::play("Joke1.wav");
        break;
      }
      case 2:
      {
        std::cout<<"Playing sound: Joke2.wav"<<std::endl;
        SoundPlayer::play("Joke2.wav");
        break;
      }
      case 3:
      {
        std::cout<<"Playing sound: Joke3.wav"<<std::endl;
        SoundPlayer::play("Joke3.wav");
        break;
      }
      case 4:
      {
        std::cout<<"Playing sound: Joke4.wav"<<std::endl;
        SoundPlayer::play("Joke4.wav");
        break;
      }
      case 5:
      {
        std::cout<<"Playing sound: Tip1.wav"<<std::endl;
        SoundPlayer::play("Tip1.wav");
        break;
      }
      case 6:
      {
        std::cout<<"Playing sound: Tip2.wav"<<std::endl;
        SoundPlayer::play("Tip2.wav");
        break;
      }
      case 7:
      {
        std::cout<<"Playing sound: Tip3.wav"<<std::endl;
        SoundPlayer::play("Tip3.wav");
        break;
      }
    }
  }
};

MAKE_CARD(IdleCard);
