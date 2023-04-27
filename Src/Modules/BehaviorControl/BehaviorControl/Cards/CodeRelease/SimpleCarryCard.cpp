#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Platform/SystemCall.h"
#include <iostream>
#include <algorithm>    

#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/BehaviorControl/BallCarrierModel/BallCarrierModel.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"

#define DEBUG_SAY(msg) if (USE_DEBUG_MESSAGES) SystemCall::say(msg)



CARD(SimpleCarryCard,
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
  REQUIRES(BallModel),
  REQUIRES(FrameInfo),
  REQUIRES(LibCheck),
  REQUIRES(BallCarrierModel),

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



class SimpleCarryCard : public SimpleCarryCardBase
{
  bool preconditions() const override
  {
    return true;
  }


  bool postconditions() const override
  {
    return true; // std::cout << "I'm HERE" << std::endl; // String saved for debug
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
        if (!state_time) DEBUG_SAY("Striker simple carry");
        first_time = true;
        theLookForwardSkill();
        theStandSkill();
      }
    }


    state(walkToBall)
    {
      transition
      {
        if (theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > ballNotSeenTimeout)
          goto searchForBall;

        
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
        theApproacher2021WithRangesSkill(chosenTarget, 350.f, 210.f, 55.f, true, 1.0f, 0.8f, 0.5f, Rangef(170, 450), Rangef(-65, 65), Rangef(-7.5, 7.5));
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


};


MAKE_CARD(SimpleCarryCard);
