/**
 * @file ApproachAndKickCard.cpp
 *
 * This file implements a behavior to kick the ball to a specific location.
 *
 * @author Emanuele Musumeci
 */
 

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/BehaviorControl/BallCarrierModel/BallCarrierModel.h"
#include "Representations/Communication/TeamData.h"
#include "Tools/Math/BHMath.h"

#include "Representations/BehaviorControl/TasksProvider/TaskController.h"

#include "Platform/SystemCall.h"
#include <string>

#define __STRINGIFY_I(arg) #arg
#define __STRINGIFY(arg) __STRINGIFY_I(arg)
#define DEBUG_CODE(code) \
  std::cout<<"[Robot #"<<std::to_string(theRobotInfo.number)<<"] "<<__STRINGIFY(code)<<": "<<std::to_string(code)<<std::endl;

CARD(ApproachAndKickCard,
{,
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(LookLeftAndRight),
  CALLS(LookRightAndLeft),
  CALLS(ParametricLookLeftAndRight),

  CALLS(TurnToTargetThenTurnToUserThenPointAndSaySomething),

  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(Kick),
  CALLS(WalkToTargetPathPlanner),
  CALLS(WalkToTargetPathPlannerStraight),

  CALLS(Stand),
  CALLS(Activity),

  CALLS(GoalTarget),
  CALLS(SetTarget),

  CALLS(Approacher2021WithRanges),
  
  REQUIRES(MotionInfo),
  REQUIRES(FieldBall),
  REQUIRES(BallModel),
  REQUIRES(LibCheck),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(BallCarrierModel),

  REQUIRES(BallSpecification),

  REQUIRES(GameInfo),
  REQUIRES(TeamPlayersModel),

  REQUIRES(TaskController),
  REQUIRES(ObstacleModel),


  USES(BehaviorStatus),
  USES(TeamData),

  LOADS_PARAMETERS(
  {,
    (int) initialWaitTime,
    (int) time_to_stand_before_kick,
    (int) time_to_stand_after_kick,
    (float) BALL_KICK_OFFSET_X,
    (float) BALL_KICK_OFFSET_Y,

    (Rangef) approachXRange,
    (Rangef) approachYRange,
    
    (Rangef) smallBallAlignmentRange,

    (int) maxKickWaitTime,
    (int) minKickWaitTime,

    (bool) USE_DEBUG_MESSAGES,
  }),
});


class ApproachAndKickCard : public ApproachAndKickCardBase
{

  Vector2f chosenTarget = Vector2f(theFieldDimensions.xPosOpponentGoal, 0.f);
  bool card_lock_in = false;


  bool preconditions() const override
  {
    //std::cout<<"theTaskController.getCurrentActionType(): "<<TypeRegistry::getEnumName(theTaskController.getCurrentActionType())<<std::endl;
    return theTaskController.getCurrentActionType() == HRI::ActionType::Kick
          &&
          theLibCheck.distance(theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()), theTaskController.currentBallDestination) > theTaskController.kickDistanceThreshold;
  }

  bool postconditions() const override
  {
    return theTaskController.getCurrentActionType() != HRI::ActionType::Kick
          ||
          theLibCheck.distance(theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()), theTaskController.currentBallDestination) <= theTaskController.kickDistanceThreshold;
  }

  option
  {
    theActivitySkill(BehaviorStatus::approachAndKick);

    initial_state(start)
    {
      
      std::cout<<"APPROACHER2021: start"<<std::endl;
      transition
      { /*
        card_lock_in = false;
        if(state_time > initialWaitTime)
        {
          std::cout<<"start -> approach: TIMEOUT"<<std::endl;
          nudged_ball = false;
          goto approach;
        }*/
        goto approach;
      }

      action
      {
        //if (!state_time) DEBUG_SAY("Striker shooting");
        card_lock_in = false;
        theLookForwardSkill();
        theStandSkill();
      }
    }

    // the target can change freely as long as we are away from the ball, and every X seconds once we get close to avoid indecision
    state(approach)
    {
      transition
      {
        Vector2f ballRelative = theBallModel.estimate.position;
        if (theApproacher2021WithRangesSkill.isDone()) {
          std::cout<<"approach -> stand_before_kick: DONE"<<std::endl;
          goto stand_before_kick;
        }
      }
      action
      {
        card_lock_in = false;
        Vector2f goalTarget = theLibCheck.goalTarget();
        chosenTarget = Vector2f(goalTarget.x() + 9000, goalTarget.y());
        //chosenTarget = theBallCarrierModel.dynamicTarget.translation;
        theApproacher2021WithRangesSkill(chosenTarget, 350.f, BALL_KICK_OFFSET_X, BALL_KICK_OFFSET_Y, false, 1.0f, 0.8f, 0.6f, Rangef(140, 450), Rangef(-60, 60), Rangef(-7.5, 7.5));
      }
    }

    state(stand_before_kick)
    {
      transition
      {
        std::cout<<state_time<<std::endl;
        if (state_time > time_to_stand_before_kick) {
          std::cout<<"stand_before_kick -> kick: TIMEOUT"<<std::endl;
          goto kick;
        }
      }
      action
      {
        card_lock_in = true;
        Vector2f chosenRelative = theBallModel.estimate.position;

        theLookAtPointSkill(Vector3f(chosenRelative.x(), chosenRelative.y(), 0.f));
        theStandSkill();
      }
    }

    state(kick)
    {
      transition
      {
        Vector2f chosenRelative = theBallModel.estimate.position;

        float offsetBallYPosition = std::abs(chosenRelative.y()) - BALL_KICK_OFFSET_Y;

        Vector2f ballPositionGlobal = theLibCheck.rel2Glob(chosenRelative.x(), chosenRelative.y()).translation;

        //Refer to the drawing BallCarriern Alignment chart for case letters:
        //https://docs.google.com/drawings/d/1FcS2yrCbGkUmbWM1GRGHuXTYnEhcrbkovdwFayTGWkc/edit?usp=sharing

        //Case 0: if the robot is too far away from the ball, just walkToBall
        if(theLibCheck.distance(theRobotPose, ballPositionGlobal) >= approachXRange.max)
        {
          std::cout<<"kick -> stand_after_kick: Case 0"<<std::endl;
          goto stand_after_kick;
        }
        
        //Case B: Robot is in the X range but not in the Y range
        if(approachXRange.isInside(chosenRelative.x()))
        {
          if(!approachYRange.isInside(offsetBallYPosition))
          {
            std::cout<<"kick -> approach: Case B"<<std::endl;
            goto approach;
          }
        }
        
        //Case E: Robot not in the X range but is behind the ball and is not in the Y range 
        if(chosenRelative.x() < approachXRange.min)
        {
          if(Rangef(-theBallSpecification.radius*2, theBallSpecification.radius*2).isInside(chosenRelative.x()))
          {
            if(!approachYRange.isInside(offsetBallYPosition))
            {
              std::cout<<"kick -> approach: Case E"<<std::endl;
              goto approach; //go back
            }
          }
        }
        
        //Case D + wrong angle: Robot not in the X range but is behind the ball and is in the Y range but is not aligned (angle) to the ball
        if(chosenRelative.x() < approachXRange.min)
        {
          if(chosenRelative.x() > theBallSpecification.radius*2)
          {
            if(approachYRange.isInside(offsetBallYPosition))
            {
              if(!smallBallAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees()))
              {
                //If it is not aligned to the target first turn to the target, then the conditions will be checked again to perform further necessary alignments
                std::cout<<"kick -> approach: Case D + wrong angle"<<std::endl;
                goto approach;
              }
            }
          }
        }
        
        //Case F: Robot is between the target and the ball
        if(chosenRelative.x() < -theBallSpecification.radius*2)
        {
          if(approachYRange.isInside(offsetBallYPosition))
          {
            std::cout<<"kick -> approach: Case F"<<std::endl;
            goto approach;
          }
        }
        
        //Case G: the robot is between the target and the ball and to the side wrt the ball
        if(chosenRelative.x() < -theBallSpecification.radius*2)
        {
          if(!approachYRange.isInside(offsetBallYPosition))
          {  
            std::cout<<"kick -> approach: Case G"<<std::endl;
            goto approach;
          }
        }

        if(state_time > maxKickWaitTime)
        {  
          std::cout<<"kick -> stand_after_kick: TIMEOUT"<<std::endl;
          goto stand_after_kick;
        }

        if(state_time > minKickWaitTime && theKickSkill.isDone()){
          std::cout<<"kick -> stand_after_kick: kick done"<<std::endl;
          goto stand_after_kick;
        }
      }

      action
      {
        card_lock_in = true;
        Vector2f chosenRelative = theBallModel.estimate.position;

        theLookAtPointSkill(Vector3f(chosenRelative.x(), chosenRelative.y(), 0.f));

        float distance = 99999.f;    // shoot as strong as possible
        theKickSkill(false, distance, false); // parameters: (kick_type, mirror, distance, armsFixed)
        
      }
    }

    state (stand_after_kick)
    {
      transition
      {
        if (state_time > time_to_stand_after_kick) {
          std::cout<<"time_to_stand_after_kick -> approach: TIMEOUT"<<std::endl;
          goto approach;
        }
      }
      action
      {
        card_lock_in = true;
        theLookForwardSkill();
        theStandSkill();
      }
    }
  }

  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }

  bool gotta_nudge_ball(float dist_thr) {
    for (Obstacle opp : theObstacleModel.obstacles) {
      if (opp.type == Obstacle::opponent) {
        if (opp.center.squaredNorm() < dist_thr*dist_thr) {
          return true;
        }
      }
    }
    return false;
  }

};

MAKE_CARD(ApproachAndKickCard);
