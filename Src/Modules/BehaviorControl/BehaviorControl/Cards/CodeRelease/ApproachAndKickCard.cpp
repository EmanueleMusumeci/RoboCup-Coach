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


  USES(BehaviorStatus),
  USES(TeamData),

  LOADS_PARAMETERS(
  {,
    (float) normalWalkSpeed,
    (float) slowerWalkSpeed,
    
    (int) initialWaitTime,
    (int) ballNotSeenTimeout,
    (int) changeTargetTimeout,
    (int) realignmentTimeout,
    (int) optionOscillationTime,

    (float) ballPositionChangedThreshold,

    //Used to align to the static approach point
    (Rangef) approachXRange,
    (Rangef) approachYRange,

    //Used to align very precisely just before kicking
    (Rangef) smallApproachXRange,
    (Rangef) smallApproachYRange,

    //Two ball alignment angle ranges: 
    //The bigger one is used to decide when to realign (so we give more room to small errors in alignment) 
    (Rangef) smallBallAlignmentRange,

    //The smaller angle range is used when aligning precisely just before the kick 
    (Rangef) smallerBallAlignmentRange,

    (int) maxKickWaitTime,

    (float) BALL_KICK_OFFSET_X,
    (float) BALL_KICK_OFFSET_Y,
    
    //Recovery behavior, when ball is lost, the robot performs a 360° turn hoping to find it again. 
    //If it doesn't find it it might have gone out so the robot goes back to the center of the field
    (bool) goToCenterWhenSearchingForBall, 

    (bool) LOOK_LEFT_AND_RIGHT_WHILE_ALIGNING,
    (bool) LOOK_LEFT_AND_RIGHT_WHILE_WALKING_TO_BALL,

    (float) PARAMETRIC_LOOK_LEFT_AND_RIGHT_SPEED,

    (bool) SHOW_CONDITIONS_DEBUG,

    (bool) DEBUG_MODE,

    (float) STANDING_TIME_BEFORE_KICKING,
    (float) STANDING_TIME_AFTER_KICKING,
  }),
});


class ApproachAndKickCard : public ApproachAndKickCardBase
{

  bool targetChosen = false;

  Vector2f chosenTarget;
  Vector2f goalTarget;
  Vector2f previousBallPosition;


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

    /*common_transition
    {
      if(theRobotPose.translation.x() > theFieldDimensions.centerCircleRadius 
      && attemptRelocalizationEveryTimeRange.isInside(option_time % (int)((attemptRelocalizationEveryTimeRange.max - attemptRelocalizationEveryTimeRange.min)/2.f)))
      {
        goto attemptRelocalization;
      }
    }*/

    initial_state(start)
    {
      std::cout<<"APPROACH_AND_KICK: start"<<std::endl;
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
            goto choose_target;
          }
        }
      }

      action
      {
        theActivitySkill(BehaviorStatus::approach_and_carry_start);
        
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
          std::cout<<"speakToHuman -> choose_target: sound finished playing"<<std::endl;
          goto choose_target;
        }
      }

      action
      {
        theTurnToTargetThenTurnToUserThenPointAndSaySomethingSkill(theTaskController.currentBallDestination, 
                                                                  Vector3f(theTaskController.userPosition.x(), theTaskController.userPosition.y(), theTaskController.userHeight),
                                                                  Vector3f(theTaskController.currentBallDestination.x(), theTaskController.currentBallDestination.y(), 0.f),
                                                                  std::string("KickingToGoal.wav"));
      }
    }

    state(turnToBall)
    {
      std::cout<<"turnToBall"<<std::endl;
      transition
      {        
        if(smallBallAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees()))
        {
          std::cout<<"turnToBall -> choose_target: aligned to ball"<<std::endl;
          goto choose_target;
        }
      }

      action
      {
        theActivitySkill(BehaviorStatus::realigning_to_ball);
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(normalWalkSpeed, normalWalkSpeed, normalWalkSpeed), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));

      }
    }

    state(choose_target)
    {
      transition
      {

        if(DEBUG_MODE)
        {
          std::cout<<"choose_target -> debug_state: DEBUG MODE"<<std::endl;
          goto debug_state;
        }

        if(targetChosen)
        {
          std::cout<<"choose_target -> walkToBall: target chosen"<<std::endl;
          targetChosen = false;
          goto secondStepAlignment;
        }
      }
      action
      {
        theActivitySkill(BehaviorStatus::choosing_target);

        goalTarget = theLibCheck.goalTarget();
        chosenTarget = theTaskController.currentBallDestination;
        previousBallPosition = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

        targetChosen = true;

        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        theStandSkill();
      }
    }

    state(debug_state)
    {
      action
      {
        theActivitySkill(BehaviorStatus::debug_standing);
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(walkToBall)
    {
      transition
      {
        Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        //float offsetBallYPosition = theBallModel.estimate.position.y() + BALL_KICK_OFFSET_Y;

        Vector2f approachPoint;
        if(theLibCheck.distance(theRobotPose, theBallCarrierModel.staticApproachPoint().translation) < theBallCarrierModel.staticApproachRadius * 1/2)
        {
          approachPoint = theBallCarrierModel.dynamicApproachPoint().translation;
        }
        else
        {
          approachPoint = theBallCarrierModel.staticApproachPoint().translation;
        }
        Pose2f secondStepXPos = theLibCheck.glob2Rel(approachPoint.x(), approachPoint.y());
        float offsetBallYPosition = secondStepXPos.translation.y() + BALL_KICK_OFFSET_Y;

        //if the LOCAL ball is inside the approach on the x axis 
        //AND the LOCAL ball is inside the approach range on the y axis 
        //AND the robot is aligned with the target
        if(approachXRange.isInside(theBallModel.estimate.position.x())
        && approachYRange.isInside(theBallModel.estimate.position.y())
        && smallerBallAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees()))
        {
          std::cout<<"walkToBall -> approachToKick: OK to approach"<<std::endl;
          goto approachToKick;
        }
        
        //NEWLY ADDED
        if(theBallModel.estimate.position.norm() < approachXRange.max
//NOTICE: potential BUG: if the robot is walking to ball and finds himself in G but turned towards the ball, this will be interpreted as B.
          //this condition here is supposed to fix it
        && theLibCheck.distance(globalBall, Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal)) < theLibCheck.distance(theRobotPose, Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal))    
        && approachXRange.isInside(theBallModel.estimate.position.x())
        && !approachYRange.isInside(offsetBallYPosition))
        {
          //Case B: Robot is in the X range but not in the Y range
          std::cout<<"walkToBall -> secondStepAlignment: kick Case B"<<std::endl;
          goto secondStepAlignment;
        }
/*
        //NEWLY ADDED
        if(theBallModel.estimate.position.norm() < approachXRange.max
//NOTICE: potential BUG: if the robot is walking to ball and finds himself in G but turned towards the ball, this will be interpreted as B.
          //this condition here is supposed to fix it
        && theLibCheck.distance(globalBall, Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal)) >= theLibCheck.distance(theRobotPose, Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal))    
        && approachXRange.isInside(theBallModel.estimate.position.x())
        && !approachYRange.isInside(offsetBallYPosition))
        {
          //Case B: Robot is in the X range but not in the Y range
          std::cout<<"walkToBall -> secondStepAlignment: kick Case B"<<std::endl;
          goto firstStepAlignment;
        }
*/
        if(state_time > changeTargetTimeout)
        {
          std::cout<<"walkToBall -> choose_target: TIMEOUT"<<std::endl;
          goto choose_target;
        }
        
      }

      action
      {
        theActivitySkill(BehaviorStatus::reaching_ball);

        Vector2f ballPositionRelative = theBallModel.estimate.position;

        if(LOOK_LEFT_AND_RIGHT_WHILE_WALKING_TO_BALL)
        {
          if(theRobotPose.translation.y() > 0)
          {
          	theParametricLookLeftAndRightSkill(20,60,20,PARAMETRIC_LOOK_LEFT_AND_RIGHT_SPEED);
          }
          else
          {
          	theParametricLookLeftAndRightSkill(60,20,20,PARAMETRIC_LOOK_LEFT_AND_RIGHT_SPEED);
          }
        }
        else
        {
          theLookAtPointSkill(Vector3f(ballPositionRelative.x(), ballPositionRelative.y(), 0.f));
        }

        theWalkToTargetPathPlannerSkill(Pose2f(normalWalkSpeed, normalWalkSpeed, normalWalkSpeed), theBallCarrierModel.staticApproachPoint());
      }
    }

    state(firstStepAlignment)
    {
      transition
      {
        if(theBallModel.estimate.position.x() > theBallSpecification.radius*2
        )
        {
          std::cout<<"firstStepAlignment -> secondStepAlignment"<<std::endl;
          goto secondStepAlignment;
        }
      }
      action
      {

        Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

        theActivitySkill(BehaviorStatus::realigning_to_ball);
        
        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));

        theWalkAtRelativeSpeedSkill(Pose2f(theLibCheck.angleToTarget(globalBall.x(), globalBall.y()), -1.0, 0.0));
      }
    }

    state(secondStepAlignment)
    {
      transition
      {


        if(theBallModel.estimate.position.norm() > approachXRange.max)
        {
          goto walkToBall;
        }

        //float offsetBallYPosition = theBallModel.estimate.position.y() + BALL_KICK_OFFSET_Y;

        Vector2f approachPoint;
        if(theLibCheck.distance(theRobotPose, theBallCarrierModel.staticApproachPoint().translation) < theBallCarrierModel.staticApproachRadius * 1/2)
        {
          approachPoint = theBallCarrierModel.dynamicApproachPoint().translation;
        }
        else
        {
          approachPoint = theBallCarrierModel.staticApproachPoint().translation;
        }
        Pose2f secondStepXPos = theLibCheck.glob2Rel(approachPoint.x(), approachPoint.y());
        float offsetBallYPosition = secondStepXPos.translation.y() + BALL_KICK_OFFSET_Y;

        DEBUG_CODE(approachXRange.isInside(theBallModel.estimate.position.x()))
        DEBUG_CODE(theBallModel.estimate.position.x())
        DEBUG_CODE(approachYRange.isInside(offsetBallYPosition))
        DEBUG_CODE(offsetBallYPosition)
        DEBUG_CODE(smallBallAlignmentRange.isInside(Angle(theLibCheck.angleToTarget(chosenTarget.x(), chosenTarget.y())).toDegrees()))
        DEBUG_CODE(Angle(theLibCheck.angleToTarget(chosenTarget.x(), chosenTarget.y())).toDegrees())
        if(approachXRange.isInside(theBallModel.estimate.position.x())
        && approachYRange.isInside(offsetBallYPosition)
        && smallBallAlignmentRange.isInside(Angle(theLibCheck.angleToTarget(chosenTarget.x(), chosenTarget.y())).toDegrees()))
        {
          std::cout<<"secondStepAlignment -> approachToKick: aligned to ball"<<std::endl;
          goto approachToKick;
        }

        //NEWLY ADDED
        if(state_time > realignmentTimeout)
        {
          std::cout<<"secondStepAlignment -> approachToKick: TIMEOUT"<<std::endl;
          goto approachToKick;
        }

//BELOW copied from the KICK state
        //Case E: Robot not in the X range but is behind the ball and is not in the Y range 
        if(theBallModel.estimate.position.x() < approachXRange.min)
        {
          if(Rangef(-theBallSpecification.radius*2, theBallSpecification.radius*2).isInside(theBallModel.estimate.position.x()))
          {
            if(!approachYRange.isInside(offsetBallYPosition))
            {
              std::cout<<"kick -> firstStepAlignment: Case E"<<std::endl;
              goto firstStepAlignment; //go back
            }
          }
        }
        
        //Case F: Robot is between the target and the ball
        if(theBallModel.estimate.position.x() < -theBallSpecification.radius*2)
        {
          if(approachYRange.isInside(offsetBallYPosition))
          {
            std::cout<<"secondStepAlignment -> walkToBall: Case F"<<std::endl;
            goto walkToBall;
          }
        }
        
        //Case G: the robot is between the target and the ball and to the side wrt the ball

        if(theBallModel.estimate.position.x() < -theBallSpecification.radius*2)
        {
          if(!approachYRange.isInside(offsetBallYPosition))
          {  
            std::cout<<"secondStepAlignment -> firstStepAlignment: Case G"<<std::endl;
            goto firstStepAlignment;
          }
        }

      }
      action
      {
        theActivitySkill(BehaviorStatus::realigning_to_ball);
        
        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        
        float localGoalYCoordinate = theLibCheck.glob2Rel(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal).translation.y();

        if(LOOK_LEFT_AND_RIGHT_WHILE_ALIGNING)
        {
          if(theRobotPose.translation.y() > 0)
          {
          	theParametricLookLeftAndRightSkill(20,60,20,PARAMETRIC_LOOK_LEFT_AND_RIGHT_SPEED);
          }
          else
          {
          	theParametricLookLeftAndRightSkill(60,20,20,PARAMETRIC_LOOK_LEFT_AND_RIGHT_SPEED);
          }
        }
        else
        {
          theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        }
        
//NOTICE: the robot hitting the ball with the side of his foot is caused by this
        Vector2f approachPoint;
        if(theLibCheck.distance(theRobotPose, theBallCarrierModel.staticApproachPoint().translation) < theBallCarrierModel.staticApproachRadius * 1/2)
        {
          approachPoint = theBallCarrierModel.dynamicApproachPoint().translation;
        }
        else
        {
          approachPoint = theBallCarrierModel.staticApproachPoint().translation;
        }
        Pose2f secondStepXPos = theLibCheck.glob2Rel(approachPoint.x(), approachPoint.y());
        Pose2f secondStepPoseRelative = Pose2f(theLibCheck.angleToTarget(chosenTarget.x(), chosenTarget.y()), secondStepXPos.translation.x() - BALL_KICK_OFFSET_X, secondStepXPos.translation.y() + BALL_KICK_OFFSET_Y);
        theWalkToTargetSkill(Pose2f(normalWalkSpeed, normalWalkSpeed, normalWalkSpeed), secondStepPoseRelative);
      }
    }

    //Approach the ball precisely (last robot movements before the kick)
    state(approachToKick){
      transition
      {

        float offsetBallYPosition = theBallModel.estimate.position.y() + BALL_KICK_OFFSET_Y;

        //Vector2f approachPoint;
        //if(theLibCheck.distance(theRobotPose, theBallCarrierModel.staticApproachPoint().translation) < theBallCarrierModel.staticApproachRadius * 1/2)
        //{
        //  approachPoint = theBallCarrierModel.dynamicApproachPoint().translation;
        //}
        //else
        //{
        //  approachPoint = theBallCarrierModel.staticApproachPoint().translation;
        //}
        //Pose2f secondStepXPos = theLibCheck.glob2Rel(approachPoint.x(), approachPoint.y());
        //float offsetBallYPosition = secondStepXPos.translation.y() + BALL_KICK_OFFSET_Y;

        if(!approachXRange.isInside(theBallModel.estimate.position.x())
        || !approachYRange.isInside(offsetBallYPosition)
        || !smallBallAlignmentRange.isInside(Angle(theLibCheck.angleToTarget(chosenTarget.x(), chosenTarget.y())).toDegrees()))
        {
          std::cout<<"approachToKick -> secondStepAlignment: disaligned"<<std::endl;
          goto secondStepAlignment;
        }


        if(smallApproachXRange.isInside(theBallModel.estimate.position.x())
           && smallApproachYRange.isInside(theBallModel.estimate.position.y()))
        {
            std::cout<<"approachToKick -> kick: ready to kick"<<std::endl;
            goto standBeforeKicking;
        }

        //Case G: the robot is between the target and the ball and to the side wrt the ball
        if(theBallModel.estimate.position.x() < -theBallSpecification.radius*2)
        {
          if(!approachYRange.isInside(offsetBallYPosition))
          {  
            std::cout<<"kick -> firstStepAlignment: Case G"<<std::endl;
            goto firstStepAlignment;
          }
        }

        /*if(state_time > APPROACH_TO_KICK_TIMEOUT)
        {
            std::cout<<"approachToKick -> kick: TIMEOUT"<<std::endl;
            goto kick;
        } */

      }

      action
      {

        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        
        Pose2f ball = theBallModel.estimate.position;
        float x_ball = ball.translation.x();         
        float y_ball = ball.translation.y();         

        x_ball = x_ball - BALL_KICK_OFFSET_X;
        y_ball = y_ball + BALL_KICK_OFFSET_Y;

        Pose2f localTarget = Pose2f(theLibCheck.angleToTarget(chosenTarget.x(), chosenTarget.y()), x_ball, y_ball);
        theWalkToTargetSkill(Pose2f(slowerWalkSpeed, slowerWalkSpeed, slowerWalkSpeed), localTarget);
        
      }
    }

    state(standBeforeKicking)
    {
      transition
      {
        if(state_time > STANDING_TIME_BEFORE_KICKING)
        {
          std::cout<<"standBeforeKicking -> kick: TIMEOUT"<<std::endl;
          goto kick;
        }
      }
      action
      {
        theStandSkill();
      }
    }

    state(standAfterKicking)
    {
      transition
      {
        if(state_time > STANDING_TIME_AFTER_KICKING)
        {
          std::cout<<"standAfterKicking -> choose_target: TIMEOUT"<<std::endl;
          goto choose_target;
        }
      }
      action
      {
        theStandSkill();
      }
    }

    state(kick)
    {
      transition
      {
        //float offsetBallYPosition = theBallModel.estimate.position.y() + BALL_KICK_OFFSET_Y;

        Vector2f approachPoint;
        if(theLibCheck.distance(theRobotPose, theBallCarrierModel.staticApproachPoint().translation) < theBallCarrierModel.staticApproachRadius * 1/2)
        {
          approachPoint = theBallCarrierModel.dynamicApproachPoint().translation;
        }
        else
        {
          approachPoint = theBallCarrierModel.staticApproachPoint().translation;
        }
        Pose2f secondStepXPos = theLibCheck.glob2Rel(approachPoint.x(), approachPoint.y());
        float offsetBallYPosition = secondStepXPos.translation.y() + BALL_KICK_OFFSET_Y;

        if(theLibCheck.distance(previousBallPosition, theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y())) > ballPositionChangedThreshold)
        {
          std::cout<<"kick -> choose_target: ball moved too much"<<std::endl;
          goto standAfterKicking;
        }

        Vector2f ballPositionGlobal = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

        //Refer to the drawing BallCarriern Alignment chart for case letters:
        //https://docs.google.com/drawings/d/1FcS2yrCbGkUmbWM1GRGHuXTYnEhcrbkovdwFayTGWkc/edit?usp=sharing

        //Case 0: if the robot is too far away from the ball, just walkToBall
        if(theLibCheck.distance(theRobotPose, ballPositionGlobal) >= approachXRange.max)
        {
          std::cout<<"kick -> walkToBall: Case 0"<<std::endl;
          goto walkToBall;
        }
        
        //Case B: Robot is in the X range but not in the Y range
        if(approachXRange.isInside(theBallModel.estimate.position.x()))
        {
          if(!approachYRange.isInside(offsetBallYPosition))
          {
            std::cout<<"kick -> secondStepAlignent: Case B"<<std::endl;
            goto secondStepAlignment;
          }
        }
        
        //Case E: Robot not in the X range but is behind the ball and is not in the Y range 
        if(theBallModel.estimate.position.x() < approachXRange.min)
        {
          if(Rangef(-theBallSpecification.radius*2, theBallSpecification.radius*2).isInside(theBallModel.estimate.position.x()))
          {
            if(!approachYRange.isInside(offsetBallYPosition))
            {
              std::cout<<"kick -> firstStepAlignment: Case E"<<std::endl;
              goto firstStepAlignment; //go back
            }
          }
        }
        
        //Case D + wrong angle: Robot not in the X range but is behind the ball and is in the Y range but is not aligned (angle) to the ball
        if(theBallModel.estimate.position.x() < approachXRange.min)
        {
          if(theBallModel.estimate.position.x() > theBallSpecification.radius*2)
          {
            if(approachYRange.isInside(offsetBallYPosition))
            {
              if(!smallBallAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees()))
              {
                //If it is not aligned to the target first turn to the target, then the conditions will be checked again to perform further necessary alignments
                std::cout<<"kick -> turnToBall: Case D + wrong angle"<<std::endl;
                goto turnToBall;
              }
            }
          }
        }
        
        //Case F: Robot is between the target and the ball
        if(theBallModel.estimate.position.x() < -theBallSpecification.radius*2)
        {
          if(approachYRange.isInside(offsetBallYPosition))
          {
            std::cout<<"kick -> walkToBall: Case F"<<std::endl;
            goto walkToBall;
          }
        }
        
        //Case G: the robot is between the target and the ball and to the side wrt the ball
        if(theBallModel.estimate.position.x() < -theBallSpecification.radius*2)
        {
          if(!approachYRange.isInside(offsetBallYPosition))
          {  
            std::cout<<"kick -> firstStepAlignment: Case G"<<std::endl;
            goto firstStepAlignment;
          }
        }

        if(state_time > maxKickWaitTime)
        {  
          std::cout<<"kick -> choose_target: TIMEOUT"<<std::endl;
          goto choose_target;
        }
      }

      action
      {

        theActivitySkill(BehaviorStatus::kicking_to_goal);

        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        theKickSkill(false, (float) 9000.f, false); // parameters: (kick_type, mirror, distance, armsFixed)
        
        theGoalTargetSkill(goalTarget);
        theSetTargetSkill(chosenTarget);
      }
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

};

MAKE_CARD(ApproachAndKickCard);
