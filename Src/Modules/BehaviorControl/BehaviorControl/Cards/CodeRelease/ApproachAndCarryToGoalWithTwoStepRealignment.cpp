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

#include "Representations/Communication/RobotInfo.h"

#include "Representations/BehaviorControl/TasksProvider/TaskController.h"

#include "Tools/Math/BHMath.h"
#include "Platform/SystemCall.h"
#include <string>


#define __STRINGIFY_I(arg) #arg
#define __STRINGIFY(arg) __STRINGIFY_I(arg)
#define DEBUG_CODE(code) \
  std::cout<<"[Robot #"<<std::to_string(theRobotInfo.number)<<"] "<<__STRINGIFY(code)<<": "<<std::to_string(code)<<std::endl;

CARD(ApproachAndCarryToGoalWithTwoStepRealignmentCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),

  CALLS(LookForward),
  CALLS(LookAtPoint),

  CALLS(LookLeftAndRight),
  CALLS(LookRightAndLeft),

  CALLS(ParametricLookLeftAndRight),

  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(Kick),
  CALLS(WalkToTargetPathPlanner),
  
  CALLS(GoalTarget),
  CALLS(SetTarget),
  
  CALLS(KeyFrameArms),

  CALLS(TurnToTargetThenTurnToUserAndSaySomething),

  REQUIRES(FieldBall),
  REQUIRES(BallModel),
  REQUIRES(BallCarrierModel),
  REQUIRES(LibCheck),
  
  REQUIRES(FieldDimensions),
  
  REQUIRES(RobotPose),
  REQUIRES(BallSpecification),
  REQUIRES(ObstacleModel),

  REQUIRES(RobotInfo),

  USES(BehaviorStatus),

  REQUIRES(TaskController),


  LOADS_PARAMETERS(
  {,
    (float) normalWalkSpeed,
    (float) slowerWalkSpeed,
    (float) walkToBallWalkSpeed,
    
    (int) initialWaitTime,

    (int) realignmentTimeout,
    (int) pathChangeTimeoutInsideCenterCircle,
    (int) pathChangeTimeoutOutsideCenterCircle,

    (int) ballNotSeenTimeout,
    (int) realignmentBallNotSeenTimeout,
    

    (float) ballPositionChangedThreshold,
    

    (Rangef) approachXRange,

    (Rangef) approachYRange,

    (Rangef) smallBallAlignmentRange,

    (float) nearbyObstacleRadius, //Radius inside which an obstacle is considered "nearby"

    (int) maxKickWaitTime,

    (bool) goToCenterWhenSearchingForBall, //Recovery behavior, when ball is lost, the robot performs a 360° turn hoping to find it again. 
                                           //If it doesn't find it it might have gone out so the robot goes back to the center of the field

    (float) safetyObstacleRadius,         // default = 500; used to move hands for stability when not around obstacles
    (float) dangerObstacleRadius,         //At and below this radius, the robot will walk at slowerWalkSpeed, between this and safetyObstacleRadius, the speed will be proportional to the distance, above safetyObstacleRadius, it will be normalWalkSpeed

    (float) robotArmsRange,               //Clearance radius of an arm movement

    (float) MAX_OBSTACLE_LOCALIZATION_DRIFT, //Maximum drift of an obstacle after which the path is recomputed

    (bool) USE_FOOT_OFFSET,               //Use an offset added to the approach point to align the ball to the foot when InWalkKicking
    (float) BALL_KICK_OFFSET_Y,           //Foot offset along the Y coordinate

    (bool) LOOK_LEFT_AND_RIGHT_WHILE_ALIGNING,
    (bool) LOOK_LEFT_AND_RIGHT_WHILE_WALKING_TO_BALL,
    (bool) LOOK_LEFT_AND_RIGHT_WHILE_KICKING,
    (float) MAX_LOOKING_ANGLE,

    (float) LEFT_AND_RIGHT_LOOKING_SPEED,

    (float) START_LOOKING_LEFT_AND_RIGHT_AT_X_COORDINATE,

    (bool) WALK_ON_BALL,
    (float) USE_IN_WALK_KICKS_FOR_DISTANCE_MORE_THAN,

    (bool) SHOW_CONDITIONS_DEBUG,

    (bool) DEBUG_MODE,

    (Rangef) FIRST_STEP_REALIGN_RANGE,

    (bool) putArmsOnBack,
  }),
});


class ApproachAndCarryToGoalWithTwoStepRealignmentCard : public ApproachAndCarryToGoalWithTwoStepRealignmentCardBase
{

  bool targetChosen = false;

  //To avoid oscillation in front of the goal
  bool footChosen = false;
  bool kickWithRightFoot = true;


  float pathChangeTime = 0.f;
  float pathChangeTimeout = pathChangeTimeoutInsideCenterCircle;

  float walkSpeed = normalWalkSpeed;

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
    return theTaskController.getCurrentActionType() == HRI::ActionType::CarryAndKickToGoal
                &&
                (
                !theBallCarrierModel.xRangeDistanceFromGoalToUseKicks.isInside(theFieldDimensions.xPosOpponentGroundline - theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation.x())
                ||
                (theBallCarrierModel.xRangeDistanceFromGoalToUseKicks.isInside(theFieldDimensions.xPosOpponentGroundline - theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation.x())
                    &&
                    !theBallCarrierModel.isTargetOnGoal)
                  ||
                  theBallModel.estimate.position.norm() > theBallCarrierModel.maximumApproachDistance
                  ||
                  !theFieldBall.ballWasSeen(ballNotSeenTimeout)
                  )           
    ;
  }

  bool postconditions() const override
  {
    return theTaskController.getCurrentActionType() != HRI::ActionType::CarryAndKickToGoal
           ||
           (theBallCarrierModel.xRangeDistanceFromGoalToUseKicks.isInside(theFieldDimensions.xPosOpponentGroundline - theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation.x())
                &&            
                theBallCarrierModel.isTargetOnGoal
                    &&
                    theBallModel.estimate.position.norm() <= theBallCarrierModel.maximumApproachDistance
                        &&
                        !isSearchingToggle)
           ||
           (countNearbyObstacles(nearbyObstacleRadius) > 0 && theRobotPose.translation.norm() <= theFieldDimensions.centerCircleRadius -200.f)
            ;
  }

  option
  {

    common_transition
    {


      if(isRobotInCenterCircle())
      {
        pathChangeTimeout = pathChangeTimeoutInsideCenterCircle;
      } 
      else
      {
        pathChangeTimeout = pathChangeTimeoutOutsideCenterCircle;
      }

      //Number of obstacles changed
      if(!isSearchingToggle && countNearbyObstacles(nearbyObstacleRadius) != previousNearbyObstaclesCount && (option_time - pathChangeTime) > pathChangeTimeout)
      {
        std::cout<<"common transition -> choose_target: obstacle number changed: "<<std::to_string(previousNearbyObstaclesCount)<<" before, "<<std::to_string(countNearbyObstacles(nearbyObstacleRadius))<<"now"<<std::endl;
        goto choose_target;
      }

      //Same number of obstacles but big position drift due to localization
      if(!isSearchingToggle && theLibCheck.distance(getNearestObstacle(true), nearestObstacleGlobalPositionWhenPathWasComputed) > MAX_OBSTACLE_LOCALIZATION_DRIFT && (option_time - pathChangeTime) > pathChangeTimeout)
      {
        std::cout<<"common transition -> choose_target: obstacle position drift too high"<<std::endl;
        goto choose_target;
      }


    }

    initial_state(start)
    {
      std::cout<<"APPROACH_AND_CARRY_WITH_TWO_STEP_REALIGNMENT: start"<<std::endl;
      transition
      {
        if(state_time > initialWaitTime)
        {
          if(theTaskController.interactWithUser)
          {
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
        if(theTurnToTargetThenTurnToUserAndSaySomethingSkill.isDone())
        {
          std::cout<<"speakToHuman -> choose_target: sound finished playing"<<std::endl;
          goto choose_target;
        }
      }

      action
      {
        theTurnToTargetThenTurnToUserAndSaySomethingSkill(theTaskController.currentBallDestination, 
                                                                  Vector3f(theTaskController.userPosition.x(), theTaskController.userPosition.y(), theTaskController.userHeight),
                                                                  //Vector3f(theTaskController.currentBallDestination.x(), theTaskController.currentBallDestination.y(), 0.f),
                                                                  std::string("TryScoreAGoal.wav"));
      }
    }


    state(turnToBall)
    {
      
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          std::cout<<"turnToBall -> searchForBall: ball not found"<<std::endl;
          goto searchForBall;
        }

        if(smallBallAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees()))
        {
          std::cout<<"turnToBall -> choose_target: turned to ball"<<std::endl;
          goto choose_target;
        }
      }

      action
      {
        theActivitySkill(BehaviorStatus::realigning_to_ball);
        
        walkSpeed = decideWalkSpeed(dangerObstacleRadius, safetyObstacleRadius, slowerWalkSpeed, normalWalkSpeed);
        theKeyFrameArmsSkill(decideArmsPosition(safetyObstacleRadius), false);

        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(1.0f, walkSpeed, walkSpeed), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));

      }
    }

    state(choose_target)
    {
      footChosen = false;

      transition
      {

        if(DEBUG_MODE)
        {
          goto debug_state;
        }

        if(targetChosen)
        {
          targetChosen = false;

          std::cout<<"choose_target -> secondStepAlignment: target chosen"<<std::endl;
          goto secondStepAlignment;
        }
      }
      action
      {

        theActivitySkill(BehaviorStatus::choosing_target);

        walkSpeed = decideWalkSpeed(dangerObstacleRadius, safetyObstacleRadius, slowerWalkSpeed, normalWalkSpeed);
        theKeyFrameArmsSkill(decideArmsPosition(safetyObstacleRadius), false);

        goalTarget = theTaskController.currentBallDestination;
        chosenTarget = theTaskController.currentBallDestination;
        previousBallPosition = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        previousNearbyObstaclesCount = countNearbyObstacles(nearbyObstacleRadius);
        nearestObstacleGlobalPositionWhenPathWasComputed = getNearestObstacle(true);
      
        
        targetChosen = true;
        pathChangeTime = option_time;
        
        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        theStandSkill();

//NOTICE: the foot is chosen (based on the nearest obstacle position) in choose_target because choose_target is visited every time 
//the number or position of nearby obstacles changes
        footChosen = true;
        kickWithRightFoot = isRightFootUsedToKick();
        //kickWithRightFoot = false;

      }
    }

    state(debug_state)
    {
      action
      {
        theActivitySkill(BehaviorStatus::debug_standing);
        //theLookForwardSkill();          
        Vector2f relativePenaltyMark = theLibCheck.glob2Rel(theFieldDimensions.xPosOpponentPenaltyMark, 0.f).translation;
        if(relativePenaltyMark.x() >= 0)
        {
          if(relativePenaltyMark.y() > 0)
          {
            theParametricLookLeftAndRightSkill(0, abs(Angle(relativePenaltyMark.angle()).toDegrees()), 20, LEFT_AND_RIGHT_LOOKING_SPEED);
          }
          else
          {
            theParametricLookLeftAndRightSkill(abs(Angle(relativePenaltyMark.angle()).toDegrees()), 0, 20, LEFT_AND_RIGHT_LOOKING_SPEED);
          }
        }
        theStandSkill();
      }
    }

    state(walkToBall)
    {
      transition
      {

        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          std::cout<<"walkToBall -> searchForBall: ball not found"<<std::endl;
          goto searchForBall;
        }

        //if the LOCAL ball is inside the approach on the x axis 
        //AND the LOCAL ball is inside the approach range on the y axis 
        //AND the robot is aligned with the target
        float offsetBallYPosition;
        if(USE_FOOT_OFFSET)
        {
          if(kickWithRightFoot)
          {
            offsetBallYPosition = theBallModel.estimate.position.y() + BALL_KICK_OFFSET_Y;
          }
          else
          {
            offsetBallYPosition = theBallModel.estimate.position.y() - BALL_KICK_OFFSET_Y;
          }
        }
        else
        {
          offsetBallYPosition = theBallModel.estimate.position.y();
        }

        if(approachXRange.isInside(theBallModel.estimate.position.x())
        && approachYRange.isInside(offsetBallYPosition)
        && smallBallAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees()))
        {
          std::cout<<"walkToBall -> kick: OK to kick"<<std::endl;
          goto kick;
        }
        
        if(theBallModel.estimate.position.norm() < approachXRange.max
        && approachXRange.isInside(theBallModel.estimate.position.x())
        && !approachYRange.isInside(offsetBallYPosition))
        {
          //Case B: Robot is in the X range but not in the Y range
          std::cout<<"walkToBall -> secondStepAlignment: Case B"<<std::endl;
          goto secondStepAlignment;
        }
        
      }

      action
      {
        theActivitySkill(BehaviorStatus::reaching_ball);

        walkSpeed = decideWalkSpeed(dangerObstacleRadius, safetyObstacleRadius, slowerWalkSpeed, normalWalkSpeed);
        theKeyFrameArmsSkill(decideArmsPosition(safetyObstacleRadius), false);

        Vector2f ballPositionRelative = theBallModel.estimate.position;

        //Look for landmarks while walking to the ball (far). If the robot is in the left side of the field start looking right, else left
        
        if(LOOK_LEFT_AND_RIGHT_WHILE_WALKING_TO_BALL && theRobotPose.translation.x() > START_LOOKING_LEFT_AND_RIGHT_AT_X_COORDINATE)
        {
          callParametricLook();
        }
        else
        {
          theLookAtPointSkill(Vector3f(ballPositionRelative.x(), ballPositionRelative.y(), 0.f));
        }
        
        theWalkToTargetPathPlannerSkill(Pose2f(1.0f, walkToBallWalkSpeed, walkToBallWalkSpeed), theBallCarrierModel.staticApproachPoint());
      }
    }

//TOTEST
    state(firstStepAlignment)
    {
      transition
      {
        
        Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

        if(theBallModel.estimate.position.x() > FIRST_STEP_REALIGN_RANGE.max
        )
        {
          std::cout<<"firstStepAlignment -> secondStepAlignment"<<std::endl;
          goto secondStepAlignment;
        }

        if(theFieldBall.ballWasSeen(realignmentBallNotSeenTimeout))
        {
          goto searchForBall;
        }

      }
      action
      {
        theActivitySkill(BehaviorStatus::realigning_to_ball);

        Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

        walkSpeed = decideWalkSpeed(dangerObstacleRadius, safetyObstacleRadius, slowerWalkSpeed, normalWalkSpeed);
        theKeyFrameArmsSkill(decideArmsPosition(safetyObstacleRadius), false);

        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));

//TOTEST add robot turning towards the last known position of the ball while going backwards

        float firstStepXPos = (approachXRange.max - approachXRange.min)/2;

        theWalkAtRelativeSpeedSkill(Pose2f(theLibCheck.angleToTarget(globalBall.x(), globalBall.y()), -1.0, -globalBall.y()/3));
      }
    }


    state(secondStepAlignment)
    {
      transition
      {

        float offsetBallYPosition;
        if(USE_FOOT_OFFSET)
        {
          if(kickWithRightFoot)
          {
            offsetBallYPosition = theBallModel.estimate.position.y() + BALL_KICK_OFFSET_Y;
          }
          else
          {
            offsetBallYPosition = theBallModel.estimate.position.y() - BALL_KICK_OFFSET_Y;
          }
        }
        else
        {
          offsetBallYPosition = theBallModel.estimate.position.y();
        }

        if(theBallModel.estimate.position.norm() > approachXRange.max)
        {
          goto walkToBall;
        }

        //DEBUG_CODE(approachYRange.isInside(offsetBallYPosition));
        //DEBUG_CODE(offsetBallYPosition);
        if(approachXRange.isInside(theBallModel.estimate.position.x())
        && approachYRange.isInside(offsetBallYPosition)
        && smallBallAlignmentRange.isInside(Angle(theLibCheck.angleToTarget(theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y())).toDegrees()))
        {
          std::cout<<"secondStepAlignment -> kick: OK to kick"<<std::endl;
          goto kick;
        }

        if(state_time > realignmentTimeout)
        {
          std::cout<<"secondStepAlignment -> kick: TIMEOUT"<<std::endl;
          goto kick;
        }

//BELOW copied from the KICK state
        //Case E: Robot not in the X range but is behind the ball and is not in the Y range
        DEBUG_CODE(theBallModel.estimate.position.x() < approachXRange.min);
        DEBUG_CODE(theBallModel.estimate.position.x());
        if(theBallModel.estimate.position.x() < approachXRange.min)
        {
          DEBUG_CODE(FIRST_STEP_REALIGN_RANGE.isInside(theBallModel.estimate.position.x()));
          if(FIRST_STEP_REALIGN_RANGE.isInside(theBallModel.estimate.position.x()))
          {
            DEBUG_CODE(!approachYRange.isInside(offsetBallYPosition));
            if(!approachYRange.isInside(offsetBallYPosition))
            {
              std::cout<<"kick -> firstStepAlignment: Case E"<<std::endl;
              goto firstStepAlignment; //go back
            }
          }
        }

        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          std::cout<<"secondStepAlignment -> searchForBall:"<<std::endl;
          goto searchForBall;
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
        DEBUG_CODE(theBallModel.estimate.position.x() < -theBallSpecification.radius*2);
        DEBUG_CODE(theBallModel.estimate.position.x());
        if(theBallModel.estimate.position.x() < -theBallSpecification.radius*2)
        {
          DEBUG_CODE(!approachYRange.isInside(offsetBallYPosition));
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
        
        walkSpeed = decideWalkSpeed(dangerObstacleRadius, safetyObstacleRadius, slowerWalkSpeed, normalWalkSpeed);
        theKeyFrameArmsSkill(decideArmsPosition(safetyObstacleRadius), false);
        
        float localGoalYCoordinate = theLibCheck.glob2Rel(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal).translation.y();

        if(LOOK_LEFT_AND_RIGHT_WHILE_ALIGNING && theRobotPose.translation.x() > START_LOOKING_LEFT_AND_RIGHT_AT_X_COORDINATE)
        {
          callParametricLook();
        }
        else
        {
          theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        }
        

        Vector2f staticApproachPoint = theBallCarrierModel.staticApproachPoint().translation;
        Pose2f secondStepXPos = theLibCheck.glob2Rel(staticApproachPoint.x(), staticApproachPoint.y());
        Pose2f secondStepPoseRelative = Pose2f(theLibCheck.angleToTarget(theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y()), secondStepXPos.translation.x(), secondStepXPos.translation.y());
        if(USE_FOOT_OFFSET)
        {
          if(kickWithRightFoot)
          {
            secondStepPoseRelative.translation.y() += BALL_KICK_OFFSET_Y;
          }
          else
          {
            secondStepPoseRelative.translation.y() -= BALL_KICK_OFFSET_Y;
          }
        }
        
        theWalkToTargetSkill(Pose2f(1.0f, walkSpeed, walkSpeed), secondStepPoseRelative);
      }
    }


    state(kick)
    {
      transition
      {

        float offsetBallYPosition;
        if(USE_FOOT_OFFSET)
        {
          if(kickWithRightFoot)
          {
            offsetBallYPosition = theBallModel.estimate.position.y() + BALL_KICK_OFFSET_Y;
          }
          else
          {
            offsetBallYPosition = theBallModel.estimate.position.y() - BALL_KICK_OFFSET_Y;
          }
        }
        else
        {
          offsetBallYPosition = theBallModel.estimate.position.y();
        }

        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          std::cout<<"kick -> searchForBall: ball not seen TIMEOUT"<<std::endl;
          goto searchForBall;
        }

        if(theLibCheck.distance(previousBallPosition, theLibCheck.rel2Glob(theBallModel.estimate.position.x(), offsetBallYPosition)) > ballPositionChangedThreshold)
        {
          std::cout<<"kick -> choose_target: ball moved too much"<<std::endl;
          goto choose_target;
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
        
        DEBUG_CODE(theBallModel.estimate.position.x());
        DEBUG_CODE(offsetBallYPosition);
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
        DEBUG_CODE(theBallModel.estimate.position.x() < approachXRange.min);
        if(theBallModel.estimate.position.x() < approachXRange.min)
        {
          DEBUG_CODE(FIRST_STEP_REALIGN_RANGE.isInside(theBallModel.estimate.position.x()));
          if(FIRST_STEP_REALIGN_RANGE.isInside(theBallModel.estimate.position.x()))
          {
            DEBUG_CODE(!approachYRange.isInside(offsetBallYPosition));
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

//NOTICE: I've put the transition to the search state before the transitions to the G and F states because it is unlikely that 
//we know that the ball is behind us
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          std::cout<<"kick -> searchForBall:"<<std::endl;
          goto searchForBall;
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
        DEBUG_CODE(theBallModel.estimate.position.x() < -theBallSpecification.radius*2);
        if(theBallModel.estimate.position.x() < -theBallSpecification.radius*2)
        {
          DEBUG_CODE(!approachYRange.isInside(offsetBallYPosition));
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

//If we get here, the robot is in case A of the diagram aka perfectly aligned. Now we check the alignment angle
        //Case A but WRONG ANGLE: Robot is perfectly aligned but with a wrong angle
        /*if(!smallBallAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees()) && option_time - timeFromLastRealignment > angleRealignmentTimeout)
        {
          goto secondStepAlignment;
        }*/
      }

      action
      {
        theActivitySkill(BehaviorStatus::kicking_to_dynamic_target);
        
        Vector2f nearestObs = getNearestObstacle();
        

        walkSpeed = decideWalkSpeed(dangerObstacleRadius, safetyObstacleRadius, slowerWalkSpeed, normalWalkSpeed);
        theKeyFrameArmsSkill(decideArmsPosition(safetyObstacleRadius), false);

        Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

        //Kick with the foot that is nearest to the obstacle that is nearest to the ball
        if(WALK_ON_BALL /*&& theLibCheck.distance(globalBall, chosenTarget) > USE_IN_WALK_KICKS_FOR_DISTANCE_MORE_THAN*/)
        {
          theWalkToTargetSkill(Pose2f(1.0f, walkSpeed, walkSpeed), theBallModel.estimate.position);
        }
        else
        {
          if(kickWithRightFoot)
          {
            theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::right), Pose2f(Angle::fromDegrees(0.f), theBallModel.estimate.position.x(), theBallModel.estimate.position.y() + BALL_KICK_OFFSET_Y));
            //theKickSkill(false, (float) 9000.f, false); // parameters: (kick_type, mirror, distance, armsFixed)
          }
          else
          {
            theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(Angle::fromDegrees(0.f), theBallModel.estimate.position.x(), theBallModel.estimate.position.y() - BALL_KICK_OFFSET_Y));
            //theKickSkill(false, (float) 9000.f, false); // parameters: (kick_type, mirror, distance, armsFixed)
          }
        }
        
        if(LOOK_LEFT_AND_RIGHT_WHILE_KICKING && theRobotPose.translation.x() > START_LOOKING_LEFT_AND_RIGHT_AT_X_COORDINATE)
        {
          callParametricLook();
        }
        else
        {
          theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        }
        theGoalTargetSkill(goalTarget);
        theSetTargetSkill(chosenTarget);
      }
    }

    state(searchForBall)
    {
      footChosen = false;

      transition
      {
        if(theFieldBall.ballWasSeen(ballNotSeenTimeout)) 
        {
          isSearchingToggle = false;
          fullLoopRevolution = false;
          
          std::cout<<"searchForBall -> choose_target: ball found"<<std::endl;
          goto choose_target;
        }

        if (goToCenterWhenSearchingForBall
                && fullLoopRevolution 
                    && !theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          fullLoopRevolution = false;

          std::cout<<"searchForBall -> lookForBallAtCenter: ball not found"<<std::endl;
          goto lookForBallAtCenter;
        }
      }

      action
      {
        
        theActivitySkill(BehaviorStatus::searching_for_ball);
        Pose2f robotPose = theRobotPose;
        previousBallPosition = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        Vector2f lastPercievedBallPosition = theLibCheck.rel2Glob(theBallModel.lastPerception.x(),theBallModel.lastPerception.y() ).translation;

        walkSpeed = decideWalkSpeed(dangerObstacleRadius, safetyObstacleRadius, slowerWalkSpeed, normalWalkSpeed);
        theKeyFrameArmsSkill(decideArmsPosition(safetyObstacleRadius), false);

        float angleToBall = Angle(theLibCheck.angleToTarget( lastPercievedBallPosition.x(), 
                                                       lastPercievedBallPosition.y())).toDegrees();

        if(angleToBall > 0)
        {
          theLookLeftAndRightSkill();
        }
        else
        {
          theLookRightAndLeftSkill();
        }

        if (!isSearchingToggle)
        {

          startingRobotAngleWhenSearching = Angle(theRobotPose.rotation);
          rotationVel = angleToBall > 0 ? walkSpeed : -walkSpeed;
          isSearchingToggle = true;          
        }
        theWalkAtRelativeSpeedSkill(Pose2f(rotationVel, 0.f, 0.f));
             
        
        float angleDiff = theRobotPose.rotation.toDegrees() - startingRobotAngleWhenSearching.toDegrees();

        if(rotationVel > 0)
        {
          fullLoopRevolution = angleDiff < 0 
                                  && Rangef(-60,60).isInside(angleDiff)
                                      && !Rangef(-10,10).isInside(angleDiff);
        }
        else
        {
          fullLoopRevolution = angleDiff > 0
                                  && Rangef(-60,60).isInside(angleDiff)
                                      && !Rangef(-10,10).isInside(angleDiff);
        }
        
        
      }
    }

    state (lookForBallAtCenter)
    {
      transition 
      {
        if(theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          isSearchingToggle = false;

          std::cout<<"lookForBallAtCenter -> walkToBall: ball found"<<std::endl;
          goto walkToBall;
        }

      }
      action
      {
        theActivitySkill(BehaviorStatus::reaching_ball);


        theLookLeftAndRightSkill();
        if(theLibCheck.distance(theRobotPose, Pose2f(0.f, -850.f, 0.f)) > 300.f)
        {
          theWalkToTargetPathPlannerSkill(Pose2f(1.0f, walkSpeed, walkSpeed), Pose2f(0.f, -850.f, 0.f));
        }
        else
        {
          walkSpeed = decideWalkSpeed(dangerObstacleRadius, safetyObstacleRadius, slowerWalkSpeed, normalWalkSpeed);
          theKeyFrameArmsSkill(decideArmsPosition(safetyObstacleRadius), false);
          theWalkAtRelativeSpeedSkill(Pose2f(-theRobotPose.rotation,0.f,0.f));
        }
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

  ArmKeyFrameRequest::ArmKeyFrameId decideArmsPosition(float maxObstacleRadius)
  {
    Vector2f nearestObs = getNearestObstacle(true);
    if(theLibCheck.distance(theRobotPose.translation, nearestObs) > maxObstacleRadius
      || theRobotPose.translation.x() > nearestObs.x() + robotArmsRange
      || !putArmsOnBack)
    {
      return ArmKeyFrameRequest::useDefault;
    }
    else 
    {
      return ArmKeyFrameRequest::back;
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

  void callParametricLook()
  {
    /*
    Vector2f relativePenaltyMark = theLibCheck.glob2Rel(theFieldDimensions.xPosOpponentPenaltyMark, 0.f).translation;
    if(relativePenaltyMark.x() >= 0)
    {
      if(relativePenaltyMark.y() < 0)
      {
        theParametricLookLeftAndRightSkill(0, abs(Angle(relativePenaltyMark.angle()).toDegrees()), 20, LEFT_AND_RIGHT_LOOKING_SPEED);
      }
      else
      {
        theParametricLookLeftAndRightSkill(abs(Angle(relativePenaltyMark.angle()).toDegrees()), 0, 20, LEFT_AND_RIGHT_LOOKING_SPEED);
      }
    }
    */

    float lookingAngle;
    if(theRobotPose.translation.x() <= theFieldDimensions.xPosOpponentPenaltyMark)
    {
        lookingAngle = theLibCheck.mapToInterval(theRobotPose.translation.x(), 0, theFieldDimensions.xPosOpponentPenaltyMark, 0, MAX_LOOKING_ANGLE);
    }
    else
    {
        lookingAngle = MAX_LOOKING_ANGLE - theLibCheck.mapToInterval(theRobotPose.translation.x(), theFieldDimensions.xPosOpponentPenaltyMark, theFieldDimensions.xPosOpponentGroundline, 0, 90);
    }

    /*if(theRobotPose.translation.y() > 0)
    {
        theParametricLookLeftAndRightSkill(0, lookingAngle, 20, LEFT_AND_RIGHT_LOOKING_SPEED);
    }
    else
    {
        theParametricLookLeftAndRightSkill(lookingAngle, 0, 20, LEFT_AND_RIGHT_LOOKING_SPEED);
    }*/

    Vector2f relativePenaltyMark = theLibCheck.glob2Rel(theFieldDimensions.xPosOpponentPenaltyMark, 0.f).translation;
    if(relativePenaltyMark.x() >= 0)
    {
      //if(relativePenaltyMark.y() > 0)
      //{
        theParametricLookLeftAndRightSkill(0, lookingAngle, 20, LEFT_AND_RIGHT_LOOKING_SPEED);
      //}
      //else
      //{
      //  theParametricLookLeftAndRightSkill(lookingAngle, 0, 20, LEFT_AND_RIGHT_LOOKING_SPEED);
      //}
    }
    else
    {
      theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
    }
  }
};

MAKE_CARD(ApproachAndCarryToGoalWithTwoStepRealignmentCard);
