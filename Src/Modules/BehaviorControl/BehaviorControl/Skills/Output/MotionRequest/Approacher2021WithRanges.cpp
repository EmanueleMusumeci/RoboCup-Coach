/**
 * @file Approacher2021WithRanges.cpp
 *
 * Implementation of the approaching behavior employed in Challenge 3
 * of RoboCup 2021. (formerly called ChallengeThreeApproach.cpp)
 * 
 * Original idea: https://docs.google.com/drawings/d/1FcS2yrCbGkUmbWM1GRGHuXTYnEhcrbkovdwFayTGWkc/edit?usp=sharing
 * 
 * @author Francesco Petri, Fabian Fonseca, Emanuele Musumeci (base structure)
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/Configuration/BallSpecification.h"

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Math/BHMath.h"
#include "Platform/SystemCall.h"
#include <iostream>

#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

#define LEFT_FOOT_OFFSET_Y 65.f
#define FIRST_STEP_REALIGNMENT_POINT_LATERAL_OFFSET 140.f

#define __STRINGIFY_I(arg) #arg
#define __STRINGIFY(arg) __STRINGIFY_I(arg)
#define DEBUG_VALUE(code) if(USE_DEBUG_PRINTS)\
  std::cout<<__STRINGIFY(code)<<": "<<code<<std::endl;

#define DEBUG_CODE(code) if(USE_DEBUG_PRINTS)\
  std::cout<<__STRINGIFY(code)<<": "<<std::to_string(code)<<std::endl;


#define DEBUG_VECTOR2F(vector) if(USE_DEBUG_PRINTS)\
  std::cout<<__STRINGIFY(vector)<<": ("<<std::to_string(vector.x())<<", "<<std::to_string(vector.y())<<")"<<std::endl;
  
#define DEBUG_PRINT(message) {std::cout<<message<<std::endl;}
#define DEBUG_SAY(msg) if (USE_DEBUG_PRINTS) SystemCall::say(msg)



SKILL_IMPLEMENTATION(Approacher2021WithRangesImpl,
{,
  IMPLEMENTS(Approacher2021WithRanges),
  REQUIRES(LibCheck),
  REQUIRES(FieldBall),
  REQUIRES(RobotPose),
  REQUIRES(BallModel),
  REQUIRES(BallSpecification),
  
  CALLS(Activity),

  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(LookLeftAndRight),
  CALLS(WalkAtAbsoluteSpeed),
  CALLS(Stand),

  CALLS(WalkToTarget),
  CALLS(WalkToTargetPathPlanner),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToApproach),


  CALLS(Kick),
  CALLS(InWalkKick),

  //TODO: check back here and remove any unused parameters
  //      same for the functions below
  //      also remove params from the cfg
  LOADS_PARAMETERS(
  {,
    (int) initialWaitTime,
    (int) realignmentTimeout,


    //The smaller angle range is used when aligning precisely just before the kick 
    (Rangef) smallerBallAlignmentRange,

    (bool) USE_DEBUG_PRINTS,

    (float) walkToBallFar_dontLookAround_squareRadius,
    (unsigned char) seenPercentageThreshold,
    (float) distanceThresholdToSkipFinalAngleCheck,
  }),
});

class Approacher2021WithRangesImpl : public Approacher2021WithRangesImplBase
{
  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }
  
  Pose2f computeApproachPoint(float radius, float angle)
  {
    Vector2f ballPositionGlobal = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
    float dynamicOffsetX = (float) (radius * cos(pi + angle));
    float dynamicOffsetY = (float) (radius * sin(pi + angle));
    return Pose2f(-angle, ballPositionGlobal.x() + dynamicOffsetX, ballPositionGlobal.y() - dynamicOffsetY);
  }

  // check if we are currently seeing the ball
  bool canSeeBall() {
    return theBallModel.seenPercentage >= seenPercentageThreshold;
  }







  option(Approacher2021WithRanges)
  {
    initial_state(start)
    {
      DEBUG_PRINT("APPROACH_AND_KICK: start")
      transition
      {
        if(state_time > initialWaitTime)
        {
          DEBUG_PRINT("start -> walkToBall: TIMEOUT")
          goto walkToBall;
          
        }
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(turnToBall)
    {
      transition
      {        
        if(p.smallBallAlignmentRange.isInside(calcAngleToTarget(p.target.translation).toDegrees()))
        {
          DEBUG_PRINT("turnToBall -> walkToBall: aligned to ball")
          goto walkToBall;
        }
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(p.walkSpeed, p.walkSpeed, p.walkSpeed), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
      }
    }

    state(walkToBall)
    {
      transition
      {
        Vector2f ballRelative = theBallModel.estimate.position;
        float offsetBallYPosition = ballRelative.y() - p.offsetY;

        Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

        //if the LOCAL ball is inside the approach on the x axis 
        //AND the LOCAL ball is inside the approach range on the y axis 
        //AND the robot is aligned with the target
        /*
        if(
          p.approachXRange.isInside(ballRelative.x())
          && 
          p.approachYRange.isInside(offsetBallYPosition)
          && 
          smallerBallAlignmentRange.isInside(calcAngleToTarget(p.target.translation).toDegrees())
          )
        {
          DEBUG_PRINT("walkToBall -> approachToKick: OK to approach")
          goto approachToKick;
        }

        if(ballRelative.norm() < p.approachXRange.max
        && p.approachXRange.isInside(ballRelative.x())
        && !p.approachYRange.isInside(offsetBallYPosition)
        && theRobotPose.translation.x()+50.f < globalBall.x())
        {
          //Case B: Robot is in the X range but not in the Y range
          DEBUG_PRINT("walkToBall -> secondStepAlignment: kick Case B")
          goto secondStepAlignment;
        }
        */

        if((theRobotPose.translation - globalBall).norm() < p.approachXRange.max && theRobotPose.translation.x() < globalBall.x()){
          DEBUG_PRINT("walktoball --> firststep")
          goto firstStepAlignment;
        }
      }

      action
      {
        Vector2f ballRelative = theBallModel.estimate.position;
        Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

        //If you are near look at the ball, else look left and right
        if (!p.allowLookAround || ballRelative.squaredNorm() <= walkToBallFar_dontLookAround_squareRadius) {
          theLookAtPointSkill(Vector3f(ballRelative.x(), ballRelative.y(), 0.f));
        }
        else {
          theLookLeftAndRightSkill();
        }

        float ballToTargetAngle = theLibCheck.angleBetweenPoints(globalBall, p.target.translation);
        Pose2f staticApproachPoint = computeApproachPoint(p.approachRadius, ballToTargetAngle);
        //theWalkToTargetPathPlannerSkill(Pose2f(p.walkSpeed, p.walkSpeed, p.walkSpeed), staticApproachPoint);
        Vector2f relativeApproachPoint = theLibCheck.glob2Rel(staticApproachPoint.translation.x(), staticApproachPoint.translation.y()).translation;
        theWalkToTargetSkill(Pose2f(p.walkSpeed, p.walkSpeed, p.walkSpeed), Pose2f(relativeApproachPoint.x(), relativeApproachPoint.y()));
      }
    }

    state(firstStepAlignment)//Turn to ball and walk
    {
      Pose2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
      Angle angle_to_ball = calcAngleToTarget(globalBall);
      transition
      {
        
        if(state_time > 3000){
          DEBUG_PRINT("firststep -> Apprach_and: timeout")
          goto approachToKick;
        }
        if((std::abs(angle_to_ball.toDegrees()) < 20_deg) && theRobotPose.translation.x() - 50.f < globalBall.translation.x()){
          DEBUG_PRINT("firststep -> Apprach_and: condizioni")
          goto approachToKick;
        }
        if(theBallModel.estimate.position.norm() > p.approachXRange.max * 1.6f){
          DEBUG_PRINT("firststep -> walktoball: outofrange")
          goto walkToBall;
        }
      }
      
      action
      {
        float y_vel = 200;

        float ang_vel = 0.8;

        if(std::abs(angle_to_ball) > 40_deg){
          ang_vel = 1.2;
        }
        if(theRobotPose.translation.y() > globalBall.translation.y()){
          y_vel = -y_vel;
        }
        if(angle_to_ball < 0.f){
          ang_vel = -ang_vel;
        }
        theWalkAtAbsoluteSpeedSkill(Pose2f(ang_vel,150,y_vel));
        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));

      }
    }

    state(secondStepAlignment)
    {
      Vector2f ballRelative = theBallModel.estimate.position;
      float offsetBallYPosition = ballRelative.y() + p.offsetY;
      
      //Global ball
      Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

      //Angle of the line passing through the target point and the target wrt angle origin
      float ballToTargetAngle = theLibCheck.angleBetweenPoints(globalBall, p.target.translation);
      Vector2f globalApproachPoint = computeApproachPoint(p.approachRadius, ballToTargetAngle).translation;
      
      //TODO VERIFY
      //To avoid angle errors, let's use a target that is on the same line but nearer (at a distance of 2 meters)
      Vector2f near_target = Eigen::ParametrizedLine<float, 2>::Through(globalApproachPoint, p.target.translation).pointAt(pow(10, 9)*2);
      near_target = p.target.translation;
    //X, Y offsets
      Vector2f relativeApproachPoint = theLibCheck.glob2Rel(globalApproachPoint.x(), globalApproachPoint.y()).translation;
      //DESIRED approach point (including offsets). NOTICE: this is where right the foot has to be
      Vector2f relativeApproachPointWithOffsets = Vector2f(relativeApproachPoint.x(), relativeApproachPoint.y() + p.offsetY);
      //DESIRED global approach point
      Vector2f globalApproachPointWithOffsets = theLibCheck.rel2Glob(relativeApproachPoint.x(), relativeApproachPoint.y()).translation;


      //ANGLES
      //DESIRED angle for the robot, use the nearer target to avoid angle errors
      float angleBetweenGlobalApproachPointWithOffsetsAndTarget = theLibCheck.angleBetweenPoints(globalApproachPointWithOffsets, near_target);

      Vector2f globalRobotFootOffset = theLibCheck.rel2Glob(0.f, -LEFT_FOOT_OFFSET_Y).translation;
      Vector2f robotFootPosition = theRobotPose.translation + globalRobotFootOffset;

      //CURRENT angle of the robot foot
      float currentAngle = theLibCheck.angleBetweenPoints(robotFootPosition, near_target);

       
      //FIRST STEP REALIGNMENT
       //Line connecting the ball to the target
       Eigen::ParametrizedLine<float, 2> ballToTargetLine = Eigen::ParametrizedLine<float, 2>::Through(globalBall, p.target.translation);
       

       Vector2f ballToTargetLinePerpendicularDirection = ballToTargetLine.direction().rotate(pi/2);

       //line that is perpendicular to the ball-target-line (rotated 90 degrees counter-clockwise) 
       Eigen::ParametrizedLine<float, 2> ballToTargetPerpendicularLine = Eigen::ParametrizedLine<float, 2>(globalBall, ballToTargetLinePerpendicularDirection);
       
       Vector2f leftFrontRealignmentPoint = ballToTargetPerpendicularLine.pointAt(FIRST_STEP_REALIGNMENT_POINT_LATERAL_OFFSET);
       Vector2f rightFrontRealignmentPoint = ballToTargetPerpendicularLine.pointAt(-FIRST_STEP_REALIGNMENT_POINT_LATERAL_OFFSET);
      


       //Point that is on the opposite side of the ball wrt the target
       Vector2f ballFarPoint = ballToTargetLine.pointAt(-theBallSpecification.radius);

       //Line that is perpendicular to the prevous one and tangent to the ball on the far side wrt the target (rotated 90 degrees counter-clockwise)
       Eigen::ParametrizedLine<float, 2> ballFarTangent = Eigen::ParametrizedLine<float, 2>(ballFarPoint, ballToTargetLinePerpendicularDirection);
       
       Vector2f leftRearRealignmentPoint = ballFarTangent.pointAt(FIRST_STEP_REALIGNMENT_POINT_LATERAL_OFFSET);
       Vector2f rightRearRealignmentPoint = ballFarTangent.pointAt(-FIRST_STEP_REALIGNMENT_POINT_LATERAL_OFFSET);
       
       
       Eigen::ParametrizedLine<float, 2> ballToTargetLineLeftParallel = Eigen::ParametrizedLine<float, 2>::Through(leftRearRealignmentPoint, leftFrontRealignmentPoint);
       Eigen::ParametrizedLine<float, 2> ballToTargetLineRightParallel = Eigen::ParametrizedLine<float, 2>::Through(rightRearRealignmentPoint, rightFrontRealignmentPoint);

      transition
      {

       
        if(ballRelative.norm() > p.approachXRange.max)
        {
          DEBUG_PRINT("secondStepAlignment -> walkToBall: out of range")
          goto walkToBall;
        }
        
          //DO NOT TOUCH
        float angleError;
        float footAngleError = -angleBetweenGlobalApproachPointWithOffsetsAndTarget -currentAngle;
        float centralAngleError = -angleBetweenGlobalApproachPointWithOffsetsAndTarget -theRobotPose.rotation;
        if(std::abs(centralAngleError) < pi/5)
        {
          angleError = centralAngleError;
        }
        else
        {
          angleError = footAngleError;
        }
        //

        //std::cout<<"Approacher2021WithRanges: secondStepAlignment"<<std::endl;
        DEBUG_CODE(p.approachXRange.isInside(ballRelative.x()));
        DEBUG_CODE(p.approachYRange.isInside(offsetBallYPosition));
        DEBUG_CODE(p.smallBallAlignmentRange.isInside(Angle(angleError).toDegrees()));
        if(p.approachXRange.isInside(ballRelative.x())
        && p.approachYRange.isInside(offsetBallYPosition)
        && p.smallBallAlignmentRange.isInside(Angle(angleError).toDegrees()))
        {
          //If using the InWalkKick, let the engine deal with the final alignment, else use the approachToKick state for higher precision
          if(p.inWalkKickAtTheEnd)
          {
            DEBUG_PRINT("secondStepAlignment -> readyToKick: aligned to ball")
            goto ready_to_kick;
          }
          else
          {
            DEBUG_PRINT("secondStepAlignment -> approachToKick: aligned to ball")
            goto approachToKick;
          }
        }

        //NEWLY ADDED
        if(state_time > realignmentTimeout)
        {
          DEBUG_PRINT("secondStepAlignment -> approachToKick: TIMEOUT")
          goto approachToKick;
        }

        
        /*
        //Case F: Robot is between the target and the ball

        if(ballRelative.x() > theBallSpecification.radius*2)
        {
          if(p.approachYRange.isInside(offsetBallYPosition))
          {
            DEBUG_PRINT("secondStepAlignment -> walkToBall: Case F")
            goto walkToBall;
          }
        }
      */
    //BELOW copied from the KICK state
        //Case E: Robot not in the X range but is behind the ball and is not in the Y range 
        /*
        if(ballRelative.x() < p.approachXRange.min)
        {
          if(Rangef(-theBallSpecification.radius*2, theBallSpecification.radius*2).isInside(ballRelative.x()))
          {
            if(!p.approachYRange.isInside(offsetBallYPosition))
            {
              DEBUG_PRINT("secondStepAlignment -> firstStepAlignment: Case E")
              goto firstStepAlignment; //go back
            }
          }
        }
        */

        /*
        //Case G: the robot is between the target and the ball and to the side wrt the ball
        if(ballRelative.x() < -theBallSpecification.radius*2)
        {
          if(!p.approachYRange.isInside(offsetBallYPosition))
          {
            DEBUG_PRINT("secondStepAlignment -> firstStepAlignment: Case G")
            goto firstStepAlignment;
          }
        }
        */

       /*
       if(ballRelative.x() < -theBallSpecification.radius*2)
       {
          goto firstStepAlignment;
       }
       */
      }

      action
      {
        if (!p.allowLookAround || canSeeBall()) {
          theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        }
        else {
          theLookLeftAndRightSkill();
        }
                
        //Pose2f approachPoint = Pose2f(theLibCheck.angleToTarget(p.target.translation.x(), p.target.translation.y()), relativeApproachPoint.x(), relativeApproachPoint.y() + p.offsetY);
        Pose2f approachPoint = Pose2f(angleBetweenGlobalApproachPointWithOffsetsAndTarget, relativeApproachPointWithOffsets.x(), relativeApproachPointWithOffsets.y());

        Vector2f normalizedDirection = relativeApproachPointWithOffsets.normalized();

        DEBUG_VECTOR2F(relativeApproachPointWithOffsets);
        DEBUG_VALUE(Angle(angleBetweenGlobalApproachPointWithOffsetsAndTarget).toDegrees());
        DEBUG_VALUE(Angle(currentAngle).toDegrees());
        //float angleError = -angleBetweenGlobalApproachPointWithOffsetsAndTarget - theRobotPose.rotation;
        
        float angleError;
        float footAngleError = -angleBetweenGlobalApproachPointWithOffsetsAndTarget -currentAngle;
        float centralAngleError = -angleBetweenGlobalApproachPointWithOffsetsAndTarget -theRobotPose.rotation;
        if(std::abs(centralAngleError) < pi/5)
        {
          //DO NOT TOUCH
          angleError = centralAngleError;
          //
        }
        else
        {
          angleError = footAngleError;
        }
        
        
        DEBUG_VALUE(Angle(angleError).toDegrees());
        DEBUG_VALUE(Angle(theRobotPose.rotation).toDegrees());
        
        /*
        float angularSpeed = p.alignmentSpeed;
        if(angleError < Angle(0.f).toDegrees())
        {
          angularSpeed *= -1.f;
        }
        */

        Vector2f speed = Vector2f(normalizedDirection.x() * p.alignmentSpeed, normalizedDirection.y() * p.kickApproachSpeed);
        DEBUG_VECTOR2F(speed);
        theWalkAtRelativeSpeedSkill(Pose2f(angleError, speed.x(), speed.y()));
        //theWalkToTargetSkill(Pose2f(p.normalWalkSpeed, p.normalWalkSpeed, p.normalWalkSpeed), Pose2f(angleError, relativeApproachPointWithOffsets.x(), relativeApproachPointWithOffsets.y()));
      }
    }

    //Approach the ball precisely (last robot movements before the kick)
    state(approachToKick)
    {
      Vector2f ballRelative = theBallModel.estimate.position;
      float offsetBallYPosition = ballRelative.y() + p.offsetY;
      
      //Global ball
      Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

      //Angle of the line passing through the target point and the target wrt angle origin
      float ballToTargetAngle = theLibCheck.angleBetweenPoints(globalBall, p.target.translation);
      Vector2f globalApproachPoint = computeApproachPoint(p.approachRadius, ballToTargetAngle).translation;
      
    //TODO VERIFY
      //To avoid angle errors, let's use a target that is on the same line but nearer (at a distance of 2 meters)
      //Vector2f near_target = Eigen::ParametrizedLine<float, 2>::Through(globalApproachPoint, p.target.translation).pointAt(pow(10, 9)*2);
      
      Vector2f near_target = p.target.translation;
    //X, Y offsets
      Vector2f relativeApproachPoint = theLibCheck.glob2Rel(globalApproachPoint.x(), globalApproachPoint.y()).translation;
      //DESIRED approach point (including offsets). NOTICE: this is where right the foot has to be
      Vector2f relativeApproachPointWithOffsets = Vector2f(relativeApproachPoint.x(), relativeApproachPoint.y() + p.offsetY);
      //DESIRED global approach point
      Vector2f globalApproachPointWithOffsets = theLibCheck.rel2Glob(relativeApproachPoint.x(), relativeApproachPoint.y()).translation;


    //ANGLES
      //DESIRED angle for the robot, use the nearer target to avoid angle errors
      float angleBetweenGlobalApproachPointWithOffsetsAndTarget = theLibCheck.angleBetweenPoints(globalApproachPointWithOffsets, near_target);

      Vector2f globalRobotFootOffset = theLibCheck.rel2Glob(0.f, -LEFT_FOOT_OFFSET_Y).translation;
      Vector2f robotFootPosition = theRobotPose.translation + globalRobotFootOffset;

      //CURRENT angle of the robot foot
      float currentAngle = theLibCheck.angleBetweenPoints(robotFootPosition, near_target);
      
      float angleError;
      float footAngleError = -angleBetweenGlobalApproachPointWithOffsetsAndTarget -currentAngle;
      float centralAngleError = -angleBetweenGlobalApproachPointWithOffsetsAndTarget -theRobotPose.rotation;
      if(std::abs(centralAngleError) < pi/5)
      {
        //DO NOT TOUCH
        angleError = centralAngleError;
        //
      }
      else
      {
        angleError = footAngleError;
      }
        
      transition
      {
        /*
        //To allow hysteresis we slightly shrink values (ranges are "larger" this way)
        float hysteresis_shrink_factor = 0.9f;
        if(!p.approachXRange.isInside(ballRelative.x() * hysteresis_shrink_factor)
        || !p.approachYRange.isInside(offsetBallYPosition * hysteresis_shrink_factor)
        || !p.smallBallAlignmentRange.isInside(Angle(angleError).toDegrees() * hysteresis_shrink_factor))
        {
          DEBUG_PRINT("approachToKick -> secondStepAlignment: disaligned")
          goto secondStepAlignment;
        }
        */

       if(theBallModel.estimate.position.norm() > p.approachXRange.max * 1.6f){
          DEBUG_PRINT("approachToKick -> walktoball: outofrange")
          goto walkToBall;
        }

        //Vector2f ballRelative = theBallModel.estimate.position;
        //float offsetBallYPosition = ballRelative.y() - p.offsetY;

        //std::cout<<"Approacher2021WithRanges: approachToKick"<<std::endl;
        //DEBUG_CODE(!p.approachXRange.isInside(ballRelative.x()));
        //DEBUG_CODE(!p.approachYRange.isInside(offsetBallYPosition));
        //DEBUG_CODE(!p.smallBallAlignmentRange.isInside(Angle(theLibCheck.angleToTarget(p.target.translation.x(), p.target.translation.y())).toDegrees()));
        //if(!p.approachXRange.isInside(ballRelative.x())
        //|| !p.approachYRange.isInside(offsetBallYPosition)
        //|| !p.smallBallAlignmentRange.isInside(Angle(theLibCheck.angleToTarget(p.target.translation.x(), p.target.translation.y())).toDegrees()))
        //{
        //  goto secondStepAlignment;
        //}

        Rangef smallApproachXRange  = Rangef(p.offsetX-15.f, p.offsetX+5.f);
        Rangef smallApproachYRange  = Rangef(-p.offsetY-60.f, -p.offsetY+10.f);

        if(smallApproachXRange.isInside(ballRelative.x())
           && smallApproachYRange.isInside(ballRelative.y()))
        {
            // in the inWalkKick case we can go right away, if we want to kick we have to align better
            // also don't align if too close to target, angles would oscillate too much
            if (p.inWalkKickAtTheEnd
            || (theRobotPose.translation - p.target.translation).squaredNorm() < distanceThresholdToSkipFinalAngleCheck*distanceThresholdToSkipFinalAngleCheck
            || p.smallBallAlignmentRange.isInside(Angle(theLibCheck.angleToTarget(p.target.translation.x(), p.target.translation.y())).toDegrees())) {
              DEBUG_PRINT("approachToKick -> ready_to_kick: ready to kick")
              //goto stand_before_kick;
              goto ready_to_kick;
            }
        }
      }

      action
      {

        
        //theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        //theStandSkill();

        /*
        Pose2f ball = theBallModel.estimate.position;
        float x_ball = ball.translation.x();         
        float y_ball = ball.translation.y();         

        x_ball = x_ball - p.offsetX;
        y_ball = y_ball - p.offsetY;

        Pose2f localTarget = Pose2f(theLibCheck.angleToTarget(p.target.translation.x(), p.target.translation.y()), x_ball, y_ball);
        theWalkToTargetSkill(Pose2f(p.slowerWalkSpeed, p.slowerWalkSpeed, p.slowerWalkSpeed), localTarget);
        */

        //Pose2f approachPoint = Pose2f(theLibCheck.angleToTarget(p.target.translation.x(), p.target.translation.y()), relativeApproachPoint.x(), relativeApproachPoint.y() + p.offsetY);
        Pose2f approachPoint = Pose2f(angleBetweenGlobalApproachPointWithOffsetsAndTarget, relativeApproachPointWithOffsets.x(), relativeApproachPointWithOffsets.y());

        Vector2f normalizedDirection = relativeApproachPointWithOffsets.normalized();

        DEBUG_VECTOR2F(relativeApproachPointWithOffsets);
        DEBUG_VALUE(Angle(angleBetweenGlobalApproachPointWithOffsetsAndTarget).toDegrees());
        DEBUG_VALUE(Angle(currentAngle).toDegrees());
        //float angleError = -angleBetweenGlobalApproachPointWithOffsetsAndTarget - theRobotPose.rotation;
        
        float angleError;
        float footAngleError = -angleBetweenGlobalApproachPointWithOffsetsAndTarget -currentAngle;
        float centralAngleError = -angleBetweenGlobalApproachPointWithOffsetsAndTarget -theRobotPose.rotation;
        if(std::abs(centralAngleError) < pi/5)
        {
          //DO NOT TOUCH
          angleError = centralAngleError;
          //
        }
        else
        {
          angleError = footAngleError;
        }
        
        
        DEBUG_VALUE(Angle(angleError).toDegrees());
        DEBUG_VALUE(Angle(theRobotPose.rotation).toDegrees());
        float angularSpeed = p.alignmentSpeed;
        if(angleError < Angle(0.f).toDegrees())
        {
          angularSpeed *= -1.f;
        }

        Vector2f speed = Vector2f(normalizedDirection.x() * p.kickApproachSpeed, normalizedDirection.y() * p.kickApproachSpeed);
        //Vector2f speed = relativeApproachPointWithOffsets;
        DEBUG_VECTOR2F(speed);
        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));

        theWalkToApproachSkill(p.target, p.offsetX, -p.offsetY, false);
        
        //theWalkToTargetSkill(Pose2f(p.normalWalkSpeed, p.normalWalkSpeed, p.normalWalkSpeed), Pose2f(angleError, relativeApproachPointWithOffsets.x(), relativeApproachPointWithOffsets.y()));
      }
    }

    target_state(ready_to_kick)
    {
      transition
      {
        Vector2f ballRelative = theBallModel.estimate.position;

        Rangef smallApproachXRange  = Rangef(p.offsetX-15.f, p.offsetX+5.f);
        Rangef smallApproachYRange  = Rangef(-p.offsetY-60.f, -p.offsetY+10.f);
        
        //To allow hysteresis we slightly shrink values (ranges are "larger" this way)
        float hysteresis_shrink_factor = 0.8f;
        if(!(smallApproachXRange.isInside(ballRelative.x() * hysteresis_shrink_factor)
           || !smallApproachYRange.isInside(ballRelative.y() * hysteresis_shrink_factor)))
        {
            DEBUG_PRINT("readytokick ->  start")
            //goto stand_before_kick;
            goto start;
        }
      }
      action
      {
        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        if (p.inWalkKickAtTheEnd) {
          theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::right), Pose2f(Angle::fromDegrees(0.f), theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));
          if (!state_time) DEBUG_SAY("pow!");
        }
        
      }
    }
  }
};

MAKE_SKILL_IMPLEMENTATION(Approacher2021WithRangesImpl);
