/**
 * @file LibCheck.h
 *
 * This file defines a representation that checks some behavior control properties
 *
 * @author Daniel Krause
 */

#pragma once

#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Modeling/FreeGoalTargetableArea.h"
#include "Representations/BehaviorControl/BallCarrierModel/BallCarrierModel.h"
#include "Tools/Function.h"
#include "Tools/RobotParts/Arms.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Representations/Modeling/ObstacleModel.h"

STREAMABLE(LibCheck,
{
  ENUM(CheckedOutput,
  {,
    motionRequest,
    headMotionRequest,
    activity,
    passTarget,
    firstTeamCheckedOutput,
    teamActivity = firstTeamCheckedOutput,
    timeToReachBall,
    teammateRoles,
    role,
  });

  /** Increments one counter */
  FUNCTION(void(LibCheck::CheckedOutput outputToCheck)) inc;

  /** Indicates that an arm has been set */
  FUNCTION(void(Arms::Arm arm)) setArm;

  /** Checks whether an arm has been set */
  FUNCTION(bool(Arms::Arm arm)) wasSetArm;

  /** Performs checks for the individual behavior */
  FUNCTION(void(const MotionRequest& theMotionRequest)) performCheck;
  /** Provides the ready position for each robot **/
  FUNCTION(Pose2f()) myReadyPosition;

  /** Provides the distance between 2 Pose2f **/
  FUNCTION(float(Pose2f p1, Pose2f p2)) distance;

  /** Dummy goal target (median of goal line) */
  FUNCTION(Vector2f()) goalTarget;

  FUNCTION(float(float value, float fromIntervalMin, float fromIntervalMax, float toIntervalMin, float toIntervalMax)) mapToInterval;

  /** Performs checks for the team behavior */
  FUNCTION(void()) performTeamCheck;

  FUNCTION(float(float x, float y)) angleToTarget;
  FUNCTION(float(float x, float y)) norm;

  FUNCTION(Pose2f()) nearestTemmate;

  FUNCTION(Pose2f(float x, float y)) glob2Rel;
  FUNCTION(Pose2f(float theta, float x, float y)) glob2RelWithAngle;
  FUNCTION(float(float x))radiansToDegree;

  FUNCTION(Pose2f(float x, float y)) rel2Glob;
  FUNCTION(Vector2f()) getSupporterPosition;
  FUNCTION(Vector2f()) getJollyPosition;
  FUNCTION(Vector2f()) getDefenderPosition;
  FUNCTION(float(Vector2f objectToCheck, Vector2f linePoint1, Vector2f linePoint2)) distanceToLine;


  FUNCTION(float(Vector2f p1, Vector2f p2)) angleBetweenPoints,


  (int) timeSinceBallWasSeen,
  (float) angleToBall,
});
