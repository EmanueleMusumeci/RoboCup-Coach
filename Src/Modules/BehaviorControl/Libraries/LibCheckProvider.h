/**
 * @file LibCheckProvider.h
 *
 * Performs some checks of the behavior control module such as
 * - How often is a MotionRequest set?
 * - How often is a HeadMotionRequest set?
 *
 * @author Tim Laue
 */

#pragma once

#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/BallCarrierModel/BallCarrierModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/TeamBallModel.h"


#include "Representations/Configuration/BallSpecification.h"


#include "Tools/Module/Module.h"
#include <math.h>

MODULE(LibCheckProvider,
{,
  USES(ActivationGraph),
  REQUIRES(BallModel),
  REQUIRES(FieldBall),
  REQUIRES(FrameInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamBallModel),
  REQUIRES(TeamPlayersModel),
  REQUIRES(ObstacleModel),
  USES(BallSpecification),
  USES(BallCarrierModel),
  USES(TeamActivationGraph),
  USES(TeamBehaviorStatus),
  USES(TeamData),
  USES(Role),
  PROVIDES(LibCheck),
  LOADS_PARAMETERS(
  {,
    (std::vector<int>) notSetCheck,       /** Assert that a request has been set at least once */
    (std::vector<int>) multipleSetCheck,  /** Print a warning if an assert has not been set at least once */
    (bool) assertValidWalkRequest,        /** Asserts that there are no strange walk parameters */
  }),
});

class LibCheckProvider : public LibCheckProviderBase
{
private:
  int callCounters[LibCheck::numOfCheckedOutputs]; /**< The counters for different checks */
  bool setArmsInThisFrame[Arms::numOfArms]; /**< This arm was set in this frame */

  /**
   * Updates LibCheck
   * @param libCheck The representation provided
   */
  void update(LibCheck& libCheck) override;

  /** Resets all status information */
  void reset();

  /**
   * Checks whether a behavior part set all its outputs
   * @param activationGraph The activation graph of the behavior part
   * @param start The first output ID to be checked
   * @param end The first output ID not to be checked after \c start
   */
  void checkOutputs(const ActivationGraph& activationGraph, LibCheck::CheckedOutput start, LibCheck::CheckedOutput end) const;

  /**
   * Checks whether the motion request is valid
   * @param activationGraph The activation graph of the individual behavior
   * @param theMotionRequest The motion request to check for validity
   */
  void checkMotionRequest(const ActivationGraph& activationGraph, const MotionRequest& theMotionRequest) const;

  /** Increments one counter */
  void inc(LibCheck::CheckedOutput outputToCheck);

  /**
   * Serializes an activation graph to a string
   * @param activationGraph The activation graph to convert
   * @return The compressed string that represents the activation graph
   */
  std::string getActivationGraphString(const ActivationGraph& activationGraph) const;

  /**
   * Provides the Pose to reach in ready state for each robot
   * @return the target pose
   */
  Pose2f myReadyPosition() const;

  /**
   * Provides the distance between two pose2f
   * @param p1 the first point
   * @param p2 the second point
   * @return the distance between p1 and p2
   */
  float distance(Pose2f p1, Pose2f p2) const;

  /**
   * Provides the distance between two points (x1,y1) and (x2,y2)
   * @param x1 the first point
   * @param y1 the second point
   * @param x2 the first point
   * @param y2 the second point
   * @return the distance between p1 and p2
   */
  float distance(float x1, float y1, float x2, float y2) const;

 Vector2f goalTarget ();


 Pose2f glob2Rel(float x, float y);
 Pose2f rel2Glob(float x, float y);
 float distanceToLine(Vector2f objectToCheck, Vector2f linePoint1, Vector2f linePoint2);


 float angleToBall;
 float radiansToDegree(float x);


 float angleToTarget(float x, float y);   // TODO This is to check orientation wrt to target x = 4500 y = 3000 to left and -3000 to right
 float norm(float x, float y);

 //Maps value from interval [fromIntervalMin, fromIntervalMax] to interval [toIntervalMin, toIntervalMax]
 float mapToInterval(float value, float fromIntervalMin, float fromIntervalMax, float toIntervalMin, float toIntervalMax);

 public: LibCheckProvider();

};
