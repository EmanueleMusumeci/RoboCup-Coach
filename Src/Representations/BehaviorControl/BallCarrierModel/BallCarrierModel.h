/**
 * @file Representations/BehaviorControl/BallCarrierModel.h
 *
 * Declaration of struct BallCarrierModel, that provides a model of the opponent goal
 * from the point of view of the striker, including obstacle coverage and
 * a score for the utility of each discretized targetable segment on the goal line
 *
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include "Representations/BehaviorControl/Libraries/LibPathPlanner.h"

/**
 * @struct BallCarrierModel
 *
 * This representation is used only for debugging purposes
 * 
 * Struct containing various modeling info useful to the ball carrier
 */


STREAMABLE(BallCarrierModel,
{

  /** Draws model on the field */
  void draw() const;
  
  STREAMABLE(Node,
  {
    Node() = default;
    Node(Vector2f center, float radius);
    ,
    (Vector2f) center,
    (float) radius,
  });

  STREAMABLE(Edge,
  {
    Edge() = default;
    Edge(BallCarrierModel::Node fromNode, BallCarrierModel::Node toNode);
    ,
    (BallCarrierModel::Node) fromNode,
    (BallCarrierModel::Node) toNode,
  });

  STREAMABLE(TrajectoryNode,
  {
    TrajectoryNode() = default;
    TrajectoryNode(Vector2f destination, float reachedAtXCoordinate);
    ,
    (Vector2f) destination,
    (float) reachedAtXCoordinate,
  });

  FUNCTION(Pose2f(float radius, float angle)) computeApproachPoint;

  FUNCTION(Pose2f()) dynamicApproachPoint;  /** Entry point for the approach area, aligned with the ball and the dynamicTarget, 
                                            at a distance from the ball equal to the MAX_APPROACH_DISTANCE */
  
  FUNCTION(Pose2f()) staticApproachPoint; /** Entry point for the approach area, aligned with the ball and the dynamicTarget, 
                                          at a distance from the ball equal to the cuX_APPROACH_DISTANCE] */

  /** Return the nearest obstacle to the ball */
  FUNCTION(Vector2f()) nearestObstacleToBall;


  ,

  (bool) graphicalDebug,                              
  (float) obstacleAvoidanceArcAngleStep,              
  (std::vector<Edge>) obstacleAvoidancePlan,          
  (std::vector<Node>) ballPath,                       
  (bool) isFallbackPath,                              
  (bool) isTargetOnGoal,                              
  (bool) isTargetInPenaltyArea,                       
  (bool) isTargetAPass,                               

  (float) dynamicGoalPostObstacleRadius,
  (float) dynamicUprightRobotObstacleRadius,
  (float) dynamicReadyRobotObstacleRadius,
  (float) dynamicFallenRobotObstacleRadius,
  (float) dynamicRadiusControlOffset,

  (Pose2f) dynamicTarget,                             
  (float) minimumApproachDistance,                    
  (float) maximumApproachDistance,                    
  (float) staticApproachRadius,                       
  (float) dynamicApproachRadius,                      

  (Rangef) xRangeDistanceFromGoalToUseKicks,
                                       
});

inline BallCarrierModel::Node::Node(Vector2f center, float radius) : center(center), radius(radius) {};
inline BallCarrierModel::Edge::Edge(BallCarrierModel::Node fromNode, BallCarrierModel::Node toNode) : fromNode(fromNode), toNode(toNode) {};
