/**
 * @file Modules/BehaviorControl/BallCarrierModel/BallCarrierModelProvider.cpp
 *
 * This module contains all info necessary for the Ball Carrier model
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "BallCarrierModelProvider.h"

BallCarrierModelProvider::BallCarrierModelProvider()
{}

void BallCarrierModelProvider::update(BallCarrierModel& ballCarrierModel)
{
    ballCarrierModel.computeApproachPoint = [&] (float radius, float angle)
    {   
        Vector2f ballPositionGlobal = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation; 
        float dynamicOffsetX = radius * cos(pi + angle);
        float dynamicOffsetY = radius * sin(pi + angle);
        return Pose2f(-angle, ballPositionGlobal.x() + dynamicOffsetX, ballPositionGlobal.y() - dynamicOffsetY);
    };
    
    ballCarrierModel.dynamicApproachPoint = [&] () -> Pose2f
    {
        Vector2f ballPositionGlobal = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation; 
        float ballToTargetAngle = theLibCheck.angleBetweenPoints(ballPositionGlobal, ballCarrierModel.dynamicTarget.translation);
        return ballCarrierModel.computeApproachPoint(ballCarrierModel.dynamicApproachRadius, ballToTargetAngle);
    };

    ballCarrierModel.staticApproachPoint = [&] () -> Pose2f
    {
        Vector2f ballPositionGlobal = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        float ballToTargetAngle = theLibCheck.angleBetweenPoints(ballPositionGlobal, ballCarrierModel.dynamicTarget.translation);
        return ballCarrierModel.computeApproachPoint(ballCarrierModel.staticApproachRadius, ballToTargetAngle);
    };
    
    ballCarrierModel.nearestObstacleToBall = [&] () -> Vector2f
    {
        Vector2f nearestObstacle;
        float nearestObstacleToBallDistance = -1;
        for(auto obs : theObstacleModel.obstacles)
        {
            float currentObsDistance = theLibCheck.distance(theBallModel.estimate.position, obs.center);
            if(nearestObstacleToBallDistance == -1 || theLibCheck.distance(theBallModel.estimate.position, obs.center) < nearestObstacleToBallDistance)
            {
                nearestObstacle = Vector2f(obs.center.x(), obs.center.y());
                nearestObstacleToBallDistance = currentObsDistance;     
            }
        }
        return nearestObstacle;
    };

    ballCarrierModel.graphicalDebug = GRAPHICAL_DEBUG;
    
    Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

    
    ballCarrierModel.maximumApproachDistance = MAXIMUM_APPROACH_DISTANCE;
    ballCarrierModel.minimumApproachDistance = MINIMUM_APPROACH_DISTANCE;
    
    ballCarrierModel.staticApproachRadius = STATIC_APPROACH_RADIUS;

    ballCarrierModel.xRangeDistanceFromGoalToUseKicks = X_RANGE_DISTANCE_FROM_GOAL_TO_USE_KICKS;
    
    ballCarrierModel.obstacleAvoidanceArcAngleStep = pi2/OBSTACLE_AVOIDANCE_ARC_ANGLE_STEP_FACTOR;
    ballCarrierModel.obstacleAvoidancePlan.clear();
    ballCarrierModel.ballPath.clear();

    bool useHeuristic = true;
    if(!ALWAYS_USE_GOAL_TARGET_HEURISTIC && globalBall.x() > theFieldDimensions.xPosOpponentGroundline - ballCarrierModel.xRangeDistanceFromGoalToUseKicks.max)
    {
        useHeuristic = false;
    }
    Vector2f goalTarget = theLibCheck.goalTarget();
    
    Pose2f target;
    target = theTaskController.currentBallDestination;
    
    ballCarrierModel.dynamicTarget = target;
    
    ballCarrierModel.isFallbackPath = true;
    ballCarrierModel.isTargetOnGoal = true;
    ballCarrierModel.isTargetAPass = false;

    Vector2f nearestObstacle = ballCarrierModel.nearestObstacleToBall();
    float nearestObstacleToBallDistance = theLibCheck.distance(theBallModel.estimate.position, nearestObstacle);

    ballCarrierModel.dynamicGoalPostObstacleRadius = theLibPathPlanner.getDefaultObstacleRadius(Obstacle::Type::goalpost);
    
    ballCarrierModel.dynamicUprightRobotObstacleRadius = (nearestObstacleToBallDistance == -1 ? BASE_UPRIGHT_ROBOT_OBSTACLE_RADIUS : std::max(MINIMUM_UPRIGHT_ROBOT_OBSTACLE_RADIUS, std::min(BASE_UPRIGHT_ROBOT_OBSTACLE_RADIUS, nearestObstacleToBallDistance)));
    
    if(globalBall.x() > theFieldDimensions.xPosOpponentGroundline - ballCarrierModel.xRangeDistanceFromGoalToUseKicks.max)
    {
        ballCarrierModel.dynamicUprightRobotObstacleRadius = BASE_UPRIGHT_ROBOT_OBSTACLE_RADIUS - theLibCheck.mapToInterval(globalBall.x(), theFieldDimensions.xPosOpponentPenaltyArea - ballCarrierModel.xRangeDistanceFromGoalToUseKicks.max, theFieldDimensions.xPosOpponentPenaltyArea, 0, BASE_UPRIGHT_ROBOT_OBSTACLE_RADIUS - MINIMUM_UPRIGHT_ROBOT_OBSTACLE_RADIUS_IN_KICKING_AREA);
    }

    if(globalBall.x() < OBSTACLE_RADIUS_MULTIPLIER_CHANGE_X_COORDINATE)
    {
        ballCarrierModel.dynamicUprightRobotObstacleRadius *= OBSTACLE_RADIUS_MULTIPLIER.min;
    }
    else
    {
        ballCarrierModel.dynamicUprightRobotObstacleRadius *= OBSTACLE_RADIUS_MULTIPLIER.max;
    }

    ballCarrierModel.dynamicReadyRobotObstacleRadius = theLibPathPlanner.getDefaultObstacleRadius(Obstacle::Type::someRobot);
    ballCarrierModel.dynamicFallenRobotObstacleRadius = theLibPathPlanner.getDefaultObstacleRadius(Obstacle::Type::fallenOpponent);
    ballCarrierModel.dynamicRadiusControlOffset = 100;

    if(theGameInfo.state == STATE_PLAYING)
    {
        Pose2f speed = Pose2f(0.8f,0.8f,0.8f);
        bool avoidPenaltyArea = false;
        std::vector<PathPlannerUtils::Node> plan = theLibPathPlanner.populatePlanWithCustomObstacleRadius(theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()), target, speed, avoidPenaltyArea,
                                                                                                            ballCarrierModel.dynamicGoalPostObstacleRadius,
                                                                                                            ballCarrierModel.dynamicUprightRobotObstacleRadius,
                                                                                                            ballCarrierModel.dynamicReadyRobotObstacleRadius,
                                                                                                            ballCarrierModel.dynamicFallenRobotObstacleRadius,
                                                                                                            ballCarrierModel.dynamicRadiusControlOffset);
        
        std::vector<PathPlannerUtils::Edge*> obstacleAvoidancePlan = theLibPathPlanner.createAvoidancePlan(plan);
        std::vector<Vector2f> ballPath = theLibPathPlanner.computePath(plan, ballCarrierModel.obstacleAvoidanceArcAngleStep);
        
        ballCarrierModel.dynamicTarget = target; 
        
        if(obstacleAvoidancePlan.size() > 0)
        {
            PathPlannerUtils::Edge* edge = obstacleAvoidancePlan[0];
            BallCarrierModel::Node fromNode(edge->fromNode->center, edge->fromNode->radius);
            BallCarrierModel::Node toNode;
            for(int i = 0; i<obstacleAvoidancePlan.size(); i++)
            {
                edge = obstacleAvoidancePlan[i];
                toNode = BallCarrierModel::Node(edge->toNode->center, edge->toNode->radius);
                ballCarrierModel.obstacleAvoidancePlan.push_back(BallCarrierModel::Edge(fromNode, toNode));
                fromNode = toNode;
            }
        }

        if(ballPath.size() > 0)
        {
            BallCarrierModel::Edge edge;
            BallCarrierModel::Node fromNode = BallCarrierModel::Node(ballPath[0], 0.f);
            ballCarrierModel.ballPath.push_back(fromNode);
            for(int i=1; i<ballPath.size(); i++)
            {
                Vector2f position = ballPath[i];
                BallCarrierModel::Node toNode(position, 0.f);
                edge = BallCarrierModel::Edge(fromNode, toNode);
                ballCarrierModel.ballPath.push_back(toNode);
            }
            ballCarrierModel.isFallbackPath = false;
        }
        else
        {
            ballCarrierModel.ballPath.push_back(BallCarrierModel::Node(theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation, 0.f));
            ballCarrierModel.ballPath.push_back(BallCarrierModel::Node(target.translation, 0.f));
            ballCarrierModel.isFallbackPath = true;
        }
        
        if(ballPath.size()==2)
        {
            float angle = theLibCheck.angleToTarget(ballCarrierModel.ballPath[1].center.x(), ballCarrierModel.ballPath[1].center.y());
            ballCarrierModel.dynamicTarget = Pose2f(angle, ballCarrierModel.ballPath[1].center.x(), ballCarrierModel.ballPath[1].center.y());

            ballCarrierModel.isTargetOnGoal = true;
        }
        else
        {
            float angle = theLibCheck.angleToTarget(ballCarrierModel.ballPath[1].center.x(), ballCarrierModel.ballPath[1].center.y());
            ballCarrierModel.dynamicTarget = Pose2f(angle, ballCarrierModel.ballPath[1].center.x(), ballCarrierModel.ballPath[1].center.y());

            ballCarrierModel.isTargetOnGoal = false;
        }
        
        Vector2f ballPositionGlobal = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        
        if(theLibCheck.distance(theRobotPose, ballPositionGlobal) > ballCarrierModel.maximumApproachDistance)
        {
            ballCarrierModel.dynamicApproachRadius = ballCarrierModel.maximumApproachDistance;
        }
        else if(theLibCheck.distance(theRobotPose, ballPositionGlobal) < ballCarrierModel.minimumApproachDistance)
        {
            ballCarrierModel.dynamicApproachRadius = ballCarrierModel.minimumApproachDistance;
        }
        else if(theLibCheck.distance(theRobotPose, ballPositionGlobal) > ballCarrierModel.minimumApproachDistance 
        && theLibCheck.distance(theRobotPose, ballPositionGlobal) < ballCarrierModel.maximumApproachDistance) 
        {
            ballCarrierModel.dynamicApproachRadius = theLibCheck.distance(theRobotPose, ballPositionGlobal);
        }
    }
}

MAKE_MODULE(BallCarrierModelProvider, behaviorControl)
