/**
 * @file TaskControllerProvider.h
 *
 * This module keeps track of the state of execution of the Obstacle Avoidance HRI routine 
 *  
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/TasksProvider/TaskController.h"
#include "Representations/Communication/ExternalServerCommunicationController/ExternalServerCommunicationControl.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"

#include "Representations/BehaviorControl/TasksProvider/TaskController.h"

#include "Platform/Linux/SoundPlayer.h"

#include <iostream>
#include <ostream>

MODULE(TaskControllerProvider,
{,
    REQUIRES(LibCheck),
    REQUIRES(FieldDimensions),
    REQUIRES(BallSpecification),
    REQUIRES(GameInfo),
    REQUIRES(RobotInfo),
    REQUIRES(RobotPose),
    REQUIRES(BallModel),

    USES(BehaviorStatus),

    PROVIDES(TaskController),
    
    LOADS_PARAMETERS(
    {,
      //goalTarget constants
      (bool) GRAPHICAL_DEBUG,                                       /** Shows a graphical debug render in SimRobot */
      (bool) PRINT_DEBUG,
      
      (float) BALL_CARRIER_DISTANCE_THRESHOLD,                      /** Minimum distance of the ball from its destination to declare the CarryBall action completed */ 
      (float) KICK_DISTANCE_THRESHOLD,                              /** Minimum distance of the ball from its destination to declare the Kick action completed */ 
      (float) REACH_POSITION_DISTANCE_THRESHOLD,                    /** Minimum distance of the robot from its destination to declare the ReachPosition action completed */ 
      (float) REACH_POSITION_ANGLE_THRESHOLD,                       /** Minimum angular distance of the robot from its target angle to declare the ReachPosition action completed */ 

      (bool) INTERACT_WITH_USER,                                    /** Perform interactions with human **/

      (Vector2f) userPosition,                                      /** Position of the human user */
      (float) userHeight,                                           /** Height of the human user */

      (bool) PERFORM_INITIAL_SPEECH,                                /** Tells whether the robot should perform the initial speech */

    }),
});

/**
 * @class TaskControllerProvider
 * A module that provides the model of the opponent goal
 */
class TaskControllerProvider: public TaskControllerProviderBase
{
public:
  /** Constructor */
  TaskControllerProvider();

  /* One constructor for each task type */
  /* These tasks take a position as argument, that has a different meaning for each one */
  static Task GoToPositionTask(Vector2f position, int taskID);
  static Task CarryBallToPositionTask(Vector2f position, int taskID);
  static Task KickBallToPositionTask(Vector2f position, int taskID);
  static Task InstructionsSpeechTask(int taskID, Vector2f position);

  /* These tasks take no additional argument (except the taskID) */
  static Task ScoreGoalTask(int taskID);
  static Task InitialSpeechTask(int taskID);

  /* This special task wraps the action ordered by the Plan */
  static Task PlanControlledTask(Action PlanAction, int taskID);

private:
  void update(TaskController& controller) override;
  bool checkCompleted(HRI::ActionType actionType);
};

inline Task TaskControllerProvider::GoToPositionTask(Vector2f position, int taskID)
{
  std::vector<Action> actionQueue;
  actionQueue.push_back(Action(HRI::ActionType::ReachPosition, position));
  
  return Task(HRI::TaskType::GoToPosition, taskID, actionQueue, position);
}

inline Task TaskControllerProvider::CarryBallToPositionTask(Vector2f position, int taskID)
{
  std::vector<Action> actionQueue;
  actionQueue.push_back(Action(HRI::ActionType::ReachBall));
  actionQueue.push_back(Action(HRI::ActionType::CarryBall, position));
  
  return Task(HRI::TaskType::CarryBallToPosition, taskID, actionQueue, position);
}

inline Task TaskControllerProvider::KickBallToPositionTask(Vector2f position, int taskID)
{
  std::vector<Action> actionQueue;
  actionQueue.push_back(Action(HRI::ActionType::ReachBall));
  actionQueue.push_back(Action(HRI::ActionType::Kick, position));
  return Task(HRI::TaskType::KickBallToPosition, taskID, actionQueue, position);
}

inline Task TaskControllerProvider::ScoreGoalTask(int taskID)
{
  std::vector<Action> actionQueue;
  actionQueue.push_back(Action(HRI::ActionType::ReachBall));
  actionQueue.push_back(Action(HRI::ActionType::CarryAndKickToGoal));
  return Task(HRI::TaskType::ScoreGoalTask, taskID, actionQueue, Vector2f(0,0));
}

inline Task TaskControllerProvider::InstructionsSpeechTask(int taskID, Vector2f position)
{
  std::vector<Action> actionQueue;
  actionQueue.push_back(Action(HRI::ActionType::PerformInstructionsSpeech));
  return Task(HRI::TaskType::InstructionsSpeech, taskID, actionQueue, position);
}

inline Task TaskControllerProvider::InitialSpeechTask(int taskID)
{
  std::vector<Action> actionQueue;
  actionQueue.push_back(Action(HRI::ActionType::PerformInitialSpeech));
  return Task(HRI::TaskType::InitialSpeech, taskID, actionQueue, Vector2f(0,0));
}

inline Task TaskControllerProvider::PlanControlledTask(Action PlanAction, int taskID)
{
  std::vector<Action> actionQueue;
  actionQueue.push_back(PlanAction);
  return Task(HRI::TaskType::PlanControlledTask, taskID, actionQueue);
}