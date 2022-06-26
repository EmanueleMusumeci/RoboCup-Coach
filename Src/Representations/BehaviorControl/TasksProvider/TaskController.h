/**
 * @file TaskController.h
 *
 * Declaration of the STREAMABLE struct TaskController to hold informations about the current state of the HRI routine
 *
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

#include "Representations/Modeling/BallModel.h"

#include <iostream>

/*#define TASK(taskName)\
\
  STREAMABLE_WITH_BASE(taskName##Task, HRI::Task,\
  {\
    taskName##Task(int taskID);\ 
    taskName##Task(int taskID, std::vector<HRI::Action> actionQueue);\ 
\
    void addAction(HRI::Action action)\
    {\
      actionQueue.push_back(action);\ 
    }\
    ,\
    (std::vector<HRI::Action>) actionQueue,\
  });\
  inline taskName##Task(int taskID) : Task(HRI::TaskType::taskName, taskID) {};\ 
  inline taskName##Task(int taskID, std::vector<HRI::Action> actionQueue) : Task(HRI::TaskType::taskName, taskID) COMMA actionQueue(actionQueue) {};

TASK(Idle);*/
/*#define IDLE_ACTION(actionName, executeBody, isCompletedBody)\
class actionName##Action : public Action\
{\
  public:\
    actionName##Action(int actionID) : Action(actionID, HRI::ActionType::actionName) {}\
\
    void execute(TaskController& controller)\
    {\
    executeBody\
    setExecuted();\
    }\
\
    bool isCompleted() override\
    isCompletedBody\
\
    bool isIdle() override {return true;}\
};

#define ACTION(actionName, executeBody, isCompletedBody)\
class actionName##Action : public Action\
{\
  public:\
    actionName##Action(int actionID) : Action(actionID, HRI::ActionType::actionName) {}\
\
    void execute(TaskController& controller)\
    {\
    executeBody\
    setExecuted();\
    }\
\
    bool isCompleted() override\
    isCompletedBody\
\
};*/

namespace HRI
{
    ENUM(ActionType,
    {,
      Idle,
      ReachPosition,
      ReachPositionAndAngle,
      ReachBall,
      CarryBall,
      Kick,
      PassBall,
      CarryAndKickToGoal,
      PerformInitialSpeech,
      PerformInstructionsSpeech,
    });

    ENUM(TaskType,
    {,
      None, //Only used when the task list is empty
      PlanControlledTask, //Special type of task that wraps the action ordered by the Plan
      GoToPosition,
      //TurnToPosition,
      KickBallToPosition,
      CarryBallToPosition,
      ScoreGoalTask,
      //PointAtPosition,
      //PointAtBall,
      InitialSpeech,
      InstructionsSpeech,
    });

}

/* An action provides an execute method that modifies the TaskController state to execute the right behavior
  to complete the action */
STREAMABLE(Action,
{
  Action() = default;
  Action(HRI::ActionType actionType);
  Action(HRI::ActionType actionType, Vector2f target);
  Action(HRI::ActionType actionType, Vector2f target, float angle);
  Action(HRI::ActionType actionType, int teammateTarget);
  
  /*Action& operator=(const Action& other)
  {
    return *this;
  }*/
  
  virtual ~Action() = default;
  
  void draw(bool graphicalDebug);
  ,
  (bool) completed,
  (HRI::ActionType) actionType,
  (Vector2f) target,
  (float) angle,
  (int) teammateTarget,
});

//CTOR for actions not based on a target or a teammate
inline Action::Action(HRI::ActionType actionType) : actionType(actionType), target(Vector2f(0,0)), angle(0.f), teammateTarget(-1)
{
  //std::cout<<"actionType: "<<TypeRegistry::getEnumName(actionType)<<std::endl;
  //Verify that the action is not target-based
  ASSERT(
    //Target-based actions
    actionType!=HRI::ActionType::CarryBall 
    &&
    actionType!=HRI::ActionType::Kick 
    &&
    actionType!=HRI::ActionType::ReachPosition 
    &&
    actionType!=HRI::ActionType::ReachPositionAndAngle
    //Teammate-based actions
    &&
    actionType!=HRI::ActionType::PassBall
  );
};
//CTOR for target-based actions
inline Action::Action(HRI::ActionType actionType, Vector2f target) : actionType(actionType), target(target), angle(0.f), teammateTarget(-1)
{
  //std::cout<<"actionType: "<<TypeRegistry::getEnumName(actionType)<<std::endl;
  //Verify that the action is target-based and not teammate-based
  ASSERT(
    actionType!=HRI::ActionType::Idle 
    &&
    actionType!=HRI::ActionType::ReachBall
    &&
    actionType!=HRI::ActionType::PerformInitialSpeech
    &&
    actionType!=HRI::ActionType::PerformInstructionsSpeech
    //Teammate-based actions
    &&
    actionType!=HRI::ActionType::PassBall
  );
};
//CTOR for target-based actions with an angle
inline Action::Action(HRI::ActionType actionType, Vector2f target, float angle) : actionType(actionType), target(target), angle(angle), teammateTarget(-1)
{
  //std::cout<<"actionType: "<<TypeRegistry::getEnumName(actionType)<<std::endl;
  //Verify that the action is target-based and not teammate-based
  ASSERT(
    actionType==HRI::ActionType::ReachPositionAndAngle
  );
};
//CTOR for teammate-based actions
inline Action::Action(HRI::ActionType actionType, int teammateNumber) : actionType(actionType), target(Vector2f(0,0)), angle(0.f), teammateTarget(teammateNumber)
{
  //std::cout<<"actionType: "<<TypeRegistry::getEnumName(actionType)<<std::endl;
  //Verify that the action is teammate-based
  ASSERT(
    actionType==HRI::ActionType::PassBall
  );
};

/*

 _____________________
|                     |
| Action declarations |
|_____________________|


*/

STREAMABLE_WITH_BASE(IdleAction, Action,
{
  IdleAction();
  ,
});
inline IdleAction::IdleAction() : Action(HRI::ActionType::Idle) {};

/////////////////////////////
//Ball target-based actions//
/////////////////////////////
/*STREAMABLE_WITH_BASE(CarryBallAction, Action,
{
  CarryBallAction(Vector2f ballDestination);
  ,
});
inline CarryBallAction::CarryBallAction(Vector2f ballDestination) : Action(HRI::ActionType::CarryBall, ballDestination) {};

STREAMABLE_WITH_BASE(KickBallAction, Action,
{
  KickBallAction(Vector2f ballDestination);
  ,
});
inline KickBallAction::KickBallAction(Vector2f ballDestination) : Action(HRI::ActionType::Kick, ballDestination) {};

//////////////////////////////
//Robot target-based actions//
//////////////////////////////
STREAMABLE_WITH_BASE(ReachBallAction, Action,
{
  ReachBallAction(Vector2f ballDestination);
  ,
});
inline ReachBallAction::ReachBallAction() : Action(HRI::ActionType::ReachBall, ballDestination) {};

STREAMABLE_WITH_BASE(KickBallAction, Action,
{
  KickBallAction(Vector2f ballDestination);
  ,
});
inline KickBallAction::KickBallAction(Vector2f ballDestination) : Action(HRI::ActionType::Kick, ballDestination) {};
*/



STREAMABLE(Task,
{     
  Task() = default; 
  Task(HRI::TaskType taskType, int taskID, std::vector<Action>& actionQueue, Vector2f finalPosition);
  Task(HRI::TaskType taskType, int taskID, std::vector<Action>& actionQueue, int teammateNumber);
  Task(HRI::TaskType taskType, int taskID, std::vector<Action>& actionQueue);
  Task(HRI::TaskType taskType, int taskID);
  /*Task& operator=(const Task& other)
  {
    return *this;
  }*/

  ,
  (std::vector<Action>) actionQueue,

  (HRI::TaskType) taskType,
  (int) taskID,
  (int) taskSize,
  (Vector2f) finalPosition,
  (int) teammateNumber,
    
});
inline Task::Task(HRI::TaskType taskType, int taskID, std::vector<Action>& actionQueue, Vector2f finalPosition) : taskType(taskType), taskID(taskID), actionQueue(actionQueue), finalPosition(finalPosition), taskSize(actionQueue.size()) {};
inline Task::Task(HRI::TaskType taskType, int taskID, std::vector<Action>& actionQueue, int teammateNumber) : taskType(taskType), taskID(taskID), actionQueue(actionQueue), teammateNumber(teammateNumber), taskSize(actionQueue.size()) {};
inline Task::Task(HRI::TaskType taskType, int taskID, std::vector<Action>& actionQueue) : taskType(taskType), taskID(taskID), actionQueue(actionQueue), taskSize(actionQueue.size()) {};
inline Task::Task(HRI::TaskType taskType, int taskID) : taskType(taskType), taskID(taskID), taskSize(0) {};

/**
 * @struct TaskController
 * 
 * Struct containing state info about the Task execution
 * 
 */
STREAMABLE(TaskController,
{
  /** Draws the current task on the field in SimRobot */
  void draw() const;

  /* Update current desired destination of the robot based on the current task */
  FUNCTION(void(Pose2f destinationPose)) updateCurrentDestination;

  /* Update current desired destination of the ball based on the current task */
  FUNCTION(void(Vector2f ballDestination)) updateCurrentBallDestination;

  FUNCTION(Task()) getCurrentTask;

  FUNCTION(void()) setPlanMode;
  FUNCTION(void()) setTaskMode;

  FUNCTION(void()) nextTask;
  FUNCTION(void()) resetTaskQueue;
  FUNCTION(HRI::ActionType()) getCurrentActionType;
  FUNCTION(bool(bool condition)) checkActionCompleted;
  FUNCTION(bool(bool playSound)) checkTaskCompleted;
  FUNCTION(void(Task)) addTask;
  FUNCTION(void(std::vector<Task>)) updateTasks;
  FUNCTION(std::string(Action action)) actionToString;
  FUNCTION(std::string()) tasksToString;
  FUNCTION(Action()) getCurrentAction;
  FUNCTION(Action()) nextAction;
  FUNCTION(bool()) isTaskComplete;
  FUNCTION(bool()) isIdle;
  FUNCTION(void(int taskID)) deleteSingleTask;

  FUNCTION(void(bool initialSpeech, int taskID)) scheduleInstructionsSpeech;
  FUNCTION(void()) scheduleIdleTask;

  FUNCTION(void(bool playSound)) signalTaskCompleted;

  TaskController();
  ,

  (bool) GRAPHICAL_DEBUG,

  (std::vector<Task>) completedTasks,
  (std::vector<Task>) taskQueue,
  (int)(0) currentAction,

  (Pose2f) currentRobotDestination,
  (Vector2f) currentBallDestination,

  (int)(-1) lastReceivedTaskID,
  (int)(-1) lastCompletedTaskID,

  (float) ballCarrierDistanceThreshold,
  (float) reachPositionDistanceThreshold,
  (float) reachPositionAngleThreshold,
  (float) kickDistanceThreshold,

  (Vector2f) userPosition,
  (float) userHeight,

  (bool) interactWithUser,

  (bool)(false) initialSpeechPerformed,

  (bool)(false) planControlledMode, //When set to true, the task controller will not use the task queue but the dfaTask field, and will not update work based on the taskID

});
inline TaskController::TaskController() : taskQueue(), completedTasks() {}
//TODO Expose function to update task queue