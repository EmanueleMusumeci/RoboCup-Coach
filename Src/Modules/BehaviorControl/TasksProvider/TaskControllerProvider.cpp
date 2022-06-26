/**
 * @file TaskControllerProvider.cpp
 *
 * This module keeps track of the state of execution of the Obstacle Avoidance HRI routine 
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "TaskControllerProvider.h"

//Prints a debug message prefixed by the robot number
#define DEBUG_NUMB(print_debug, message) \
  if(print_debug) std::cout<<"[Robot #"<<theRobotInfo.number<<"] " << message << std::endl; 

//Prints a debug message prefixed by the robot number, showing the code being debugged and its computed value
#define __STRINGIFY_I(arg) #arg
#define __STRINGIFY(arg) __STRINGIFY_I(arg)
#define DEBUG_CODE(code) \
  std::cout<<"[Robot #"<<std::to_string(theRobotInfo.number)<<"] "<<__STRINGIFY(code)<<": "<<std::to_string(code)<<std::endl;

TaskControllerProvider::TaskControllerProvider()
{
    
}

void TaskControllerProvider::update(TaskController& controller)
{
    
    DEBUG_NUMB(PRINT_DEBUG, "TaskController update function\n\n\n");

    //CFG parameters
    controller.ballCarrierDistanceThreshold = BALL_CARRIER_DISTANCE_THRESHOLD;
    controller.kickDistanceThreshold = KICK_DISTANCE_THRESHOLD;
    controller.reachPositionDistanceThreshold = REACH_POSITION_DISTANCE_THRESHOLD;
    controller.reachPositionAngleThreshold = REACH_POSITION_ANGLE_THRESHOLD;

    controller.interactWithUser = INTERACT_WITH_USER;
    
    controller.userPosition = userPosition;
    controller.userHeight = userHeight;


    controller.setPlanMode = [&] () -> void
    {
        controller.planControlledMode = true;
    };

    controller.setTaskMode = [&] () -> void
    {
        controller.planControlledMode = false;
    };

    //DEBUG_NUMB(PRINT_DEBUG, std::to_string(controller.taskQueue.empty()));
    
    /* Update current desired destination for the robot */
    controller.updateCurrentDestination = [&] (Pose2f destinationPose) -> void
    {
        controller.currentRobotDestination = destinationPose;
    };

    /* Update current desired destination for the ball */
    controller.updateCurrentBallDestination = [&] (Vector2f ballDestination) -> void
    {
        controller.currentBallDestination = ballDestination;
    };

  

    /* Schedules an instruction speech task by adding it at the head of the queue: 
        - if at the beginning of the routine a InitialSpeechTask or an InstructionsSpeech (depending on the value of the initialSpeech flag) is scheduled (in which the robot will face the human and play its speech sound) 
        OR
        - if the first task in the queue is not an InitialSpeech or an InstructionsSpeech (to avoid repeating it)
    */
    controller.scheduleInstructionsSpeech = [&] (bool initialSpeech, int taskID) -> void
    {
        if(controller.taskQueue.empty() || (controller.taskQueue.at(0).taskType != HRI::TaskType::InitialSpeech && controller.taskQueue.at(0).taskType != HRI::TaskType::InstructionsSpeech))
        {
            if(initialSpeech)
            {
                controller.taskQueue.insert(controller.taskQueue.begin(), InitialSpeechTask(taskID));
                controller.lastReceivedTaskID = taskID;
            }
            else
            {
                controller.taskQueue.insert(controller.taskQueue.begin(), InstructionsSpeechTask(taskID, theRobotPose.translation));
            }
        }
    };

    /* Get the current task that the robot has to execute, based on the task queue:
        - if the queue is empty:
            if the PERFORM_INITIAL_SPEECH flag is true:
                -- schedules an InitialSpeech if it has never been performed before
                -- else returns an IdleTask (WITHOUT ADDING IT TO THE QUEUE), during which the robot 
                    will face the human and wait (saying something from time to time) 
        then return the first task in the queue                   
     */
    controller.getCurrentTask = [&] () -> Task
    {
        //DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.getCurrentTask");
        
        //If no task is requested
        if(controller.taskQueue.empty()) 
        {
            DEBUG_NUMB(PRINT_DEBUG, "Task queue empty");
            if(!controller.initialSpeechPerformed && PERFORM_INITIAL_SPEECH)
            {
                DEBUG_NUMB(PRINT_DEBUG, "Perform initial speech");
                controller.scheduleInstructionsSpeech(true, 0);
                controller.initialSpeechPerformed = true;
            }
            else
            {
                DEBUG_NUMB(PRINT_DEBUG, "Idle task");
                ::std::vector<Action> idleQueue = ::std::vector<Action>({IdleAction()});
                Task idleTask = Task(HRI::TaskType::None, -1, idleQueue, theRobotPose.translation);
                return idleTask; //SINGLETON;
            }
        }
        return controller.taskQueue.at(0);
    };

    /* Pushes the first task in the queue (current task) to the completedTasks queue, 
        if playSound is true plays a sound announcing that the task has been completed
        and then calls the nextTask method */
    controller.signalTaskCompleted = [&] (bool playSound) -> void
    {
        DEBUG_NUMB(PRINT_DEBUG, "Current tasks: "<<controller.tasksToString());
        //DEBUG_NUMB(PRINT_DEBUG, "taskCompleted");
        controller.completedTasks.push_back(controller.taskQueue.at(0));
        if(playSound && !controller.planControlledMode) SoundPlayer::play("TaskCompleted.wav");
        controller.nextTask();
    };

    /* IF the task queue is NOT EMPTY:
        1) sets the lastCompletedTaskID to the one of the first task in the queue (the currently executed one) 
        2) erases that task from the taskQueue
        3) resets the currentAction index (tracking the index of the action in the actionQueue of the current task)
    */
    controller.nextTask = [&] () -> void
    {
        //DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.nextTask");
        if(!controller.taskQueue.empty())
        {
            //DEBUG_NUMB(PRINT_DEBUG, "Task queue not empty");
            //DEBUG_NUMB(PRINT_DEBUG, std::to_string(controller.taskQueue.at(0).taskID));
            controller.lastCompletedTaskID = controller.taskQueue.at(0).taskID;
            controller.taskQueue.erase(controller.taskQueue.begin());
            controller.currentAction = 0;
            //DEBUG_NUMB(PRINT_DEBUG, std::to_string(controller.currentAction));
        }    
        //DEBUG_NUMB(PRINT_DEBUG, "Gone to next task");    
    };

    /* Returns the current action:
        - If the task queue is empty or the actionQueue of the current task is empty, returns the IdleAction
        - Else, return the action in the actionQueue of the current task at the controller.currentAction position
    */
    controller.getCurrentAction = [&] () -> Action
    {
        //DEBUG_NUMB(PRINT_DEBUG, "getCurrentAction");
        if(controller.taskQueue.empty() || controller.taskQueue.at(0).actionQueue.empty()) return IdleAction();
        return controller.taskQueue.at(0).actionQueue.at(controller.currentAction);
    };

    /* Transitions to the next action:
        - If the current task is not completed, increment the currentAction counter
        Then
            1) Check if task is completed 
                -> If it is completed, the checkTaskCompleted method will transition to the next task!
            2) Return the new current action
    */
    controller.nextAction = [&] () -> Action
    {
        //DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.nextAction");
        if(!controller.isTaskComplete())
        {
            //DEBUG_NUMB(PRINT_DEBUG, "task not complete: current action before "<<controller.currentAction);
            controller.currentAction+=1;
            //DEBUG_NUMB(PRINT_DEBUG, "task not complete: current action after "<<controller.currentAction);
        }
        controller.checkTaskCompleted(true);
        return controller.getCurrentAction();
    };

    /* Performs all necessary operations to transition to next task */
    controller.deleteSingleTask = [&] (int taskID) -> void
    {
        //DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.deleteSingleTask");
        
        //DEBUG_NUMB(PRINT_DEBUG, "Current tasks: "<<controller.tasksToString());

        
        //If the task to delete is the current one, simply go to the next task
        if(controller.getCurrentTask().taskID == taskID) 
        {
            controller.nextTask();
        }
        else
        {
            //ELSE
            //Find the task with the taskID to be deleted
            Task* selectedTask = nullptr;
            int i=0;
            for(auto task : controller.taskQueue)
            {
                //DEBUG_NUMB(PRINT_DEBUG, std::to_string(task.taskID));
                if(task.taskID == taskID) 
                {
                    DEBUG_NUMB(PRINT_DEBUG, "found task "+std::to_string(task.taskID));
                    selectedTask = &task;
                    break;
                }
                i++;
            }
            //If it's not been found, simply return
            if(!selectedTask) return;
            
            //else erase it from the queue (the lastCompletedTaskID mechanism will do the rest)
            controller.taskQueue.erase(controller.taskQueue.begin() + i);   
        }
    };

    /* Resets the current task queue and sets the controller accordingly */
    controller.resetTaskQueue = [&] () -> void
    {
        DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.resetTaskQueue");
        //Find the maximum taskID (lastReceivedTaskID and lastCompletedTaskID will be set to this value +1)
        int maxTaskID = -1;
        for(auto task : controller.taskQueue)
        {
            maxTaskID = std::max(task.taskID, maxTaskID);
        }
        controller.taskQueue.clear();
        controller.lastReceivedTaskID = maxTaskID + 1;
        controller.lastCompletedTaskID = maxTaskID + 1;
        controller.currentAction = 0;
    };
    
    /* Return the actionType field of the current action (or the Idle ActionType in case of IdleTask) */
    controller.getCurrentActionType = [&] () -> HRI::ActionType
    {
        //DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.getCurrentActionType");
        //DEBUG_CODE(controller.taskQueue.size());
        //DEBUG_CODE(controller.currentAction);
        if(controller.taskQueue.empty() || controller.taskQueue.at(0).actionQueue.empty()) return HRI::ActionType::Idle;
        return controller.taskQueue.at(0).actionQueue.at(controller.currentAction).actionType;
    };

    /* Check if action was completed using the boolean value passed as input (that is supposed to be the one associated with the condition) 
            -> in case of COMPLETED action CALL the nextAction method
    */
    controller.checkActionCompleted = [&] (bool condition) -> bool
    {
        //DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.checkActionCompleted");
        //std::cout<<"taskID: "<<controller.getCurrentTask().taskID<<std::endl;
        if(condition)
        {
            controller.nextAction();
            return true;
        }
        return false;
    };

    /* Tells if the the currentAction index is equal to the size of the actionQueue of the current task (NOTICE: tjhe IdleTask is NEVER completed)*/
    controller.isTaskComplete = [&] () -> bool
    {
        //DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.isTaskComplete");
        if(controller.taskQueue.empty()) return false; //Idle task is never complete
        //DEBUG_NUMB(PRINT_DEBUG, "Current tasks: "<<controller.tasksToString());
        //DEBUG_NUMB(PRINT_DEBUG, "taskQueue not empty");
        //DEBUG_NUMB(PRINT_DEBUG, "controller.taskQueue.at(0).taskSize: "<<controller.taskQueue.at(0).taskSize);
        //DEBUG_NUMB(PRINT_DEBUG, "controller.currentAction: "<<controller.currentAction);
        return controller.currentAction >= controller.taskQueue.at(0).taskSize;
    };

    controller.isIdle = [&] () -> bool
    {
        return controller.taskQueue.empty(); //Idle task is never complete
    };

    /* Checks if the current Task has been completed and in that case transitions to the next task. Returns a boolean value */
    controller.checkTaskCompleted = [&] (bool playSound) -> bool
    {
        //DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.checkTaskCompleted");
        if(controller.isTaskComplete())
        {
            DEBUG_NUMB(PRINT_DEBUG, "Current tasks: "<<controller.tasksToString());
            //DEBUG_NUMB(PRINT_DEBUG, "taskCompleted");
            controller.completedTasks.push_back(controller.taskQueue.at(0));

            if(playSound && !controller.planControlledMode) SoundPlayer::play("TaskCompleted.wav");

            controller.nextTask();
            return true;
        }
        return false;
    };

    /* Add a new task to the current task queue ONLY if its taskID lower than the lastReceivedTaskID (else this task is obsolete and has to be ignored) */  
    controller.addTask = [&] (Task task) -> void
    {
        //DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.addTask");
        //DEBUG_NUMB(PRINT_DEBUG, "Adding task: task.taskID: "<<std::to_string(task.taskID)<<", controller.lastReceivedTaskID: "<<std::to_string(controller.lastReceivedTaskID));
        
        //IF in Plan mode, a task with the same taskID as the current one will OVERWRITE it
        //ELSE, if in Task mode, only newer tasks (tasks with a taskID higher than the lastReceivedTaskID) will be added to the task queue
        if(controller.planControlledMode)
        {
            //Only add newer tasks, that have a higher taskID
            if(task.taskID < controller.lastReceivedTaskID) return;
            else if(task.taskID == controller.lastReceivedTaskID)
            {
                if(!controller.taskQueue.empty())
                {
                    controller.taskQueue.pop_back();
                }
            }
        }
        else
        {
            //Only add newer tasks, that have a higher taskID
            if(task.taskID <= controller.lastReceivedTaskID) return;
            DEBUG_NUMB(PRINT_DEBUG, "Task controlled mode: adding task (ID: "<<task.taskID<<")");
        }

        controller.taskQueue.push_back(task);
        controller.lastReceivedTaskID = task.taskID;

    };
    
    /* Adds all tasks from a std::vector of tasks */
    controller.updateTasks = [&] (std::vector<Task> newTasks) -> void
    {
        //DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.updateTasks");
        //DEBUG_NUMB(PRINT_DEBUG, "Updating tasks");
        for(auto task : newTasks) controller.addTask(task);
        //DEBUG_NUMB(PRINT_DEBUG, "Tasks updated");
    };

    /* [DEBUG] Prints an action-specific debug string */
    controller.actionToString = [&] (Action action) -> std::string
    {
        //DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.actionToString");
        std::stringstream result;
        result<<"----Action: "<<TypeRegistry::getEnumName(action.actionType);
        switch(action.actionType)
        {
            case HRI::ActionType::Idle:
            {
                break;
            }
            case HRI::ActionType::ReachPosition:
            {
                result<<" (location: ("<<action.target.x()<<", "<<action.target.y()<<"))";
                break;
            }
            case HRI::ActionType::ReachPositionAndAngle:
            {
                result<<" (location: (angle: "<<action.angle<<", "<<action.target.x()<<", "<<action.target.y()<<"))";
                break;
            }
            case HRI::ActionType::ReachBall:
            {
                Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
                result<<" ("<<globalBall.x()<<", "<<globalBall.y()<<")";
                break;
            }
            case HRI::ActionType::CarryBall:
            {
                result<<" (ballDestination: ("<<action.target.x()<<", "<<action.target.y()<<")";
                break;
            }
            case HRI::ActionType::Kick:
            {
                Vector2f goalTarget = theLibCheck.goalTarget();
                result<<" (ballDestination: ("<<goalTarget.x()<<", "<<goalTarget.y()<<")";
                break;
            }
            case HRI::ActionType::CarryAndKickToGoal:
            {
                Vector2f goalTarget = theLibCheck.goalTarget();
                result<<" (goalTarget: ("<<goalTarget.x()<<", "<<goalTarget.y()<<")";
                break;
            }
        }
        return result.str();
    };

    /* [DEBUG] Prints a task-specific debug string and the action-specific string of the current action and all the following ones */
    controller.tasksToString = [&] () -> std::string
    {
        //DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.tasksToString");
        std::stringstream result;
        //DEBUG_NUMB(PRINT_DEBUG, "controller.taskQueue.size(): "<<controller.taskQueue.size());
        int taskIndex = 0;
        for(auto task : controller.taskQueue)
        {
            result<<std::string("Task ID: ")<<std::to_string(task.taskID)<<std::string("\n");
            result<<std::string("--Task: ")<<std::string(TypeRegistry::getEnumName(task.taskType))<<std::string("\n");
            int actionIndex = 0;
            for(auto action : task.actionQueue)
            {
                if(taskIndex>0 || actionIndex == controller.currentAction)
                    result<<controller.actionToString(action)<<std::string("\n");
                actionIndex++;
            }
            taskIndex++;
        }
        return result.str();
    };


    /* 
     _________________
    |                 |
    |   Update loop   |
    |_________________|

    */

    controller.GRAPHICAL_DEBUG = (GRAPHICAL_DEBUG==1 ? true : false);

    //1) Get current task
    Task currentTask = controller.getCurrentTask();

    /*2) Call checkTaskCompleted:
        2.1) Checks if current task is completed
        2.2 A) In case it is, transitions to the next task (if there is one)
    */
    if(controller.checkTaskCompleted(true))
    {
        return;
    }
    // 2.2 B) ELSE
    else
    {
        // 2.2.1) Get current action
        Action currentAction = controller.getCurrentAction();

        Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        
        // 2.2.2) Based on the current action type, check if the completion conditions for that type are verified
        //          -> checkActionCompleted TRANSITIONS to the next action if conditions are verified
        switch(currentAction.actionType)
        {
            //No completion CONDITION for the Idle action (the only action of the Idle task) -> It is automatically overwritten once a real task is added
            case HRI::ActionType::Idle:
            {
                controller.updateCurrentDestination(theRobotPose.translation);
                controller.updateCurrentBallDestination(globalBall);
                
                break;
            }
            //ReachPosition CONDITION: the robot is within controller.reachPositionDistanceThreshold mm from the desired destination controller.currentRobotDestination
            case HRI::ActionType::ReachPosition:
            {
                controller.updateCurrentDestination(currentAction.target);
                controller.updateCurrentBallDestination(globalBall);
                
                bool completed = controller.checkActionCompleted(
                    theLibCheck.distance(theRobotPose.translation, controller.currentRobotDestination)<controller.reachPositionDistanceThreshold
                );
                break;
            }
            //ReachPositionAndAngle CONDITION: the robot is within controller.reachPositionDistanceThreshold mm from the desired destination controller.currentRobotDestination
            //and its angle controller.currentRobotDestination.rotation (it's a Pose2f) is within (-Angle::fromDegrees(controller.reachPositionAngleThreshold), Angle::fromDegrees(controller.reachPositionAngleThreshold)) 
            case HRI::ActionType::ReachPositionAndAngle:
            {
                controller.updateCurrentDestination(Pose2f(currentAction.angle, currentAction.target.x(), currentAction.target.y()));
                controller.updateCurrentBallDestination(globalBall);
                
                bool completed = controller.checkActionCompleted(
                    theLibCheck.distance(theRobotPose.translation, controller.currentRobotDestination)<controller.reachPositionDistanceThreshold
                    &&
                    std::abs(theRobotPose.rotation - controller.currentRobotDestination.rotation) < Angle::fromDegrees(controller.reachPositionAngleThreshold)
                );
                break;
            }
            //ReachBall CONDITION:: the robot is within controller.reachPositionDistanceThreshold mm from the ball
            case HRI::ActionType::ReachBall:
            {
                controller.updateCurrentDestination(globalBall);
                controller.updateCurrentBallDestination(globalBall);

                controller.checkActionCompleted(
                    theLibCheck.distance(theRobotPose.translation, controller.currentRobotDestination)<controller.reachPositionDistanceThreshold
                );
                break;
            }
            //CarryBall CONDITION:: the ball is within controller.ballCarrierDistanceThreshold mm from the desired target controller.currentBallDestination
            case HRI::ActionType::CarryBall:
            {
                controller.updateCurrentDestination(globalBall);
                controller.updateCurrentBallDestination(currentAction.target);
                
                controller.checkActionCompleted(
                    theLibCheck.distance(globalBall, controller.currentBallDestination)<controller.ballCarrierDistanceThreshold
                );
                break;
            }
            //Kick CONDITION:: the ball is within controller.kickDistanceThreshold mm from the desired target controller.currentBallDestination
            case HRI::ActionType::Kick:
            {
                controller.updateCurrentDestination(globalBall);
                controller.updateCurrentBallDestination(currentAction.target);
                
                controller.checkActionCompleted(
                    theLibCheck.distance(globalBall, controller.currentBallDestination)<controller.kickDistanceThreshold
                );
                break;
            }
            //Kick CONDITION:: the ball is within controller.kickDistanceThreshold mm from the desired target controller.currentBallDestination (which is the goal center)
            case HRI::ActionType::CarryAndKickToGoal:
            {
                Vector2f goalTarget = theLibCheck.goalTarget();
                controller.updateCurrentDestination(globalBall);
                controller.updateCurrentBallDestination(goalTarget);
                
                controller.checkActionCompleted(
                    theLibCheck.distance(globalBall, controller.currentBallDestination)<controller.kickDistanceThreshold
                );
                break;
            }
        }
    }
    
}


MAKE_MODULE(TaskControllerProvider, modeling)