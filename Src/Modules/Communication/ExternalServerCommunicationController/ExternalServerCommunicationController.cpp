#include "ExternalServerCommunicationController.h"
#include "Modules/BehaviorControl/TasksProvider/TaskControllerProvider.h"
#include "Tools/Modeling/Obstacle.h"
#include "Platform/SystemCall.h"
#include <unistd.h>
#include <iostream>
#include "Representations/BehaviorControl/Libraries/LibCheck.h"

#include "Platform/File.h"


#define CONTROL_DEVICE_COMMUNICATION_READ_PORT_BASE 65001
#define CONTROL_DEVICE_COMMUNICATION_WRITE_PORT_BASE 65101

#define DEBUG_NUMB(print_debug, message) \
  if(print_debug) std::cout<<"[Robot #"<<theRobotInfo.number<<"] " << message << std::endl; \

#define BUFFER_SIZE 1024

#define ACTION_QUEUE_TAG_STRING std::string("taskQueue")
#define PLAN_ACTION_TAG_STRING std::string("PlanAction")
#define LAST_TASK_ID_REQUEST_STRING std::string("lastTaskID?")
#define LAST_TASK_QUEUE_REQUEST_STRING std::string("lastTaskQueue?")
#define RESET_TASKS_STRING std::string("resetTasks")
#define DELETE_TASK_STRING std::string("deleteTask")

//Macro used to determine if a string starts with another string
#define string_startswith(string, cmp_string) \
    string.rfind(cmp_string, 0) == 0

//Strings used by the protocol to represent parameter types
#define INT_TYPE_STRING "int"
#define FLOAT_TYPE_STRING "float"
#define STR_TYPE_STRING "str"
#define BOOL_TYPE_STRING "bool"

/*
#define __STRINGIFY__(taskType) taskType
#define CREATE_TASK(taskType, position, taskID) TaskControllerProvider::__STRINGIFY__(##taskType)##Task(position, taskID)
#define CORRECT_TASK_SELECTOR(taskVar, position, taskID, taskQueue) \
switch(##taskVar##)\
{\
    FOREACH_ENUM(HRI::TaskType, task)\
    {\
        case HRI::TaskType::task:\
        {\
            taskQueue.push_back(CREATE_TASK(task, position, taskID));\
        }\
    }\
}\
*/

//TODO Create an external NetworkProtocol class to customize network protocol
//TODO Automatize using factories, based on the provided NetworkProtocol instance

ExternalServerCommunicationController::ExternalServerCommunicationController(){
    //Load file containing dir of the json specifications file
    std::string bhdir = File::getBHDir();
    InMapFile stream(bhdir + "/Config/Scenarios/Default/externalServerCommunicationController.cfg", true);
    ASSERT(stream.exists());
    
    //Load the TARGET_IP_ADDRESS (address of the Python controller server)
    stream.select("TARGET_IP_ADDRESS", -2, nullptr);
    stream >> this->TARGET_IP_ADDRESS;
    stream.deselect();

    //Load the READ_IP_ADDRESS (local ip address of the controlled robot)
    stream.select("READ_IP_ADDRESS", -2, nullptr);
    stream >> this->READ_IP_ADDRESS;
    stream.deselect();

    ASSERT(this->TARGET_IP_ADDRESS.length()>0);
    ASSERT(this->READ_IP_ADDRESS.length()>0);

    //IF on the robot, make this print well visible in the log
    if(SystemCall::getMode() == SystemCall::Mode::physicalRobot)
    {
        READ_IP_ADDRESS = SystemCall::getHostAddr();
    }
    else
    {
        READ_IP_ADDRESS = std::string("127.0.0.1");
        TARGET_IP_ADDRESS = std::string("127.0.0.1");
    }
    std::cout<<"#################################\n\n# LOCAL INTERFACE IP: "<<READ_IP_ADDRESS<<" - "<<SystemCall::getHostName()<<" #\n\n#################################"<<std::endl;
    std::cout<<"#################################\n\n# TARGET INTERFACE IP: "<<TARGET_IP_ADDRESS<<" #\n\n#################################"<<std::endl;

    //Setup the WRITE socket (for outgoing messages)
    int write_port_number;
    DEBUG_NUMB(true, "theOwnTeamInfo.teamColor: "<<std::to_string(theOwnTeamInfo.teamColor));    
    DEBUG_NUMB(true, "theOwnTeamInfo.teamColor == 3: "<<std::to_string(theOwnTeamInfo.teamColor == 3));    
    if(ONE_VS_ONE_MODE && theOwnTeamInfo.teamColor == 3)
    {
        write_port_number = CONTROL_DEVICE_COMMUNICATION_WRITE_PORT_BASE + theRobotInfo.number-1;
    }
    else
    {
        write_port_number = CONTROL_DEVICE_COMMUNICATION_WRITE_PORT_BASE + theRobotInfo.number-1 - 1000;
    }
    DEBUG_NUMB(true, "Trying to setup write socket with target address: "<<TARGET_IP_ADDRESS<<":"<<write_port_number);
    this->udp_write_socket.setTarget(TARGET_IP_ADDRESS.c_str(), write_port_number);
    this->udp_write_socket.setBlocking(false);
    DEBUG_NUMB(true, "Write socket set with target address: "<<TARGET_IP_ADDRESS<<":"<<write_port_number);

    //Setup the READ socket (for incoming messages)

    int read_port_number;
    if(ONE_VS_ONE_MODE && theOwnTeamInfo.teamColor == 3)
    {
        read_port_number = CONTROL_DEVICE_COMMUNICATION_READ_PORT_BASE + theRobotInfo.number-1;
    }
    else
    {
        read_port_number = CONTROL_DEVICE_COMMUNICATION_READ_PORT_BASE + theRobotInfo.number-1 - 1000;
    }
    DEBUG_NUMB(true, "Trying to bind read socket on address: "<<READ_IP_ADDRESS<<":"<<read_port_number);
    if(!this->udp_read_socket.bind(READ_IP_ADDRESS.c_str(), read_port_number))
    {
        DEBUG_NUMB(true, "Could not bind read socket! Exiting...");
        exit(1);
    }
    this->udp_read_socket.setBlocking(false);
    DEBUG_NUMB(true, "Read socket bound on address: "<<READ_IP_ADDRESS<<":"<<read_port_number);
    

    //Setup all necessary counters
    this->cycles_since_last_keepalive_check = 0; 
    this->client_alive = false;
    this->awaiting_keepalive_response = false;
    
    this->cycles_since_robot_pose_update = 0;
    this->cycles_since_ball_update = 0;
    this->cycles_since_obstacles_update = 0; 
}


//String for MESSAGE requesting a KEEPALIVE 
std::string ExternalServerCommunicationController::keepalive_check_string(){
    std::string ret_string;
    ret_string.append("uthere?");
    return ret_string;
}

//String for MESSAGE communicating the last received task ID and the last completed task ID
std::string ExternalServerCommunicationController::last_task_id_string(){
    std::string ret_string;
    ret_string.append("lastTaskID,");
    ret_string.append(std::to_string(theTaskController.lastReceivedTaskID));
    ret_string.append(",");
    ret_string.append(std::to_string(theTaskController.lastCompletedTaskID));
    return ret_string;
}

//String for MESSAGE communicating the latest version of the task queue. Sent:
// a) When the Python server asks for it
// b) Periodically
std::string ExternalServerCommunicationController::last_task_queue_string(){
    std::string ret_string;
    ret_string.append("lastTaskQueue;");
    ret_string.append("lastTaskID,");
    ret_string.append(std::to_string(theTaskController.lastReceivedTaskID));
    ret_string.append(",");
    ret_string.append(std::to_string(theTaskController.lastCompletedTaskID));
    for(auto task : theTaskController.taskQueue)
    {
        ret_string.append(";");
        switch(task.taskType)
        {
            case HRI::TaskType::GoToPosition:
            {
                ret_string.append("GoToPosition,"+std::to_string(task.taskID)+","+std::to_string(task.finalPosition.x())+","+std::to_string(task.finalPosition.y()));
                break;
            }
            case HRI::TaskType::CarryBallToPosition:
            {
                ret_string.append("CarryBallToPosition,"+std::to_string(task.taskID)+","+std::to_string(task.finalPosition.x())+","+std::to_string(task.finalPosition.y()));
                break;
            }
            case HRI::TaskType::KickBallToPosition:
            {
                ret_string.append("KickBallToPosition,"+std::to_string(task.taskID)+","+std::to_string(task.finalPosition.x())+","+std::to_string(task.finalPosition.y()));
                break;
            }
            case HRI::TaskType::ScoreGoalTask:
            {
                ret_string.append("ScoreGoal,"+std::to_string(task.taskID));
                break;
            }
            case HRI::TaskType::InitialSpeech:
            {
                ret_string.append("InitialSpeech,"+std::to_string(task.taskID));
                break;
            }
        }
    }
        
    return ret_string;
}

//String for MESSAGE containing the robot <rotation, position_x, position_y>
std::string ExternalServerCommunicationController::robot_pose_to_sendable_string(){

    std::string ret_string;
    ret_string.append("robot_pose,");
    ret_string.append(std::to_string(theRobotPose.rotation));
    ret_string.append(",");
    ret_string.append(std::to_string(theRobotPose.translation.x()));
    ret_string.append(",");
    ret_string.append(std::to_string(theRobotPose.translation.y()));
    return ret_string;
    
}

//String for MESSAGE containing the ball <position_x, position_y>
std::string ExternalServerCommunicationController::ball_position_to_sendable_string(){

    Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
    std::string ret_string;
    ret_string.append("ball_position,");
    ret_string.append(std::to_string(globalBall.x()));
    ret_string.append(",");
    ret_string.append(std::to_string(globalBall.y()));
    return ret_string;
    
}

//String for MESSAGE containing the ball <position_x, position_y>
std::string ExternalServerCommunicationController::role_to_sendable_string(){
    std::string ret_string;
    ret_string.append("robot_role,");
    ret_string.append(TypeRegistry::getEnumName(static_cast<Role::RoleType>(theRole.role)));
    return ret_string;
}

//String for MESSAGE containing all the obstacles <position_x, position_y>
std::string ExternalServerCommunicationController::obstacles_to_sendable_string(){

    std::string ret_string;
    std::string obs_string;

    int obs_number = 0;
    
    ret_string.append("obstacles");
    
    for(auto obs : theObstacleModel.obstacles){
        obs_number++;
        if(obs.type == Obstacle::opponent){
            Vector2f globalObstacle = theLibCheck.rel2Glob(obs.center.x(), obs.center.y()).translation;
            ret_string.append(";");
            ret_string.append(std::to_string(globalObstacle.x()));
            ret_string.append(",");
            ret_string.append(std::to_string(globalObstacle.y()));
        }
    }
    return ret_string;
}

//String for MESSAGE requesting a KEEPALIVE 
std::string ExternalServerCommunicationController::plan_action_completed_string(){
    std::string ret_string;
    ret_string.append("planActionCompleted;");
    ret_string.append("lastTaskID,");
    ret_string.append(std::to_string(theTaskController.lastCompletedTaskID));
    return ret_string;
}

//Given a string to be sent as a MESSAGE, prefixes a HEADER to it containing the timestamp (if required) and the robot number (if required)
void ExternalServerCommunicationController::send_data_string(std::string str, bool prefix_timestamp, bool prefix_robot_number, bool print_message)
{
    std::string header;
    if(prefix_timestamp)
    {
        header.append("timestamp,");
        header.append(std::to_string(Time::getCurrentSystemTime()));
        header.append(std::string("."));
    }
    if(prefix_robot_number)
    {
        header.append("robot_number,");
        header.append(std::to_string(theRobotInfo.number));
    }
    header.append("|");

    str = header + str;
    const char *s_str = str.c_str();
    if(print_message) std::cout<<str.c_str()<<std::endl;
    this->udp_write_socket.write(s_str, str.length());    
}

//Given a string and a delimiter string, returns a std::vector with all the strings delimited by the "delimiter" string 
// (or the string itself if it does not contain the delimiter)
std::vector<std::string> ExternalServerCommunicationController::getTokens(std::string& str, std::string delimiter)
{
    std::vector<std::string> tokens;    

    int delimPosition = 0;
    while ((delimPosition = str.find(delimiter)) != std::string::npos) {
        tokens.push_back(str.substr(0, delimPosition));
        str.erase(0, delimPosition + delimiter.length());
    }
    tokens.push_back(str);

    return tokens;
}

/* Given a string received as a MESSAGE, handles the message based on its content:
    1) if it starts with the ACTION_QUEUE_TAG_STRING "taskQueue", it handles it as a task queue:
        1.1) Divide the message into header and payload
        1.2) Extract the task queue from the message
        1.3) For every token in sliced vector excluding first token (message tag):
            1.3.1) Extract the task parameters
            1.3.2) Parse the task type (and check that it is known)
            1.3.3) Based on task type, create the task instance and add it to the local task queue

    2) if it starts with the PLAN_ACTION_TAG_STRING "PlanAction", it handles it as an action:
        1.1) Divide the message into header and payload
        1.2) Extract the desired action from the message
            1.2.1) Extract the action type
            1.2.1) Extract the action parameters
            1.3.3) Based on action type, run the selected action with the required parameters
        
    2) if it starts with the LAST_TASK_ID_REQUEST_STRING "lastTaskId?", it sends the last_task_queue_string() in response

    3) if it starts with the RESET_TASKS_STRING "resetTasks", it resets the taskQueue in the TaskController

    4) if it starts with the DELETE_TASK_STRING "deleteTask,<taskID>", it calls the deleteSingleTask method of the TaskController
*/
void ExternalServerCommunicationController::handleMessage(std::string message, std::vector<Task>& currentTaskQueue)
{
    DEBUG_NUMB(PRINT_DEBUG,"Handling message: "<<message);
    //DEBUG_NUMB(PRINT_DEBUG,"Message length:"<<std::to_string(message.length()));
    
    if(string_startswith(message, ACTION_QUEUE_TAG_STRING))
    {
        theTaskController.setTaskMode();
        //if(PRINT_DEBUG) std::cout<<message<<std::endl;
        
        //1.1) Divide the message into header and payload
        std::vector<std::string> taskTokens = getTokens(message, std::string("|"));
        
        ASSERT(taskTokens.size() == 2);

        //1.2) Extract the task queue from the message
        taskTokens = getTokens(taskTokens[1], std::string(";"));
        //DEBUG_NUMB(PRINT_DEBUG, taskTokens.size());

        //1.3) For every token in sliced vector excluding first token (message tag)
        for(auto taskToken : std::vector<std::string>(taskTokens.begin(), taskTokens.end())) 
        {
            //std::cout<<taskToken<<std::endl;
            
            //1.3.1) Extract the task parameters
            std::vector<std::string> taskParameters = getTokens(taskToken, std::string(","));
            
            //ASSERT(taskParameters.size()==2 || taskParameters.size()==4);
            
            //1.3.2) Parse the task type (and check that it is known)
            //ASSERT(static_cast<HRI::TaskType>(TypeRegistry::getEnumValue(typeid(HRI::TaskType).name(), taskParameters[0])) != -1);
            if(static_cast<HRI::TaskType>(TypeRegistry::getEnumValue(typeid(HRI::TaskType).name(), taskParameters[0])) == -1)
            {
                std::cout<<"Unknown TaskType: "<<std::string(taskParameters[0])<<std::endl;
                return;
            }

            //DEBUG_NUMB(PRINT_DEBUG, taskParameters.size());
            HRI::TaskType taskType = static_cast<HRI::TaskType>(TypeRegistry::getEnumValue(typeid(HRI::TaskType).name(), taskParameters[0]));
            
            //1.3.3) Based on task type, create the task instance and add it to the local task queue
            if(taskParameters.size()==4)
            {

                float coordX = std::stof(taskParameters[2]);
                float coordY = std::stof(taskParameters[3]);
                Vector2f position = Vector2f(coordX, coordY);

                //Parse taskID
                //std::cout<<taskParameters[3]<<std::endl;
                int taskID = std::stoi(taskParameters[1]);

                switch(taskType)
                {
                    case HRI::TaskType::GoToPosition:
                    {
                        currentTaskQueue.push_back(TaskControllerProvider::GoToPositionTask(position, taskID));
                        break;
                    }
                    case HRI::TaskType::CarryBallToPosition:
                    {
                        currentTaskQueue.push_back(TaskControllerProvider::CarryBallToPositionTask(position, taskID));
                        break;
                    }
                    case HRI::TaskType::KickBallToPosition:
                    {
                        currentTaskQueue.push_back(TaskControllerProvider::KickBallToPositionTask(position, taskID));
                        break;
                    }
                    default:
                    {
                        std::cout << "ERROR: task "<<taskParameters[0]<<" was not recognized"<<std::endl;
                    }
                }
            }
            else if(taskParameters.size()==2)
            {
                //Parse taskID
                int taskID = std::stoi(taskParameters[1]);

                switch(taskType)
                {
                    case HRI::TaskType::ScoreGoalTask:
                    {
                        currentTaskQueue.push_back(TaskControllerProvider::ScoreGoalTask(taskID));
                        break;
                    }
                    case HRI::TaskType::InstructionsSpeech:
                    {
                        //currentTaskQueue.push_back(TaskControllerProvider::InstructionsSpeechTask(taskID, theRobotPose.translation));
                        theTaskController.scheduleInstructionsSpeech(false, taskID);
                        break;
                    }
                    default:
                    {
                        std::cout << "ERROR: task "<<taskParameters[0]<<" was not recognized"<<std::endl;
                        return;
                    }
                }
                //DEBUG_NUMB(PRINT_DEBUG, currentTaskQueue.size());
                
            }
            else
            {
                if(PRINT_DEBUG) std::cout<<"WRONG MESSAGE STRUCTURE: "<<message<<std::endl;
            }
        }
    }
    else if(string_startswith(message, PLAN_ACTION_TAG_STRING))
    {
        theTaskController.setPlanMode();
        //DEBUG_NUMB(PRINT_DEBUG, "theTaskController.isTaskComplete(): "<<std::to_string(theTaskController.isTaskComplete()));
        //if(theTaskController.isTaskComplete() || theTaskController.isIdle())
        //{
        //    send_data_string(plan_action_completed_string(), PREFIX_TIMESTAMP, PREFIX_ROBOT_NUMBER);
        //}
        //else
        //{
            //if(PRINT_DEBUG) std::cout<<message<<std::endl;
            
            //1.1) Divide the message into header and payload
            std::vector<std::string> actionTokens = getTokens(message, std::string("|"));
            ASSERT(actionTokens.size() == 2);

            //1.2) Extract the action and its parameters from the message
            std::vector<std::string> actionFields = getTokens(actionTokens[1], std::string(";"));
            ASSERT(actionFields.size() == 2);

            //1.3) Parse the action information
            std::vector<std::string> actionInfo = getTokens(actionFields[0], std::string(","));
            ASSERT(actionInfo.size() == 2);
            std::string actionName = actionInfo[0];
            
            //1.3.1) Parse the action type (and check that it is known)
            HRI::ActionType actionType = static_cast<HRI::ActionType>(TypeRegistry::getEnumValue(typeid(HRI::ActionType).name(), actionName));
            if(actionType == -1)
            {
                std::cout<<"Unknown ActionType: "<<std::string(actionName)<<std::endl;
                return;
            }
            
            //1.3.2) Parse taskID
            int taskID = std::stoi(actionInfo[1]);

            std::vector<std::string> actionParameters;

            //1.4) Extract the action parameters
            if(actionFields[1].size() > 0)
            {
                actionParameters = getTokens(actionFields[1], std::string("/"));
            }

            //DEBUG_NUMB(PRINT_DEBUG, "actionParameters.size(): "<<std::to_string(actionParameters.size()));

            //1.5) Create an Action instance based on the number of parameters
            Action PlanAction;
            if(actionParameters.size() == 3)
            {

                //std::cout<<actionParameters[0]<<std::endl;
                std::vector<std::string> firstParamInfo = getTokens(actionParameters[0], std::string(":"));
                std::string firstParamName = firstParamInfo[0];

                std::vector<std::string> firstParamValueInfo = getTokens(firstParamInfo[1], std::string(","));
                std::string firstParamTypeString = firstParamValueInfo[1];
                std::string firstParamValueString = firstParamValueInfo[0];



                //std::cout<<actionParameters[1]<<std::endl;
                std::vector<std::string> secondParamInfo = getTokens(actionParameters[1], std::string(":"));
                std::string secondParamName = secondParamInfo[0];

                std::vector<std::string> secondParamValueInfo = getTokens(secondParamInfo[1], std::string(","));
                std::string secondParamTypeString = secondParamValueInfo[1];
                std::string secondParamValueString = secondParamValueInfo[0];



                //std::cout<<actionParameters[1]<<std::endl;
                std::vector<std::string> thirdParamInfo = getTokens(actionParameters[2], std::string(":"));
                std::string thirdParamName = thirdParamInfo[0];

                std::vector<std::string> thirdParamValueInfo = getTokens(thirdParamInfo[1], std::string(","));
                std::string thirdParamTypeString = thirdParamValueInfo[1];
                std::string thirdParamValueString = thirdParamValueInfo[0];



                //std::cout<<firstParamTypeString<<std::endl;
                //std::cout<<secondParamTypeString<<std::endl;
                //std::cout<<thirdParamTypeString<<std::endl;

                switch(actionType)
                {
                    case HRI::ActionType::ReachPositionAndAngle:
                    {
                        ASSERT(string_startswith(firstParamTypeString, INT_TYPE_STRING) || string_startswith(firstParamTypeString, FLOAT_TYPE_STRING));
                        ASSERT(string_startswith(secondParamTypeString, INT_TYPE_STRING) || string_startswith(secondParamTypeString, FLOAT_TYPE_STRING));

                        //Coordinates
                        float coordX = std::stof(secondParamValueString);
                        float coordY = std::stof(thirdParamValueString);
                        Vector2f position = Vector2f(coordX, coordY);

                        ASSERT(thirdParamValueString.length() > 0);
                        ASSERT(thirdParamTypeString.length() > 0);
                        ASSERT(string_startswith(thirdParamTypeString, INT_TYPE_STRING) || string_startswith(thirdParamTypeString, FLOAT_TYPE_STRING));
                        
                        float angle = std::stof(firstParamValueString);

                        PlanAction = Action(HRI::ActionType::ReachPositionAndAngle, position, angle);
                        break;
                        
                    }
                    default:
                    {
                        std::cout << "ERROR: action "<<actionName<<" was not recognized"<<std::endl;
                        return;
                    }
                }
            }
            else if(actionParameters.size() == 2)
            {

                //std::cout<<actionParameters[0]<<std::endl;
                std::vector<std::string> firstParamInfo = getTokens(actionParameters[0], std::string(":"));
                std::string firstParamName = firstParamInfo[0];

                std::vector<std::string> firstParamValueInfo = getTokens(firstParamInfo[1], std::string(","));
                std::string firstParamTypeString = firstParamValueInfo[1];
                std::string firstParamValueString = firstParamValueInfo[0];



                //std::cout<<actionParameters[1]<<std::endl;
                std::vector<std::string> secondParamInfo = getTokens(actionParameters[1], std::string(":"));
                std::string secondParamName = secondParamInfo[0];

                std::vector<std::string> secondParamValueInfo = getTokens(secondParamInfo[1], std::string(","));
                std::string secondParamTypeString = secondParamValueInfo[1];
                std::string secondParamValueString = secondParamValueInfo[0];


                //std::cout<<firstParamTypeString<<std::endl;
                //std::cout<<secondParamTypeString<<std::endl;


                switch(actionType)
                {
                    case HRI::ActionType::ReachPosition:
                    {
                        ASSERT(string_startswith(firstParamTypeString, INT_TYPE_STRING) || string_startswith(firstParamTypeString, FLOAT_TYPE_STRING));
                        ASSERT(string_startswith(secondParamTypeString, INT_TYPE_STRING) || string_startswith(secondParamTypeString, FLOAT_TYPE_STRING));

                        //Coordinates
                        float coordX = std::stof(firstParamValueString);
                        float coordY = std::stof(secondParamValueString);
                        Vector2f position = Vector2f(coordX, coordY);

                        PlanAction = Action(HRI::ActionType::ReachPosition, position);
                        break;
                        
                    }
                    case HRI::ActionType::CarryBall:
                    {
                        ASSERT(string_startswith(firstParamTypeString, INT_TYPE_STRING) || string_startswith(firstParamTypeString, FLOAT_TYPE_STRING));
                        ASSERT(string_startswith(secondParamTypeString, INT_TYPE_STRING) || string_startswith(secondParamTypeString, FLOAT_TYPE_STRING));

                        //Coordinates
                        float coordX = std::stof(firstParamValueString);
                        float coordY = std::stof(secondParamValueString);
                        Vector2f position = Vector2f(coordX, coordY);

                        PlanAction = Action(HRI::ActionType::CarryBall, position);
                        break;
                    }
                    case HRI::ActionType::Kick:
                    {
                        ASSERT(string_startswith(firstParamTypeString, INT_TYPE_STRING) || string_startswith(firstParamTypeString, FLOAT_TYPE_STRING));
                        ASSERT(string_startswith(secondParamTypeString, INT_TYPE_STRING) || string_startswith(secondParamTypeString, FLOAT_TYPE_STRING));
                        
                        //Coordinates
                        float coordX = std::stof(firstParamValueString);
                        float coordY = std::stof(secondParamValueString);
                        Vector2f position = Vector2f(coordX, coordY);

                        PlanAction = Action(HRI::ActionType::Kick, position);
                        break;
                    }
//TODO Pass action
                    default:
                    {
                        std::cout << "ERROR: action "<<actionName<<" was not recognized"<<std::endl;
                        return;
                    }
                }
            }
            else if(actionParameters.size()==1)
            {
                //std::cout<<actionParameters[0]<<std::endl;
                std::vector<std::string> firstParamInfo = getTokens(actionParameters[0], std::string(":"));
                std::string firstParamName = firstParamInfo[0];

                std::vector<std::string> firstParamValueInfo = getTokens(firstParamInfo[1], std::string(","));
                std::string firstParamTypeString = firstParamValueInfo[1];
                std::string firstParamValueString = firstParamValueInfo[0];

                switch(actionType)
                {
                    case HRI::ActionType::PassBall:
                    {
                        ASSERT(string_startswith(firstParamTypeString, INT_TYPE_STRING));
                        int other_robot_number = std::stoi(firstParamValueString);
                        PlanAction = Action(HRI::ActionType::PassBall, other_robot_number);
                        break;
                    }
                    default:
                    {
                        std::cout << "ERROR: task "<<actionName<<" was not recognized"<<std::endl;
                        return;
                    }
                }
            }
            else if(actionParameters.size()==0)
            {

                switch(actionType)
                {
                    case HRI::ActionType::ReachBall:
                    {
                        PlanAction = Action(HRI::ActionType::ReachBall);
                        break;
                    }
                    case HRI::ActionType::CarryAndKickToGoal:
                    {
                        PlanAction = Action(HRI::ActionType::CarryAndKickToGoal);
                        break;
                    }
                    case HRI::ActionType::Idle:
                    {
                        PlanAction = Action(HRI::ActionType::Idle);
                        break;
                    }
                    default:
                    {
                        std::cout << "ERROR: task "<<actionName<<" was not recognized"<<std::endl;
                        return;
                    }
                }
            }
            else
            {
                if(PRINT_DEBUG) std::cout<<"WRONG MESSAGE STRUCTURE: "<<message<<std::endl;
            }

            //1.6) Add current action as PlanControlledTask to task queue
            currentTaskQueue.push_back(TaskControllerProvider::PlanControlledTask(PlanAction, taskID));
        //}
    }
    else if(string_startswith(message, LAST_TASK_ID_REQUEST_STRING))
    {
        send_data_string(last_task_id_string(), PREFIX_TIMESTAMP, PREFIX_ROBOT_NUMBER, PRINT_SENT_MESSAGES);
    }
    else if(string_startswith(message, LAST_TASK_QUEUE_REQUEST_STRING))
    {
        send_data_string(last_task_queue_string(), PREFIX_TIMESTAMP, PREFIX_ROBOT_NUMBER, PRINT_SENT_MESSAGES);
    }
    else if(string_startswith(message, RESET_TASKS_STRING))
    {
        theTaskController.resetTaskQueue();
    }
    else if(string_startswith(message, DELETE_TASK_STRING))
    {
        //std::cout<<message<<std::endl;
        
        std::vector<std::string> taskTokens = getTokens(message, std::string(","));

        int taskID = std::stoi(taskTokens[1]);

        if(PRINT_DEBUG) std::cout<<"Delete single task: "<<std::to_string(taskID)<<std::endl;
        
        theTaskController.deleteSingleTask(taskID);
    }
    else
    {
        DEBUG_NUMB(PRINT_DEBUG,"Unknown message: "<<message); 
    }
    std::cout<<"\n"<<std::endl;
}


void ExternalServerCommunicationController::update(ExternalServerCommunicationControl& externalServerCommunicationControl) {

    /*
    
     ------------------
    | LAMBDA FUNCTIONS |
     ------------------

    Lambda functions allow modifying the representation from other modules
    
    */

   /* Returns a Vector of Tasks having a greater task ID than the one passed as argument */
    externalServerCommunicationControl.getNewTasks = [&] (long lastTaskID) -> std::vector<Task>
    {
        std::vector<Task> newTasks;
        for(auto task : externalServerCommunicationControl.currentTaskQueue)
        {
            if(task.taskID > lastTaskID) newTasks.push_back(task);
        }

        return newTasks;
    };


    /* 
     ____________________________________
    |                                    |
    |   Update tasks in TaskController   |
    |____________________________________|

    */

   /* Update the tasks in the TaskController with the current version of task queue */
    theTaskController.updateTasks(externalServerCommunicationControl.currentTaskQueue);

   /* Clear the current version of the task queue */
    externalServerCommunicationControl.currentTaskQueue.clear();


    /* 
     ____________________________________
    |                                    |
    |         Read new message           |
    |____________________________________|

    */

    std::string recv_string;
    
    bool received = false;
    
    char* buffer = (char*) malloc(sizeof(char) * BUFFER_SIZE);


   /* Read the latest message from the READ SOCKET */
    //DEBUG_NUMB(PRINT_DEBUG,"HELLO");
    int bt = this->udp_read_socket.read(buffer, sizeof(char) * BUFFER_SIZE);

    if(bt != -1){
        received = true;

        recv_string += std::string(buffer);
        //std::cout<<"Data received"<<std::endl;
    }
    //DEBUG_NUMB(PRINT_DEBUG,"HELLO");

    free(buffer);
    
    if(received)
    {
        DEBUG_NUMB(PRINT_DEBUG,"Received string: "<<recv_string);
    }
    else
    {
        //DEBUG_NUMB(PRINT_DEBUG,"Nothing received");
    }
    

    /* 
     ____________________________________
    |                                    |
    |      Perform keepalive check       |
    |____________________________________|

    */

   /* Perform the KEEPALIVE check or wait for a keepalive response if the keepalive message has already been sent */
    if(PERFORM_KEEPALIVE_CHECK)
    {
        //DEBUG_NUMB(PRINT_DEBUG,"this->cycles_since_last_keepalive_check: "<<this->cycles_since_last_keepalive_check);
        /* 
           if the Python server is NOT ALIVE
           OR
           the latest keepalive check has been performed more than KEEPALIVE_CHECK_FREQUENCY cycles ago
        */
        if(!this->client_alive || this->cycles_since_last_keepalive_check % KEEPALIVE_CHECK_FREQUENCY == 0)
        {
            // IF the keepalive request message has already been sent and we're waiting for a response
            if(this->awaiting_keepalive_response)
            {
                //DEBUG_NUMB(PRINT_DEBUG,"Checking keepalive response");
                // IF ANY message is received, reset the python server as alive and restart normal functioning
                if(received)
                {
                    //DEBUG_NUMB(PRINT_DEBUG,"Received keepalive response");
                    this->awaiting_keepalive_response = false;
                    this->client_alive = true;
                    this->cycles_since_last_keepalive_check = 1;
                }
                // ELSE Send another keepalive request after a while
                else if(this->cycles_since_last_keepalive_check % KEEPALIVE_CHECK_FREQUENCY == 0)
                {
                    this->awaiting_keepalive_response = false;
                }
            }
            // ELSE IF the latest keepalive check has been performed more than KEEPALIVE_CHECK_FREQUENCY cycles ago -> send another keepalive request
            else if(this->cycles_since_last_keepalive_check % KEEPALIVE_CHECK_FREQUENCY == 0)
            {
                this->client_alive = false;
                //DEBUG_NUMB(PRINT_DEBUG,"Checking if client is alive");
                send_data_string(keepalive_check_string(), PREFIX_TIMESTAMP, PREFIX_ROBOT_NUMBER, PRINT_SENT_MESSAGES);
                this->awaiting_keepalive_response = true;
            }
        }
        //ALSO,
        //IF the Python server is not alive, return prematurely
        if(!this->client_alive) return;

        this->cycles_since_last_keepalive_check++;

        //RETURN after keepalive responses (they're not useful messages for the rest of the update loop)
        if(recv_string == "yeah") return;

        //DEBUG_NUMB(PRINT_DEBUG,"After keepalive");
    }


    /* 
     ____________________________________
    |                                    |
    |  Update server with latest values  |
    |____________________________________|

    */

    //DEBUG_NUMB(PRINT_DEBUG,"B");
    
    //Send a MESSAGE with the robot pose every ROBOT_POSE_UPDATE_FREQUENCY millisecs
    if(this->cycles_since_robot_pose_update % ROBOT_POSE_UPDATE_FREQUENCY == 0){
        send_data_string(robot_pose_to_sendable_string(), PREFIX_TIMESTAMP, PREFIX_ROBOT_NUMBER, PRINT_SENT_MESSAGES);
        this->cycles_since_robot_pose_update = 0;
    }
    this->cycles_since_robot_pose_update++;

    //DEBUG_NUMB(PRINT_DEBUG,"C");
    
    //Send a MESSAGE with the ball position every BALL_POSITION_UPDATE_FREQUENCY millisecs
    if(this->cycles_since_ball_update % BALL_POSITION_UPDATE_FREQUENCY == 0){
        send_data_string(ball_position_to_sendable_string(), PREFIX_TIMESTAMP, PREFIX_ROBOT_NUMBER, PRINT_SENT_MESSAGES);
        this->cycles_since_ball_update = 0;
    }
    this->cycles_since_ball_update++;

    //Send a MESSAGE with the role every ROLE_UPDATE_FREQUENCY millisecs
    if(this->cycles_since_role_update % ROLE_UPDATE_FREQUENCY == 0){
        send_data_string(role_to_sendable_string(), PREFIX_TIMESTAMP, PREFIX_ROBOT_NUMBER, PRINT_SENT_MESSAGES);
        this->cycles_since_role_update = 0;
    }
    this->cycles_since_role_update++;


    //DEBUG_NUMB(PRINT_DEBUG,"D");

    //Send a MESSAGE with the obstacles position every OBSTACLES_UPDATE_FREQUENCY millisecs
    if(this->cycles_since_obstacles_update % OBSTACLES_UPDATE_FREQUENCY == 0){
        send_data_string(obstacles_to_sendable_string(), PREFIX_TIMESTAMP, PREFIX_ROBOT_NUMBER, PRINT_SENT_MESSAGES);
        this->cycles_since_obstacles_update = 0;
    }
    this->cycles_since_obstacles_update++;
    
    //DEBUG_NUMB(PRINT_DEBUG,"E");
    
    //Send a MESSAGE with the task queue every LAST_TASK_QUEUE_UPDATE_FREQUENCY millisecs, to ensure synchronization of the task queue
    if(this->cycles_since_task_queue_update % LAST_TASK_QUEUE_UPDATE_FREQUENCY == 0){
        send_data_string(last_task_queue_string(), PREFIX_TIMESTAMP, PREFIX_ROBOT_NUMBER, PRINT_SENT_MESSAGES);
        this->cycles_since_task_queue_update = 0;
    }
    this->cycles_since_task_queue_update++;

    
    //Send a MESSAGE with the boolean flags every BOOLEANS_UPDATE_FREQUENCY millisecs, to stream boolean flags to the Behavior Controller
    if(this->cycles_since_boolean_flags_update % BOOLEANS_UPDATE_FREQUENCY == 0 && theBooleanRegistry.ALWAYS_SEND){
        send_data_string(theBooleanRegistry.getString(), PREFIX_TIMESTAMP, PREFIX_ROBOT_NUMBER, PRINT_SENT_MESSAGES);
        this->cycles_since_boolean_flags_update = 0;
    }
    this->cycles_since_task_queue_update++;


    /* 
     ____________________________________
    |                                    |
    |     Handle the current message     |
    |____________________________________|

    */

    //Message analysis
    if(recv_string.length()>0) handleMessage(recv_string, externalServerCommunicationControl.currentTaskQueue);

}


MAKE_MODULE(ExternalServerCommunicationController, modeling)
