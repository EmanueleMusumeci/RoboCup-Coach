#pragma once

#include <iostream>
#include <set>
#include "Tools/Module/Module.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Communication/UdpComm.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/TasksProvider/TaskController.h"
#include "Representations/BehaviorControl/TasksProvider/BooleanRegistry.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/Role.h"
#include "Tools/Math/Pose2f.h"

#include "Representations/Communication/ExternalServerCommunicationController/ExternalServerCommunicationControl.h"

#include "Representations/BehaviorControl/Libraries/LibCheck.h"

#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include <mutex>

MODULE(ExternalServerCommunicationController,
{,
 REQUIRES(BallModel),
 REQUIRES(RobotInfo),
 REQUIRES(LibCheck),
 REQUIRES(TeamData),
 REQUIRES(RobotPose),
 REQUIRES(ObstacleModel),
 REQUIRES(GameInfo),
 REQUIRES(OwnTeamInfo),
 REQUIRES(OpponentTeamInfo),
 REQUIRES(FieldDimensions),
 REQUIRES(Role),

 REQUIRES(BooleanRegistry),
 REQUIRES(TaskController),

 USES(ActivationGraph),
 USES(BehaviorStatus),

 PROVIDES(ExternalServerCommunicationControl),
 
 LOADS_PARAMETERS(
    {,
      (bool) PRINT_DEBUG,                           /** Used to switch debug prints */ 
      (bool) PERFORM_KEEPALIVE_CHECK,               /** Perform a keepalive check (send a message to signal being active and wait for a response) every KEEPALIVE_CHECK_FREQUENCY */ 
      (int) KEEPALIVE_CHECK_FREQUENCY,              /** Number of cycles between two different keepalive requests to client device */

      (int) ROBOT_POSE_UPDATE_FREQUENCY,            /** Number of cycles between two different robot pose updates to client device */
      (int) BALL_POSITION_UPDATE_FREQUENCY,         /** Number of cycles between two different ball info updates to client device */
      (int) ROLE_UPDATE_FREQUENCY,                  /** Number of cycles between two different role updates to client device */
      (int) OBSTACLES_UPDATE_FREQUENCY,             /** Number of cycles between two different obstacles info updates to client device */
      (int) LAST_TASK_QUEUE_UPDATE_FREQUENCY,       /** Number of cycles between two different task queue updates to client device */
      (int) BOOLEANS_UPDATE_FREQUENCY,              /** Number of cycles between two different boolean flags updates to client device */

      (bool) PREFIX_TIMESTAMP,                      /** Add the timestamp at the beginning of the message */
      (bool) PREFIX_ROBOT_NUMBER,                   /** Add the robot number to the message */

      (bool) PRINT_SENT_MESSAGES,                   /** Print each message sent by this controller */

      (bool) ONE_VS_ONE_MODE,                       /** In this mode, we're having two opponent robots play one against each other, so we'll have different ports to stream different behaviors */

    }),
});

class ExternalServerCommunicationController : public ExternalServerCommunicationControllerBase
{
private:
    void isDone();

public:
    UdpComm udp_write_socket;
    UdpComm udp_read_socket;

    std::string TARGET_IP_ADDRESS;          /** IP address of the Python server in the frontend pipeline */
    std::string READ_IP_ADDRESS;            /** IP address of this robot in the LAN */
    
    int cycles_since_robot_pose_update;
    int cycles_since_ball_update;
    int cycles_since_role_update;
    int cycles_since_obstacles_update;
    int cycles_since_task_queue_update;
    int cycles_since_boolean_flags_update;

    int cycles_since_last_keepalive_check;
    
    bool client_alive;                      /* is the Python server alive */
    bool awaiting_keepalive_response;       /* has the robot sent a keepalive request to the Python server and is awaiting a response */

    void update(ExternalServerCommunicationControl &ExternalServerCommunicationControl);

    void check_client_alive();

    std::string keepalive_check_string();

    std::string last_task_id_string();
    std::string last_task_queue_string();

    std::string robot_pose_to_sendable_string();
    std::string ball_position_to_sendable_string();
    std::string role_to_sendable_string();
    std::string obstacles_to_sendable_string();
    std::string plan_action_completed_string();

    void send_data_string(std::string str, bool prefix_timestamp, bool prefix_robot_number, bool print_message);
    bool read_data_string_from_socket(UdpComm sock, std::string& recv_str);

    std::vector<std::string> getTokens(std::string& str, std::string delimiter);
    void handleMessage(std::string message, std::vector<Task>& currentTaskQueue);

    ExternalServerCommunicationController();
};
