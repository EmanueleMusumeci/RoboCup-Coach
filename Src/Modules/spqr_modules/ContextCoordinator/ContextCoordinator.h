#pragma once

#include <iostream>
#include <set>
#include "Tools/Module/Module.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Communication/RoboCupGameControlData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"


#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include <mutex>

//NOTICE: this version of the ContextCoordinator assigns roles statically. For a complete version, please contact suriani@diag.uniroma1.it

MODULE(ContextCoordinator, 
{,
 REQUIRES(GameInfo),
 REQUIRES(OpponentTeamInfo),
 REQUIRES(OwnTeamInfo),
 REQUIRES(RobotInfo),
 REQUIRES(RobotPose),
 REQUIRES(BallModel),
 REQUIRES(FrameInfo),
 REQUIRES(FallDownState),
 REQUIRES(TeamData),
 REQUIRES(TeamBallModel),
 REQUIRES(FieldDimensions),
 REQUIRES(ObstaclesFieldPercept),
 REQUIRES(LibCheck),
 USES(Role),
 PROVIDES(Role),

 LOADS_PARAMETERS(
 {,
  (int) fall_down_penalty,
  (unsigned int) time_when_last_seen,
 }),
       });

class ContextCoordinator: public ContextCoordinatorBase
{

private:

    /** Utilities */
  float norm(float x, float y){ return (float)sqrt(x*x + y*y); }
  float sign(float x){ if (x >= 0) return 1.f; else return -1.f; }


    bool ballSeen;
    bool teamBall;

public:

    bool flag = true;


    Role::Context prev_status = Role::no_context;

    Role::Context current_status = Role::no_context;

    ContextCoordinator();
    void update(Role& role);

};
