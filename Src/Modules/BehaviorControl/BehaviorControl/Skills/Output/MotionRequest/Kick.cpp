/**
 * @file Kick.cpp
 *
 * This file implements the implementation of the Kick skill.
 *
 * @author Graziano Specchi
 */

//NOTICE: This Skill always uses a fixed distance kick. For a complete version, please contact suriani@diag.uniroma1.it

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/KickRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Platform/SystemCall.h"
#include "Tools/RobotParts/Limbs.h"

#include <algorithm>
#include <iostream>
#include <string>

SKILL_IMPLEMENTATION(KickImpl,
{,
  IMPLEMENTS(Kick),
  REQUIRES(GetUpEngineOutput),
  REQUIRES(KickEngineOutput),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  REQUIRES(FieldBall),
  MODIFIES(MotionRequest),
});

class KickImpl : public KickImplBase
{
  option(Kick)
  {
    initial_state(launch)
    {
      transition
      {
        if(theMotionInfo.motion == MotionRequest::kick)
          goto execute;
        if(theMotionInfo.motion == MotionRequest::fall)
          goto fallen;
      }
      action
      {
        setRequest(p, true);
      }
    }

    state(execute)
    {
      transition
      {
        if(theKickEngineOutput.isLeavingPossible)
          goto finished;
        if(theMotionInfo.motion == MotionRequest::fall)
          goto fallen;
      }
      action
      {
        setRequest(p, true);
      }
    }

    target_state(finished)
    {
      action
      {
        setRequest(p, false);
      }
    }

    aborted_state(fallen)
    {
      transition
      {
        if(theMotionInfo.motion != MotionRequest::fall && (theMotionInfo.motion != MotionRequest::getUp || theGetUpEngineOutput.isLeavingPossible))
          goto launch;
      }
      action
      {
        theMotionRequest.motion = MotionRequest::getUp;
        theLibCheck.inc(LibCheck::motionRequest);
      }
    }
  }


  void setRequest(const Kick& p, bool requestKick)
  {

    if(requestKick)
      theMotionRequest.motion = MotionRequest::kick;
    else
      theMotionRequest.motion = MotionRequest::stand;
          	
    theMotionRequest.kickRequest.mirror = p.mirror;
    theMotionRequest.kickRequest.armsBackFix = p.armsBackFix;
    theMotionRequest.kickRequest.dynPoints.clear();
    theMotionRequest.kickRequest.kickMotionType = KickRequest::strongKick;

    #ifdef TARGET_ROBOT  
      theMotionRequest.kickRequest.kickMotionType = KickRequest::strongKick;
   #endif

   #ifndef TARGET_ROBOT
    theMotionRequest.kickRequest.kickMotionType = KickRequest::kick_4_5;
   #endif

    theLibCheck.inc(LibCheck::motionRequest);
  }
};

MAKE_SKILL_IMPLEMENTATION(KickImpl);
