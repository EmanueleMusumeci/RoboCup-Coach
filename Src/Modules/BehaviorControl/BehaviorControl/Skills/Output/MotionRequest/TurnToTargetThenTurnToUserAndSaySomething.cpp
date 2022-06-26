/**
 * @file TurnToTargetThenTurnToUserAndSaySomething.cpp
 *
 * This skill turns the robot to the target, then wraps the skill TurnToUserAndSaySomething
 *
 * @author Emanuele Musumeci
 */
#include "Platform/Linux/SoundPlayer.h"

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"

#include "Representations/BehaviorControl/TasksProvider/TaskController.h"

#include "Tools/Math/BHMath.h"
#include "Platform/SystemCall.h"
#include <string>

SKILL_IMPLEMENTATION(TurnToTargetThenTurnToUserAndSaySomethingImpl,
{,
  IMPLEMENTS(TurnToTargetThenTurnToUserAndSaySomething),
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(Stand),
  CALLS(WalkToTargetPathPlanner),
  CALLS(WalkToTarget),

  CALLS(PointAt),
  CALLS(TurnToUserAndSaySomething),

  REQUIRES(LibCheck),
  REQUIRES(RobotPose),

  REQUIRES(TaskController),

  USES(BehaviorStatus),

  MODIFIES(MotionRequest),

  LOADS_PARAMETERS(
  {,
    (float) walkSpeed,

    (float) initialWaitTime,

    (Rangef) smallAlignmentRange,
  }),
});

class TurnToTargetThenTurnToUserAndSaySomethingImpl : public TurnToTargetThenTurnToUserAndSaySomethingImplBase
{
    bool soundPlaying = false;

    option(TurnToTargetThenTurnToUserAndSaySomething)
    {
        initial_state(start)
        {
            std::cout<<"TurnToTargetThenTurnToUserAndSaySomethingSkill: start"<<std::endl;
            transition
            {
                if(state_time > initialWaitTime)
                goto turnToTarget;
            }

            action
            {
                theActivitySkill(BehaviorStatus::reaching_position);
                theLookForwardSkill();
                theStandSkill();
            }
        }

        state(turnToTarget)
        {
            transition
            {
                if(smallAlignmentRange.isInside(calcAngleToTarget(Vector2f(p.targetPosition.x(), p.targetPosition.y())).toDegrees()))
                {
                    std::cout<<"turnToTarget -> interactWithHuman: aligned to target"<<std::endl;
                    goto interactWithHuman;
                }
            }

            action
            {
                theActivitySkill(BehaviorStatus::interacting_with_human);
            
                theWalkToTargetSkill(Pose2f(1.0f, walkSpeed, walkSpeed), Pose2f(calcAngleToTarget(Vector2f(p.targetPosition.x(), p.targetPosition.y())), 0.f, 0.f));      
                theLookForwardSkill();
            }
        }
        
        state(interactWithHuman)
        {
            transition
            {
                if(theTurnToUserAndSaySomethingSkill.isDone())
                {
                    std::cout << "interactWithHuman -> terminalState: sound played" << std::endl;
                    goto terminalState;
                }
            }
            
            action
            {
                theTurnToUserAndSaySomethingSkill(Vector3f(p.userPosition.x(), p.userPosition.y(), p.userPosition.z()), 
                                                    p.sound_file_name);
            }
        }

        target_state(terminalState)
        {
            action
            {
                theActivitySkill(BehaviorStatus::interacting_with_human);

                theStandSkill();

                theLookForwardSkill();
            }
        }
    }

    Angle calcAngleToTarget(Pose2f target) const
    {
        return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
    }
};

MAKE_SKILL_IMPLEMENTATION(TurnToTargetThenTurnToUserAndSaySomethingImpl);