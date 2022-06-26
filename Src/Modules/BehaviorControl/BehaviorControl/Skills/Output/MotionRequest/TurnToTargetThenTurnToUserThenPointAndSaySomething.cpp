/**
 * @file TurnToTargetThenTurnToUserAndSaySomething.cpp
 *
 * This skill turns the robot to the target, then wraps the skill TurnToUserThenPointAndSaySomething
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

SKILL_IMPLEMENTATION(TurnToTargetThenTurnToUserThenPointAndSaySomethingImpl,
{,
  IMPLEMENTS(TurnToTargetThenTurnToUserThenPointAndSaySomething),
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(Stand),
  CALLS(WalkToTargetPathPlanner),
  CALLS(WalkToTarget),

  CALLS(PointAtWithArm),
  CALLS(TurnToUserThenPointAndSaySomething),

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

class TurnToTargetThenTurnToUserThenPointAndSaySomethingImpl : public TurnToTargetThenTurnToUserThenPointAndSaySomethingImplBase
{
    bool soundPlaying = false;
    bool finishedPlaying = false;

    option(TurnToTargetThenTurnToUserThenPointAndSaySomething)
    {
        initial_state(start)
        {
            std::cout<<"TurnToTargetThenTurnToUserThenPointAndSaySomethingSkill: start"<<std::endl;
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
                    std::cout<<"turnToPosition -> speakToHuman: aligned to target"<<std::endl;
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
                if(theTurnToUserThenPointAndSaySomethingSkill.isDone())
                {
                    std::cout << "interactWithHuman -> terminalState: sound played" << std::endl;
                    goto terminalState;
                }
            }
            
            action
            {
                theTurnToUserThenPointAndSaySomethingSkill(Vector3f(p.userPosition.x(), p.userPosition.y(), p.userPosition.z()), 
                                                        Vector3f(p.pointAtPosition.x(), p.pointAtPosition.y(), 0.f), 
                                                        p.sound_file_name);
            }
        }

        target_state(terminalState)
        {
            action
            {
                theActivitySkill(BehaviorStatus::interacting_with_human);

                theStandSkill();

                Vector2f localTarget = theLibCheck.glob2Rel(p.pointAtPosition.x(), p.pointAtPosition.y()).translation;
                thePointAtWithArmSkill(Vector3f(localTarget.x(), localTarget.y(), 0.f), Arms::Arm::left);

                Vector2f localLookTarget = theLibCheck.glob2Rel(p.userPosition.x(), p.userPosition.y()).translation;
                theLookAtPointSkill(Vector3f(localLookTarget.x(), localLookTarget.y(), p.userPosition.z()));
            }
        }
    }
    
    Angle calcAngleToTarget(Pose2f target) const
    {
        return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
    }
};

MAKE_SKILL_IMPLEMENTATION(TurnToTargetThenTurnToUserThenPointAndSaySomethingImpl);