/**
 * @file TurnToUserAndSaySomething.cpp
 *
 * This skill turns the robot's head to the user and plays an audio file
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

SKILL_IMPLEMENTATION(TurnToUserAndSaySomethingImpl,
{,
  IMPLEMENTS(TurnToUserAndSaySomething),
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(Stand),
  CALLS(WalkToTargetPathPlanner),
  CALLS(WalkToTarget),

  CALLS(PointAt),
  

  REQUIRES(LibCheck),
  REQUIRES(RobotPose),

  REQUIRES(TaskController),

  USES(BehaviorStatus),

  MODIFIES(MotionRequest),

  LOADS_PARAMETERS(
  {,
    (float) walkSpeed,

    (float) initialWaitTime,
    (float) waitBeforePlayingSound,
    (float) waitForSoundToStartPlaying,

    (Rangef) smallAlignmentRange,
  }),
});

class TurnToUserAndSaySomethingImpl : public TurnToUserAndSaySomethingImplBase
{
    bool soundPlaying = false;

    option(TurnToUserAndSaySomething)
    {
        initial_state(start)
        {
            std::cout<<"TurnToUserAndSaySomethingSkill: start"<<std::endl;
            transition
            {
                if(state_time > initialWaitTime)
                goto turnHeadToUser;
            }

            action
            {
                theActivitySkill(BehaviorStatus::reaching_position);
                theLookForwardSkill();
                theStandSkill();
            }
        }

        state(turnHeadToUser)
        {
            transition
            {
                if(state_time > waitBeforePlayingSound)
                {
                    std::cout<<"turnHeadToUser -> speakToHuman: TIMEOUT"<<std::endl;
                    goto speakToHuman;
                }
            }

            action
            {
                theActivitySkill(BehaviorStatus::interacting_with_human);

                theStandSkill();

                Vector2f localLookTarget = theLibCheck.glob2Rel(p.userPosition.x(), p.userPosition.y()).translation;
                theLookAtPointSkill(Vector3f(localLookTarget.x(), localLookTarget.y(), p.userPosition.z()));
            }
        }

        state(speakToHuman)
        {
            transition
            {
                if(soundPlaying)
                {
                    std::cout<<"speakToHuman -> waitForSoundToPlay: sound started playing"<<std::endl;
                    goto waitForSoundToPlay;
                }
            }

            action
            {
                theActivitySkill(BehaviorStatus::interacting_with_human);

                theStandSkill();

                if(!soundPlaying)
                {
                    SoundPlayer::play(p.sound_file_name);
                    soundPlaying = true;
                }
            }
        }

        state(waitForSoundToPlay)
        {
            transition
            {
                if(state_time > waitForSoundToStartPlaying && !SoundPlayer::isPlaying())
                {
                    std::cout << "waitForSoundToPlay -> terminalState: sound played" << std::endl;
                    soundPlaying = false;
                    goto terminalState;
                }
            }

            action
            {
                theActivitySkill(BehaviorStatus::interacting_with_human);

                theStandSkill();

                Vector2f localLookTarget = theLibCheck.glob2Rel(p.userPosition.x(), p.userPosition.y()).translation;
                theLookAtPointSkill(Vector3f(localLookTarget.x(), localLookTarget.y(), p.userPosition.z()));
            }
        }

        target_state(terminalState)
        {
            action
            {
                theActivitySkill(BehaviorStatus::interacting_with_human);

                theStandSkill();

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

MAKE_SKILL_IMPLEMENTATION(TurnToUserAndSaySomethingImpl);