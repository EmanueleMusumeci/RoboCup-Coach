/**
* @file ContextCoordinator.cpp
*   This file implements the team coordination module
* @author Francesco Riccio, Emanuele Borzi
*/

//NOTICE: this version of the ContextCoordinator assigns roles statically. For a complete version, please contact suriani@diag.uniroma1.it

#include "ContextCoordinator.h"

#include <unistd.h>
#include <iostream>
#include "Representations/SPQR-Libraries/ConfigFile/ConfigFile.h"




#define NORM(x, y) sqrt(x*x + y*y)

MAKE_MODULE(ContextCoordinator, behaviorControl)

ContextCoordinator::ContextCoordinator() {}

void ContextCoordinator::update(Role& role)
{

    if(theGameInfo.state == STATE_PLAYING)
    {
        if (theFrameInfo.time - theBallModel.timeWhenLastSeen < time_when_last_seen)
        {

            ballSeen=true;
        }
        else if ( theTeamBallModel.isValid)
        {
            teamBall=true;
            ballSeen=false;
        }
        else
        {
            ballSeen=false;
            teamBall=false;
        }

        if(!ballSeen && !teamBall)
        {
            prev_status = current_status;
            current_status = Role::search_for_ball;
        }

        else
        {
            prev_status = current_status;
            current_status = Role::playing;
        }


        /// playing context
        if(current_status == Role::playing)
        {
            switch(theRobotInfo.number)
            {
                case 2:
                {
                    role.role = Role::supporter;
                    break;
                }
                case 3:
                {
                    role.role = Role::striker;
                    break;
                }
                case 4:
                {
                    role.role = Role::defender;
                    break;
                }
                case 5:
                {
                    role.role = Role::jolly;
                    break;
                }
            }
        }
        /// search for ball context
        else if(current_status == Role::search_for_ball && (prev_status==Role::no_context || prev_status==Role::playing))
        {
            switch(theRobotInfo.number)
            {
                case 2:
                {
                    role.role = Role::searcher_1;
                    break;
                }
                case 3:
                {
                    role.role = Role::searcher_3;
                    break;
                }
                case 4:
                {
                    role.role = Role::searcher_2;
                    break;
                }
                case 5:
                {
                    role.role = Role::searcher_4;
                    break;
                }
            }
        }

    }
    else
    {
        role.role = Role::undefined;
    }

    if(theRobotInfo.number == 1)
    {
        role.role = Role::goalie;
    }
}
