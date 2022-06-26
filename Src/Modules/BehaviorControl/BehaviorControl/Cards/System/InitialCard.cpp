/**
 * @file InitialCard.cpp
 *
 * This file specifies the behavior for a robot in the Initial game state.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

#include "Tools/BehaviorControl/Framework/Card/Dealer.h"

CARD(InitialCard,
{,
  CALLS(Activity),
  CALLS(LookAtAngles),
  CALLS(SpecialAction),
  REQUIRES(GameInfo),
  
});

class InitialCard : public InitialCardBase
{
  bool preconditions() const override
  {
    //std::cout<<"INITIAL_CARD"<<std::endl;
    return theGameInfo.state == STATE_INITIAL;
  }

  bool postconditions() const override
  {
    //return theGameInfo.state != STATE_INITIAL;
    return theSpecialActionSkill.isDone();
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::initial);
    theLookAtAnglesSkill(0.f, 0.f, 150_deg);
    theSpecialActionSkill(SpecialActionRequest::standHigh);
  }
};

MAKE_CARD(InitialCard);
