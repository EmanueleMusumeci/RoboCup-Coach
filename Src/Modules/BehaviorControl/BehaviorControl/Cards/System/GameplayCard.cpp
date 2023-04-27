/**
 * @file GameplayCard.cpp
 *
 * This file implements a card that represents the behavior of a robot in states in which it is not forced to do nothing.
 *
 * @author Arne Hasselbring
 */

#define PAPER

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/BehaviorControl/TasksProvider/TaskController.h"

CARD(GameplayCard,
{,
  REQUIRES(GameInfo),
  REQUIRES(Role),
  REQUIRES(OwnTeamInfo),
  REQUIRES(FallDownState),
  REQUIRES(TaskController),
  CALLS(Activity),
  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) PlanBasedStriker,
    (DeckOfCards<CardRegistry>) PlanBasedSupporter,
    (DeckOfCards<CardRegistry>) PlanBasedJolly,
    (DeckOfCards<CardRegistry>) searcher,
  }),
});

class GameplayCard : public GameplayCardBase
{
  bool preconditions() const override
  {
    //return theGameInfo.state == STATE_PLAYING;
    return theFallDownState.state != FallDownState::fallen &&
              theFallDownState.state != FallDownState::squatting && 
                (!theTaskController.isIdle() 
                  ||
                 theRole.role == Role::RoleType::searcher_1
                  ||
                 theRole.role == Role::RoleType::searcher_2
                  ||
                 theRole.role == Role::RoleType::searcher_3
                  ||
                 theRole.role == Role::RoleType::searcher_4
                );
  }

  bool postconditions() const override
  {
    //return theGameInfo.state != STATE_PLAYING;
    return theFallDownState.state == FallDownState::fallen ||
           theFallDownState.state == FallDownState::squatting ||
           (theTaskController.isIdle()
            &&
            theRole.role != Role::RoleType::searcher_1
            &&
            theRole.role != Role::RoleType::searcher_2
            &&
            theRole.role != Role::RoleType::searcher_3
            &&
            theRole.role != Role::RoleType::searcher_4
          );
  }

  void execute() override
  {
    if(theRole.role == Role::striker){
      dealer.deal(PlanBasedStriker)->call();
      setState("PlanBasedStriker");
    }
    else if(theRole.role == Role::jolly){
      dealer.deal(PlanBasedJolly)->call();
      setState("PlanBasedJolly");
    }
    else if(theRole.role == Role::searcher_1){
      dealer.deal(searcher)->call();
      setState("searcher1");
    }
    else if(theRole.role == Role::searcher_2){
      dealer.deal(searcher)->call();
      setState("searcher2");
    }
    else if(theRole.role == Role::searcher_3){
      dealer.deal(searcher)->call();
      setState("searcher3");
    }
    else if(theRole.role == Role::searcher_4){
      dealer.deal(searcher)->call();
      setState("searcher4");
    }
  }

  void reset() override
  {
    dealer.reset();
  }

  PriorityListDealer dealer; /**< The dealer which selects the card to play. */
};

MAKE_CARD(GameplayCard);
