/**
 * @file Representations/Communication/ExternalServerCommunicationController/ExternalServerCommunicationControl.h
 *
 * Declaration of struct ExternalServerCommunicationControl to hold all useful info to receive commands 
 * and manage the communication protocol
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"

#include "Representations/BehaviorControl/TasksProvider/TaskController.h"

/**
 * @struct ExternalServerCommunicationControl
 *
 * This representation is used to store received commands and manage the communication protocol
 * 
 * Struct containing modeling info about the opponent goal targetable areas
 */

STREAMABLE(ExternalServerCommunicationControl,
{

  FUNCTION(std::vector<Task>(long lastTaskID)) getNewTasks;

  ,

  //Loaded from ExternalServerCommunicationControl.cfg, see ExternalServerCommunicationControl for explanation

  (bool) graphicalDebug,
  (std::vector<Task>) currentTaskQueue,

  
});
