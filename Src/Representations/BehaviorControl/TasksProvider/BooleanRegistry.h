/**
 * @file BooleanRegistry.h
 *
 * Declaration of the STREAMABLE struct BooleanRegistry to hold all booleans that have to be periodically sent to the control client
 * and the function to compute them
 *
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Function.h"

#include <iostream>


/**
 * @struct BooleanRegistry
 * 
 * Struct containing the functions that compute the booleans to be sent and a function to create a string to be sent as a message
 * 
 */
STREAMABLE(BooleanRegistry,
{
  void draw() const; //NOT CURRENTLY USED

  /* Functions that compute booleans to send */
  //FUNCTION(bool()) isPassAvailable;
  FUNCTION(bool()) isCurrentActionCompleted;
  FUNCTION(bool()) isBatteryLow;

  /* Returns a string with all booleans, ready to be sent to the client */
  FUNCTION(std::string()) getString;

  BooleanRegistry();
  ,

  /* When false, booleans are sent only when the TaskController is in Plan mode (overriding the sending frequency of the ExternalCommunicationController) */
  (bool)(false) ALWAYS_SEND,
});
inline BooleanRegistry::BooleanRegistry() {}