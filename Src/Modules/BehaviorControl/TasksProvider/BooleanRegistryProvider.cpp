/**
 * @file BooleanRegistryProvider.cpp
 *
 * This module keeps track of the state of execution of the Obstacle Avoidance HRI routine 
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "BooleanRegistryProvider.h"

//Prints a debug message prefixed by the robot number
#define DEBUG_NUMB(print_debug, message) \
  if(print_debug) std::cout<<"[Robot #"<<theRobotInfo.number<<"] " << message << std::endl; 

//Prints a debug message prefixed by the robot number, showing the code being debugged and its computed value
#define __STRINGIFY_I(arg) #arg
#define __STRINGIFY(arg) __STRINGIFY_I(arg)
#define DEBUG_CODE(code) \
  std::cout<<"[Robot #"<<std::to_string(theRobotInfo.number)<<"] "<<__STRINGIFY(code)<<": "<<std::to_string(code)<<std::endl;

BooleanRegistryProvider::BooleanRegistryProvider(){}

void BooleanRegistryProvider::update(BooleanRegistry& booleanRegistry)
{
    
    //DEBUG_NUMB(PRINT_DEBUG, "BooleanRegistry update function\n\n\n");

    //CFG parameters
    booleanRegistry.ALWAYS_SEND = ALWAYS_SEND;

    booleanRegistry.isCurrentActionCompleted = [&] () -> bool
    {
      return theTaskController.isTaskComplete();
    };

    booleanRegistry.isBatteryLow = [&] () -> bool
    {
      return theRobotHealth.batteryLevel < BATTERY_LOW;
    };

    booleanRegistry.getString = [&] () -> std::string
    {
      std::string ret_string;
      ret_string.append("booleanFlags;");
      ret_string.append("is_current_action_completed:");
      ret_string.append(std::to_string(booleanRegistry.isCurrentActionCompleted()));
      return ret_string;
    };
    
}


MAKE_MODULE(BooleanRegistryProvider, modeling)