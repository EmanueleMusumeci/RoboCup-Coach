#!/bin/sh

#modify the right hand side with the root of your robocup folder
FRAMEWORK_ROOT=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
REPO_ROOT=$( cd "$(dirname "$FRAMEWORK_ROOT")" ; pwd -P )

export SPLROOT=${FRAMEWORK_ROOT}
export SPLCLIENTS=${FRAMEWORK_ROOT}/robocup_spl_temporal_goals
export SPLBUILD=${FRAMEWORK_ROOT}/Build
export SPLCONFIG=${FRAMEWORK_ROOT}/Config
export SPLSCENES=${FRAMEWORK_ROOT}/Config/Scenes
export SPLMAKE=${FRAMEWORK_ROOT}/Make/Linux
export SPLREP=${FRAMEWORK_ROOT}/Src/Representations
export SPLMOD=${FRAMEWORK_ROOT}/Src/Modules
export SPLSRC=${FRAMEWORK_ROOT}/Src
export SPLSIM=${FRAMEWORK_ROOT}/Build/Linux/SimRobot/Develop
export SPLCONTROLLER=${REPO_ROOT}/GameController
export SPLBUSH=${FRAMEWORK_ROOT}/Build/Linux/bush/Develop
export SPLINSTALL=${FRAMEWORK_ROOT}/Install


#Compile
alias pltlmcd="cd $SPLMAKE; make -C $SPLMAKE CONFIG=Develop -j4"


#Copyfiles
alias pltl_copd_noargs="$SPLMAKE/copyfiles Develop"
alias pltl_copd_loud="$SPLMAKE/copyfiles Develop -b -v 70 -c red -m 19 -o 10019 -t 19"
alias pltl_copd_lowvolume="$SPLMAKE/copyfiles Develop -b -v 50 -c red -m 19 -o 10019 -t 19"
alias pltl_copd_silent="$SPLMAKE/copyfiles Develop -b -v 0 -c red -m 19 -o 10019 -t 19"

#Run SimRobot without scenes
alias pltlsr="cd $SPLSIM; ./SimRobot"



#############
# 6x4 Field #
#############

#Run single-robot scenario with obstacle
alias fond_striker_no_obstacle="reset; ${SPLSIM}/SimRobot ${SPLSCENES}/fond_striker_no_obstacle.ros2"

#Run single-robot scenario with obstacle
alias fond_striker_with_obstacle="reset; ${SPLSIM}/SimRobot ${SPLSCENES}/fond_striker_with_obstacle.ros2"

#Run multi-robot scenario without obstacle
alias fond_striker_jolly_no_obstacle="reset; ${SPLSIM}/SimRobot ${SPLSCENES}/fond_striker_jolly_no_obstacle.ros2"

#Run multi-robot scenario with obstacle
alias fond_striker_jolly_with_obstacle="reset; ${SPLSIM}/SimRobot ${SPLSCENES}/fond_striker_jolly_with_obstacle.ros2"

#Run adversarial scenario
alias adversarial_strikers="reset; ${SPLSIM}/SimRobot ${SPLSCENES}/GameFast1vs1.ros2"


#############
# 4x3 Field #
#############

#Run single-robot scenario with obstacle
alias fond_striker_no_obstacle_4x3="reset; ${SPLSIM}/SimRobot ${SPLSCENES}/fond_striker_no_obstacle_4x3.ros2"

#Run single-robot scenario with obstacle
alias fond_striker_with_obstacle_4x3="reset; ${SPLSIM}/SimRobot ${SPLSCENES}/fond_striker_with_obstacle_4x3.ros2"

#Run multi-robot scenario without obstacle
alias fond_striker_jolly_no_obstacle_4x3="reset; ${SPLSIM}/SimRobot ${SPLSCENES}/fond_striker_jolly_no_obstacle_4x3.ros2"

#Run multi-robot scenario with obstacle
alias fond_striker_jolly_with_obstacle_4x3="reset; ${SPLSIM}/SimRobot ${SPLSCENES}/fond_striker_jolly_with_obstacle_4x3.ros2"

#Run adversarial scenario
alias adversarial_strikers_4x3="reset; ${SPLSIM}/SimRobot ${SPLSCENES}/GameFast1vs1_4x3.ros2"

#Shortcut for running an experiment (insert command-line arguments)
alias pltlre="reset; /home/asc/anaconda3/envs/robocup/bin/python3 ${SPLCLIENTS}/run_experiment.py "

#Shortcut for running a benchmark (insert command-line arguments)
alias pltlrb="reset; /home/asc/anaconda3/envs/robocup/bin/python3 ${SPLCLIENTS}/run_benchmark.py "

#Shortcut for running the vocal frontend interface (on localhost by default, use command-line arguments to change IP of the main server)
alias pltlrvi="reset; /home/asc/anaconda3/envs/robocup/bin/python3 ${SPLCLIENTS}/run_vocal_interface.py "