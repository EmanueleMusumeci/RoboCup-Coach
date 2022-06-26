# Dynamic Behaviors Generation in RoboCup SPL from PDDL+PLTLf formal specification

The idea behind this project is to make a first step towards real-time coaching of robots in RoboCup Soccer SPL, by using online planning in a PDDL domain using temporal goals and constraints. The presented architecture allows a human in the loop to speak temporal goals over PDDL predicates, influencing robot behaviors in real-time.

![Possible use case of this project, where a human conditions robot behaviors](/Intro_image.png "The human is now enabled to coach robots at a high-level, through textual or vocal commands, conditioning the generated policies with temporal goals over a pre-defined and context-specific set of conditionable predicates.")


## Paper: [Adaptive Team Behavior Planning using Human Coach Commands](https://sites.google.com/diag.uniroma1.it/robocupcoach/home?authuser=0) 
### Abstract

In the robot’s operating life, the agent is required to act in real environments dealing with rules and constraints that humans ask to satisfy. The set of rules specified by the human might influence the role of the agent without changing the goal or the current task. With similar constraints, a robotic agent in a RoboCup soccer match deals with a partially unobservable, unpredictable and dynamic scenario and can benefit from precise human indications that can condition the robot behavior before or during the time of the match. To this end, classical planning methodologies can be enriched with temporal goals and constraints that enforce non-Markovian properties on past traces. The proposed work aims at exploring the application of real-time dynamic generation of policies whose possible trajectories are compliant with a set of PPLTL rules, introducing novel human-robot interaction modalities for the high-level control of team strategies in RoboCup SPL matches.

## Install

### Supported distros:
* Ubuntu 16.04/16.10/17.04/17.10/18.04/18.10/19.04/19.10/20.04
* Linux Mint 18.x / 20.x

### Install needed libraries
Open a terminal `Ctrl`+`Alt`+`t` (usually) and type the followings commands: <br>
```
sudo apt install build-essential cmake clang make qtbase5-dev libqt5opengl5-dev libqt5svg5-dev libglew-dev net-tools graphviz xterm qt5-default libvtk6-dev zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libopenexr-dev libgdal-dev libtbb-dev libpng16-16:i386 libtbb2:i386 libjpeg62:i386 libtiff5:i386 libv4l-0:i386
```
<br>

### Configure GitHub :
  
  Install git
  ```
  sudo apt-get install git git-core git-gui git-doc gitg
  ```
  
  Follow [this guide](http://help.github.com/linux-set-up-git) to set up git (Attention: "Your Name here" means "Name Surname")

  Colorize git: 
  ```
  git config --global color.status auto
  ```
<br>


### Update and upgrade your pc and reboot
```
sudo apt-get update
sudo apt-get upgrade
sudo reboot
```
<br>


### Install OpenCV3.1
Dowload source of OpenCV:
 `$ wget https://github.com/opencv/opencv/archive/3.1.0.zip`

Unzip the files
 `$ unzip 3.1.0.zip`

Generate the make file
```
cd opencv-3.1.0
mkdir build && cd build
cmake -DWITH_QT=ON -DWITH_OPENGL=ON -DFORCE_VTK=ON -DWITH_TBB=ON -DWITH_GDAL=ON -DWITH_XINE=ON -DWITH_CUDA=OFF ..
```

**NOTICE**: if you have trouble with _stdlib_ add this flag
```
-DENABLE_PRECOMPILED_HEADERS=OFF
```

Compile
```
make -j<num of cores of your cpu>
```

Run OpenCV install script 
```
sudo make install
``` 
and then
```
sudo ldconfig
```

#### Possible problems at this step

The following errors may happen for some users, but not others. *If you incur into one, please report it to us as soon as possible, so we can update this wiki with the proper error message.*

* **IMPORTANT FOR ANACONDA/MINICONDA USERS:** Either of them can mess with the installation process by generating seemingly unrelated errors *(e.g. an undefined reference to libtiff)*. The easiest way is to temporarily disable them by following this procedure: [(source)](https://github.com/colmap/colmap/issues/188#issuecomment-440665679)
  - Open your .bashrc: `$ nano ~/.bashrc`
  - Locate the line that adds <ana/mini>conda to your PATH. It may look like `export PATH="/home/<user>/anaconda3/bin:$PATH` for anaconda, and similarly for miniconda. If you can't find it in .bashrc, try your .profile as well.
  - Comment out that line *(add a `#` as the first character of that line)*.
  - *Open a new terminal window* and carry out the OpenCV installation procedure as above.
  - Un-comment that line in your .bashrc
* **If you get an error message related to ffmpeg:** Install ffmpeg with the command `sudo apt install ffmpeg`
* **If the compiler says some flag (one of which may be CODEC_FLAG_GLOBAL_HEADER) was not declared:** You may have to add the missing defines to the OpenCV code. Open the file `<opencv folder>/modules/videoio/src/cap_ffmpeg_impl.hpp` and add the folowing lines: [(source)](https://stackoverflow.com/a/47005401)
```c++
    #define AV_CODEC_FLAG_GLOBAL_HEADER (1 << 22)
    #define CODEC_FLAG_GLOBAL_HEADER AV_CODEC_FLAG_GLOBAL_HEADER
    #define AVFMT_RAWPICTURE 0x0020
```
* **If you get errors related to libasound:** Install the corresponding headers with the command `sudo apt install libasound2-dev`

### Clone this repo
```
git clone https://github.com/EmanueleMusumeci/SPL_RoboCup_Dynamic_Behavior_Generation_From_PDDL_PLTLf_specification.git
```

### Compile the code 
#### Ubuntu 16 or Mint 18 users
Set Clang-6.0 before you compile: 
```
sudo apt-get install -y clang-6.0 
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-3.8 100 
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-6.0 1000 
sudo update-alternatives --install /usr/bin/clang++ clang /usr/bin/clang-3.8 100 
sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-3.8 100 
sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-6.0 1000 
sudo update-alternatives --config clang 
sudo update-alternatives --config clang++
```
#### All users
To compile **(for all OS versions)**:
Compile the code (move to the `Make/Linux` folder of the repo)
```
cd Make/Linux
make CONFIG=<Develop/Debug/Release>
```

### Install Python dependencies
Install the ssl library:
```
sudo apt-get install libssl-dev
```


<!--### Install NodeJS dependencies
* Install NodeJS
* Using npm install the `websocket` package (e.g. `npm install websocket`)




### Install MONA
* Clone the following repo `https://github.com/cs-au-dk/MONA`
* Follow the install instructions at [this page](https://github.com/cs-au-dk/MONA/blob/master/INSTALL)

HINT:
```
cd <MONA_DIR>
sudo make install-strips
```

NOTICE: you might need to install **flex**, **bison** and **byacc**: 
```
sudo apt-get install flex bison byacc
```
Then, go into the MONA directory and uninstall:
```
sudo make uninstall
sudo make clean
```
then rebuild:
```
sudo make install-strip
```
-->

### Download and install the dependencies for the `planning-for-past-temporal-goals` repo 
This repo is included in the current package and is provided by the [Whitemech research group](https://github.com/whitemech).
* Install the dependencies required for pygraphviz: 
```
sudo apt-get install python3-dev graphviz libgraphviz-dev pkg-config
```

* Install the other required dependencies for these repos:
```
sudo apt-get install gcc-multilib g++-multilib
```
* Install OpenJDK 8 (NOTICE: the next repo can not be built with Java versions above 8):
```
sudo apt-get install openjdk-8-jdk
```
* Install swig:
```
sudo apt-get install swig3.0
```
* Install latexmk:
```
sudo apt-get install latexmk
```
* Install PyAudio:
```
sudo apt-get install portaudio19-dev python3-pyaudio
```


### Install the required Python packages

Using a python package manager install the required packages using:
```
pip install -r requirements.tx
```
in the root dir of the repo. 

This command will install the required packages:
* numpy
* twisted
* ltlf2dfa
* graphviz
* pygraphviz
* shortuuid
* service_identity

<!--### Download and install the `planning-for-past-temporal-goals` repo from the [Whitemech research group](https://github.com/whitemech)
Follow the [install guide for this repo](https://github.com/whitemech/planning-for-past-temporal-goals/tree/benchmark)
 Clone the repo (anywhere):
```
git clone -b benchmark https://github.com/whitemech/planning-for-past-temporal-goals
```
* Install the dependencies required for pygraphviz: 
```
sudo apt-get install python3-dev graphviz libgraphviz-dev pkg-config
```
* Install the pip package:
```
cd planning-for-past-temporal-goals
pip install .
```
* Go to `third_party` directory and manually clone the required repositories:
```
cd third_party
git clone https://github.com/aibasel/downward; git clone https://github.com/robertmattmueller/myND; git clone https://github.com/whitemech/pddl; git clone https://github.com/QuMuLab/planner-for-relevant-policies
```

* Install the dependencies required for these repos:
```
sudo apt-get install gcc-multilib g++-multilib
```
* Move to the main repo directory and run the `build.sh` script
```
cd ..
./scripts/build.sh
```
-->



## Running

The network infrastructure is set to run on localhost. Make sure you're connected to a network anyway, even if not connected to the internet (required by SimRobot).

NOTICE: source the aliases.sh file with:
```
source aliases.sh
```
 for some useful one-word aliases (look for comments in the file)


### Running experiments

To run an experiment in the SimRobot simulator:
1) Run SimRobot
2) Run the Python server providing the plan

#### Run SimRobot
Move to `Develop/` folder
```
cd Build/Linux/SimRobot/<Develop/Debug/Release>/
```

Run SimRobot
```
./SimRobot
```

Click on File->Open and then move to the `Config/Scenes` folder and open the required scene, which is a `.ros2`. 

(NOTICE: you can specify the full path to the `.ros2` file to avoid navigating to it in SimRobot) 

##### Available scenes
* `fond_striker_no_obstacle.ros2` scene. This scene features one robot, in idle mode, waiting for plan commands. Used for the  `basic_striker`, `striker_with_battery_power_conditioning` and `striker_with_conditioning` experiments.
* `fond_striker_with_obstacle.ros2` scene. This scene features one robot, in idle mode, waiting for plan commands. Used for the multi-robot `striker_with_pass` experiment (opponent present, no jolly available case).
* `fond_striker_jolly_no_obstacle.ros2` scene. This scene features one striker and one jolly, in idle mode, waiting for plan commands. Used for the multi-robot `striker_with_pass` experiment (opponent not present, jolly available case).
* `fond_striker_jolly_with_obstacle.ros2` scene. This scene features one striker and one jolly, in idle mode, waiting for plan commands. Used for the multi-robot `striker_with_pass` experiment (opponent present, jolly available case).
* `GameFast1vs1.ros2` scene. This scene features two strikers of opposing teams, used in the qualitative experiments section of the paper to perform benchmarks (see below for the section about the adversarial example).


If the experiment is being run on a physical robot, contact <suriani@diag.uniroma1.it> for the necessary instructions.

#### Run the Python server with a specific experiment

  To run an experiment execute the following command on a terminal:

  ```
  python run_experiment.py <EXPERIMENT_SUBFOLDER.EXPERIMENT_NAME> --localhost
  ```

  where:
  * The module run_experiment.py is located in the repo root
  * `EXPERIMENT_SUBFOLDER` is a subfolder of the "experiments" folder
  * `EXPERIMENT_NAME` is the name of the file containing the necessary methods to run the experiment

In order to be able to insert conditioning commands (which in the current version have to be inserted when the experiment is run, right before the plan is created), the `--ask_constraints` command-line argument has to be used.

##### Runnable experiments
Experiments can be found in the directory `robocup_spl_temporal_goals/experiments/`. The available experiments are:
* `classical_planning.basic_striker`, providing the simple single-agent scenario
* `fond_planning_with_conditioning.striker_with_battery_power_conditioning`, providing the simple single-agent scenario with conditioning over battery power consumption using the `highBatteryConsumption` conditionable predicate, as well as the `isat` conditionable predicate
* `fond_planning_with_conditioning.striker_with_conditioning`, providing the simple single-agent scenario with a `isat` conditionable predicate
* `fond_planning.striker_with_pass`, providing the multi-agent scenario. (NOTICE: in this case, fluents depend on the selected SimRobot scene).

#### Running the adversarial experiment
We need to first run the `GameFast1vs1.ros2` scene in SimRobot, following the guide above.


Then, run the following commands in two different terminals:
First terminal:
```
python run_experiment.py adversarial_experiment.adversarial_striker_A --localhost
```
to run the first player.

Second terminal:
```
python run_experiment.py adversarial_experiment.adversarial_striker_B --localhost --opponent_team
```
to run the second player.

### Vocal interface
A vocal frontend is included, allowing a user to condition generated behaviors in the following way:
1) The frontend connects to the server
2) Then it waits for the user to pronounce a role (for example "striker") or for a keyword addressing the whole team (for example "everyone", "all", "team", "guys", "robots"). 
3) After the role/keyword has been pronounced and correctly recognized, the frontend will ask for a constraint, expressed using the sentence structures specified in the file `robocup_spl_temporal_goals/lib/constraints.py`, in the function `get_available_constraint_templates`. The provided constraints are:

* The **never SOMETHING** template, which requires the **SOMETHING** predicate to never be verified in the plan, representing the *!O(SOMETHING)* PLTLf formula. The structure is:
  
    {*never*|*avoid*|*not once*|*prevent*} SOMETHING
  
  where the keywords in the curly brackets are all synonims therefore interchangeable.

* The **SOMETHING at least once** template, which requires the **SOMETHING** predicate to be verified at least once in the plan, representing the *O(SOMETHING)* PLTLf formula. The structure is:
  
    {*at least once*|*once*|*sometimes*} SOMETHING
  
* The **SOMETHING at least once then SOMETHING_ELSE** template, which requires the **SOMETHING** predicate to be verified at least once in the plan, followed by the **SOMETHING_ELSE** predicate somewhere along the trace, representing the *O(SOMETHING) AND YO(SOMETHING_ELSE)* PLTLf formula. The structure is:
  
    {*if at least once*|*if once*|*if sometimes*} SOMETHING {*then*|*and then*|*finally*} SOMETHING_ELSE
  
* The **always SOMETHING** template, which requires the **SOMETHING** predicate to be verified at every step along the plan, representing the *H(SOMETHING)* PLTLf formula. The structure is:
  
    {*historically*|*always*} SOMETHING
  
* The **SOMETHING just before goal** template, which requires the **SOMETHING** predicate to be verified at the last step before the goal, representing the *Y(SOMETHING)* PLTLf formula. The structure is:
  
    {*just before goal*|*at last*} SOMETHING

Additional constraints can be added (see wiki).

The predicate keywords (*SOMETHING*, *SOMETHING_ELSE* from above), accept predicates featured in the current domain/experiment description file or their known synonims (see wiki for more about conditional predicates or their synonims).

4) A packet is sent over network to the server, which in turn generates a new plan entailing the known constraint. NOTICE that only one constraint at a time can be specified through this vocal interface.


#### Running the vocal interface

To run the vocal interface, move to the `robocup_spl_temporal_goals` subdirectory and execute the following command on a terminal:

  ```
  python run_vocal_interface.py --connect_to_IP <PYTHON_SERVER_IP_ADDRESS>
  ```

where the `PYTHON_SERVER_IP_ADDRESS` is the IP address of the communication manager. If no `--connect_to_IP` argument is specified, then localhost will be used as a target address.

### Running benchmarks

  To run a benchmark, move to the `robocup_spl_temporal_goals` subdirectory and execute the following command on a terminal:

  ```
  python run_benchmark.py <BENCHMARK_SUBFOLDER> --perform_benchmarks
  ```

  where:
  * `BENCHMARK_SUBFOLDER` is a subfolder of the "experiments" folder

  Use the `--ask_constraints` command-line argument in the scenarios with constrainable predicates to add constraints.

  Just run:

  ```
  python run_benchmark.py
  ```
  to have a full list of possible command-line arguments.

##### Runnable benchmarks
The only benchmark available is the one featured in the paper: `striker_with_waypoint_conditioning_with_increasing_size`, featuring a planning problems with a domain where the goal is to bring the ball to the goal and a grid of adjacent waypoints is modeled (in a case where adjacency is also diagonal and a case where it is not). This will automatically perform benchmarks for all scenarios described.

<!--
### How to add low-level behaviors
TODO
  Link to Skills&Cards introduction

### How to create a new experiment
TODO
  #### How to initialize the environment (small guide about registries)


-->