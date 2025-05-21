# educrys-control-basic

This repository provides basic control scripts in Python for the EduCrys setup. A new version with a modular, object-oriented Python code is being developed [here](https://github.com/poc-handsome/educrys-control-modular), but has not been finished yet.

## Setup

The EduCrys setup is shown in the photo below. See [educrys-hardware](https://github.com/poc-handsome/educrys-hardware) for a further description of the hardware.
Description of experiments is provided in [educrys-experiments](https://github.com/poc-handsome/educrys-experiments).

<img src="pics/setup.jpg" width="600">

## Control software

The software can be started by running the "RunExperiment.sh" script on the Raspberry Pi desktop. Alternatively, the following manual steps can be executed in a command-line Terminal:
- Start the python environment in the "Documents" folder: ```source venv_2/bin/activate```
- Make a new folder containing all files from [software](https://github.com/poc-handsome/educrys-control-basic/tree/main/software)
- Change to this folder in the terminal and run ```python3 democz_game.py```

A screenshot of the control software GUI is shown below.

<img src="pics/screenshot.jpg" width="800">

**Once the GUI is running, the components of EduCrys are being controlled by the GUI elements until the window is closed.**
The key elements of the GUI are:
- Image from Raspberry Pi camera, taken (and stored) when "Set/Photo" is pressed or at a specified sample rate during logging. The camera should be adjusted before running GUI, for example with the command ```libcamera-still -t 0```.
- Image from the infrared sensor, taken (and stored) when "Get IR" is pressed or at a specified sample rate during logging. Note that only surfaces with emissivity near 1.0 (e.g. the hot plate surface, but NOT the metallic crucible) show accurate temperatures.
- Plot of sensor values and process parameters, where individual curves can be disabled with checkboxes. Data in the plot is only updated if logging is active. Note that axes ranges are usually not updated automatically, so that pressing "Autoscale" or zooming out may be needed.
- "Start/Stop" button for data logging: camera and infrared images and all parameters in a text file
- Current values from temperature sensors
  - T_PT_1: Pt100 with 3 mm thickness used for crucible temperature
  - T_TC_1: Type K thermocouple with 1 mm thickness used for crucible or melt temperature
  - T_TC_2: another Type K thermocouple
  - T_air and H_air: air temperature and humidity in the small sensor box
- Energy: power consumption of the 230V heating plate (calculated from nominal power and active time). Counting can be set to zero with "Reset"
- 5V Current: current consumption on the Raspberry Pi 5V line used for motors (should not exceed a few 100 mA)
- Weight: weight cell attached to the seed holder. Use "Tare" after attaching the seed to obtain the weight of the growing crystal. Note the maximum weight of 1 kg.
- Control of the hot plate temperature with specified temperature (PID control of a chosen temperature sensor "PID Input") or power. The power in % = active time in a 10 sec on/off cycle. E.g. 5% = 0.5 sec on and 9.5 sec off.
- Set speed for the linear pulling (Lin. vel.), seed rotatation (Rot. vel.) and cooling fan (Fan. vel.). Note that the values are only applied once "Set" is pressed.
- The current vertical (Z) coordinate of the seed holder. The default range is 0...220 mm and the motor is automatically stopped when reaching these limits. The programm always starts with a default value Z=100! Use "Set" to set the real position.
- The checkboxes "Start/stop recipe" initiate reading the time recipes from "recipes.txt" if enabled. This is possibly only when data logging is already running! During an active recipe, the manual parameter input is disabled.

## Acknowledgements

[This project](https://poc-handsome.github.io/) has received funding from the European Research Council (ERC) under the 
European Union’s Horizon Europe framework programme for research and innovation (grant agreement No. 101122970).

<img src="https://raw.githubusercontent.com/poc-handsome/poc-handsome.github.io/master/EN_FundedbytheEU_RGB_POS.png">

