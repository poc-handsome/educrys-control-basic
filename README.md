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

**NOTE: once the GUI is running, the components of EduCrys are being controlled by the GUI elements until the window is closed.**
Then heating and all motors are stopped as indicated in the command line output.

The key elements of the GUI are described in the sections below.

### Logging

The "Start/Stop" button on the top right activates data logging: 
- Camera and infrared images are stored in image files with a timestamp in the file name
- All sensor and process parameters are written to a text file (new file with a timestamp in the file name on each program)
- Values are added to the plot

### Images

Image from Raspberry Pi camera is taken (and stored) when "Set/Photo" is pressed or at a specified sample rate during logging. The camera should be adjusted before running GUI, for example with the command ```libcamera-still -t 0```.
  
Image from the infrared sensor is taken (and stored) when "Get IR" is pressed or at a specified sample rate during logging. Note that only surfaces with emissivity near 1.0 show accurate temperatures. For example, the hot plate surface is fine, but NOT the metallic crucible.

### Plot

The plot shows sensor values and process parameters, where individual curves can be disabled with checkboxes. Data in the plot is only updated if logging is active. Note that ranges of axes are usually not updated automatically to keep the GUI responsive. Therefore, pressing "Autoscale" or zooming out may be needed.

### Measurements

Temperature sensors:
- **T_PT_1**: Pt100 resistance sensor with 3 mm thickness used for crucible temperature
- **T_TC_1**: Type K thermocouple with 1 mm thickness used for crucible or melt temperature
- **T_TC_2**: another Type K thermocouple
- **T_air and H_air**: air temperature and humidity in the small sensor box

**Energy** denotes power consumption of the 230V heating plate, which is calculated from nominal power and active time. Energy usage counter can be set to zero with "Reset".

**5V Current** denotes current consumption on the Raspberry Pi 5V line, which is used for motors, LED and several sensors. This should usually not exceed a few 100 mA.

**Weight** comes from the weight cell attached to the seed holder. Use "Tare" after attaching the seed to obtain the weight of the growing crystal. Note the maximum weight of 1 kg.

### Heating control

The 1500W hotplate inside the setup is controlled by an Solid State Relay, which is periodically switched on/off. Two modes are available:
- In the **PID** mode the temperature of the chosen sensor **PID Input** is controlled to reach **Target T**
- In the **Manual** mode the power **Target P** is set to a given value in %. The percentage means the active time in a 10 sec on/off cycle. E.g. 5% = 0.5 sec on and 9.5 sec off. The following power limits are active in both (!) PID and Manual modes:
  - 2.5% or lower: hotplate remains off
  - 50% = maximum power. This power may still produce temperatures over 300 °C on the hotplate!

### Motor control

The following motor speeds can be set. Typical limits are specified.
- **Lin. vel.**: pulling wire (linear motion), -100...100 mm/min
- **Rot. vel.**: seed rotation, -12...12 rpm. Note that the motors needs a value of about 3 rpm to start moving.
- **Fan. vel.**: cooling fan, 0...80%. 
The values are only applied once "Set" is pressed.

**Coord. Z** denotes the vertical (Z) coordinate of the seed holder. The default range is 0...220 mm, and the motor is automatically stopped when reaching these limits. The programm always starts with a default value Z=100! Use "Set" to set the real position.

### Recipes

The checkboxes **Start/stop recipe** initiate reading the time recipes from "recipes.txt" if enabled. This is possibly only when data logging is already running! During an active recipe, the manual parameter input is disabled.

## Acknowledgements

[This project](https://poc-handsome.github.io/) has received funding from the European Research Council (ERC) under the 
European Union’s Horizon Europe framework programme for research and innovation (grant agreement No. 101122970).

<img src="https://raw.githubusercontent.com/poc-handsome/poc-handsome.github.io/master/EN_FundedbytheEU_RGB_POS.png">

