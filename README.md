# CBPi4 HERMS PID KettleLogic 

### This Kettle Logic can be used to run a HERMS System with CraftbeerPi4. 

## Mash temperature is controlled via PID logic
- The Kettle logic is intended to be used in a HERMS system with HLT. It is a PID logic. PID parameters can be for instance derived from the PID AutoTune plugin (https://github.com/avollkopf/cbpi4-PIDAutoTune)
- It runs on PID control until it reaches a specified temperature. Above that temperature it heates w/o PID logic until a specified boil temp is reached.
- Power to run boil can be specified in the plugin
- Kettle Agitator (Pump) is siwtched on in Automode
- Pump can be switched of in intervalls

![CBPi4 Settings](https://github.com/avollkopf/cbpi4-PIDHerms/blob/main/Settings.png?raw=true)

## Parameters:
- Configurable:
	- P: Proportional - Takes current value into account
	- I: Integral - Takes past values into account
	- D: Derivative - Takes future values into account
	- Max Output: Maximum Power (%) to be used for PID during Ramp up
	- Max Boil Output: Maximum Power during when Boil Temp is reached
	- Max Boil Temp: When Temp is reached,  power is set to Max Boil Output
	- Max PID Temp: PID is not used above this temp and max output is used
	- Rest Intervall: Intervall for pump activity in seconds
	- Rest Time: Pump Rest Time in seconds
	- SampleTime: 2 or 5 seconds. Determines PID recalcultation frequency
	- HLT Sensor: Sensor that measures your HLT temperature
	- DeltaTemp: Plugin allows control of HLT temp. This is the maximum overshoot the HLT is allowed above the mash target temp before heating is paused. It is a system-dependent ceiling - insulation, hose length/run, and HERMS coil surface area/efficiency all affect how much hotter the HLT must run to move heat into the mash. Tune it to your rig: lower keeps the HLT close to the mash temp (gentler and more accurate, slower ramp), higher ramps faster but risks denaturing enzymes in the coil. Set 0 to disable the cap and run a pure mash-temp PID.

## Installation:
- sudo pip3 install cbpi4-PIDHerms 
- (or from the GIT repo) -> sudo pip3 install https://github.com/avollkopf/cbpi4-PIDHerms/archive/main.zip

## Requirements:
- CBPi4 Version 4.0.0.45 or later (currently only available from my fork)

Changelog:

- 29.07.26: (0.0.5) DeltaTemp is now a proper ceiling: values <= 0 disable the cap and run a pure mash-temp PID instead of pausing all heating (previous behaviour with the default of 0 stalled the mash). Clarified the DeltaTemp description, narrowed a bare except and removed an unused import.

- 11.05.22: (0.0.4) Updated README (removed cbpi add)
- 10.05.22: (0.0.3) Removed cbpi dependency
- 21.11.21: (0.0.2) Plugin can now use actor power incl. PWM actors
- 23.09.21: (0.0.1) Initial commit
