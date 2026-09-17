# CBPi4 HERMS PID KettleLogic 

### This Kettle Logic can be used to run a HERMS System with CraftbeerPi4. 

## Mash temperature is controlled via a cascaded (two-loop) PID
- The Kettle logic is intended to be used in a HERMS system with an HLT. Two PID loops work together:
  - **Outer (mash) loop** reads the mash temperature and decides how much hotter than the mash target the HLT should run, within the `DeltaTemp` band.
  - **Inner (HLT) loop** modulates the heater power to hold the HLT at that setpoint.
- This bounds how far the HLT overshoots the mash target (protecting the enzymes in the coil) while eliminating the on/off heater "bang-bang" behaviour of the previous single-loop cap. PID parameters can be derived from the PID AutoTune plugin (https://github.com/avollkopf/cbpi4-PIDAutoTune).
- Above `Max PID Temp` it heats at max output (mashout ramp); above `Max Boil Temp` it uses the configurable boil power.
- Kettle Agitator (Pump) is switched on in Automode and can be rested in intervals.

![CBPi4 Settings](https://github.com/avollkopf/cbpi4-PIDHerms/blob/main/Settings.png?raw=true)

## Parameters:
- Configurable:
	- P / I / D: parameters of the **mash (outer)** PID that drives the HLT setpoint from the mash temperature error
	- HLT_P / HLT_I / HLT_D: parameters of the **HLT (inner)** PID that holds the HLT at the requested setpoint
	- Max Output: Maximum Power (%) to be used for PID during Ramp up
	- Max Boil Output: Maximum Power during when Boil Temp is reached
	- Max Boil Temp: When Temp is reached,  power is set to Max Boil Output
	- Max PID Temp: PID is not used above this temp and max output is used
	- Rest Intervall: Intervall for pump activity in seconds
	- Rest Time: Pump Rest Time in seconds
	- SampleTime: 2 or 5 seconds. Determines PID recalcultation frequency
	- HLT Sensor: Sensor that measures your HLT temperature
	- DeltaTemp: Maximum overshoot (in degrees) the HLT is allowed to run above the mash target. The mash loop scales the HLT setpoint within this band - full mash demand targets `mash_target + DeltaTemp`, and as the mash nears its target the HLT setpoint eases back toward `mash_target`. It is system-dependent (insulation, hose length/run, and HERMS coil surface area/efficiency all affect how much hotter the HLT must run to move heat into the mash), so tune it to your rig: lower keeps the HLT close to the mash temp (gentler and more accurate, slower ramp), higher ramps faster but risks denaturing enzymes in the coil. Set 0 to make the HLT track the mash target directly (gentlest, HLT approximately equals MT).

## Installation:
- sudo pip3 install cbpi4-PIDHerms 
- (or from the GIT repo) -> sudo pip3 install https://github.com/avollkopf/cbpi4-PIDHerms/archive/main.zip

## Requirements:
- CBPi4 Version 4.0.0.45 or later (currently only available from my fork)

Changelog:

- 29.07.26: (0.0.6) Reworked the HLT control into a cascaded (two-loop) PID. The mash (outer) PID now sets a clamped HLT setpoint within the DeltaTemp band and a new HLT (inner) PID modulates heater power to hold it. This removes the on/off heater chatter of the previous cap and bounds HLT overshoot. Added HLT_P/HLT_I/HLT_D parameters for the inner loop. DeltaTemp=0 now makes the HLT track the mash target directly.

- 29.07.26: (0.0.5) DeltaTemp is now a proper ceiling: values <= 0 disable the cap and run a pure mash-temp PID instead of pausing all heating (previous behaviour with the default of 0 stalled the mash). Clarified the DeltaTemp description, narrowed a bare except and removed an unused import.

- 11.05.22: (0.0.4) Updated README (removed cbpi add)
- 10.05.22: (0.0.3) Removed cbpi dependency
- 21.11.21: (0.0.2) Plugin can now use actor power incl. PWM actors
- 23.09.21: (0.0.1) Initial commit
