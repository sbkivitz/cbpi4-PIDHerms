# CBPi4 HERMS PID KettleLogic 

### This Kettle Logic can be used to run a HERMS System with CraftbeerPi4. 

## Mash temperature is controlled via a cascaded (two-loop) PID
- The Kettle logic is intended to be used in a HERMS system with an HLT. Two PID loops work together:
  - **Outer (mash) loop** reads the mash temperature and decides how much hotter than the mash target the HLT should run, within the `DeltaTemp` band.
  - **Inner (HLT) loop** modulates the heater power to hold the HLT at that setpoint.
- This bounds how far the HLT overshoots the mash target (protecting the enzymes in the coil) while eliminating the on/off heater "bang-bang" behaviour of the previous single-loop cap.
- Because the outer loop backs the HLT setpoint off as the mash approaches target, the HLT settles at the smallest offset your system actually needs rather than sitting at a fixed offset above the mash. Widening `DeltaTemp` raises the ceiling, not the resting HLT temperature.
- PID AutoTune (https://github.com/avollkopf/cbpi4-PIDAutoTune) measures mash response to **heater power**, so its output applies to the inner loop only: use it for `HLT_P` / `HLT_I` / `HLT_D`. The outer `P` / `I` / `D` are in different units (see below) and must not be taken from AutoTune.
- Above `Max PID Temp` it heats at max output (mashout ramp); above `Max Boil Temp` it uses the configurable boil power.
- If the HLT sensor stops reading, the heater holds its last power briefly and is then switched off - the inner loop is the only thing bounding HLT temperature, so it does not keep heating blind.
- Kettle Agitator (Pump) is switched on in Automode and can be rested in intervals.

![CBPi4 Settings](https://github.com/avollkopf/cbpi4-PIDHerms/blob/main/Settings.png?raw=true)

## Parameters:
- Configurable:
	- P / I / D: parameters of the **mash (outer)** PID. These are in **degrees of HLT offset per degree of mash error**, not heater percent, so AutoTune values do not apply here. Defaults: P=2.0, I=0.005, D=0.0. If a value carried over from 0.0.6 or earlier is detected (P above 20), the plugin warns you in the UI and keeps running - the effect is the old fixed-offset behaviour, not a safety problem.
	- HLT_P / HLT_I / HLT_D: parameters of the **HLT (inner)** PID that holds the HLT at the requested setpoint. These drive heater power, so AutoTune values belong here.
	- Max Output: Maximum Power (%) to be used for PID during Ramp up
	- Max Boil Output: Maximum Power during when Boil Temp is reached
	- Max Boil Temp: When Temp is reached,  power is set to Max Boil Output
	- Max PID Temp: PID is not used above this temp and max output is used
	- Rest Intervall: Intervall for pump activity in seconds
	- Rest Time: Pump Rest Time in seconds
	- SampleTime: 2 or 5 seconds. Determines PID recalcultation frequency
	- HLT Sensor: Sensor that measures your HLT temperature
	- DeltaTemp: How far above the mash target the HLT is allowed to run, in degrees. The mash loop raises the HLT setpoint within this band when it wants heat and eases it back toward `mash_target` as the mash reaches setpoint, so at rest the HLT sits close to the mash temperature. How much offset your rig needs depends on insulation, hose length/run, and HERMS coil surface area and efficiency, so there is no universally correct value - tune it to your system. Lower tracks the mash more tightly and is gentler on enzymes but ramps slower; higher ramps faster but risks denaturing enzymes in the coil. A HERMS can only move heat while the HLT is hotter than the wort, so values of 0 or less are raised to a small floor. If the band is too narrow for your system the mash will hold below setpoint; the plugin detects this and notifies you to increase DeltaTemp rather than silently running cold. Default: 3 C / 5 F.

## Installation:
- sudo pip3 install cbpi4-PIDHerms 
- (or from the GIT repo) -> sudo pip3 install https://github.com/avollkopf/cbpi4-PIDHerms/archive/main.zip

## Requirements:
- CBPi4 Version 4.0.0.45 or later (currently only available from my fork)

Changelog:

- 15.09.26: (0.0.7) Fixed DeltaTemp having no usable effect at its old default. The outer PID now emits the HLT offset directly in degrees and is bounded by DeltaTemp, so the band limit and the PID's anti-windup boundary are the same value. DeltaTemp defaults to 3 C / 5 F and values <= 0 are floored, since a HERMS cannot transfer heat without a positive offset. The outer P/I/D are now in degrees-of-offset per degree-of-error (defaults 2.0 / 0.005 / 0.0) - AutoTune values apply to HLT_P/HLT_I/HLT_D only. The outer loop now runs slower than the inner loop as cascade control requires, a too-narrow DeltaTemp is detected and reported, and a lost HLT sensor now switches the heater off instead of heating unbounded.

- 29.07.26: (0.0.6) Reworked the HLT control into a cascaded (two-loop) PID. The mash (outer) PID now sets a clamped HLT setpoint within the DeltaTemp band and a new HLT (inner) PID modulates heater power to hold it. This removes the on/off heater chatter of the previous cap and bounds HLT overshoot. Added HLT_P/HLT_I/HLT_D parameters for the inner loop. (Note: this release also removed the DeltaTemp <= 0 handling added in 0.0.5, which left the default configuration unable to heat; fixed in 0.0.7.)

- 29.07.26: (0.0.5) DeltaTemp is now a proper ceiling: values <= 0 disable the cap and run a pure mash-temp PID instead of pausing all heating (previous behaviour with the default of 0 stalled the mash). Clarified the DeltaTemp description, narrowed a bare except and removed an unused import. (Superseded by 0.0.6/0.0.7, which handle DeltaTemp <= 0 by flooring it instead.)

- 11.05.22: (0.0.4) Updated README (removed cbpi add)
- 10.05.22: (0.0.3) Removed cbpi dependency
- 21.11.21: (0.0.2) Plugin can now use actor power incl. PWM actors
- 23.09.21: (0.0.1) Initial commit
