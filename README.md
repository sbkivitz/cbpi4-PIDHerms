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
- If the HLT sensor stops reading, the heater holds its last power for a few cycles and is then switched off, with a notification. This applies in **every** mode, including the mashout ramp and the boil, where the controller would otherwise command full power without consulting the HLT at all. Nothing else bounds HLT temperature: `Max Boil Temp`, `Max PID Temp` and `Max Pump Temp` are all measured on the **mash** sensor.
- Kettle Agitator (Pump) runs in Automode once you have started it, and can be rested in intervals. It never starts by itself: with CraftBeerPi 4.7.7.post94 or later, AUTO asks you to start it ("Start the wort pump to recirculate.", **Start pump**), once a step, and until you do nothing heats the mash through the coil. Your start stands through its rests and into the next rest step, until you stop it, STOP or ALL OFF, or CraftBeerPi stops it for safety (the dough-in, the sparge, the boil).

![CBPi4 Settings](https://github.com/avollkopf/cbpi4-PIDHerms/blob/main/Settings.png?raw=true)

## Parameters:
- Configurable:
	- P / I / D: parameters of the **mash (outer)** PID. These are in **degrees of HLT offset per degree of mash error**, not heater percent, so AutoTune values do not apply here. Defaults: P=2.0, I=0.005, D=0.0. If a value carried over from 0.0.6 or earlier is detected (P above 20), the plugin warns you in the UI and keeps running - the effect is the old fixed-offset behaviour, not a safety problem.
	- HLT_P / HLT_I / HLT_D: parameters of the **HLT (inner)** PID that holds the HLT at the requested setpoint. These drive heater power, so AutoTune values belong here.
	- Max Output: Maximum Power (%) to be used for PID during Ramp up. If it isn't a number, 100 is used, with a notification.
	- Max Boil Output: Maximum Power during when Boil Temp is reached
	- Max Boil Temp: When Temp is reached,  power is set to Max Boil Output
	- Max PID Temp: PID is not used above this temp and max output is used
	- Rest Intervall: Intervall for pump activity in seconds. It has to be more than 0; anything else is 600, with a notification.
	- Rest Time: Pump Rest Time in seconds. 0 is no rest; less than 0 is 60, with a notification.
	- SampleTime: 2 or 5 seconds. Determines PID recalcultation frequency. Anything from 1 to 60 seconds works; outside that, or if it isn't a number, 5 is used, with a notification.
	- HLT Sensor: Sensor that measures your HLT temperature
	- DeltaTemp: How far above the mash target the HLT is allowed to run, in degrees. The mash loop raises the HLT setpoint within this band when it wants heat and eases it back toward `mash_target` as the mash reaches setpoint, so at rest the HLT sits close to the mash temperature. How much offset your rig needs depends on insulation, hose length/run, and HERMS coil surface area and efficiency, so there is no universally correct value - tune it to your system. Lower tracks the mash more tightly and is gentler on enzymes but ramps slower; higher ramps faster but risks denaturing enzymes in the coil. A HERMS can only move heat while the HLT is hotter than the wort, so values of 0 or less are raised to a small floor. If the band is too narrow for your system the mash will hold below setpoint; the plugin detects this and notifies you to increase DeltaTemp rather than silently running cold. Default: 3, which is a reasonable starting point in Celsius. In Fahrenheit 3 degrees is tight (about 1.7 C) - most F rigs will want 5-8. Values below 1 C / 2 F are raised to that floor.
	- MashIn_Pump_Within: Mash In only, while the tun holds water and no grain. The pump stays off and the HLT heats alone until it is within this many degrees of the mash target; then you are asked to start the pump, which runs until the grain goes in. Hot water then circulates through the hoses for the end of the heat-up only. The element runs flat out either way, so the heat-up takes about as long. "Near the target" is judged on the HLT, because the mash probe at the coil outlet reads still water while nothing flows. Set it wider than the element's proportional band, about 100 / HLT_P degrees (7 F at HLT_P 15), or the element backs off while it waits. If the HLT stops warming for 10 minutes while its element is on - the selector on the boil kettle, a dead element or SSR - you are asked for the pump anyway, with a warning. Blank asks for the pump from the start.
- Left empty, a setting takes the value in the plugin's `BLANK` table, with the temperature limits and DeltaTemp in the server's unit: on a Fahrenheit rig Max_Pump_Temp and Max_PID_Temp are 190.4, Max_Boil_Temp 208.4 and DeltaTemp 5.4. The plugin tells CraftBeerPi these values (`blank_defaults`), so the Hardware page can show them and offer to fill them in. MashIn_DeltaTemp and MashIn_Pump_Within have none: empty, each is a way of working, not a number.

## Tuning, in order

Tune the inner loop first. The outer loop assumes the HLT can hold the setpoint it
is given; if that is not true, nothing above it will behave.

**1. Inner loop (`HLT_P` / `HLT_I` / `HLT_D`)** — these drive heater percent from HLT
error, which is exactly what the PID AutoTune plugin measures. Run AutoTune on the
HLT and put its numbers here. The defaults (117.0795 / 0.2747 / 41.58) are a
reasonable starting point for a typical electric HLT.

Check it before going further: set an HLT target, and confirm it settles without
oscillating or overshooting by more than a degree or so.

**2. `DeltaTemp`** — start at 3 (C). Run a mash and watch the resting HLT
temperature once the mash is holding target:

| What you see | What it means | Do |
|---|---|---|
| HLT settles a degree or two above the mash and holds | correct | nothing |
| Mash never reaches target; you get the "DeltaTemp may be too small" notification | the band cannot cover your heat loss | raise by 2 and repeat |
| HLT sits pinned at `mash + DeltaTemp` the whole rest | same as above | raise by 2 and repeat |
| Mash overshoots on arrival | band wider than needed | lower by 1 |

The offset your rig needs depends on insulation, hose run and coil area, so there is
no correct value to copy. What you are looking for is the smallest band that still
lets the mash hold setpoint.

**3. Outer loop (`P` / `I` / `D`)** — usually leave these alone. They are in
**degrees of HLT offset per degree of mash error**, so `P = 2.0` means a 1 C mash
error asks for 2 C of HLT offset, bounded by `DeltaTemp`.

| Symptom | Change |
|---|---|
| Mash approaches target very slowly despite HLT having headroom | raise `P` to 3-4 |
| Mash oscillates around target, HLT setpoint visibly hunting | lower `P` to 1.0-1.5 |
| Mash holds slightly below target indefinitely | raise `I` to 0.01 |
| Mash overshoots after a long ramp | lower `I`, or leave `D` at 0 |

Leave `D` at 0 unless you have a specific reason. The inner loop already damps the
response, and derivative on a cascaded outer loop mostly amplifies sensor noise.

> ⚠️ Do not put AutoTune values in `P` / `I` / `D`. AutoTune produces heater-percent
> gains; used here they saturate the outer loop at a few hundredths of a degree of
> error, which pins the HLT at the top of the band and turns the cascade back into
> fixed-offset control. The plugin warns if it sees a `P` above 20.

## Installation:
- sudo pip3 install cbpi4-PIDHerms 
- (or from the GIT repo) -> sudo pip3 install https://github.com/avollkopf/cbpi4-PIDHerms/archive/main.zip

## Requirements:
- CBPi4 Version 4.0.0.45 or later (currently only available from my fork)

Changelog:

- 25.11.26: (0.0.9.post12) The pump's loop waits a second after the brewer stops the pump before it looks again, so it can never go round without letting the rest of CraftBeerPi run.
- 25.11.26: (0.0.9.post11) The wort pump never starts by itself. When AUTO wants it and you haven't started it, CraftBeerPi asks, once a step: "Start the wort pump to recirculate.", with **Start pump**; your own press of the pump answers it too. Until you start it the HLT element isn't driven for the mash, so no heat goes through the coil without flow. Your start stands through the pump's rests and into the next step; stopped by your own press during a step, it isn't asked for again in that step (the step's second line says it's off). The dough-in hold, the sparge and the boil stop it, and the next step asks again. MashIn_Pump_Within now says when you are asked. With an older CraftBeerPi nothing changes.
- 25.11.26: (0.0.9.post10) Nothing is pumped into the mash tun, and nothing heats it through the coil, before the brewer has said its strike water is in it (CraftBeerPi 4.7.7.post76 on: his NEXT or answer out of BrewStart or a MashIn on the HLT, his START of a mash tun step, his own press of the wort pump or of this AUTO, or **Water's in**). Until then, with AUTO on, the wort pump stays off (switched off once if it was running), the HLT element is left to the HLT's own AUTO, and CraftBeerPi shows "Wort pump held" with **Water's in**; within a second of the word it circulates and heats as before. With an older CraftBeerPi nothing changes.
- 16.11.26: (0.0.9.post9) The plugin tells CraftBeerPi what it uses for each setting left empty (`blank_defaults`), for CraftBeerPi 4.7.7.post41 or later to show and fill in. The control loops read their empty values from the same table, so the two can't differ. No change in behaviour.
- 15.11.26: (0.0.9.post8) When the PIDs ask for no heat, the element is switched off on that decision. It used to keep its last duty for two more decisions, 10 seconds at SampleTime 5. A `Rest_Interval` of 0 or less could freeze CraftBeerPi, with `Pump_Rest` set to No; it is now 600, with a notification, and the pump's loop always lets the rest of CraftBeerPi run. A setting that is not a number, such as "nan" or "inf", is replaced by its default, with a notification, instead of reaching the PIDs or the limits. That includes `SampleTime` and `Max_Output`, which used to stop AUTO starting; a `SampleTime` outside 1 to 60 seconds is 5. If a PID's output is ever not a number, the element is switched off, with a notification, and both PIDs start again.
- 13.11.26: (0.0.9.post7) Comments describe the code. No change in behaviour.
- 27.10.26: (0.0.9.post6) New `MashIn_Pump_Within`: during Mash In the pump can wait while the HLT heats alone, and start once the HLT is within that many degrees of the mash target - so an unattended heat-up circulates hot water for its last part only. While it waits the plugin watches the HLT itself, because the step's heat-stall watch is rightly told that no heat can reach the mash: ten minutes without warming, element on, starts the pump with a warning. Blank changes nothing.
- 17.09.26: (0.0.8) Fixed two paths that could drive the element with no valid HLT reading. The HLT failure check now runs before every heating mode - previously it sat inside the PID branch only, so above Max PID Temp or Max Boil Temp the code commanded full or boil power without ever checking the HLT, and a dead probe meant the heater ran blind indefinitely. Nothing else bounds HLT temperature, since Max Boil Temp, Max PID Temp and Max Pump Temp are all measured on the mash sensor. Also, run() awaited the pump task before the temperature task, so a crash in temperature control was never surfaced while the pump kept running and the heater-off never executed; both tasks are now awaited together and either one stopping cancels the other.
- 15.09.26: (0.0.7) Fixed DeltaTemp having no usable effect at its old default. The outer PID now emits the HLT offset directly in degrees and is bounded by DeltaTemp, so the band limit and the PID's anti-windup boundary are the same value. DeltaTemp defaults to 3 C / 5 F and values <= 0 are floored, since a HERMS cannot transfer heat without a positive offset. The outer P/I/D are now in degrees-of-offset per degree-of-error (defaults 2.0 / 0.005 / 0.0) - AutoTune values apply to HLT_P/HLT_I/HLT_D only. The outer loop now runs slower than the inner loop as cascade control requires, a too-narrow DeltaTemp is detected and reported, and a lost HLT sensor now switches the heater off instead of heating unbounded.

- 29.07.26: (0.0.6) Reworked the HLT control into a cascaded (two-loop) PID. The mash (outer) PID now sets a clamped HLT setpoint within the DeltaTemp band and a new HLT (inner) PID modulates heater power to hold it. This removes the on/off heater chatter of the previous cap and bounds HLT overshoot. Added HLT_P/HLT_I/HLT_D parameters for the inner loop. (Note: this release also removed the DeltaTemp <= 0 handling added in 0.0.5, which left the default configuration unable to heat; fixed in 0.0.7.)

- 29.07.26: (0.0.5) DeltaTemp is now a proper ceiling: values <= 0 disable the cap and run a pure mash-temp PID instead of pausing all heating (previous behaviour with the default of 0 stalled the mash). Clarified the DeltaTemp description, narrowed a bare except and removed an unused import. (Superseded by 0.0.6/0.0.7, which handle DeltaTemp <= 0 by flooring it instead.)

- 11.05.22: (0.0.4) Updated README (removed cbpi add)
- 10.05.22: (0.0.3) Removed cbpi dependency
- 21.11.21: (0.0.2) Plugin can now use actor power incl. PWM actors
- 23.09.21: (0.0.1) Initial commit
