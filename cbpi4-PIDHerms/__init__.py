import asyncio
import logging
from cbpi.api import *
from cbpi.api.dataclasses import NotificationType
import time
import datetime

@parameters([Property.Sensor(label = "HLT_Sensor",
                             description="Sensor of HLT Kettle"),
             Property.Number(label="DeltaTemp", configurable=True, default_value=3,
                             description="Width of the HLT overshoot band, in the configured temperature unit. The mash (outer) PID raises the HLT setpoint up to mash_target + DeltaTemp when it wants heat, and eases it back toward mash_target as the mash reaches setpoint, so at rest the HLT sits close to the mash temperature. A HERMS only moves heat while the HLT is hotter than the wort, so this must be greater than 0; values of 0 or less are raised to a small floor. System-dependent (insulation, hose length/run, HERMS coil surface area and efficiency) - tune it to your rig. Lower tracks the mash more tightly and is gentler on enzymes but ramps slower; higher ramps faster but risks denaturing enzymes in the coil."),
             Property.Number(label="P", configurable=True, default_value=2.0,
                             description="P value of the mash (outer) PID, in degrees of HLT offset per degree of mash error. NOTE: these are NOT heater-percent gains and must not be taken from the PID AutoTune plugin - use HLT_P/HLT_I/HLT_D for that."),
             Property.Number(label="I", configurable=True, default_value=0.005,
                             description="I value of the mash (outer) PID (degrees of HLT offset per degree of mash error per second)"),
             Property.Number(label="D", configurable=True, default_value=0.0,
                             description="D value of the mash (outer) PID. 0 is a good starting point - the inner HLT loop already damps the response."),
             Property.Number(label="HLT_P", configurable=True, default_value=117.0795,
                             description="P value of the HLT (inner) PID - holds the HLT at the setpoint the mash loop asks for"),
             Property.Number(label="HLT_I", configurable=True, default_value=0.2747,
                             description="I value of the HLT (inner) PID"),
             Property.Number(label="HLT_D", configurable=True, default_value=41.58,
                             description="D value of the HLT (inner) PID"),
             Property.Select(label="SampleTime", options=[2,5], 
                             description="PID Sample time in seconds. Default: 5 (How often is the output calculation done)"),
             Property.Number(label="Max_Pump_Temp", configurable=True, default_value=88,
                             description="Max temp the pump can work in."),
             Property.Number(label = "Max_Output", configurable = True, default_value = 100, 
                             description="Max power for PID and Ramp up."),
             Property.Number(label="Max_Boil_Output", configurable=True, default_value=85,
                             description="Power when Max Boil Temperature is reached."),
             Property.Number(label="Max_Boil_Temp", configurable=True, default_value=98,
                             description="When Temperature reaches this, power will be reduced to Max Boil Output."),
             Property.Number(label="Max_PID_Temp", configurable=True,
                             description="When this temperature is reached, PID will be turned off"),
             Property.Number(label="Rest_Interval", configurable=True, default_value=600,
                             description="Rest the pump after this many seconds during the mash."),
             Property.Number(label="Rest_Time", configurable=True, default_value=60,
                             description="Rest the pump for this many seconds every rest interval.")])

class PID_HERMS(CBPiKettleLogic):

    # The inner (HLT) loop must settle faster than the outer (mash) loop or the two
    # hunt against each other. The outer PID runs this many times slower.
    OUTER_LOOP_RATIO = 5
    # Consecutive failed HLT reads tolerated before the heater is cut.
    MAX_HLT_READ_FAILURES = 5
    # Outer gains are degrees of HLT offset per degree of mash error, so sane values
    # are single digits. Anything this large is almost certainly a heater-percent gain
    # carried over from a pre-0.0.7 config or from AutoTune.
    LEGACY_GAIN_THRESHOLD = 20.0
    # How long the outer loop may sit pinned at the top of the DeltaTemp band, while
    # the mash is still short of target, before we tell the user the band is too narrow
    # for their rig. The required offset depends on insulation, hose run and coil
    # surface area, so it cannot be guessed - but it can be detected.
    SATURATION_WARN_SECONDS = 900

    def __init__(self, cbpi, id, props):
        super().__init__(cbpi, id, props)
        self._logger = logging.getLogger(type(self).__name__)
        self.sample_time, self.max_output, self.pid = None, None, None
        self.work_time, self.rest_time, self.max_output_boil = None, None, None
        self.max_boil_temp, self.max_pid_temp, self.max_pump_temp = None, None, None
        self.kettle, self.heater, self.agitator = None, None, None

    def _float_prop(self, name, default):
        """Read a numeric property, falling back to default when it is missing, blank
        or unparsable, so a cleared field in the UI cannot kill the control loop."""
        value = self.props.get(name, None)
        if value is None or str(value).strip() == "":
            return float(default)
        try:
            return float(value)
        except (TypeError, ValueError):
            self._logger.warning(
                "PIDHerms: could not parse %s=%r, using %s", name, value, default
            )
            return float(default)

    async def on_stop(self):
        await self.actor_off(self.agitator)
    
    # subroutine that controlls pump aue and ump stop if max pump temp is reached
    async def pump_control(self):
        #get pump based on agitator id
        self.pump = self.cbpi.actor.find_by_id(self.agitator)

        while self.running:
            # get current pump status
            if self.pump.instance:
                pump_on = self.pump.instance.state
            else:
                pump_on = False
            # if the current temp is below the max pump temp, check if pause time is reached to pause pump
            if self.get_sensor_value(self.kettle.sensor).get("value") < self.max_pump_temp:
                self._logger.debug("starting pump")
                #switch the pump on
                await self.actor_on(self.agitator)
                # calculate time, when pump should do the next pause
                off_time = time.time() + self.work_time
                # run pump until next pause time is reached
                while time.time() < off_time:
                    await asyncio.sleep(1)
                    # stop cycle, if current temp is higher than max pump temp
                    if self.get_sensor_value(self.kettle.sensor).get("value") >= self.max_pump_temp:
                        break
                # pause pump when active pump Interval is completed
                self._logger.debug("resting pump")
                await self.actor_off(self.agitator)
                await asyncio.sleep(self.rest_time)
            # If temeprature is above max pump temp, and pump is on, switch it off
            # Staops also the pump if user switches it on and temp is abouve max pump temp
            else:
                if pump_on:
                    self._logger.debug("pump max temp reached, pump turned off")
                    await self.actor_off(self.agitator)
                await asyncio.sleep(1)

    # subroutine that controls temperature via a cascaded PID:
    #   outer (mash) PID -> clamped HLT setpoint -> inner (HLT) PID -> heater power
    async def temp_control(self):
        await self.actor_on(self.heater,0)
        heat_percent_old = 0
        hlt_read_failures = 0
        hlt_fault_notified = False
        saturated_seconds = 0
        band_warning_sent = False

        while self.running:
            try:
                hlt_value = self.get_sensor_value(self.sensor).get("value")
                self.HLT_Temp = float(hlt_value)
            except (TypeError, ValueError, AttributeError):
                self.HLT_Temp = None

            # current mash temperature and its target
            current_temp = self.get_sensor_value(self.kettle.sensor).get("value")
            target_temp = self.get_kettle_target_temp(self.id)

            # Nothing else in this plugin bounds HLT temperature - Max_Boil_Temp,
            # Max_PID_Temp and Max_Pump_Temp are all evaluated against the mash
            # sensor. So a missing HLT reading has to be handled before any
            # heating decision, not only inside the PID branch: during a mashout
            # or boil ramp the branches below command full power unconditionally,
            # and would otherwise drive the element blind indefinitely.
            if self.HLT_Temp is None:
                hlt_read_failures += 1
                if hlt_read_failures <= self.MAX_HLT_READ_FAILURES:
                    # Ride out a momentary 1-wire glitch on the last known power.
                    heat_percent = heat_percent_old
                else:
                    heat_percent = 0
                    await self.actor_off(self.heater)
                    heat_percent_old = 0
                    if not hlt_fault_notified:
                        hlt_fault_notified = True
                        self.cbpi.notify(
                            "PIDHerms",
                            "No HLT sensor reading - heater turned off",
                            NotificationType.ERROR,
                        )
                await asyncio.sleep(self.sample_time)
                continue

            hlt_read_failures = 0
            hlt_fault_notified = False

            # if current temperature is higher than the defined boil temp, use a fixed
            # heating percent instead of PID values for controlled boiling
            if current_temp >= self.max_boil_temp:
                heat_percent = self.max_output_boil
            # above max pid temp (mashout ramp) use max output until boil temp is reached
            elif current_temp >= self.max_pid_temp:
                heat_percent = self.max_output
            # mash/PID band: cascade control
            else:
                # Outer loop: the mash PID's output IS the HLT setpoint offset, in
                # degrees, and the PID itself bounds it to [0, DeltaTemp]. Because the
                # band limit and the PID's anti-windup boundary are the same value, the
                # integral cannot wind up against the cap. Full mash demand targets
                # mash_target + DeltaTemp; as the mash nears its target the offset eases
                # back toward mash_target, so at rest the HLT sits close to the mash.
                hlt_setpoint = target_temp + self.pid.calc(current_temp, target_temp)

                # If the outer loop has been asking for the full band for a long time
                # and the mash is still short, the band itself is the limit: the rig
                # needs a bigger HLT-to-mash offset than DeltaTemp allows.
                if hlt_setpoint >= target_temp + self.delta - 1e-9 and current_temp < target_temp - 0.5:
                    saturated_seconds += self.sample_time
                    if saturated_seconds >= self.SATURATION_WARN_SECONDS and not band_warning_sent:
                        band_warning_sent = True
                        self.cbpi.notify(
                            "PIDHerms",
                            "DeltaTemp ({}) may be too small for this system - the HLT "
                            "has been held at its limit and the mash is still below "
                            "target. Increase DeltaTemp if the mash cannot hold "
                            "setpoint.".format(self.delta),
                            NotificationType.WARNING,
                        )
                else:
                    saturated_seconds = 0
                    band_warning_sent = False

                # Inner loop: modulate heater power to hold the HLT at the setpoint.
                # HLT_Temp is known valid here - the blind case returned above.
                heat_percent = self.hlt_pid.calc(self.HLT_Temp, hlt_setpoint)

            # only push a new power value to the actor when it changes
            if heat_percent != heat_percent_old:
                await self.actor_set_power(self.heater,heat_percent)
                heat_percent_old = heat_percent
            await asyncio.sleep(self.sample_time)


    async def run(self):
        self._logger = logging.getLogger(type(self).__name__)
        try:
            self.TEMP_UNIT = self.get_config_value("TEMP_UNIT", "C")
            boilthreshold = 98 if self.TEMP_UNIT == "C" else 208
            maxpidtemp = 88 if self.TEMP_UNIT == "C" else 190
            maxpumptemp = 88 if self.TEMP_UNIT == "C" else 190
            default_delta = 3 if self.TEMP_UNIT == "C" else 5
            # A HERMS only moves heat while the HLT is hotter than the wort. A zero or
            # negative band leaves the outer loop no authority at all, so the mash would
            # never reach target. Hold a small floor rather than stalling. This floor
            # only guarantees the loop can act; the offset a given rig actually needs
            # depends on its insulation, hose run and coil, so a band that is too narrow
            # is reported at runtime (see SATURATION_WARN_SECONDS) instead of guessed.
            min_delta = 1.0 if self.TEMP_UNIT == "C" else 2.0

            self.sample_time = int(self.props.get("SampleTime", 5))
            self.max_output = int(self.props.get("Max_Output", 100))

            self.delta = self._float_prop("DeltaTemp", default_delta)
            if self.delta < min_delta:
                logging.warning(
                    "PIDHerms: DeltaTemp %s is below the usable minimum for a HERMS, "
                    "using %s %s instead", self.delta, min_delta, self.TEMP_UNIT
                )
                self.delta = min_delta

            # Outer loop: mash error -> HLT setpoint offset, in degrees. Bounding the
            # PID's own output by DeltaTemp makes the band limit and the PID's
            # anti-windup boundary the same thing, so the integral cannot wind up
            # against the cap.
            p = self._float_prop("P", 2.0)
            i = self._float_prop("I", 0.005)
            d = self._float_prop("D", 0.0)
            # Guard against a config written for <=0.0.6, where P/I/D were heater-percent
            # gains. Such a value saturates the outer loop almost instantly, which pins
            # the HLT at the top of the band and silently reverts to fixed-offset
            # behaviour. Not unsafe, but not what the cascade is for - so say so plainly.
            if p > self.LEGACY_GAIN_THRESHOLD:
                message = (
                    "P={} looks like an old heater-percent gain. Since 0.0.7 the mash "
                    "P/I/D are degrees of HLT offset per degree of mash error - try "
                    "P=2.0, I=0.005, D=0.0 and put AutoTune values in HLT_P/HLT_I/HLT_D. "
                    "Running as configured, but the HLT will sit at its DeltaTemp limit."
                ).format(p)
                logging.warning("PIDHerms: %s", message)
                self.cbpi.notify("PIDHerms", message, NotificationType.WARNING)
            # calc() returns its previous output until its own sample time has elapsed,
            # so a longer sample time here is all that is needed to slow the outer loop.
            self.pid = PIDArduino(
                self.sample_time * self.OUTER_LOOP_RATIO, p, i, d, 0, self.delta
            )

            # Inner loop: HLT error -> heater percent. These are the AutoTune-style gains.
            hp = self._float_prop("HLT_P", 117.0795)
            hi = self._float_prop("HLT_I", 0.2747)
            hd = self._float_prop("HLT_D", 41.58)
            self.hlt_pid = PIDArduino(self.sample_time, hp, hi, hd, 0, self.max_output)

            self.work_time = float(self.props.get("Rest_Interval", 600))
            self.rest_time = float(self.props.get("Rest_Time", 60))
            self.max_output_boil = float(self.props.get("Max_Boil_Output", 85))

            self.max_boil_temp = float(self.props.get("Max_Boil_Temp", boilthreshold))
            self.max_pid_temp = float(self.props.get("Max_PID_Temp", maxpidtemp))
            self.max_pump_temp = float(self.props.get("Max_Pump_Temp", maxpumptemp))

            self.kettle = self.get_kettle(self.id)
            self.heater = self.kettle.heater
            self.agitator = self.kettle.agitator
            self.sensor = self.props.get("HLT_Sensor", None)

            logging.info(
                "PIDHerms outer P:{} I:{} D:{} band:{} / inner P:{} I:{} D:{} {} {}".format(
                    p, i, d, self.delta, hp, hi, hd, self.kettle, self.heater
                )
            )

            pump_controller = asyncio.create_task(self.pump_control())
            temp_controller = asyncio.create_task(self.temp_control())

            # Await both together. Awaiting the pump first meant a crash in
            # temp_control() was never surfaced while the pump kept running, so
            # run()'s heater-off finally was never reached and the element stayed
            # on. Cancel the sibling as soon as either one stops.
            done, pending = await asyncio.wait(
                {pump_controller, temp_controller},
                return_when=asyncio.FIRST_COMPLETED,
            )
            for task in pending:
                task.cancel()
            if pending:
                await asyncio.gather(*pending, return_exceptions=True)
            # Re-raise whatever stopped first so it is logged rather than lost.
            for task in done:
                task.result()

        except asyncio.CancelledError as e:
            pass
        except Exception as e:
            logging.error("PIDHerms Error {}".format(e))
        finally:
            self.running = False
            await self.actor_off(self.heater)

# Based on Arduino PID Library
# See https://github.com/br3ttb/Arduino-PID-Library
class PIDArduino(object):

    def __init__(self, sampleTimeSec, kp, ki, kd, outputMin=float('-inf'),
                 outputMax=float('inf'), getTimeMs=None):
        if kp is None:
            raise ValueError('kp must be specified')
        if ki is None:
            raise ValueError('ki must be specified')
        if kd is None:
            raise ValueError('kd must be specified')
        if float(sampleTimeSec) <= float(0):
            raise ValueError('sampleTimeSec must be greater than 0')
        if outputMin >= outputMax:
            raise ValueError('outputMin must be less than outputMax')

        self._logger = logging.getLogger(type(self).__name__)
        self._Kp = kp
        self._Ki = ki * sampleTimeSec
        self._Kd = kd / sampleTimeSec
        self._sampleTime = sampleTimeSec * 1000
        self._outputMin = outputMin
        self._outputMax = outputMax
        self._iTerm = 0
        self._lastInput = 0
        self._lastOutput = 0
        self._lastCalc = 0

        if getTimeMs is None:
            self._getTimeMs = self._currentTimeMs
        else:
            self._getTimeMs = getTimeMs

    def calc(self, inputValue, setpoint):
        now = self._getTimeMs()

        if (now - self._lastCalc) < self._sampleTime:
            return self._lastOutput

        # Compute all the working error variables
        error = setpoint - inputValue
        dInput = inputValue - self._lastInput

        # In order to prevent windup, only integrate if the process is not saturated
        if self._lastOutput < self._outputMax and self._lastOutput > self._outputMin:
            self._iTerm += self._Ki * error
            self._iTerm = min(self._iTerm, self._outputMax)
            self._iTerm = max(self._iTerm, self._outputMin)

        p = self._Kp * error
        i = self._iTerm
        d = -(self._Kd * dInput)

        # Compute PID Output
        self._lastOutput = p + i + d
        self._lastOutput = min(self._lastOutput, self._outputMax)
        self._lastOutput = max(self._lastOutput, self._outputMin)

        # Log some debug info
        self._logger.debug('P: {0}'.format(p))
        self._logger.debug('I: {0}'.format(i))
        self._logger.debug('D: {0}'.format(d))
        self._logger.debug('output: {0}'.format(self._lastOutput))

        # Remember some variables for next time
        self._lastInput = inputValue
        self._lastCalc = now
        return self._lastOutput

    def _currentTimeMs(self):
        return time.time() * 1000

def setup(cbpi):

    '''
    This method is called by the server during startup 
    Here you need to register your plugins at the server
    
    :param cbpi: the cbpi core 
    :return: 
    '''

    cbpi.plugin.register("PID_HERMS", PID_HERMS)
