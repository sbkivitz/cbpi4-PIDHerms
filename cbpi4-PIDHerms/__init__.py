import asyncio
import logging
from cbpi.api import *
from cbpi.api import clock
from cbpi.api.dataclasses import NotificationType
import math
import time
import datetime

@parameters([Property.Sensor(label = "HLT_Sensor",
                             description="Sensor of HLT Kettle"),
             Property.Number(label="DeltaTemp", configurable=True, default_value=3,
                             description="Width of the HLT overshoot band, in the configured temperature unit. The mash (outer) PID raises the HLT setpoint up to mash_target + DeltaTemp when it wants heat, and eases it back toward mash_target as the mash reaches setpoint, so at rest the HLT sits close to the mash temperature. A HERMS only moves heat while the HLT is hotter than the wort, so this must be greater than 0; values of 0 or less are raised to a small floor. System-dependent (insulation, hose length/run, HERMS coil surface area and efficiency) - tune it to your rig. Lower tracks the mash more tightly and is gentler on enzymes but ramps slower; higher ramps faster but risks denaturing enzymes in the coil."),
             Property.Number(label="MashIn_DeltaTemp", configurable=True,
                             description="Width of the HLT overshoot band while the mash tun holds only water, i.e. during Mash In before the grain goes in. DeltaTemp is a compromise between ramp rate and not cooking enzymes in the coil - but there are no enzymes in the tun yet, so that compromise is being paid for nothing. Measured on a 30 L / 6.5 kW HERMS heating tap water to a 163 F strike: at DeltaTemp 5.4 F the ramp takes 64 min and 13.6 of those are spent on the final 5 F; at 12 F it is 54 min with 5.2 on the final 5 F. Returns flatten past about 18 F because the element saturates. Leave blank to use DeltaTemp and change nothing. The wider band applies only from the start of Mash In until strike temperature is reached, which is exactly the window in which the tun is known to be grain-free."),
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
                             description="Max temp the pump can work in. Interpreted in the configured temperature unit; the default is 88 C / 190.4 F."),
             Property.Number(label = "Max_Output", configurable = True, default_value = 100, 
                             description="Max power for PID and Ramp up."),
             Property.Number(label="Max_Boil_Output", configurable=True, default_value=85,
                             description="Power when Max Boil Temperature is reached."),
             Property.Number(label="Max_Boil_Temp", configurable=True, default_value=98,
                             description="When Temperature reaches this, power will be reduced to Max Boil Output. Interpreted in the configured temperature unit; the default is 98 C / 208.4 F."),
             Property.Number(label="Max_PID_Temp", configurable=True,
                             description="When this temperature is reached, PID will be turned off"),
             Property.Number(label="Sensor_Stale_Seconds", configurable=True, default_value=300,
                             description="Seconds a mash or HLT sensor may repeat the exact same numeric value before heating is suspended. Default: 300 seconds."),
             Property.Number(label="Rest_Interval", configurable=True, default_value=600,
                             description="Rest the pump after this many seconds during the mash."),
             Property.Number(label="Rest_Time", configurable=True, default_value=60,
                             description="Rest the pump for this many seconds every rest interval."),
             Property.Select(label="Pump_Rest", options=["Yes", "No"],
                             description="Rest the pump periodically. No runs it continuously - the coil then never sits full of static wort."),
             Property.Kettle(label="HLT_Kettle",
                             description="Optional. The HLT, so its setpoint shows what this cascade is commanding. Found from HLT_Sensor when left blank.")])

class PID_HERMS(CBPiKettleLogic):

    # The inner (HLT) loop must settle faster than the outer (mash) loop or the two
    # hunt against each other. The outer PID runs this many times slower.
    OUTER_LOOP_RATIO = 5
    # Consecutive failed HLT reads tolerated before the heater is cut.
    MAX_HLT_READ_FAILURES = 5
    # Fallback if an older core does not tell us a sensor-specific max_age.
    MAX_SENSOR_AGE = 30
    # Exact repeated values are weaker evidence than sensor age because a mash
    # at rest can legitimately sit on one quantised reading for a while. Five
    # minutes is still far longer than the 15s/30s source-age conventions, but
    # bounds a frozen mash probe before an active HERMS can drift badly.
    DEFAULT_SENSOR_STALE_SECONDS = 300
    # DS18B20 12-bit resolution is 0.0625 C, or 0.1125 F. Treat adjacent
    # quantised values as unchanged so a probe flapping one count cannot reset
    # the stale timer forever.
    SENSOR_REPEAT_EPSILON_C = 0.07
    SENSOR_REPEAT_EPSILON_F = 0.12
    # Consecutive zero-demand samples before the heater actor is switched off
    # rather than merely held at 0%. Purely to stop a PID sitting at zero from
    # chattering a contactor - at 0% duty no heat is produced either way.
    OFF_DWELL_SAMPLES = 3
    # Outer gains are degrees of HLT offset per degree of mash error, so sane values
    # are single digits. Anything this large is almost certainly a heater-percent gain
    # carried over from a pre-0.0.7 config or from AutoTune.
    LEGACY_GAIN_THRESHOLD = 20.0
    # How long the outer loop may sit pinned at the top of the DeltaTemp band, while
    # the mash is still short of target, before we tell the user the band is too narrow
    # for their rig. The required offset depends on insulation, hose run and coil
    # surface area, so it cannot be guessed - but it can be detected.
    SATURATION_WARN_SECONDS = 900

    # Integral state carried across restarts of the same kettle's logic, keyed by
    # kettle id, as {kettle_id: (brewing_time, outer_iTerm, inner_iTerm)}.
    #
    # A step transition tears the logic down and builds it again: MashInStep's
    # on_stop() switches AutoMode off, which cancels this task, and the next
    # step's on_start() switches it back on, which constructs a fresh one. Two
    # consecutive rests on the same kettle at the same target therefore restart
    # the controller for no reason.
    #
    # The integral is what holds the steady-state HLT-over-mash offset - roughly
    # two degrees on a typical rig, purely to cover losses. A new instance starts
    # at zero and has to rebuild it with I=0.005, which takes many minutes, and
    # the mash sags for all of them. On a four-rest profile that is four dips,
    # each at a rest boundary, each costing exactly the enzyme activity the rest
    # exists to produce.
    #
    # Carried rather than persisted: this is live control state, not
    # configuration, and it must not survive a restart of the server where the
    # kettle may have gone cold.
    _carried_integral = {}

    # How long carried integral state stays usable, in brewing seconds. Beyond
    # this the rig has probably been left alone and the thermal situation has
    # changed, so starting clean is safer than resuming a stale bias.
    INTEGRAL_CARRY_MAX_AGE = 300

    # How much the mash must rise for the ramp to count as still climbing.
    # Small and unit-agnostic on purpose: the question is whether the ramp is
    # moving at all, not how fast.
    #
    # Together with SATURATION_WARN_SECONDS this sets the slowest rise still
    # counted as progress - 0.05 degrees per 900s, about 0.2 degrees an hour.
    # Anything slower is a stall in every sense that matters to a brewer.
    CLIMB_MIN_RISE = 0.05

    # How much hotter the HLT must be than the mash before heat is considered to
    # be flowing. Small: the question is the direction of the gradient, not its
    # size. A HERMS with the HLT half a degree above the mash is delivering, just
    # slowly.
    HEAT_FLOW_MARGIN = 0.5

    #: Reported to anything that wants to know whether heat can reach the mash
    #: right now - the step's heat-stall watch in particular. Starts True so a
    #: logic that has not completed a cycle yet is not assumed broken.
    heat_available = True

    def _clamp_hlt_setpoint(self, hlt_setpoint, target_temp):
        """Cap the HLT demand at the mash target when nothing is flowing.

        pump_control() and temp_control() are independent tasks that never talk
        to each other, so during the pump's rest the outer loop carried on
        asking for mash_target + DeltaTemp. The wort trapped in the coil has no
        flow and a lot of surface area against hot liquor, so it heads for HLT
        temperature - and a slug of overheated wort is pushed into the mash when
        the pump restarts. On a sixty minute rest with the default one-minute
        rest in every ten, that happens five times.

        Clamping to the mash target while nothing is moving means the trapped
        wort tends towards exactly the temperature it is supposed to be. It
        costs a little ramp rate, which is the right trade: there is no benefit
        to superheating static wort, and a brewer who wants a faster ramp should
        shorten the rest rather than cook the coil.

        This is a method rather than two inline lines so that it can be called
        directly. The test for it used to recompute `min(setpoint, target)` in
        its own body and assert on that arithmetic, which passes whatever
        production does - including deleting the clamp entirely.
        """
        if not self._pump_is_running():
            return min(hlt_setpoint, target_temp)
        return hlt_setpoint

    def _pump_is_running(self):
        """Is wort actually moving through the coil right now?

        A HERMS coil transfers heat only while the mash is being recirculated.
        With no agitator configured the rig is assumed to circulate some other
        way, which keeps a differently-plumbed system behaving as it did.
        """
        # getattr, not direct access: this can be reached before run() has
        # assigned the agitator, and a temperature controller must not fall over
        # because it asked about a pump too early.
        agitator = getattr(self, "agitator", None)
        if not agitator:
            return True
        try:
            pump = self.cbpi.actor.find_by_id(agitator)
            return bool(pump is not None and pump.instance is not None
                        and pump.instance.state)
        except Exception:  # noqa: BLE001
            return True

    def _find_hlt_kettle(self):
        """The kettle whose setpoint should show what this cascade is asking for.

        While a HERMS mash is running, the HLT is not being controlled to its
        own configured target - it is being driven to whatever offset the outer
        loop currently wants. The interface still showed the configured number,
        which during a mash is stale and means nothing: a brewer looking at the
        HLT sees 168.8 while the cascade is actually asking for 157.

        Named explicitly by HLT_Kettle when set, otherwise found by matching the
        HLT sensor - the same sensor the inner loop already reads, so a rig
        wired sensibly needs nothing configured.

        The field was briefly withdrawn because it rendered as an unlabelled
        "Kettle" picker inside the Mash Tun's own settings, which reads like
        nonsense. That was a UI bug rather than a naming one: PropsEdit.js
        passed label= for text, number and select fields but not for kettle,
        sensor, actor or fermenter, so all four fell back to their component's
        default. It is why the HLT sensor field said only "Sensor" too.
        """
        kettle_id = self.props.get("HLT_Kettle", None)
        try:
            if kettle_id:
                # Never this kettle, however it was chosen.
                #
                # The sensor fallback below has always refused to select the
                # controlled kettle; the explicit picker did not, and it offers
                # every kettle including this one. Choosing the mash tun is an
                # ordinary configuration mistake with an extraordinary
                # consequence: _publish_hlt_setpoint writes the commanded HLT
                # target onto the selected kettle, so the mash setpoint becomes
                # the outer loop's own output and the next iteration reads it
                # back as the rest temperature.
                #
                # Measured progression: 65 -> 68 -> 71 -> 74 -> 77 over four
                # decisions, climbing for as long as it runs. A mash does not
                # recover from that.
                if kettle_id == self.id:
                    self._logger.error(
                        "PIDHerms: HLT_Kettle is this kettle. Ignoring it - "
                        "the mash setpoint would become this loop's own "
                        "output and climb without limit. Select the HLT, or "
                        "leave it empty and set HLT_Sensor."
                    )
                else:
                    return self.cbpi.kettle.find_by_id(kettle_id)
            sensor_id = self.props.get("HLT_Sensor", None)
            if not sensor_id:
                return None
            for item in self.cbpi.kettle.data:
                # Never this kettle: the mash tun's own setpoint is the rest
                # temperature and must keep saying so.
                if item.id != self.id and item.sensor == sensor_id:
                    return item
        except Exception as e:
            self._logger.warning("PIDHerms: could not find the HLT kettle: %s", e)
        return None

    async def _publish_hlt_setpoint(self, value):
        """Show the commanded HLT setpoint in the interface.

        Pushed over the websocket rather than saved. This is a transient
        commanded value, not the brewer's configuration: writing it to disk
        every sample would wear the card for nothing and would quietly overwrite
        the number they actually set. on_stop puts the original back.
        """
        if getattr(self, "_hlt_kettle", None) is None:
            return
        # Never write this kettle's own setpoint.
        #
        # _hlt_kettle() refuses to resolve to the controlled kettle, so this
        # should be unreachable - and it is guarded anyway because the failure
        # is a mash setpoint that climbs without limit, driven by this loop's
        # own output. That is worth two lines.
        if getattr(self._hlt_kettle, "id", None) == self.id:
            return
        rounded = round(float(value), 1)
        if self._hlt_kettle.target_temp == rounded:
            return
        self._hlt_kettle.target_temp = rounded
        try:
            await self.cbpi.kettle.push_udpate()
        except Exception as e:
            self._logger.warning("PIDHerms: could not push the HLT setpoint: %s", e)

    def __init__(self, cbpi, id, props):
        super().__init__(cbpi, id, props)
        self._logger = logging.getLogger(type(self).__name__)
        self.sample_time, self.max_output, self.pid = None, None, None
        self.work_time, self.rest_time, self.max_output_boil = None, None, None
        self.max_boil_temp, self.max_pid_temp, self.max_pump_temp = None, None, None
        self.kettle, self.heater, self.agitator = None, None, None
        self._hlt_kettle = None
        self._hlt_target_before = None
        self._sensor_watch = {}
        self.sensor_stale_seconds = self.DEFAULT_SENSOR_STALE_SECONDS
        self.pump_rest_enabled = True

    def _mash_is_climbing(self, current_temp):
        """Has the mash risen measurably since the last confirmed rise?

        Deliberately coarse. The question is whether the ramp is making headway
        at all, not how fast - a rig pinned at the top of its band and still
        climbing is working exactly as intended, however slowly.

        The anchor only moves on a confirmed rise or on a new low, so a ramp
        that gains less than CLIMB_MIN_RISE per sample still registers as
        climbing once enough samples have accumulated, rather than being judged
        sample-to-sample and dismissed.
        """
        threshold = self.CLIMB_MIN_RISE
        anchor = getattr(self, "_climb_anchor", None)
        if anchor is None or current_temp >= anchor + threshold:
            self._climb_anchor = current_temp
            return True
        if current_temp < anchor:
            # Going backwards is not climbing, and the new low is the anchor to
            # judge any recovery against.
            self._climb_anchor = current_temp
            return False
        return False

    def _restore_integral(self):
        """Pick the integral back up if this kettle was being controlled moments ago.

        See _carried_integral. Silently starts clean if there is nothing recent to
        resume, which is the safe direction: a missing bias costs a slow approach,
        a wrong one costs an overshoot.
        """
        carried = self._carried_integral.get(self.id)
        if not carried:
            return
        when, outer_i, inner_i = carried
        age = clock.now() - when
        if (
            age < 0 or age > self.INTEGRAL_CARRY_MAX_AGE
            or not math.isfinite(float(outer_i))
            or not math.isfinite(float(inner_i))
        ):
            self._carried_integral.pop(self.id, None)
            return
        self.pid._iTerm = outer_i
        self.hlt_pid._iTerm = inner_i
        self._logger.info(
            "PIDHerms: resuming integral state for kettle %s after %.0fs "
            "(outer %.3f, inner %.3f)", self.id, age, outer_i, inner_i,
        )

    def _remember_integral(self):
        """Hand the integral to whatever instance controls this kettle next."""
        try:
            if (
                not math.isfinite(float(self.pid._iTerm))
                or not math.isfinite(float(self.hlt_pid._iTerm))
            ):
                self._carried_integral.pop(self.id, None)
                return
            self._carried_integral[self.id] = (
                clock.now(), self.pid._iTerm, self.hlt_pid._iTerm,
            )
        except Exception as e:
            self._logger.debug("Could not carry integral state: %s", e)

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

    def _temp_default(self, celsius):
        """Return a temperature default in the configured unit."""
        celsius = float(celsius)
        if getattr(self, "TEMP_UNIT", "C") == "C":
            return celsius
        return celsius * 9.0 / 5.0 + 32.0

    def _temperature_prop(self, name, celsius_default):
        """Read a temperature property whose historical metadata default was Celsius.

        CraftBeerPi stores plugin defaults in props, so a Fahrenheit install can
        arrive here with Max_Boil_Temp='98' even though 98 was meant as Celsius.
        Treat the exact historical Celsius default as unset on non-Celsius
        systems; any other configured value is the brewer's value in their
        configured unit.
        """
        default = self._temp_default(celsius_default)
        value = self.props.get(name, None)
        if value is None or str(value).strip() == "":
            return default
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            self._logger.warning(
                "PIDHerms: could not parse %s=%r, using %.1f %s",
                name, value, default, getattr(self, "TEMP_UNIT", ""),
            )
            return default
        if getattr(self, "TEMP_UNIT", "C") != "C" and abs(parsed - float(celsius_default)) < 1e-9:
            self._logger.warning(
                "PIDHerms: %s=%s is the old Celsius default on a %s system; "
                "using %.1f %s instead",
                name, value, self.TEMP_UNIT, default, self.TEMP_UNIT,
            )
            return default
        return parsed

    def _delta_prop(self):
        """Read DeltaTemp, converting the historical Celsius default on F rigs.

        DeltaTemp is a temperature difference, so the conversion is a scale
        factor only: 3 C of HERMS band is 5.4 F, not 37.4 F.
        """
        default = 3.0 if getattr(self, "TEMP_UNIT", "C") == "C" else 5.4
        value = self.props.get("DeltaTemp", None)
        if value is None or str(value).strip() == "":
            return default
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            self._logger.warning(
                "PIDHerms: could not parse DeltaTemp=%r, using %.1f %s",
                value, default, getattr(self, "TEMP_UNIT", ""),
            )
            return default
        if getattr(self, "TEMP_UNIT", "C") != "C" and abs(parsed - 3.0) < 1e-9:
            self._logger.warning(
                "PIDHerms: DeltaTemp=3 is the old Celsius default on a %s system; "
                "using %.1f %s instead",
                self.TEMP_UNIT, default, self.TEMP_UNIT,
            )
            return default
        return parsed

    def _read_trusted_temp(self, sensor_id, label):
        """Return (temperature, reason) where reason explains why it is unsafe.

        A numeric value alone is not enough: some sensor drivers keep publishing
        their last value after a failed read. Core exposes `age` for drivers that
        can say when they last changed, and the repeated-value watch here covers
        drivers that keep resetting that age around a frozen number.
        """
        if not sensor_id:
            return None, "{} sensor is not configured".format(label)

        try:
            state = self.get_sensor_value(sensor_id)
            value = float(state.get("value"))
        except (AttributeError, TypeError, ValueError):
            return None, "No {} sensor reading".format(label)
        if not math.isfinite(value):
            return None, "{} sensor reading is not finite".format(label)

        age = state.get("age")
        if age is not None:
            try:
                age = float(age)
                if not math.isfinite(age):
                    age = None
            except (TypeError, ValueError):
                age = None
        max_age = state.get("max_age")
        if max_age is not None:
            try:
                max_age = float(max_age)
                if not math.isfinite(max_age):
                    max_age = None
            except (TypeError, ValueError):
                max_age = None
        if not max_age or max_age <= 0:
            max_age = self.MAX_SENSOR_AGE
        if age is not None and age > max_age:
            return None, "{} sensor last updated {:.0f}s ago (limit {:.0f}s)".format(
                label, age, max_age
            )
        if age is None:
            return value, None

        try:
            if not isinstance(getattr(self, "_sensor_watch", None), dict):
                self._sensor_watch = {}
            stale_seconds = float(getattr(
                self, "sensor_stale_seconds", self.DEFAULT_SENSOR_STALE_SECONDS
            ))
            if stale_seconds <= 0:
                stale_seconds = self.DEFAULT_SENSOR_STALE_SECONDS

            timestamp = state.get("timestamp")
            try:
                now = float(timestamp)
            except (TypeError, ValueError):
                # SensorController timestamps and ages in wall-clock seconds.
                # Use the same clock for this freshness watch on hardware; tests
                # and simulators that want accelerated time can provide their own
                # timestamp in the sensor state they feed to this consumer.
                now = time.time()
            key = sensor_id
            watched = self._sensor_watch.get(key)
            epsilon = (
                self.SENSOR_REPEAT_EPSILON_C
                if getattr(self, "TEMP_UNIT", "C") == "C"
                else self.SENSOR_REPEAT_EPSILON_F
            )
            watched_value = None if watched is None else float(watched["value"])
            if (
                watched is None
                or not math.isfinite(watched_value)
                or abs(watched_value - value) > epsilon
            ):
                self._sensor_watch[key] = {"value": value, "changed_at": now}
                return value, None

            unchanged_for = now - watched["changed_at"]
            if unchanged_for > stale_seconds:
                return None, (
                    "{} sensor has been unchanged at {:.2f} for {:.0f}s "
                    "(limit {:.0f}s)"
                ).format(label, value, unchanged_for, stale_seconds)
        except Exception as e:  # noqa: BLE001
            getattr(self, "_logger", logging.getLogger(type(self).__name__)).error(
                "PIDHerms: cannot validate %s sensor freshness: %s", label, e
            )
            return None, "{} sensor freshness could not be validated".format(label)
        return value, None

    async def on_stop(self):
        await self.actor_off(self.agitator)
        # Give the HLT its own setpoint back. While the cascade was running its
        # displayed target was whatever the outer loop was commanding; leaving
        # that behind would silently replace the number the brewer configured.
        if getattr(self, "_hlt_kettle", None) is not None and \
                getattr(self, "_hlt_target_before", None) is not None:
            self._hlt_kettle.target_temp = self._hlt_target_before
            try:
                await self.cbpi.kettle.push_udpate()
            except Exception as e:
                self._logger.warning(
                    "PIDHerms: could not restore the HLT setpoint: %s", e
                )
        self._hlt_kettle = None
        self._hlt_target_before = None
    
    # subroutine that controlls pump aue and ump stop if max pump temp is reached
    async def _pump_wait(self, seconds, sensor_check=False):
        """Wait, but stay responsive to a hold or a stop.

        The pump's work interval was a loop that only ever tested the
        temperature, and its rest was a single clock.sleep of the whole rest
        time. Neither looked at pump_hold or at self.running, so a hold raised
        one second into a 600 second interval left the pump recirculating for
        the remaining 599.

        That defeats the entire point of the hold. Mash-in raises it the moment
        strike is reached so the grain goes into a still tun and so the ramp
        does not overshoot - measured on this plant at +3.0 F with the pump
        running against -0.1 F with it stopped. Ten minutes late is the same as
        never.

        Returns the reason it stopped waiting, so the caller can tell a
        completed interval from an interruption.
        """
        deadline = clock.now() + max(0.0, float(seconds))
        while clock.now() < deadline:
            if not self.running:
                return "stopped"
            if getattr(self, "pump_hold", False):
                return "hold"
            if sensor_check:
                try:
                    reading = self.get_sensor_value(self.kettle.sensor).get("value")
                    if reading is not None and reading >= self.max_pump_temp:
                        return "max_temp"
                except Exception:  # noqa: BLE001 - a missing reading is not fatal here
                    pass
            await clock.sleep(1)
        return "elapsed"

    async def pump_control(self):
        #get pump based on agitator id
        self.pump = self.cbpi.actor.find_by_id(self.agitator)

        while self.running:
            # A step can ask for no flow at all.
            #
            # Mash-in does, the moment strike temperature is reached: grain
            # should go into a still tun, not be pulled into a coil that has no
            # grain bed in front of it yet. It is also what makes a flat-out
            # mash-in ramp safe. The coil is the only path from the HLT to the
            # mash, so stopping the pump cuts that path immediately - measured
            # on this plant, a full-authority ramp overshoots strike by 3.0 F
            # with the pump running and by -0.1 F with it stopped. An element
            # cannot do that, because an element coasts.
            if getattr(self, "pump_hold", False):
                try:
                    if (self.pump is not None and self.pump.instance
                            and self.pump.instance.state):
                        await self.actor_off(self.agitator)
                except Exception as e:  # noqa: BLE001
                    self._logger.error("pump hold could not stop the pump: %s", e)
                await clock.sleep(1)
                continue
            # get current pump status
            if self.pump.instance:
                pump_on = self.pump.instance.state
            else:
                pump_on = False
            # if the current temp is below the max pump temp, check if pause time is reached to pause pump
            if self.get_sensor_value(self.kettle.sensor).get("value") < self.max_pump_temp:
                self._logger.debug("starting pump")
                # Checked immediately before starting, not only at the top of
                # the outer loop: a hold raised while the previous interval was
                # running would otherwise start the pump one more time before
                # anything noticed.
                if getattr(self, "pump_hold", False) or not self.running:
                    continue
                #switch the pump on
                await self.actor_on(self.agitator)
                # Run the interval, but stay responsive to a hold or a stop.
                # Brewing seconds, like every other duration here - the clock
                # decides how fast those pass.
                reason = await self._pump_wait(self.work_time, sensor_check=True)
                if reason in ("hold", "stopped"):
                    # The hold branch at the top of the loop does the switching
                    # off, so it sees a consistent state either way.
                    continue

                # Rests are optional. The pumps on plenty of rigs are rated for
                # continuous duty, and the rest is not free: while it is resting
                # the coil holds static wort against hot liquor, which is why
                # the HLT setpoint is clamped during a rest. A brewer who does
                # not need the rest should not pay for it.
                #
                # Upstream documents these two fields only by restating their
                # names, with no stated rationale, so "off" is a perfectly
                # reasonable thing to want.
                if not self.pump_rest_enabled:
                    continue

                # pause pump when active pump Interval is completed
                self._logger.debug("resting pump")
                await self.actor_off(self.agitator)
                # Interruptible for the same reason as the interval above: this
                # was a single sleep of the whole rest time, so a stop during a
                # rest waited it out before anything else could happen.
                await self._pump_wait(self.rest_time)
            # If temeprature is above max pump temp, and pump is on, switch it off
            # Staops also the pump if user switches it on and temp is abouve max pump temp
            else:
                if pump_on:
                    self._logger.debug("pump max temp reached, pump turned off")
                    await self.actor_off(self.agitator)
                await clock.sleep(1)

    # subroutine that controls temperature via a cascaded PID:
    #   outer (mash) PID -> clamped HLT setpoint -> inner (HLT) PID -> heater power
    async def temp_control(self):
        # Deliberately not actor_on(heater, 0) here. A plain GPIOActor's on()
        # drives the pin HIGH immediately and ignores the power argument, so that
        # call energized the element before any demand existed.
        #
        # Switching it off instead establishes the same known starting state
        # without the pulse, and matters because actor state survives a restart:
        # without this, an element left on by a previous run would stay on while
        # this loop believed it was off, and nothing here would ever switch it
        # off. The element is switched on below, once the loop asks for heat.
        await self.actor_off(self.heater)
        heater_is_on = False
        zero_demand_samples = 0
        heat_percent_old = 0
        sensor_fault_notified = None
        saturated_seconds = 0
        band_warning_sent = False

        while self.running:
            self.HLT_Temp, hlt_fault = self._read_trusted_temp(self.sensor, "HLT")

            # current mash temperature and its target
            current_temp, mash_fault = self._read_trusted_temp(
                self.kettle.sensor, "Mash"
            )
            try:
                target_temp = float(self.get_kettle_target_temp(self.id))
                if not math.isfinite(target_temp):
                    target_temp = None
            except (TypeError, ValueError):
                target_temp = None

            # Tell anything watching whether heat can actually reach the mash.
            #
            # There is no element in the mash tun - it is heated through a coil
            # in the HLT - so while the HLT is at or below the mash temperature
            # no heat flows, however hard the element is driven. That is the
            # normal state for several minutes after every step change, when the
            # target jumps and the HLT is still where the last rest left it: the
            # mash drifts down, flattens, and only then begins to climb.
            #
            # Without this, the step's heat-stall watch reads that plateau as a
            # dead element and warns at exactly the moment the rig is working
            # hardest. Observed on a running rig: warned 21.5 F short of target,
            # reached target half an hour of brewing time later having climbed
            # the whole way.
            try:
                # Two conditions, not one. A gradient is necessary but not
                # sufficient: the coil only moves heat while wort is actually
                # being pumped through it, so during the pump's rest interval -
                # a minute in every ten on a typical configuration - the mash is
                # thermally isolated no matter how hot the HLT is.
                #
                # Without the pump term the stall watch counts those rests as
                # time the mash should have been rising, which is the same
                # mistake in miniature as the one that made it warn during a
                # step change.
                pumping = self._pump_is_running()
                self.heat_available = (
                    pumping
                    and self.HLT_Temp is not None
                    and current_temp is not None
                    and self.HLT_Temp > float(current_temp) + self.HEAT_FLOW_MARGIN
                )
            except (TypeError, ValueError):
                self.heat_available = True

            sensor_fault = hlt_fault or mash_fault
            sensor_fault_key = "hlt" if hlt_fault else "mash" if mash_fault else None
            if target_temp is None:
                sensor_fault = sensor_fault or "Mash target temperature is missing"
                sensor_fault_key = sensor_fault_key or "target"

            # Nothing else in this plugin can make a safe heating decision
            # without both temperatures. Do not ride out bad samples on the last
            # known power: a repeated last value is exactly the dangerous fault.
            if sensor_fault:
                self.heat_available = False
                await self.actor_off(self.heater)
                heater_is_on = False
                zero_demand_samples = 0
                heat_percent_old = 0
                if target_temp is not None:
                    await self._publish_hlt_setpoint(target_temp)
                elif getattr(self, "_hlt_target_before", None) is not None:
                    await self._publish_hlt_setpoint(self._hlt_target_before)
                if sensor_fault_notified != sensor_fault_key:
                    sensor_fault_notified = sensor_fault_key
                    message = "{} - heater turned off".format(sensor_fault)
                    getattr(self, "_logger", logging.getLogger(type(self).__name__)).error(
                        "PIDHerms: %s", message
                    )
                    self.cbpi.notify(
                        "PIDHerms",
                        message,
                        NotificationType.ERROR,
                    )
                await clock.sleep(self.sample_time)
                continue

            if sensor_fault_notified is not None:
                self.cbpi.notify(
                    "PIDHerms",
                    "Temperature readings restored - heating resumed",
                    NotificationType.INFO,
                )
            sensor_fault_notified = None

            # if current temperature is higher than the defined boil temp, use a fixed
            # heating percent instead of PID values for controlled boiling
            if current_temp >= self.max_boil_temp:
                heat_percent = self.max_output_boil
            # above max pid temp (mashout ramp) use max output until boil temp is reached
            elif current_temp >= self.max_pid_temp:
                heat_percent = self.max_output
            # mash/PID band: cascade control
            else:
                # The band is only a compromise while there is something in the
                # tun worth protecting. During mash-in the tun holds water, so
                # the outer loop is allowed a wider HLT-over-mash offset and the
                # final approach stops crawling. MashInStep clears grain_present
                # at its start and sets it the moment strike temperature is
                # reached, which is exactly the grain-free window.
                #
                # Every attribute here is read with a default. run() sets them,
                # but not every caller goes through run() - the test suites
                # construct this object directly, and so can a plugin. A band
                # optimisation must never be the reason the mash control loop
                # dies.
                active_band = getattr(self, "_active_band", self.delta)
                wanted_band = (
                    self.delta if getattr(self, "grain_present", True)
                    else getattr(self, "mashin_delta", self.delta)
                )
                if wanted_band != active_band:
                    try:
                        self.pid.set_output_max(wanted_band)
                        self._active_band = active_band = wanted_band
                        logging.info(
                            "PIDHerms: HLT band now %s %s (%s)",
                            wanted_band, self.TEMP_UNIT,
                            "grain in" if getattr(self, "grain_present", True)
                            else "water only",
                        )
                    except Exception as e:  # noqa: BLE001
                        logging.error("PIDHerms: could not change band: %s", e)

                # Outer loop: the mash PID's output IS the HLT setpoint offset, in
                # degrees, and the PID itself bounds it to [0, DeltaTemp]. Because the
                # band limit and the PID's anti-windup boundary are the same value, the
                # integral cannot wind up against the cap. Full mash demand targets
                # mash_target + DeltaTemp; as the mash nears its target the offset eases
                # back toward mash_target, so at rest the HLT sits close to the mash.
                hlt_setpoint = self._clamp_hlt_setpoint(
                    target_temp + self.pid.calc(current_temp, target_temp),
                    target_temp,
                )

                # Show it. While the cascade is running the HLT is not being
                # held at its own configured target, it is being driven to this
                # - so displaying the configured number is worse than useless,
                # it is a plausible-looking lie.
                await self._publish_hlt_setpoint(hlt_setpoint)

                # If the outer loop has been asking for the full band for a long
                # time, the mash is still short, AND the mash has stopped rising,
                # the band itself is the limit: the rig needs a bigger
                # HLT-to-mash offset than DeltaTemp allows.
                #
                # The "stopped rising" part is what makes this a diagnosis rather
                # than a nuisance. Any ramp from cold sits pinned at the top of
                # the band for as long as the ramp takes - that is the band doing
                # its job, not failing. Warning on saturation alone fired on
                # every heat-up, and a warning that cries wolf gets ignored when
                # it is finally right.
                saturated = (
                    hlt_setpoint >= target_temp + active_band - 1e-9
                    and current_temp < target_temp - 0.5
                )
                if saturated and self._mash_is_climbing(current_temp):
                    # Pinned at the band limit, but the mash is still rising, so
                    # the band is still delivering heat and there is nothing to
                    # warn about. Reset rather than pause: an earlier version
                    # only paused the clock here, which still accumulated on the
                    # samples between confirmed rises and so warned partway
                    # through every ordinary ramp - the exact false positive this
                    # check exists to prevent.
                    saturated_seconds = 0
                elif saturated:
                    saturated_seconds += self.sample_time
                    if saturated_seconds >= self.SATURATION_WARN_SECONDS and not band_warning_sent:
                        band_warning_sent = True
                        self.cbpi.notify(
                            "PIDHerms",
                            "DeltaTemp ({}) may be too small for this system - the HLT "
                            "has been held at its limit and the mash has stopped short "
                            "of target. Increase DeltaTemp if the mash cannot hold "
                            "setpoint.".format(self.delta),
                            NotificationType.WARNING,
                        )
                else:
                    saturated_seconds = 0
                    band_warning_sent = False

                # Inner loop: modulate heater power to hold the HLT at the setpoint.
                # HLT_Temp is known valid here - the blind case returned above.
                heat_percent = self.hlt_pid.calc(self.HLT_Temp, hlt_setpoint)

            # Drive the actor from what it IS doing, not from what this loop
            # last told it to do.
            #
            # heater_is_on and heat_percent_old were locals and the loop
            # commanded only on a believed transition, so anything else
            # touching the actor left it wrong for the rest of the brew: switch
            # the element off from the dashboard and the loop went on believing
            # it was driving it and never commanded again.
            #
            # ActorController.set_power() only forwards a number to the
            # instance; it never changes state. For a plain GPIOActor that
            # means a demand of 0% left the actor nominally on at 0% duty
            # rather than genuinely off, which is why the off below exists at
            # all. The dwell keeps it from chattering around zero demand.
            actual_on, actual_power, known = self._heater_now()
            if not known:
                actual_on = heater_is_on
                actual_power = heat_percent_old

            if heat_percent > 0:
                zero_demand_samples = 0
                if not actual_on:
                    await self.actor_on(self.heater, heat_percent)
                elif actual_power != heat_percent:
                    await self.actor_set_power(self.heater, heat_percent)
                heater_is_on = True
                heat_percent_old = heat_percent
            else:
                zero_demand_samples += 1
                if zero_demand_samples >= self.OFF_DWELL_SAMPLES:
                    # Unconditional once the dwell has passed: this was guarded
                    # on the local flag, so a swallowed failure flipped it and
                    # the loop never retried.
                    await self.actor_off(self.heater)
                    heater_is_on = False
                    heat_percent_old = 0
            await clock.sleep(self.sample_time)


    def _heater_now(self):
        """What the heater actor is actually doing: (on, power, known).

        Read rather than remembered. See the call site in temp_control: belief
        about hardware goes stale the moment anything else touches the actor,
        and on a rig the brewer is something else that touches it.

        The driver's state is preferred over the container's, because
        find_by_id returns a container whose `instance` is the driver and
        commands update the instance - reading the container can report off
        while the element is on.

        known=False means "no idea" and must not be read as off, because off
        means command it on. Never raises.
        """
        try:
            registry = getattr(self.cbpi, "actor", None)
            if registry is None:
                return False, None, False
            actor = registry.find_by_id(self.heater)
            if actor is None:
                return False, None, True
            instance = getattr(actor, "instance", None)
            if instance is not None:
                state = getattr(instance, "state", None)
                power = getattr(instance, "power", getattr(actor, "power", None))
            else:
                state = getattr(actor, "state", None)
                power = getattr(actor, "power", None)
            if state is None:
                return False, None, False
            return bool(state), power, True
        except Exception:  # noqa: BLE001
            return False, None, False

    async def run(self):
        self._logger = logging.getLogger(type(self).__name__)
        try:
            self.TEMP_UNIT = self.get_config_value("TEMP_UNIT", "C")
            # A HERMS only moves heat while the HLT is hotter than the wort. A zero or
            # negative band leaves the outer loop no authority at all, so the mash would
            # never reach target. Hold a small floor rather than stalling. This floor
            # only guarantees the loop can act; the offset a given rig actually needs
            # depends on its insulation, hose run and coil, so a band that is too narrow
            # is reported at runtime (see SATURATION_WARN_SECONDS) instead of guessed.
            min_delta = 1.0 if self.TEMP_UNIT == "C" else 2.0

            self.sample_time = int(self.props.get("SampleTime", 5))
            self.max_output = int(self.props.get("Max_Output", 100))

            # A simulated rig runs its thermal model faster than real time. If the
            # control loop kept deciding once every sample_time REAL seconds while
            # the plant advanced sample_time * scale SIMULATED seconds between
            # decisions, the closed loop under test would not be the one that runs
            # on hardware - it would be the same rig sampled far too slowly, and it
            # limit cycles accordingly.
            #
            # Timing goes through cbpi.api.clock rather than the wall clock, so this
            # loop states its period in brewing seconds and has no idea whether it
            # is being simulated. On hardware the clock is real time and this is
            # exactly what it always was.
            #
            # Scaling each sleep here instead was the first attempt, and it put a
            # simulator's config key into a hardware control path - a setting left
            # at 60 would have run a real heater's integral sixty times fast.
            pid_clock = lambda: clock.now() * 1000.0

            self.delta = self._delta_prop()
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
                self.sample_time * self.OUTER_LOOP_RATIO, p, i, d, 0, self.delta,
                getTimeMs=pid_clock
            )

            # The wider band used while the tun is known to hold only water.
            # Blank means "same as DeltaTemp", so nothing changes for anyone who
            # does not set it. Clamped to at least DeltaTemp: a *narrower* band
            # for mash-in would be a configuration mistake, not an intention.
            mashin = self.props.get("MashIn_DeltaTemp", None)
            self.mashin_delta = self.delta
            if mashin not in (None, ""):
                try:
                    self.mashin_delta = max(self.delta, float(mashin))
                except (TypeError, ValueError):
                    logging.warning(
                        "PIDHerms: could not parse MashIn_DeltaTemp=%r, using "
                        "DeltaTemp=%s", mashin, self.delta
                    )
            # True unless a step positively tells us otherwise, so every rest,
            # ramp and mashout behaves exactly as before.
            if not hasattr(self, "grain_present"):
                self.grain_present = True
            if not hasattr(self, "pump_hold"):
                self.pump_hold = False
            self._active_band = self.delta
            if self.mashin_delta != self.delta:
                logging.info(
                    "PIDHerms: mash-in band %s %s, rest band %s %s",
                    self.mashin_delta, self.TEMP_UNIT, self.delta, self.TEMP_UNIT
                )

            # Inner loop: HLT error -> heater percent. These are the AutoTune-style gains.
            hp = self._float_prop("HLT_P", 117.0795)
            hi = self._float_prop("HLT_I", 0.2747)
            hd = self._float_prop("HLT_D", 41.58)
            self.hlt_pid = PIDArduino(self.sample_time, hp, hi, hd, 0, self.max_output,
                                      getTimeMs=pid_clock)
            self._restore_integral()

            self.work_time = float(self.props.get("Rest_Interval", 600))
            self.rest_time = float(self.props.get("Rest_Time", 60))
            # Defaults to resting, so an existing configuration behaves exactly
            # as it did. A rest time of zero also disables it, but only after
            # switching the pump off and straight back on again every interval,
            # which is a needless contactor cycle - this skips the rest
            # entirely.
            self.pump_rest_enabled = self.props.get("Pump_Rest", "Yes") != "No"
            self.max_output_boil = float(self.props.get("Max_Boil_Output", 85))

            self.sensor_stale_seconds = self._float_prop(
                "Sensor_Stale_Seconds", self.DEFAULT_SENSOR_STALE_SECONDS
            )
            if self.sensor_stale_seconds <= 0:
                logging.warning(
                    "PIDHerms: Sensor_Stale_Seconds %s is not positive, using %s",
                    self.sensor_stale_seconds, self.DEFAULT_SENSOR_STALE_SECONDS
                )
                self.sensor_stale_seconds = self.DEFAULT_SENSOR_STALE_SECONDS

            self.max_boil_temp = self._temperature_prop("Max_Boil_Temp", 98)
            self.max_pid_temp = self._temperature_prop("Max_PID_Temp", 88)
            self.max_pump_temp = self._temperature_prop("Max_Pump_Temp", 88)

            self.kettle = self.get_kettle(self.id)
            self.heater = self.kettle.heater
            self.agitator = self.kettle.agitator
            self.sensor = self.props.get("HLT_Sensor", None)

            # Remember the HLT's own setpoint before the cascade starts
            # overwriting the displayed one, so on_stop can put it back.
            self._hlt_kettle = self._find_hlt_kettle()
            if self._hlt_kettle is not None:
                self._hlt_target_before = self._hlt_kettle.target_temp
                logging.info(
                    "PIDHerms: publishing the commanded setpoint to '%s' "
                    "(was %s)", self._hlt_kettle.name, self._hlt_target_before
                )

            logging.info(
                "PIDHerms outer P:{} I:{} D:{} band:{} / inner P:{} I:{} D:{} {} {}".format(
                    p, i, d, self.delta, hp, hi, hd, self.kettle, self.heater
                )
            )

            pump_controller = asyncio.create_task(self.pump_control())
            temp_controller = asyncio.create_task(self.temp_control())
            # Held on self so the finally can reach them.
            #
            # These were locals, and the only cancellation was after
            # asyncio.wait() returned normally. Cancel run() itself while it is
            # awaiting that call - which is what stopping a step does - and
            # those lines never execute: CancelledError propagates straight
            # past them to the handler below. Both children survive run()
            # returning, and can then command the heater or the pump back on
            # after the finally has switched them off.
            self._children = (pump_controller, temp_controller)

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

            # Stop the children before commanding anything off.
            #
            # Setting running=False does not stop a task already inside an
            # iteration or awaiting an actor command, so switching the heater
            # off first leaves a window in which a surviving child turns it
            # straight back on - after stop() has returned and the interface
            # says the logic is gone.
            for task in getattr(self, "_children", ()):
                if task is not None and not task.done():
                    task.cancel()
            children = [t for t in getattr(self, "_children", ()) if t is not None]
            if children:
                await asyncio.gather(*children, return_exceptions=True)
            self._children = ()

            # Hand the integral on before shutting down, so the next instance to
            # control this kettle - usually the very next step in the profile -
            # does not have to rebuild the steady-state offset from nothing.
            self._remember_integral()

            # Each actuator independently: if one fails, the others are exactly
            # the equipment still running.
            for actor_id in (getattr(self, "heater", None),
                             getattr(self, "agitator", None)):
                if not actor_id:
                    continue
                try:
                    await self.actor_off(actor_id)
                except Exception as e:  # noqa: BLE001
                    logging.error(
                        "PIDHerms could not switch off %s: %s", actor_id, e
                    )

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
        # None, not 0, so the first calc() can seed it from the real reading.
        # Starting at 0 made the first derivative term -(Kd * inputValue): with a
        # kettle at 152 F and Kd 23.1 that is about -3500, which clamps the output
        # to zero. Every fresh controller therefore commanded the heater OFF for
        # its first sample, and the kettle logic is rebuilt at every step
        # transition, so that happened at every rest boundary of every brew.
        self._lastInput = None
        self._lastOutput = 0
        self._lastCalc = None

        if getTimeMs is None:
            self._getTimeMs = self._currentTimeMs
        else:
            self._getTimeMs = getTimeMs

    def set_output_max(self, value):
        """Change the output ceiling on a running controller.

        The outer HERMS loop's ceiling IS the HLT-over-mash band, and that band
        should not be the same while the tun holds only water as it is once
        grain is in. Both the clamp and the anti-windup test read _outputMax
        fresh on every calc(), so moving it is safe - but the accumulated
        integral must be brought down with it, or lowering the ceiling would
        leave a term above the new rail that the anti-windup logic then refuses
        to unwind.
        """
        value = float(value)
        if value <= self._outputMin:
            raise ValueError("outputMax must be greater than outputMin")
        self._outputMax = value
        self._iTerm = min(self._iTerm, value)
        self._lastOutput = min(self._lastOutput, value)

    def calc(self, inputValue, setpoint):
        now = self._getTimeMs()
        # First call: seed the history from the reading in hand rather than from
        # zero, and answer immediately instead of waiting out a sample period.
        if self._lastCalc is None:
            self._lastInput = inputValue
            self._lastCalc = now
        elif (now - self._lastCalc) < self._sampleTime:
            return self._lastOutput

        # Compute all the working error variables
        error = setpoint - inputValue
        dInput = inputValue - self._lastInput

        # Anti-windup: stop integrating only when the integral is pushing further
        # into the rail it is already against. Freezing whenever the output was
        # saturated - regardless of direction - meant a controller that had
        # bottomed out could not begin recovering until the proportional term
        # alone lifted it off the rail, which on a slow thermal plant is a long
        # steady-state error.
        at_ceiling = self._lastOutput >= self._outputMax
        at_floor = self._lastOutput <= self._outputMin
        winding_up = (at_ceiling and error > 0) or (at_floor and error < 0)
        if not winding_up:
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
