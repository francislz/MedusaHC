"""MedusaHC tool-change controller for Klipper.

The module replaces the long SET/DROP Jinja chains while retaining the normal
MedusaHC G-code names through ``MHC_macros.cfg``. It deliberately reads the
existing macro variables, so printer-specific geometry stays in
``MHC_variables.cfg`` instead of being hard-coded here.

Code map for maintainers
------------------------
* Configuration/status helpers read TOOL_CFG, GLOBAL_STATE, TOOL_STATE_n and
  TOOL_OFFSET.
* Sensor helpers read the existing ``pin_watch io`` Klipper object.
* Feeder and offset helpers contain the small reusable physical operations.
* ``_drop_active`` and ``_pick`` contain the dock motion paths.
* ``_after_pick`` contains printing-only prime and brush-cleaning behavior.
* ``cmd_MHC_*`` methods are the public Klipper G-code command handlers.

Normal tuning should be done in ``MHC_variables.cfg``. Edit motion formulas in
this file only when adapting MedusaHC to geometry that cannot be represented by
the existing coordinates, direction and speed variables. Motion strings use
Klipper feedrates in mm/min; user-facing speed variables are in mm/s and are
multiplied by 60 where needed.
"""

import logging


class _OperationPaused(Exception):
    """Internal control-flow signal after an active print was safely paused."""


class MedusaHC:
    """Own one MedusaHC operation at a time and expose Klipper commands."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.pin_watch_name = config.get("pin_watch", "pin_watch io")
        self.sensor_timeout = config.getfloat(
            "sensor_timeout", 0.50, minval=0.0, maxval=5.0
        )
        self.sensor_poll_interval = config.getfloat(
            "sensor_poll_interval", 0.01, above=0.0, maxval=0.25
        )
        # Trust the hall sensors, or fall back to GLOBAL_STATE.current_tool.
        # pin_watch cannot report a mounted tool without its head sensor
        # ("e" pin), so set this to 0 while that sensor is still being fitted:
        # the module then keeps software state exactly like the Jinja macros do.
        self.require_sensors = config.getboolean("require_sensors", True)
        # Register the bare DROP / TOOL_OFFSET_T / LAYER_SET / PRIME_FLAGS_SET
        # aliases. Turn this off when the same names already exist as
        # [gcode_macro] sections - Klipper refuses to start on a duplicate
        # command registration.
        self.legacy_commands = config.getboolean("legacy_commands", True)
        self.pin_watch = None
        self.operation = "idle"
        self.target_tool = -1
        self.last_error = ""
        self.feeder_open = False
        self.layer = 0
        self.return_pos = None
        self._brush_warned = False
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self._init_timer = self.reactor.register_timer(self._initialize_timer)
        self._register_commands()

    def _register_commands(self):
        """Register internal MHC_* commands; legacy names live in the CFG."""
        commands = {
            "MHC_SET": (self.cmd_MHC_SET, "Pick or change to a MedusaHC tool"),
            "MHC_DROP": (self.cmd_MHC_DROP, "Park the active MedusaHC tool"),
            "MHC_OPEN": (self.cmd_MHC_OPEN, "Open the MedusaHC feeder"),
            "MHC_CLOSE": (self.cmd_MHC_CLOSE, "Close the MedusaHC feeder"),
            "MHC_CLEAN": (self.cmd_MHC_CLEAN, "Clean the active MedusaHC tool"),
            "MHC_ERROR": (self.cmd_MHC_ERROR, "Run MedusaHC error recovery"),
            "MHC_TOOL_OFFSET": (self.cmd_MHC_TOOL_OFFSET, "Apply a tool offset"),
            "MHC_ASSIGN_TOOL": (self.cmd_MHC_ASSIGN_TOOL, "Sync klipper-toolchanger"),
            "MHC_LAYER_SET": (self.cmd_MHC_LAYER_SET, "Update the current layer"),
        }
        if self.legacy_commands:
            # Invisible compatibility commands for existing slicer and common
            # macro files. Unlike [gcode_macro] wrappers, these do not create
            # buttons in the Mainsail macro panel.
            commands.update({
                "DROP": (self.cmd_MHC_DROP, "Park the active MedusaHC tool"),
                "TOOL_OFFSET_T": (
                    self.cmd_MHC_TOOL_OFFSET, "Apply a MedusaHC tool offset"
                ),
                "LAYER_SET": (self.cmd_MHC_LAYER_SET, "Update the current layer"),
                "PRIME_FLAGS_SET": (
                    self.cmd_PRIME_FLAGS_SET,
                    "Mark first prime complete for all tools"
                ),
            })
        for name, (handler, description) in commands.items():
            self.gcode.register_command(name, handler, desc=description)

    def _handle_ready(self):
        """Resolve runtime objects and put the feeder servo in a known state."""
        self.pin_watch = self.printer.lookup_object(self.pin_watch_name, None)
        if self.pin_watch is None and self.require_sensors:
            raise self.printer.config_error(
                "[medusahc] could not find [%s]. Add the section, or set "
                "require_sensors: False to run on software tool state."
                % self.pin_watch_name
            )
        if self.printer.is_shutdown():
            return
        self.feeder_open = bool(int(self._global().get("feeder_open", 0)))
        # DO NOT issue G-code here.
        #
        # This used to move the servo to its closed position from inside the
        # klippy:ready handler. SET_SERVO schedules a timed pulse against
        # toolhead.get_last_move_time(), and at the instant ready fires the
        # toolhead's print_time base is not yet reliably ahead of the MCU
        # clock. The result was a queue_digital_out asking the toolhead MCU to
        # change a pin ~28 ms in the PAST, which the MCU rejects by shutting
        # down with "Timer too close" - on a board that was otherwise 1.4%
        # busy with zero CAN retransmits.
        #
        # The feeder is put in its known closed state by _initialize_timer
        # below, which runs off the reactor two seconds later and is safe.
        # Set the servo's initial_angle in the [servo] section to the same
        # closed angle so it never rests mid-travel before then.
        self.reactor.update_timer(self._init_timer, self.reactor.monotonic() + 2.0)

    def _initialize_timer(self, eventtime):
        """Populate derived runtime values after all Klipper objects are ready."""
        # Bail out if something else has already shut the printer down. Every
        # command below goes through run_script_from_command, and after a
        # shutdown Klipper swaps in its restricted handler table, so even
        # SET_GCODE_VARIABLE comes back as the shutdown message. Reporting that
        # as "MedusaHC initialization failed" buries the real fault.
        if self.printer.is_shutdown():
            self.last_error = "Printer shut down before MedusaHC could initialize"
            logging.warning("MedusaHC: %s", self.last_error)
            return self.reactor.NEVER
        try:
            cfg = self._tool_cfg()
            for name, multiplier in (
                ("fast_feedrate", float(cfg["fast_speed"]) * 60.0),
                ("slow_feedrate", float(cfg.get("slow_speed", 40.0)) * 60.0),
            ):
                self._set_compat(name, multiplier)
            tmc = self.printer.lookup_object("tmc2209 extruder", None)
            if tmc is not None:
                status = tmc.get_status(eventtime)
                current = float(status.get("run_current", 0.0))
                self._set_compat("e_cur", current)
                # TOOL_CFG.e_cur_high (absolute amps) wins when present. The
                # multiplier form is kept for upstream configs, but an absolute
                # value cannot silently become 0.0 A if run_current is unreadable.
                if "e_cur_high" in cfg:
                    high = float(cfg["e_cur_high"])
                else:
                    high = current * float(cfg.get("e_cur_high_mult", 1.7))
                self._set_compat("e_cur_high", high)
            self._restore_saved_offsets()
            self._close()
            self.gcode.respond_info("MedusaHC controller initialized")
        except Exception as exc:
            if self.printer.is_shutdown():
                # The printer went down mid-initialization. The MCU shutdown
                # reason is the real error; do not dress it up as ours.
                self.last_error = "Printer shut down during MedusaHC init"
                logging.warning("MedusaHC: %s (%s)", self.last_error, exc)
            else:
                logging.exception("MedusaHC initialization failed")
                self.last_error = "Initialization failed; see klippy.log"
        return self.reactor.NEVER

    def _restore_saved_offsets(self):
        """Restore per-tool XYZ offsets previously stored by SAVE_VARIABLE."""
        saved = self.printer.lookup_object("save_variables", None)
        values = getattr(saved, "allVariables", {}) if saved is not None else {}
        offset_macro = self._macro_name("TOOL_OFFSET")
        for tool in range(self._tool_count()):
            for axis in ("x", "y", "z"):
                saved_name = "t%d_gcode_%s_offset" % (tool, axis)
                if saved_name not in values:
                    continue
                self._run(
                    "SET_GCODE_VARIABLE MACRO=%s VARIABLE=t%d_off_%s VALUE=%s"
                    % (offset_macro, tool, axis, float(values[saved_name]))
                )

    def get_status(self, eventtime):
        """Return a compact status dictionary for Klipper API consumers."""
        source = self._sensor_source()
        raw_state = getattr(source, "state", {}) or {}
        sensors = {}
        for name, value in raw_state.items():
            try:
                sensors[str(name)] = int(value)
            except (TypeError, ValueError):
                pass
        return {
            "operation": self.operation,
            "require_sensors": self.require_sensors,
            "current_tool": self._current_tool(),
            "target_tool": self.target_tool,
            "last_error": self.last_error,
            "feeder_open": self.feeder_open,
            "layer": self.layer,
            "sensor_error": self._current_tool() == -2,
            "tool_count": self._tool_count(),
            "sensors": sensors,
        }

    # ---------------------------------------------------------------------
    # Existing MedusaHC configuration and sensor access
    # ---------------------------------------------------------------------

    def _macro_name(self, name):
        """Prefer the future hidden macro name while accepting legacy config."""
        candidates = (name,) if name.startswith("_") else ("_" + name, name)
        for candidate in candidates:
            if self.printer.lookup_object("gcode_macro %s" % candidate, None) is not None:
                return candidate
        raise self.printer.command_error(
            "MedusaHC requires [gcode_macro %s] or [gcode_macro _%s]"
            % (name, name)
        )

    def _macro(self, name):
        resolved = self._macro_name(name)
        obj = self.printer.lookup_object("gcode_macro %s" % resolved, None)
        if obj is None:
            raise self.printer.command_error(
                "MedusaHC requires [gcode_macro %s]" % resolved
            )
        return obj.variables

    def _tool_cfg(self):
        return self._macro("TOOL_CFG")

    def _global(self):
        return self._macro("GLOBAL_STATE")

    def _offsets(self):
        return self._macro("TOOL_OFFSET")

    def _tool_state(self, tool):
        """Per-tool prime/brush settings, or an empty mapping.

        Every value read from here has a default, so a printer with no purge
        bin and a delegated brush macro never needs these sections at all.
        A missing one must not abort a tool change, which is what the hard
        _macro() lookup used to do:
            "MedusaHC requires [gcode_macro TOOL_STATE_0] or [_TOOL_STATE_0]"
        """
        name = "TOOL_STATE_%d" % tool
        for candidate in ("_" + name, name):
            obj = self.printer.lookup_object("gcode_macro %s" % candidate, None)
            if obj is not None:
                return obj.variables
        return {}

    def _sensor_source(self):
        if self.pin_watch is None:
            self.pin_watch = self.printer.lookup_object(self.pin_watch_name, None)
        return self.pin_watch

    def _servo(self):
        """Servo name for the feeder, so this is not hard-coded to my_servo."""
        return str(self._tool_cfg().get("servo_name", "my_servo"))

    def _current_tool(self):
        """Authoritative tool state: hall sensors, or software when disabled."""
        if not self.require_sensors:
            return int(self._global().get("current_tool", -1))
        source = self._sensor_source()
        return int(getattr(source, "current_tool", -2)) if source else -2

    def _record_tool(self, tool):
        """Mirror the tool state into GLOBAL_STATE for macros and the UI."""
        self._set_compat("current_tool", int(tool))

    def _tool_count(self):
        return int(self._global().get("max_tool", 0))

    def _validate_tool(self, gcmd, tool):
        count = self._tool_count()
        if tool < 0 or tool >= count:
            raise gcmd.error("Tool T%d is outside configured range T0..T%d" % (tool, count - 1))

    def _run(self, script):
        self.gcode.run_script_from_command(script)

    def _set_compat(self, variable, value):
        """Mirror a runtime value into GLOBAL_STATE for macros and the UI.

        Klipper's SET_GCODE_VARIABLE refuses to create a variable that the
        macro did not declare, so a config that trims the unused compatibility
        variables would otherwise abort initialization on the first one. This
        is only a mirror - nothing here is load-bearing - so a missing variable
        is skipped rather than raised.
        """
        global_macro = self._macro_name("GLOBAL_STATE")
        if variable not in self._global():
            logging.debug(
                "MedusaHC: %s does not declare '%s'; skipping mirror",
                global_macro, variable,
            )
            return
        self._run(
            "SET_GCODE_VARIABLE MACRO=%s VARIABLE=%s VALUE=%s"
            % (global_macro, variable, value)
        )

    def _wait_moves(self):
        # M400 waits via Klipper's reactor, so button callbacks continue to run.
        self._run("M400")

    def _wait_for_tool(self, expected):
        if not self.require_sensors:
            # Nothing measures the head, so the move is taken on trust.
            self._record_tool(expected)
            return True
        if self._current_tool() == expected:
            self._record_tool(expected)
            return True
        deadline = self.reactor.monotonic() + self.sensor_timeout
        while self.reactor.monotonic() < deadline:
            wake = min(deadline, self.reactor.monotonic() + self.sensor_poll_interval)
            self.reactor.pause(wake)
            if self._current_tool() == expected:
                self._record_tool(expected)
                return True
        if self._current_tool() == expected:
            self._record_tool(expected)
            return True
        return False

    def _begin(self, operation, target=-1):
        if self.operation != "idle":
            raise self.printer.command_error(
                "MedusaHC is busy: %s" % self.operation
            )
        self.operation = operation
        self.target_tool = target
        self.last_error = ""

    def _finish(self):
        self.operation = "idle"

    def _fail(self, message):
        self.last_error = message
        logging.error("MedusaHC: %s", message)
        stats = self.printer.lookup_object("print_stats", None)
        print_was_active = getattr(stats, "state", "") in ("printing", "paused")
        try:
            self._run("MHC_ERROR")
        finally:
            self._finish()
        if print_was_active:
            raise _OperationPaused(message)
        raise self.printer.command_error(message)

    def _is_printing(self):
        stats = self.printer.lookup_object("print_stats", None)
        return getattr(stats, "state", "") == "printing"

    def _heater_temperature(self, tool):
        name = "extruder" if tool == 0 else "extruder%d" % tool
        heater = self.printer.lookup_object(name, None)
        if heater is None:
            return 0.0
        status = heater.get_status(self.reactor.monotonic())
        return float(status.get("temperature", 0.0))

    def _motion_values(self, tool):
        """Resolve one tool's geometry and convert configured speeds."""
        cfg = self._tool_cfg()
        direction = int(cfg.get("tools_direction", 1))
        if direction not in (-1, 1):
            raise self.printer.command_error("TOOL_CFG.tools_direction must be 1 or -1")
        # Latch axis is inferred from the configuration rather than set by a
        # flag: a config that defines z_dock_open/z_dock_lock is a moving-gantry
        # machine that works the dock latch vertically; anything else is the
        # original fixed-gantry layout that works it sideways with x_shift.
        has_z = "z_dock_open" in cfg and "z_dock_lock" in cfg
        if not has_z and "x_shift" not in cfg:
            raise self.printer.command_error(
                "TOOL_CFG needs either z_dock_open + z_dock_lock (vertical "
                "latch) or x_shift (horizontal latch)"
            )
        values = {
            "x": float(cfg["x_t%d" % tool]),
            "y_safe": float(cfg["y_safe"]),
            "y_latch": float(cfg["y_latch"]),
            "accel": float(cfg["fast_accel"]),
            "feed": float(cfg["fast_speed"]) * 60.0,
            "slow_feed": float(cfg.get("slow_speed", 40.0)) * 60.0,
            "direction": direction,
            "latch_axis": "z" if has_z else "x",
            # Prime and brush geometry is optional. A machine with no purge bin
            # and a single shared brush leaves these unset and delegates
            # cleaning to the CLEAN macro instead.
            "y_prime": float(cfg["y_prime"]) if "y_prime" in cfg else None,
            "y_brush": float(cfg["y_brush"]) if "y_brush" in cfg else None,
            "x_prime_shift": float(cfg.get("x_prime_shift", 0.0)),
            "x_shift": float(cfg.get("x_shift", 0.0)),
        }
        if has_z:
            values.update({
                "z_open": float(cfg["z_dock_open"]),
                "z_lock": float(cfg["z_dock_lock"]),
                "z_feed": float(cfg.get("z_speed", 15.0)) * 60.0,
                "z_latch_feed": float(cfg.get("z_latch_speed", 5.0)) * 60.0,
            })
        return values

    # ---------------------------------------------------------------------
    # Reusable physical operations
    # ---------------------------------------------------------------------

    def _old_accel(self):
        toolhead = self.printer.lookup_object("toolhead")
        return float(toolhead.get_status(self.reactor.monotonic())["max_accel"])

    def _home(self):
        self._run(self._macro_name("HOME_REQUEST"))

    def _save_return(self):
        """Remember where to come back to after a change.

        Only needed on a vertical-latch machine: the dock sits at a fixed
        machine Z well above the print, so a change leaves the toolhead far
        from where the slicer left off. A fixed-gantry build never leaves print
        height and has nothing to restore.
        """
        self.return_pos = None
        toolhead = self.printer.lookup_object("toolhead")
        status = toolhead.get_status(self.reactor.monotonic())
        if "xyz" not in status.get("homed_axes", ""):
            return
        move = self.printer.lookup_object("gcode_move")
        pos = move.get_status(self.reactor.monotonic()).get("gcode_position")
        if pos is not None:
            self.return_pos = (float(pos[0]), float(pos[1]), float(pos[2]))

    def _do_return(self, v):
        """Travel back to the pre-change position, Z last."""
        if self.return_pos is None:
            return
        rx, ry, rz = self.return_pos
        self.return_pos = None
        move = self.printer.lookup_object("gcode_move")
        cz = float(move.get_status(self.reactor.monotonic())["gcode_position"][2])
        # Cross the bed at whichever is higher: where we already are, or 3 mm
        # above where we left off. The second term is what stops us ploughing
        # through a print taller than the current height.
        z_travel = max(cz, rz + 3.0)
        # The brush retreat now ends just above the print instead of back at
        # dock height, so this travel can be low. Cross in the Y >= y_safe
        # corridor and only then drop to a return point in front of it, so a
        # diagonal never cuts over the dock row.
        y_safe = v["y_safe"]
        self._run("""G90
G1 Z{ztravel} F{zfeed}
G1 X{rx} Y{ycorr} F{feed}
G1 Y{ry} F{feed}
G1 Z{rz} F{zfeed}""".format(
            ztravel=z_travel, zfeed=v.get("z_feed", v["feed"]),
            rx=rx, ry=ry, ycorr=max(ry, y_safe), rz=rz, feed=v["feed"]
        ))

    def _clear_offsets(self):
        """Zero every axis of the gcode offset before dock motion.

        Z matters here even though upstream only clears X and Y: dock heights
        are raw machine coordinates, so a leftover per-tool Z offset shifts the
        latch travel and drives the toolhead into the dock.
        """
        self._run("SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0")

    def _open(self):
        """Release the feeder latch using its servo and extruder movement."""
        if self.feeder_open:
            return
        state = self._global()
        cfg = self._tool_cfg()
        # These are cached by _initialize_timer, but a failed or not-yet-run
        # init would leave them at 0.0 and silently drive the extruder at zero
        # current, so fall back to the configuration on every call.
        high = float(state.get("e_cur_high", 0.0))
        base = float(state.get("e_cur", 0.0))
        if base <= 0.0:
            tmc = self.printer.lookup_object("tmc2209 extruder", None)
            if tmc is not None:
                base = float(tmc.get_status(
                    self.reactor.monotonic()).get("run_current", 0.0))
        if high <= 0.0:
            # Absolute value first, then the upstream multiplier form.
            if "e_cur_high" in cfg:
                high = float(cfg["e_cur_high"])
            elif base > 0.0:
                high = base * float(cfg.get("e_cur_high_mult", 1.7))
        if high <= 0.0 or base <= 0.0:
            raise self.printer.command_error(
                "MedusaHC: extruder current is unset (high=%s base=%s). Set "
                "TOOL_CFG.e_cur_high, or e_cur_high_mult with a readable "
                "[tmc2209 extruder]." % (high, base)
            )
        accel = float(cfg["fast_accel"])
        old_accel = self._old_accel()
        e_open = float(cfg.get("e_open", -5.0))
        servo_angle = float(cfg.get("servo_open_angle", 90.0))
        # M83 (relative extrusion) inside a saved gcode state, rather than a
        # bare G91. G91 makes every axis relative, so an abort part way through
        # would leave the machine in relative mode and send the next absolute
        # move somewhere unexpected.
        self._run("""SAVE_GCODE_STATE NAME=MHC_OPEN
SET_STEPPER_ENABLE STEPPER=extruder ENABLE=1
SET_VELOCITY_LIMIT ACCEL={accel}
SET_SERVO SERVO={servo} ANGLE={servo_angle}
G4 P200
SET_TMC_CURRENT STEPPER=extruder CURRENT={high}
M83
G1 E-0.3 F1000
G1 E0.3 F1000
G1 E-0.3 F1000
G1 E0.3 F1000
G1 E{e_open} F2500
M400
SET_VELOCITY_LIMIT ACCEL={old}
SET_TMC_CURRENT STEPPER=extruder CURRENT={base}
RESTORE_GCODE_STATE NAME=MHC_OPEN MOVE=0""".format(
            high=high, accel=accel, servo=self._servo(),
            servo_angle=servo_angle, e_open=e_open, old=old_accel, base=base
        ))
        self.feeder_open = True
        self._set_compat("feeder_open", 1)

    def _close(self):
        """Engage the feeder latch and restore its logical state."""
        cfg = self._tool_cfg()
        e_close = float(cfg.get("e_close", 3.0))
        servo_angle = float(cfg.get("servo_close_angle", 180.0))
        self._run("""SET_STEPPER_ENABLE STEPPER=extruder ENABLE=1
SET_SERVO SERVO={servo} ANGLE={servo_angle}
SAVE_GCODE_STATE NAME=MHC_CLOSE
M83
G1 E{e_close} F6000
M400
RESTORE_GCODE_STATE NAME=MHC_CLOSE MOVE=0""".format(
            servo=self._servo(), servo_angle=servo_angle, e_close=e_close))
        self.feeder_open = False
        self._set_compat("feeder_open", 0)

    def _activate_extruder(self, tool):
        """Make the mounted hotend the active extruder.

        Without this Klipper stays on `extruder` (T0) forever, so a bare
        M104/M109 from the slicer heats the wrong cartridge and the wrong
        tool's pressure_advance is applied. Every extruder object shares one
        physical stepper and TMC driver, so feeder moves are unaffected.
        """
        name = "extruder" if tool == 0 else "extruder%d" % tool
        target = self.printer.lookup_object(name, None)
        if target is None:
            return
        # Skip the call when it is already active; Klipper answers that with an
        # "Extruder %s already active" notice that looks like a fault in the
        # console during an otherwise clean tool change.
        toolhead = self.printer.lookup_object("toolhead", None)
        if toolhead is not None and toolhead.get_extruder() is target:
            return
        self._run("ACTIVATE_EXTRUDER EXTRUDER=%s" % name)

    # Names that would call straight back into this module.
    _BRUSH_RESERVED = ("CLEAN", "MHC_CLEAN")

    def _brush_macro(self):
        """Name of the macro that wipes the nozzle, or None.

        TOOL_CFG.brush_macro names it explicitly, so a printer can swap in its
        own routine without touching this file. With nothing configured, fall
        back to the conventional primitive names.

        CLEAN is rejected: MHC_macros.cfg wires CLEAN to MHC_CLEAN, so calling
        it from here would recurse forever.
        """
        configured = str(self._tool_cfg().get("brush_macro", "")).strip()
        if configured:
            if configured.upper() in self._BRUSH_RESERVED:
                if not self._brush_warned:
                    self._brush_warned = True
                    self.gcode.respond_info(
                        "MedusaHC: TOOL_CFG.brush_macro cannot be '%s' - that "
                        "command calls the toolchanger, which would recurse. "
                        "Point it at the macro that moves the toolhead."
                        % configured
                    )
                return None
            if self.printer.lookup_object(
                    "gcode_macro %s" % configured, None) is not None:
                return configured
            if not self._brush_warned:
                self._brush_warned = True
                self.gcode.respond_info(
                    "MedusaHC: TOOL_CFG.brush_macro is '%s' but no "
                    "[gcode_macro %s] exists; nozzle cleaning is disabled."
                    % (configured, configured)
                )
            return None
        for name in ("_BRUSH_WIPE", "BRUSH_WIPE"):
            if self.printer.lookup_object("gcode_macro %s" % name, None) is not None:
                return name
        return None

    def _shared_prime_params(self, tool, state):
        """Prime amounts for a brush macro that owns a shared purge bucket.

        Same amounts and first-use rules as the dock-relative prime in
        _after_pick, handed to the brush macro as parameters instead of being
        extruded here, because the bucket sits next to the shared brush rather
        than at a per-dock position. Empty outside a print or below 190C.
        """
        if not self._is_printing() or self._heater_temperature(tool) <= 190.0:
            return {}
        params = {}
        first = False
        if (int(state.get("first_prime_enabled", 1)) != 0
                and int(state.get("first_prime_flag", 1)) == 0):
            params["FIRST_PRIME"] = float(state.get("first_prime_amount", 0.0))
            params["FIRST_PRIME_SPEED"] = float(state.get("first_prime_speed", 1.0))
            self._run(
                "SET_GCODE_VARIABLE MACRO=%s VARIABLE=first_prime_flag VALUE=1"
                % self._macro_name("TOOL_STATE_%d" % tool)
            )
            first = True
        params["PRIME"] = float(state.get("prime_amount", 0.0))
        params["PRIME_SPEED"] = float(state.get("prime_speed", 1.0))
        params["PRIME_RETRACT"] = float(state.get(
            "first_prime_prime_retract", 0.2
        )) if first else float(state.get("prime_retract", 0.0))
        params["PRIME_RETRACT_SPEED"] = float(state.get("prime_retract_speed", 1.0))
        params["CLEAN_RETRACT"] = float(state.get(
            "first_prime_clean_retract", 0.1
        )) if first else float(state.get("clean_retract", 0.0))
        params["CLEAN_RETRACT_SPEED"] = float(state.get("clean_retract_speed", 1.0))
        return params

    def _brush(self, tool, min_temp=140.0, extra=None):
        """Wipe on the shared brush, skipping a nozzle too cold to clean."""
        macro = self._brush_macro()
        if macro is None:
            return False
        temp = self._heater_temperature(tool)
        if temp < min_temp:
            self.gcode.respond_info(
                "MHC: T%d is %.1fC, below %.1fC - skipping the brush"
                % (tool, temp, min_temp)
            )
            return True
        params = dict(extra or {})
        if self.return_pos is not None:
            # Mid-change: tell the brush it only has to lift clear of the
            # print, not climb back to dock height. _do_return then crosses
            # at that same height (rz + 3) and goes straight down to rz.
            params["RETREAT_Z"] = self.return_pos[2] + 3.0
        self._run(" ".join([macro] + [
            "%s=%.3f" % (key, value) for key, value in sorted(params.items())
        ]))
        return True

    def _apply_offset(self, tool, move=1):
        """Apply the stored XYZ correction for a selected hotend."""
        offsets = self._offsets()
        state = self._global()
        x = float(offsets.get("t%d_off_x" % tool, 0.0))
        y = float(offsets.get("t%d_off_y" % tool, 0.0))
        z = float(offsets.get("t%d_off_z" % tool, 0.0))
        self._run("SET_GCODE_OFFSET X=%s Y=%s Z=%s MOVE=%d" % (x, y, z, move))
        self.gcode.respond_info(
            "MHC_TOOL_OFFSET T%d: X=%s Y=%s Z=%s MOVE=%d" % (tool, x, y, z, move)
        )

    # ---------------------------------------------------------------------
    # Dock motion paths
    #
    # All X/Y formulas are expressed from the configured dock center. The
    # ``tools_direction`` multiplier mirrors the path for front/back layouts.
    # Keep the safety move before any latch movement when adapting this code.
    # ---------------------------------------------------------------------

    def _drop_active(self):
        """Park the attached hotend using whichever latch this machine has."""
        tool = self._current_tool()
        if tool == -1:
            self.gcode.respond_info("MHC_DROP: nothing installed")
            return
        if tool < 0 or tool >= self._tool_count():
            self._fail("MHC_DROP: ambiguous sensor state")
        v = self._motion_values(tool)
        if v["latch_axis"] == "z":
            self._drop_z(tool, v)
        else:
            self._drop_x(tool, v)

    def _drop_z(self, tool, v):
        """Park a hotend on a moving-gantry machine (vertical dock latch).

        Move order is safety critical. The docks stand up from the bed at
        Y < y_safe, so Z and X may only move while the toolhead is clear of
        them in Y:

            Y -> y_safe     get clear of the docks
            Z -> z_open     latch-open height, dock can accept the hotend
            X -> dock       line up with the column
            Y -> y_latch    slide the hotend into the cradle, feeder still shut
            Z -> z_lock     latch travel; the dock closes onto the hotend
            OPEN            release the feeder now the dock is holding it
            Y -> y_safe     withdraw, leaving the tool docked

        The hotend is positively held - by the feeder or by the dock - through
        every vertical move.
        """
        d = v["direction"]
        old_accel = self._old_accel()
        # Optional wipe before docking, while this tool's offset still applies.
        if int(self._tool_cfg().get("clean_on_drop", 0)) != 0:
            self._brush(tool)
        self._clear_offsets()
        self._run("""SET_VELOCITY_LIMIT ACCEL={accel}
G90
G1 Y{safe} F{feed}
G1 Z{zopen} F{zfeed}
G1 X{x} F{feed}
G1 Y{approach} F{feed}
G1 Y{latch} F{slow}
G1 Z{zlock} F{zlatch}""".format(
            accel=v["accel"], safe=v["y_safe"], zopen=v["z_open"],
            zfeed=v["z_feed"], x=v["x"], approach=v["y_latch"] + 20*d,
            latch=v["y_latch"], slow=v["slow_feed"], zlock=v["z_lock"],
            zlatch=v["z_latch_feed"], feed=v["feed"]
        ))
        self._open()
        self._run("""G4 P200
G1 Y{safe} F{feed}
SET_VELOCITY_LIMIT ACCEL={old}""".format(
            safe=v["y_safe"], feed=v["feed"], old=old_accel))
        self._wait_moves()
        if not self._wait_for_tool(-1):
            self._fail("MHC_DROP: sensors did not confirm an empty toolhead")
        self.gcode.respond_info("MHC_DROP OK: T%d parked" % tool)

    def _drop_x(self, tool, v):
        """Park a hotend on a fixed-gantry machine (horizontal dock latch)."""
        d = v["direction"]
        old_accel = self._old_accel()
        # Change the coordinate transform without compensating motion over the
        # print. The active offset is applied again only after reaching safety.
        self._apply_offset(0, move=0)
        self._clear_offsets()
        self._run("""SET_VELOCITY_LIMIT ACCEL={accel}
G90
G1 Y{safe} X{xapproach} F{feed}""".format(
            accel=v["accel"], safe=v["y_safe"], xapproach=v["x"] + 10*d,
            feed=v["feed"]
        ))
        if not self.feeder_open:
            self._open()
        self._run("""M106 S255
G1 Y{brushapproach} F{feed}
G1 X{xprime} F{feed}
G1 Y{latchapproach} F{feed}
G1 X{xshift} F{feed}
G1 Y{latch} F{feed}
G1 X{x} F{slow}
G1 Y{safe} F{feed}
SET_VELOCITY_LIMIT ACCEL={old}""".format(
            safe=v["y_safe"], brushapproach=v["y_brush"] + 3*d,
            xprime=v["x"] - v["x_prime_shift"]*d,
            feed=v["feed"], latchapproach=v["y_latch"] + 8*d,
            xshift=v["x"] - v["x_shift"]*d, latch=v["y_latch"], x=v["x"],
            slow=v["slow_feed"], old=old_accel
        ))
        self._wait_moves()
        if not self._wait_for_tool(-1):
            self._fail("MHC_DROP: dock sensors did not confirm an empty toolhead")
        self.gcode.respond_info("MHC_DROP OK: T%d parked" % tool)

    def _pick(self, tool):
        """Collect one hotend using whichever latch this machine has."""
        v = self._motion_values(tool)
        if v["latch_axis"] == "z":
            self._pick_z(tool, v)
        else:
            self._pick_x(tool, v)

    def _pick_z(self, tool, v):
        """Collect a hotend on a moving-gantry machine (vertical dock latch).

        The exact reverse of _drop_z:

            Y -> y_safe     clear the docks
            Z -> z_lock     the parked hotend sits at the locked height
            X -> dock       line up with the column
            OPEN            open the feeder so it can accept the hotend
            Y -> y_latch    slide onto the hotend (with a small seating jiggle)
            CLOSE           clamp it BEFORE any vertical move
            Z -> z_open     latch travel; the dock releases the hotend
            Y -> y_safe     withdraw, carrying the tool

        CLOSE deliberately happens before the latch travel, not after it as the
        horizontal path does: with the latch in Z the hotend must be positively
        held before the toolhead moves vertically.
        """
        d = v["direction"]
        old_accel = self._old_accel()
        self._clear_offsets()
        self._run("""SET_VELOCITY_LIMIT ACCEL={accel}
G90
G1 Y{safe} F{feed}
G1 Z{zlock} F{zfeed}
G1 X{x} F{feed}""".format(
            accel=v["accel"], safe=v["y_safe"], zlock=v["z_lock"],
            zfeed=v["z_feed"], x=v["x"], feed=v["feed"]))
        self._open()
        self._run("""G90
G1 Y{approach} F{feed}
G1 Y{latch} F{slow}
G1 Y{jiggle} F{slow}
G1 Y{latch} F{slow}""".format(
            approach=v["y_latch"] + 20*d, latch=v["y_latch"],
            jiggle=v["y_latch"] - 0.1*d, slow=v["slow_feed"], feed=v["feed"]))
        self._close()
        self._run("""G4 P200
G1 Z{zopen} F{zlatch}
G1 Y{out} F{slow}
G1 Y{safe} F{feed}
SET_VELOCITY_LIMIT ACCEL={old}""".format(
            zopen=v["z_open"], zlatch=v["z_latch_feed"],
            out=v["y_latch"] + 5*d, safe=v["y_safe"], slow=v["slow_feed"],
            feed=v["feed"], old=old_accel))
        self._wait_moves()
        if not self._wait_for_tool(tool):
            self._fail("MHC_SET: sensors did not confirm T%d" % tool)
        self._activate_extruder(tool)
        self._after_pick(tool, v)
        self.gcode.respond_info("MHC_SET OK: T%d installed" % tool)

    def _pick_x(self, tool, v):
        """Collect a hotend on a fixed-gantry machine (horizontal latch)."""
        d = v["direction"]
        old_accel = self._old_accel()
        self._apply_offset(0, move=0)
        self._clear_offsets()
        self._run("""SET_VELOCITY_LIMIT ACCEL={accel}
G90
G1 Y{safe} X{x} F{feed}""".format(
            accel=v["accel"], safe=v["y_safe"], x=v["x"], feed=v["feed"]
        ))
        if not self.feeder_open:
            self._open()
        self._run("""G1 Y{latch3} F{feed}
G1 Y{latch} F{slow}
G1 Y{latch03} F{slow}
G1 Y{latch} F{slow}
G1 X{xshift2} F{feed}
G1 X{xshift} F{slow}
G1 Y{latch5} F{slow}
G1 X{xshift_more} F{slow}
M106 S255""".format(
            feed=v["feed"],
            latch3=v["y_latch"] + 20*d, latch=v["y_latch"], latch03=v["y_latch"] - .1*d,
            slow=v["slow_feed"], xshift2=v["x"] - (v["x_shift"] - 4)*d,
            xshift=v["x"] - v["x_shift"]*d, latch5=v["y_latch"] + 5*d,
            xshift_more=v["x"] - (v["x_shift"] + 2)*d
        ))
        self._wait_moves()
        if not self._wait_for_tool(tool):
            self._fail("MHC_SET: sensors did not confirm T%d" % tool)
        self._close()
        self._activate_extruder(tool)
        self._after_pick(tool, v)
        self._run("SET_VELOCITY_LIMIT ACCEL=%s" % old_accel)
        self._run("M106 S0")
        self.gcode.respond_info("MHC_SET OK: T%d installed" % tool)

    def _after_pick(self, tool, v):
        """Prime, brush-clean and apply offsets after a successful pickup.

        Extrusion values come from TOOL_STATE_n. On a tool's first use during
        a print, the two dedicated short retracts replace the normal prime and
        cleaning retracts because the slicer has no matching unretract yet.
        """
        state = self._tool_state(tool)
        first_prime_executed = False
        # Priming needs somewhere to put the filament. A machine with no purge
        # bin leaves y_prime unset, and the whole prime block is skipped.
        if (v["y_prime"] is not None
                and self._is_printing()
                and self._heater_temperature(tool) > 190.0):
            self._run("G90\nG1 X%s F%s\nG1 Y%s F%s" % (
                v["x"] - v["x_prime_shift"] * v["direction"], v["feed"],
                v["y_prime"], v["feed"]))
            first_prime_enabled = int(state.get("first_prime_enabled", 1)) != 0
            if first_prime_enabled and int(state.get("first_prime_flag", 1)) == 0:
                amount = float(state.get("first_prime_amount", 0.0))
                speed = float(state.get("first_prime_speed", 1.0)) * 60.0
                self._run("G91\nG1 E%s F%s\nG90" % (amount, speed))
                state_macro = self._macro_name("TOOL_STATE_%d" % tool)
                self._run(
                    "SET_GCODE_VARIABLE MACRO=%s VARIABLE=first_prime_flag VALUE=1"
                    % state_macro
                )
                first_prime_executed = True
            amount = float(state.get("prime_amount", 0.0))
            speed = float(state.get("prime_speed", 1.0))
            retract = float(state.get(
                "first_prime_prime_retract", 0.2
            )) if first_prime_executed else float(state.get("prime_retract", 0.0))
            retract_speed = float(state.get("prime_retract_speed", 1.0))
            self._run("""G91
G1 E{e1} F{f1}
G1 E{e2} F{f2}
G1 E{e3} F{f3}
G1 E-{retract} F{rf}
G90""".format(
                e1=amount * .2, e2=amount * .3, e3=amount * .5,
                f1=speed * .5 * 60., f2=speed * .75 * 60., f3=speed * 60.,
                retract=retract, rf=retract_speed * 60.
            ))
        cfg = self._tool_cfg()
        if v["y_brush"] is None:
            # No dock-relative brush geometry. Either a standalone CLEAN macro
            # owns the brush (one shared brush with its own approach routing),
            # or there is no brush at all.
            cleaned = False
            if int(cfg.get("clean_on_pickup", 0)) != 0:
                cleaned = self._brush(
                    tool, extra=self._shared_prime_params(tool, state))
            if not cleaned:
                self._run("G90\nG1 Y%s F%s" % (v["y_safe"], v["feed"]))
            self._apply_offset(tool)
            return
        if self._is_printing() and int(state.get("clean_move", 1)) != 0:
            cmx = float(state.get("x_clean_move", 0.0))
            cmy = float(state.get("y_clean_move", 0.0))
            cmf = float(state.get("clean_move_speed", 250.0)) * 60.0
            # On a tool's first use the slicer has no matching tool-change
            # unretract queued. Use dedicated short retracts after both prime
            # and cleaning; later changes retain the normal retract values.
            retract = float(state.get(
                "first_prime_clean_retract", 0.1
            )) if first_prime_executed else float(state.get("clean_retract", 0.0))
            rf = float(state.get("clean_retract_speed", 1.0)) * 60.0
            d = v["direction"]
            ptfe = float(state.get("ptfe_clean_slow_speed", 12.5)) * 60.0
            self._run("""G90
G1 Y{brush} F{feed}
G1 X{xprime} F{feed}
G91
G1 X{xptfe} F{ptfe}
G1 Y{yptfe} F{feed}
G1 X{xptfe_back} F{feed}
G1 X{xptfe} F{ptfe}
G1 X{xbrush} F{feed}
G1 Y{ybrush} F{feed}
G1 X{cmx1} Y{cmy1} F{cmf}
G1 Y{cmy2} F{cmf}
G1 X{cmx2} Y{cmy1} F{cmf}
G1 Y{cmy2} F{cmf}
G1 X{cmx1} Y{cmy1} F{cmf}
G1 E-{retract} F{rf}
G90
G1 Y{safe} F{feed}""".format(
                xprime=v["x"]-v["x_prime_shift"]*d, brush=v["y_brush"], feed=v["feed"],
                xptfe=10*d, ptfe=ptfe, yptfe=6*d, xptfe_back=-10*d, xbrush=10*d,
                ybrush=-8*d,
                cmx1=-cmx*d, cmy1=cmy*d, cmy2=-cmy*d, cmx2=cmx*d,
                cmf=cmf, retract=retract, rf=rf, safe=v["y_safe"]
            ))
            self._apply_offset(tool)
        else:
            self._run("G1 Y%s F%s" % (v["y_safe"], v["feed"]))
            self._apply_offset(tool)

    # ---------------------------------------------------------------------
    # Public MHC_* G-code handlers
    # ---------------------------------------------------------------------

    def cmd_MHC_SET(self, gcmd):
        """Pick T, parking another attached tool first when necessary."""
        tool = gcmd.get_int("T", None)
        if tool is None:
            raise gcmd.error("MHC_SET requires T=<number>")
        self._validate_tool(gcmd, tool)
        self._begin("changing", tool)
        try:
            self._set_compat("error_state", 0)
            self._set_compat("target_tool", tool)
            self._home()
            # Capture the return position after homing, so a change that has to
            # home first still comes back to the right place.
            self._save_return()
            self._apply_offset(tool, move=0)
            direction = int(self._tool_cfg().get("tools_direction", 1))
            self._run("G91\nG1 %s F14000\nG90" % (("Y%s Z3" % (-2*direction)) if self._is_printing() else "Z1"))
            current = self._current_tool()
            if current == -2:
                self._fail("MHC_SET: ambiguous sensor state")
            if current == tool:
                self._apply_offset(tool)
                self.gcode.respond_info("MHC_SET: T%d already installed" % tool)
                return
            if current >= 0:
                self.operation = "dropping"
                self._drop_active()
            self.operation = "picking"
            self._pick(tool)
            # Vertical-latch machines end the change at dock height, far from
            # the print. _do_return is a no-op when nothing was saved.
            self._do_return(self._motion_values(tool))
        except _OperationPaused as exc:
            self.gcode.respond_info("MedusaHC paused: %s" % exc)
        finally:
            if self.operation != "idle":
                self._finish()

    def cmd_MHC_DROP(self, gcmd):
        """Park the attached hotend, if any."""
        self._begin("dropping")
        try:
            self._home()
            self._drop_active()
        except _OperationPaused as exc:
            self.gcode.respond_info("MedusaHC paused: %s" % exc)
        finally:
            if self.operation != "idle":
                self._finish()

    def cmd_MHC_OPEN(self, gcmd):
        self._open()

    def cmd_MHC_CLOSE(self, gcmd):
        self._close()

    def cmd_MHC_CLEAN(self, gcmd):
        tool = self._current_tool()
        if tool < 0:
            self.gcode.respond_info("MHC_CLEAN: no tool installed")
            return
        self._home()
        v = self._motion_values(tool)
        if v["y_brush"] is None:
            if not self._brush(tool):
                self.gcode.respond_info(
                    "MHC_CLEAN: no brush configured (set TOOL_CFG.y_brush for a "
                    "dock-relative brush, or provide a _BRUSH_WIPE macro)"
                )
            return
        state = self._tool_state(tool)
        old_accel = self._old_accel()
        cmx = float(state.get("x_clean_move", 0.0))
        cmy = float(state.get("y_clean_move", 0.0))
        cmf = float(state.get("clean_move_speed", 250.0)) * 60.0
        ptfe = float(state.get("ptfe_clean_slow_speed", 12.5)) * 60.0
        d = v["direction"]
        self._run("""SET_VELOCITY_LIMIT ACCEL={accel}
G90
G1 Y{safe} F{feed}
G1 X{xprime} F{feed}
G1 Y{brush} F{feed}
G91
G1 X{xptfe} F{ptfe}
G1 Y{yptfe} F{feed}
G1 X{xptfe_back} F{feed}
G1 X{xptfe} F{ptfe}
G1 X{xbrush} F{feed}
G1 Y{ybrush} F{feed}
G1 X{cmx1} Y{cmy1} F{cmf}
G1 Y{cmy2} F{cmf}
G1 X{cmx2} Y{cmy1} F{cmf}
G1 Y{cmy2} F{cmf}
G1 X{cmx1} Y{cmy1} F{cmf}
G90
G1 Y{safe} F{feed}
SET_VELOCITY_LIMIT ACCEL={old}""".format(
            accel=v["accel"], safe=v["y_safe"], feed=v["feed"],
            xprime=v["x"] - v["x_prime_shift"]*d, brush=v["y_brush"],
            xptfe=10*d, ptfe=ptfe, yptfe=6*d, xptfe_back=-10*d,
            xbrush=10*d, ybrush=-8*d,
            cmx1=-cmx*d, cmy1=cmy*d,
            cmy2=-cmy*d, cmx2=cmx*d, cmf=cmf, old=old_accel
        ))

    def cmd_MHC_ERROR(self, gcmd):
        """Move away from the docks and pause only when a print is active."""
        stats = self.printer.lookup_object("print_stats", None)
        state = getattr(stats, "state", "")
        if state in ("printing", "paused"):
            self._set_compat("error_state", 1)
            cfg = self._tool_cfg()
            safe = float(cfg["y_safe"]) + 50.0 * int(cfg.get("tools_direction", 1))
            # Retreat in Y first - the toolhead may still be between the docks -
            # and only then lift, so the escape never crosses a dock column.
            script = "G90\nG1 Y%s F6000" % safe
            if "z_dock_open" in cfg:
                script += "\nG1 Z%s F%s" % (
                    float(cfg["z_dock_open"]),
                    float(cfg.get("z_speed", 15.0)) * 60.0,
                )
            self._run(script + "\nPAUSE")
        else:
            self.gcode.respond_info("MHC_ERROR: no active print; printer was not paused")

    def cmd_MHC_TOOL_OFFSET(self, gcmd):
        tool = gcmd.get_int("T", None)
        if tool is None:
            raise gcmd.error("MHC_TOOL_OFFSET requires T=<number>")
        self._validate_tool(gcmd, tool)
        self._apply_offset(tool, gcmd.get_int("MOVE", 1, minval=0, maxval=1))

    def cmd_MHC_ASSIGN_TOOL(self, gcmd):
        tool = self._current_tool()
        if tool >= 0:
            self._run("INITIALIZE_TOOLCHANGER T=%d" % tool)
        else:
            self.gcode.respond_info("No tool installed; initialization deferred")

    def cmd_MHC_LAYER_SET(self, gcmd):
        layer = gcmd.get_int("L", None)
        if layer is None:
            return
        self.layer = layer
        self._set_compat("layer", layer)

    def cmd_PRIME_FLAGS_SET(self, gcmd):
        for tool in range(self._tool_count()):
            name = "TOOL_STATE_%d" % tool
            for candidate in ("_" + name, name):
                obj = self.printer.lookup_object(
                    "gcode_macro %s" % candidate, None)
                if obj is None:
                    continue
                # Only write a flag the macro actually declares.
                if "first_prime_flag" in obj.variables:
                    self._run(
                        "SET_GCODE_VARIABLE MACRO=%s "
                        "VARIABLE=first_prime_flag VALUE=1" % candidate
                    )
                break


def load_config(config):
    """Klipper entry point for the ``[medusahc]`` config section."""
    return MedusaHC(config)
