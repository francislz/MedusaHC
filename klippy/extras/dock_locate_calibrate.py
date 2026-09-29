"""Dock location calibration for a MedusaHC-style toolchanger.

Adapted from 3dfiyMyLife/Lineux-Hotswap's dock_locate_calibrate.py, which in
turn derives from Klipper_ToolChanger/probe_multi_axis.py.

WHY THE MCU STEP COUNTERS
-------------------------
The toolhead is placed at the dock by hand, usually with the X/Y motors
disabled, because feel beats jogging for seating a hotend. Klipper therefore
has no idea where the carriage actually is, and `GET_POSITION` would lie.

What is still true is that the stepper MCU step counter only advances when the
motor is *driven*. So homing from the hand-set position back to the endstop
turns the motors by exactly the physical distance between the two, and the
change in MCU counts measures it. That also makes the result immune to steps
skipped on the way in.

CoreXY, with Klipper's convention that stepper_x tracks (x + y) and stepper_y
tracks (x - y):

    dx = final_x_counts - initial_x_counts          (steps)
    dy = final_y_counts - initial_y_counts

    travelled_x = (dx + dy) / 2 * xy_resolution     (mm)
    travelled_y = (dx - dy) / 2 * xy_resolution

    dock_x = x_endstop - travelled_x
    dock_y = y_endstop - travelled_y

The maths is independent of which way each axis homes, because it measures a
displacement. It does depend on x_endstop/y_endstop being the real
`position_endstop` values - on this printer X homes to 0 and Y homes to 361,
which is not the usual both-to-max arrangement.

WHAT THIS ADDS OVER UPSTREAM
----------------------------
* Z is reported as well. A vertical-latch dock needs z_dock_lock, and Z is read
  straight from the toolhead because the gantry is never hand-moved (four
  belt-driven steppers - disabling them drops it).
* The escape move is a Y retract, not upstream's 3 mm X nudge. Here the
  toolhead is engaged with a hotend that is captured in the dock; shoving it
  sideways would side-load the dock. Pulling straight out in Y is the release
  direction.
* Results are printed in the exact form MedusaHC's TOOL_CFG wants, and compared
  against the configured values so a sign or setup error is obvious instead of
  silently producing a plausible-looking number.
"""

import logging


class DockLocateCalibrate:

    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')

        # mm of travel per microstep:
        #     rotation_distance / full_steps_per_rotation / microsteps
        # This printer: 40 / 200 / 16 = 0.0125
        self.xy_resolution = config.getfloat('xy_resolution', above=0.)

        # The real [stepper_x]/[stepper_y] position_endstop values - NOT
        # position_max, and NOT assumed to be the same direction on both axes.
        self.x_endstop = config.getfloat('x_endstop')
        self.y_endstop = config.getfloat('y_endstop')

        # Escape from the dock before homing. Z first (clear the latch), then
        # Y (pull the toolhead off the hotend). Both relative, both optional.
        self.escape_z = config.getfloat('escape_z', 0.)
        self.escape_y = config.getfloat('escape_y', 40.)
        self.escape_speed = config.getfloat('escape_speed', 30., above=0.)

        # Sensorless homing needs the StallGuard registers to settle between
        # moves, so this is longer than upstream's fixed 1 s.
        self.settle_ms = config.getint('settle_ms', 1500, minval=0)

        # Optional sanity check against the configured geometry.
        self.tool_cfg_name = config.get('tool_cfg', 'TOOL_CFG')
        self.warn_tolerance = config.getfloat('warn_tolerance', 5.0, minval=0.)

        self.last_result = {}
        self.gcode.register_command(
            'DOCK_LOCATE_CALIBRATE', self.cmd_DOCK_LOCATE_CALIBRATE,
            desc=self.cmd_DOCK_LOCATE_CALIBRATE_help)

    def get_status(self, eventtime):
        return dict(self.last_result)

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    def _mcu_counts(self):
        """Raw step counts for the two CoreXY steppers."""
        toolhead = self.printer.lookup_object('toolhead')
        counts = {}
        for stepper in toolhead.kin.get_steppers():
            name = stepper.get_name()
            if name in ('stepper_x', 'stepper_y'):
                counts[name] = stepper.get_mcu_position()
        missing = {'stepper_x', 'stepper_y'} - set(counts)
        if missing:
            raise self.printer.command_error(
                "Could not read MCU position for %s" % ", ".join(sorted(missing)))
        return counts['stepper_x'], counts['stepper_y']

    def _macro_vars(self, name):
        for candidate in ('_' + name, name):
            obj = self.printer.lookup_object('gcode_macro %s' % candidate, None)
            if obj is not None:
                return obj.variables
        return {}

    def _run(self, script):
        self.gcode.run_script_from_command(script)

    # ------------------------------------------------------------------
    # command
    # ------------------------------------------------------------------

    cmd_DOCK_LOCATE_CALIBRATE_help = (
        "Measure a dock's true X/Y/Z. Seat the toolhead in the dock by hand "
        "first, then run this. Optional: TOOL=<n> to compare against that "
        "tool's configured coordinates.")

    def cmd_DOCK_LOCATE_CALIBRATE(self, gcmd):
        tool = gcmd.get_int('TOOL', None, minval=0)
        toolhead = self.printer.lookup_object('toolhead')

        # Z is read before anything moves. It is only meaningful if Z is homed,
        # because unlike X/Y the gantry cannot be hand-positioned.
        status = toolhead.get_status(self.printer.get_reactor().monotonic())
        homed = status.get('homed_axes', '')
        dock_z = toolhead.get_position()[2]
        z_trustworthy = 'z' in homed

        initial_x, initial_y = self._mcu_counts()
        logging.info("dock_locate_calibrate: initial counts x=%s y=%s",
                     initial_x, initial_y)

        # Adopt the current position so the escape move is allowed. Requires
        # [force_move] enable_force_move: True.
        self._run('SET_KINEMATIC_POSITION')

        # Escape: lift clear of the latch, then pull out in Y.
        escape = ['G91']
        if self.escape_z:
            escape.append('G1 Z%.3f F%.0f' % (self.escape_z, self.escape_speed * 60.))
        if self.escape_y:
            escape.append('G1 Y%.3f F%.0f' % (self.escape_y, self.escape_speed * 60.))
        escape.append('G90')
        self._run('\n'.join(escape))

        # Home Y first: on this machine Y homes to max, which carries the
        # toolhead away from the dock rack before X moves at all.
        self._run('G28 Y')
        if self.settle_ms:
            self._run('G4 P%d' % self.settle_ms)
        self._run('G28 X')

        final_x, final_y = self._mcu_counts()
        logging.info("dock_locate_calibrate: final counts x=%s y=%s",
                     final_x, final_y)

        dx = final_x - initial_x
        dy = final_y - initial_y

        travelled_x = (dx + dy) / 2.0 * self.xy_resolution
        travelled_y = (dx - dy) / 2.0 * self.xy_resolution

        dock_x = self.x_endstop - travelled_x
        dock_y = self.y_endstop - travelled_y

        self.last_result = {
            'x': dock_x, 'y': dock_y, 'z': dock_z,
            'tool': -1 if tool is None else tool,
        }

        lines = [
            "Dock location calibration",
            "-------------------------",
            "MCU counts   initial X=%d Y=%d" % (initial_x, initial_y),
            "             final   X=%d Y=%d" % (final_x, final_y),
            "             delta   X=%d Y=%d" % (dx, dy),
            "Travelled    X=%.3f  Y=%.3f mm" % (travelled_x, travelled_y),
            "Endstops     X=%.2f  Y=%.2f" % (self.x_endstop, self.y_endstop),
            "",
            "MEASURED DOCK POSITION",
            "  X = %.2f" % dock_x,
            "  Y = %.2f" % dock_y,
        ]
        if z_trustworthy:
            lines.append("  Z = %.2f" % dock_z)
        else:
            lines.append("  Z = %.2f  (Z NOT HOMED - ignore this value)" % dock_z)

        cfg = self._macro_vars(self.tool_cfg_name)
        if tool is not None and cfg:
            lines += ["", "Put into %s:" % self.tool_cfg_name,
                      "  variable_x_t%d: %.2f" % (tool, dock_x)]
            self._compare(lines, cfg, 'x_t%d' % tool, dock_x)
            self._compare(lines, cfg, 'y_latch', dock_y)
            if z_trustworthy:
                self._compare(lines, cfg, 'z_dock_lock', dock_z)
                self._compare(lines, cfg, 'z_dock_open', dock_z)

        gcmd.respond_info("\n".join(lines))

    def _compare(self, lines, cfg, key, measured):
        """Flag a measurement that is nowhere near the configured value."""
        if key not in cfg:
            return
        try:
            current = float(cfg[key])
        except (TypeError, ValueError):
            return
        delta = measured - current
        flag = ""
        if abs(delta) > self.warn_tolerance:
            flag = "   <-- CHECK: %.2f mm from the configured value" % delta
        lines.append("  %-14s configured %8.2f   measured %8.2f%s"
                     % (key, current, measured, flag))


def load_config(config):
    return DockLocateCalibrate(config)
