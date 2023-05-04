// Copyright (c) 2020 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    This is a full featured TTL PWM spindle This does not include speed/power
    compensation. Use the Laser class for that.
*/
#include "PWMSpindle.h"
#include "../GCode.h"   // gc_state.modal

//ATC:
#include "../Protocol.h"
#include "../Uart.h"
#include "../Machine/MachineConfig.h"
#include "../Limits.h"  // limitsMaxPosition
#include "../System.h"  // sys
//ATC^

// ======================= PWM ==============================
/*
    This gets called at startup or whenever a spindle setting changes
    If the spindle is running it will stop and need to be restarted with M3Snnnn
*/

namespace Spindles {
    void PWM::init() {
        log_info("Initilising PWM ATC...");
        is_reversable = _direction_pin.defined();

        if (_output_pin.defined()) {
            if (_output_pin.capabilities().has(Pin::Capabilities::PWM)) {
                auto outputNative = _output_pin.getNative(Pin::Capabilities::PWM);
                _pwm              = new PwmPin(_output_pin, _pwm_freq);
            } else {
                log_error(name() << " output pin " << _output_pin.name().c_str() << " cannot do PWM");
            }
        } else {
            log_error(name() << " output pin not defined");
        }

        _current_state    = SpindleState::Disable;
        _current_pwm_duty = 0;

        _enable_pin.setAttr(Pin::Attr::Output);
        _direction_pin.setAttr(Pin::Attr::Output);

        if (_speeds.size() == 0) {
            // The default speed map for a PWM spindle is linear from 0=0% to 10000=100%
            linearSpeeds(10000, 100.0f);
        }
        setupSpeeds(_pwm->period());

//ATC Stuff:
        _atc_valve_pin.setAttr(Pin::Attr::Output);
        _atc_dustoff_pin.setAttr(Pin::Attr::Output);
        _toolsetter_dustoff.setAttr(Pin::Attr::Output);
        
        if (!_atc_valve_pin.defined()) {
            log_error("ATC: " << _atc_valve_pin.name() << " must be defined");
            return;
        }

        log_info("ATC Init Valve:" << _atc_valve_pin.name() << " Dustoff:" << _atc_dustoff_pin.name());

        // determine top of z for safest XY travel above things
        auto axisConfig = config->_axes->_axis[Z_AXIS];
        top_of_z        = limitsMaxPosition(Z_AXIS) - axisConfig->_motors[0]->_pulloff;

        if (_ets_mpos.size() != 3) {  // will use a for loop...and include tool locations...n_axis
            log_error("ATC ETS mpos wrong");
            return;  // failed
        }

        //New Handler functions to get tool height setter:
        tool[ETS_INDEX].mpos[X_AXIS] = _ets_mpos.at(0);
        tool[ETS_INDEX].mpos[Y_AXIS] = _ets_mpos.at(1);
        tool[ETS_INDEX].mpos[Z_AXIS] = _ets_mpos.at(2);

        //New Handler functions to set tool holder positions:
        for (int i = 0; i < TOOL_COUNT; i++) {
            if (_tool_mpos[i].size() != 3) {
                log_error("ATC Tool mpos wrong. Tool:" << i + 1);
                return;  // failed
            }
            tool[i + 1].mpos[X_AXIS] = _tool_mpos[i].at(0);
            tool[i + 1].mpos[Y_AXIS] = _tool_mpos[i].at(1);
            tool[i + 1].mpos[Z_AXIS] = _tool_mpos[i].at(2);
        }

        _atc_ok = true;
//ATC^
        config_message();
        log_info("Completed PWN ATC Init");
    }

    void IRAM_ATTR PWM::setSpeedfromISR(uint32_t dev_speed) {
        set_enable(gc_state.modal.spindle != SpindleState::Disable);
        set_output(dev_speed);
    }

    // XXX this is the same as OnOff::setState so it might be possible to combine them
    void PWM::setState(SpindleState state, SpindleSpeed speed) {
        if (sys.abort) {
            return;  // Block during abort.
        }

        if (!_output_pin.defined()) {
            log_warn(name() << " spindle output_pin not defined");
        }

        // We always use mapSpeed() with the unmodified input speed so it sets
        // sys.spindle_speed correctly.
        uint32_t dev_speed = mapSpeed(speed);
        if (state == SpindleState::Disable) {  // Halt or set spindle direction and speed.
            if (_zero_speed_with_disable) {
                dev_speed = offSpeed();
            }
        } else {
            // XXX this could wreak havoc if the direction is changed without first
            // spinning down.
            set_direction(state == SpindleState::Cw);
        }

        // rate adjusted spindles (laser) in M4 set power via the stepper engine, not here

        // set_output must go first because of the way enable is used for level
        // converters on some boards.

        if (isRateAdjusted() && (state == SpindleState::Ccw)) {
            dev_speed = offSpeed();
            set_output(dev_speed);
        } else {
            set_output(dev_speed);
        }

        set_enable(state != SpindleState::Disable);
        spindleDelay(state, speed);
    }

    // prints the startup message of the spindle config
    void PWM::config_message() {
        log_info(name() << " Spindle Ena:" << _enable_pin.name() << " Out:" << _output_pin.name() << " Dir:" << _direction_pin.name()
                        << " Freq:" << _pwm->frequency() << "Hz Period:" << _pwm->period()

        );
    }

    void IRAM_ATTR PWM::set_output(uint32_t duty) {
        if (!_pwm) {
            return;
        }

        // to prevent excessive calls to pwmSetDuty, make sure duty has changed
        if (duty == _current_pwm_duty) {
            return;
        }

        _current_pwm_duty = duty;
        _pwm->setDuty(duty);
    }

    void PWM::deinit() {
        stop();
        if (_pwm) {
            delete _pwm;
            _pwm = nullptr;
        }
        _output_pin.setAttr(Pin::Attr::Input);
        _enable_pin.setAttr(Pin::Attr::Input);
        _direction_pin.setAttr(Pin::Attr::Input);
    }


//ATC Functions:
    bool PWM::tool_change(uint8_t new_tool, bool pre_select) {
        log_debug(name() << " tool change to:" << new_tool << " From:" << current_tool << " Preselect:" << pre_select);
        bool  spindle_was_on         = false;
        bool  was_incremental_mode   = false;  // started in G91 mode
        float saved_mpos[MAX_N_AXIS] = {};     // the position before the tool change

        if (pre_select) {
            log_error("Tool preselect:" << new_tool);
            return true;
        }

        if (!is_ATC_ok())
            return false;

        if (new_tool > MANUAL_CHG) {
            log_error(name() << ":invalid tool number:" << new_tool);
            return false;
        }

        if (new_tool == current_tool) {
            if (current_tool == MANUAL_CHG) {
                log_info("ATC: Manual Change...");
                set_ATC_open(true);
                //gc_exec_linef(true, Uart0, "G4P10");
                gc_exec_linef(true, Uart0, "M0"); //wait until maual change done
                set_ATC_open(false);
                current_tool = 0;
            }
            return true;
        }

        protocol_buffer_synchronize();  // wait for all previous moves to complete
                                        //motor_steps_to_mpos(saved_mpos, motor_steps);
        motor_steps_to_mpos(saved_mpos, get_motor_steps());

        // see if we need to switch out of incremental (G91) mode
        if (gc_state.modal.distance == Distance::Incremental) {
            gc_exec_linef(false, Uart0, "G90");
            was_incremental_mode = true;
        }

        goto_top_of_z();

        // is spindle on? Turn it off and determine when the spin down should be done.
        if (gc_state.modal.spindle != SpindleState::Disable) {
            spindle_was_on = true;
            gc_exec_linef(true, Uart0, "M5");  // this should add a delay if there is one
            if (spindle->_spindown_ms == 0) {
                vTaskDelay(10000);  // long delay for safety and to prevent ATC damage
            }
        }

        // ============= Start of tool change ====================

        if (current_tool == 0 && new_tool == MANUAL_CHG) {
            log_info("Grab manual tool change");
            // open...pause...close
            set_ATC_open(true);
            log_info("ATC: Manual Change...");
            //gc_exec_linef(true, Uart0, "G4P2");
            gc_exec_linef(true, Uart0, "M0"); //wait for manual change
            set_ATC_open(false);
            current_tool = 5;
            return true;
        }

        if (current_tool == MANUAL_CHG) {
            log_info("Drop manual tool change...done");
            set_ATC_open(true);
            //gc_exec_linef(true, Uart0, "G4P0.5");
            gc_exec_linef(true, Uart0, "M0"); //wait for manual change
            set_ATC_open(false);
            if (new_tool == 0) {
                current_tool = 0;
                return true;
            }
        }

        // return the current tool if there is one.
        if (!return_tool(current_tool)) {  // does nothing if we have no tool
            gc_exec_linef(true, Uart0, "G53 G0 X%0.3f Y%0.3f", tool[new_tool].mpos[X_AXIS], tool[new_tool].mpos[Y_AXIS]);
        }

        current_tool = 0;  // now we have no tool

        if (new_tool == 0) {  // if changing to tool 0...we are done.
            gc_exec_linef(true, Uart0, "G53 G0 Y%0.3f", tool[0].mpos[Y_AXIS] - RACK_SAFE_DIST_Y);
            current_tool = new_tool;
            return true;
        }

        if (new_tool == MANUAL_CHG) {
            // manual tool change
            log_info("Grab manual tool");
            set_ATC_open(true);
            //gc_exec_linef(true, Uart0, "G4P2");
            gc_exec_linef(true, Uart0, "M0"); //wait for manual change
            set_ATC_open(false);
            return true;
        }

        log_info("ATC: Auto Change...");
        log_info(new_tool);

        go_above_tool(new_tool);

        set_ATC_open(true);                                                        // open ATC
        gc_exec_linef(true, Uart0, "G53 G0 Z%0.3f", tool[new_tool].mpos[Z_AXIS]);  // drop down to tool
        set_ATC_open(false);                                                       // Close ATC
        gc_exec_linef(true, Uart0, "G4 P%0.2f", TOOL_GRAB_TIME);                   // wait for grab to complete and settle
        goto_top_of_z();

        current_tool = new_tool;

        log_info("ATC: Probe Tool Height...");

        if (!atc_toolsetter()) {  // check the length of the tool
            return false;
        }

        // ================== return old states ===================

        // If the spindle was on before we started, we need to turn it back on.
        if (spindle_was_on) {
            gc_exec_linef(false, Uart0, "M3");  // spindle should handle spinup delay
        }

        // return to saved mpos in XY
        gc_exec_linef(false, Uart0, "G53 G0 X%0.3f Y%0.3f Z%0.3f", saved_mpos[X_AXIS], saved_mpos[Y_AXIS], top_of_z);

        // return to saved mpos in Z if it is not outside of work area.
        gc_exec_linef(false, Uart0, "G53 G0 Z%0.3f", saved_mpos[Z_AXIS] + gc_state.tool_length_offset);

        // was was_incremental on? If so, return to that state
        if (was_incremental_mode) {
            gc_exec_linef(false, Uart0, "G91");
        }

        return true;
    }

bool PWM::return_tool(uint8_t tool_num) {
        log_debug("Return tool: " << tool_num);
        if (tool_num == 0) {
            return false;
        }

        if (tool_num == TOOL_COUNT + 1)
            return true;

        go_above_tool(tool_num);
        gc_exec_linef(true, Uart0, "G53 G0 Z%0.3f", tool[tool_num].mpos[Z_AXIS]);  // drop down to tool
        set_ATC_open(true);
        gc_exec_linef(true, Uart0, "G53 G0 Z%0.3f", _empty_safe_z);
        set_ATC_open(false);  // close ATC

        return true;
    }

    void PWM::go_above_tool(uint8_t tool_num) {
        if (current_tool != 0)
            goto_top_of_z();
        else
            gc_exec_linef(false, Uart0, "G53 G0 Z%0.3f", _empty_safe_z);

        if (current_tool != 0) {
            // move in front of tool
            gc_exec_linef(false, Uart0, "G53 G0 X%0.3f Y%0.3f", tool[tool_num].mpos[X_AXIS], tool[tool_num].mpos[Y_AXIS] - RACK_SAFE_DIST_Y);
        }
        // Move over tool
        gc_exec_linef(true, Uart0, "G53 G0 X%0.3f Y%0.3f", tool[tool_num].mpos[X_AXIS], tool[tool_num].mpos[Y_AXIS]);
    }

    bool PWM::set_ATC_open(bool open) {
        if (gc_state.modal.spindle != SpindleState::Disable) {
            return false;
        }
        _atc_valve_pin.synchronousWrite(open);

        if (open) {
            log_info("ATC: Tool Release...");
        } else {
            log_info("ATC: Tool Clamp...");
        }

        return true;
    }

    bool PWM::atc_toolsetter() {
        float probe_to;  // Calculated work position
        float probe_position[MAX_N_AXIS];

        if (current_tool == 1) {
            // we can go straight to the ATC because tool 1 is next to the toolsetter
            gc_exec_linef(true, Uart0, "G53 G0 X%0.3f Y%0.3f", tool[ETS_INDEX].mpos[X_AXIS], tool[ETS_INDEX].mpos[Y_AXIS]);  // Move over tool
        } else {
            gc_exec_linef(false, Uart0, "G91");
            // Arc out of current tool
            gc_exec_linef(false, Uart0, "G2 X-%0.3f Y-%0.3f I-%0.3f F4000", RACK_SAFE_DIST_Y, RACK_SAFE_DIST_Y, RACK_SAFE_DIST_Y);

            // Move it to arc start
            gc_exec_linef(false,
                          Uart0,
                          "G53 G0X%0.3f Y%0.3f",
                          tool[ETS_INDEX].mpos[X_AXIS] + RACK_SAFE_DIST_Y,
                          tool[ETS_INDEX].mpos[Y_AXIS] - RACK_SAFE_DIST_Y);

            // arc in
            gc_exec_linef(false, Uart0, "G2 X-%0.3f Y%0.3f J%0.3f F4000", RACK_SAFE_DIST_Y, RACK_SAFE_DIST_Y, RACK_SAFE_DIST_Y);
            gc_exec_linef(false, Uart0, "G90");
            // Move over tool
            gc_exec_linef(true, Uart0, "G53 G0 X%0.3f Y%0.3f", tool[ETS_INDEX].mpos[X_AXIS], tool[ETS_INDEX].mpos[Y_AXIS]);
        }

        //atc_ETS_dustoff();

        float wco = gc_state.coord_system[Z_AXIS] + gc_state.coord_offset[Z_AXIS] + gc_state.tool_length_offset;
        probe_to  = tool[ETS_INDEX].mpos[Z_AXIS] - wco;

        // https://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G38-probe
        tool_setter_probing = true;
        gc_exec_linef(true, Uart0, "G38.2 F%0.3f Z%0.3f", PROBE_FEEDRATE, probe_to);  // probe
        tool_setter_probing = false;

        // Was probe successful?
        if (sys.state == State::Alarm) {
            if (rtAlarm == ExecAlarm::ProbeFailInitial) {
                log_info("ATC Probe Switch Error");
            } else {
                log_info("ATC Missing Tool");
            }
            return false;  // fail
        }

        motor_steps_to_mpos(probe_position, probe_steps);
        tool[current_tool].offset[Z_AXIS] = probe_position[Z_AXIS];  // Get the Z height ...

        if (zeroed_tool_index != 0) {
            float tlo = tool[current_tool].offset[Z_AXIS] - tool[zeroed_tool_index].offset[Z_AXIS];
            log_info("ATC Tool No:" << current_tool << " TLO:" << tlo);
            // https://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G43_1
            gc_exec_linef(false, Uart0, "G43.1 Z%0.3f", tlo);  // raise up
        }

        goto_top_of_z();
        // move forward
        gc_exec_linef(false, Uart0, "G53 G0 X%0.3f Y%0.3f", tool[ETS_INDEX].mpos[X_AXIS], tool[ETS_INDEX].mpos[Y_AXIS] - RACK_SAFE_DIST_Y);

        return true;
    }

    bool PWM::is_ATC_ok() {
        if (!_atc_ok) {
            log_warn("ATC failed to initialize");
            return false;
        }
        return true;
    }

    void PWM::goto_top_of_z() {
        gc_exec_linef(true, Uart0, "G53 G0 Z%0.3f", top_of_z);  // Go to top of Z travel
    }

    void PWM::probe_notification() {
        if (sys.state == State::Alarm) {
            return;  // probe failed
        }

        if (tool_setter_probing) {
            return;  // ignore these probes. They are handled elsewhere.
        }

        zeroed_tool_index = current_tool;
    }
//ATC^

    // Configuration registration
    namespace {
        SpindleFactory::InstanceBuilder<PWM> registration("PWM");
    }
}