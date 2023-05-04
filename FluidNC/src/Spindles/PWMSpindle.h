// Copyright (c) 2020 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*
	This is a full featured TTL PWM spindle This does not include speed/power
	compensation. Use the Laser class for that.
*/

#include "OnOffSpindle.h"
#include "Driver/PwmPin.h"

#include <cstdint>

const int TOOL_COUNT = 4;
const int MANUAL_CHG = TOOL_COUNT + 1;

namespace Spindles {
    // This adds support for PWM
    class PWM : public OnOff {
    public:
        PWM() = default;

        // PWM(Pin&& output, Pin&& enable, Pin&& direction, uint32_t minRpm, uint32_t maxRpm) :
        //     _min_rpm(minRpm), _max_rpm(maxRpm), _output_pin(std::move(output)), _enable_pin(std::move(enable)),
        //     _direction_pin(std::move(direction)) {}

        PWM(const PWM&)            = delete;
        PWM(PWM&&)                 = delete;
        PWM& operator=(const PWM&) = delete;
        PWM& operator=(PWM&&)      = delete;

        void init() override;

        void setSpeedfromISR(uint32_t dev_speed) override;
        void setState(SpindleState state, SpindleSpeed speed) override;
        void config_message() override;
        void validate() override { Spindle::validate(); }

        //ATC:
        bool tool_change(uint8_t new_tool, bool pre_select) override;
        void probe_notification() override;
        bool is_ATC_ok();
        //ATC^

        void group(Configuration::HandlerBase& handler) override {
            // The APB clock frequency is 80MHz and the maximum divisor
            // is 2^10.  The maximum precision is 2^20. 80MHz/2^(20+10)
            // is 0.075 Hz, or one cycle in 13.4 seconds.  We cannot
            // represent that in an integer so we set the minimum
            // frequency to 1 Hz.  Frequencies of 76 Hz or less use
            // the full 20 bit resolution, 77 to 152 Hz uses 19 bits,
            // 153 to 305 uses 18 bits, ...
            // At the other end, the minimum useful precision is 2^2
            // or 4 levels of control, so the max is 80MHz/2^2 = 20MHz.
            // Those might not be practical for many CNC applications,
            // but the ESP32 hardware can handle them, so we let the
            // user choose.
            handler.item("pwm_hz", _pwm_freq, 1, 20000000);

            //ATC Handlers
            handler.item("atc_valve_pin", _atc_valve_pin);
            handler.item("atc_dustoff_pin", _atc_dustoff_pin);
            handler.item("ets_dustoff_pin", _toolsetter_dustoff);
            handler.item("ets_mpos_mm", _ets_mpos);
            handler.item("tool1_mpos_mm", _tool_mpos[0]);
            handler.item("tool2_mpos_mm", _tool_mpos[1]);
            handler.item("tool3_mpos_mm", _tool_mpos[2]);
            handler.item("tool4_mpos_mm", _tool_mpos[3]);
            handler.item("empty_safe_z", _empty_safe_z);
            //ATC^
            OnOff::group(handler);
        }

        // Name of the configurable. Must match the name registered in the cpp file.
        const char* name() const override { return "PWM"; }

        virtual ~PWM() {}

    protected:
        uint32_t _current_pwm_duty = 0;
        PwmPin*  _pwm              = nullptr;

        // Configurable
        uint32_t _pwm_freq = 5000;

        void         set_output(uint32_t duty) override;
        virtual void deinit();

        //ATC
        bool return_tool(uint8_t tool_num);
        void go_above_tool(uint8_t tool_num);
        bool set_ATC_open(bool open);
        bool atc_toolsetter();
        void goto_top_of_z();

        typedef struct {
            float mpos[MAX_N_AXIS];    // the pickup location in machine coords
            float offset[MAX_N_AXIS];  // TLO from the zero'd tool
        } tool_t;

        const int   ETS_INDEX        = 0;     // electronic tool setter index
        const float TOOL_GRAB_TIME   = 0.25;  // seconds. How long it takes to grab a tool
        const float RACK_SAFE_DIST_Y = 25.0;  // how far in front of rack is safe to move in X
        const float PROBE_FEEDRATE   = 300.0;

        Pin                _atc_valve_pin;
        Pin                _atc_dustoff_pin;
        Pin                _toolsetter_dustoff;
        std::vector<float> _ets_mpos;
        std::vector<float> _tool_mpos[TOOL_COUNT];

        int   zeroed_tool_index   = 1;  // Which tool was zero'd on the work piece
        bool  _atc_ok             = false;
        float top_of_z            = -1.0;   // position of top of Z in mpos, for safe XY travel
        bool  tool_setter_probing = false;  // used to determine if current probe cycle is for the setter
        float _empty_safe_z       = 0;      // machine space location where is it safe to cross over tools when empty

        //float tool_location[TOOL_COUNT][MAX_N_AXIS];

        tool_t tool[TOOL_COUNT + 1];  // 0 is the toolsetter
        //ATC^
    };
}