// Copyright (c) 2020 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.
// Needed updates for ATC

#pragma once

#include <cstdint>

#include "../SpindleDatatypes.h"

#include "../Configuration/Configurable.h"
#include "../Configuration/GenericFactory.h"

#include "src/GCode.h"  // MaxToolNumber

// ===============  No floats! ===========================
// ================ NO FLOATS! ==========================

namespace Spindles {
    class Spindle;
    using SpindleList = std::vector<Spindle*>;

    // This is the base class. Do not use this as your spindle
    class Spindle : public Configuration::Configurable {
    public:
        Spindle() = default;

        Spindle(const Spindle&)            = delete;
        Spindle(Spindle&&)                 = delete;
        Spindle& operator=(const Spindle&) = delete;
        Spindle& operator=(Spindle&&)      = delete;

        bool     _defaultedSpeeds;
        uint32_t offSpeed() { return _speeds[0].offset; }
        uint32_t maxSpeed() { return _speeds[_speeds.size() - 1].speed; }
        uint32_t mapSpeed(SpindleSpeed speed);
        void     setupSpeeds(uint32_t max_dev_speed);
        void     shelfSpeeds(SpindleSpeed min, SpindleSpeed max);
        void     linearSpeeds(SpindleSpeed maxSpeed, float maxPercent);

        static void switchSpindle(uint32_t new_tool, SpindleList spindles, Spindle*& spindle);

        void         spindleDelay(SpindleState state, SpindleSpeed speed);
        virtual void init() = 0;  // not in constructor because this also gets called when $$ settings change

        // Used by Protocol.cpp to restore the state during a restart
        virtual void setState(SpindleState state, uint32_t speed) = 0;
        SpindleState get_state() { return _current_state; };
        void         stop() { setState(SpindleState::Disable, 0); }
        virtual void config_message() = 0;
        virtual bool isRateAdjusted();
        virtual bool use_delay_settings() const { return true; }

        virtual void setSpeedfromISR(uint32_t dev_speed) = 0;

        void spinDown() { setState(SpindleState::Disable, 0); }

        bool                  is_reversable;
        volatile SpindleState _current_state = SpindleState::Unknown;
        volatile SpindleSpeed _current_speed = 0;

        // scaler units are ms/rpm * 2^16.
        // The computation is deltaRPM * scaler >> 16
        uint32_t _spinup_ms   = 0;
        uint32_t _spindown_ms = 0;

        int                                    _tool = -1;
        std::vector<float>                     _offset;
        std::vector<Configuration::speedEntry> _speeds;
        
        //ATC
        std::vector<float>                     _cfg_float_test;

        bool _off_on_alarm = false;

        // ATC Stuff ADDED BY RLG from GRBL ATC code
        virtual void atc_init() {}     //
        //virtual void activate();       // can't get these functions to compile
        //virtual void deactivate() {};  // can't get these functions to compile

        // preselect is used to notify the ATC of a pending tool change in case the ATC can prepare
        // for the future M6.  This is done with M61, which is not parsed yet.
        virtual bool tool_change(uint8_t new_tool, bool pre_select) { return true; }
        virtual void probe_notification() {};
        // END ATC STUFF ADDED BY RLG from GRBL ATC code

        // Name is required for the configuration factory to work.
        virtual const char* name() const = 0;

        // Configuration handlers:
        void validate() override {
            // TODO: Validate spinup/spindown delay?
        }

        //ATC ADDtion by RLG
        //protected:
        uint8_t current_tool = 0;

        //virtual void applyOffset(bool activating);

        void afterParse() override;

        void group(Configuration::HandlerBase& handler) override {
            if (use_delay_settings()) {
                handler.item("spinup_ms", _spinup_ms, 0, 60000);
                handler.item("spindown_ms", _spindown_ms, 0, 60000);
            }
            handler.item("tool_num", _tool, 0, MaxToolNumber);
            handler.item("speed_map", _speeds);
            handler.item("vec_float", _cfg_float_test);
            handler.item("off_on_alarm", _off_on_alarm);
        }

        // Virtual base classes require a virtual destructor.
        virtual ~Spindle() {}
    };
    using SpindleFactory = Configuration::GenericFactory<Spindle>;
}
extern Spindles::Spindle* spindle;
