// Copyright (c) 2021 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "../Pin.h"
#include "HandlerBase.h"

#include <vector>

namespace Configuration {
    class Configurable;

    class Validator : public HandlerBase {
        Validator(const Validator&) = delete;
        Validator& operator=(const Validator&) = delete;

        std::vector<const char*> _path;

    protected:
        void        enterSection(const char* name, Configurable* value) override;
        bool        matchesUninitialized(const char* name) override { return false; }
        HandlerType handlerType() override { return HandlerType::Validator; }

    public:
        Validator();

        void item(const char* name, bool& value) override {}
        void item(const char* name, int32_t& value, int32_t minValue, int32_t maxValue) override {}
        void item(const char* name, uint32_t& value, uint32_t minValue, uint32_t maxValue) override {}
        void item(const char* name, float& value, float minValue, float maxValue) override {}
        void item(const char* name, std::vector<speedEntry>& value) override {}
        
        //ATC
        void item(const char* name, std::vector<float>& value) override {}
        
        void item(const char* name, UartData& wordLength, UartParity& parity, UartStop& stopBits) override {}
        void item(const char* name, std::string& value, int minLength, int maxLength) override {}
        void item(const char* name, Pin& value) override {}
        void item(const char* name, IPAddress& value) override {}
        void item(const char* name, int& value, EnumItem* e) override {}
    };
}
