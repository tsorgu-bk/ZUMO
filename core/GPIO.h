
#pragma once

#include <cstdint>

class GPIO {
        public:
        enum class Mode : uint8_t {
            INPUT,
            OUTPUT,
            TIMER,
        };

        virtual void set() = 0;
        virtual void reset() = 0;
        virtual bool get() = 0;
        virtual void toggle() = 0;
};