#pragma once

#include <cstdint>
#include <cstring>
#include <cstdio>

class Stream{

public:
    virtual int available() = 0;
    virtual uint8_t read() = 0;
    virtual int write(uint8_t) = 0;
    virtual int write(uint8_t*, uint16_t len) = 0;
    virtual void init() = 0;

/*    void write_int(int data) {
        char buffer[10];
        sprintf(reinterpret_cast<char *>(buffer), "%d\n", data);

        write(reinterpret_cast<uint8_t *>(buffer), strlen(reinterpret_cast<const char *>(buffer)));
    }*/
};