#ifndef HX711_H
#define HX711_H

#include <cstdint>

class HX711 {
public:
    void begin(int dout_pin, int sck_pin);
    bool is_ready() const;
    int32_t read();
    void tare();

private:
    int dout_pin_ = -1;
    int sck_pin_ = -1;
    int32_t offset_ = 0;
};

#endif // HX711_H
