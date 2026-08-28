#include "tb_cxxrtl_io.h"

int main(void) {
    const uint32_t mask = (1u << 2); // gpio[2]

    // Make gpio[2] an output
    gpio_set_dir(mask);

    while (1) {
        // drive high
        gpio_write(mask);
        usleep(25);

        // drive low
        gpio_write(0u);
        usleep(25);
    }

    return 0;
}
