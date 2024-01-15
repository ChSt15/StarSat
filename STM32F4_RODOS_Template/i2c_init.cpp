# include "i2c_init.hpp"

static bool initialized = false;

HAL_I2C i2c(I2C_IDX2);

void init_i2c()
{
    if (!initialized)
    {
        i2c.init(400000);
        initialized = true;
    }
}