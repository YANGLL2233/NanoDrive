#include "error.h"

uint32_t all_error = ERROR_NONE;

void set_error(uint32_t error)
{
    all_error |= error;
}

void clear_error(uint32_t error)
{
    all_error &= ~error;
}

bool is_error(uint32_t error)
{
    return all_error & error;
}
