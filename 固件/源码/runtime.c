/* Minimal freestanding memory primitives for compiler-generated calls.
 *
 * Newer GCC releases may lower aggregate initialization or copy loops to
 * memset/memcpy even when firmware is linked without a hosted C library. Keep
 * these implementations local so the bare-metal build is toolchain portable.
 */

#include <stddef.h>


void *memset(void *destination, int value, size_t count)
{
    unsigned char *output = (unsigned char *)destination;
    while (count-- != 0U) *output++ = (unsigned char)value;
    return destination;
}


void *memcpy(void *destination, const void *source, size_t count)
{
    unsigned char *output = (unsigned char *)destination;
    const unsigned char *input = (const unsigned char *)source;
    while (count-- != 0U) *output++ = *input++;
    return destination;
}


void *memmove(void *destination, const void *source, size_t count)
{
    unsigned char *output = (unsigned char *)destination;
    const unsigned char *input = (const unsigned char *)source;

    if (output < input) {
        while (count-- != 0U) *output++ = *input++;
    } else if (output > input) {
        output += count;
        input += count;
        while (count-- != 0U) *--output = *--input;
    }
    return destination;
}
