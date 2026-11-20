/* gpio.c - the Raspberry Pi 5 GPIO pins. They live in the RP1 chip; we map its registers
 * into our memory and read/write them directly (no driver, no interrupt delay). Needs root. */
#include <stddef.h>
#include <sys/mman.h>
#include "starsync.h"

#define RP1_BASE  0x1F00000000ULL                   /* where the RP1 chip's registers start */
#define IO_BANK0  0xD0000                           /* pin functions                        */
#define SYS_RIO0  0xE0000                           /* pin levels                           */
#define PADS0     0xF0000                           /* pull-up / pull-down                  */
#define RIO_OUT   0x00                              /* output levels                        */
#define RIO_OE    0x04                              /* output enable (drive the pin)        */
#define RIO_IN    0x08                              /* input levels                         */
#define RIO_SET   0x2000                            /* write 1s here to SET bits            */
#define RIO_CLR   0x3000                            /* write 1s here to CLEAR bits          */
#define PAD_OD    0x80                              /* pad: output disabled                 */
#define PAD_IE    0x40                              /* pad: input enabled                   */
#define PAD_PUE   0x08                              /* pad: pull-up                         */
#define PAD_PDE   0x04                              /* pad: pull-down                       */
static volatile uint32_t *io, *rio, *pads;
/* map(): map 16 KB of RP1 registers at a physical address into our memory (uncached). */
static volatile uint32_t *map(uint64_t addr)
{
    void *p = mmap_device_memory(NULL, 0x4000, PROT_READ | PROT_WRITE | PROT_NOCACHE, 0, addr);
    if (p == MAP_FAILED) return NULL;
    return p;
}
/* gpio_init(): map the three register blocks we need. Returns 0 on success. */
int gpio_init(void)
{
    io = map(RP1_BASE + IO_BANK0); rio = map(RP1_BASE + SYS_RIO0); pads = map(RP1_BASE + PADS0);
    if (io && rio && pads) return 0;
    return -1;
}
/* gpio_setup(): make a pin an input (pull-down, not driven) or an output (driven, low). */
void gpio_setup(int pin, int output)
{
    uint32_t bit = 1u << pin;
    if (output) {
        rio[(RIO_CLR + RIO_OUT) / 4] = bit;         /* level low */
        rio[(RIO_SET + RIO_OE) / 4] = bit;          /* drive the pin */
    } else {
        rio[(RIO_CLR + RIO_OE) / 4] = bit;          /* don't drive: input */
    }
    uint32_t pad = pads[1 + pin];
    pad &= ~(PAD_OD | PAD_PUE | PAD_PDE);           /* output allowed, no pulls yet */
    pad |= PAD_IE;                                  /* input buffer on */
    if (!output) pad |= PAD_PDE;   /* inputs: pull-down */
    pads[1 + pin] = pad;
    volatile uint32_t *ctrl = &io[(8 * pin + 4) / 4];   /* this pin's control register */
    *ctrl = (*ctrl & ~0x3F01Fu) | 5u;               /* function 5 = RIO, no overrides */
}
/* gpio_read_all(): the level of every pin in ONE read (bit n = GPIO n). */
uint32_t gpio_read_all(void) { return rio[RIO_IN / 4]; }
/* gpio_write(): set (level 1) or clear (level 0) all pins in the mask with ONE write,
 * so they change at exactly the same instant (SKY's COMMON pulse needs this). */
void gpio_write(uint32_t pins, int level)
{
    if (level) rio[(RIO_SET + RIO_OUT) / 4] = pins;
    else rio[(RIO_CLR + RIO_OUT) / 4] = pins;
}
