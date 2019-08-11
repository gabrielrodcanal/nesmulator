#include "cpu_ppu_interface.h"
#include "components.h"
#include "../ppu/ppu_components.h"


inline void dma_transfer() {
    unsigned short dma_addr = memory[0x4004] << 8;

    for(int i = 0; i < 0xFF; i++)
        oam[i] = memory[dma_addr++];
}