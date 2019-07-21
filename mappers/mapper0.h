#define HEADER_SIZE_b 16

extern unsigned char *cartridge;
extern unsigned char prg_rom16kB;   // number of 16 kB blocks of PRG ROM
extern unsigned char chr_rom8kB;    // number of 8 kB blocks of CHR ROM
extern unsigned int catridge_size;  // size in bytes;

void initCartridge();
void mapCPUMemory(unsigned char *cpummap);