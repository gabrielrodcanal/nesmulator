#include <stdlib.h>
#include <stdio.h>
#include "mapper0.h"
#include <string.h>

unsigned char *cartridge;
unsigned char chr_rom8kB;
unsigned char prg_rom16kB;
unsigned int cartridge_size;

void initCartridge(char *filename) {
    FILE *f_cartridge;
    if((f_cartridge = fopen(filename, "r")) == NULL)
        printf("Cartridge not found\n");

    unsigned char header[HEADER_SIZE_b];
    if((fread(header, HEADER_SIZE_b, 1, f_cartridge)) == 0)
        printf("Unsupported file format\n");

    prg_rom16kB = header[4];
    chr_rom8kB = header[5];
    cartridge_size = 16 * prg_rom16kB * 1024 + 8 * chr_rom8kB * 1024;
    cartridge = (unsigned char*)malloc(sizeof(unsigned char) * (cartridge_size));
    fseek(f_cartridge, HEADER_SIZE_b, SEEK_SET);
    if((fread(cartridge, cartridge_size, 1, f_cartridge) == 0))
        printf("Error reading cartridge\n");
}

void mapCPUMemory(unsigned char *cpummap) {
    memcpy(cpummap + 0x8000, cartridge, 16 * prg_rom16kB * 1024);
    memcpy(cpummap + 0xC000, cartridge, 16 * prg_rom16kB * 1024);
}

