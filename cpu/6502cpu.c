/*
 * Copyright (C) 2018 gabriel
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DEBUG_MODE

#include "../mappers/mapper0.h"
#include "components.h"
#include "cpu_ppu_interface.h"
#include <stdio.h>
#include "../ppu/ppu_components.h"
#include "../ppu/ppuregs.h"

int initialisation();
static void inline set_bit(char bit, char * src);
static void inline rst_bit(char bit, char * src);
static unsigned char inline get_bit(char bit, unsigned char * src);
static void inline set_read_flags(char * reg);
static void inline fetch();
static void inline decode();
static void inline brk();
static void inline rti();
static void inline rts();
static void inline push(char reg);
static void inline pull(char reg);
static void inline jsr();
static void inline impl_addr();
static unsigned short inline abs_addr();
static unsigned char inline zero_page();
static unsigned char inline zero_page_ind(boolean X_reg);
static unsigned short inline abs_ind_addr(boolean X_reg);
static unsigned short inline index_indir();
static unsigned short inline indir_index();
static void inline rel_addr(boolean cond_met);
static unsigned char inline imm_addr();
static void inline jmp();
static void inline lda();
static void inline ldx();
static void inline ldy();
static void inline eor();
static void inline and();
static void inline ora();
static void inline adc();
static void inline sbc();
static void inline cmp();
static void inline cpy();
static void inline cpx();
static void inline bit();
static void inline nop();
static void inline asl();
static void inline lsr();
static void inline rol();
static void inline ror();
static void inline inc();
static void inline dec();
static void inline sta();
static void inline stx();
static void inline sty();
static void inline slo();
static void inline rla();
static void inline stp();
static void inline anc();

// Functions for memory/IO buses treatment
static void inline write_acum(unsigned short addr, unsigned char *reg);
static void inline write_acum_2007(unsigned short addr, unsigned char *reg);
static void inline write_acum_2006(unsigned short addr, unsigned char *reg);
static void inline write_acum_4014(unsigned short addr, unsigned char *reg);
static void inline rot_right(unsigned short addr);
static void inline rot_right_2006(unsigned short addr);
static void inline rot_right_2007(unsigned short addr);
static void inline rot_right_4014(unsigned short addr);
static void inline rot_left(unsigned short addr);
static void inline rot_left_2006(unsigned short addr);
static void inline rot_left_2007(unsigned short addr);
static void inline rot_left_4014(unsigned short addr);
static void inline inc_mem(unsigned short addr);
static void inline inc_mem_2006(unsigned short addr);
static void inline inc_mem_2007(unsigned short addr);
static void inline inc_mem_4014(unsigned short addr);
static void inline dec_mem(unsigned short addr);
static void inline dec_mem_2006(unsigned short addr);
static void inline dec_mem_2007(unsigned short addr);
static void inline dec_mem_4014(unsigned short addr);
static void inline ppu_2006();
static void inline ppu_2007();

unsigned char prev_2006;
unsigned char prev_2007;
unsigned char prev_4014;

unsigned char break_flag;

// Functions that enable efficient handling of writes to PPU memory.
void inline ppu_2006() {
    VRAM_ADDR = (VRAM_ADDR << 8) | memory[0x2006];
}

void inline ppu_2007() {
    ppu_mmap[VRAM_ADDR] = memory[0x2007];
    if(memory[0x2000] & 0x04 == 0)
        VRAM_ADDR++;
    else
        VRAM_ADDR += 32;
}

void inline write_acum(unsigned short addr, unsigned char *reg) {
    memory[addr] = *reg;
}
void inline write_acum_2007(unsigned short addr, unsigned char *reg) {
    write_acum(addr, reg);
    ppu_2007();
}
void inline write_acum_2006(unsigned short addr, unsigned char *reg) {
    prev_2006 = memory[0x2006];
    write_acum(addr, reg);
    ppu_2006();
}
void inline write_acum_4014(unsigned short addr, unsigned char *reg) {
    write_acum(addr, reg);
    dma_transfer();
}

static void inline rot_right(unsigned short addr) {
    memory[addr] >>= 1;
}
static void inline rot_right_2006(unsigned short addr) {
    rot_right(addr);
    ppu_2006();
}
static void inline rot_right_2007(unsigned short addr) {
    rot_right(addr);
    ppu_2007();
}
static void inline rot_right_4014(unsigned short addr) {
    rot_right(addr);
    dma_transfer();
}
static void inline rot_left(unsigned short addr) {
    memory[addr] <<= 1;
}
static void inline rot_left_2006(unsigned short addr) {
    rot_left(addr);
    ppu_2006();
}
static void inline rot_left_2007(unsigned short addr) {
    rot_left(addr);
    ppu_2007();
}
static void inline rot_left_4014(unsigned short addr) {
    rot_left(addr);
    dma_transfer();
}
static void inline inc_mem(unsigned short addr) {
    memory[addr]++;
}
static void inline inc_mem_2006(unsigned short addr) {
    inc_mem(addr);
    ppu_2006();
}
static void inline inc_mem_2007(unsigned short addr) {
    inc_mem(addr);
    ppu_2007();
}
static void inline inc_mem_4014(unsigned short addr) {
    inc_mem(addr);
    dma_transfer();
}
static void inline dec_mem(unsigned short addr) {
    memory[addr]--;
}
static void inline dec_mem_2006(unsigned short addr) {
    dec_mem(addr);
    ppu_2006();
}
static void inline dec_mem_2007(unsigned short addr) {
    dec_mem(addr);
    ppu_2007();
}
static void inline dec_mem_4014(unsigned short addr) {
    dec_mem(addr);
    dma_transfer();
}


void (*write[0xFFFF])(unsigned short addr, unsigned char *reg);
void (*rr[0xFFFF])(unsigned short addr);
void (*rl[0xFFFF])(unsigned short addr);
void (*inc_m[0xFFFF])(unsigned short addr);
void (*dec_m[0xFFFF])(unsigned short addr);

unsigned int cycle;
unsigned short opcode;
unsigned short PC;

boolean fetched_in_adv = FALSE;
boolean execution = TRUE;

int main() {
    initialisation();
}

int initialisation() {
    //initCartridge("../../resources/roms/donkey_kong.nes");
    initCartridge("../../resources/roms/nestest.nes");
    mapCPUMemory(memory);
    
    PC = 0xC000;    // for nestest (temporary)
    //PC = memory[0xFFFD] << 8 | memory[0xFFFC];  // reset vector
    //printf("PC: %x\n", PC);
    S = 0xFD;
    A = X = Y = 0x0;
    P = 0x24;
    
    //load address to stp opcode on stack
    memory[0x100 + 0xFE] = 0xFE;
    memory[0x100 + 0xFF] = 0x0B;
    memory[0xBFF] = 0x02;

    cycle = 7;
    
    // Donkey Kong
    //memory[0x2002] = 0x80;
    unsigned char prev_4014 = memory[0x4014];    // OAM DMA register previous value
    unsigned char prev_2006 = memory[0x2006];
    unsigned char prev_2007 = memory[0x2007];

    VRAM_ADDR = 0x2000;

    ppu_regs_ini();

    for(int i = 0; i < 0xFFFF; i++) {
        write[i] = write_acum;
        rr[i] = rot_right;
        rl[i] = rot_left;
        inc_m[i] = inc_mem;
        dec_m[i] = dec_mem;
    }

    write[0x2006] = write_acum_2006;
    write[0x2007] = write_acum_2007;
    write[0x4014] = write_acum_4014;
    rr[0x2006] = rot_right_2006;
    rr[0x2007] = rot_right_2007;
    rr[0x4014] = rot_right_4014;
    rl[0x2006] = rot_left_2006;
    rl[0x2007] = rot_right_2007;
    rl[0x4014] = rot_right_4014;
    inc_m[0x2006] = inc_mem_2006;
    inc_m[0x2007] = inc_mem_2007;
    inc_m[0x4014] = inc_mem_4014;
    dec_m[0x2006] = dec_mem_2006;
    dec_m[0x2007] = dec_mem_2007;
    dec_m[0x4014] = dec_mem_4014;


    int bg_access_counter = 0;  // background data

    break_flag = 0;

    while(execution) {
        if(fetched_in_adv == FALSE)
            fetch();
        fetched_in_adv = FALSE;
        decode();
        // temporary
        /* if(memory[0x2006] != prev_2006)
            bg_access_counter++;
        if(bg_access_counter == 50)
            break; */
        /* if(break_flag)
            break; */
    }
    /* for(int i = 0x2000; i < 0x23FF; i++)
        printf("%X ", ppu_mmap[i]);
    printf("\n");
    initPPU(); */
    return 0;
}

void set_bit(char bit, char * src) {
    char mask = 1;
    
    for(int i = 0; i < bit; i++)
        mask <<= 1;
    *src |= mask;
}

void rst_bit(char bit, char * src) {
    char mask = 0xFE;
    
    for(int i = 0; i < bit; i++) {
        mask <<= 1;
        mask += 1;
    }
    
    *src &= mask;
}

unsigned char get_bit(char bit, unsigned char * src) {
    unsigned char mask = 0x1;
    
    for(int i = 0; i < bit; i++)
        mask <<= 1;
    
    return (*src & mask) >> bit;
}

void inline set_read_flags(char * reg) {
    if(*reg == 0)
        set_bit(Z,&P);
    else
        rst_bit(Z,&P);
    if(get_bit(7,reg) == 0)
        rst_bit(N,&P);
    else
        set_bit(N,&P);
}

void inline fetch() {
    // First NMI interrupt in DK
    /* if(cycle == 86968) {
        cycle+=4;
        PC = 0xC85F;
        break_flag = 1;
    } */
    opcode = memory[PC];
    /* if(cycle == 27396)
        memory[0x2002] = 0x80; */
    
#ifdef DEBUG_MODE
    printInstr();
#endif
    PC++;
    cycle++;
}

void inline brk() {
    switch(opcode) {
        case 0x04:
            nop();
            return;
            break;
        case 0x08:
            push(P_reg);
            return;
            break;
        case 0x0C:
            nop();
            return;
            break;
        case 0x10:
            rel_addr(get_bit(N,&P) == 0);
            return;
            break;
        case 0x14:
            nop();
            return;
            break;
        case 0x18:
            rst_bit(C,&P);
            cycle++;
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0x1C:
            nop();
            return;
            break;
    }
    PC++;
    cycle++;
    memory[0x100 + S] = PC >> 8;
    S--;
    cycle++;
    memory[0x100 + S] = PC & 0xFF;
    S--;
    cycle++;
    PC = memory[0xFFFE];
    cycle++;
    PC |= memory[0xFFFF] << 8;
    cycle++;
    fetch();
    fetched_in_adv = TRUE;
}

void inline rti() {
    switch(opcode) {
        case 0x44:
            nop();
            return;
            break;
        case 0x48:
            push(A_reg);
            return;
            break;
        case 0x4C:
            jmp();
            return;
            break;
        case 0x50:
            rel_addr(get_bit(V,&P) == 0);
            return;
            break;
        case 0x54:
            nop();
            return;
            break;
        case 0x58:
            rst_bit(I,&P);
            cycle++;
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0x5C:
            nop();
            return;
            break;
    }
    PC++;
    cycle++;
    S++;
    cycle++;
    P = (memory[0x100+S] | 0x20) & 0xEF;
    S++;
    cycle++;
    PC = memory[0x100+S];
    S++;
    cycle++;
    PC |= memory[0x100+S] << 8;
    cycle++;
    fetch();
    fetched_in_adv = TRUE;
}

void rts() {
    switch(opcode) {
        case 0x64:
            nop();
            return;
            break;
        case 0x68:
            pull(A_reg);
            return;
            break;
        case 0x6C:
            jmp();
            return;
            break;
        case 0x70:
            rel_addr(get_bit(V,&P) == 1);
            return;
            break;
        case 0x74:
            nop();
            return;
            break;
        case 0x78:
            set_bit(I,&P);
            cycle++;
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0x7C:
            nop();
            return;
            break;
    }
    PC++;
    cycle++;
    S++;
    cycle++;
    PC = memory[0x100+S];
    S++;
    cycle++;
    PC |= memory[0x100+S] << 8;
    cycle++;
    PC++;
    cycle++;
    fetch();
    fetched_in_adv = TRUE;
}

void inline push(char reg) {
    if(reg == A_reg)
        memory[0x100+S] = A;
    else
        memory[0x100+S] = P | 0x30;
    cycle++;
    S--;
    fetched_in_adv = FALSE;
    cycle++;
}

void inline pull(char reg) {
    cycle++; //read next instruction byte and throw it away
    S++;
    cycle++;
    if(reg == A_reg) {
        A = memory[0x100+S];
        set_read_flags(&A);
    }
    else
        P = (memory[0x100+S] | 0x20) & 0xEF;  //bit 5 is always set, bit 4 (flag B) is cleared
    cycle++;
    fetch();
    fetched_in_adv = TRUE;
}

void inline jsr() {
    switch(opcode) {
        case 0x24:
            bit();
            return;
            break;
        case 0x28:
            pull(P_reg);
            return;
            break;
        case 0x2C:
            bit();
            return;
            break;
        case 0x30:
            rel_addr(get_bit(N,&P) == 1);
            return;
            break;
        case 0x34:
            nop();
            return;
            break;
        case 0x38:
            set_bit(C,&P);
            cycle++;
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0x3C:
            nop();
            return;
            break;
        default:
            break;
    }
    unsigned char pcl = memory[PC];
    PC++;
    cycle++;
    cycle++;    //internal operation
    memory[0x100+S] = (PC & 0xFF00) >> 8;
    S--;
    cycle++;
    memory[0x100+S] = PC & 0xFF;
    S--;
    cycle++;
    PC = (memory[PC] << 8) | pcl; 
    cycle++;
    fetched_in_adv = FALSE;
}


// ADDRESSING
void inline impl_addr() {
    cycle++;
}

unsigned short inline abs_addr() {        
    unsigned char low = memory[PC];
    PC++;
    cycle++;
    unsigned char high = memory[PC];
    PC++;
    cycle++;
    
    return high << 8 | low;
}

unsigned char inline zero_page() {
    unsigned char addr = memory[PC];
    PC++;
    cycle++;
    
    return addr;
}

unsigned char inline zero_page_ind(boolean X_reg) {
    unsigned char index;
    if(X_reg == TRUE)
        index = X;
    else 
        index = Y;
    unsigned char addr = memory[PC];
    PC++;
    cycle++;
    addr = (addr + index) % 256;
    cycle++;
    return addr;
}

unsigned short inline abs_ind_addr(boolean X_reg) {
    unsigned char index;
    unsigned char check_rmw_w = opcode & 0xF;
    boolean extra_cycle = FALSE;
    if(X_reg == TRUE) {
        index = X;
        if(check_rmw_w == 0xF || check_rmw_w == 0xE || opcode == 0x9D || opcode == 0x9C) {    //extra cycle always reserved in RMW and W instr.
            cycle++;
            extra_cycle = TRUE;
        }
    }
    else  {
        index = Y;
        if(check_rmw_w != 0x9 && check_rmw_w != 0xF && opcode != 0xBE && opcode != 0xBF || opcode == 0x99) {
            cycle++;
            extra_cycle = TRUE;
        }
    }
    unsigned char low = memory[PC];
    PC++;
    cycle++;
    unsigned char high = memory[PC];
    unsigned char old_low = low;
    low += index;
    PC++;
    cycle++;
    
    unsigned short addr;
    if(0xFF - old_low < index) {
        addr = (high + 1) << 8 | low;
        if(extra_cycle == FALSE)
            cycle++;
    }
    else
        addr = high << 8 | low;
    
    return addr;
}

unsigned short index_indir() {
    unsigned char point_addr = memory[PC];
    PC++;
    cycle++;
    point_addr = (point_addr + X) % 256;
    cycle++;
    unsigned short addr = memory[point_addr];
    cycle++;
    addr |= memory[(point_addr + 1) % 256] << 8;
    cycle++;
    return addr;
}

unsigned short indir_index() {
    unsigned char point_addr = memory[PC];
    PC++;
    cycle++;
    unsigned char low = memory[point_addr];
    cycle++;
    unsigned char high = memory[(point_addr + 1) % 256];
    unsigned char old_low = low;
    low += Y;
    cycle++;
    unsigned short addr;
    if(0xFF - old_low < Y) {
        addr = (high + 1) << 8 | low;
        cycle++;
    }
    else if((opcode & 0xF == 0x3) || (opcode == 0x91)) {    //RMW or W instructions
        cycle++;
        addr = high << 8 | low;
    }
    else
        addr = high << 8 | low;
    
    return addr;    
}

void inline rel_addr(boolean cond_met) {
    signed char offset = memory[PC];
    PC++;
    cycle++;
    opcode = memory[PC];
    if(cond_met == TRUE) {
        unsigned short pch = PC & 0xFF00;
        unsigned char pcl = PC & 0xFF;
        unsigned char old_pcl = pcl;
        pcl += offset;
        cycle++;
        opcode = memory[PC];
        if(0xFF - old_pcl < offset) {
            PC = (pch + 0x0100 | pcl) % 0x10000;
            cycle++;
            fetch();
        }
        else {
            PC = (pch | pcl) % 0x10000;
            fetch();
        }
    }
    else {
#ifdef DEBUG_MODE
        printInstr();
#endif
        PC ++;
        cycle++;
    }
    fetched_in_adv = TRUE;
}

unsigned char imm_addr() {
    unsigned char addr = memory[PC];
    PC++;
    cycle++;
    return addr;
}

void inline jmp() {
    unsigned char pcl, point_low, point_high;
    switch(opcode) {
        case 0x4C:
            pcl = memory[PC];
            PC++;
            cycle++;
            PC = (memory[PC] << 8) | pcl;
            fetched_in_adv = FALSE;
            cycle++;
            break;
        case 0x6C:
            point_low = memory[PC];
            PC++;
            cycle++;
            point_high = memory[PC];
            PC++;
            cycle++;
            pcl = memory[(point_high << 8) | point_low];
            cycle++;
            PC = memory[((point_high << 8) | (point_low + 1) % 256) % 0x10000] << 8 | pcl;
            fetched_in_adv = FALSE;
            cycle++;
            break;
    }
}

//READ

void inline lda() {
    switch(opcode) {
        case 0xA9:
            A = imm_addr();
            break;
        case 0xA5:
            A = memory[zero_page()];
            cycle++;
            break;
        case 0xB5:
            A = memory[zero_page_ind(TRUE)];
            cycle++;
            break;
        case 0xAD:
            A = memory[abs_addr()];
            cycle++;
            break;
        case 0xBD:
            A = memory[abs_ind_addr(TRUE)];
            cycle++;
            break;
        case 0xB9:
            A = memory[abs_ind_addr(FALSE)];
            cycle++;
            break;
        case 0xA1:
            A = memory[index_indir()];
            cycle++;
            break;
        case 0xB1:
            A = memory[indir_index()];
            cycle++;
            break;
    }
    
    set_read_flags(&A);
    fetch();
    fetched_in_adv = TRUE;
}

void inline ldx() {
    switch(opcode) {
        case 0xA2:
            X = imm_addr();
            break;
        case 0xAA:
            X = A;
            set_read_flags(&X);
            cycle++;
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0xA6:
            X = memory[zero_page()];
            cycle++;
            break;
        case 0xB6:
            X = memory[zero_page_ind(FALSE)];
            cycle++;
            break;
        case 0xAE:
            X = memory[abs_addr()];
            cycle++;
            break;
        case 0xB2:
            stp();
            return;
            break;
        case 0xBA:
            X = S;
            set_read_flags(&X);
            cycle++;
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0xBE:
            X = memory[abs_ind_addr(FALSE)];
            cycle++;
            break;
    }
    
    set_read_flags(&X);
    fetch();
    fetched_in_adv = TRUE;
}

void inline ldy() {
    switch(opcode) {
        case 0xA0:
            Y = imm_addr();
            break;
        case 0xA4:
            Y = memory[zero_page()];
            cycle++;
            break;
        case 0xA8:
            Y = A;
            set_read_flags(&Y);
            cycle++;
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0xAC:
            Y = memory[abs_addr()];
            cycle++;
            break;
        case 0xB0:
            rel_addr(get_bit(C,&P) == 1);
            return;
            break;
        case 0xB4:
            Y = memory[zero_page_ind(TRUE)];
            cycle++;
            break;
        case 0xB8:
            rst_bit(V,&P);
            cycle++;
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0xBC:
            Y = memory[abs_ind_addr(TRUE)];
            cycle++;
            break;
    }
                
    set_read_flags(&Y);
    fetch();
    fetched_in_adv = TRUE;
}

void inline eor() {
    switch(opcode) {
        case 0x49:
            A ^= imm_addr();
            break;
        case 0x45:
            A ^= memory[zero_page()];
            cycle++;
            break;
        case 0x55:
            A ^= memory[zero_page_ind(TRUE)];
            cycle++;
            break;
        case 0x4D:
            A ^= memory[abs_addr()];
            cycle++;
            break;
        case 0x5D:
            A ^= memory[abs_ind_addr(TRUE)];
            cycle++;
            break;
        case 0x59:
            A ^= memory[abs_ind_addr(FALSE)];
            cycle++;
            break;
        case 0x41:
            A ^= memory[index_indir()];
            cycle++;
            break;
        case 0x51:
            A ^= memory[indir_index()];
            cycle++;
            break;
    }
    
    set_read_flags(&A);
    fetch();
    fetched_in_adv = TRUE;
}

void inline and() {
    switch(opcode) {
        case 0x29:
            A &= imm_addr();
            break;
        case 0x25:
            A &= memory[zero_page()];
            cycle++;
            break;
        case 0x35:
            A &= memory[zero_page_ind(TRUE)];
            cycle++;
            break;
        case 0x2D:
            A &= memory[abs_addr()];
            cycle++;
            break;
        case 0x3D:
            A &= memory[abs_ind_addr(TRUE)];
            cycle++;
            break;
        case 0x39:
            A &= memory[abs_ind_addr(FALSE)];
            cycle++;
            break;
        case 0x21:
            A &= memory[index_indir()];
            cycle++;
            break;
        case 0x31:
            A &= memory[indir_index()];
            cycle++;
            break;
    }
    
    set_read_flags(&A);
    fetch();
    fetched_in_adv = TRUE;
}

void inline ora() {
    switch(opcode) {
        case 0x09:
            A |= imm_addr();
            break;
        case 0x05:
            A |= memory[zero_page()];
            cycle++;
            break;
        case 0x15:
            A |= memory[zero_page_ind(TRUE)];
            cycle++;
            break;
        case 0x0D:
            A |= memory[abs_addr()];
            cycle++;
            break;
        case 0x1D:
            A |= memory[abs_ind_addr(TRUE)];
            cycle++;
            break;
        case 0x19:
            A |= memory[abs_ind_addr(FALSE)];
            cycle++;
            break;
        case 0x01:
            A |= memory[index_indir()];
            cycle++;
            break;
        case 0x11:
            A |= memory[indir_index()];
            cycle++;
            break;
    }
    
    set_read_flags(&A);
    fetch();
    fetched_in_adv = TRUE;
}

void inline adc() {    
    unsigned char val;
    switch(opcode) {
        case 0x69:
            val = imm_addr();
            break;
        case 0x65:
            val = memory[zero_page()];
            cycle++;
            break;
        case 0x75:
            val = memory[zero_page_ind(TRUE)];
            cycle++;
            break;
        case 0x6D:
            val = memory[abs_addr()];
            cycle++;
            break;
        case 0x7D:
            val = memory[abs_ind_addr(TRUE)];
            cycle++;
            break;
        case 0x79:
            val = memory[abs_ind_addr(FALSE)];
            cycle++;
            break;
        case 0x61:
            val = memory[index_indir()];
            cycle++;
            break;
        case 0x71:
            val = memory[indir_index()];
            cycle++;
            break;    
    }
    
    unsigned char A_old = A;
    unsigned char old_C = get_bit(C,&P);
    A += old_C + val;
    
    if(0xFF - A_old < val + old_C)
        set_bit(C,&P);
    else
        rst_bit(C,&P);
    
    if((A_old ^ A) & (val ^ A) & 0x80)
        set_bit(V,&P);
    else
        rst_bit(V,&P);
    
    set_read_flags(&A);
    
    fetch();
    fetched_in_adv = TRUE;
}

void inline sbc() {
    unsigned char val;
    switch(opcode) {
        case 0xE9:
            val = imm_addr();
            break;
        case 0xE5:
            val = memory[zero_page()];
            cycle++;
            break;
        case 0xF5:
            val = memory[zero_page_ind(TRUE)];
            cycle++;
            break;
        case 0xED:
            val = memory[abs_addr()];
            cycle++;
            break;
        case 0xFD:
            val = memory[abs_ind_addr(TRUE)];
            cycle++;
            break;
        case 0xF9:
            val = memory[abs_ind_addr(FALSE)];
            cycle++;
            break;
        case 0xE1:
            val = memory[index_indir()];
            cycle++;
            break;
        case 0xEB:
            val = imm_addr();
            break;
        case 0xF1:
            val = memory[indir_index()];
            cycle++;
            break;      
    }
    unsigned char A_old = A;
    unsigned char old_C = get_bit(C,&P);
    unsigned char comp_val = val ^ 0xFF;
    A += old_C + comp_val;
    
    if(0xFF - A_old < comp_val + old_C)
        set_bit(C,&P);
    else
        rst_bit(C,&P);
    
    if((A_old ^ A) & (comp_val ^ A) & 0x80)
        set_bit(V,&P);
    else
        rst_bit(V,&P);
    
    set_read_flags(&A);
    
    fetch();
    fetched_in_adv = TRUE;
}

void inline cmp() {
    unsigned char val;
    switch(opcode) {
        case 0xC9:
            val = imm_addr();
            break;
        case 0xC5:
            val = memory[zero_page()];
            cycle++;
            break;
        case 0xD5:
            val = memory[zero_page_ind(TRUE)];
            cycle++;
            break;
        case 0xCD:
            val = memory[abs_addr()];
            cycle++;
            break;
        case 0xDD:
            val = memory[abs_ind_addr(TRUE)];
            cycle++;
            break;
        case 0xD9:
            val = memory[abs_ind_addr(FALSE)];
            cycle++;
            break;
        case 0xC1:
            val = memory[index_indir()];
            cycle++;
            break;
        case 0xD1:
            val = memory[indir_index()];
            cycle++;
            break;
    }
    char check = A - val;  
    if(val <= A)
        set_bit(C,&P);
    else
        rst_bit(C,&P);
    
    set_read_flags(&check);
    fetch();
    fetched_in_adv = TRUE;    
}

void inline cpy() {
    unsigned char val;
    switch(opcode) {
        case 0xC0:
            val = imm_addr();
            break;
        case 0xC4:
            val = memory[zero_page()];
            cycle++;
            break;
        case 0xC8:
            Y++;
            set_read_flags(&Y);
            cycle++;
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0xCC:
            val = memory[abs_addr()];
            cycle++;
            break;            
        case 0xD0:
            rel_addr(get_bit(Z,&P) == 0);
            return;
            break;
        case 0xD4:
            nop();
            return;
            break;
        case 0xD8:
            rst_bit(D,&P);
            cycle++;
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0xDC:
            nop();
            return;
            break;
    }
    unsigned char check = Y - val;
    if(Y < val)
        rst_bit(C,&P);
    else
        set_bit(C,&P);
    
    set_read_flags(&check);
    
    fetch();
    fetched_in_adv = TRUE;
}

void inline cpx() {
    unsigned char val;
    switch(opcode) {
        case 0xE0:
            val = imm_addr();
            break;
        case 0xE4:
            val = memory[zero_page()];
            cycle++;
            break;
        case 0xE8:
            X++;
            set_read_flags(&X);
            cycle++;
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0xEC:
            val = memory[abs_addr()];
            cycle++;
            break;            
        case 0xF0:
            rel_addr(get_bit(Z,&P) == 1);
            return;
            break;
        case 0xF4:
            nop();
            return;
            break;
        case 0xF8:
            set_bit(D,&P);
            cycle++;
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0xFC:
            nop();
            return;
            break;
    }
    char check = X - val;
    if(X < val)
        rst_bit(C,&P);
    else
        set_bit(C,&P);
    
    set_read_flags(&check);
    
    fetch();
    fetched_in_adv = TRUE;
}

void inline bit() {
    unsigned char res;
    switch(opcode) {
        case 0x24:
            res = memory[zero_page()];
            cycle++;
            break;
        case 0x2C:
            res = memory[abs_addr()];
            cycle++;
            break;
    }    
    
    if(get_bit(7,&res) == 0)
        rst_bit(N,&P);
    else
        set_bit(N,&P);
    if(get_bit(6,&res) == 0)
        rst_bit(V,&P);
    else
        set_bit(V,&P);
    
    res &= A;
    
    if(res == 0)
        set_bit(Z,&P);
    else
        rst_bit(Z,&P);
    
    fetch();
    fetched_in_adv = TRUE;
}

void inline nop() {
    switch(opcode) {
        case 0x04:
            zero_page();
            cycle++;
            break;
        case 0x0C:
            abs_addr();
            cycle++;
            break;
        case 0x14:
            zero_page_ind(TRUE);
            cycle++;
            break;
        case 0x1C:
            abs_ind_addr(TRUE);
            cycle++;
            break;
        case 0x34:
            zero_page_ind(TRUE);
            cycle++;
            break;
        case 0x3C:
            abs_ind_addr(TRUE);
            cycle++;
            break;
        case 0x44:
            zero_page();
            cycle++;
            break;
        case 0x54:
            zero_page_ind(TRUE);
            cycle++;
            break;
        case 0x5C:
            abs_ind_addr(TRUE);
            cycle++;
            break;
        case 0x64:
            zero_page();
            cycle++;
            break;
        case 0x74:
            zero_page_ind(TRUE);
            cycle++;
            break;
        case 0x7C:
            abs_ind_addr(TRUE);
            cycle++;
            break;
        case 0x80:
            imm_addr();
            break;
        case 0xD4:
            zero_page_ind(TRUE);
            cycle++;
            break;
        case 0xDC:
            abs_ind_addr(TRUE);
            cycle++;
            break;
        case 0xF4:
            zero_page_ind(TRUE);
            cycle++;
            break;
        case 0xFC:
            abs_ind_addr(TRUE);
            cycle++;
            break;
        case 0x89:
            imm_addr();
            break;
        case 0xC2:
            imm_addr();
            break;
        case 0xE2:
            imm_addr();
            break;
        default:    //implied
            cycle++;
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;            
    }
    fetch();
    fetched_in_adv = TRUE;
}

// READ-MODIFY-WRITE

void inline asl() {
    unsigned short addr;
    
    switch(opcode) {
        case 0x02:
            stp();
            return;
            break;
        case 0x0A:
            if(get_bit(7,&A) == 1)
                set_bit(C,&P);
            else
                rst_bit(C,&P);
            
            A <<= 1;
            cycle++;
            set_read_flags(&A);
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0x012:
            stp();
            return;
            break;
        case 0x06:
            addr = zero_page();
            cycle++;
            break;
        case 0x16:
            addr = zero_page_ind(TRUE);
            cycle++;
            break;
        case 0x1A:
            nop();
            return;
            break;
        case 0x0E:
            addr = abs_addr();
            cycle++;
            break;
        case 0x1E:
            addr = abs_ind_addr(TRUE);
            cycle++;
            break;
    }
    
    if(get_bit(7,&memory[addr]) == 1)
        set_bit(C,&P);
    else
        rst_bit(C,&P);
    
    (rl[addr])(addr);
    cycle++;
    set_read_flags(&memory[addr]);
    
    cycle++;
}

void inline lsr() {
    unsigned short addr;
    
    switch(opcode) {
        case 0x4A:
            if(get_bit(0,&A) == 1)
                set_bit(C,&P);
            else
                rst_bit(C,&P);
            A >>= 1;
            cycle++;
            set_read_flags(&A);

            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0x5A:
            nop();
            return;
            break;
        case 0x46:
            addr = zero_page();
            cycle++;
            break;
        case 0x56:
            addr = zero_page_ind(TRUE);
            cycle++;
            break;
        case 0x4E:
            addr = abs_addr();
            cycle++;
            break;
        case 0x5E:
            addr = abs_ind_addr(TRUE);
            cycle++;
            break;
    }
    
    if(get_bit(0,&memory[addr]) == 1)
        set_bit(C,&P);
    else
        rst_bit(C,&P);
    
    (rr[addr])(addr);
    cycle++;
    set_read_flags(&memory[addr]);

    cycle++;
}

void inline rol() {
    unsigned char * dest;
    unsigned short addr;
    unsigned char bit7;
    
    switch(opcode) {
        case 0x22:
            stp();
            return;
            break;
        case 0x2A:
            bit7 = A & 0x80;
            A <<= 1;
            cycle++;
            if(get_bit(C,&P) == 0)
                rst_bit(0,&A);
            else
                set_bit(0,&A);
            
            set_read_flags(&A);

            if(bit7 == 0)
                rst_bit(C,&P);
            else
                set_bit(C,&P);
            
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0x26:
            addr = zero_page();
            cycle++;
            break;
        case 0x32:
            stp();
            return;
            break;
        case 0x36:
            addr = zero_page_ind(TRUE);
            cycle++;
            break;
        case 0x3A:
            nop();
            return;
            break;
        case 0x2E:
            addr = abs_addr();
            cycle++;
            break;
        case 0x3E:
            addr = abs_ind_addr(TRUE);
            cycle++;
            break;
    }
    
    bit7 = memory[addr] & 0x80;
    (rl[addr])(addr);
    cycle++;
    if(get_bit(C,&P) == 0)
        rst_bit(0,&memory[addr]);
    else
        set_bit(0,&memory[addr]);
    
    set_read_flags(&memory[addr]);

    if(bit7 == 0)
        rst_bit(C,&P);
    else
        set_bit(C,&P);
    
    cycle++;
}

void inline ror() {
    unsigned short addr;
    unsigned char old_bit0;
    
    switch(opcode) {
        case 0x62:
            stp();
            return;
            break;
        case 0x6A:
            old_bit0 = A & 0x1;
            A >>= 1;
            cycle++;
            
            if(get_bit(C,&P) == 0)
                rst_bit(7,&A);
            else
                set_bit(7,&A);
            
            if(old_bit0 == 1)
                set_bit(C,&P);
            else
                rst_bit(C,&P);
            
            set_read_flags(&A);
            
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0x72:
            stp();
            return;
            break;
        case 0x66:
            addr = zero_page();
            cycle++;
            break;
        case 0x76:
            addr = zero_page_ind(TRUE);
            cycle++;
            break;
        case 0x6E:
            addr = abs_addr();
            cycle++;
            break;
        case 0x7A:
            nop();
            return;
            break;
        case 0x7E:
            addr = abs_ind_addr(TRUE);
            cycle++;
            break;
    }
    
    old_bit0 = memory[addr] & 0x1;
    (rr[addr])(addr);
    cycle++;
    
    if(get_bit(C,&P) == 0)
        rst_bit(7,&memory[addr]);
    else
        set_bit(7,&memory[addr]);
    
    if(old_bit0 == 1)
        set_bit(C,&P);
    else
        rst_bit(C,&P);
    
    set_read_flags(&memory[addr]);
    
    cycle++;
}

void inline inc() {
    unsigned short addr;
    switch(opcode) {
        case 0xE2:
            nop();
            return;
            break;
        case 0xE6:
            addr = zero_page();
            break;
        case 0xEA:
            nop();
            return;
            break;
        case 0xF2:
            stp();
            return;
            break;
        case 0xF6:
            addr = zero_page_ind(TRUE);
            break;
        case 0xFA:
            nop();
            return;
            break;
        case 0xEE:
            addr = abs_addr();
            break;
        case 0xFE:
            addr = abs_ind_addr(TRUE);
            break;
    }
    cycle++;
    (inc_m[addr])(addr);
    cycle++;
    set_read_flags(&memory[addr]);
    fetched_in_adv = FALSE;
    cycle++;
}

void inline dec() {
    unsigned short addr;
    switch(opcode) {
        case 0xC2:
            nop();
            return;
            break;
        case 0xC6:
            addr = zero_page();
            cycle++;
            break;
        case 0xCA:
            X--;
            cycle++;
            
            set_read_flags(&X);
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0xD2:
            stp();
            return;
            break;
        case 0xD6:
            addr = zero_page_ind(TRUE);
            cycle++;
            break;
        case 0xCE:
            addr = abs_addr();
            cycle++;
            break;
        case 0xDA:
            nop();
            return;
            break;
        case 0xDE:
            addr = abs_ind_addr(TRUE);
            cycle++;
            break;
    }
    (dec_m[addr])(addr);
    cycle++;
    
    set_read_flags(&memory[addr]);
    fetched_in_adv = FALSE;
    cycle++;
}

// WRITE

void inline sta() {
    unsigned short addr;
    switch(opcode) {
        case 0x85:
            addr = zero_page();
            (*write[addr])(addr, &A);
            break;
        case 0x89:
            nop();
            return;
            break;
        case 0x95:
            addr = zero_page_ind(TRUE);
            (*write[addr])(addr, &A);
            break;
        case 0x8D:
            addr = abs_addr();
            (*write[addr])(addr, &A);
            break;
        case 0x9D:
            addr = abs_ind_addr(TRUE);
            (*write[addr])(addr, &A);
            break;
        case 0x99:
            addr = abs_ind_addr(FALSE);
            (*write[addr])(addr, &A);
            break;
        case 0x81:
            addr = index_indir();
            (*write[addr])(addr, &A);
            break;
        case 0x91:
            addr = indir_index();
            (*write[addr])(addr, &A);
            break;
    }
    cycle++;
    fetched_in_adv = FALSE;
}

void inline stx() {
    unsigned short addr;
    switch(opcode) {
        case 0x82:
            nop();
            return;
            break;
        case 0x86:
            addr = zero_page();
            (*write[addr])(addr, &X);
            cycle++;
            break;
        case 0x8A:
            A = X;
            set_read_flags(&A);
            cycle++;
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0x92:
            stp();
            return;
            break;
        case 0x96:
            addr = zero_page_ind(FALSE);
            (write[addr])(addr, &X);
            cycle++;
            break;
        case 0x9A:
            S = X;
            cycle++;
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0x8E:
            addr = abs_addr(FALSE);
            (write[addr])(addr, &X);
            cycle++;
            break;
        case 0x9E:
            addr = abs_addr();
            unsigned char stored = X & (addr >> 8) + 1;
            (write[addr])(addr, &stored);
            cycle++;
            return;
            break;
    }
    fetch();
    fetched_in_adv = TRUE;
}

void inline sty() {
    unsigned short addr;
    switch(opcode) {
        case 0x80:
            nop();
            return;
            break;
        case 0x84:
            addr = zero_page();
            (write[addr])(addr, &Y);
            cycle++;
            break;
        case 0x88:
            Y--;
            set_read_flags(&Y);
            cycle++;
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0x8C:
            addr = abs_addr();
            (write[addr])(addr, &Y);
            cycle++;
            break;
        case 0x90:
            rel_addr(get_bit(C,&P) == 0);
            return;
            break;
        case 0x94:
            addr = zero_page_ind(TRUE);
            (write[addr])(addr, &Y);
            cycle++;
            break;
        case 0x98:
            A = Y;
            set_read_flags(&A);
            cycle++;
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0x9C:
            addr = abs_ind_addr(FALSE);
            unsigned char stored = Y & (addr >> 8) + 1;
            (write[addr])(addr, &stored);
            cycle++;
            break;
    }
    fetched_in_adv = FALSE;
    
}

void inline slo() {
    unsigned char * dest;
    unsigned short addr;
    switch(opcode) {
        case 0x07:
            addr = zero_page();
            cycle++;
            break;
        case 0x0B:
            anc();
            return;
            break;
        case 0x17:
            addr = zero_page_ind(TRUE);
            cycle++;
            break;
        case 0x0F:
            addr = abs_addr();
            cycle++;
            break;
        case 0x1F:
            addr = abs_ind_addr(TRUE);
            cycle++;
            break;
        case 0x1B:
            addr = abs_ind_addr(FALSE);
            cycle++;
            break;
        case 0x03:
            addr = index_indir();
            cycle++;
            break;
        case 0x13:
            addr = indir_index();
            cycle++;
            break;
    }
    cycle++;
    if(get_bit(7,&memory[addr]) == 1)
        set_bit(C,&P);
    else
        rst_bit(C,&P);
    
    (rl[addr])(addr);
    cycle++;
    
    A |= memory[addr];
    set_read_flags(&A);
    fetch();
    fetched_in_adv = TRUE;
}

void anc() {
    switch(opcode) {
        case 0x0B:
            A &= imm_addr();
            break;
        case 0x2B:
            A &= imm_addr();
            break;
    }
    fetch();
    fetched_in_adv = TRUE;
    set_read_flags(&A);
    if(A < 0)
        set_bit(C,&P);
    cycle++;
}

void inline rla() {
    unsigned char * dest;
    unsigned short addr;
    switch(opcode) {
        case 0x27:
            addr = zero_page();
            break;
        case 0x2B:
            anc();
            return;
            break;
        case 0x37:
            addr = zero_page_ind(TRUE);
            break;
        case 0x2F:
            addr = abs_addr();
            break;
        case 0x3F:
            addr = abs_ind_addr(TRUE);
            break;
        case 0x3B:
            addr = abs_ind_addr(FALSE);
            break;
        case 0x23:
            addr = index_indir();
            break;
        case 0x33:
            addr = indir_index();
            break;
    }
    cycle++;
    
    unsigned char val = memory[addr];
    unsigned char bit7 = memory[addr] & 0x80;
    (rl[addr])(addr);
    cycle++;
    if(get_bit(C,&P) == 0)
        rst_bit(0,&memory[addr]);
    else
        set_bit(0,&memory[addr]);
    
    set_read_flags(&memory[addr]);
    
    if(bit7 == 0)
        rst_bit(C,&P);
    else
        set_bit(C,&P);
    cycle++;
    A &= memory[addr];
    set_read_flags(&A);
    fetch();
    fetched_in_adv = TRUE;
}

void sre() {
    unsigned char * dest;
    unsigned short addr;
    switch(opcode) {
        case 0x47:
            addr = zero_page();
            break;
        case 0x57:
            addr = zero_page_ind(TRUE);
            break;
        case 0x4F:
            addr = abs_addr();
            break;
        case 0x5F:
            addr = abs_ind_addr(TRUE);
            break;
        case 0x4B:
            A |= imm_addr();
            cycle++;
            if(get_bit(0,&A) == 0)
                rst_bit(C,&P);
            else
                set_bit(C,&P);
            A >>= 1;
            set_read_flags(&A);
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;            
        case 0x5B:
            addr = abs_ind_addr(TRUE);
            break;
        case 0x43:
            addr = index_indir();
            break;
        case 0x53:
            addr = indir_index();
            break;
    }
    cycle++;
    
    if(get_bit(0,&memory[addr]) == 0)
        rst_bit(C,&P);
    else
        set_bit(C,&P);
    cycle++;
    (rr[addr])(addr);
    cycle++;
    A ^= memory[addr];
    set_read_flags(&A);
    fetch();
    fetched_in_adv = TRUE;
}

void rra() {
    unsigned char * dest;
    unsigned short addr;
    switch(opcode) {
        case 0x67:
            addr = zero_page();
            break;
        case 0x77:
            addr = zero_page_ind(TRUE);
            break;
        case 0x6F:
            addr = abs_addr();
            break;
        case 0x7F:
            addr = abs_ind_addr(TRUE);
            break;
        case 0x7B:
            addr = abs_ind_addr(TRUE);
            break;
        case 0x63:
            addr = index_indir();
            break;
        case 0x73:
            addr = indir_index();
            break;
    }
    cycle++;
    
    unsigned char val = memory[addr];
    unsigned char old_bit0 = memory[addr] & 0x1;
    (rr[addr])(addr);
    cycle++;
    if(get_bit(C,&P) == 0)
        rst_bit(7,&memory[addr]);
    else
        set_bit(7,&memory[addr]);
    if(old_bit0 == 0)
        rst_bit(C,&P);
    else
        set_bit(C,&P);
    
    cycle++;
    
    unsigned char A_old = A;
    unsigned char old_C = get_bit(C,&P);
    A += old_C + memory[addr];
    
    if(0xFF - A_old < memory[addr] + old_C)
        set_bit(C,&P);
    else
        rst_bit(C,&P);
    
    if((A_old ^ A) & (memory[addr] ^ A) & 0x80)
        set_bit(V,&P);
    else
        rst_bit(V,&P);
    
    set_read_flags(&A);
    
    fetch();
    fetched_in_adv = TRUE;
}

void sax() {
    unsigned char * dest;
    unsigned short addr;
    unsigned char stored;
    switch(opcode) {
        case 0x8B:
            A = X;
            cycle++;
            A &= imm_addr();
            fetch();
            fetched_in_adv = TRUE;
            return;
            break;
        case 0x87:
            addr = zero_page();
            break;
        case 0x97:
            addr = zero_page_ind(FALSE);
            break;
        case 0x83:
            addr = index_indir();
            break;
        case 0x93:
            addr = indir_index();
            stored = A & X & (addr >> 4);
            (write[addr])(addr, &stored);
            cycle++;
            return;
            break;
        case 0x8F:
            addr = abs_addr();
            break;
        case 0x9B:
            S = A & X;
            addr = abs_ind_addr(FALSE);
            stored = S & (addr >> 4) + 1;
            (write[addr])(addr, &stored);
            cycle++;
            return;
            break;
        case 0x9F:
            addr = abs_ind_addr(FALSE);
            stored = A & X & (addr >> 4);
            (write[addr])(addr, &stored);
            cycle++;
            return;
            break;
        case 0xDB:
            addr = imm_addr();
            break;
    }
    
    stored = A & X;
    (write[addr])(addr, &stored);
    fetched_in_adv = FALSE;
    cycle++;
}

void lax() {
    unsigned char val;
    switch(opcode) {
        case 0xA7:
            val = memory[zero_page()];
            cycle++;
            break;
        case 0xB7:
            val = memory[zero_page_ind(FALSE)];
            cycle++;
            break;
        case 0xAF:
            val = memory[abs_addr()];
            cycle++;
            break;
        case 0xBB:
            A = X = S = memory[abs_ind_addr(FALSE)] & S;
            fetch();
            fetched_in_adv = TRUE;
            set_read_flags(&A);
            cycle++;
            return;
            break;
        case 0xBF:
            val = memory[abs_ind_addr(FALSE)];
            cycle++;
            break;
        case 0xA3:
            val = memory[index_indir()];
            cycle++;
            break;
        case 0xB3:
            val = memory[indir_index()];
            cycle++;
            break;
    }
    
    A = X = val;
    set_read_flags(&A);
    fetch();
    fetched_in_adv = TRUE;
}

void dcp() {
    unsigned char * dest;
    unsigned short addr;
    switch(opcode) {
        case 0xC7:
            addr = zero_page();
            cycle++;
            break;
        case 0xD7:
            addr = zero_page_ind(TRUE);
            cycle++;
            break;
        case 0xCF:
            addr = abs_addr();
            cycle++;
            break;
        case 0xDF:
            addr = abs_ind_addr(TRUE);
            cycle++;
            break;
        case 0xCB:
            sax();
            return;
            break;
        case 0xDB:
            addr = abs_ind_addr(FALSE);
            cycle++;
            break;
        case 0xC3:
            addr = index_indir();
            cycle++;
            break;
        case 0xD3:
            addr = indir_index();
            cycle++;
            break;
    }
    cycle++;
    (dec_m[addr])(addr);
    cycle++;
    char check = A - memory[addr];  
    if(memory[addr] <= A)
        set_bit(C,&P);
    else
        rst_bit(C,&P);
    
    set_read_flags(&check);
    
    fetch();
    fetched_in_adv = TRUE;
}

void isc() {
    unsigned char * dest;
    unsigned short addr;
    switch(opcode) {
        case 0xE7:
            addr = zero_page();
            break;
        case 0xF7:
            addr = zero_page_ind(TRUE);
            break;
        case 0xEF:
            addr = abs_addr();
            break;
        case 0xFF:
            addr = abs_ind_addr(TRUE);
            break;
        case 0xEB:
            sbc();
            return;
            break;
        case 0xFB:
            addr = abs_ind_addr(FALSE);
            break;
        case 0xE3:
            addr = index_indir();
            break;
        case 0xF3:
            addr = indir_index();
            break;
    }
    
    cycle++;
    cycle++;
    (inc_m[addr])(addr);
    cycle++;
    
    unsigned char A_old = A;
    unsigned char old_C = get_bit(C,&P);
    unsigned char comp_val = memory[addr] ^ 0xFF;
    A += old_C + comp_val;
    
    if(0xFF - A_old < comp_val + old_C)
        set_bit(C,&P);
    else
        rst_bit(C,&P);
    
    if((A_old ^ A) & (comp_val ^ A) & 0x80)
        set_bit(V,&P);
    else
        rst_bit(V,&P);
    
    set_read_flags(&A);
    
    fetch();
    fetched_in_adv = TRUE;
}

void stp() {
    execution = FALSE;
    return;
}

void (*control[]) (void) = {
    brk, brk, jsr, jsr, rti, rti, rts, rts, sty, sty, ldy, ldy, cpy, cpy, cpx, cpx
};

void (*alu[]) (void) = {
    ora, ora, and, and, eor, eor, adc, adc, sta, sta, lda, lda, cmp, cmp, sbc, sbc
};

void (*rmw[]) (void) = {
    asl, asl, rol, rol, lsr, lsr, ror, ror, stx, stx, ldx, ldx, dec, dec, inc, inc
};
void (*move[]) (void) = {
    slo, slo, rla, rla, sre, sre, rra, rra, sax, sax, lax, lax, dcp, dcp, isc, isc
};

void (**pf[])(void) = {
    control, alu, rmw, move
};

void decode() {
    (*pf[opcode & 0x3][opcode >> 4])();
}