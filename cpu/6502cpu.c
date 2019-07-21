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

#include "components.h"
#include <stdio.h>

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

unsigned int cycle;
unsigned short opcode;
unsigned short PC;

boolean fetched_in_adv = FALSE;
boolean execution = TRUE;

int main() {
    initialisation();
}

int initialisation() {
    FILE * rom;
    if((rom = fopen("../../resources/roms/nestest.nes","r")) == NULL)
        return -1;
    fseek(rom,0,SEEK_END);
    int size = 16 * 1024;
    fseek(rom,16,SEEK_SET);
    fread(memory+0xC000,size,1,rom);
    
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
    
    while(execution) {
        if(fetched_in_adv == FALSE)
            fetch();
        decode();
    }
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
    opcode = memory[PC];
#ifdef DEBUG_MODE
    printInstr();
#endif
    PC++;
    cycle++;
    fetched_in_adv = FALSE;
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
    unsigned char * dest;
    
    switch(opcode) {
        case 0x02:
            stp();
            return;
            break;
        case 0x0A:
            dest = &A;
            break;
        case 0x012:
            stp();
            return;
            break;
        case 0x06:
            dest = &memory[zero_page()];
            cycle++;
            break;
        case 0x16:
            dest = &memory[zero_page_ind(TRUE)];
            cycle++;
            break;
        case 0x1A:
            nop();
            return;
            break;
        case 0x0E:
            dest = &memory[abs_addr()];
            cycle++;
            break;
        case 0x1E:
            dest = &memory[abs_ind_addr(TRUE)];
            cycle++;
            break;
    }
    
    if(get_bit(7,dest) == 1)
        set_bit(C,&P);
    else
        rst_bit(C,&P);
    
    *dest <<= 1;
    cycle++;
    set_read_flags(dest);
    
    if(opcode == 0x0A) {
        fetch();
        fetched_in_adv = TRUE;
    }
    else
        cycle++;
}

void inline lsr() {
    unsigned char * dest;
    
    switch(opcode) {
        case 0x4A:
            dest = &A;
            break;
        case 0x5A:
            nop();
            return;
            break;
        case 0x46:
            dest = &memory[zero_page()];
            cycle++;
            break;
        case 0x56:
            dest = &memory[zero_page_ind(TRUE)];
            cycle++;
            break;
        case 0x4E:
            dest = &memory[abs_addr()];
            cycle++;
            break;
        case 0x5E:
            dest = &memory[abs_ind_addr(TRUE)];
            cycle++;
            break;
    }
    
    if(get_bit(0,dest) == 1)
        set_bit(C,&P);
    else
        rst_bit(C,&P);
    
    *dest >>= 1;
    cycle++;
    set_read_flags(dest);

    if(opcode == 0x4A) {
        fetch();
        fetched_in_adv = TRUE;
    }
    else
        cycle++;
}

void inline rol() {
    unsigned char * dest;
    
    switch(opcode) {
        case 0x22:
            stp();
            return;
            break;
        case 0x2A:
            dest = &A;
            break;
        case 0x26:
            dest = &memory[zero_page()];
            cycle++;
            break;
        case 0x32:
            stp();
            return;
            break;
        case 0x36:
            dest = &memory[zero_page_ind(TRUE)];
            cycle++;
            break;
        case 0x3A:
            nop();
            return;
            break;
        case 0x2E:
            dest = &memory[abs_addr()];
            cycle++;
            break;
        case 0x3E:
            dest = &memory[abs_ind_addr(TRUE)];
            cycle++;
            break;
    }
    
    unsigned char bit7 = *dest & 0x80;
    *dest <<= 1;
    cycle++;
    if(get_bit(C,&P) == 0)
        rst_bit(0,dest);
    else
        set_bit(0,dest);
    
    set_read_flags(dest);

    if(bit7 == 0)
        rst_bit(C,&P);
    else
        set_bit(C,&P);
    
    if(opcode == 0x2A) {
        fetch();
        fetched_in_adv = TRUE;
    }
    else
        cycle++;
}

void inline ror() {
    unsigned char * dest;
    
    switch(opcode) {
        case 0x62:
            stp();
            return;
            break;
        case 0x6A:
            dest = &A;
            break;
        case 0x72:
            stp();
            return;
            break;
        case 0x66:
            dest = &memory[zero_page()];
            cycle++;
            break;
        case 0x76:
            dest = &memory[zero_page_ind(TRUE)];
            cycle++;
            break;
        case 0x6E:
            dest = &memory[abs_addr()];
            cycle++;
            break;
        case 0x7A:
            nop();
            return;
            break;
        case 0x7E:
            dest = &memory[abs_ind_addr(TRUE)];
            cycle++;
            break;
    }
    
    unsigned char old_bit0 = *dest & 0x1;
    *dest >>= 1;
    cycle++;
    
    if(get_bit(C,&P) == 0)
        rst_bit(7,dest);
    else
        set_bit(7,dest);
    
    if(old_bit0 == 1)
        set_bit(C,&P);
    else
        rst_bit(C,&P);
    
    set_read_flags(dest);
    
    if(opcode == 0x6A) {
        fetch();
        fetched_in_adv = TRUE;
    }
    else
        cycle++;
}

void inline inc() {
    unsigned char * dest;
    switch(opcode) {
        case 0xE2:
            nop();
            return;
            break;
        case 0xE6:
            dest = &memory[zero_page()];
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
            dest = &memory[zero_page_ind(TRUE)];
            break;
        case 0xFA:
            nop();
            return;
            break;
        case 0xEE:
            dest = &memory[abs_addr()];
            break;
        case 0xFE:
            dest = &memory[abs_ind_addr(TRUE)];
            break;
    }
    cycle++;
    (*dest)++;
    cycle++;
    set_read_flags(dest);
    fetched_in_adv = FALSE;
    cycle++;
}

void inline dec() {
    unsigned char * dest;
    switch(opcode) {
        case 0xC2:
            nop();
            return;
            break;
        case 0xC6:
            dest = &memory[zero_page()];
            cycle++;
            break;
        case 0xCA:
            dest = &X;
            break;
        case 0xD2:
            stp();
            return;
            break;
        case 0xD6:
            dest = &memory[zero_page_ind(TRUE)];
            cycle++;
            break;
        case 0xCE:
            dest = &memory[abs_addr()];
            cycle++;
            break;
        case 0xDA:
            nop();
            return;
            break;
        case 0xDE:
            dest = &memory[abs_ind_addr(TRUE)];
            cycle++;
            break;
    }
    (*dest)--;
    cycle++;
    
    set_read_flags(dest);
    if(opcode == 0xCA) {
        fetch();
        fetched_in_adv = TRUE;
    }
    else {
        fetched_in_adv = FALSE;
        cycle++;
    }
}

// WRITE

void inline sta() {
    switch(opcode) {
        case 0x85:
            memory[zero_page()] = A;
            break;
        case 0x89:
            nop();
            return;
            break;
        case 0x95:
            memory[zero_page_ind(TRUE)] = A;
            break;
        case 0x8D:
            memory[abs_addr()] = A;
            break;
        case 0x9D:
            memory[abs_ind_addr(TRUE)] = A;
            break;
        case 0x99:
            memory[abs_ind_addr(FALSE)] = A;
            break;
        case 0x81:
            memory[index_indir()] = A;
            break;
        case 0x91:
            memory[indir_index()] = A;
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
            memory[zero_page()] = X;
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
            memory[zero_page_ind(FALSE)] = X;
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
            memory[abs_addr(FALSE)] = X;
            cycle++;
            break;
        case 0x9E:
            addr = abs_addr();
            memory[addr] = X & (addr >> 8) + 1;
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
            memory[zero_page()] = Y;
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
            memory[abs_addr()] = Y;
            cycle++;
            break;
        case 0x90:
            rel_addr(get_bit(C,&P) == 0);
            return;
            break;
        case 0x94:
            memory[zero_page_ind(TRUE)] = Y;
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
            memory[addr] = Y & (addr >> 8) + 1;
            cycle++;
            break;
    }
    fetched_in_adv = FALSE;
    
}

void inline slo() {
    unsigned char * dest;
    switch(opcode) {
        case 0x07:
            dest = &memory[zero_page()];
            cycle++;
            break;
        case 0x0B:
            anc();
            return;
            break;
        case 0x17:
            dest = &memory[zero_page_ind(TRUE)];
            cycle++;
            break;
        case 0x0F:
            dest = &memory[abs_addr()];
            cycle++;
            break;
        case 0x1F:
            dest = &memory[abs_ind_addr(TRUE)];
            cycle++;
            break;
        case 0x1B:
            dest = &memory[abs_ind_addr(FALSE)];
            cycle++;
            break;
        case 0x03:
            dest = &memory[index_indir()];
            cycle++;
            break;
        case 0x13:
            dest = &memory[indir_index()];
            cycle++;
            break;
    }
    cycle++;
    if(get_bit(7,dest) == 1)
        set_bit(C,&P);
    else
        rst_bit(C,&P);
    
    *dest <<= 1;
    cycle++;
    
    A |= *dest;
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
    switch(opcode) {
        case 0x27:
            dest = &memory[zero_page()];
            break;
        case 0x2B:
            anc();
            return;
            break;
        case 0x37:
            dest = &memory[zero_page_ind(TRUE)];
            break;
        case 0x2F:
            dest = &memory[abs_addr()];
            break;
        case 0x3F:
            dest = &memory[abs_ind_addr(TRUE)];
            break;
        case 0x3B:
            dest = &memory[abs_ind_addr(FALSE)];
            break;
        case 0x23:
            dest = &memory[index_indir()];
            break;
        case 0x33:
            dest = &memory[indir_index()];
            break;
    }
    cycle++;
    
    unsigned char val = *dest;
    unsigned char bit7 = *dest & 0x80;
    *dest <<= 1;
    cycle++;
    if(get_bit(C,&P) == 0)
        rst_bit(0,dest);
    else
        set_bit(0,dest);
    
    set_read_flags(dest);
    
    if(bit7 == 0)
        rst_bit(C,&P);
    else
        set_bit(C,&P);
    cycle++;
    A &= *dest;
    set_read_flags(&A);
    fetch();
    fetched_in_adv = TRUE;
}

void sre() {
    unsigned char * dest;
    switch(opcode) {
        case 0x47:
            dest = &memory[zero_page()];
            break;
        case 0x57:
            dest = &memory[zero_page_ind(TRUE)];
            break;
        case 0x4F:
            dest = &memory[abs_addr()];
            break;
        case 0x5F:
            dest = &memory[abs_ind_addr(TRUE)];
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
            dest = &memory[abs_ind_addr(FALSE)];
            break;
        case 0x43:
            dest = &memory[index_indir()];
            break;
        case 0x53:
            dest = &memory[indir_index()];
            break;
    }
    cycle++;
    
    if(get_bit(0,dest) == 0)
        rst_bit(C,&P);
    else
        set_bit(C,&P);
    cycle++;
    *dest >>= 1;
    cycle++;
    A ^= *dest;
    set_read_flags(&A);
    fetch();
    fetched_in_adv = TRUE;
}

void rra() {
    unsigned char * dest;
    switch(opcode) {
        case 0x67:
            dest = &memory[zero_page()];
            break;
        case 0x77:
            dest = &memory[zero_page_ind(TRUE)];
            break;
        case 0x6F:
            dest = &memory[abs_addr()];
            break;
        case 0x7F:
            dest = &memory[abs_ind_addr(TRUE)];
            break;
        case 0x7B:
            dest = &memory[abs_ind_addr(FALSE)];
            break;
        case 0x63:
            dest = &memory[index_indir()];
            break;
        case 0x73:
            dest = &memory[indir_index()];
            break;
    }
    cycle++;
    
    unsigned char val = *dest;
    unsigned char old_bit0 = *dest & 0x1;
    *dest >>= 1;
    cycle++;
    if(get_bit(C,&P) == 0)
        rst_bit(7,dest);
    else
        set_bit(7,dest);
    if(old_bit0 == 0)
        rst_bit(C,&P);
    else
        set_bit(C,&P);
    
    cycle++;
    
    unsigned char A_old = A;
    unsigned char old_C = get_bit(C,&P);
    A += old_C + *dest;
    
    if(0xFF - A_old < *dest + old_C)
        set_bit(C,&P);
    else
        rst_bit(C,&P);
    
    if((A_old ^ A) & (*dest ^ A) & 0x80)
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
    switch(opcode) {
        case 0x8B:
            A = X;
            cycle++;
            A &= imm_addr();
            fetch();
            fetched_in_adv = TRUE;
            break;
            return;
        case 0x87:
            dest = &memory[zero_page()];
            break;
        case 0x97:
            dest = &memory[zero_page_ind(FALSE)];
            break;
        case 0x83:
            dest = &memory[index_indir()];
            break;
        case 0x93:
            addr = indir_index();
            memory[addr] = A & X & (addr >> 4);
            cycle++;
            break;
            return;
        case 0x8F:
            dest = &memory[abs_addr()];
            break;
        case 0x9B:
            S = A & X;
            addr = abs_ind_addr(FALSE);
            memory[addr] = S & (addr >> 4) + 1;
            cycle++;
            break;
            return;
        case 0x9F:
            addr = abs_ind_addr(FALSE);
            memory[addr] = A & X & (addr >> 4);
            cycle++;
            break;
            return;
        case 0xDB:
            dest = &memory[imm_addr()];
            break;
    }
    
    unsigned char val = A & X;
    *dest = val;
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
    switch(opcode) {
        case 0xC7:
            dest = &memory[zero_page()];
            cycle++;
            break;
        case 0xD7:
            dest = &memory[zero_page_ind(TRUE)];
            cycle++;
            break;
        case 0xCF:
            dest = &memory[abs_addr()];
            cycle++;
            break;
        case 0xDF:
            dest = &memory[abs_ind_addr(TRUE)];
            cycle++;
            break;
        case 0xCB:
            sax();
            return;
            break;
        case 0xDB:
            dest = &memory[abs_ind_addr(FALSE)];
            cycle++;
            break;
        case 0xC3:
            dest = &memory[index_indir()];
            cycle++;
            break;
        case 0xD3:
            dest = &memory[indir_index()];
            cycle++;
            break;
    }
    cycle++;
    (*dest)--;
    cycle++;
    char check = A - *dest;  
    if(*dest <= A)
        set_bit(C,&P);
    else
        rst_bit(C,&P);
    
    set_read_flags(&check);
    
    fetch();
    fetched_in_adv = TRUE;
}

void isc() {
    unsigned char * dest;
    switch(opcode) {
        case 0xE7:
            dest = &memory[zero_page()];
            break;
        case 0xF7:
            dest = &memory[zero_page_ind(TRUE)];
            break;
        case 0xEF:
            dest = &memory[abs_addr()];
            break;
        case 0xFF:
            dest = &memory[abs_ind_addr(TRUE)];
            break;
        case 0xEB:
            sbc();
            return;
            break;
        case 0xFB:
            dest = &memory[abs_ind_addr(FALSE)];
            break;
        case 0xE3:
            dest = &memory[index_indir()];
            break;
        case 0xF3:
            dest = &memory[indir_index()];
            break;
    }
    
    cycle++;
    cycle++;
    (*dest)++;
    cycle++;
    
    unsigned char A_old = A;
    unsigned char old_C = get_bit(C,&P);
    unsigned char comp_val = *dest ^ 0xFF;
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