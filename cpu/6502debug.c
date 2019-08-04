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

#include "6502debug.h"
#include "components.h"
#include <stdio.h>
#include <stdlib.h>

void printInstr() {  
    static unsigned short ppu_x, ppu_y;
    char * str = (char *)malloc(50 * sizeof(char));

    printf("%04X  %02X ", PC, opcode);
    char nops;
    
    switch(addr_mode[opcode]) {
        case 4:
            nops = 2;
            break;
        case 5:
            nops = 2;
            break;
        case 6:
            nops = 2;
            break;
        case 8:
            nops = 0;
            break;
        case 11:
            nops = 2;
            break;
        default:
            nops = 1;
            break;
    }
    
    
    switch(nops) {
        case 0:
            if(unoff_op[opcode])
                printf("      *");
            else
                printf("       ");
            break;
        case 1:
            if(unoff_op[opcode])
                printf("%02X    *", memory[PC+1]);
            else
                printf("%02X     ", memory[PC+1]);
            break;
        case 2:
            if(unoff_op[opcode])
                printf("%02X %02X *", memory[PC+1], memory[PC+2]);
            else
                printf("%02X %02X  ", memory[PC+1], memory[PC+2]);
            break;
    }
    
    printf("%s ", op_name[opcode]);

    if(nops > 0) {
        unsigned char zero_addr;
        unsigned short abs_addr;
        unsigned short point_addr;
        unsigned char pcl, point_low, point_high, old_pcl;
        unsigned short pch;
        signed char offset;
        
        switch(addr_mode[opcode]) {
            case 0:
                printf("#$%02X                        ", memory[PC+1]);
                break;
            case 1:
                sprintf(str, "$%02X = %02X", memory[PC+1], memory[memory[PC+1]]);
                printf("%-28s", str);
                break;
            case 2:
                zero_addr = X + memory[PC+1];
                printf("$%02X,X @ %02X = %02X             ", memory[PC+1], zero_addr, memory[zero_addr]);
                break;
            case 3:
                zero_addr = Y + memory[PC+1];
                printf("$%02X,Y @ %02X = %02X             ", memory[PC+1], zero_addr, memory[zero_addr]);
                break;
            case 5:
                abs_addr = memory[PC+2] << 8 | memory[PC+1];
                printf("$%04X,X @ %04X = %02X         ", abs_addr, (abs_addr + X) % 0x10000, memory[(abs_addr + X) % 0x10000]);
                break;
            case 6:
                abs_addr = memory[PC+2] << 8 | memory[PC+1];
                printf("$%04X,Y @ %04X = %02X         ", abs_addr, (abs_addr + Y) % 0x10000, memory[(abs_addr + Y) % 0x10000]);
                break;
            case 7:
                offset = memory[PC+1];
                pch = (PC+2) & 0xFF00;
                pcl = (PC+2) & 0x00FF;
                old_pcl = pcl;
                pcl += offset;
                if(0xFF - old_pcl < offset) 
                    printf("$%04X                       ", (pch + 0x100 | pcl) % 0x10000);
                else
                    printf("$%04X                       ", (pch | pcl) % 0x10000);
                break;
            case 9:
                zero_addr = (memory[PC+1] + X) % 256;
                printf("($%02X,X) @ %02X = %04X = %02X    ", memory[PC+1], zero_addr, memory[(zero_addr+1) % 256] << 8 | memory[zero_addr],
                        memory[memory[(zero_addr+1) % 256] << 8 | memory[zero_addr]]);
                break;
            case 10:
                zero_addr = memory[PC+1];
                point_addr = memory[(zero_addr+1) % 256] << 8 | (memory[zero_addr]);
                printf("($%02X),Y = %04X @ %04X = %02X  ", zero_addr, point_addr, (point_addr + Y) % 0x10000,
                        memory[(point_addr + Y) % 0x10000]);
                break;
            case 11:
                abs_addr = memory[PC+2] << 8 | memory[PC+1];
                point_low = memory[PC+1];
                point_high = memory[PC+2];
                pcl = memory[(point_high << 8) | point_low];
                sprintf(str, "($%04X) = %04X", abs_addr, memory[((point_high << 8) | (point_low + 1) % 256) % 0x10000] << 8 | pcl);
                printf("%-28s", str);
                break;
            case 4:
                abs_addr = memory[PC+2] << 8 | memory[PC+1];
                switch(opcode) {
                    case 0x4C:
                    case 0x20:
                        printf("$%04X                       ", memory[PC+2] << 8 | memory[PC+1]);
                        break;
                    default:
                        sprintf(str, "$%04X = %02X", abs_addr, memory[abs_addr]);
                        printf("%-28s", str);
                        break;
                }
                break;
            default:
                abs_addr = memory[PC+2] << 8 | memory[PC+1];
                printf("$%04X                       ", memory[PC+2] << 8 | memory[PC+1]);
                break;
        }
    }
    
    else  {
        switch(opcode) {
            case 0x0A:
            case 0x6A:
            case 0x2A:
            case 0x4A:
                sprintf(str, "A");
                printf("%-28s", str);
                break;
            default:
                printf("                            ");
                break;
        }
    }
    
    unsigned char prev_ppu_x = ppu_x;
    ppu_x = (cycle-7) * 3 % 341;
    if(prev_ppu_x > ppu_x)
        ppu_y = (ppu_y + 1) % 262;
    printf("A:%02X X:%02X Y:%02X P:%02X SP:%02X PPU:%3d,%3d CYC:%d\n", A, X, Y, P, S, ppu_x, ppu_y, cycle);
}