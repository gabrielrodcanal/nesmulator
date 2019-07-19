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

/* 
 * File:   components.h
 * Author: gabriel
 *
 * Created on August 14, 2018, 6:50 PM
 */

#define NBYTES 16 * 1024
#define TRUE 1
#define FALSE 0

// FLAGS
#define C 0
#define Z 1
#define I 2
#define D 3
#define V 6
#define N 7


unsigned char memory[0xFFFF];

// Registers
unsigned char A;
unsigned char X;
unsigned char Y;
extern unsigned short PC;
unsigned short S;
unsigned char P;

#define A_reg 0
#define P_reg 1

extern unsigned short opcode;
extern unsigned int cycle;

typedef char boolean;

void printInstr();
