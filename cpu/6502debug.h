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

char * op_name[] = {
    "BRK",	"ORA",	"STP",	"SLO",	"NOP",	"ORA",	"ASL",	"SLO",	"PHP",	"ORA",	"ASL",	"ANC",	"NOP",	"ORA",	"ASL",	"SLO",	"BPL",	"ORA",	"STP",	"SLO",	"NOP",	"ORA",	"ASL",	"SLO",	"CLC",	"ORA",	"NOP",	"SLO",	"NOP",	"ORA",	"ASL",	"SLO",
    "JSR",	"AND",	"STP",	"RLA",	"BIT",	"AND",	"ROL",	"RLA",	"PLP",	"AND",	"ROL",	"ANC",	"BIT",	"AND",	"ROL",	"RLA",	"BMI",	"AND",	"STP",	"RLA",	"NOP",	"AND",	"ROL",	"RLA",	"SEC",	"AND",	"NOP",	"RLA",	"NOP",	"AND",	"ROL",	"RLA",
    "RTI",	"EOR",	"STP",	"SRE",	"NOP",	"EOR",	"LSR",	"SRE",	"PHA",	"EOR",	"LSR",	"ALR",	"JMP",	"EOR",	"LSR",	"SRE",	"BVC",	"EOR",	"STP",	"SRE",	"NOP",	"EOR",	"LSR",	"SRE",	"CLI",	"EOR",	"NOP",	"SRE",	"NOP",	"EOR",	"LSR",	"SRE",
    "RTS",	"ADC",	"STP",	"RRA",	"NOP",	"ADC",	"ROR",	"RRA",	"PLA",	"ADC",	"ROR",	"ARR",	"JMP",	"ADC",	"ROR",	"RRA",	"BVS",	"ADC",	"STP",	"RRA",	"NOP",	"ADC",	"ROR",	"RRA",	"SEI",	"ADC",	"NOP",	"RRA",	"NOP",	"ADC",	"ROR",	"RRA",
    "NOP",	"STA",	"NOP",	"SAX",	"STY",	"STA",	"STX",	"SAX",	"DEY",	"NOP",	"TXA",	"XAA",	"STY",	"STA",	"STX",	"SAX",	"BCC",	"STA",	"STP",	"AHX",	"STY",	"STA",	"STX",	"SAX",	"TYA",	"STA",	"TXS",	"TAS",	"SHY",	"STA",	"SHX",	"AHX",
    "LDY",	"LDA",	"LDX",	"LAX",	"LDY",	"LDA",	"LDX",	"LAX",	"TAY",	"LDA",	"TAX",	"LAX",	"LDY",	"LDA",	"LDX",	"LAX",	"BCS",	"LDA",	"STP",	"LAX",	"LDY",	"LDA",	"LDX",	"LAX",	"CLV",	"LDA",	"TSX",	"LAS",	"LDY",	"LDA",	"LDX",	"LAX",
    "CPY",	"CMP",	"NOP",	"DCP",	"CPY",	"CMP",	"DEC",	"DCP",	"INY",	"CMP",	"DEX",	"AXS",	"CPY",	"CMP",	"DEC",	"DCP",	"BNE",	"CMP",	"STP",	"DCP",	"NOP",	"CMP",	"DEC",	"DCP",	"CLD",	"CMP",	"NOP",	"DCP",	"NOP",	"CMP",	"DEC",	"DCP",
    "CPX",	"SBC",	"NOP",	"ISC",	"CPX",	"SBC",	"INC",	"ISC",	"INX",	"SBC",	"NOP",	"SBC",	"CPX",	"SBC",	"INC",	"ISC",	"BEQ",	"SBC",	"STP",	"ISC",	"NOP",	"SBC",	"INC",	"ISC",	"SED",	"SBC",	"NOP",	"ISC",	"NOP",	"SBC",	"INC",	"ISC"
};

/* 0: imm_addr
 * 1: zero_page
 * 2: zero_page, X
 * 3: zero_page, Y
 * 4: abs_addr
 * 5: abs_addr, X
 * 6: abs_addr, Y
 * 7: rel_addr
 * 8: implied
 * 9: index_indir
 * 10: indir_index
 * 11: jmp_indir
 */
int addr_mode[] = {
    8, 9, 8, 9, 1, 1, 1, 1, 8, 0, 8, 0, 4, 4, 4, 4, 7, 10, 8, 10, 2, 2, 2, 2, 8, 6, 8, 6, 5, 5, 5, 5,
    4, 9, 8, 9, 1, 1, 1, 1, 8, 0, 8, 0, 4, 4, 4, 4, 7, 10, 8, 10, 2, 2, 2, 2, 8, 6, 8, 6, 5, 5, 5, 5,
    8, 9, 8, 9, 1, 1, 1, 1, 8, 0, 8, 0, 4, 4, 4, 4, 7, 10, 8, 10, 2, 2, 2, 2, 8, 6, 8, 6, 5, 5, 5, 5,
    8, 9, 8, 9, 1, 1, 1, 1, 8, 0, 8, 0, 11, 4, 4, 4, 7, 10, 8, 10, 2, 2, 2, 2, 8, 6, 8, 6, 5, 5, 5, 5,
    0, 9, 0, 9, 1, 1, 1, 1, 8, 0, 8, 0, 4, 4, 4, 4, 7, 10, 8, 10, 2, 2, 3, 3, 8, 6, 8, 6, 5, 5, 6, 6,
    0, 9, 0, 9, 1, 1, 1, 1, 8, 0, 8, 0, 4, 4, 4, 4, 7, 10, 8, 10, 2, 2, 3, 3, 8, 6, 8, 6, 5, 5, 6, 6,
    0, 9, 0, 9, 1, 1, 1, 1, 8, 0, 8, 0, 4, 4, 4, 4, 7, 10, 8, 10, 2, 2, 2, 2, 8, 6, 8, 6, 5, 5, 5, 5,
    0, 9, 0, 9, 1, 1, 1, 1, 8, 0, 8, 0, 4, 4, 4, 4, 7, 10, 8, 10, 2, 2, 2, 2, 8, 6, 8, 6, 5, 5, 5, 5
};


