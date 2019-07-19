#include <stdlib.h>
#include <stdio.h>
#include <SDL2/SDL.h>
#include "ppuregs.h"

#define WINDOW_WIDTH 600

#define MAGNIFY 2

struct Tile {
        unsigned char pixel[64][4];   //color of every pixel
};

struct Palette {
        unsigned char col[4][4];
};

unsigned char ppummap[0x4000];
unsigned char cpummap[0x10000];
unsigned char pattab[0x2000];
unsigned char oam[0x100];

struct Palette pal;

void drawPixel(int pixel, struct Tile tile, SDL_Renderer * renderer, int x, int y); 
void drawScanline(char scanline, struct Palette pal, SDL_Renderer * renderer);
void drawSprite(int sprite, SDL_Renderer * renderer, struct Palette palsb, unsigned char pattable);
void regsIni();
void fillTile(struct Tile * tile, unsigned char tile_ind, struct Palette pal, unsigned char pattable);

int main(void) {
    regsIni();
    //read game ROM (mapper 0)
    //unsigned char pattab[0x2000];   //pattern table 0 and 1
    unsigned char nametable[0x800]; //nametable 0 and 1
    FILE * rom;
    if((rom = fopen("../../resources/roms/donkey_kong.nes","r")) == NULL)
        printf("Error opening ROM");
    fseek(rom, 16400, SEEK_SET);
    
    if((fread(pattab,0x2000,1,rom)) == 0) 
       printf("Error reading ROM");   
    
    FILE * ppudump;
    if((ppudump = fopen("../../resources/dumps/donkey_kong.initial_animation.ppumem","r")) == NULL)
        printf("Error opening PPU dump\n");
    
    FILE * cpudump;
    if((cpudump = fopen("../../resources/dumps/donkey_kong.initial_animation.cpumem","r")) == NULL)
        printf("Error opening CPU dump\n");
    
    if((fread(ppummap,0x3120,1,ppudump)) == 0)
        printf("Error reading PPU dump\n");
    
    fseek(cpudump, 0, SEEK_END);
    int cpulog_size = ftell(cpudump);
    fseek(cpudump, 0, SEEK_SET);
    
    if((fread(cpummap,cpulog_size,1,cpudump)) == 0)
        printf("Error reading CPU dump\n");
    
    fseek(ppudump, 0x3000, SEEK_SET);
    
    if((fread(oam,0x100,1,ppudump)) == 0)
        printf("Error reading OAM dump\n");
    
    struct Palette palbg = {{{255,255,255,1},{23,29,146,1},{255,26,26,1},{68,255,26,1}}}; 
    struct Palette palsp = {{{255,255,255,1},{245,210,10,1},{245,10,182,1},{245,92,10,1}}};
    
    struct Tile tile;
    
    printf("Nametable[%d] = %X\n", 0, ppummap[0x2000 + 32 +0xA]);
    printf("%X\n", cpummap[0x2002]);
    
    
    SDL_Event event;
    SDL_Renderer *renderer;
    SDL_Window *window;
    int i;

    SDL_Init(SDL_INIT_VIDEO);
    SDL_CreateWindowAndRenderer(WINDOW_WIDTH, WINDOW_WIDTH, 0, &window, &renderer);
    SDL_SetRenderDrawColor(renderer, 146, 109, 171, 1);
    SDL_RenderClear(renderer);
    
    
    for(int sl = 0; sl < 30; sl++) {
        drawScanline(sl, palbg, renderer);
    }
    
    
    for(int i = 0; i < 64; i++) {
        drawSprite(i, renderer, palsp,0);
    }
    
    SDL_RenderPresent(renderer);
    while (1) {
        if (SDL_PollEvent(&event) && event.type == SDL_QUIT)
            break;
    }
    //fclose(ppulog);
    //fclose(cpulog);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return EXIT_SUCCESS;
}

void drawPixel(int pixel, struct Tile tile, SDL_Renderer * renderer, int x, int y) {
    unsigned char pix_col[4];
    for(int i = 0; i < 4; i++) {
        pix_col[i] = tile.pixel[pixel][i];
    }
    
    SDL_SetRenderDrawColor(renderer, pix_col[0], pix_col[1], pix_col[2], pix_col[3]);
    SDL_RenderDrawPoint(renderer,x,y);
}


void drawScanline(char scanline, struct Palette palbg, SDL_Renderer * renderer) {
    printf("PPUCTRL: %X\n", *PPUCTRL);

    unsigned short colour;
    
    unsigned char b0;
    unsigned char b1;
    int mask, row;
    unsigned char pattable;
    
    struct Tile tile;
    
    //printf("Nametable[%d] = %X\n", 0, ppummap[0x2000 + 32 * 3 + 0xD]);
    //PPUCTRL = &cpummap[0x2000];
    //printf("pRUEBA: %X\N", *PPUCTRL);
    //fflush(stdin);
    pattable = 1;
    
    unsigned char tile_ind;
    
    for(int t = 0; t < 32; t++) {
        //pattable = *PPUCTRL >> 4 & 1;
        //pattable = 1;
        //printf("%X\n", pattable);
        tile_ind = ppummap[0x2000 + 0x400 * pattable + scanline * 32 + t];
        
        fillTile(&tile, tile_ind, palbg, pattable);
        
        row = 0;
        for(int pixel = 0; pixel < 64; pixel++) {
            if(pixel % 8 == 0 && (pixel > 0))
                row++;
            for(int x = t * MAGNIFY * 8 + MAGNIFY*(pixel % 8); x < t * MAGNIFY * 8 + MAGNIFY + MAGNIFY*(pixel % 8); x++) {
                for(int y = MAGNIFY * 8 * scanline + MAGNIFY*row; y < MAGNIFY * 8 * scanline + MAGNIFY + MAGNIFY*row; y++) {
                    drawPixel(pixel,tile,renderer,x,y);
                }
            }
        }
    }
}

void fillTile(struct Tile * tile, unsigned char tile_ind, struct Palette pal, unsigned char pattable) {
    unsigned short colour;
    int mask;
    unsigned char b0, b1;
    
    for(int t = 0; t < 32; t++) {
        for(int row = 0; row < 8; row++) {
            b0 = pattab[0x1000 * pattable + row + tile_ind*16];
            b1 = pattab[0x1000 * pattable + row + tile_ind*16 + 8];

            mask = 1;
            colour = 0;
            for(int i = 0; i < 8; i++) {
                colour |= (b0 & (mask << i)) << i;
                colour |= (b1 & (mask << i)) << (i+1);
            }

            mask = 0x30000;
            for(int i = row*8; i < row*8+8; i++) {
                mask >>= 2;
                for(int j = 0; j < 4; j++) {            
                    (*tile).pixel[i][j] = pal.col[(colour & mask) >> 2 * (7 - (i % 8))][j];
                }
            }
        }
    }
    
}

void drawSprite(int sprite, SDL_Renderer * renderer, struct Palette palsb, unsigned char pattable) {
    unsigned short ind = 4 * sprite;
    unsigned short x_pos = oam[ind + 3]; //x position
    unsigned short y_pos = oam[ind]+1; //y position
    
    //printf("Sprite %d X:%d, Y:%d, Tile: %X\n", sprite,x_pos,y_pos, oam[ind+1]);
    
    struct Tile spr;
    
    //Tile generation
    for(int pix = 0; pix < 64; pix++) {
        fillTile(&spr, oam[ind+1], palsb, pattable);
    }
    
    //printf("spr.pixel[0][0]: %X\n", spr.pixel[0][0]);
    unsigned char row = 0;
    for(int pixel = 0; pixel < 64; pixel++) {
        if(pixel % 8 == 0 && (pixel > 0))
            row++;
        for(int x = x_pos * MAGNIFY + MAGNIFY*(pixel % 8); x < x_pos * MAGNIFY + MAGNIFY + MAGNIFY*(pixel % 8); x++) {
            for(int y = MAGNIFY * y_pos + MAGNIFY*row; y < MAGNIFY * y_pos + MAGNIFY + MAGNIFY*row; y++) {
                //printf("X: %d, Y: %d\n", x, y);
                drawPixel(pixel,spr,renderer,x,y);
            }
        }
    }
}


void regsIni() {
    PPUCTRL = &cpummap[0x2000];
    PPUMASK = &cpummap[0x2001];
    PPUSTATUS = &cpummap[0x2002];
    OAMADDR = &cpummap[0x2003];
    OAMDATA = &cpummap[0x2004];
    PPUSCROLL = &cpummap[0x2005];
    PPUADDR = &cpummap[0x2006];
    PPUDATA = &cpummap[0x2007];
    OAMDMA = &cpummap[0x4014];
    
    return;
}
