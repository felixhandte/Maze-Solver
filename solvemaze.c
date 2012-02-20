/******************************************************************************
 * solvemaze.c                                                                *
 *                                                                            *
 * A C program to solve a 2D maze using the A* Search Algorithm.              *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the GNU General Public License as published by       *
 * the Free Software Foundation; either version 2, or (at your option)        *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * GNU General Public License for more details.                               *
 *                                                                            *
 * Copyright (C) 2011, 2012 Felix Handte (w@felixhandte.com)                  *
 *                                                                            *
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/ioctl.h>

#ifndef __MACH__
#include <time.h>

#define CLOCK_ID CLOCK_REALTIME
//#define CLOCK_ID CLOCK_PROCESS_CPUTIME_ID

struct timespec t_res;
struct timespec t_diff;

struct timespec t_start;
struct timespec t_dimensions;
struct timespec t_malloc;
struct timespec t_zero;
struct timespec t_parse;
struct timespec t_initheap;
struct timespec t_solve;
struct timespec t_path;

void timespec_diff(struct timespec *s, struct timespec *e, struct timespec *o);
#endif

#define printdiff(S,T1,T2) { timespec_diff(&T1, &T2, &t_diff); \
	fprintf(stderr, "Time to " S ": %3ld.%09ld\n", t_diff.tv_sec, t_diff.tv_nsec); }

// The following sets of defines and typedefs represent a choice between
// different heuristics to use for the A* Search Algorithm. Using no distance
// heuristic reduces the algorithm to be equivalent to a breadth-first search.
// The 'pretty' distance heuristic produces paths which prioritize heading
// straight towards the goal, all other things being equal. The efficient
// heuristic, using integer arithmetic only, is faster and allows each maze
// node to be stored in 16 rather than 20 (32bit) or 24 (64bit) bytes.
// It also allows the discarding of more nodes faster, because it more
// accurately predicts the distance between node and target. To see the
// difference in output, run on a completely open maze.

//Pretty Distance Heuristic:
//#define dist(X1,Y1,X2,Y2) sqrt((X1 - X2) * (X1 - X2) + (Y1 - Y2) * (Y1 - Y2))
//typedef double h_t;

//Efficient Distance Heuristic:
#define dist(X1,Y1,X2,Y2) (abs(X1 - X2) + abs(Y1 - Y2))
typedef int h_t;

//No Distance Heuristic:
//#define dist(X1,Y1,X2,Y2) (0)
//typedef double h_t;


typedef struct _node {
	h_t   hscore;
	int   gscore;
	int   hindex;
	char  neighbors;
	char  parent;
	char  state;
} node;

unsigned int mx;
unsigned int my;

int sx;
int sy;
int ex;
int ey;

node **m;

// Heap Realloc Increment
#define HRI 4096

int *ohx; // Open node heap of x coordinates
int *ohy; // Open node heap of y coordinates
h_t *ohf; // Open node heap of f distance
int ah;   // Allocated size of heap
int nh;   // Used size of heap

unsigned long long sc[4] = {0, 0, 0, 0};
unsigned int hswaps = 0;

#define FANCY_TERM

#ifdef FANCY_TERM
int isttyi;
int isttyo;
int isttye;

char *tcolors[4] = {"\033[0m", "\033[32m", "\033[31m", "\033[34m"};
#endif

int  alloc_maze(void);
int  parse_maze(FILE *in);
void print_maze(void);
void calc_results(int sx, int sy, int ex, int ey);
void print_solution(int sx, int sy, int ex, int ey, FILE *f);
void print_graphic_solution(int sx, int sy, int ex, int ey);
int  check_heapness(void);
void print_help(void);

int main(int argc, char *argv[]){
	
	#ifdef FANCY_TERM
	isttyi = isatty(fileno(stdin ));
	isttyo = isatty(fileno(stdout));
	isttye = isatty(fileno(stderr));
	#endif
	
	if(argc < 2){
		print_help();
		return 1;
	}
	char *fn = argv[1];
	if(argc == 6){
		sx = atoi(argv[2]);
		sy = atoi(argv[3]);
		ex = atoi(argv[4]);
		ey = atoi(argv[5]);
	} else
	if(argc == 2){
		sx = -1;
		sy = -1;
		ex = -1;
		ey = -1;
	} else {
		print_help();
		return 1;
	}
	
	fprintf(stderr, "Reading dimensions... ");
	#ifndef __MACH__
	clock_getres (CLOCK_ID, &t_res);
	clock_gettime(CLOCK_ID, &t_start);
	#endif
	
	FILE *in;
	if(!strcmp(fn,"-")){
		in = stdin;
	} else {
		in = fopen(fn, "r");
	}
	if(in == NULL){
		fprintf(stderr, "fopen on '%s' failed (%m).\n", fn);
	}
	
	char wstr[12], hstr[12];
	int nstr = 0;
	int c;
	while(1){
		c = fgetc(in);
		if(c == EOF){
			fprintf(stderr, "File ended while reading dimensions.\n");
			return 1;
		}
		if(nstr == 12){
			fprintf(stderr, "Dimension is too big! What are you thinking!?\n");
			return 1;
		}
		if(c == ' '){
			hstr[nstr] = '\0';
			break;
		}
		hstr[nstr++] = c;
	}
	while(1){
		c = fgetc(in);
		if(c == EOF){
			fprintf(stderr, "File ended while reading dimensions.\n");
			return 1;
		}
		if(c != ' '){
			break;
		}
	}
	nstr = 0;
	while(1){
		if(c == EOF){
			fprintf(stderr, "File ended while reading dimensions.\n");
			return 1;
		}
		if(nstr == 12){
			fprintf(stderr, "Dimension is too big! What are you thinking!?\n");
			return 1;
		}
		if(c == '\n'){
			wstr[nstr] = '\0';
			break;
		}
		wstr[nstr++] = c;
		c = fgetc(in);
	}
	
	mx = atoi(wstr);
	my = atoi(hstr);
	
	fprintf(stderr, "width by height = %d x %d\n", mx, my);
	
	if(sx == -1){
		sx = 0;
		sy = my - 1;
		ex = mx - 1;
		ey = 0;
	} else {
		if(sx < 0 || sx > mx - 1 ||
		   ex < 0 || ex > mx - 1 ||
		   sy < 0 || sy > my - 1 ||
		   ey < 0 || ey > my - 1){
			fprintf(stderr, "Invalid start/end coordinates.\n");
			return 1;
		}
	}
	
	if(alloc_maze()){
		return 1;
	}
	
	fprintf(stderr, "Parsing...\n");
	#ifndef __MACH__
	clock_gettime(CLOCK_ID, &t_zero);
	#endif
	
	if(parse_maze(in)){
		return 1;
	}
	
	fprintf(stderr, "Initializing heap (%d nodes)...\n", HRI);
	#ifndef __MACH__
	clock_gettime(CLOCK_ID, &t_parse);
	#endif
	
	nh = 1;
	ah = HRI;
	ohx = malloc(ah * sizeof(int));
	ohy = malloc(ah * sizeof(int));
	ohf = malloc(ah * sizeof(h_t));
	ohx[0] = ex;
	ohy[0] = ey;
	ohf[0] = dist(ex,ey,sx,sy);
	m[ey][ex].gscore = 0;
	m[ey][ex].state = 1;
	
	fprintf(stderr, "Solving (%d, %d) -> (%d, %d)...\n", sx, sy, ex, ey);
	#ifndef __MACH__
	clock_gettime(CLOCK_ID, &t_initheap);
	#endif
	
	int x, y, solved = 0;
	node *n, *tn;
	int d;
	int nx, ny;
	int tg;
	int better;
	int hcur;
	int hchild;
	int hswap;
	int tempx, tempy;
	h_t tempf;
	while(nh){
		//printf("Heap length: %d\n", nh);
		//printf("Heap:\n");
		//for(hcur = 0; hcur < nh; hcur++){
		//	printf("  (%d, %d, %.3lf)\n", ohx[hcur], ohy[hcur], ohf[hcur]);
		//}
		x = ohx[0];
		y = ohy[0];
		//fprintf(stderr, "Looking at (%d, %d)\n", x, y);
		n = &(m[y][x]);
		n->state = 2;
		
		if(x == sx && y == sy){
			solved = 1;
			break;
		}
		
		// Remove n from the heap, replace it at the root with the last node.
		ohx[0] = ohx[nh - 1];
		ohy[0] = ohy[nh - 1];
		ohf[0] = ohf[nh - 1];
		m[ohy[0]][ohx[0]].hindex = 0;
		
		nh--;
		
		// Sift the new root back up.
		hcur = 0;
		while(2 * hcur + 1 < nh){
			hchild = 2 * hcur + 1;
			hswap = hcur;
			if(ohf[hswap] > ohf[hchild]){
				hswap = hchild;
			}
			if(hchild + 1 < nh && ohf[hswap] > ohf[hchild + 1]){
				hswap = hchild + 1;
			}
			if(hswap != hcur){
				tempx      = ohx[hcur];
				tempy      = ohy[hcur];
				tempf      = ohf[hcur];
				ohx[hcur ] = ohx[hswap];
				ohy[hcur ] = ohy[hswap];
				ohf[hcur ] = ohf[hswap];
				ohx[hswap] = tempx;
				ohy[hswap] = tempy;
				ohf[hswap] = tempf;
				m[ohy[hcur ]][ohx[hcur ]].hindex = hcur;
				m[ohy[hswap]][ohx[hswap]].hindex = hswap;
				hcur = hswap;
				hswaps++;
			} else {
				break;
			}
		}
		
		for(d = 1; d < 16; d <<= 1){
			if(!(n->neighbors & d)){
				continue;
			}
			switch(d){
				case 1:
					nx = x;
					ny = y - 1;
					break;
				case 2:
					nx = x + 1;
					ny = y;
					break;
				case 4:
					nx = x;
					ny = y + 1;
					break;
				case 8:
					nx = x - 1;
					ny = y;
					break;
			}
			tn = &(m[ny][nx]);
			if(tn->state & 2){
				continue;
			}
			tg = n->gscore + 1;
			if(tn->state == 0){
				tn->state = 1;
				if(nh == ah){
					/*if(check_heapness()){
						fprintf(stderr, "HEAPFAIL\n");
					} else {
						fprintf(stderr, "heappass\n");
					}*/
					ah += HRI;
					fprintf(stderr, "Expanding heap to %d nodes, %d swaps so far.\n", ah, hswaps);
					ohx = realloc(ohx, ah * sizeof(int));
					ohy = realloc(ohy, ah * sizeof(int));
					ohf = realloc(ohf, ah * sizeof(h_t));
					if(ohx == NULL || ohy == NULL || ohf == NULL){
						fprintf(stderr, "Heap realloc failed.\n");
						return 1;
					}
				}
				ohx[nh] = nx;
				ohy[nh] = ny;
				tn->hindex = nh;
				nh++;
				better = 1;
			} else
			if(tg < tn->gscore){
				better = 1;
			} else {
				better = 0;
			}
			if(better){
				tn->parent = ((d << 2) | (d >> 2)) & 15;
				tn->gscore = tg;
				tn->hscore = dist(nx,ny,sx,sy);
				ohf[tn->hindex] = tg + tn->hscore;
				
				hcur = tn->hindex;
				while(hcur > 0){
					hswap = (hcur - 1) / 2;
					if(ohf[hswap] > ohf[hcur]){
						tempx      = ohx[hcur];
						tempy      = ohy[hcur];
						tempf      = ohf[hcur];
						ohx[hcur ] = ohx[hswap];
						ohy[hcur ] = ohy[hswap];
						ohf[hcur ] = ohf[hswap];
						ohx[hswap] = tempx;
						ohy[hswap] = tempy;
						ohf[hswap] = tempf;
						tn->hindex = hcur;
						m[ohy[hswap]][ohx[hswap]].hindex = hswap;
						hcur = hswap;
						hswaps++;
					} else {
						break;
					}
				}
			}
		}
	}
	
	#ifndef __MACH__
	clock_gettime(CLOCK_ID, &t_solve);
	#endif
	
	if(!solved){
		fprintf(stderr, "No path exists.\n");
	} else {
		fprintf(stderr, "Solved (length %d).\n", m[sy][sx].gscore);
		if(m[sy][sx].gscore > 100){
			fprintf(stderr, "The solution is longer than I want to print to stdout.\n"
			                "  You may find it in solution.txt\n");
			FILE *sf = fopen("solution.txt", "w");
			if(sf == NULL){
				fprintf(stderr, "fopen() on solution.txt failed (%m).\n");
			} else {
				print_solution(sx, sy, ex, ey, sf);
			}
			fclose(sf);
		} else {
			print_solution(sx, sy, ex, ey, stdout);
		}
		calc_results(sx, sy, ex, ey);
		int termwidth;
		#ifdef FANCY_TERM
		if(isttyo){
			struct winsize termsize;
			ioctl(0, TIOCGWINSZ, &termsize);
			termwidth = termsize.ws_col;
		} else {
			termwidth = -1;
		}
		#else
		termwidth = 80;
		#endif
		fprintf(stderr, "Termwidth: %d\n", termwidth);
		if(termwidth > 0 && 2 * mx > termwidth){
			fprintf(stderr, "The maze is too wide to be printed on your terminal.\n"
			                "  I am therefore eliding the graphical representation.\n");
		} else {
			print_graphic_solution(sx, sy, ex, ey);
		}
	}
	
	#ifndef __MACH__
	clock_gettime(CLOCK_ID, &t_path);
	
	printdiff("get dimensions ", t_start     , t_dimensions);
	printdiff("allocate memory", t_dimensions, t_malloc    );
	printdiff("zero memory    ", t_malloc    , t_zero      );
	printdiff("parse file     ", t_zero      , t_parse     );
	printdiff("init the heap  ", t_parse     , t_initheap  );
	printdiff("solve the maze ", t_initheap  , t_solve     );
	printdiff("display results", t_solve     , t_path      );
	printdiff("do everything  ", t_start     , t_path      );
	#else
	fprintf(stderr, "Mac OSX does not support clock_gettime(), so I didn't time anything.\n");
	#endif
	
	free(m[0]);
	free(m);
	free(ohx);
	free(ohy);
	free(ohf);
	if(in != stdin){
		fclose(in);
	}
	
	return 0;
}

#ifndef __MACH__
void timespec_diff(struct timespec *s, struct timespec *e, struct timespec *o){
	if((e->tv_nsec - s->tv_nsec) < 0){
		o->tv_sec  =              e->tv_sec  - s->tv_sec - 1;
		o->tv_nsec = 1000000000 + e->tv_nsec - s->tv_nsec;
	} else {
		o->tv_sec  = e->tv_sec  - s->tv_sec;
		o->tv_nsec = e->tv_nsec - s->tv_nsec;
	}
}
#endif

int alloc_maze(void){
	int i;
	unsigned long long l = (unsigned long long) my * sizeof(node *)
	                     + (unsigned long long) my * mx * sizeof(node);
	if(l > 0xffffffffu && sizeof(size_t) == 4){
		fprintf(stderr, "Size of malloc (0x%08llx,%08llx or 0d%llu bytes) "
		                "overflows a 32 bit unsigned int (size_t).\n",
		        l >> 32, l & 0xffffffffu, l);
		return 1;
	}
	if(sizeof(node *) == 8){
		fprintf(stderr, "malloc()'ing 0x%08llx,%08llx (0d%llu) bytes (%d rows x %d columns x %lu bytes per node)...\n",
		                l >> 32, l & 0xffffffffu, l,
		                my, mx, sizeof(node));
	} else {
		fprintf(stderr, "malloc()'ing 0x%08llx (0d%llu) bytes (%d rows x %d columns x %lu bytes per node)...\n",
		                l, l,
		                my, mx, sizeof(node));
	}
	#ifndef __MACH__
	clock_gettime(CLOCK_ID, &t_dimensions);
	#endif
	
	m = malloc(my * sizeof(node *));
	if(m == NULL){
		fprintf(stderr, "Initial malloc() failed (%m).\n");
		return 1;
	}
	m[0] = malloc(my * mx * sizeof(node));
	if(m[0] == NULL){
		fprintf(stderr, "Full malloc() failed (%m).\n");
		return 1;
	}
	for(i = 1; i < my; i++){
		m[i] = m[0] + i * mx;
	}
	
	fprintf(stderr, "Zeroing out...\n");
	#ifndef __MACH__
	clock_gettime(CLOCK_ID, &t_malloc);
	#endif
	
	memset(m[0], 0, my * mx * sizeof(node));
	
	return 0;
}

int parse_maze(FILE *in){
	int i, j, r;
	char *lbuf = malloc((4 * mx - 2) * sizeof(char));
	if(lbuf == NULL){
		fprintf(stderr, "Line buffer malloc() failed.\n");
		return 1;
	}
	for(i = 0; i < my - 1; i++){
		r = fread(lbuf, sizeof(char), 4 * mx - 2, in);
		lbuf[4 * mx - 3] = '\0';
		if(r < 4 * mx - 2){
			fprintf(stderr, "File ended prematurely (%d bytes of %d expected): '%s'.\n", r, 4 * mx + 1, lbuf);
			return 1;
		}
		for(j = 0; j < mx - 1; j++){
			if(lbuf[4 * j + 2] == '.'){
				m[i][j    ].neighbors |= 2;
				m[i][j + 1].neighbors |= 8;
			}
		}
		r = fread(lbuf, sizeof(char), 4 * mx - 2, in);
		lbuf[4 * mx - 3] = '\0';
		if(r < 4 * mx - 2){
			fprintf(stderr, "File ended prematurely (%d bytes of %d expected): '%s'.\n", r, 4 * mx + 1, lbuf);
			return 1;
		}
		for(j = 0; j < mx; j++){
			if(lbuf[4 * j] == '.'){
				m[i    ][j].neighbors |= 4;
				m[i + 1][j].neighbors |= 1;
			}
		}
	}
	r = fread(lbuf, sizeof(char), 4 * mx - 2, in);
	lbuf[4 * mx - 3] = '\0';
	if(r < 4 * mx - 2){
		fprintf(stderr, "File ended prematurely (%d bytes of %d expected): '%s'.\n", r, 4 * mx + 1, lbuf);
		return 1;
	}
	for(j = 0; j < mx - 1; j++){
		if(lbuf[4 * j + 2] == '.'){
			m[i][j    ].neighbors |= 2;
			m[i][j + 1].neighbors |= 8;
		}
	}
	free(lbuf);
	return 0;
}

void calc_results(int sx, int sy, int ex, int ey){
	int x = sx;
	int y = sy;
	while(x != ex || y != ey){
		m[y][x].state |= 4;
		switch(m[y][x].parent){
			case 1:
				y--;
				break;
			case 2:
				x++;
				break;
			case 4:
				y++;
				break;
			case 8:
				x--;
				break;
		}
	}
	m[y][x].state |= 4;
	int i, j;
	for(i = 0; i < my; i++){
		for(j = 0; j < mx; j++){
			if(m[i][j].state & 4){
				sc[0]++;
			} else
			if(m[i][j].state & 2){
				sc[1]++;
			} else
			if(m[i][j].state & 1){
				sc[2]++;
			} else {
				sc[3]++;
			}
		}
	}
	fprintf(stderr, "Heap swaps     : %u\n", hswaps);
	fprintf(stderr, "Path      nodes: %llu\n", sc[0]);
	fprintf(stderr, "Closed    nodes: %llu\n", sc[1]);
	fprintf(stderr, "Open      nodes: %llu\n", sc[2]);
	fprintf(stderr, "Unvisited nodes: %llu\n", sc[3]);
	fprintf(stderr, "Total     nodes: %llu\n", sc[0] + sc[1] + sc[2] + sc[3]);
}

void print_graphic_solution(int sx, int sy, int ex, int ey){
	char *reprs[16] = {"  ", "╵ ", "╶─", "└─",
	                   "╷ ", "│ ", "┌─", "├─",
	                   "╴ ", "┘ ", "──", "┴─",
	                   "┐ ", "┤ ", "┬─", "┼─"};
	
	char *beprs[16] = {"  ", "╹ ", "╺━", "┗━",
	                   "╻ ", "┃ ", "┏━", "┣━",
	                   "╸ ", "┛ ", "━━", "┻━",
	                   "┓ ", "┫ ", "┳━", "╋━"};
	int i, j;
	int cstate = 0;
	for(i = 0; i < my; i++){
		for(j = 0; j < mx; j++){
			if(m[i][j].state & 4){
				#ifdef FANCY_TERM
				if(isttyo && cstate != 1){ printf("%s", tcolors[1]); cstate = 1; }
				#endif
				printf("%s", beprs[(int) m[i][j].neighbors]);
			} else
			if(m[i][j].state & 2){
				#ifdef FANCY_TERM
				if(isttyo && cstate != 2){ printf("%s", tcolors[2]); cstate = 2; }
				#endif
				printf("%s", beprs[(int) m[i][j].neighbors]);
			} else
			if(m[i][j].state & 1){
				#ifdef FANCY_TERM
				if(isttyo && cstate != 3){ printf("%s", tcolors[3]); cstate = 3; }
				#endif
				printf("%s", beprs[(int) m[i][j].neighbors]);
			} else {
				#ifdef FANCY_TERM
				if(isttyo && cstate != 0){ printf("%s", tcolors[0]); cstate = 0; }
				#endif
				printf("%s", reprs[(int) m[i][j].neighbors]);
			}
		}
		printf("\n");
	}
	#ifdef FANCY_TERM
	if(isttyo){
		printf("%s", tcolors[0]);
		fflush(stdout);
	}
	#endif
}

void print_solution(int sx, int sy, int ex, int ey, FILE *f){
	int x = sx;
	int y = sy;
	while(x != ex || y != ey){
		fprintf(f, "(%d, %d)\n", x, y);
		switch(m[y][x].parent){
			case 1:
				y--;
				break;
			case 2:
				x++;
				break;
			case 4:
				y++;
				break;
			case 8:
				x--;
				break;
		}
	}
	fprintf(f, "(%d, %d)\n", x, y);
}

void print_maze(void){
	char *reprs[16] = {"  ", "╵ ", "╶─", "└─",
	                   "╷ ", "│ ", "┌─", "├─",
	                   "╴ ", "┘ ", "──", "┴─",
	                   "┐ ", "┤ ", "┬─", "┼─"};
	int i, j;
	for(i = 0; i < my; i++){
		for(j = 0; j < mx; j++){
			printf("%s", reprs[(int) m[i][j].neighbors]);
		}
		printf("\n");
	}
}

int check_heapness(void){
	int i;
	for(i = 0; i < nh; i++){
		if(ohf[i] < ohf[(i - 1) / 2]){
			return 1;
		}
	}
	return 0;
}

void print_help(void){
	fprintf(stderr, "Usage: ./solvemaze FILE [START_X] [START_Y] [END_X] [END_Y]\n"
	                "\tFILE can be - to read from stdin.\n"
	                "\tLeaving the starting and ending coordinates out will\n"
	                "\tautomatically choose the bottom left and top right\n"
	                "\tcorners, respectively.\n");
}
