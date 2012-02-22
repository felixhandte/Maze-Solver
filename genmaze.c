/* Author: Felix Handte */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int mx;
int my;
char **maze;
FILE *rf;
FILE *of;

int odds;

void random_connections(void);
void depth_first(void);

void alloc_maze(void);
void print_maze(void);
int randdir(int opts);

int main(int argc, char *argv[]){
	if(argc != 5 && argc != 6){
		printf("Usage: ./genmaze OUTPUT_FILE ALGORITHM WIDTH HEIGHT [RANDOMNESS]\n"
		       "\n"
		       "ALGORITHM: rand OR dfs\n"
		       "RANDOMNESS: odds of adding a(n extra, for dfs) connection, out of 256\n\n");
		return 0;
	}
	if(!strcmp(argv[1], "-")){
		of = stdout;
	} else {
		of = fopen(argv[1], "w");
	}
	mx = atoi(argv[3]);
	my = atoi(argv[4]);
	if(argc == 6){
		odds = atoi(argv[5]);
	} else {
		if(!strcmp(argv[2], "rand")){
			odds = 128;
		} else {
			odds = 0;
		}
	}
	rf = fopen("/dev/urandom", "r");
	fprintf(stderr, "Allocating...\n");
	alloc_maze();
	fprintf(stderr, "Generating...\n");
	if(!strcmp(argv[2], "rand")){
		random_connections();
	} else
	if(!strcmp(argv[2], "dfs")){
		depth_first();
	} else {
		printf("Invalid algorithm.\n");
		return 0;
	}
	fprintf(stderr, "Printing...\n");
	print_maze();
	fprintf(stderr, "Done.\n");
	return 0;
}

void depth_first(void){
	int *xstack = malloc(mx * my * sizeof(int));
	int *ystack = malloc(mx * my * sizeof(int));
	xstack[0] = 0;
	ystack[0] = 0;
	int sp = 0;
	int x, y;
	int dir;
	while(sp >= 0){
		if(sp >= mx * my){
			return;
		}
		x = xstack[sp];
		y = ystack[sp];
		//printf("sp = %d, (x, y) = (%d, %d)\n", sp, x, y);
		if((y == 0      || maze[y - 1][x]) &&
		   (y == my - 1 || maze[y + 1][x]) &&
		   (x == 0      || maze[y][x - 1]) &&
		   (x == mx - 1 || maze[y][x + 1])){
			sp--;
			continue;
		}
		dir = randdir(((y > 0      && !maze[y - 1][x]) << 3) |
		              ((y < my - 1 && !maze[y + 1][x]) << 1) |
		              ((x > 0      && !maze[y][x - 1]) << 2) |
		              ((x < mx - 1 && !maze[y][x + 1]) << 0));
		sp++;
		//printf("dir = %d\n", dir);
		maze[y][x] |= dir;
		switch(dir){
			case 1:
				maze[y    ][x + 1] |= 4;
				xstack[sp] = x + 1;
				ystack[sp] = y;
				break;
			case 2:
				maze[y + 1][x    ] |= 8;
				xstack[sp] = x;
				ystack[sp] = y + 1;
				break;
			case 4:
				maze[y    ][x - 1] |= 1;
				xstack[sp] = x - 1;
				ystack[sp] = y;
				break;
			case 8:
				maze[y - 1][x    ] |= 2;
				xstack[sp] = x;
				ystack[sp] = y - 1;
				break;
			default:
				fprintf(stderr, "INVALID DIRECTION.\n");
				break;
		}
	}
	if(odds){
		random_connections();
	}
}

void random_connections(void){
	int i, j;
	for(i = 0; i < my; i++){
		for(j = 0; j < mx; j++){
			maze[i][j] |= (fgetc(rf) < odds) * 2 + (fgetc(rf) < odds);
			if(j < mx - 1 && (maze[i][j] & 1)){
				maze[i][j + 1] |= 4;
			}
			if(i < my - 1 && (maze[i][j] & 2)){
				maze[i + 1][j] |= 8;
			}
		}
	}
}

void alloc_maze(void){
	int i, j;
	maze    = malloc(my *      sizeof(char *));
	maze[0] = malloc(my * mx * sizeof(char  ));
	for(i = 0; i < my; i++){
		maze[i] = maze[0] + i * mx;
		for(j = 0; j < mx; j++){
			maze[i][j] = 0;
		}
	}
}

void print_maze(void){
	int i, j;
	fprintf(of, "%d %d\n", my, mx);
	for(i = 0; i < my - 1; i++){
		for(j = 0; j < mx - 1; j++){
			if(maze[i][j] & 1){
				fprintf(of, "O . ");
			} else {
				fprintf(of, "O | ");
			}
		}
		fprintf(of, "O\n");
		for(j = 0; j < mx - 1; j++){
			if(maze[i][j] & 2){
				fprintf(of, ".   ");
			} else {
				fprintf(of, "-   ");
			}
		}
		if(maze[i][j] & 2){
			fprintf(of, ".\n");
		} else {
			fprintf(of, "-\n");
		}
	}
	for(j = 0; j < mx - 1; j++){
		if(maze[i][j] & 1){
			fprintf(of, "O . ");
		} else {
			fprintf(of, "O | ");
		}
	}
	fprintf(of, "O\n");
}

int randdir(int opts){
	//printf("opts = %d\n", opts);
	int dir = 0;
	char bitcounts[16] = {0,1,1,2,
	                      1,2,2,3,
	                      1,2,2,3,
	                      2,3,3,4};
	unsigned int rb = fgetc(rf);
	switch(bitcounts[opts]){
		case 0:
			//fprintf(stderr, "NO NEIGHBORS TO SUPPOSEDLY OPEN CELL!\n");
			break;
		case 1:
			dir = opts;
			break;
		case 2:
			if(rb & 1){
				if(opts & 1){
					if(opts & 2){
						dir = 2;
					} else
					if(opts & 4){
						dir = 4;
					} else {
						dir = 8;
					}
				} else
				if(opts & 2){
					if(opts & 4){
						dir = 4;
					} else {
						dir = 8;
					}
				} else {
					dir = 8;
				}
			} else {
				if(opts & 1){
					dir = 1;
				} else
				if(opts & 2){
					dir = 2;
				} else {
					dir = 4;
				}
			}
			break;
		case 3:
			while(rb == 0xFF){
				rb = fgetc(rf);
			}
			if(~opts & 1){
				dir = 2 << (rb % 3);
			} else
			if(~opts & 2){
				switch(rb % 3){
					case 0:
						dir = 1;
						break;
					case 1:
						dir = 4;
						break;
					case 2:
						dir = 8;
						break;
				}
			} else
			if(~opts & 4){
				switch(rb % 3){
					case 0:
						dir = 1;
						break;
					case 1:
						dir = 2;
						break;
					case 2:
						dir = 8;
						break;
				}
			} else {
				dir = 1 << (rb % 3);
			}
			break;
	}
	return dir;
}
