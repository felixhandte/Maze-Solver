UNAME := $(shell uname)

all : genmaze solvemaze


genmaze : genmaze.c
	gcc -ggdb -Wall          -o genmaze   genmaze.c

solvemaze : solvemaze.c
ifeq ($(UNAME), Linux)
	gcc -ggdb -Wall solvemaze.c -lm -lrt -o solvemaze
endif
ifeq ($(UNAME), Darwin)
	gcc -ggdb -Wall solvemaze.c -lm      -o solvemaze
endif


clean :
	rm -f genmaze solvemaze *~
