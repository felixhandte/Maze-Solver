UNAME := $(shell uname)

all : genmaze solvemaze genmaze-optimized solvemaze-optimized


genmaze : genmaze.c
	gcc -ggdb -Wall genmaze.c            -o genmaze

solvemaze : solvemaze.c
ifeq ($(UNAME), Linux)
	gcc -ggdb -Wall solvemaze.c -lm -lrt -o solvemaze
endif
ifeq ($(UNAME), Darwin)
	gcc -ggdb -Wall solvemaze.c -lm      -o solvemaze
endif

genmaze-optimized : genmaze.c
	gcc -march=native -O6 -Wall genmaze.c            -o genmaze-optimized

solvemaze-optimized : solvemaze.c
ifeq ($(UNAME), Linux)
	gcc -march=native -O6 -Wall solvemaze.c -lm -lrt -o solvemaze-optimized
endif
ifeq ($(UNAME), Darwin)
	gcc -march=native -O6 -Wall solvemaze.c -lm      -o solvemaze-optimized
endif


clean :
	rm -f genmaze solvemaze genmaze-optimized solvemaze-optimized *~

force : clean all