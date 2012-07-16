UNAME := $(shell uname)

CCOPTS := -ggdb -Wall -Wextra -Wno-format -pedantic -std=gnu99 -march=native -O6
LDOPTS := -lm

ifeq ($(UNAME), Linux)
LDOPTS := $(LDOPTS) -lrt
endif

all : genmaze solvemaze

clean :
	rm -f genmaze solvemaze *~

force : clean all

test :
	echo $(DERP)

genmaze : genmaze.c
	gcc $(CCOPTS) genmaze.c   $(LDOPTS) -o genmaze

solvemaze : solvemaze.c
	gcc $(CCOPTS) solvemaze.c $(LDOPTS) -o solvemaze