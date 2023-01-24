BINARY = player
CSTD = -std=c2x

LDSCRIPT = build/nucleo-f411re.ld

.DEFAULT_GOAL := all

player.o: sound_funny.raw11.cdata

%.raw11: %.raw16
	./reduce_sample_size.py --input $(*).raw16 --output $(*).raw11 --n 11

%.raw11.cdata: %.raw11
	./make_cdata.py --input $(*).raw11 --output $(*).raw11.cdata

include build/Makefile.include


