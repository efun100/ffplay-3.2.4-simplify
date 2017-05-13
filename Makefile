ffplay:*.c *.h
	cc ffplay.c cmdutils.c -lSDL2 -lavformat -lavcodec -lavutil -lswscale -lswresample -lavdevice -lavfilter -lm -o ffplay -Wall

clean:
	rm ffplay

