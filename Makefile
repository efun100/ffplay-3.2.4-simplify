ffplay:*.c
	cc ffplay.c -lSDL2 -lavformat -lavcodec -lavutil -lswscale -lswresample -lavdevice -lavfilter -lm -o ffplay -Wall

clean:
	rm ffplay
