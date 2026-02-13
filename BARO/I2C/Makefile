bmp3:main.o bmp3.o bmp3_common.o
	gcc -Wall -o bmp3 main.o bmp3.o bmp3_common.o -lwiringPi -std=gnu99
main.o: main.c bmp3.h bmp3_defs.h bmp3_common.h
	gcc -Wall -c main.c -lwiringPi -std=gnu99
bmp3.o: bmp3.c bmp3.h bmp3_defs.h
	gcc -Wall -c bmp3.c -lwiringPi -std=gnu99
bmp3_common.o:bmp3_common.c bmp3_common.h
	gcc -Wall -c bmp3_common.c -lwiringPi -std=gnu99
clean:
	rm bmp3 main.o bmp3.o bmp3_common.o