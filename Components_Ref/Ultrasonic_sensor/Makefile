all:
	arm-linux-gnueabihf-gcc -Wall -g -std=c99 -pthread -D _POSIX_C_SOURCE=200809L ultrasonic.c -o ultrasonic
	cp ultrasonic $(HOME)/cmpt433/public/myApps/

clean:
	rm -f ultrasonic 