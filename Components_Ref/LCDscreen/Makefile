
all:
#For LCD_dislay.c
	arm-linux-gnueabihf-gcc -Wall -g -std=c99 -pthread -D _POSIX_C_SOURCE=200809L LCD_display.c -o LCD_display
	cp LCD_display $(HOME)/cmpt433/public/myApps/

#For Testing_LCD.c
#	arm-linux-gnueabihf-gcc -Wall -g -std=c99 -pthread -D _POSIX_C_SOURCE=200809L Testing_LCD.c -o Testing_LCD
#	cp Testing_LCD $(HOME)/cmpt433/public/myApps/

clean:

	rm -f LCD_display 
#	rm -f Testing_LCD
