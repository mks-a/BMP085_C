:: -g generate debug information
:: -c compile only
cls
"D:\Arduino\arduino_1.8.3\hardware\tools\avr/bin/avr-gcc" -c -g -Os -w -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10801 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR .\code\main.c -o tmp\main.o
"D:\Arduino\arduino_1.8.3\hardware\tools\avr/bin/avr-gcc" -c -g -Os -w -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10801 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR .\bmp085\bmp085.c -o tmp\bmp085.o
"D:\Arduino\arduino_1.8.3\hardware\tools\avr/bin/avr-gcc" -c -g -Os -w -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10801 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR .\timer1\timer1.c -o tmp\timer1.o
"D:\Arduino\arduino_1.8.3\hardware\tools\avr/bin/avr-gcc" -c -g -Os -w -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10801 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR .\twi\twi.c -o tmp\twi.o
"D:\Arduino\arduino_1.8.3\hardware\tools\avr/bin/avr-gcc" -c -g -Os -w -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10801 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR .\usart_dbg\usart_dbg.c -o tmp\usart_dbg.o
"D:\Arduino\arduino_1.8.3\hardware\tools\avr/bin/avr-gcc" -mmcu=atmega328p -o main.elf tmp\main.o tmp\bmp085.o tmp\timer1.o tmp\twi.o tmp\usart_dbg.o -lm -emain
"D:\Arduino\arduino_1.8.3\hardware\tools\avr/bin/avr-objcopy" -O ihex -R .eeprom  main.elf main.hex
