import serial
import time

ser = serial.Serial(
    port='/dev/ttyUITube',
    baudrate=9600,
    timeout=.1,
)

time.sleep(5)

# commands start with '>'
# and end with '\n'
# all commands have a type: 'l' for led strip, or 'h' for headlamp

# the first three args to the led command are the color
# the first four to the headlamp are the intensity: 1100 pwm (off), 255 is full brightness
# command format: > type arg1 arg2 arg3 arg4 arg5 \n
# led control example. >lf, g byte, r byte, b byte, any byte, terminator

# any time you make a change using a command, that change will stick till you restart the arduino
packet = bytearray([ord('>'), ord('l'), ord('f'), 255, 255, 255, 0, ord('\n')])
print(packet)
ser.write(packet)

time.sleep(5)
# headlamp example. >lf, g byte, r byte, b byte, any byte, terminator
packet = bytearray([ord('>'), ord('h'), 125, 125, 125, 125, 0, ord('\n')])
ser.write(packet)
print(packet)

# type, arg1 to arg5 grb
packet = bytearray([ord('>'), ord('l'), ord('s'), 255, 0, 255, 0, ord('\n')])
print(packet)
ser.write(packet)

time.sleep(1)
packet = bytearray([ord('>'), ord('h'), 125, 0, 0, 0, 0, ord('\n')])
ser.write(packet)
print(packet)

time.sleep(1)
packet = bytearray([ord('>'), ord('h'), 125, 125, 0, 0, 0, ord('\n')])
ser.write(packet)
print(packet)

time.sleep(1)
packet = bytearray([ord('>'), ord('h'), 125, 125, 125, 0, 0, ord('\n')])
ser.write(packet)
print(packet)

time.sleep(1)
packet = bytearray([ord('>'), ord('h'), 125, 125, 125, 125, 0, ord('\n')])
ser.write(packet)
print(packet)

time.sleep(1)
packet = bytearray([ord('>'), ord('h'), 125, 0, 0, 0, 0, ord('\n')])
ser.write(packet)
print(packet)

time.sleep(1)
packet = bytearray([ord('>'), ord('h'), 125, 125, 0, 0, 0, ord('\n')])
ser.write(packet)
print(packet)

# type, arg1 to arg5 grb
packet = bytearray([ord('>'), ord('l'), ord('c'), 255, 255, 0, 0, ord('\n')])
print(packet)
ser.write(packet)

time.sleep(1)
packet = bytearray([ord('>'), ord('h'), 125, 125, 125, 0, 0, ord('\n')])
ser.write(packet)
print(packet)

time.sleep(1)
packet = bytearray([ord('>'), ord('h'), 125, 125, 125, 125, 0, ord('\n')])
ser.write(packet)
print(packet)

ser.close()