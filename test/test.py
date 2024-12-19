import serial
import struct

ser = serial.Serial("COM4", 115200)

fmt = "II"

x = struct.pack(fmt, 2, 0)

ser.write(x)

y = ser.read(8)

print(y)

print(x)

print(x == y)