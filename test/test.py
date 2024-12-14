import serial

ser = serial.Serial("COM4", 115200)

x = ser.read();
print(x)
ser.write(x)