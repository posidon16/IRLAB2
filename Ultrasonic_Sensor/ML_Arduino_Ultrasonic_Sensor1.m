clear
clc

a = arduino("COM8", "Uno", "Libraries", "Ultrasonic")

ultrasonicObj = ultrasonic(a, "A0", "A1")

while 1
    distance = readDistance(ultrasonicObj)
    pause(0.1)
end