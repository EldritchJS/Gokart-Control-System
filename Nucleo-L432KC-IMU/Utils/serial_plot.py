import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import serial

ser = serial.Serial()
ser.port = '/dev/ttyUSB0' 
ser.baudrate = 115200
ser.timeout = 30 
ser.open()

fig = plt.figure()
acc = fig.add_subplot(3, 1, 1)
gyr = fig.add_subplot(3, 1, 2)
mag = fig.add_subplot(3, 1, 3)

x_axis = []
acc_x = []
acc_y = []
acc_z = []
gyr_x = []
gyr_y = []
gyr_z = []
mag_x = []
mag_y = []
mag_z = []
index = 0

def animate(i, acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, mag_x, mag_y, mag_z):
    index = index + 1
    line=ser.readline()
    line_as_list = line.split(b',')
    acc_x.append(line_as_list[0])
    acc_y.append(line_as_list[1])
    acc_z.append(line_as_list[2])
    gyr_x.append(line_as_list[3])
    gyr_y.append(line_as_list[4])
    gyr_z.append(line_as_list[5])
    mag_x.append(line_as_list[6])
    mag_y.append(line_as_list[7])
    mag_z.append(line_as_list[8])
    x_axis.append(index)

    acc.clear()
    acc.plot(x_axis, acc_x, label="AccelX")
    acc.plot(x_axis, acc_y, label="AccelY")
    acc.plot(x_axis, acc_z, label="AccelZ")

    gyr.clear()
    gyr.plot(x_axis, gyro_x, label="GyroX")
    gyr.plot(x_axis, gyro_y, label="GyroY")
    gyr.plot(x_axis, gyro_z, label="GyroZ")

    mag.clear()
    mag.plot(x_axis, mag_x, label="MagX")
    mag.plot(x_axis, mag_y, label="MagY")
    mag.plot(x_axis, mag_z, label="MagZ")
    
ani = animation.FuncAnimation(fig, animate, fargs=(acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, mag_x, mag_y, mag_z), interval=100)
plt.show()

