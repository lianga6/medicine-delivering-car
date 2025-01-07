# Untitled - By: 李鑫亮 - 周五 5月 26 2023

THRESHOLD = (16, 60, 11, 63, 7, 49)#原本的(20, 50, 21, 62, 7, 45)
THRESHOLD1=(91, 100, -11, 14, -7, 9)
import sensor, image, time,ustruct,pyb
from pyb import LED
import car
from pid import PID
rho_pid = PID(p=0.6, i=0)
theta_pid = PID(p=0.001, i=0)
#LED(1).on()
#LED(2).on()
#LED(3).on()
uart = pyb.UART(3, 9600, timeout_char = 1000)
uart.init(9600,bits=8,parity=None,stop=1)
sensor.reset()
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQQVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()
my_roi =	[(0, 15, 10,45),
            (70,15,10,45),
            (25,45,30,15),
            (0,0,80,60)]


my_roi1 =[  (0,0,50,60),
            (30,0,50,60),
            (0,0,80,60)]

LOR=0

def UartReceiveDate():
    global Task_Flag
    global LOR
    global x
    global data
    data[0] = uart.readchar()
    data[1] = uart.readchar()
    data[2] = uart.readchar()
    data[3] = uart.readchar()
    data[4] = uart.readchar()
    data[5] = uart.readchar()
    data[6] = uart.readchar()
    data[7] = uart.readchar()
    if data[x] == 42 and data[x+3] == 38 and x < 5:
        Task_Flag =  data[x+1]
        LOR= data[x+2]
        Task_Flag =  Task_Flag - 48
        LOR =LOR-48
        x = 0
    elif x >= 5: x = 0
    x+=1


def Follow_line(lor):
    global img
    img = sensor.snapshot().binary([THRESHOLD])
    line = img.get_regression([(100,100)],roi=my_roi1[lor], robust = True)
    if (line):
        rho_err = abs(line.rho())-img.width()/2
        if line.theta()>90:
            theta_err = line.theta()-180
        else:
            theta_err = line.theta()
        img.draw_line(line.line(), color = 127)
        if line.magnitude()>8:
            rho_output = rho_pid.get_pid(rho_err,1)
            theta_output = theta_pid.get_pid(theta_err,1)
            output = rho_output+theta_output
            output_int=int(output)
            if output_int<0:
                output_int=abs(output_int)+100
            FH = bytearray([0xfe,0x2C,0x12,output_int,0x5B])
            uart.write(FH)
            time.sleep_ms(50)
    else:
        FH = bytearray([0xfe,0x2C,0x12,0x00,0x5B])
        uart.write(FH)
        time.sleep_ms(10)


while(True):
    clock.tick()
    UartReceiveDate()
    for i in my_roi:
        img.draw_rectangle(i, color=(255,0,0))
    if LOR==0:
        Follow_line(0)
    elif LOR==1:
        Follow_line(1)
    elif LOR==2:
        Follow_line(2)
