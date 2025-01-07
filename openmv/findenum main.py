import sensor, image, time, os, tf, math, uos, gc,ustruct,pyb
from pyb import LED
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((240, 240))
sensor.skip_frames(time=2000)
sensor.set_vflip(True)
sensor.set_hmirror(True)
net = None
labels = None
min_confidence = 0.5
try:
    net = tf.load("trained3.tflite", load_to_fb=uos.stat('trained3.tflite')[6] > (gc.mem_free() - (64*1024)))
except Exception as e:
    raise Exception('Failed to load "trained3.tflite", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')
try:
    labels = [line.rstrip('\n') for line in open("labels.txt")]
except Exception as e:
    raise Exception('Failed to load "labels.txt", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')
colors = [
    (255,   0,   0),
    (  0, 255,   0),
    (255, 255,   0),
    (  0,   0, 255),
    (255,   0, 255),
    (  0, 255, 255),
    (255, 255, 255),
    (255,   100, 255),
    (  100, 255, 255),
    (255, 255, 100),
]
center_x =120
center_y =0
Task_Flag=1
Target_num=0
lor=0
num=0
red_led=LED(1)
green_led=LED(2)
blue_led=LED(3)
x=0
data = [0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
uart = pyb.UART(3,9600,bits=8, parity=None, stop=1, timeout_char = 1000)
rx_buff=[]
state = 0
tx_flag = 0
def UartReceiveDate():
    global Task_Flag
    global Target_num
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
        Target_num = data[x+2]
        Task_Flag =  Task_Flag - 48
        Target_num =Target_num-48
        x = 0
    elif x >= 5: x = 0
    x+=1
def firstdetect():
    global Task_Flag
    global num
    if (Task_Flag==1):
        result=net.detect(img, thresholds=[(math.ceil(0.8 * 255), 255)])
        num=0
        for i, detection_list in enumerate(result):
            if (i == 0): continue
            if (len(detection_list) == 0): continue
            for d in detection_list:
                    [x, y, w, h] = d.rect()
                    tem_x = math.floor(x + (w / 2))
                    tem_y = math.floor(y + (h / 2))
                    img.draw_circle((tem_x, tem_y, 12), color=colors[i], thickness=2)
            num=int((labels[i])[len(labels[i])-1])
    else:
        return 0
def nextdetect(num):
    global center_x
    global center_y
    global Task_Flag
    if Task_Flag==2:
        for i, detection_list in enumerate(net.detect(img, thresholds=[(math.ceil(min_confidence * 255), 255)])):
            if (i == 0): continue
            if (len(detection_list) == 0): continue
            if (i==num+1):
                for d in detection_list:
                    [x, y, w, h] = d.rect()
                    center_x = math.floor(x + (w / 2))
                    center_y = math.floor(y + (h / 2))
                    img.draw_circle((center_x, center_y, 12), color=colors[i], thickness=2)
def LOR():
    global center_x
    if(center_x<110):
        lor=1
    elif(center_x>130):
        lor=2
    else:
        lor=0
    return lor
clock = time.clock()
cnt_send=0
cnt=0
cnt1=0
lit=[]
lit1=[]
nn=0
nm=0

while(True):
    clock.tick()
    img = sensor.snapshot()
    img.draw_line(120, 0, 120, 240, color = (100, 100, 100), thickness = 2)
    UartReceiveDate()
    if Task_Flag==1:
        red_led.on()
        blue_led.off()
        firstdetect()
        if num and cnt1<20:
            lit1.append(num)
            cnt1+=1
            if lit1.count(num)>=10:
                print(num)
                sent2=bytearray([0x2e,0x99,num,lor,0xee])
                uart.write(sent2)
                cnt1=0
    if Task_Flag==2:
        red_led.off()
        if nn==0:
             time.sleep_ms(1000)
             nn=1
        blue_led.on()
        nextdetect(Target_num)
        lor=LOR()
        if lor and cnt<20:
            lit.append(lor)
            cnt+=1
            if lit.count(lor)>=5:
                sent2 = bytearray([0x2e,0x99,num,lor,0xee])
                uart.write(sent2)
                nm+=1
                print(lor,nm,Target_num)
                cnt=0
