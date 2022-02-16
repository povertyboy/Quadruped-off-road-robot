import sensor, image, time, math, utime
from pyb import UART
from pyb import LED

start = time.ticks()
ir_led = LED(4)
uart = UART(3, 9600)
uart.init(9600, bits=8, parity=None, stop=1)  # init with given parameters

# Tracks a black line. Use [(128, 255)] for a tracking a white line.
red_threshold =[(0, 100, -76, 22, -120, 127)]#[(0, 100, -115, 23, -126, 127)]#[(100, 0, 127, 20, 127, 6)]
black_threshold = (0, 40, -32, 48, -120, 127)#(0, 49, -69, 31, -70, 78)
 #红色反向，黑色没有
ROIS_B = [(0, 100, 320, 140)]

ROIS = [
    #(0, 50, 320, 20, 0.1),#0.1
    #(0, 120, 320, 50, 0.15),#0.2
    (50, 180, 220, 45, 1),
]
# ROIS = [
#                     #(0, 35, 320, 45, 0.1),
#                     (0, 100, 320, 50, 0.15),
#                     (0, 190, 320, 45, 0.85),
#                 ]
# ROIS = [
#                 (0, 100, 320, 50, 0.15),
#                (0, 190, 320, 45, 0.85),
#             ]
# roi代表三个取样区域，（x,y,w,h,weight）,代表左上顶点（x,y）宽高分别为w和h的矩形，
# weight为当前矩形的权值。使用的QVGA图像大小为320x240，roi即把图像横分成三个矩形。


weight_sum = 0  # 权值和初始化
for r in ROIS: weight_sum += r[4]  # r[4] is the roi weight.

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()


def RunActionGroup(actNum, num):
    global uart
    buf = bytearray(b'\x55\x55\x05\x06')
    buf.append(0xff & actNum)
    buf.append(0xff & num)
    buf.append(0xff & (num >> 8))
    uart.write(buf)


def StopActionGroup():
    uart.write(b'\x55\x55\x02\x07')


go_straight = 53
Time = 900
go_l = 54
go_r = 55
step = 1 ########1
state = 0
count = 0  #0
state1 = True
black_cy_min = 150   #115
black_cy_max = 225    #190
center_r = 138
center_l = 195
black_change = False
# ir_led.on()
flag = 0#下台标识
count2 = 0
count1 = 0
A=2
B=0   #ZUO
D=0   #YOU
C=0

while (True):
    img = sensor.snapshot()
    centroid_sum_red = 0
    centroid_sum_black = 0

    # 红色,便利三个区域
    for r in ROIS:
        blobs_red = img.find_blobs(red_threshold, roi=r[0:4], merge=True, invert=1)

        if blobs_red:
            # Find the index of the blob with the most pixels.
            most_pixels = 0
            largest_blob = 0
            for i in range(len(blobs_red)):
                # 目标区域找到的颜色块最大的一个，作为本区域内的目标直线
                #print(blobs_red[i].pixels())
                if blobs_red[i].pixels() > most_pixels:  #过滤红色干扰
                    most_pixels = blobs_red[i].pixels()
                    largest_blob = i  # 下标

            # 将此区域的像素数最大的颜色块画矩形和十字形标记出来
            img.draw_rectangle(blobs_red[largest_blob].rect())
            img.draw_cross(blobs_red[largest_blob].cx(),
                           blobs_red[largest_blob].cy())

            centroid_sum_red += blobs_red[largest_blob].cx() * r[4]
            # print(centroid_sum_red, )
            # 计算centroid_sum，centroid_sum等于每个区域的最大颜色块的中心点的x坐标值乘本区域的权值

    # 黑色
    blobs_black = img.find_blobs([black_threshold], roi=ROIS_B[0], merge=True, invert=0)#invert=1
    if blobs_black:
        most_pixels = 0
        largest_blob = 0
        for i in range(len(blobs_black)):
            # 目标区域找到的颜色块最大的一个，作为本区域内的目标直线
            if blobs_black[i].pixels() > most_pixels:
                most_pixels = blobs_black[i].pixels()
                largest_blob = i
                print(most_pixels)  ##打印出pixels

        img.draw_rectangle(blobs_black[largest_blob].rect())
        img.draw_cross(blobs_black[largest_blob].cx(), blobs_black[largest_blob].cy())
        print(blobs_black[largest_blob].cy())
        print(blobs_black[largest_blob].cx())
        if state1 and most_pixels > 1850:  ##设定pixels大于2000避免检测到台阶边缘
            print(most_pixels)######################################打印pixel
            if (blobs_black[largest_blob].cy() > black_cy_min
                    and blobs_black[largest_blob].cy() < black_cy_max
                    and blobs_black[largest_blob].cx() < 200
                    and blobs_black[largest_blob].cx() > 120):
                state1 = False
                black_change = True
                # print(step)
    center_pos_red = centroid_sum_red
    print(count)
    # print(state1)

    #print(black_change)
    if black_change:
        if step == 1:  # 上第一个阶梯   yuanlai8700
            center_r = 120
            center_l = 200   #由于会向左偏，初值是向右较大，此处登台后重新修正
            go_straight = 56
            RunActionGroup(go_straight, 1)
            time.sleep(7300)
            # RunActionGroup(0, 1)
            # time.sleep(200)
            black_cy_min = 139
            step = 2


        elif step == 2:
            flag += 1
            A = 2
            #RunActionGroup(0, 1)  ##防止偏，回原来动作
            #time.sleep(350)
            RunActionGroup(0, 1)  ##防止偏，回高姿态
            time.sleep(300)
            RunActionGroup(57, 1)##########57->41高姿态前进
            time.sleep(6300 ) #3500
            RunActionGroup(0, 1)  ##防止偏，回高姿态
            time.sleep(300)
            center_r = 120
            center_l = 190
            go_straight = 57




        elif step == 3:  # 上高台  10770
            center_r = 140
            center_l = 180
            black_cy_min = 148  #调节此处可以调节下台时台阶距离
            A = 1
            RunActionGroup(31, 2)#先右移两步再上台
            time.sleep(660)
            RunActionGroup(58, 1)#上台动作往左偏
            time.sleep(11000)
            step = 4
            count = 0    #第二弯道参数清0
            count1 = 0
            state1 = True
            ROIS = [
                (0, 50, 320, 20, 0.1),
                (0, 100, 320, 50, 0.2),
                (0, 190, 320, 45, 0.7),
            ]

        elif step == 4 and most_pixels > 4000:  # 下高台 10550
            A = 2
            RunActionGroup(30, 2)  # 先左移两步再下台
            time.sleep(950)
            RunActionGroup(59, 1)
            time.sleep(12000)
            step = 5
            ROIS = [
                (0, 100, 320, 50, 0.15),
               (0, 190, 320, 45, 0.85),
            ]

        elif step == 5:  # 上斜坡
            ROIS = [
                (0, 50, 320, 20, 0.1),
                (0, 100, 320, 50, 0.2),
                (0, 190, 320, 45, 0.7),
            ]
            center_r = 125
            center_l = 195
            A = 3
            go_straight = 9#21   51是抬高动作  52是顿挫动作
            step = 6 #防止执行下面弯道程序
        black_change = False  ##设定退出

    print(center_pos_red)
    if center_pos_red > 0 and center_pos_red < center_r:  # 向左转and step !=2
        if count1 == 0 and flag==0 and count != 0  and step !=6 and (C==0 or count > 38):
            RunActionGroup(30, 2)#左移  避免卡台阶
            time.sleep(665)
            # RunActionGroup(41, 2)  ##########57->41高姿态前进
            # time.sleep(500)
            count1 = 1
        if D!=0:
            RunActionGroup(2, 4)  # 矫正偏航
            if step==3:
                time.sleep(500)
            elif step == 5:
                time.sleep(1150)
            if count >15 and count <17 and step ==5:
                time.sleep(300)
        elif D==0:
            RunActionGroup(go_l, 1)
            time.sleep(900)

        count2 += 1
        if(step==2 and flag !=0 and count2>4):
            RunActionGroup(30, 3)  # 左移  避免卡台阶
            time.sleep(1000)
            count2 = 0


    elif center_pos_red >= center_r and center_pos_red <= center_l:  # 直走
        print(count)
        if step == 2:
            count += 1
            if count >= 4:
                C +=1
                state1 = True
                count = 0
                A = 1
                go_straight = 53
                center_r = 152
                center_l = 178
                RunActionGroup(5, 1)
                time.sleep(4500)
                ROIS = [
                    (0, 50, 320, 20, 0.1),#0.1
                    (0, 120, 320, 50, 0.15),#0.2
                    (0, 180, 320, 45, 0.75),
                ]


            if 1 <= flag <= 3:#高姿态下台
                flag += 1
                RunActionGroup(57, 1)  ##########57->41高姿态前进
                time.sleep(6300)   #5--2800   调节此处时注意弯道会有影响
                # RunActionGroup(25, 1)  ##防止偏，回原来动作
                # time.sleep(300)
                if flag < 2:
                    RunActionGroup(0, 1)  ##防止偏，回高姿态
                    time.sleep(300)
            elif flag == 4:
                center_r = 145
                center_l = 175
                step=3
                A = 2
                C=0
                black_cy_min = 172
                black_cy_max = 225
                # RunActionGroup(41, 50)  ##########57->41高姿态前进
                # time.sleep(4300)
                ROIS = [
                    #(0, 35, 320, 45, 0.1),
                    (0, 100, 320, 50, 0.15),
                    (0, 190, 320, 45, 0.85),
                ]  #复原
                flag=0

        # RunActionGroup(go_straight, 2)
        if not(step == 2 and flag != 0 ) :#or (step) or ():     #防止下台阶直走
            if go_straight == 56:#8700
                RunActionGroup(53, 1)
                time.sleep(900)
                RunActionGroup(go_straight, 1)
                time.sleep(7300)

            elif go_straight==53 or go_straight==9 :#700
                RunActionGroup(go_straight, A)
                if step==2 or step ==4:
                    time.sleep(900)
                elif A==3:
                    time.sleep(2600)
                elif step==3 or step == 5:
                    time.sleep(900)
                else:
                    time.sleep(1700)

        count1 = 0
        count2 = 0

        ####若场地为全白，建议将
        if step == 3:
            print(count)
            count += 1
            A = 2
            B = 1
            D = 1
            if count > 3 and count < 17:
                center_r = 120
                center_l = 215

            if count >= 17:#计步数
                black_change = False
                state1 = True
                black_cy_max = 230
                B=0
                D=0
                A=2


        if step == 5:
            count += 1
            B = 1
            D = 1
            A = 2
            if count > 2 and count < 17 :#7-15 注意下面的步数与此时需要识别黑色的相匹配

                center_r = 105
                center_l = 200

            if count >= 17:  #计步数
                black_change = False
                state1 = True
                A= 1
                B = 0
                D = 0
                #count = 0

        ######这部分去掉

    elif center_pos_red > center_l and center_pos_red < 320:  # 向右转
        if count1 == 0 and flag==0 and count != 0 and step !=6 and (C==0 or count > 38):#右移 避免卡台阶 and step !=2
            RunActionGroup(31, 2)#41 10
            time.sleep(665)
            count1 = 1
        if B!=0:
            RunActionGroup(1, 4)  # 矫正偏航
            if step==3:
                time.sleep(1150)
            if step == 5:
                time.sleep(500)
            if count == 10 and step ==3:
                time.sleep(200)
            if count >15 and count <17 and step ==3:
                time.sleep(300)

        elif B==0:
            RunActionGroup(go_r, 1)
            time.sleep(900)
        count2 += 1
        if (step == 2 and flag != 0 and count2 > 4):
            RunActionGroup(31, 2)  # 右移  避免卡台阶
            time.sleep(665)
            count2 = 0


