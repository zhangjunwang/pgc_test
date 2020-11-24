#!/usr/bin/env python
# coding: utf-8
# ubuntu 18.04 & python2.7
import sys 
# sys.path.append("..")
import serial 
import time 
import rospy 
import math 
import csv 
from reader.msg import pos

'''
    识别tag标签
'''
def check_communication_right(message):
    '''
    校验字节,用于验证端口是否正确收到数据
    '''
    check_statue = 0 
    for i in range(20):
        check_statue ^= int(message[i * 2: i * 2 + 2], 16)
    return check_statue == int(message[40: 42], 16)
    
def status_information(message):
    '''
    状态信息：颜色轨道，位置码带，tag标签
    '''
    pass 
    
def x_position(message):
    '''
    input: string 
    Byte3~6：反馈 X 方向位置值，一共 24 bits
    byte3反映读头识别到颜色轨道的颜色
    byte3_1==byte3 and H07
    x position = byte3_1 * H80 * H4000 + byte4 * H4000 + byte5 * H80 + byte6
    '''
    # byte3_1 = hex(hex(message[4:6]) & hex(0x07))
    byte3_1 = hex(int(message[4:6], 16) & int('7', 16))
    x_pos_value = int(byte3_1, 16) * int('80', 16) * int('4000', 16) + int(message[6:8], 16) * int('4000', 16) + int(message[8:10], 16) * int('80', 16) + int(message[10:12], 16)
    # 33554431==2**25-1
    x_valid_bits = x_pos_value & 33554431
    # 有效位数
    width = 24
    if x_valid_bits > 2 ** (width - 1) - 1:
        x_valid_bits = 2 ** width - x_valid_bits
        return -x_valid_bits
    else:
        return x_valid_bits

def y_position(message):
    '''
    y位置信息
    Byte7~8：反馈 Y方向偏差值，一共 14 bits
    '''
    y_pos_value = int(message[12:14], 16) * int('80', 16) + int(message[14:16], 16)
    # 取14bits的有效位, 32767==2**15-1
    y_valid_bits = y_pos_value & 32767
    # 有效位数
    width = 14 
    if y_valid_bits > 2 ** (width - 1) - 1:
        y_valid_bits = 2 ** width - y_valid_bits
        return -y_valid_bits 
    else:
        return y_valid_bits


def angle_value(message):
    '''
    input: hex string 
    output: int, degree
    '''
    angle_v = int(message[20:22], 16) * int('80', 16) + int(message[22:24], 16)
    return angle_v

def tag_number(message):
    '''
    input: hex string 
    output: int, tag标签号码
    '''
    tag_bit = int(message[2], 16)
    if tag_bit&4:
        tag_num = int(message[28:30], 16) * int('80', 16) * \
            int('4000', 16) + int(message[30:32], 16) * \
            int('4000', 16) + int(message[32:34], 16) * \
            int('80', 16) + int(message[34:36], 16)
        return tag_num
    else:
        return -1 

def warning_message(message):
    '''
    Byte19~20：反馈报警信息
    '''
    byte_1 = int(message[0:2], 16)
    byte_19 = int(message[36:38], 16)
    byte_20 = int(message[38:40], 16)
    # 判断bit2是否为1,bit2为1时,表示可以检测报警信号
    wrn_flag = byte_1 & 4
    # 反馈报警信息
    if wrn_flag:
        # print('Warning!') 
        if byte_20 & 1:
            print("WRN00: NOT STANDARD CODE")
        elif byte_20 & 2:
            print("WRN01: TOO CLOSE")
        elif byte_20 & 4:
            print("WRN02: TOO FAR")
        elif byte_20 & 32:
            print("WRN05:  OVER TOTATE")
        elif byte_20 & 64:
            print("WRN06: LOW CONTRAST")
        elif byte_19 & 1:
            print("WRN07: FIX CODE DETECTED")
        elif byte_19 & 2:
            print("WRN08: HIGH TEMPERATURE")
        elif byte_19 & 4:
            print("WRN09: CLOSE TO CROSSING")
        elif byte_19 & 8:
            print("WRN10: NOT STANDARD CODE")

def read_coord_data(path):
    '''
    读取绝对坐标文件
    文件格式: tag_num, coord_x, coord_y 
    output: coord = [tag_num, coord_x, coord_y], 单位为毫米
    convert_idx_dict: {tag_num: idx}, 将读取到的tag_num转为coord中对应的索引,以字典的形式保存
    '''
    csv_file = csv.reader(open(path, 'r'))
    coord = [] 
    # 保存tag_num在coord数组中对应的索引
    convert_idx_dict = {}
    i = 0 
    # coord_tag_num = [] # 保存所有的tag_num
    for line in csv_file:
        coord.append([int(line[0]), float(line[1]), float(line[2])]) 
        convert_idx_dict[coord[i][0]] = i 
        i += 1 
    return coord, convert_idx_dict

    # 将tag_num映射为数组索引
    # tag_num_min = min(coord_tag_num)
    # tag_num_max = max(coord_tag_num)
    # # convert_idx_tag_num
    # convert_idx_tag_num = [0] * (tag_num_max - tag_num_min + 1) 
    # for i in range(len(coord_tag_num)):
    #     convert_idx_tag_num[coord_tag_num[i] - tag_num_min] = i 
    # return coord, convert_idx_tag_num, tag_num_min

def convert_cord(tag_num, x, y, coord, convert_idx_dict):
    '''
    将读码器读取到的信息转为绝对坐标,输入x, y单位均为毫米
    input: tag_num, x, y, coord, convert_idx_tag_num
    output: 绝对坐标, std_x, std_y 
    '''
    std_x = x + coord[convert_idx_dict[tag_num]][1]
    std_y = y + coord[convert_idx_dict[tag_num]][2]
    return std_x, std_y 

if __name__=="__main__":
    ser = serial.Serial("/dev/ttyUSB0", 115200, parity=serial.PARITY_EVEN)
    if ser.is_open:
        print("port open success")
        # 选择右边轨道
        select_lane_message = 'E41B'
        # left lane
        # select_lane_message = 'E817'
        select_lane_message = select_lane_message.decode('hex')
        # 发送报文
        ser.write(select_lane_message)
        time.sleep(0.1)
        len_receive_message = ser.inWaiting 

        # 定义ros节点,talker
        pub = rospy.Publisher('position_info', pos, queue_size=10)
        # init_node 第一个参数为节点名称，第二个参数 anonymous=True 表示自动在第一个参数后面添加一个随机数，保证节点的唯一性
        rospy.init_node('pytalker', anonymous=True)
        # 节点频率：单位 Hz
        rate = rospy.Rate(10)  # 此处设置的频率不会考虑一个循环运行的时间
        # 定义参数
        state = 'working'

        if len_receive_message:
            # 接收报文,3个字节
            receive_message = ser.read(3)            
            hex_receive_message = receive_message.encode('hex')
            # print(hex_receive_message)

            # 读取绝对坐标数据,文件格式为csv,数据格式为 tag_num, coord_x, coord_y 
            # path = ""
            # coord, convert_idx_dict = read_coord_data(path)
            # 读取数据
            try:               
                while not rospy.is_shutdown():
                    start_time = time.time()
                    # 默认地址为0,指令为'C837'；当地址为1时,指令为'C936'
                    send_code = 'C837'
                    send_code = send_code.decode('hex')
                    ser.write(send_code)
                    # time.sleep(0.1)
                    # 从端口读取21个字节
                    receive_message = ser.read(21)
                    hex_receive_message = receive_message.encode('hex')
                    # print(hex_receive_message)
                    # 判断端口数据是否有效
                    if not check_communication_right(hex_receive_message):
                        print("data error!")
                        continue 
                    # 检测报警信息 
                    warning_message(hex_receive_message)
                    tag_num = tag_number(hex_receive_message)
                    # 获取偏移信息
                    x = x_position(hex_receive_message)
                    y = y_position(hex_receive_message)
                    # 将偏移信息转为绝对坐标
                    # std_x, std_y = convert_cord(tag_num, x, y, coord, convert_idx_dict)
                    std_x, std_y = x, y

                    angle = angle_value(hex_receive_message)
                    stamp = rospy.get_time()
                    rospy.loginfo('Talker: position: tag_num=%d, std_x=%f, std_y= %f, angle= %f', tag_num, std_x, std_y, angle)
                    pub.publish(pos(state, tag_num, std_x, std_y, angle, stamp))
                    circu_time = time.time()
                    # print('circur time: ', circu_time - start_time)
                    rate.sleep()
                    end_time = time.time()
                    # 输出频率
                    # print('frequence: ', 1.0 / (end_time - start_time))
                    # 输出时间
                    # seconds = rospy.get_rostime()
                    # print(seconds)
                    # print(x_position(hex_receive_message))
                    # print('string ', str(hex_receive_message)[2:4])
            except KeyboardInterrupt:
                if ser != None:
                    print('Port closed!')
                    ser.close()
                             
    else:
        print("port open failed")

