#! /usr/bin/env python3
# coding=utf-8


# 直线的Bresenham格点生成(器)函数
# 详见 https://blog.csdn.net/qq_43306298/article/details/109351600


def bresenham_line(begin, end):
    x1, y1 = begin
    x2, y2 = end
    result = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    # 根据直线的走势方向，设置变化的单位是正是负
    s1 = 1 if ((x2 - x1) > 0) else -1
    s2 = 1 if ((y2 - y1) > 0) else -1
    # 根据斜率的大小，交换dx和dy，可以理解为变化x轴和y轴使得斜率的绝对值为[0,1]
    bool_exchange = False
    if dy > dx:
        dy, dx = dx, dy
        bool_exchange = True
    # 初始误差
    e = 2 * dy - dx
    x = x1
    y = y1
    for i in range(0, int(dx)):
        if e >= 0:
            # 此时要选择横纵坐标都不同的点，根据斜率的不同，让变化小的一边变化一个单位
            if bool_exchange:
                x += s1
            else:
                y += s2
            e -= 2 * dx
        # 根据斜率的不同，让变化大的方向改变一单位，保证两边的变化小于等于1单位，让直线更加均匀
        if bool_exchange:
            y += s2
        else:
            x += s1
        e += 2 * dy
        # yield (x, y)
        result.append((x, y))
    return result


if __name__ == '__main__':
    b = bresenham_line((1, 1), (10, 15))
    xl = []
    yl = []
    for p in b:
        print(p)
        xl.append(p[0])
        yl.append(p[1])
    import matplotlib.pyplot as plt
    plt.scatter(xl, yl)
    plt.show()

