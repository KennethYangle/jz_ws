import time
import VehicleApi



Swarm=50
row=0
col=0
i=1
mav={}


for i in range(Swarm):
    mav[i+1] = VehicleApi.Vehicle(i+1, 100, 'OldFactory')
    #time.sleep(1)
    #mav[i+1].SendMaxSpeed(200)

    if row<5:
        mav[i+1].initSimpleModel([-225, -120+row, 0])
    if row>=5 and row<10:
        col=col+1
        mav[i+1].initSimpleModel([-225+2, -120+row-5, 0])
    if row>=10 and row<15:
        col=col+1
        mav[i+1].initSimpleModel([-225+4, -120+row-10, 0])
    if row>=15 and row<20:
        col=col+1
        mav[i+1].initSimpleModel([-225+6, -120+row-15, 0])
    if row>=20 and row<25:
        col=col+1
        mav[i+1].initSimpleModel([-225+8, -120+row-20, 0])
    if row>=25 and row<30:
        col=col+1
        mav[i+1].initSimpleModel([-225+10, -120+row-25, 0])
    if row>=30 and row<35:
        col=col+1
        mav[i+1].initSimpleModel([-225+12, -120+row-30, 0])
    if row>=35 and row<40:
        col=col+1
        mav[i+1].initSimpleModel([-225+14, -120+row-35, 0])
    if row>=40 and row<45:
        col=col+1
        mav[i+1].initSimpleModel([-225+16, -120+row-40, 0])
    if row>=45 and row<50:
        col=col+1
        mav[i+1].initSimpleModel([-225+18, -120+row-45, 0])
    #time.sleep(1)

    row=row+1

    print(i)

