# By Alberto Caro
# Ingeniero Civil Informatico
# Universidad Catolica de Temuco
# Dr.(c) Ciencias de la Ingenieria - PUC
# Dr. Billy Peralta - UNAB
#-----------------------------------------------------------------------------
from __future__ import division   
import math, time as ti, serial as RS, random as ra,struct as ST
from visual import *

#-----------------------------------------------------------------------------
# Fuerza G Maxima del Sensor
#-----------------------------------------------------------------------------
nMAX_G = 16 ; nOld_R = nOld_P = nOld_Y = 0; nROLL = 0 ; nPITCH = 1 ; nYAW = 2

#-----------------------------------------------------------------------------
# Definicion del Mundo 3D y sus Objetos.-
#-----------------------------------------------------------------------------
WinM = display(title='Kinect',x=50,y=0,width=1000,height=1000,center=(0,0,0))
Base = box(pos=(0,0,0),size=(500,3,500),color=color.gray(0.5))
BoxL = box(pos=(-130,25,100),size=(30,50,30),color=color.gray(0.5))
BoxR = box(pos=(+130,25,100),size=(30,50,30),color=color.gray(0.5))
#BoxR = box(pos=(0,0,0),size=(500,3,500),color=color.gray(0.5))

#-----------------------------------------------------------------------------
# Declaracion Frame Main
# Retorna el Frame Base donde se acoplan todos los obetos del brazo
#-----------------------------------------------------------------------------
def Get_Frame(tFXYZ,nClr):
    #Frame Main
    oFr_B = frame(pos=tFXYZ)
    oFr_B.oC1 = cylinder(frame=oFr_B,pos=(0,20,0),radius=30,axis=(0,10,0),color=color.yellow)
    oFr_B.oB1 = box(frame=oFr_B,pos=(0,10,0),size=(80,20,80),color=nClr,)
    # Brazos Kinect Vertical y Horizontal
    oFr_B.oB2 = box(frame=oFr_B,pos=(0,100,0),size=(20,150,20),color=color.blue)
    oFr_B.oB3 = box(frame=oFr_B,pos=(0,130,+22),size=(140,20,20),color=color.green)
    return oFr_B


def Rota(nEje,nAng,xObj):
    global nOld_R,nOld_P,nOld_Y  
    if (nEje == nROLL):
        nDif = nAng - nOld_R
        xObj.oB3.rotate(angle = -1*radians(nDif),axis=(0,0,1),origin =(0,130,-22))
        nOld_R = nAng # Guardamos el nuevo Roll
        print(nAng)

        if (nAng >=30) and (nAng <=45):
           BoxR.color = color.red
        else:
           BoxR.color = color.gray(0.5)

        if (nAng >=315) and (nAng <=330):
           BoxL.color = color.red
        else:
           BoxL.color = color.gray(0.5)

    if (nEje == nYAW):
        nDif = nAng - nOld_Y
        xObj.rotate(angle = 1*radians(nDif),axis=(0,1,0),origin =(0,0,0))
        nOld_Y = nAng # Guardamos el nuevo YAW
    if (nEje == nPITCH):
        nDif = nAng - nOld_Y
        xObj.oB3.rotate(angle = 1*radians(nDif),axis=(1,0,0),origin =(40,160,+22))
        nOld_Y = nAng # Guardamos el nuevo YAW
    return

#-----------------------------------------------------------------------------
# Codigo Main
# Open Serial Port, 115.200 Bps + 8N1
#-----------------------------------------------------------------------------

s = RS.Serial('COM10')
s.baudrate = 115200

aObj = [Get_Frame((0,0,0),color.blue)]

while 1:
 sT = s.read(2)
 aD = ST.unpack('B'*len(sT),sT)
 if (aD[0] == 0x55):
     if (aD[1] == 0x53): # Inclinacion = (Roll, Pitch, Yaw)
         sT   = s.read(9) # Paquete de datos..
         aD   = ST.unpack('B'*len(sT),sT)
         Rx   = ((aD[1] << 8 | aD[0])/32768.0) * 180   # Roll
         Py   = ((aD[3] << 8 | aD[2])/32768.0) * 180   # Pitch
         Yz   = ((aD[5] << 8 | aD[4])/32768.0) * 180   # Yaw
         Te   = ((aD[7] << 8 | aD[6])/340.0  ) + 36.53 # Temperatura
         nRol = Rx # Roll
         nPit = 360-Py # Pitch
         nYaw = Yz # Yaw
         Rota(nROLL,nRol,aObj[0])
         #Rota(nYAW ,nYaw,aObj[0])   
         #Rota(nPITCH,nPit,aObj[0])   
         rate(10)

s.close() 

#print(sLine_3 %(Rx,Py,Yz))