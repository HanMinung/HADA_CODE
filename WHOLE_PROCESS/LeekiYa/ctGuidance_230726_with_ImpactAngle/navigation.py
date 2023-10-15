from os import access
from turtle import pos
from numpy import *
from math  import *

from GPS import R2D

# Earth
EARTH_Req =				(double)   ( 6378135.00 )
EARTH_Rn  =				(double)   ( 6357467.39 )
EARTH_Re  =				(double)   ( 6390915.75 )
EARTH_g0  =				(double)   ( 9.780318 )
EARTH_e2  =				(double)   ( 0.00669437999014 )
EARTH_We  =				(double)   ( 0.00007292115 )

# Pure Navigation
def getNAVRn(L):
    return ( EARTH_Req * ( 1 - EARTH_e2 ) / pow(1-EARTH_e2*pow(sin(L),2),1.5))

def getNAVRe(L):
    return ( EARTH_Req / pow( 1-EARTH_e2*pow(sin(L),2),0.5) )

def getNAVgl(L,h,R0):
    return ( EARTH_g0 * ( 1 + 5.3024e-3*pow(sin(L),2) - 5.9e-6*pow(sin(2*L),2) ) / pow( 1 + h/R0,2 ) )

def getDCM(phi, theta, psi):

    result       = zeros((3,3))
    result[0][0] = cos(theta) * cos(psi)
    result[0][1] = cos(theta) * sin(psi)
    result[0][2] = -sin(theta)
    result[1][0] = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi)
    result[1][1] = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi)
    result[1][2] = sin(phi) * cos(theta)
    result[2][0] = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)
    result[2][1] = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)
    result[2][2] = cos(phi) * cos(theta)
    
    return result

D2R        =            pi/180
R2D        =            180/pi

# Barometer
BARO_R     =			(double)   ( 8.31446261815324 )  # [J/K/mol]
BARO_M     =			(double)   ( 0.0289644 )         # [kg/mol]
BARO_GAMMA =    		(double)   ( 0.0064 )            # [K/m]

# LATRAD2M	=			(double)   ( 6357590.358839842 ) # HGU
# LONRAD2M	=			(double)   ( 5159243.427952315 ) # HGU
LATRAD2M	=	(double)   ( 6358805.404762348 ) # K-city
LONRAD2M	=	(double)   ( 5083989.602644125 ) # K-city

def cross_product(vector1, vector2):
    if len(vector1) != 3 or len(vector2) != 3:
        raise ValueError("Vectors must have three elements")
    
    result = [
        vector1[1] * vector2[2] - vector1[2] * vector2[1],
        vector1[2] * vector2[0] - vector1[0] * vector2[2],
        vector1[0] * vector2[1] - vector1[1] * vector2[0]
    ]
    
    return result

def getMatTranspose(X):
    Nrow, Ncol = X.shape
    result = zeros((Ncol, Nrow), dtype=X.dtype)
    
    for i in range(Ncol):
        for j in range(Nrow):
            result[i][j] = X[j][i]
    
    return result
    
class PNAV:
        geoPos = zeros((3,1))
        VeN    = zeros((3,1))

class NAV:
        Rn,Re,R0                = 0.0,0.0,0.0
        lat2m,lon2m,m2lat,m2lon = 0.0,0.0,0.0,0.0
        geoPos                  = zeros((3,1))
        geoPosDot               = zeros((3,1))
        VeN                     = zeros((3,1))
        AeN                     = zeros((3,1))
        CeN                     = zeros((3,3))
        CbN                     = zeros((3,3))
        PureEuler               = zeros((3,1))
        WieE                    = zeros((3,1))
        WenN                    = zeros((3,1))
        WibB                    = zeros((3,1))
        glN                     = zeros((3,1))
        isFixyaw                = 0
        Filtercount             = 0

class BARO:
        H0,T0,P0                = 0.0,0.0,0.0


class IMUdata:
        vel_NED                 = zeros((3,1))
        acc_Body                = zeros((3,1))
        att                     = zeros((3,1))

class GPSdata:
        Pos                     = zeros((3,1))             

def ImportSensorforNAV(L,l,h,accX,accY,accZ,roll,pitch,yaw):
        GPSdata.Pos[0][0]       = L     *D2R
        GPSdata.Pos[1][0]       = l     *D2R
        GPSdata.Pos[2][0]       = h     *D2R
        IMUdata.att[0][0]       = roll  *D2R
        IMUdata.att[1][0]       = pitch *D2R
        IMUdata.att[2][0]       = yaw   *D2R
 
        IMUdata.acc_Body[0][0]   = accX
        IMUdata.acc_Body[1][0]   = accY
        IMUdata.acc_Body[2][0]   = accZ

        # IMUdata.acc_NED         = transpose(getDCM(IMUdata.att[0][0], IMUdata.att[1][0], IMUdata.att[2][0]))@IMUdata.acc_Body
 
       

class Filter:

    def __init__(self,Ts):
        
        # print(NAV.Re)
        #sampling time
        self.Ts= Ts    

        # Vertical Channel Damping Filter
        VFILT_TAU      = 1.0
        VFILT_WS       = sqrt(EARTH_g0/EARTH_Req)

        self.VFILT_C1  = 3.0/VFILT_TAU
        self.VFILT_C2  = 2.0*(VFILT_WS**2)+3.0/(VFILT_TAU**2)
        self.VFILT_C3  = 1.0/(VFILT_TAU**3)

        self.VFILT_acc =0.0
        self.VFILT_vel =0.0

        # Horizontal Channel Filter (EKF)
        self.HFILT_cnt= 0
      
        self.F        = zeros((6,6))  # sysyem matrix after  Discretization
        self.Fc       = zeros((6,6))
        self.H        = zeros((2,6))  # measurement matrix (위도,경도,N방향가속도,E방향가속도) 
        self.Q        = zeros((6,6))  # 시스템모델 잡음
        self.R        =       eye(2)  # 측정잡음
        self.Xbar     = zeros((6,1))  # 상태변수 = [위도;경도;N방향속도;E방향속도;Rolldot;Pitchdot]
        self.Xhat     = zeros((6,1))  # 상태변수 = [위도;경도;N방향속도;E방향속도;Rolldot;Pitchdot]
        self.eP       = zeros((6,6))
        self.P        = zeros((6,6))
        self.Kf       = zeros((6,2))
        self.residual = zeros((2,1))
        self.Xpure    = zeros((2,1))  # [위도;경도;N방향속도;E방향속도] from 순수항법
        self.Xsensor  = zeros((2,1))  # [위도;경도;N방향속도;E방향속도] from GPS, IMU
        self.Y        = zeros((2,1))

        # Q 행렬 초기화

        self.Q[0][0]=(1/36)*(Ts**6) 
        self.Q[0][2]=(1/12)*(Ts**5)
        self.Q[0][4]=(1/6) *(Ts**4)

        self.Q[1][1]=(1/36)*(Ts**6)
        self.Q[1][3]=(1/12)*(Ts**5)
        self.Q[1][5]=(1/6) *(Ts**4)

        self.Q[2][0]=(1/12)*(Ts**5) 
        self.Q[2][2]=(1/4) *(Ts**4)
        self.Q[2][4]=(1/2) *(Ts**3)

        self.Q[3][1]=(1/12)*(Ts**5) 
        self.Q[3][3]=(1/4) *(Ts**4)
        self.Q[3][5]=(1/2) *(Ts**3)

        self.Q[4][0]=(1/6) *(Ts**4) 
        self.Q[4][2]=(1/2) *(Ts**3)
        self.Q[4][4]=       (Ts**2)

        self.Q[5][1]=(1/6) *(Ts**4) 
        self.Q[5][3]=(1/2) *(Ts**3)
        self.Q[5][5]=       (Ts**2)

        # Q행렬,R행렬 scaling
        HFILT_q = 0.01
        HFILT_r = (2.0*(0.1408/6400000))**2 

        self.Q  = HFILT_q*self.Q
        self.R  = HFILT_r*self.R

    def vFilter(self):
        
        GPS_h = 50.0 #이거로 고정해서 씀
        yI0   = 0.0

        delH  = PNAV.geoPos[2][0] - GPS_h
        altC2 = self.VFILT_C2 *delH
        altC3 = self.VFILT_C2 *delH

        self.VFILT_acc =altC2 + yI0
        self.VFILT_vel =0.0

        yI0   = yI0 + altC3*self.Ts


    def hFilter(self):
                
        # H 행렬(위도,경도,N방향,E방향 속도)
        self.H[0][0]=1.0
        self.H[1][1]=1.0
        # self.H[2][2]=1.0
        # self.H[3][3]=1.0

        ts= self.Ts
        WE=EARTH_We
        Rn=NAV.Rn
        Re = NAV.Re
        L = PNAV.geoPos[0][0]
        psi = IMUdata.att[2][0]
        vn = PNAV.VeN[0][0]
        ve = PNAV.VeN[1][0]
        vd = PNAV.VeN[2][0]

                
        self.Fc[0][2] = 1 / Rn

        self.Fc[1][0] = ve * tan(L) / ( Re * cos(L) )
        self.Fc[1][3] = 1 / ( Re * cos(L) )

        self.Fc[2][0] = ( -2.0 * WE * ve * cos(L) - (ve**2) * (1.0/(cos(L)**2)*Re))
        self.Fc[2][3] = ( -2 * WE * sin(L) - 2 * ve * tan(L) / Re )
        self.Fc[2][4] = cos(psi)
        self.Fc[2][5] = -sin(psi)

        self.Fc[3][0] = ( 2.0 * WE * vn * cos(L) + vn * ve *  (1.0/(cos(L)**2)*Re))
        self.Fc[3][2] = ( 2.0 * WE * sin(L) + ve * tan(L) /Re )
        self.Fc[3][3] = ( vn * tan(L) + vd ) / Re
        self.Fc[3][4] = sin(psi)
        self.Fc[3][5] = cos(psi)
        
        self.F = eye(6) + self.Ts*self.Fc

       
        # system propagation
        self.Xbar=self.F@self.Xhat
        self.eP =self.F@self.eP@transpose(self.F)+self.Q
        self.Kf =self.eP@transpose(self.H)@linalg.inv(self.H@self.eP@transpose(self.H) + self.R)
        
        #GPS값이 이 사이에 있을때만 센서값을 받아들이겠다 (라디안 단위임)
        if(GPSdata.Pos[0][0] > 0.6 and GPSdata.Pos[0][0] < 0.7):
            
            NAV.Filtercount=NAV.Filtercount+1
             #순수항법으로만 구한 위도,경도,N방향속도,E방향속도
            self.Xpure[0][0]=NAV.geoPos[0][0]
            self.Xpure[1][0]=NAV.geoPos[1][0]
           
            #GPS,IMU로 들어온 위도,경도,N방향속도,E방향속도
            self.Xsensor[0][0]=GPSdata.Pos[0][0]
            self.Xsensor[1][0]=GPSdata.Pos[1][0]
                       
            # measurement update
            self.Y    = self.Xsensor - self.Xpure
            self.Xhat = self.Xbar + self.Kf@(self.Y-self.H@self.Xbar)
            self.P    = self.eP - self.Kf@self.H@self.eP
            self.eP   = self.P
        
        # 아닐경우 그냥 그대로 유지
        else:
            self.Xhat = self.Xbar
        
        
    def filterUpdate(self):

        NAV.AeN[2][0]  		=   - NAV.AeN[2][0]  + self.VFILT_acc 
        PNAV.VeN[2][0]  	=   - PNAV.VeN[2][0] + self.VFILT_vel 

        NAV.geoPos[0][0] 	= PNAV.geoPos[0][0] + self.Xhat[0][0]
        NAV.geoPos[1][0] 	= PNAV.geoPos[1][0] + self.Xhat[1][0]
        
        NAV.VeN[0][0] 		= PNAV.VeN[0][0] 	+ self.Xhat[2][0]
        NAV.VeN[1][0] 		= PNAV.VeN[1][0] 	+ self.Xhat[3][0]
     
        
        if(NAV.Filtercount  == 20):
            PNAV.geoPos[0][0]    = NAV.geoPos[0][0]
            PNAV.geoPos[1][0]    = NAV.geoPos[1][0]
            PNAV.VeN[0][0]       = NAV.VeN[0][0]
            PNAV.VeN[1][0]       = NAV.VeN[1][0]

            self.Xhat            = zeros((6,1))
            NAV.Filtercount = 0
        


class Navigation:

    def __init__(self,Ts):
        self.Ts = Ts

        BARO.H0 =   50.0
        BARO.T0 =   28.0
        BARO.P0 = 1000.0
        NAV.WieE[2][0]         	= EARTH_We
        GPS_flag=1
                
        NAV.Rn  = getNAVRn(GPSdata.Pos[0][0]) if GPS_flag == 1 else EARTH_Rn
        NAV.Re  = getNAVRe(GPSdata.Pos[0][0]) if GPS_flag == 1 else EARTH_Re
        NAV.R0  = sqrt(NAV.Rn*NAV.Re)

        NAV.lat2m =  NAV.Rn if GPS_flag == 1 else LATRAD2M
        NAV.lon2m =  NAV.Re*cos(GPSdata.Pos[0][0]) if GPS_flag == 1 else LONRAD2M
        NAV.m2lat =  1/NAV.lat2m
        NAV.m2lon =  1/NAV.lon2m

        PNAV.geoPos[0][0]=GPSdata.Pos[0][0]  # 위도
        PNAV.geoPos[1][0]=GPSdata.Pos[1][0]  # 경도
        PNAV.geoPos[2][0]=GPSdata.Pos[2][0]  # 고도 

        NAV.geoPos[0][0]=PNAV.geoPos[0][0]
        NAV.geoPos[1][0]=PNAV.geoPos[1][0]
        NAV.geoPos[2][0]=PNAV.geoPos[2][0]

        PNAV.VeN[0][0]= 0.0
        PNAV.VeN[1][0]= 0.0
        PNAV.VeN[2][0]= 0.0                 # D방향 속도-> horizontal filter에서는 안쓰나 translational dynamics에서 사용됨

        NAV.VeN[0][0] = PNAV.VeN[0][0]
        NAV.VeN[1][0] = PNAV.VeN[1][0]
        NAV.VeN[2][0] = PNAV.VeN[2][0]

    def getLocationDynamics(self):

        NAV.Rn = getNAVRn(NAV.geoPos[0][0])
        NAV.Re = getNAVRe(NAV.geoPos[0][0])
        NAV.R0 = sqrt(NAV.Rn * NAV.Re)
        
        # geoPosDot = [Ldot; ldot; hdot]
        NAV.geoPosDot[0][0] = PNAV.VeN[0][0] / (NAV.Rn + PNAV.geoPos[2][0])                                 # Ldot = V_N/(R_N+h)
        NAV.geoPosDot[1][0] = PNAV.VeN[1][0] / ((NAV.Re + PNAV.geoPos[2][0]) * cos(PNAV.geoPos[0][0]))      # ldot = V_E/(R_E+h)
        NAV.geoPosDot[2][0] = -PNAV.VeN[2][0]                                                               # hdot = -V_D

    #Imu roll,pitch,yaw 값 인자로 받기
    def translationalDynamics(self):

        NAV.glN[2][0] = getNAVgl(PNAV.geoPos[0][0],PNAV.geoPos[2][0], NAV.R0)
        NAV.CeN       = getDCM(0, (-PNAV.geoPos[0][0] - pi / 2),PNAV.geoPos[1][0])                 # E-frame to N-frame DCM (시변특성 가짐)-> 위도, 경도,고도가 계속 바뀌니까        
        NAV.CbN       = transpose(getDCM(IMUdata.att[0][0], IMUdata.att[1][0], IMUdata.att[2][0])) # B-frame to N-frame DCM (시변특성 가짐)-> roll,pitch,yaw가 계속 바뀌니까
        
        # WieE = I-frame에 관한 E-frame의 회전각속도, WieE = [0;0;EARTH.We];
        NAV.WieE[2][0] = EARTH_We
        
        # WenN = E-frame에 관한 N-frame의 회전각속도, WenN = [ldot * cos(Lat); -Ldot;-ldot*sin(Lat)]; 
        NAV.WenN[0][0] = NAV.geoPosDot[1][0] * cos(PNAV.geoPos[0][0])                                                                                                
        NAV.WenN[1][0] = -NAV.geoPosDot[0][0]
        NAV.WenN[2][0] = -NAV.geoPosDot[1][0] * sin(PNAV.geoPos[0][0])
        
        # WibB = I-frame에 관한 B-frame의 회전각속도, WibB => 매트랩 파일이랑 인수인계 자료 참고
        temp1,temp2,temp3 =zeros((3,1)),zeros((3,1)),zeros((3,1))
        temp1[0][0]   = IMUdata.acc_Body[0]
        temp2[1][0]   = IMUdata.acc_Body[1]
        temp3[2][0]   = IMUdata.acc_Body[2]
        NAV.WibB      = temp1 + getDCM(IMUdata.acc_Body[0],0,0)@temp2 + getDCM(IMUdata.acc_Body[0],IMUdata.acc_Body[1],0)@temp3 
   
        # NAV.PureEuler = [Rolldot;Pitchdot;Yawdot];
        NAV.PureEuler[0][0] = NAV.WibB[1][0]*sin(IMUdata.att[0][0])+NAV.WibB[2][0]*cos(IMUdata.att[0][0])*tan(IMUdata.att[1][0])+NAV.WibB[0][0] # Rolldot
        NAV.PureEuler[1][0] = NAV.WibB[1][0]*cos(IMUdata.att[0][0])-NAV.WibB[2][0]*sin(IMUdata.att[0][0])                                       # Pitchdot  
        NAV.PureEuler[2][0] = NAV.WibB[1][0]*sin(IMUdata.att[0][0])+NAV.WibB[2][0]*cos(IMUdata.att[0][0])*1/cos(IMUdata.att[2][0])              # Yawdot

        # 순수항법으로 산출된 동체의 N방향,E방향,D방향 가속도
        NAV.AeN       = (NAV.CbN@IMUdata.acc_Body)-cross_product( (2*NAV.CeN@NAV.WieE+NAV.WenN), NAV.VeN)+NAV.glN

    def integration(self):
        
        PNAV.VeN    = NAV.VeN    + self.Ts *NAV.AeN
        PNAV.geoPos = NAV.geoPos + self.Ts *NAV.geoPosDot  

class Estimation:    

    def __init__(self,L,l,h,accX,accY,accZ,roll,pitch,yaw):

        ImportSensorforNAV(L,l,h,accX,accY,accZ,roll,pitch,yaw)
        self.filter= Filter(0.01)
        self.navigation = Navigation(0.01)
      

    def main(self,L,l,h,accX,accY,accZ,roll,pitch,yaw):

        ImportSensorforNAV(L,l,h,accX,accY,accZ,roll,pitch,yaw)
        self.navigation.getLocationDynamics()
        self.navigation.translationalDynamics()
        self.filter.vFilter()
        self.filter.hFilter()
        self.filter.filterUpdate()
        self.navigation.integration()

class VelFilter:

    def __init__(self,lat,lon,v,yaw):
        
        ts=0.1
        beta=0.56      
        Lat=lat*pi/180*LATRAD2M
        Lon=lon*pi/180*LONRAD2M
        Vn=v*cos(yaw)
        Ve=v*sin(yaw)

        self.F = array([[1.0,  ts,   0,   0],
                        [  0, 1.0,   0,   0],
                        [  0,   0, 1.0,  ts],
                        [  0,   0,   0, 1.0]])

        self.K = array([[(1.0-beta**2)             ,0],
                        [(1.0-beta)**2/ts          ,0],
                        [0,             (1.0-beta**2)],
                        [0,          (1.0-beta)**2/ts]])

        # self.K = array([[(1.0-beta**2)    ,  ((1.0-beta)**2)*ts   ,     0               ,                  0],
        #                 [0 ,  (1.0-beta**2)        ,     0               ,                  0],
        #                 [0                ,    0                  , (1.0-beta**2)       , ((1.0-beta)**2)*ts ],
        #                 [0                ,    0                  , 0   ,      (1.0-beta**2)]])

        self.H = array([[1.0, 0,   0, 0],
                        [  0, 0, 1.0, 0]])

        # self.H = array([[1.0,  0,   0,    0],
        #                 [  0, 1.0,   0,   0],
        #                 [  0,   0, 1.0,   0],
        #                 [  0,   0,   0, 1.0]])

        self.Xbar = array([[Lat],
                            [0],
                            [Lon],
                            [0]])

        # self.Xbar = array([[Lat],
        #                     [Vn],
        #                     [Lon],
        #                     [Ve]])

        self.Xhat = array([[0],
                           [0],
                           [0],
                           [0]])

        self.ePos = array([[0],
                           [0],
                           [0],
                           [0]])

        self.y = array([[Lat],
                        [Lon]])

        # self.y = array([[Lat],
        #                 [Vn],
        #                 [Lon],
        #                 [Ve]])


    def main(self,lat,lon,V_encoder,yaw):  

        Lat=lat*pi/180*LATRAD2M
        Lon=lon*pi/180*LONRAD2M     
        Vn=V_encoder*cos(yaw)
        Ve=V_encoder*sin(yaw)

        # print(f"Lat: {Lat}, Lon: {Lon} est_vel={((Lat)-(Lon))/0.1*3.6}")

        self.y[0][0]=Lat
        self.y[1][0]=Lon
        # self.y[2][0]=Lon
        # self.y[3][0]=Ve

        residual    = self.y - self.H@self.Xbar
        self.Xhat   = self.Xbar + self.K@residual
        self.ePos   = self.F@self.Xhat
        self.Xbar   = self.ePos

        eR=hypot(self.ePos[0][0],self.ePos[1][0])
        Vn=self.ePos[1][0]
        Ve=self.ePos[3][0]
        # V =hypot(Vn,Ve)
        V=hypot(self.ePos[1],self.ePos[3])

        error=V-V_encoder

        return V*3.6,error



