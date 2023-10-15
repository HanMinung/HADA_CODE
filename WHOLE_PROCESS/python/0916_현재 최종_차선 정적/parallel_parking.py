####################################
# Main Code For Mission Driving    #
# 2023-08-23                       #
# Author : NKH                     #
####################################

from scripts.core import *       

if __name__ == "__main__":

    Wa = WayMaker("waypoint/HADA3BONSEONV5.csv")
    data_IO = DataCommunication("real")
    cmd_gear = 0
    cmd_steer = 0
    cmd_velocity = 10
    cmd_brake = 0
    lat,lon,x,y,yaw   = data_IO.get_att_data(37.2,126.7)
    target_ind ,_ = Wa.Trajectory.search_target_index(x,y)

    para = PARALLEL()
    
    while  isEnd() == False :

        lat,lon,x,y,yaw = data_IO.get_att_data(37.2,126.7)
        azim, dist      = data_IO.get_lidar_data()

        # # calculate guidance command
        target_ind ,_ = Wa.Trajectory.search_target_index(x,y)
        tx = Wa.cx[target_ind]
        ty = Wa.cy[target_ind]
        cmd_steer  = impact_Angle_steer_control(x,y,yaw,tx,ty)
        
        gearcmd, velcmd, steercmd, brakecmd = para.parallel_parking(x, y, lat, lon, yaw, azim, dist ,[cmd_gear,cmd_velocity,cmd_steer,cmd_brake])

        data_IO.command_to_vehicle(gearcmd,  steercmd, velcmd, brakecmd)
        #data_IO.command_to_vehicle(para.gearcmd,  para.steercmd, para.velcmd, para.brakecmd)
    
    
    
    
