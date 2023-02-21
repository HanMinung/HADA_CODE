# LASERSCANNER	: SICK LIDAR (2D)

* TCP/IP communication
* Resolution : 0.25 [deg]
* Real time structure with 50 [Hz] sampling frequency
* Purpose of usage :
  * Static obstacle avoidance
    * VFF algorithm
    * Obstacle force field, Steering force field, Integrated force field
  * Dynamic obstacle avoidance
    * Filter
  * Track guidance
    * Clustering : DB scan method
    * Deep learning model for detecting rubber cone



# VELODYNE VLP-16 : 3D LIDAR

* Horizontal resolution : 0.2 [deg]

  Vertical resoulution : 16 channel ( -15 [deg] ~ 15[deg] )

* Purpose of usage : 

  * Dynamic obstacle avoidance
  * Calibration with camera
    * Get exact distance that camera sensor detected with YOLO V5 model

