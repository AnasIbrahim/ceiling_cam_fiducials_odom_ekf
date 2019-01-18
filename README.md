# ceiling_cam_fiducials_odom_ekf
localize robots from a ceiling camera with fiducial and fuse odom with ekf

this package uses a ceiling camera detecting fiducials on the robots to detect their pose (PoseWithCovarianceStamped) then 
fuse this pose with the odom msg (nav_msgs/Odometry) coming from robots odom using EKF from robot_localization pkg. If you 
are using multiple machines, don't forget to synchronize their time (with ntp/chrony), otherwise the filter will fail.
