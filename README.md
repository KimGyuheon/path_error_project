# path_error_project

 
 <b>Topic</b> : Study of path planning (including Line Of Sight Error & Cross track error)

<b>Related Course</b> : 이동로봇공학

<b>Date</b> : 2022.11


<br>
<b>Project Explanation</b>  
- 4 waypoints are made by using Matlab  
- Vehicle Kinematics is based on Unicycle Model  
- Used PD control  
- Compare difference of path between including (LOS error & Cross Track error) and not  
- Compare difference of path about making radius acceptence of waypoint lager and smaller  


<br>
<b>Code</b>  
- u(1) : P Control  
- u(2) : PD Control  
- kp_psi_los, kd_psi_los : los error's PD gain  
- kp_psi_cte, kd_psi_cte : cte error's PD gain  
