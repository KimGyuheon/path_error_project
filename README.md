# path_error_project : Guidance Path Tracking

 
 <b>Topic</b> : Study of Tracking Guidance path & waypoint following (including Line Of Sight Error & Cross track error)

<b>Related Course</b> : 이동로봇공학 (Mobile Robotics)

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
<br>
<hr>

### Ineffient result

![los_pd(1,0 5)cte_pd(1,0 5)_비효율](https://github.com/KimGyuheon/path_error_project/assets/97663910/dce0daa8-3b45-462e-8c34-a39522f78dd8)
<p><b>[los p gain : 1 , los d gain : 0.5  & cte p gain 1 , cte d gain 0.5 & acceptence radius : 1.0]</b>
<br>
Shows huge curve path to reach next waypoint -> inefficient path (will cause long time to reach waypoints)</p>

![los_pd(5, 0)cte_pd(5,0)](https://github.com/KimGyuheon/path_error_project/assets/97663910/ceffe5cb-c37e-4a4c-904d-db9fbc28a623)
<p><b>[los p gain : 5 , los d gain : 0  & cte p gain 5 , cte d gain 0 & acceptence radius : 1.0]</b>
<br>
Shows curve after passesd by waypoint -> inefficient path (will cause long time to reach waypoints)</p>
</p>

<hr>

### Effient result

![los_pd(5, 0 5)cte_pd(0 5,1),rad(1 0)](https://github.com/KimGyuheon/path_error_project/assets/97663910/f470ca1a-3681-406e-b338-ae1aa4b5e448)
<p><b>[los p gain : 5 , los d gain : 0.5  & cte p gain 0.5 , cte d gain 1 & acceptence radius : 1.0]</b>
<br>
Shows Recovery to reach next waypoint earlier -> efficient path</p>
</p>

<hr>

#### Provment about Efficient result
Reduced Acceptence radius of Efficient result's parameter to 0.5 

![los_pd(5, 0 5)cte_pd(0 5,1),rad(0 5)](https://github.com/KimGyuheon/path_error_project/assets/97663910/1286883c-4281-4e30-8b43-d9aa5885a14d)
<p><b>[los p gain : 5 , los d gain : 0.5  & cte p gain 0.5 , cte d gain 1 & acceptence radius : 0.5]</b>
<br>
Perfectly reach next waypoint even if accept radius reduce to 0.5 -> efficient path</p>
</p>
