# prototype manipulator matlab simulation

> Authors:	Seonghyeon Jo(cpsc.seonghyeon@gmail.com)
> 
> Date:		 Des, 24, 2021
> 

This repository is a MATLAB simulation of prototype manipulator using RK4(Runge-Kutta 4). The robot manipulator dynamic model (M, C, G) is obtained by Recursive Newton-Euler method. 

### Controller
- fig : controller result figure folder
- model : prototype robot model library folder
  + get_CoriolisMatrix.m : Coriolis matrix function <img src="https://render.githubusercontent.com/render/math?math=\C(\q,\dot{\q})&mode=inline"> 
  + get_GravityVector.m : Gravity vector function <img src="https://render.githubusercontent.com/render/math?math=\G(\q)&mode=inline"> 
  + get_MassMatrix.m : Inertia matrix function <img src="https://render.githubusercontent.com/render/math?math=\M(\q)&mode=inline"> 
  + plant.m :  robot manipulator plant function <img src=
  "https://render.githubusercontent.com/render/math?math=%5Ctextstyle+%5Cddot%7B%5Cq%7D%3D%5CM%5E%7B-1%7D%28%5Cq%29%28-%5CC%28%5Cq%2C%5Cdot%7B%5Cq%7D%29%5Cdot%7B%5Cq%7D-%5CG%28%5Cq%29%29%2B%5Cboldsymbol%5Ctau" 
  alt="\ddot{\q}=\M^{-1}(\q)(-\C(\q,\dot{\q})\dot{\q}-\G(\q))+\boldsymbol\tau">  
  + rk.m :  Runge-Kutta 함수
- 1. main_cartesian_joint_pd_controller.m : cartesian joint pd controller code
- 1. main_cartesian_admittance_controller.m : cartesian admittance controller code
- 1. main_driect_cartesian_pd_controller.m : cartesian pd controller code
- 1. main_joint_admittance_controller.m : joint admittance controller Code
- 1. main_joint_pd_controller.m : simple joint pd controller code
- 
#### joint pd controller
<img src="./fig/sim_pd_con_result1.png" alt="sim_pd_con_result1"  width="375" />
<img src="./fig/sim_pd_con_result2.png" alt="sim_pd_con_result2"  width="375" /> 

#### cartesian pd controller
<img src="./fig/car_pd_con_result1.png" alt="car_pd_con_result1"  width="375" />
<img src="./fig/car_pd_con_result2.png" alt="car_pd_con_result2"  width="375" /> 

#### cartesian joint pd controller
<img src="./fig/joint_car_jt_con_result1.png" alt="car_pd_con_result1"  width="375" />
<img src="./fig/joint_car_jt_con_result2.png" alt="car_pd_con_result2"  width="375" /> 

####  joint admittance controller
<img src="./fig/joint_ad_con_result1.png" alt="joint_ad_con_result1"  width="375" />
<img src="./fig/joint_ad_con_result2.png" alt="joint_ad_con_result2"  width="375" /> 
<img src="./fig/joint_ad_con_result3.png" alt="joint_ad_con_result3"  width="375" /> 

#### cartesian admittance controller
<img src="./fig/car_ad_con_result1.png" alt="car_ad_con_result1"  width="375" />
<img src="./fig/car_ad_con_result2.png" alt="car_ad_con_result2"  width="375" /> 
<img src="./fig/car_ad_con_result3.png" alt="car_ad_con_result3"  width="375" /> 

