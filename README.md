## Footstep Planner
To achieve high-frequency footstep planning, we can use the Linear Inverted Pendulum Model (LIP), i.e., a point mass, approximating the robot's center of mass (CoM), attached to a massless prismatic actuator. The acceleration of the point mass is given in [this paper](https://ieeexplore.ieee.org/document/6907116):

$$
\ddot{p}_{com}=\frac{g}{h_{com}}(p_{com}-p_{base})
$$

where $g$ is the gravitational constant, $h_{com}$ is the constant height of the CoM and $p_{com}$ and $p_{base}$ are the $x$, $y$ CoM and base (foot) positions, respectively. The equations of motion of the LIP are given by

$$
\begin{bmatrix}
x_{com}\\
x_{com} \\
v_{x, com} \\
v_{y, com} \\
\end{bmatrix}_{k+1} = \begin{bmatrix}\frac{1}{2e^{\frac{\Delta t}{\tau}}} + \frac{e^{\frac{\Delta t}{\tau}}}{2} & 0 & \frac{-\tau}{2e^{\frac{\Delta t}{\tau}}} + \frac{\tau e^{\frac{\Delta t}{\tau}}}{2} & 0 \\
0 & \frac{1}{2e^{\frac{\Delta t}{\tau}}} + \frac{e^{\frac{\Delta t}{\tau}}}{2} & 0 & \frac{-\tau}{2e^{\frac{\Delta t}{\tau}}} + \frac{\tau e^{\frac{\Delta t}{\tau}}}{2}\\
\frac{e^{\frac{\Delta t}{\tau}}}{2\tau} - \frac{1}{2\tau e^{\frac{\Delta t}{\tau}}} & 0 & \frac{e^{\frac{\Delta t}{\tau}}}{2} + \frac{1}{2e^{\frac{\Delta t}{\tau}}} & 0 \\
0 & \frac{e^{\frac{\Delta t}{\tau}}}{2\tau} - \frac{1}{2\tau e^{\frac{\Delta t}{\tau}}} & 0 & \frac{e^{\frac{\Delta t}{\tau}}}{2} + \frac{1}{2e^{\frac{\Delta t}{\tau}}}
\end{bmatrix}\begin{bmatrix}
x_{com}\\
x_{com} \\
v_{x, com} \\
v_{y, com} \\
\end{bmatrix}_{k} + \begin{bmatrix}
\frac{-1}{2e^{\frac{\Delta t}{\tau}}} - \frac{e^{\frac{\Delta t}{\tau}}}{2} & 0 \\
0 & \frac{-1}{2e^{\frac{\Delta t}{\tau}}} - \frac{e^{\frac{\Delta t}{\tau}}}{2} \\
\frac{1}{2\tau e^{\frac{\Delta t}{\tau}}} - \frac{e^{\frac{\Delta t}{\tau}}}{2\tau} & 0 \\
0 & \frac{1}{2\tau e^{\frac{\Delta t}{\tau}}} - \frac{e^{\frac{\Delta t}{\tau}}}{2\tau} 
\end{bmatrix} \begin{bmatrix}
x_{base} \\
y_{base}
\end{bmatrix}_k
$$

where $\tau = \sqrt{\frac{h_{com}}{g}}$ and $\Delta t$ is a chosen discretization time-step. We can represent this discrete linear system as 

$$
q_{k+1} = Aq_k + Bu_k
$$

where the input $u_k$ is the foot position. In order to build the cost, we plan a reference plan as done in [this paper](https://www.researchgate.net/publication/301463868_Robust_and_Agile_3D_Biped_Walking_With_Steering_Capability_Using_a_Footstep_Predictive_Approach). Using this reference path, our final cost function for the MPC looks as followers:

$$
\begin{align*}
\min_{q, u} & \quad \sum_{k=1}^N (p_{com, k+1} + p_{com, k} - 2p_{m, k})^\top Q_q (p_{com, k+1} + p_{com, k} - 2p_{m, k})\\
&+ \sum_{k=1}^N \left((p_{com, k+1} - p_{com, k}) - v_{des, k}\Delta t \right)^\top Q_{dq} \left((p_{com, k+1} - p_{com, k}) - v_{des, k}\Delta t \right)\\
&+ \sum_{k=1}^N (u_k - u_{des, k})^\top Q_u (u_k - u_{des, k})^\ \\
&+ \sum_{k=1}^N \left((u_{k+1} - u_k) - (u_{des, k+1} - u_{des, k}) \right)^\top Q_{du} \left((u_{k+1} - u_k) - (u_{des, k+1} - u_{des, k}) \right)\\
\textrm{s.t.} & \quad q_{k+1} = Aq_k + Bu_k
\end{align*}
$$