# Whole Body Control
<div style="display: flex; align-items: center;">
    <img src="https://github.com/AlexKoldy/whole_body_control/assets/52255127/b92ee60c-a9ff-4868-b9ee-1ad66abe9ab7" height="255" alt="First GIF">
    <img src="https://github.com/AlexKoldy/whole_body_control/assets/52255127/f3d5e513-711c-46dd-b3d8-7477760e9f7c" height="255" alt="Second GIF">
</div>





## Footstep Planner
To achieve high-frequency footstep planning, we can use the Linear Inverted Pendulum Model (LIP), i.e., a point mass, approximating the robot's center of mass (CoM), attached to a massless prismatic actuator. The acceleration of the point mass is given in [this paper](https://ieeexplore.ieee.org/document/6907116):

$$
\ddot{p}\_{com}=\frac{g}{h\_{com}}(p\_{com}-p\_{base})
$$

where $g$ is the gravitational constant, $h\_{com}$ is the constant height of the CoM and $p\_{com}$ and $p\_{base}$ are the $x$, $y$ CoM and base (foot) positions, respectively. The equations of motion of the LIP are given by

$$
\begin{bmatrix}
x\_{com}\\
x\_{com} \\
v\_{x, com} \\
v\_{y, com} \\
\end{bmatrix}\_{k+1} = \begin{bmatrix}\frac{1}{2e^{\frac{\Delta t}{\tau}}} + \frac{e^{\frac{\Delta t}{\tau}}}{2} & 0 & \frac{-\tau}{2e^{\frac{\Delta t}{\tau}}} + \frac{\tau e^{\frac{\Delta t}{\tau}}}{2} & 0 \\
0 & \frac{1}{2e^{\frac{\Delta t}{\tau}}} + \frac{e^{\frac{\Delta t}{\tau}}}{2} & 0 & \frac{-\tau}{2e^{\frac{\Delta t}{\tau}}} + \frac{\tau e^{\frac{\Delta t}{\tau}}}{2}\\
\frac{e^{\frac{\Delta t}{\tau}}}{2\tau} - \frac{1}{2\tau e^{\frac{\Delta t}{\tau}}} & 0 & \frac{e^{\frac{\Delta t}{\tau}}}{2} + \frac{1}{2e^{\frac{\Delta t}{\tau}}} & 0 \\
0 & \frac{e^{\frac{\Delta t}{\tau}}}{2\tau} - \frac{1}{2\tau e^{\frac{\Delta t}{\tau}}} & 0 & \frac{e^{\frac{\Delta t}{\tau}}}{2} + \frac{1}{2e^{\frac{\Delta t}{\tau}}}
\end{bmatrix} \begin{bmatrix}
x\_{com}\\
x\_{com} \\
v\_{x, com} \\
v\_{y, com} \\
\end{bmatrix}\_{k}
$$

$$
+\begin{bmatrix}
\frac{-1}{2e^{\frac{\Delta t}{\tau}}} - \frac{e^{\frac{\Delta t}{\tau}}}{2} & 0 \\
0 & \frac{-1}{2e^{\frac{\Delta t}{\tau}}} - \frac{e^{\frac{\Delta t}{\tau}}}{2} \\
\frac{1}{2\tau e^{\frac{\Delta t}{\tau}}} - \frac{e^{\frac{\Delta t}{\tau}}}{2\tau} & 0 \\
0 & \frac{1}{2\tau e^{\frac{\Delta t}{\tau}}} - \frac{e^{\frac{\Delta t}{\tau}}}{2\tau} 
\end{bmatrix} \begin{bmatrix}
x\_{base} \\
y\_{base}
\end{bmatrix}\_k
$$

where $\tau = \sqrt{\frac{h\_{com}}{g}}$ and $\Delta t$ is a chosen discretization time-step. We can represent this discrete linear system as 

$$
q\_{k+1} = Aq\_k + Bu\_k
$$

where the input $u\_k$ is the foot position. In order to build the cost, we plan a reference plan as done in [this paper](https://www.researchgate.net/publication/301463868\_Robust\_and\_Agile\_3D\_Biped\_Walking\_With\_Steering\_Capability\_Using\_a\_Footstep\_Predictive\_Approach). Using this reference path, our final cost function for the MPC looks as followers:

$$
\begin{align*}
\min\_{q, u} & \quad \sum\_{k=1}^N (p\_{com, k+1} + p\_{com, k} - 2p\_{m, k})^\top Q\_q (p\_{com, k+1} + p\_{com, k} - 2p\_{m, k})\\
&+ \sum\_{k=1}^N \left((p\_{com, k+1} - p\_{com, k}) - v\_{des, k}\Delta t \right)^\top Q\_{dq} \left((p\_{com, k+1} - p\_{com, k}) - v\_{des, k}\Delta t \right)\\
&+ \sum\_{k=1}^N (u\_k - u\_{des, k})^\top Q\_u (u\_k - u\_{des, k})^\ \\
&+ \sum\_{k=1}^N \left((u\_{k+1} - u\_k) - (u\_{des, k+1} - u\_{des, k}) \right)^\top Q\_{du} \left((u\_{k+1} - u\_k) - (u\_{des, k+1} - u\_{des, k}) \right)\\
\textrm{s.t.} & \quad q\_{k+1} = Aq\_k + Bu\_k
\end{align*}
$$

where $p_m$ is the midpoint between feet and $u_{des}$ and $v_{des}$ are the desired foot positions and CoM body frame velocities given by the reference path computations.
