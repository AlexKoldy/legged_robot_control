# Whole Body Control
<div style="display: flex; align-items: center;">
    <img src="https://github.com/AlexKoldy/whole_body_control/assets/52255127/b92ee60c-a9ff-4868-b9ee-1ad66abe9ab7" height="235" alt="First GIF">
    <img src="https://github.com/AlexKoldy/whole_body_control/assets/52255127/f3d5e513-711c-46dd-b3d8-7477760e9f7c" height="235" alt="Second GIF">
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

## Whole Body Controller
We represent the dynamics of the robot via 

$$
M\dot{v}+Cv+G = Bu + J^\top\_h \lambda\_h
$$

where $M$ is the mass matrix, $Cv$ is the Coriolis term, $G$ is the gravitational force vector, $B$ is the actuation matrix and $J$ is the Jacobian that relates the velocities of the contact points to the velocities of the robot. The state vector is represented by

$$
x = \begin{bmatrix}
q \\
v
\end{bmatrix}
$$

where $q$ represents the quaternion orientation of the pelvis link, the position of the CoM and the joint positions, while $v$ represents the angular velocity of the pelvis, the velocity of the CoM and the joint velocities. In the case of our high-frequency whole-body controller, we apply $\hat{x}$, a constant estimate of our state, meaning $x$ itself is not a decision variable in our later optimization. This means that the dynamics become linear with respect to our decision variables $\dot{v}$, $u$ and $\lambda\_h$.

Now, we have some list of tracking objectives for our robot. These include a CoM trajectory, torso angle and foot trajectories. Let's defined $y\_i$ as our task space state for our $i\text{th}$ tracking objective. We can relate the derivative of our task state space to our configuration with

$$
\begin{align*}
\dot{y}\_i &= \frac{\partial\_i}{\partial q}\dot{q} = J\_iv  \\
\ddot{y}\_i &= \dot{J}\_iv + J\dot{v}
\end{align*}
$$


For each tracking objective, we apply task space PD control, i.e., 

$$
\ddot{y}\_{cmd, i}=\ddot{y}\_{des, i} + k\_p(y\_{des, i} - y\_i)+k\_d(\dot{y}\_{des, i} - \dot{y}\_i)
$$

where $\ddot{y}\_{cmd, i}$ will be used as a reference to our whole-body controller and $y\_{des}$ comes from our planned trajectories or objectives, i.e., what we actually want to track. 

Before we formulating the optimization problem, we formulate the contact and friction constraints. First, we want to enforce that the acceleration of the stance foot (or stance feet in double support) via

$$
\dot{J}\_hv + J\_h\dot{v} = 0
$$

Next, we build a two-dimensional friction cone as a linear inequality constraint, i.e., we use a linear representation of the friction cone to preserve convexity

$$
\begin{bmatrix}
-1 & 0 & -\mu \\
1 & 0 & -\mu \\
\end{bmatrix}\lambda\_h \leq \begin{bmatrix} 
0 \\ 0
\end{bmatrix}
$$

where $\mu$ is the friction coefficient. Let's call the matrix on the left hand side $A$ and the vector on the right hand side $b$ such that $A\lambda\_h \leq b$. Given our task state space and our task space command, we formulate the following cost function

$$
J = \min\_{u, \dot{v}, \lambda\_h} \quad \sum\_{i=0}(\ddot{y}\_i-\ddot{y}\_{cmd, i})^\top W\_i (\ddot{y}\_i-\ddot{y}\_{cmd, i})
$$

where $W\_i$ is the tracking cost of the $i\text{th}$ tracking objective. Due to $\ddot{y}\_i$ being an affine function of $\dot{v}$, we can represent this cost as a quadratic function of our decision variable $\dot{v}$:

$$
(\ddot{y}\_i-\ddot{y}\_{cmd, i})^\top W\_i (\ddot{y}\_i-\ddot{y}\_{cmd, i}) = \frac{1}{2}\dot{v}^\top Q\_i \dot{v} + b\_i^\top \dot{v} + c\_i
$$

where 

$$
\begin{align*}
Q\_i &= 2J\_i^\top W\_iJ\_i\\
b\_i &= -2J\_i^\top W\_i (\ddot{y}\_{cmd, i} - \dot{J}\_iv) \\
c\_i &= (\ddot{y}\_{cmd, i} - \dot{J}\_iv)^\top W\_i (\ddot{y}\_{cmd, i} - \dot{J}\_iv)
\end{align*}
$$

We can finally formulate our whole-body controller optimization problem as a convex quadratic program as follows:

$$
\begin{align*}
\min\_{u, \dot{v}, \lambda\_h} & \quad \sum\_{i=0}\frac{1}{2}\dot{v}^\top Q\_i \dot{v} + b\_i^\top \dot{v} + c\_i \\
\textrm{s.t.} & \quad M\dot{v} + Cv + G = Bu + J\_h^\top \lambda\_h \\
& \quad \dot{J}\_hv + J\_h\dot{v} = 0 \\
& \quad A\lambda\_h  \leq b \\
& \quad u\_{min} \leq u \leq u\_{max}
\end{align*}
$$
