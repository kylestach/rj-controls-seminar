---
title: Week 1 Slides
---
# Motion Planning and Control
## Week 1: Model-Based Control
### Kyle Stachowicz

---

# Introduction
<!-- 3 minutes -->

---

# Prerequisites
<!-- 5 minutes -->

 - Mathematics (_know the concepts_)
    - Linear algebra: matrices, matrix operations
    - Calculus: derivatives, partial derivatives, Jacobians, Hessians, Taylor expansion
    - Differential equations: basic concepts only

---

# Prerequisites
 - Programming
    - Rudimentary C++, Python, or Matlab
 - Controls
    - Basics of PID control

---

# Prerequisites
_If all of these sound familiar, great! If not, it's not the end of the world - we will go
over them as we run into them to refresh everyone's memory, but it might
be a bit fast-paced if you've never seen them in a class before._

---

# Exercises
https://bit.ly/38urqWL

Click "Open in Playground" to copy this notebook to your Google Drive.

https://github.com/kylestach/rj-controls-seminar

---

# What is a system?
<!-- 4 minutes -->
![image](system_block.jpg)
 - Inputs
 - Outputs

---

# What is a system?
<!-- 3 minutes -->
![image](system_block.jpg)
 - Inputs `$u$`
 - _Internal state_ `$x$`
 - Outputs `$y$`

---

_Note\: For most of these lectures we'll assume full access to state variables `$x$`.
 We may talk in a later lecture about how exactly you can get this information from your outputs._

---

# Shooter Wheel
<!-- 3 minutes -->
![image](shooter_wheel.png)
 - Input: Motor voltage
 - State: Wheel velocity
 - Output: Wheel velocity measurement (encoder)

---

# Differential drive
![image](diffdrive.png)
 - Input: Left/right motor voltage
 - State: Robot `$(x, y, \theta)$` position, velocity
 - Output: Wheel velocity measurements (encoders), gyro,
   other sensors (camera, LIDAR, etc.)
 
---

# Omni Drive
![image](robocup.jpg)
 - Input: Four wheel PWM values
 - State: Robot `$(x, y, \theta)$` position, velocity
 - Output: Wheel velocity measurements (wheel encoders), IMU,
   camera measurements

---

# Controllers
![image](controller_block.jpg)
 - Gets sensor data from system outputs
 - Calculates control signal
 - Outputs results to actuators (system **inputs**)
 - Usually, we have some _goal_ or _reference_ for what we want the system's state to be

---

# Open-loop Control
 - Calculate the control signal that you _expect_ to make the system do what you want.
 - Example: for a shooter wheel, calculate the steady-state voltage at the desired speed.
    - **Definition** _Steady-state: the limit as time goes to infinity and the state stops changing._

---

## So we've solved control, right? Pack it up and go home?**

Yeah, not quite. This only works if the system dynamics are _exactly_ how you modeled them...

 - Multiple robots
 - Different ground conditions
 - Low battery

---

# Bang-Bang Control
<!-- 3 minutes -->
 - When state is less than reference, give `$u_max$`.
 - When state is greater than reference, give `$u_min$`.

---

# PID(F) Control
<!-- 10 minutes -->
Problem\: bang-bang is too aggressive, we want to give smaller control signal when we're close.

_Solution: make control signal **proportional** to the error (goal - current state)._
`$$e = x - r$$`
`$$u = k_pe$$`

---

# PID(F) Control
Problem\: now we don't actually reach the target.
 - What happens when `$e = 0$`?

_Solution 1: combine this with open-loop control! This is called feed-forward._
`$$u_t = k_pe_t + u_ff$$`

---

# PID(F) Control
Problem\: we still don't always reach the target
 - What happens when `$e = 0$` and there are still modeling errors?

_Solution 2: take the **history** of the error into account._

---

# PID(F) Control
Add in the _integral_ (running sum) of errors:
`$$u_t = k_pe_t + k_i\sum_{i=1}^te_i + u_ff$$`

---

# PID(F) Control
Now we overshoot the target! We need to "dampen" the system by slowing it down when it's changing too fast.

Add in the _derivative_ (rate of change) of error:
`$$u_t = k_pe_t + k_i\sum_{i=1}^te_i + k_d(e_t - e_{t-1})+ u_ff$$`

---

# Notes on PID
 - Tuning integral sucks because of _integral windup_
    - Minimize it if you can; integral just compensates for model errors
    - Integral on velocity can be okay, integral on position is just a miserable time.
 - Look up Zeiger-Nichols if you're interested

---

# Exercise: PID and Bang-Bang
<!-- 10 minutes -->

---

# Break
(5 minutes)

---

# Review: Diff. Eq
<!-- 3 minutes -->

| | |
|---|---|
| ![image](diffeq.png) | Let `$x \in \mathbb{R}^n$`. Then `$$\frac{dx}{dt} = f(x)$$` Find `$x(t)$` given `$x(0)$` |

---

# Modeling a System
<!-- 8 minutes -->
1. Choose `$n$` state variables and `$m$` control inputs
   - Position and velocity, motor PWM or voltage
2. Write down equations from physics
   - Acceleration: `$F = ma, \tau = J\alpha$`.
   - Motors: `$\tau = K_ti, V_{emf} = K_v\omega$`.
   - Electronics: `$V = IR$` (or similar).
3. Derive `$f$` such that `$\frac{dx}{dt} = f(x, u)$`.

---

# Review: Matrices
<!-- 4 minutes -->
 - Matrices represent a system of linear equations:
`$$y_1 = a_1x_1 + a_2x_2, y_2 = a_3x_1 + a_4x_2$$`
`$$\begin{pmatrix}y_1 \\ y_2\end{pmatrix} = \begin{pmatrix}a_1 & a_2 \\ a_3 & a_4\end{pmatrix}\begin{pmatrix}x_1 \\ x_2\end{pmatrix}$$`

---

# Review: Matrices
- Geometric intuition:
![image](matmul.gif)

---

# Example: Motors
<!-- 8 minutes -->
   - Acceleration: `$\tau = J\alpha$` for rotation
   - Motor equations: `$\tau = K_ti$`, `$V_{emf} = K_v\omega$`.
   - Electronics: `$V = IR$`.
   - Gear ratio: `$\tau_{in} = G\tau_{out}$`, `$\omega_{out} = G\omega_{in}$`.

---

# Linear Systems
<!-- 4 minutes -->
 - Some systems are easier to work with than others.
 - The most obvious example is linear systems:
_Definition: a system is linear if `$f$` is a linear function of `$x$` and `$u$`._

---

# Representing Linear Systems
<!-- 4 minutes -->
 - From linear algebra: any linear function `$f(x, u)$` can be written using matrices:
 `$$f(x, u) = Ax + Bu$$`

---

# Exercise: State-Space Equations for a Motor
<!-- 5 minutes -->
`$$\dot\omega = \frac{K_t}{JR}V - \frac{K_tK_v}{JR}\omega$$`

---

# Review: Diff Eq. Solutions
Fact: with one variable, `$\dot x = ax$` the solutions are `$x_0e^{at}$`.
With multiple variables and `$\dot x = Ax$` the solutions are `$x_0e^{At}$`,
but how do we take `$e$` to the power of a matrix?

---

# Review: Eigenvalues
The _eigenvectors_ `$v$` of a matrix are the vectors only change length.
The _eigenvalues_ `$\lambda$` are the amount these vectors are stretched.

![image](eigenvalues.gif)

---

# Solution to State-Space Equations
<!-- 5 minutes -->
Fact: we can find `$\frac{dx}{dt} = Ax$` for `$x(0) = v$` easily:
`$$\dot x = Ax = \lambda x \implies x(t) = x(0)e^{\lambda t}$$`

Fact: we can write other solutions by writing linear combinations of "eigensolutions".

---

# Full-State Feedback
Let's go back to talking about the full model\: `$\dot x = Ax + Bu$`. How do we calculate `$u$`?

Well, let's try making it linear: `$u = -Kx$` for some matrix `$K$`.

_Question: `$K$` is analogous to some other constants we talked about earlier. Which ones?_

---

# Pole Placement
If we have `$\dot x = Ax + Bu = (A - BK)x$`, it's like our system is an unforced system!

How can we use this? Well, let's pick the eigenvalues of our system.

---

# Pole Placement
_What properties do we want our eigenvalues to have? Recall: `$e^{at}$`_
 - For negative `$a$`, this converges as time goes to infinity. This is good.
 - For positive `$a$`, this diverges (explodes) as time goes to infinity. This is bad.
 - For complex `$a = \sigma + j\omega$`, it will act like a exponentially-decaying sinusoid:

---

# Exponentials
![image](s_plane.png)

---

# Pole Placement
We want our `$\lambda$`s to be negative in the real part!

More negative values will cause more aggressive controllers.

---

# Exercise: Pole Placement

---

# Motion profiling
 - So far we tell our systems to jump instantly to the right goal
 - Is this a problem?

---

# Motion profiling
 - So far we tell our systems to jump instantly to the right goal
 - Is this a problem? _Maybe_

---

# Motion profiling
 - Saturation
 - Stress

---

# Saturation
 - Look back at your plots, especially at `$u$`.
 - Can we give arbitrary controls in real life?
 - What happens when we _think_ we can but we can't?

---

# Motion profiling
 - Limit _acceleration_ and _velocity_
![image](motion_profiling.jpg)

---

# Thanks for coming!
Next seminar: Multi-Input Systems and Optimal Control - February 29