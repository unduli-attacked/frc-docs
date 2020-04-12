Introduction to state-space control
===================================

.. note:: This article is from `Controls Engineering in FRC <https://file.tavsys.net/control/controls-engineering-in-frc.pdf>`__ by Tyler Veness with permission.
 
From PID to model-based control
-------------------------------

PID controller designers are focused on fiddling with controller parameters relating to the current, past, and future :term:`error` rather than the underlying system states. While this approach works in a lot of situations, it is an incomplete view of the world.

Model-based control focus on developing an accurate model of the system they are trying to control. These models help inform :term:`gain`\s picked for feedback controllers based on the physical responses of the system, rather than an arbitrary proportional :term:`gain` derived through testing. This allows us not only to predict ahead of time how a system will react, but also test our controllers without a physical robot and save time debugging simple bugs.

.. note:: State-space control makes extensive use of linear algebra. More on linear algebra in modern control theory, including an introduction to linear algebra and resources, can be found in Chapter 4 of `Controls Engineering in FRC <https://file.tavsys.net/control/controls-engineering-in-frc.pdf>`__

If you've used WPILib's feedforward classes for ``SimpleMotorFeedforward`` or its sister classes, or used FRC-Characterization to pick PID :term:`gain`\s for you, you're already familiar with model-based control! The ``kv`` and ``ka`` :term:`gain`\s can be used to describe how a motor (or arm, or drivetrain) will react to voltage. We can put these constants into standard state-space notation using WPILib's ``LinearSystem``, something we will do in a later article.

Vocabulary
----------

For the background vocabulary that will be used throught this article, see the `Glossary`_.

What is state-space?
--------------------

Recall that 2D space has two axes: x and y. We represent locations within this space as a pair of numbers packaged in a vector, and each coordinate is a measure of how far to move along the corresponding axis. State-space is a Cartesian coordinate system with an axis for each state variable, and we represent locations within it the same way we do for 2D space: with a list of numbers in a vector. Each element in the vector corresponds to a state of the system. This example shows two example state vectors in the state-space of an elevator model with the states :math`[\text{position}, \text{velocity}]`

.. image:: images/state-space-graph.png

In this image, the vectors representing states in state-space are arrows. From now on these vectors will be represented simply by a point at the vector's tip, but remember that the rest of the vector is still there.

In addition to the state, inputs and :term:`output`\s are represented as vectors. Since the mapping from the current states and inputs to the change in state is a system of equations, it’s natural to write it in matrix form. This matrix equation can be written in state-space notation.

What is state-space notation?
-----------------------------

State-space notation is a set of matrix equations which describe how a system will evolve over time. These equations relate the change in state :math:`\dot{\mathbf{x}}`, and the :term:`output` :math:`\mathbf{y}`, to linear combinations of the current state vector :math:`\mathbf{x}` and :term:`input` vector :math:`\mathbf{u}`. See section 4.2 of `Controls Engineering in FRC <https://file.tavsys.net/control/controls-engineering-in-frc.pdf>`__ for an introduction to linear combinations. The core idea of linear transformations is that we are simply summing or scaling the elements of :math:`\mathbf{x}` and :math:`\mathbf{u}`. For example, an operation involving an exponent or trigonometric function would not be considered a linear transformation. 

State-space control can deal with continuous-time and discrete-time systems. A continuous-time system is modeled by a system of differential equations (as seen in the continuous-time case below). However, modern computer processors such as the RoboRIO run in discrete "steps," making it impossible to precisely model a system that is constantly evaluated. A continuous state-space system can be converted into a discrete-time system through a process called discretization. A discrete-time system expresses the state of the system at our next timestep based on the previous state and inputs, as opposed to the state derivative :math:`\dot{\mathbf{x}}`.

The following two sets of equations are the standard form of continuous and discrete state-space notation:

.. math::
    \text{Continuos: }
    \dot{\mathbf{x}} &= \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u} \\
    \mathbf{y} &= \mathbf{C}\mathbf{x} + \mathbf{D}\mathbf{u} \\
    \nonumber \\
    \text{Discrete: }
    \mathbf{x}_{k+1} &= \mathbf{A}\mathbf{x}_k + \mathbf{B}\mathbf{u}_k \\
    \mathbf{y}_k &= \mathbf{C}\mathbf{x}_k + \mathbf{D}\mathbf{u}_k

.. math::
    \begin{tabular}{llll}
      $\mathbf{A}$ & system matrix      & $\mathbf{x}$ & state vector \\
      $\mathbf{B}$ & input matrix       & $\mathbf{u}$ & input vector \\
      $\mathbf{C}$ & output matrix      & $\mathbf{y}$ & output vector \\
      $\mathbf{D}$ & feedthrough matrix &  &  \\
    \end{tabular}

Systems are often modeled first as continuous systems, and later converted to the discrete form. WPILib's LinearSystem takes the continuous system matrices, and converts them internally where necessary. 

..note:: Since a microcontroller performs discrete steps, there is a sample delay that introduces phase loss in the controller. Large amounts of phase loss can make a stable controller in the continuous domain become unstable in the discrete domain. The easiest way to combat phase loss and increase performance is to decrease the time between updates. WPILib's ``Notifier`` class can be used if updates faster than the main robot loop are desired. 

State-space notation example -- Flywheel from kV and kA
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Recall that we can model the motion of a flywheel connected to a brushed DC motor with the equation :math:`V = kV \dot v + kA \dot a`, where V is voltage output, v is the flywheel's angular velocity and a is its angular acceleration. This equation can be rewritten as :math:`a = (V - kV \dot v) / kA`, or :math:`a = ((-kV / kA) \dot v + 1/kA \dot V)`. Notice anything familiar? This equation relates the angular acceleration of the flywheel to its angular velocity and the voltage applied. 

We can convert this equation to state-space notation. We can create a system with one state (velocity), one :term:`input` (voltage), and one :term:`output` (velocity). Recalling that the first derivative of velocity is acceleration, we can write our equation as follows:

.. math:: 
    \mathbf{\dot{x}} &= [\frac{-kV}{kA}] \cdot v + \frac{1}{kA} \cdot V

That's it! That's the state-space model of a system for which we have the kV and kA constants. This same math is use in FRC-Characterization to model flywheels and drivetrain velocity systems.

Visualizing State-space responses: phase portrait
-------------------------------------------------

A `phase portrait <https://en.wikipedia.org/wiki/Phase_portrait>`__ can help give a visual intuition for the response of a system in state-space. The vectors on the graph have their roots at some point :math:`\mathbf{x}` in state-space, and point in the direction of :math:`\mathbf{\dot{x}}`, the direction that the system will evolve over time. This example shows a model of a pendulum with the states of angle and angular velocity. 

.. .. raw:: html

..     <div style="text-align: center; margin-bottom: 2em;">
..     <iframe width="100%" height="350" src="https://raw.githubusercontent.com/mcm001/state-space-animations/master/videos/phase-space/720p30/PendulumCirclingOrigin.mp4" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
..     </div>


To trace a potential trajectory that a system could take through state-space, choose a point to start at and follow the arrows around. In this example, we might start at :math:`[-2, 0]`. From there, the velocity increases as we swing through vertical and starts to decrease until we reach the opposite extreme of the swing. This cycle of spinning about the origin repeats indefinitely.

.. image:: images/pendulum-markedup.jpg

Note that near the edges of the phase plot, the X axis wraps around as a rotation of :math:`\pi` radians counter clockwise and a rotation of :math:`\pi` radians clockwise will end at the same point.

For more on differential equations and phase portraits, see `3Blue1Brown's Differential Equations video <https://www.youtube.com/watch?v=p_di4Zn4wz4>`__ -- they do a great job of animating the pendulum phase space at around 15:30.

Visualizing Feedforward
~~~~~~~~~~~~~~~~~~~~~~~

This phase portrait shows the "open loop" responses of the system -- that is, how it will react if we were to let the state evolve naturally. If we want to, say, balance the pendulum horizontal (at :math:`(\frac{\pi}{2}, 0)` in state space), we would need to somehow apply a control :term:`input` to counteract the open loop tendency of the pendulum to swing downward. This is what feedforward is trying to do -- make it so that our phase portrait will have an equilibrium at the :term:`reference` position (or setpoint) in state-space. Looking at our phase portrait from before, we can see that at :math:`(\frac{\pi}{2}, 0)` in state space, gravity is pulling the pendulum down with some torque T, and producing some downward angular acceleration with magnitude :math:`\frac{\tau}{i}`, where I is angular `moment of inertia <https://en.wikipedia.org/wiki/Moment_of_inertia>`__ of the pendulum. If we want to create an equilibrium at our :term:`reference` of :math:`(\frac{\pi}{2}, 0)`, we would need to apply an :term:`input` that produces a :math:`\mathbf{\dot{x}}` is equal in magnitude and opposite in direction to the :math:`\mathbf{\dot{x}}` produced by the system's open-loop response to due to gravity. The math for this will be presented later. Here is the phase portrait where we apply a constant :term:`input` that opposes the force of gravity at :math:`(\frac{\pi}{2}, 0)`:

.. image:: images/pendulum-balance.png

Feedback Control and LQR
------------------------

In the case of a DC motor, with just a mathematical model and knowledge of all current states of the system(i.e., angular velocity), we can predict all future states given the future voltage inputs. But if the system is disturbed in any way that isn’t modeled by our equations, like a load or unexpected friction,the angular velocity of the motor will deviate from the model over time. To combat this, we can give the motor corrective commands to account for model uncertainty. 

A PID controller is a form of feedback control. State-space control often uses the control law (a mathematical formula that generates inputs to drive a system to a desired state) :math:`\mathbf{u} = \mathbf{K(r - x)}`, where K is some controller :term:`gain` matrix, r is the :term:`reference`\state and x is the current state in state-space. The difference between these two vectors, :math:`r - x`, is known as :term:`error`. This control law is essentially a multidimensional proportional controller. Because model-based control means that we can predict the future states of a system given an initial condition and future control inputs, we can pick a mathematically optimal :term:`gain` matrix K. 

Let's start with the open loop pendulum example. The case where K is the zero matrix would mean that no control :term:`input` is applied, and the phase portrait would look identical to the one above. Let's pick a K of [[2, 0], [0, 2]], where are :term:`input` to the pendulum is angular acceleration. This K would mean that for every degree of position :term:`error`, the angular acceleration would be 1 degree per second squared; similarly, we accelerate by 1 degree per second squared for every degree per second of :term:`error`. Try following an arrow from somewhere in state-space inwards -- no matter the initial conditions, the state will settle at the :term:`reference` rather than circle endlessly with pure feedforward. 

.. image:: images/pendulum-closed-loop.png

But with a real system, how can we choose an optimal :term:`gain` matrix K? While we can manually choose :term:`gain`\s and simulate the system response, or use tools like pole placement, modern control theory has a better answer: the Linear Quadratic Regulator (LQR).

The Linear Quadratic Regulator
------------------------------

Linear Quadratic Regulators pick the closed loop :term:`gain` matrix :math:`\mathbf{K}` for us based on acceptable:term:`error` and :term:`control effort` constraints for linear systems. LQR works by minimizing the sum of:term:`error` and :term:`control effort` over time.

.. math::
    J = \int\limits_0^\infty \left(\mathbf{x}^T\mathbf{Q}\mathbf{x} +
    \mathbf{u}^T\mathbf{R}\mathbf{u}\right) dt

where :math:`\mathbf{J}` represents a trade-off between the state excursion and :term:`control effort`. The trade-off is weighted with the  factors :math:`\mathbf{Q}` and :math:`\mathbf{R}`, where :math:`\mathbf{Q}` weights state excursion and :math:`\mathbf{R}` weights :term:`control effort`.

The minimum of LQR's cost function is found by setting the derivative of the cost function to zero and solving for the control law :math:`\mathbf{u}`. However, matrix calculus is used instead of normal calculus to take the derivative.

.. note:: LQR design's :math:`\mathbf{Q}` and :math:`\mathbf{R}` matrices don't need discretization, but the :math:`\mathbf{K}` calculated for continuous time and discrete time system will be different.

The next obvious question is what values to choose for :math:`\mathbf{Q}` and :math:`\mathbf{R}`. While :math:`\mathbf{Q}` and :math:`\mathbf{R}` can be chosen almost arbitrary, Bryson's rule provides a simple form for these cost matrices. With Bryson's rule, the diagonals of the :math:`\mathbf{Q}` and :math:`\mathbf{R}` matrices are chosen based on the maximum acceptable value for each \gls{state} and actuator. The nondiagonal elements are zero.

.. math::   
    \begin{array}{cc}
        \mathbf{Q} = \begin{bmatrix}
            \frac{\rho}{x_{1,max}^2} & 0 & \ldots & 0 \\
            0 & \frac{\rho}{x_{2,max}^2} & & \vdots \\
            \vdots & & \ddots & 0 \\
            0 & \ldots & 0 & \frac{\rho}{x_{n,max}^2}
        \end{bmatrix} &
        \mathbf{R} = \begin{bmatrix}
            \frac{1}{u_{1,max}^2} & 0 & \ldots & 0 \\
            0 & \frac{1}{u_{2,max}^2} & & \vdots \\
            \vdots & & \ddots & 0 \\
            0 & \ldots & 0 & \frac{1}{u_{n,max}^2}
        \end{bmatrix}
    \end{array}

where the weighting factor :math:`\rho` can be used to change the balance of :term:`control effort` and state excursion. Small values of :math:`\rho` penalize :term:`control effort`, while large values of :math:`\rho` penalize state excursion. The values of :math:`x_1, x_2...x_m` are the maximum desired :term:`error` tolerance for each state of the system, and :math:`u_1, u_2...u_n` are maximum desired :term:`control effort`\s for each input. WPILib's LinearQuadraticRegulator takes simply a list of :math:`x_1, x_2...x_m` elements for Q and :math:`u_1, u_2...u_n` for R. By choosing Q and R elements to feed to an LQR through Bryson's rule, the response of the plat can be tuned.

For example, take a flywheel velocity system determined through system identification to have kV = 2.9 volts per radian per second and kA = 0.3 volts per radian per second squared. Because we would like our flywheel to be within 0.1rad/sec of the :term:`reference` and apply at most 12 volts, we choose q = 0.1 and r = 12.0 to give to Bryson's rule and compute LQR with. After discretization with a timestep of 20ms, we find a :term:`gain` of K = ~13. This K :term:`gain` can be thought exactly as the Proportional of a PID loop on flywheel's velocity. If this were true, we'd except that increasing the q elements or decreasing the r elements we give Bryson's rule would make our controller more heavily penalize :term:`control effort`, analogous to trying to conserve fuel in a space ship or drive a car more conservatively by applying less gas. In fact, if we increase our :term:`error` tolerance q from 0.1 to 1.0, our :term:`gain` K drops from ~13 to ~6. Similarly, decreasing our maximum voltage r to 1.2 from 12.0 would have produced the same resultant K.

WPILib's LinearSystemLoop
-------------------------

WPILib's state-space control is based on the ``LinearSystemLoop`` class. This class contains all the components needed to control a mechanism using state-space control. It contains the following members:

- A ``LinearSystem`` representing the continuous state-space equations of the system.
- A Kalman Filter, used to filter noise from sensor measurements.
- A Linear Quadratic Regulator, which combines feedback and feedforward to generate inputs.

As the system being controlled is in discrete domain, we follow the following steps at each update cycle:

- ``correct(measurement, nextReference)`` "fuses" the measurement and Kalman Filter :math:`\dot{\mathbf{x}}` to update the filter's estimate :math:`\dot{\mathbf{x}}`. This updated state estimate is used by the Linear Quadratic Regulator to generate an updated :term:`input` :math`\mathbf{u}` to drive the system towards the next :term:`reference` (or setpoint).

- ``predict()`` is called to update the Kalman Filter's state vector estimate :math:`\dot{\mathbf{x}}` based on applied inputs.

- The updated :term:`input` is set to motors or other physical actuator.


Glossary
========

.. glossary::

    Control Effort
        A term describing how much force, pressure, etc. an actuator is exerting

    Controller
        Applies an :term:`input` to a :term:`plant` to drive the difference between a :term:`reference` and :term:`output`, or :term:`error`, to zero.

    Error
        :term:`Reference` minus an output or state.

    Gain
        A proportional value that relates the magnitude of an input signal to the magnitude of an output signal. In the signal-dimensional case, gain can be thought of as the proportional term of a PID controller. A gain greater than one would amplify an input signal, while a gain less than one would dampen an input signal. A gain less than one would negate the input.

    Input
        Any :term:`input` to the :term:`plant` (or physical system) that can change the :term:`plant`\'s state. Think about inputs as being put *into* the physical system being controlled.

            - Ex. A flywheel will have 1 input: the voltage of the motor driving it.
            - Ex. A drivetrain might have 2 inputs: the voltages of the left and right motors.

    Observer
        In control theory, a system that provides an estimate of the internal :term:`state` of a given real :term:`system` from measurements of the :term:`input` and :term:`output` of the real :term:`system`. WPILib includes a Kalman Filter class for observing linear systems, and ExtendedKalmanFilter and UnscentedKalmanFilter classes for nonlinear systems. TODO maybe clarify more?

    Output
        Measurements from sensors. Think about this as information coming *out* of the physical system being controlled. There can be more measurements then states. These outputs are used to "correct"

            - Ex. A flywheel might have 1 :term:`output` from a encoder that measures it's velocity.
            - Ex. A drivetrain might use solvePNP and V-SLAM to find it's x/y/heading position on the field. It's fine that there are 6 measurements (solvePNP x/y/heading and V-SLAM x/y/heading) and 3 states (robot x/y/heading).

    Plant
        The system or collection of actuators being controlled. 

    Reference
        The desired :term:`state`. This value is used as the reference point for a :term:`controller` 's :term:`error` calculation.

    System
        The physical thing being controlled. Has States, Inputs and Outputs associated with it. In state-space control, a System refers to a :term:`plant` as well as it's interactions with a controller and observer. Mathematically, a system maps inputs to outputs through a linear combination of :term:`state`\s.

    State
        A characteristic of a system that can be used to determine the system's future behavior. In state-space notation, the state of a system is written as a column vector describing it's position in state-space.

            - Ex. A drivetrain system might have the states :math:`\begin{bmatrix}x \\ y \\ \theta \end{bmatrix}` [x, y, heading]^T to describe it's position on the field.
            - Ex. An elevator system might have the states [position, velocity]^T to describe its current height and velocity.


Common Variable Names
---------------------

- :math:`\mathbf{x}`, the :term:`state` vector. A column vector with one entry per :term:`state`.
- :math:`\dot{\mathbf{x}}`, or xdot: the derivative of the :term:`state` vector :math:`\mathbf{x}`. If the :term:`system` had just a velocity :term:`state`, then :math:`\dot{\mathbf{x}}` would represent the :term:`system`\'s acceleration.
- :math:`\hat{\mathbf{x}}`, or xhat: the estimated :term:`state` of a system, as estimated by an :term:`observer`. 
- :math:`\mathbf{u}`, or control :term:`input`. A column vector with one entry per :term:`input` to the :term:`system`.
- :math:`\mathbf{y}`, the :term:`output`, or measurement, vector. A column vector with one entry per :term:`output` (or thing we can measure) of the :term:`system`. For example, if our :term:`system` had states for velocity and acceleration but our sensor could only measure velocity, our, our :term:`output` vector would only include the :term:`system`\'s velocity.