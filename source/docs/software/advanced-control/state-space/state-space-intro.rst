Introduction to state-space control
===================================

.. note:: This article is from `Controls Engineering in FRC <https://file.tavsys.net/control/controls-engineering-in-frc.pdf>`__ by Tyler Veness with permission.
 
From PID to model-based control
-------------------------------

PID controller designers are focused on fiddling with controller parameters relating to the current, past, and future error rather than the underlying system states. While this approach works in a lot of situations, it is an incomplete view of the world.

Model-based control focus on developing an accurate model of the system they are trying to control. These models help inform gains picked for feedback controllers based on the physical responses of the system, rather than an arbitrary proportional gain derived through testing. This allows us not only to predict ahead of time how a system will react, but also test our controllers without a physical robot and save time debugging simple bugs.

..note:: State-space control makes extensive use of linear algebra. More on linear algebra in modern control theory, including an introduction to linear algebra and resources, can be found in Chapter 4 of `Controls Engineering in FRC <https://file.tavsys.net/control/controls-engineering-in-frc.pdf>`__

If you've used WPILib's feedforward classes for ``SimpleMotorFeedforward`` or its sister classes, or used FRC-Characterization to pick PID gains foryou, you're already familiar with model-based control! The ``kv`` and ``ka`` gains can be used to describe how a motor (or arm, or drivetrain) will react to voltage. We can put these constants into standard state-space notation using WPILib's ``LinearSystem``, something we will do in a later article.

Vocabulary
----------

- System: the physical thing being controlled. Has States, Inputs and Outputs associated with it.

- State: A characteristic of a system that can be used to determine the system's future behavior. In state-space notation, the state of a system is written as a column vector describing it's position in state-space.

    - Ex. A drivetrain system might have the states :math:`\begin{bmatrix}x \\ y \\ \theta \end{bmatrix}` [x, y, heading]^T to describe it's position on the field.
    - Ex. An elevator system might have the states [position, velocity]^T to describe its current height and velocity.

- Input: any input to the plant (or physical system) that can change the plant's state. Think about inputs as being put *into* the physical system being controlled.

    - Ex. A flywheel will have 1 input: the voltage of the motor driving it.
    - Ex. A drivetrain might have 2 inputs: the voltages of the left and right motors.

- Output: measurements from sensors. Think about this as information coming *out* of the physical system being controlled. There can be more measurements then states. These outputs are used to "correct"

    - Ex. A flywheel might have 1 output from a encoder that measures it's velocity.
    - Ex. A drivetrain might use solvePNP and V-SLAM to find it's x/y/heading position on the field. It's fine that there are 6 measurements (solvePNP x/y/heading and V-SLAM x/y/heading) and 3 states (robot x/y/heading).

What is state-space notation?
-----------------------------

State-space notation is a set of matrix equations which describe how a system will evolve over time. These equations relate the change in state :math:`\dot{\textbf{x}}`, and the output :math:`\textbf{y}`, to linear combinations of the current state vector :math:`\textbf{x}` and input vector :math:`\textbf{u}`. See section 4.2 of `Controls Engineering in FRC <https://file.tavsys.net/control/controls-engineering-in-frc.pdf>`__ for an introduction to linear combinations. The core idea of linear transformations is that we are simply summing or scaling the elements of :math:`\textbf{x}` and :math:`\textbf{u}`. For example, an operation involving an exponent or trigonometric function would not be considered a linear transformation. The following two sets of equations are the standard form of continuous and discrete state-space notation:

.. math::
    \text{Continuos:}
    \dot{\textbf{x}} &= \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u} \\
    \mathbf{y} &= \mathbf{C}\mathbf{x} + \mathbf{D}\mathbf{u} \\
    \nonumber \\
    \text{Discrete:}
    \mathbf{x}_{k+1} &= \mathbf{A}\mathbf{x}_k + \mathbf{B}\mathbf{u}_k \\
    \mathbf{y}_k &= \mathbf{C}\mathbf{x}_k + \mathbf{D}\mathbf{u}_k

.. math::
  \begin{figurekey}
    \begin{tabular}{llll}
      $\mtx{A}$ & system matrix, states \times states       & $\mtx{x}$ & state vector, states \times 1 \\
      $\mtx{B}$ & input matrix, states \times inputs        & $\mtx{u}$ & input vector, inptus \times 1 \\
      $\mtx{C}$ & output matrix, outputs \times states      & $\mtx{y}$ & output vector, outputs \times 1 \\
      $\mtx{D}$ & feedthrough matrix, outputs \times inputs &  &  \\
    \end{tabular}
  \end{figurekey}

State-space control can deal with "continuous" or "discrete" systems. In decades past, plants and controllers were implemented using analog electronics, which are continuous in nature. These analog controllers didn't have discrete steps to them, analogous to an infinitely fast computer processor. However, processors such as the RoboRIO run in discrete "steps." Systems are often modeled first as continuous systems, and later converted to the discrete form in a process called discretization. WPILib's LinearSystem takes the continuous system matrices, and converts them internally where necessary. 

..note:: Since a microcontroller performs discrete steps, there is a sample delay that introduces phase loss in the controller. Large amounts of phase loss can make a stable controller in the continuous domain become unstable in the discrete domain. The easiest way to combat phase loss and increase performance is to decrease the time between updates. WPILib's ``Notifier`` class if updates faster than the main robot loop are desired. 

State-space notation example -- Flywheel from kV and kA
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Recall that we can model the motion of a flywheel connected to a brushed DC motor with the equation :math:`V = kV \dot v + kA \dot a`, where V is voltage output, v is velocity and a is acceleration. This equation can be rewritten as :math:`a = (V - kV \dot v) / kA`, or :math:`a = ((-kV / kA) \dot v + 1/kA \dot V)`. Notice anything familiar? This equation relates the acceleration of the flywheel to its velocity and the voltage applied. 

We can convert this equation to state-space notation. We can create a system with one state (velocity), one input (voltage), and one output (velocity). Recalling that the first derivative of velocity is acceleration, we can write our equation as follows:

.. math:: 
    \textbf{\dot{x}} &= [\frac{-kV}{kA}] \cdot v + \frac{1}{kA} \cdot V

That's it! That's the state-space model of a system for which we have the kV and kA constants. This same math is use in FRC-Characterization to model flywheels and drivetrain velocity systems.

WPILib's LinearSystemLoop
-------------------------

WPILib's state-space control is based on the ``LinearSystemLoop`` class. This class contains all the components needed to control a mechanism using state-space control. It contains the following members:

- A ``LinearSystem`` representing the continuous state-space equations of the system.
- A Kalman Filter, used to filter noise from sensor measurements.
- A Linear Quadratic Regulator, which combines feedback and feedforward to generate inputs.

As the system being controlled is in discrete domain, we follow the following steps at each update cycle:

- ``predict()`` is called to update the Kalman Filter's state vector estimate :math:`\dot{\textbf{x}}` based on applied inputs.

- ``correct(measurement, nextReference)`` "fuses" the measurement and Kalman Filter :math:`\dot{\textbf{x}}` to update the filter's estimate :math:`\dot{\textbf{x}}. This updated state estimate is used by the Linear Quadratic Regulator to generate an updated input :math`\textbf{u}` to drive the system towards the next reference (or setpoint).

- The updated input is set to motors or other physical actuator.

