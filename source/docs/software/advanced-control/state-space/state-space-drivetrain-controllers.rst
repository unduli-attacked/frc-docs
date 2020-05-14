State-space Differential Drive Controllers
==========================================

WPILib includes two new path following controls for differential drivetrains: the ``LTVUnicycleController`` and ``LTVDiffDriveController``. These controllers can be used to accurately track trajectories and account for external disturbances.

Linear Time-Varying Unicycle Controller
---------------------------------------

Similar to Ramsete, the LTV (Linear Time-Varying) Unicycle Controller generates ``ChassisSpeed`` (C++/Java) references to steer the robot along a trajectory. The system used by this controller has states :math:`\begin{bmatrix}x & y & \theta \end{bmatrix}^T` in the global frame, and linear and angular velocity inputs :math:`\begin{bmatrix}v & \omega \end{bmatrix}^T`. However, this controller utilizes tolerances grounded in reality to pick gains rather than the magical Beta and Zeta gains used by Ramsete.

The LTV Unicycle controller generates feedback gains using a Linear Quadratic Regulator to balance control effort and state excursion. Because the system has states :math:`\begin{bmatrix}x & y & \theta \end{bmatrix}^T` and inputs :math:`\begin{bmatrix}v & \omega \end{bmatrix}^T`, the LQR used to compute the controller's feedback gain takes maximum desired error tolerances for :math:`x`, :math:`y` and :math:`\theta` (in meters and radians) and maximum desired control effort in meters per second and radians per second. Note that because the controller deals with cross-track error, a tight heading tolerance will penalize steering the robot back toward the trajectory. Furthermore, the control effort tolerances are the maximum effort that the controller will apply on top of feedforward to steer the robot along the trajectory, not the robot's maximum velocity.

.. note:: Unit conversion can be easily accomplished with C++'s unit library or Java's Units class.

The following example shows how a LTV Unicycle controller could be used to compute wheel speeds while tracking a trajectory. After computing these speeds, a PID loop combined with the ``SimpleMotorFeedforward`` class can be used to generate left and right voltage commands, similar to Ramsete. A more complete example is available in WPILib's examples.

.. tabs::

   .. code-tab:: java

      LTVUnicycleController controller = new LTVUnicycleController(
          VecBuilder.fill(0.1, 0.1, 10), // X, Y and heading tolerances
          VecBuilder.fill(2.5, 1.5), 0.020); // linear and angular velocity input tolerances

      <...>

      ChassisSpeeds adjustedSpeeds = controller.calculate(currentRobotPose, goal);
      DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);
      double left = wheelSpeeds.leftMetersPerSecond;
      double right = wheelSpeeds.rightMetersPerSecond;

   .. code-tab:: cpp

      frc::LTVUnicycleController controller{
          {0.1, 0.1, 10}, // X, Y and heading tolerances
          {2.5, 2.5}, 20_ms}; // linear and angular velocity input tolerances

      <...>

      ChassisSpeeds adjustedSpeeds = controller.Calculate(currentRobotPose, goal);
      DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.ToWheelSpeeds(adjustedSpeeds);
      units::meter_per_second_t left = wheelSpeeds.left;
      units::meter_per_second_t right = wheelSpeeds.right;

Ramsete in the Command-Based Framework
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For the sake of ease for users, a ``LTVUnicycleCommand`` class is built in to WPILib. This command is a drop-in replacement for the existing ``RamseteCommand``