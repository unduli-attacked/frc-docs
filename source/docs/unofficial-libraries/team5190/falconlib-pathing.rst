Pathing with FalconLibrary
=============================

Summery
---------

.. image:: images/dash1.png
   :width: 900

To get from point A to B in the fastest way possible, FalconLibrary generates
a spline between the two points. (iirc) Team 254 teh Chezy Pofs were the first
to use splines, back in 2014. FalconLibrary generates splines using user
provided
waypoints, initial and ending velocities, maximum speeds and accelerations, and
additional constraints including velocity limiting regions and
drivetrain models (see the article on these). These trajectories are then
uploaded to the robot, which follows them using a selection of avalible
followers. To do this, though, the robot needs to know where it is on the
field and update it in real-time using a techique called Odometry, meaning
that your robot needs encoders
and a form of gyroscope such as NavX or Pigeon IMU.

Odometry
-----------

Odometry is the process of deriving robot position using
`Dead Reckoning <https://en.wikipedia.org/wiki/Dead_reckoning>`_.
Using information about driven distance and heading, a robot can
be "localized" on the field. The process of relocalization is deriving
an absolute robot position by way of known robot pose or a vision target.
FalconLibrary implements for users a build in Tank Drive Odometry
class - all that users need to do is in give it Suppliers for drivetrain
distances and robot heading. Review this for more information on
`Functional Interfaces in Java. <https://www.geeksforgeeks.org/functional-interfaces-java/>`_

.. tabs::


   .. code-tab:: java
        // Implememntation from Team 5940

        /* Create a localization object because lamda expressions are fun */
        localization = new TankEncoderLocalization(
                // the gyro needs to be positive counter-clockwise
                () -> Rotation2dKt.getDegree(getGyro(true)),
                // and these need to return a Length
                () -> getLeft().getDistance(), 
                () -> getRight().getDistance());

        /* set the robot pose to 0,0,0 */
        localization.reset(new Pose2d());

        // the update() method must be called periodically,
        // as fast as possible. 100hz is ideal, but 20 will work.
        Notifier localizationNotifier = new Notifier( 
                () -> {localization.update();}
        );
        localizationNotifier.startPeriodic(1d / 100d);


   .. code-tab:: kotlin
   
        // coming soon, coz i don't know Kotlin at all

Following paths
-----------------------------

Paths should be generated with :doc:`/docs/unnoficial-libraries/team5190/falcon-dash`
(TODO fix that link)
Paths are stored as a :code:`TimedTrajectory<Pose2dWithCurvature>`,
which can be followed using:

- Feedforward, using no pose feedback
- Pure Pursuit, which uses a lookahead point and angle to follow
        a path. This should be phased out for tank drive in favor
        of
- RAMSETE, non-linear feedback based on robot pose.

It is reccomended that teams make their drivetrains implement
:code:`DifferentialTrackerDriveBase` and convert their motors
to :code:`FalconMotor<Length>`, or it's subclasses, such as
:code:`FalconSRX<Length>`. 

The DifferentialTrackerDriveBase
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

:code:`DifferentialTrackerDriveBase` is an interface for teams to quickly
make their drivetrains integrate with FalconLibrary path following. The 
interface requires you to have charicterized and modeled your drivetrain,
to have drive motors which subclass FalconMotor, and have implemented
a form of localization. The method contains and inherits methods for
utilizing the feedforward models of your drivetrain to estimate
the voltage required for a (velocity, acceleration) command and setting
motor output to a PID setpoint + feedforward voltage. See (TODO LINK)
charicterizing your drivetrain for information on the DifferentialDrive
class. A bare-bones example of a DifferentialTrackerDriveBase can be
found (TODO LINK) HERE.

An example path following command
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabs::


   .. code-tab:: java

        package frc.robot.commands.subsystems.drivetrain;

        import java.util.function.Supplier;

        import org.ghrobotics.lib.debug.LiveDashboard;
        import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
        import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
        import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
        import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry;
        import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
        import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TrajectorySamplePoint;
        import org.ghrobotics.lib.mathematics.units.Length;
        import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
        import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput;
        import org.team5940.pantry.experimental.command.SendableCommandBase;

        import edu.wpi.first.wpilibj.Notifier;
        import edu.wpi.first.wpilibj.Timer;
        import frc.robot.Robot;
        import frc.robot.commands.auto.Trajectories;
        import frc.robot.lib.Logger;
        import frc.robot.subsystems.DriveTrain;

        public class TrajectoryTrackerCommand extends SendableCommandBase {
                private TrajectoryTracker trajectoryTracker;
                private Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource;
                private DriveTrain driveBase;
                private boolean reset;
                private TrajectoryTrackerOutput output;
                Length mDesiredLeft;
                Length mDesiredRight;
                double mCurrentLeft;
                double mCurrentRight;

                Notifier mUpdateNotifier;

                public TrajectoryTrackerCommand(DriveTrain driveBase, Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource) {
                        this(driveBase, trajectorySource, false);
                }

                public TrajectoryTrackerCommand(DriveTrain driveBase, Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource, boolean reset) {
                        this(driveBase, Robot.drivetrain.getTrajectoryTracker(), trajectorySource, reset);
                }

                public TrajectoryTrackerCommand(DriveTrain driveBase, TrajectoryTracker trajectoryTracker, Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource, boolean reset) {
                        addRequirements(driveBase);
                        this.driveBase = driveBase;
                        this.trajectoryTracker = trajectoryTracker;
                        this.trajectorySource = trajectorySource;
                        this.reset = reset;
                }

                @Override
                public void initialize() {
                        LiveDashboard.INSTANCE.setFollowingPath(false);

                        if (trajectorySource == null) {
                                Logger.log("Sadly the trajectories are not generated. the person responsible for the trajectories has been sacked.");
                                Trajectories.generateAllTrajectories();
                        }

                        trajectoryTracker.reset(this.trajectorySource.get());

                        if (reset == true) {
                                Robot.drivetrain.getLocalization().reset(trajectorySource.get().getFirstState().getState().getPose());
                        }

                        LiveDashboard.INSTANCE.setFollowingPath(true);

                        mUpdateNotifier = new Notifier(() -> {
                                output = trajectoryTracker.nextState(driveBase.getRobotPosition(), TimeUnitsKt.getSecond(Timer.getFPGATimestamp()));

                                TrajectorySamplePoint<TimedEntry<Pose2dWithCurvature>> referencePoint = trajectoryTracker.getReferencePoint();
                                if (referencePoint != null) {
                                        Pose2d referencePose = referencePoint.getState().getState().getPose();

                                        LiveDashboard.INSTANCE.setPathX(referencePose.getTranslation().getX().getFeet());
                                        LiveDashboard.INSTANCE.setPathY(referencePose.getTranslation().getY().getFeet());
                                        LiveDashboard.INSTANCE.setPathHeading(referencePose.getRotation().getRadian());

                                }

                                driveBase.setOutput(output);

                        });
                        mUpdateNotifier.startPeriodic(0.01);
                }

                @Override
                public void end(boolean interrupted) {
                        mUpdateNotifier.stop();
                        driveBase.stop();
                        LiveDashboard.INSTANCE.setFollowingPath(false);
                }

                @Override
                public boolean isFinished() {
                        return trajectoryTracker.isFinished();
                }

                public TimedTrajectory<Pose2dWithCurvature> getTrajectory() {
                        return this.trajectorySource.get();
                }

        }



   .. code-tab:: kotlin
   
        // coming soon, coz i don't know Kotlin at all


