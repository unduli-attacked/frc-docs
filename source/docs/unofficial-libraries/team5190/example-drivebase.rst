Example path following drivebases
=====================================

Some example drivebases to get you started on path following quickly

.. tabs::


   .. code-tab:: java

        package frc.robot.commands.subsystems.drivetrain;

        import org.ghrobotics.lib.localization.Localization;
        import org.ghrobotics.lib.localization.TankEncoderLocalization;
        import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker;
        import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
        import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
        import org.ghrobotics.lib.mathematics.units.Length;
        import org.ghrobotics.lib.mathematics.units.LengthKt;
        import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
        import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
        import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
        import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;
        import org.ghrobotics.lib.subsystems.drive.DifferentialTrackerDriveBase;
        import org.ghrobotics.lib.wrappers.ctre.FalconSRX;
        import org.team5940.pantry.experimental.command.SendableSubsystemBase;
        import org.team5940.pantry.lib.Util;

        import com.kauailabs.navx.frc.AHRS;
        import com.team254.lib.physics.DCMotorTransmission;
        import com.team254.lib.physics.DifferentialDrive;

        import edu.wpi.first.wpilibj.I2C.Port;
        import edu.wpi.first.wpilibj.Notifier;

        public class DriveTrain extends SendableSubsystemBase /* or Subsystem */ implements DifferentialTrackerDriveBase {

            public static final double kRobotMass = 50 /* Robot, kg */ + 5f /* Battery, kg */ + 2f /* Bumpers, kg */;
            public static final double kRobotMomentOfInertia = 10.0; // kg m^2 // TODO Tune
            public static final double kRobotAngularDrag = 12.0; // N*m / (rad/sec)

            public static final double kWheelRadius = Util.toMeters(2f / 12f);// meters. TODO tune
            public static final double kTrackWidth = Util.toMeters(26f / 12f);// meters

            private static final double kVDriveLeftLow = 0.274 * 1d; // Volts per radians per second - Calculated emperically
            private static final double kADriveLeftLow = 0.032 * 1d; // Volts per radians per second per second TODO tune
            private static final double kVInterceptLeftLow = 1.05 * 1d; // Volts - tuned!

            private static final double kVDriveRightLow = 0.265 * 1d; // Volts per radians per second - Calculated emperically
            private static final double kADriveRightLow = 0.031 * 1d; // Volts per radians per second per second TODO tune
            private static final double kVInterceptRightLow = 1.02 * 1d; // Volts - tuned!

            public static final DCMotorTransmission kLeftTransmissionModelLowGear = new DCMotorTransmission(1 / kVDriveLeftLow,
                    kWheelRadius * kWheelRadius * kRobotMass / (2.0 * kADriveLeftLow),
                    kVInterceptLeftLow);

            public static final DCMotorTransmission kRightTransmissionModelLowGear = new DCMotorTransmission(1 / kVDriveRightLow,
                    kWheelRadius * kWheelRadius * kRobotMass / (2.0 * kADriveRightLow),
                    kVInterceptRightLow);

            private FalconSRX<Length> left, right;

            private Localization localization;

            private RamseteTracker ramseteTracker;

            private AHRS gyro; // a NavX

            private Notifier localizationNotifier;

            /* Ramsete constants */
            public static final double kDriveBeta = 2 * 1d; // Inverse meters squared
            public static final double kDriveZeta = 0.7 * 1d; // Unitless dampening co-efficient

            public mlem() {
                var nativeUnitModel = new NativeUnitLengthModel(NativeUnitKt.getNativeUnits(4096), LengthKt.getInch(2));
                left = new FalconSRX<Length>(0, nativeUnitModel, TimeUnitsKt.getMillisecond(10));
                right = new FalconSRX<Length>(0, nativeUnitModel, TimeUnitsKt.getMillisecond(10));

                gyro = new AHRS(Port.kMXP);

                /* Create a localization object because lamda expressions are fun */
                localization = new TankEncoderLocalization(() -> Rotation2dKt.getDegree(getGyro(true)),
                        () -> getLeftMotor().getSensorPosition(), () -> getRightMotor().getSensorPosition());
                /* set the robot pose to 0,0,0 */
                localization.reset(new Pose2d());

                ramseteTracker = new RamseteTracker(kDriveBeta, kDriveZeta);

                localizationNotifier = new Notifier(() -> {
                    this.getLocalization().update();
                });
                localizationNotifier.startPeriodic(1d / 100d);

            }

            private Localization getLocalization() {
                return localization;
            }

            private double getGyro(boolean isReversed) {
                return gyro.getAngle() * ((isReversed) ? -1 : 1);
            }

            @Override
            public FalconSRX<Length> getLeftMotor() {
                return left;
            }

            @Override
            public FalconSRX<Length> getRightMotor() {
                return right;
            }

            @Override
            public Pose2d getRobotPosition() {
                return null;
            }

            @Override
            public TrajectoryTracker getTrajectoryTracker() {
                return null;
            }

            @Override
            public DifferentialDrive getDifferentialDrive() {
                return null;
            }

        }



   .. code-tab:: kotlin
   
        // coming soon, coz i don't know Kotlin at all