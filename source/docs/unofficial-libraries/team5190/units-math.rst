FalconLibrary Units and Math
==============================

Summery
---------

FalconLibrary includes all common SI units and derived units as typesafe
objects. This includes base units such as Length, Time and Voltage, as
well as derived units such as Velocity and Acceleration. These units
in turn can be used to construct mathematical representations such
as Rotation2d (representing a 2d rotation, such as an angle, as
essensially a unit vector), Translation2d (representing a 2d translation
as a 2 dimensional typesafe vector), Pose2d (representing a 
Translation2d and a Rotation2d) and Pose2dWithCurvature. (used
to represent states of a robot's path on an arc)

Examples with comments:

.. tabs::


   .. code-tab:: java
        var aVelocity = VelocityKt.getVelocity(LengthKt.getFeet(5.0))
        // creating a Length
        far aLength = LengthKt.getFeet(10);

        // getting a Length
        var aLengthInInches = aLength.getInch();

        var aVelocity = VelocityKt.getVelocity(LengthKt.getFeet(5.0));

        var anAccel = AccelerationKt.getAcceleration(LengthKt.getFeet(7));

        // a vector with 
        var aTranslation = new Pose2d(LengthKt.getFeet(4), LengthKt.getFeet(10));

        var anAngle = Rotation2dKt.getDegree(-45);

        var anAngleInRadians = anAngle.getRadian();

        var anotherAngle = anAngle.absoluteValue();

        // used to represent a 2d location + angle
        var aPose2d = new Pose2d(
                aTranslation, anAngle
        );

   .. code-tab:: kotlin
   
        // coming soon, coz i don't know Kotlin at all



