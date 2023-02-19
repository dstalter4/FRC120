**Acknowledgement: This guide and corresponding code is a C++ port of Team 364's Java implementation.  Thanks to them for making this possible.**

# C++ Swerve Drive

Basic swerve drive for a robot with Swerve Drive Specialties modules using CTRE TalonFX motors (e.g. MK3, MK4 and MKri), CTRE CANCoders and a CTRE Pigeon Gyro.

**Setup and Configuration**
----
A number of configuration parameters must be adjusted to a specific robot build.  Distances must be in meters and rotation units in radians.

1. In ```SwerveConfig.hpp``` the following constants are swerve module specific and should be updated:
    * ```DRIVE_GEAR_RATIO``` - The ratio from the TalonFX output to the wheel drive gear (from SDS or equivalent module information)
    * ```ANGLE_GEAR_RATIO``` - The ratio from the TalonFX output to the wheel angle gear (from SDS or equivalent module information)
    * ```WHEEL_CIRCUMFERENCE``` - The wheel circumference
    * ```WHEEL_BASE``` - The distance between the center points of the front/back wheels
    * ```TRACK_WIDTH``` - The distance between the center points of the left/right wheels
    * ```MAX_DRIVE_VELOCITY_MPS``` - Maximum linear travel speed of the robot in meters per second
    * ```MAX_ANGULAR_VELOCITY_RAD_PER_SEC``` - Maximum rotational speed of the robot in radians per second

<br>For the max drive/angular velocities, the theoretical values can be used, but it is better to physically drive the robot and find the actual max values.

2. Set ```PIGEON_CAN_ID``` in ```SwerveDrive.hpp``` to the CAN ID of the Pigeon IMU.  There is currently no configuration setting to invert the gyro as the default should work.  The invert setting controls the gyro rotation direction (i.e. CCW+ Counter Clockwise Positive).
<br><b>Note: The Pigeon is created on the CANivore network</b>

3. Update the four ```SwerveModuleConfig``` constant expressions in ```SwerveDrive.hpp```.  This is a custom data structure used to create the individual ```SwerveModule``` objects.  It takes a name (as a string), a ```ModulePosition``` enum value for which swerve module it is, the TalonFX CAN IDs (on the RIO CAN network), the CANCoder CAN ID (on the CANivore network), and the CANCoder angle offset (measured later in step 5).

4. The ```SwerveModule``` constructor also sets a number of configuration options, mostly for the TalonFX controllers.  The constructor body is in ```SwerveModule.cpp``` and configures the following parameters, which may need to be tuned to robot specific values:
    * Supply current limiting configuration (drive and angle motors)
    * PID values (drive and angle motors)
    * Open and closed loop ramp values
    * Drive and angle motor invert and neutral mode
    * CANCoder invert, sensor range and initialization strategy

<br>The drive motor invert should always be able to remain false, since the calibration step below sets a positive input to the drive motor to cause the robot to move forward.  However, this can be set to true if it is preferred to have the bevel gears face a different direction when setting the offsets in step 5.

<br>The angle motor invert and CANCoder invert must both be set such that they are CCW+.

5. Find and set the module offsets.
    * Point the bevel gears of all the wheels in the same direction (either facing left or right), where a postive input to the drive motor drives the robot forward (Phoenix Tuner can be used to test this).  If for some reason the offsets are set with the wheels backwards, the drive motor TalonFX config can be changed to correct the behavior.
    * Use a piece of metal or other technique to ensure that the wheel of each module is aligned straight (as close to pointing straight ahead for the robot frame).
    * Using Phoenix Tuner, pull up the current information on the CANCoder for each swerve module.  Copy those numbers exactly (including decimal places) to the ```SwerveModuleConfig``` structures in ```SwerveDrive.hpp```.
    <br><b>Note: The CANCoder values are in degrees.  When constructing Rotation2d objects, be sure to initialize from the proper units.</b>

6. Update the drive characterization values.  These are the ```KS``` ```KV``` ```KA```  constants in ```SwerveModule.hpp```.  They can be measured using the WPILib drive characterization tool found [here](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html). The swerve modules will need to be locked straight forward and the characterization completed as if the robot was using standard tank drive.

7. Tune the drive motor PID values.  After completeing characterization and inserting the KS, KV, and KA values into the code, tune the drive motor kP until it doesn't overshoot and doesn't oscilate around a target velocity.  The default may be good enough.  Leave the drive motor kI, kD, and kF at zero.

8. Tune the angle motor PID Values.  The defaults should be good enough, but they can be changed to provide a different robot driving response.  These steps can be followed to tune the values:
    * Start with a low P value (0.01).
    * Multiply by 10 until the module starts oscillating around the set point (which is a velocity for the drive motor or angle/position for the angle motor)
    * Scale back by searching for the value (e.g., if it starts oscillating at a P of 10, then try (10 -> 5 -> 7.5 -> etc.)) until the module doesn't oscillate around the. setpoint.
    * If there is any overshoot, add in some D by repeating the same process, leave at 0 if not. Always leave I at 0.



**Controller Mappings**
----
This code is natively setup to use a XBox controller to control the swerve drive.  The codebase as a whole can support easily switching between controller types by updating some controller configuration constants, but that is not convered here.

* Left Stick: Translation control (forwards and sideways movement)
* Right Stick: Rotation control
* Left Bumper: Toggle between field relative and robot centric
* Right Bumper: Resets the gyro yaw to zero (corrects drift or sets a new 'forward' direction)



**Autonomous Example**
----
There is also an example autonomous routine in ```YtaRobotAutonomousTest.cpp```.  The code is in the function ```AutonomousTestSwerveRoutine()```.  It will make the robot draw the outline of a square shape, spin both clockwise/counter clockwise in place, and finally move forward while rotating.  Due to the robot template being time based instead of command based, advanced techniques like trajectory configs are not supported yet.
