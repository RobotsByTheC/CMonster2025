// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final LinearVelocity maxSpeed = MetersPerSecond.of(4.5);
    public static final AngularVelocity maxAngularSpeed = RotationsPerSecond.of(2.0);

    // Chassis configuration
    public static final Distance trackWidth = Inches.of(24.5);
    // Distance between centers of right and left wheels on robot
    public static final Distance wheelBase = Inches.of(24.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics driveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase.div(2), trackWidth.div(2)),
            new Translation2d(wheelBase.div(2), trackWidth.div(-2)),
            new Translation2d(wheelBase.div(-2), trackWidth.div(2)),
            new Translation2d(wheelBase.div(-2), trackWidth.div(-2)));

    // Angular offsets here describe how the swerve modules are physically rotated with respect to
    // to the chassis. There should be offsets at 0, 90, 180, and 270 degrees for a rectangular
    // chassis.
    public static final Rotation2d frontLeftChassisAngularOffset = Rotation2d.fromDegrees(0);
    public static final Rotation2d frontRightChassisAngularOffset = Rotation2d.fromDegrees(0);
    public static final Rotation2d rearLeftChassisAngularOffset = Rotation2d.fromDegrees(0);
    public static final Rotation2d rearRightChassisAngularOffset = Rotation2d.fromDegrees(0);

    // SPARK MAX CAN IDs
    public static final int frontLeftDrivingCanId = 2;
    public static final int rearLeftDrivingCanId = 4;
    public static final int frontRightDrivingCanId = 8;
    public static final int rearRightDrivingCanId = 6;

    public static final int frontLeftTurningCanId = 1;
    public static final int rearLeftTurningCanId = 3;
    public static final int frontRightTurningCanId = 7;
    public static final int rearRightTurningCanId = 5;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int drivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean turningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final Distance wheelDiameter = Inches.of(3);
    public static final Distance wheelCircumference = wheelDiameter.times(Math.PI);
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
    // bevel pinion
    public static final double drivingMotorReduction = (45.0 * 22) / (drivingMotorPinionTeeth * 15);
    // Gear ratio of the turning (aka "azimuth") motor. ~46.42:1
    public static final double turningMotorReduction = 9424.0 / 203;
    public static final LinearVelocity driveWheelFreeSpeed =
        wheelCircumference
            .times(NeoMotorConstants.freeSpeedRpm.in(RotationsPerSecond))
            .div(drivingMotorReduction)
            .per(Second);

    public static final Distance drivingEncoderPositionFactor =
        wheelDiameter.times(Math.PI / drivingMotorReduction);
    public static final LinearVelocity drivingEncoderVelocityFactor =
        wheelDiameter.times(Math.PI / drivingMotorReduction).per(Minute);

    public static final Angle turningEncoderPositionFactor = Rotations.of(1.0);
    public static final AngularVelocity turningEncoderVelocityFactor = RotationsPerSecond.of(1.0);

    public static final Current drivingCurrentLimit = Amps.of(50);
    public static final Current turningCurrentLimit = Amps.of(20);

    public static final double drivingP = 0.04;
    public static final double drivingI = 0;
    public static final double drivingD = 0;
    public static final double drivingFF = 1 / driveWheelFreeSpeed.in(MetersPerSecond);

    public static final double turningP = 1;
    public static final double turningI = 0;
    public static final double turningD = 0;
    public static final double turningFF = 0;

    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Note: All unit-based configuration values should use SI units (meters, radians, seconds,
      // etc) for consistency

      drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit((int) drivingCurrentLimit.in(Amps));
      drivingConfig
          .encoder
          .positionConversionFactor(drivingEncoderPositionFactor.in(Meters))
          .velocityConversionFactor(drivingEncoderVelocityFactor.in(MetersPerSecond));
      drivingConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(drivingP, drivingI, drivingD)
          .velocityFF(drivingFF)
          .outputRange(-1, 1);

      turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit((int) turningCurrentLimit.in(Amps));
      turningConfig
          .absoluteEncoder
          .inverted(turningEncoderInverted)
          .positionConversionFactor(turningEncoderPositionFactor.in(Radians))
          .velocityConversionFactor(turningEncoderVelocityFactor.in(RadiansPerSecond));
      turningConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pid(turningP, turningI, turningD)
          .velocityFF(turningFF)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, Rotations.one().in(Radians));
    }
  }

  public static final class ElevatorConstants {
    // Top face of the carriage rail to the carpet
    public static final Distance zeroOffset = Inches.of(7.5);
    public static final Distance l1 = Inches.of(22).minus(zeroOffset);
    public static final Distance l2 = Inches.of(36).minus(zeroOffset);
    public static final Distance l3 = Inches.of(52).minus(zeroOffset);
    public static final Distance l4 = Inches.of(76).minus(zeroOffset);

    public static final Current stallThreshold = Amps.of(70);
    public static final Time stallDuration = Milliseconds.of(500);

    public static final Distance minHeight = Inches.of(0);
    // Slightly less than max elevator extension
    public static final Distance maxHeight = Inches.of(67.5);

    public static final int leftCanID = 10;
    public static final int rightCanID = 9;

    public static final double gearboxReduction = 4.86; // 4.86:1

    // Raw: 21.3689 rotations from bottom to top
    // Actual extension = 67.5"
    // 67.5 / 21.3689 = 3.1588
    public static final double positionConversionFactor = 3.344;
    public static final double velocityConversionFactor = positionConversionFactor / 60;

    public static final int currentLimit = 40;

    public static final double KS = 0.92435;
    public static final double KG = 1.2623;
    public static final double KV = 1.1023;
    public static final double KA = 0.28148;
    public static final double KP = 20;
    public static final double KI = 0;
    public static final double KD = 1;
  }

  public static final class CoralConstants {
    public static final int leftCanID = 14;
    public static final int rightCanID = 12;
    public static final int wristCanID = 13;

    public static final Current grabCurrentLimit = Amps.of(15);
    public static final Current wristCurrentLimit = Amps.of(40);

    public static final Voltage grabIntakeVoltage = Volts.of(6);
    public static final Voltage grabScoreVoltage = Volts.of(6);

    public static final Angle intakeAngle = Degrees.of(30);
    public static final Angle troughScoreAngle = Degrees.of(-15);
    public static final Angle branchScoreAngle = Degrees.of(-35);
    public static final Angle tipScoreAngle = Degrees.of(-60);
    public static final Angle stowAngle = Degrees.of(48);

    public static final Angle minWristAngle = Degrees.of(-75);
    public static final Angle maxWristAngle = Degrees.of(50);
    public static final Angle wristTolerance = Degrees.of(2);

    public static final double KS = 0.34646;
    public static final double KG = 0.13; // 0.1159
    public static final double KV = 52.183;
    public static final double KA = 4.3241;
    public static final double KP = 12 / 1.6;
    public static final double KI = 0;
    public static final double KD = 0;
  }

  public static final class AlgaeConstants {
    public static final int leftCanID = 0;
    public static final int rightCanID = 0;
    public static final int wristCanID = 0;

    public static final Current grabCurrentLimit = Amps.of(15);
    public static final Current wristCurrentLimit = Amps.of(40);

    public static final Voltage grabIntakeVoltage = Volts.of(6);
    public static final Voltage grabScoreVoltage = Volts.of(6);

    public static final Angle groundIntakeAngle = Degrees.of(30);
    public static final Angle reefIntakeAngle = Degrees.of(-35);
    public static final Angle processorScoreAngle = Degrees.of(0);
    public static final Angle stowAngle = Degrees.of(125);

    public static final Angle minWristAngle = Degrees.of(-35);
    public static final Angle maxWristAngle = Degrees.of(130);
    public static final Angle wristTolerance = Degrees.of(2);

    public static final double KS = 0;
    public static final double KG = 0;
    public static final double KV = 0;
    public static final double KA = 0;
    public static final double KP = 0;
    public static final double KI = 0;
    public static final double KD = 0;
  }

  public static final class OIConstants {
    public static final int driverControllerPort = 1;
    public static final int leftJoystickPort = 2;
    public static final int rightJoystickPort = 3;
  }

  public static final class AutoConstants {
    public static final double pXController = 4;
    public static final double pYController = 4;
    public static final double pThetaController = 2;
  }

  public static final class NeoMotorConstants {
    public static final AngularVelocity freeSpeedRpm = RPM.of(5676);
  }
}
