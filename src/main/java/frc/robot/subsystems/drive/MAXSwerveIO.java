package frc.robot.subsystems.drive;

import static frc.robot.Constants.DriveConstants.frontLeftChassisAngularOffset;
import static frc.robot.Constants.DriveConstants.frontLeftDrivingCanId;
import static frc.robot.Constants.DriveConstants.frontLeftTurningCanId;
import static frc.robot.Constants.DriveConstants.frontRightChassisAngularOffset;
import static frc.robot.Constants.DriveConstants.frontRightDrivingCanId;
import static frc.robot.Constants.DriveConstants.frontRightTurningCanId;
import static frc.robot.Constants.DriveConstants.rearLeftChassisAngularOffset;
import static frc.robot.Constants.DriveConstants.rearLeftDrivingCanId;
import static frc.robot.Constants.DriveConstants.rearLeftTurningCanId;
import static frc.robot.Constants.DriveConstants.rearRightChassisAngularOffset;
import static frc.robot.Constants.DriveConstants.rearRightDrivingCanId;
import static frc.robot.Constants.DriveConstants.rearRightTurningCanId;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.swerve.MAXSwerveModuleIO;
import frc.robot.subsystems.drive.swerve.SwerveModule;

/**
 * IO for a swerve drive using REV MAXSwerve modules driven by a NEO motor and turned by a NEO 550
 * motor.
 */
public class MAXSwerveIO implements SwerveIO {
  private final SwerveModule frontLeft =
      new SwerveModule(
          new MAXSwerveModuleIO(
              new SparkMax(frontLeftDrivingCanId, MotorType.kBrushless),
              new SparkMax(frontLeftTurningCanId, MotorType.kBrushless)),
          frontLeftChassisAngularOffset);
  private final SwerveModule frontRight =
      new SwerveModule(
          new MAXSwerveModuleIO(
              new SparkMax(frontRightDrivingCanId, MotorType.kBrushless),
              new SparkMax(frontRightTurningCanId, MotorType.kBrushless)),
          frontRightChassisAngularOffset);
  private final SwerveModule rearLeft =
      new SwerveModule(
          new MAXSwerveModuleIO(
              new SparkMax(rearLeftDrivingCanId, MotorType.kBrushless),
              new SparkMax(rearLeftTurningCanId, MotorType.kBrushless)),
          rearLeftChassisAngularOffset);
  private final SwerveModule rearRight =
      new SwerveModule(
          new MAXSwerveModuleIO(
              new SparkMax(rearRightDrivingCanId, MotorType.kBrushless),
              new SparkMax(rearRightTurningCanId, MotorType.kBrushless)),
          rearRightChassisAngularOffset);

  // The gyro sensor
  // TODO: Need to wait for the 2025 navx vendor library release
  // AHRS gyro = new AHRS(SPI.Port.kMXP);

  @Override
  public SwerveModule frontLeft() {
    return frontLeft;
  }

  @Override
  public SwerveModule frontRight() {
    return frontRight;
  }

  @Override
  public SwerveModule rearLeft() {
    return rearLeft;
  }

  @Override
  public SwerveModule rearRight() {
    return rearRight;
  }

  @Override
  public Rotation2d getHeading() {
    // return gyro.getRotation2d();
    return Rotation2d.kZero; // TODO: Replace with gyro reading
  }

  @Override
  public void resetHeading(Rotation2d heading) {
    // gyro.setAngleAdjustment(heading.getDegrees());
  }

  @Override
  public void zeroHeading() {
    // gyro.reset();
  }
}
