package frc.robot.subsystems.drive.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ModuleConstants.drivingConfig;
import static frc.robot.Constants.ModuleConstants.turningConfig;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

/**
 * Swerve module IO implemented using REV SparkMaxes, a NEO driving motor, and a NEO 550 turning
 * motor.
 */
@Logged
public class MAXSwerveModuleIO implements ModuleIO {
  private final SparkMax drivingSparkMax;
  private final SparkMax turningSparkMax;

  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder turningEncoder;

  @NotLogged private final SparkClosedLoopController drivingPIDController;
  @NotLogged private final SparkClosedLoopController turningPIDController;

  public MAXSwerveModuleIO(SparkMax drivingSparkMax, SparkMax turningSparkMax) {
    this.drivingSparkMax = drivingSparkMax;
    drivingPIDController = drivingSparkMax.getClosedLoopController();
    drivingEncoder = drivingSparkMax.getEncoder();

    this.turningSparkMax = turningSparkMax;
    turningEncoder = turningSparkMax.getAbsoluteEncoder();
    turningPIDController = turningSparkMax.getClosedLoopController();

    // Send SPARK MAX configurations to the controllers.
    drivingSparkMax.configure(
        drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turningSparkMax.configure(
        turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    resetEncoders();
  }

  @Override
  public LinearVelocity getWheelVelocity() {
    return MetersPerSecond.of(drivingEncoder.getVelocity());
  }

  @Override
  public Distance getWheelDistance() {
    return Meters.of(drivingEncoder.getPosition());
  }

  @Override
  public Rotation2d getModuleRotation() {
    return Rotation2d.fromRadians(turningEncoder.getPosition());
  }

  @Override
  public void setDesiredState(SwerveModuleState state) {
    // Command driving and turning SPARKS MAX towards their respective setPoints.
    drivingPIDController.setReference(state.speedMetersPerSecond, SparkBase.ControlType.kVelocity);
    turningPIDController.setReference(state.angle.getRadians(), SparkBase.ControlType.kPosition);
  }

  @Override
  public void stop() {
    // Write 0 volts to the motor controllers
    drivingSparkMax.setVoltage(0);
    turningSparkMax.setVoltage(0);
    drivingPIDController.setReference(0, SparkBase.ControlType.kVoltage);
    turningPIDController.setReference(0, SparkBase.ControlType.kVoltage);
  }

  @Override
  public void resetEncoders() {
    drivingEncoder.setPosition(0);
  }

  @Override
  public void close() {
    drivingSparkMax.close();
    turningSparkMax.close();
  }

  @Override
  public Voltage getTurnVoltage() {
    return Volts.of(turningSparkMax.getAppliedOutput());
  }

  @Override
  public void setDrivingMotorVoltage(Voltage v) {
    drivingSparkMax.setVoltage(v.in(Volts));
    turningPIDController.setReference(0, SparkBase.ControlType.kPosition);
  }
}
