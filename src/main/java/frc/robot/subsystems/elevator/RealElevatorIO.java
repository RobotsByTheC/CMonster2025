package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class RealElevatorIO implements ElevatorIO {
  private final SparkMax left;
  private final SparkMax right;
  private final SparkMaxConfig sparkConfig;
  private final RelativeEncoder rightEncoder;
  private final RelativeEncoder leftEncoder;

  public RealElevatorIO() {
    left = new SparkMax(leftCanID, SparkLowLevel.MotorType.kBrushless);
    right = new SparkMax(rightCanID, SparkLowLevel.MotorType.kBrushless);
    rightEncoder = right.getEncoder();
    leftEncoder = left.getEncoder();

    sparkConfig = new SparkMaxConfig();
    sparkConfig.encoder.positionConversionFactor(positionConversionFactor);
    sparkConfig.encoder.velocityConversionFactor(velocityConversionFactor);
    sparkConfig.secondaryCurrentLimit(currentLimit);
    sparkConfig.inverted(true);

    left.configure(
        sparkConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    right.configure(
        sparkConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    left.setVoltage(voltage);
    right.setVoltage(voltage);
  }

  @Override
  public Current getCurrentDraw() {
    return Amps.of(left.getOutputCurrent() + right.getOutputCurrent());
  }

  @Override
  public Voltage getAppliedVoltage() {
    return Volts.of(left.getAppliedOutput() * left.getBusVoltage());
  }

  @Override
  public Distance getHeight() {
    return Inches.of((leftEncoder.getPosition() + rightEncoder.getPosition()) / 2);
  }

  @Override
  public LinearVelocity getVelocity() {
    return InchesPerSecond.of((leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2);
  }

  @Override
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
}
