package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.CoralConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class RealCoralIO implements CoralIO {
  private final SparkMax grabLeft;
  private final SparkMax grabRight;
  private final SparkMax wrist;
  private final SparkMaxConfig grabConfig;
  private final SparkMaxConfig wristConfig;
  private final SparkLimitSwitch leftLimitSwitch;
  private final SparkLimitSwitch rightLimitSwitch;
  private final AbsoluteEncoder wristEncoder;

  public RealCoralIO() {
    grabLeft = new SparkMax(leftCanID, SparkLowLevel.MotorType.kBrushless);
    grabRight = new SparkMax(rightCanID, SparkLowLevel.MotorType.kBrushless);
    leftLimitSwitch = grabLeft.getForwardLimitSwitch();
    rightLimitSwitch = grabRight.getForwardLimitSwitch();

    wrist = new SparkMax(wristCanID, SparkLowLevel.MotorType.kBrushless);
    wristEncoder = wrist.getAbsoluteEncoder();

    grabConfig = new SparkMaxConfig();
    grabConfig.secondaryCurrentLimit(grabCurrentLimit.in(Amps));
    wristConfig = new SparkMaxConfig();
    wristConfig.secondaryCurrentLimit(wristCurrentLimit.in(Amps));
    wristConfig.absoluteEncoder.inverted(true);

    grabLeft.configure(
        grabConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    grabRight.configure(
        grabConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    wrist.configure(
        wristConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void setWristVoltage(Voltage voltage) {
    wrist.setVoltage(voltage);
  }

  @Override
  public void setGrabVoltage(Voltage voltage) {
    grabRight.setVoltage(voltage);
    grabLeft.setVoltage(voltage);
  }

  @Override
  public Current getWristCurrentDraw() {
    return Amps.of(wrist.getOutputCurrent());
  }

  @Override
  public Current getGrabCurrentDraw() {
    return Amps.of(grabLeft.getOutputCurrent() + grabRight.getOutputCurrent());
  }

  @Override
  public Voltage getWristAppliedVoltage() {
    return Volts.of(wrist.getAppliedOutput() * wrist.getBusVoltage());
  }

  @Override
  public boolean hasLeftCoral() {
    return leftLimitSwitch.isPressed();
  }

  @Override
  public boolean hasRightCoral() {
    return rightLimitSwitch.isPressed();
  }

  @Override
  public Angle getWristAngle() {
    return Rotations.of(wristEncoder.getPosition());
  }

  @Override
  public AngularVelocity getWristVelocity() {
    return RPM.of(wristEncoder.getVelocity());
  }
}
