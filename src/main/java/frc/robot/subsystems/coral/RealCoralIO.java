package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CoralConstants.grabCurrentLimit;
import static frc.robot.Constants.CoralConstants.leftCanID;
import static frc.robot.Constants.CoralConstants.rightCanID;
import static frc.robot.Constants.CoralConstants.wristCanID;
import static frc.robot.Constants.CoralConstants.wristCurrentLimit;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class RealCoralIO implements CoralIO {
  @NotLogged private final SparkMax grabLeft;
  @NotLogged private final SparkMax grabRight;
  @NotLogged private final SparkMax wrist;
  @NotLogged private final SparkMaxConfig grabConfig;
  @NotLogged private final SparkMaxConfig wristConfig;
  @NotLogged private final SparkLimitSwitch leftLimitSwitch;
  @NotLogged private final SparkLimitSwitch rightLimitSwitch;
  @NotLogged private final AbsoluteEncoder wristEncoder;

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
    Angle r = Rotations.of(wristEncoder.getPosition());
    if (r.gt(Degrees.of(180))) {
      return r.minus(Degrees.of(360));
    } else {
      return r;
    }
  }

  @Override
  public AngularVelocity getWristVelocity() {
    return RPM.of(wristEncoder.getVelocity());
  }
}
