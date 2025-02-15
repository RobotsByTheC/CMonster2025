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
  private final SparkMax grabLeft;
  private final SparkMax grabRight;
  private final SparkMax wrist;
  @NotLogged private final SparkMaxConfig grabLeftConfig;
  @NotLogged private final SparkMaxConfig grabRightConfig;
  @NotLogged private final SparkMaxConfig wristConfig;
  private final AbsoluteEncoder wristEncoder;

  public RealCoralIO() {
    grabLeft = new SparkMax(leftCanID, SparkLowLevel.MotorType.kBrushless);
    grabRight = new SparkMax(rightCanID, SparkLowLevel.MotorType.kBrushless);

    wrist = new SparkMax(wristCanID, SparkLowLevel.MotorType.kBrushless);
    wristEncoder = wrist.getAbsoluteEncoder();

    grabLeftConfig = new SparkMaxConfig();
    grabLeftConfig.secondaryCurrentLimit(grabCurrentLimit.in(Amps));
    grabLeftConfig.inverted(true);
    grabRightConfig = new SparkMaxConfig();
    grabRightConfig.secondaryCurrentLimit(grabCurrentLimit.in(Amps));
    wristConfig = new SparkMaxConfig();
    wristConfig.secondaryCurrentLimit(wristCurrentLimit.in(Amps));
    wristConfig.absoluteEncoder.inverted(true);

    grabLeft.configure(
        grabLeftConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    grabRight.configure(
        grabRightConfig,
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
    return true;
  }

  @Override
  public boolean hasRightCoral() {
    return true;
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
