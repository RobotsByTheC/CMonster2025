package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CoralConstants.grabCurrentLimit;
import static frc.robot.Constants.CoralConstants.motorCanID;
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
  private final SparkMax grabMotor;
  private final SparkMax wrist;

  @SuppressWarnings("FieldCanBeLocal")
  @NotLogged
  private final SparkMaxConfig grabMotorConfig; //NOPMD

  @SuppressWarnings("FieldCanBeLocal")
  @NotLogged
  private final SparkMaxConfig wristConfig; //NOPMD

  private final AbsoluteEncoder wristEncoder;

  public RealCoralIO() {
    grabMotor = new SparkMax(motorCanID, SparkLowLevel.MotorType.kBrushless);

    wrist = new SparkMax(wristCanID, SparkLowLevel.MotorType.kBrushless);
    wristEncoder = wrist.getAbsoluteEncoder();

    grabMotorConfig = new SparkMaxConfig();
    grabMotorConfig.secondaryCurrentLimit(grabCurrentLimit.in(Amps));
    grabMotorConfig.inverted(true);

    wristConfig = new SparkMaxConfig();
    wristConfig.secondaryCurrentLimit(wristCurrentLimit.in(Amps));
    wristConfig.absoluteEncoder.inverted(false);
    wristConfig.inverted(true);

    grabMotor.configure(
        grabMotorConfig,
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
    grabMotor.setVoltage(voltage);
  }

  @Override
  public Current getWristCurrentDraw() {
    return Amps.of(wrist.getOutputCurrent());
  }

  @Override
  public Current getGrabCurrentDraw() {
    return Amps.of(grabMotor.getOutputCurrent());
  }

  @Override
  public Voltage getWristAppliedVoltage() {
    return Volts.of(wrist.getAppliedOutput() * wrist.getBusVoltage());
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
