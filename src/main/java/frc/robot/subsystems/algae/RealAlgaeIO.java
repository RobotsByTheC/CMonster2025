package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.AlgaeConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.*;

@Logged
public class RealAlgaeIO implements AlgaeIO {
  private final SparkMax grabLeft;
  private final SparkMax grabRight;
  private final SparkMax wrist;
  private final SparkMaxConfig grabConfig;
  private final SparkMaxConfig wristConfig;
  private final AbsoluteEncoder wristEncoder;

  private Time grabberActionStart = Milliseconds.of(System.currentTimeMillis());
  private boolean hasAlgae = false;
  private Voltage grabberVoltage = Volts.zero();

  public RealAlgaeIO() {
    grabLeft = new SparkMax(leftCanID, SparkLowLevel.MotorType.kBrushless);
    grabRight = new SparkMax(rightCanID, SparkLowLevel.MotorType.kBrushless);

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
    if (!grabberVoltage.equals(voltage)) {
      grabberActionStart = Milliseconds.of(System.currentTimeMillis());
    }
    if (System.currentTimeMillis() - grabberActionStart.in(Milliseconds) > 500) {
      if (grabberVoltage.magnitude() > 0) {
        hasAlgae = true;
      } else if (grabberVoltage.magnitude() < 0) {
        hasAlgae = false;
      }
    }
    grabberVoltage = voltage;

    grabLeft.setVoltage(voltage);
    grabRight.setVoltage(voltage);
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
    return Volts.of(wrist.getAppliedOutput());
  }

  @Override
  public boolean hasAlgae() {
    return hasAlgae;
  }

  @Override
  public Angle getWristAngle() {
    return Degrees.of(wristEncoder.getPosition());
  }

  @Override
  public AngularVelocity getWristVelocity() {
    return RadiansPerSecond.of(wristEncoder.getVelocity());
  }
}
