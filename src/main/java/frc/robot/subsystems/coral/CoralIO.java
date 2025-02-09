package frc.robot.subsystems.coral;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface CoralIO {
  /**
   * Up is positive voltage, down is negative voltage.
   *
   * @param voltage The voltage to set the wrist to.
   */
  void setWristVoltage(Voltage voltage);

  /**
   * Intake is positive voltage, outtake is negative voltage.
   *
   * @param voltage The voltage to set the grabber to.
   */
  void setGrabVoltage(Voltage voltage);

  Current getWristCurrentDraw();

  Current getGrabCurrentDraw();

  Voltage getWristAppliedVoltage();

  boolean hasLeftCoral();

  boolean hasRightCoral();

  default boolean hasAnyCoral() {
    return hasLeftCoral() || hasRightCoral();
  }

  Angle getWristAngle();

  AngularVelocity getWristVelocity();
}
