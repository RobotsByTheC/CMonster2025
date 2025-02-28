package frc.robot.subsystems.algae;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface AlgaeIO {
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

  Angle getWristAngle();

  AngularVelocity getWristVelocity();
}
