package frc.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface ElevatorIO {

  void setVoltage(Voltage voltage);

  Current getCurrentDraw();

  Voltage getAppliedVoltage();

  Distance getHeight();

  LinearVelocity getVelocity();

  void resetEncoders();
}
