package frc.robot.sim;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ModuleConstants.drivingMotorReduction;
import static frc.robot.Constants.ModuleConstants.turningMotorReduction;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulates a MAX Swerve module with a single NEO driving the wheel and a single NEO 550
 * controlling the module's heading.
 */
@Logged
public class MAXSwerveModuleSim implements MechanismSim {
  // Simulate both the wheel and module azimuth control as simple flywheels
  // Note that this does not perfectly model reality - in particular, friction is not modeled.
  // The wheel simulation has a much higher moment of inertia than the physical wheel in order to
  // account for the inertia of the rest of the robot. (This is a /very/ rough approximation)
  private final FlywheelSim wheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.025, drivingMotorReduction),
          DCMotor.getNEO(1));
  private final FlywheelSim turnSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 0.001, turningMotorReduction),
          DCMotor.getNeo550(1));

  private final MutAngularVelocity wheelVelocity = RadiansPerSecond.mutable(0);
  private final MutAngularVelocity turnVelocity = RadiansPerSecond.mutable(0);

  public MAXSwerveModuleSim() {
    SimulationContext.getDefault().addMechanism(this);
  }

  @Override
  public void update(double timestep) {
    // Update the simulations with most recently commanded voltages
    wheelSim.update(timestep);
    turnSim.update(timestep);

    // Cache the computed velocities
    wheelVelocity.mut_replace(wheelSim.getAngularVelocityRadPerSec(), RadiansPerSecond);
    turnVelocity.mut_replace(turnSim.getAngularVelocityRadPerSec(), RadiansPerSecond);
  }

  public void setDriveVoltage(Voltage volts) {
    wheelSim.setInputVoltage(outputVoltage(volts.in(Volts)));
  }

  public void setTurnVoltage(Voltage volts) {
    turnSim.setInputVoltage(outputVoltage(volts.in(Volts)));
  }

  /** Gets the angular velocity of the wheel. */
  public AngularVelocity getWheelVelocity() {
    return wheelVelocity;
  }

  /** Gets the angular velocity of the azimuth control. */
  public AngularVelocity getTurnVelocity() {
    return turnVelocity;
  }

  @Override
  public double getCurrentDraw() {
    return wheelSim.getCurrentDrawAmps() + turnSim.getCurrentDrawAmps();
  }
}
