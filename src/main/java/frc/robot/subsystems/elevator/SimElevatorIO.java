package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.sim.MechanismSim;
import frc.robot.sim.SimulationContext;

@Logged
public class SimElevatorIO implements ElevatorIO {
  @NotLogged private final ElevatorSim elevatorSim;
  @NotLogged private final MechanismSim mechanismSim;

  public SimElevatorIO() {
    elevatorSim =
        new ElevatorSim(
            DCMotor.getNEO(2),
            5.5,
            Kilograms.convertFrom(8 + (20 * 2), Pounds),
            Meters.convertFrom(0.866, Inches),
            Constants.ElevatorConstants.minHeight.in(Meters),
            Constants.ElevatorConstants.maxHeight.in(Meters),
            true,
            Constants.ElevatorConstants.minHeight.in(Meters));
    mechanismSim =
        new MechanismSim() {
          @Override
          public double getCurrentDraw() {
            return elevatorSim.getCurrentDrawAmps();
          }

          @Override
          public void update(double time) {
            elevatorSim.update(time);
          }
        };
    SimulationContext.getDefault().addMechanism(mechanismSim);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    elevatorSim.setInputVoltage(mechanismSim.outputVoltage(voltage.in(Volts)));
  }

  @Override
  public Current getCurrentDraw() {
    return Amps.of(elevatorSim.getCurrentDrawAmps());
  }

  @Override
  public Voltage getAppliedVoltage() {
    return Volts.of(elevatorSim.getInput(0));
  }

  @Override
  public Distance getHeight() {
    return Meters.of(elevatorSim.getPositionMeters());
  }

  @Override
  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(elevatorSim.getVelocityMetersPerSecond());
  }

  @Override
  public void resetEncoders() {
    elevatorSim.setState(0, 0);
  }
}
