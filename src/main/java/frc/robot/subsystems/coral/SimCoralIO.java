package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.CoralConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.sim.MechanismSim;
import frc.robot.sim.SimulationContext;

@Logged
public class SimCoralIO implements CoralIO {
  private boolean hasCoral;
  private Time grabberActionStart = Milliseconds.of(System.currentTimeMillis());

  private Voltage grabberVoltage = Volts.zero();
  @NotLogged private final SingleJointedArmSim wristSim;
  @NotLogged private final MechanismSim mechanismSim;

  public SimCoralIO() {
    wristSim =
        new SingleJointedArmSim(
            DCMotor.getNEO(1),
            wristGearing,
            0.1,
            Meters.convertFrom(10, Inches),
            minWristAngle.in(Radians),
            maxWristAngle.in(Radians),
            true,
            stowAngle.in(Radians));
    mechanismSim =
        new MechanismSim() {
          @Override
          public double getCurrentDraw() {
            return wristSim.getCurrentDrawAmps();
          }

          @Override
          public void update(double timestep) {
            wristSim.update(timestep);
          }
        };
    SimulationContext.getDefault().addMechanism(mechanismSim);
  }

  @Override
  public void setWristVoltage(Voltage voltage) {
    wristSim.setInputVoltage(mechanismSim.outputVoltage(voltage.in(Volts)));
  }

  @Override
  public void setGrabVoltage(Voltage voltage) {
    if (!grabberVoltage.equals(voltage)) {
      grabberActionStart = Milliseconds.of(System.currentTimeMillis());
    }
    if (System.currentTimeMillis() - grabberActionStart.in(Milliseconds) > 500) {
      if (grabberVoltage.magnitude() > 0) {
        hasCoral = true;
      } else if (grabberVoltage.magnitude() < 0) {
        hasCoral = false;
      }
    }

    grabberVoltage = voltage;
  }

  @Override
  public Current getWristCurrentDraw() {
    return Amps.of(wristSim.getCurrentDrawAmps());
  }

  @Override
  public Current getGrabCurrentDraw() {
    return Amps.zero();
  }

  @Override
  public Voltage getWristAppliedVoltage() {
    return Volts.of(wristSim.getInput(0));
  }

  @Override
  public boolean hasLeftCoral() {
    return hasCoral;
  }

  @Override
  public boolean hasRightCoral() {
    return hasCoral;
  }

  @Override
  public Angle getWristAngle() {
    return Radians.of(wristSim.getAngleRads());
  }

  @Override
  public AngularVelocity getWristVelocity() {
    return RadiansPerSecond.of(wristSim.getVelocityRadPerSec());
  }
}
