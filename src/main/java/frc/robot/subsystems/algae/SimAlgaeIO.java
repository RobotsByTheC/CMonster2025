package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.AlgaeConstants.maxWristAngle;
import static frc.robot.Constants.AlgaeConstants.minWristAngle;
import static frc.robot.Constants.AlgaeConstants.stowAngle;
import static frc.robot.Constants.AlgaeConstants.wristGearing;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.sim.MechanismSim;
import frc.robot.sim.SimulationContext;

@Logged
public class SimAlgaeIO implements AlgaeIO {
  @NotLogged private final SingleJointedArmSim wristSim;
  @NotLogged private final MechanismSim mechanismSim;

  private Time grabberActionStart = Milliseconds.of(System.currentTimeMillis());
  private boolean hasAlgae = false;
  private Voltage grabberVoltage = Volts.zero();

  public SimAlgaeIO() {
    wristSim =
        new SingleJointedArmSim(
            DCMotor.getNEO(1),
            wristGearing,
            0.1,
            Meters.convertFrom(12, Inches),
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
        hasAlgae = true;
      } else if (grabberVoltage.magnitude() < 0) {
        hasAlgae = false;
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
  public boolean hasAlgae() {
    return hasAlgae;
  }

  @Override
  public Angle getWristAngle() {
    Angle r = Radians.of(wristSim.getAngleRads());
    if (r.gt(Degrees.of(180))) {
      return r.minus(Degrees.of(360));
    } else {
      return r;
    }
  }

  @Override
  public AngularVelocity getWristVelocity() {
    return RadiansPerSecond.of(wristSim.getVelocityRadPerSec());
  }
}
