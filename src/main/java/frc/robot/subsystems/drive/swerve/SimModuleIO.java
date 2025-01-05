package frc.robot.subsystems.drive.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ModuleConstants.wheelCircumference;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.sim.MAXSwerveModuleSim;
import frc.robot.sim.Simulation;
import frc.robot.sim.SimulationContext;

@Logged
public class SimModuleIO implements ModuleIO {
  /** Hardware simulation for the swerve module. */
  private final MAXSwerveModuleSim sim;

  // PID constants are different in simulation than in real life
  private final PIDController drivePID = new PIDController(12, 0, 0);
  private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.8, 2.445);
  private final PIDController turnPID = new PIDController(10, 0, 0);

  private final SwerveModulePosition currentPosition = new SwerveModulePosition();
  private final Simulation update = this::update;

  private Voltage lastVoltage = Volts.zero();

  private SwerveModuleState desiredState = new SwerveModuleState();

  private Voltage turnVolts = Volts.zero();

  public SimModuleIO(MAXSwerveModuleSim sim) {
    this.sim = sim;

    turnPID.enableContinuousInput(0, 2 * Math.PI);
    SimulationContext.getDefault().addPeriodic(update);
  }

  public SimModuleIO() {
    this(new MAXSwerveModuleSim());
  }

  private void update(double timestep) {
    // Set simulation motor voltages based on the commanded inputs to the module
    Voltage driveVolts =
        Volts.of(
            drivePID.calculate(getWheelVelocity().in(MetersPerSecond))
                + driveFeedForward.calculate(desiredState.speedMetersPerSecond));
    turnVolts = Volts.of(turnPID.calculate(getModuleRotation().getRadians()));

    lastVoltage = driveVolts;

    sim.setDriveVoltage(driveVolts);
    sim.setTurnVoltage(turnVolts);

    // Write "sensor" values by integrating the simulated velocities over the past timestep
    currentPosition.angle =
        currentPosition.angle.plus(
            Rotation2d.fromRadians(sim.getTurnVelocity().in(RadiansPerSecond) * timestep));
    currentPosition.distanceMeters += getWheelVelocity().in(MetersPerSecond) * timestep;
  }

  @Override
  public LinearVelocity getWheelVelocity() {
    return MetersPerSecond.of(
        sim.getWheelVelocity().in(RotationsPerSecond) * wheelCircumference.in(Meters));
  }

  @Override
  public Distance getWheelDistance() {
    return Meters.of(currentPosition.distanceMeters);
  }

  @Override
  public Rotation2d getModuleRotation() {
    return currentPosition.angle;
  }

  @Override
  public void setDesiredState(SwerveModuleState state) {
    this.desiredState = state;
    drivePID.setSetpoint(state.speedMetersPerSecond);
    turnPID.setSetpoint(state.angle.getRadians());
  }

  @Override
  public void stop() {
    setDesiredState(new SwerveModuleState(0, getModuleRotation()));
  }

  @Override
  public void resetEncoders() {
    currentPosition.distanceMeters = 0;
  }

  public Voltage getDriveVoltage() {
    return lastVoltage;
  }

  @Override
  public void close() {
    // Stop simulating the mechanism
    SimulationContext.getDefault().removeMechanism(sim);
    SimulationContext.getDefault().removePeriodic(update);
  }

  @Override
  public Voltage getTurnVoltage() {
    return turnVolts;
  }

  @Override
  public void setDrivingMotorVoltage(Voltage v) {
    sim.setDriveVoltage(v);
  }
}
