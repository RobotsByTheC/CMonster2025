package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CoralConstants.KA;
import static frc.robot.Constants.CoralConstants.KD;
import static frc.robot.Constants.CoralConstants.KG;
import static frc.robot.Constants.CoralConstants.KI;
import static frc.robot.Constants.CoralConstants.KP;
import static frc.robot.Constants.CoralConstants.KS;
import static frc.robot.Constants.CoralConstants.KV;
import static frc.robot.Constants.CoralConstants.branchScoreAngle;
import static frc.robot.Constants.CoralConstants.grabDoneLimit;
import static frc.robot.Constants.CoralConstants.grabIntakeVoltage;
import static frc.robot.Constants.CoralConstants.grabScoreVoltage;
import static frc.robot.Constants.CoralConstants.grabStallDuration;
import static frc.robot.Constants.CoralConstants.grabStallLimit;
import static frc.robot.Constants.CoralConstants.intakeAngle;
import static frc.robot.Constants.CoralConstants.maxWristAngle;
import static frc.robot.Constants.CoralConstants.minWristAngle;
import static frc.robot.Constants.CoralConstants.stowAngle;
import static frc.robot.Constants.CoralConstants.tipScoreAngle;
import static frc.robot.Constants.CoralConstants.troughScoreAngle;
import static frc.robot.Constants.CoralConstants.wristTolerance;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.math.MovingAverage;
import java.util.function.BooleanSupplier;

@Logged
public class Coral extends SubsystemBase {
  private CoralIO io;
  private final ProfiledPIDController profiledPIDController;
  private final ArmFeedforward feedforward;
  @NotLogged private final SysIdRoutine sysIdRoutine;

  private double pidVoltage;
  private double feedForwardVoltage;

  private final MovingAverage movingAverage = new MovingAverage(9);

  public final Trigger atMaxAngle = new Trigger(() -> io.getWristAngle().gte(maxWristAngle));
  public final Trigger atMinAngle = new Trigger(() -> io.getWristAngle().lte(minWristAngle));

  private final Debouncer stallingDebouncer = new Debouncer(grabStallDuration.in(Seconds));

  public final Trigger isGrabberStalling =
      new Trigger(() -> stallingDebouncer.calculate(getFilteredCurrentDraw().gte(grabStallLimit)));

  public final Trigger isGrabberDone =
      new Trigger(() -> stallingDebouncer.calculate(getFilteredCurrentDraw().lte(grabDoneLimit)));

  public Coral(CoralIO io) {
    this.io = io;
    feedforward = new ArmFeedforward(KS, KG, KV, KA);
    profiledPIDController =
        new ProfiledPIDController(
            KP,
            KI,
            KD,
            new TrapezoidProfile.Constraints(feedforward.maxAchievableVelocity(12.5, 0, 20), 20));
    profiledPIDController.setTolerance(wristTolerance.in(Radians));
    profiledPIDController.enableContinuousInput(0, 2 * Math.PI);
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(4), null),
            new SysIdRoutine.Mechanism(io::setWristVoltage, null, this));
  }

  public Current getFilteredCurrentDraw() {
    return Amps.of(movingAverage.getAverage());
  }

  @Override
  public void periodic() {
    movingAverage.addNumber(io.getGrabCurrentDraw().in(Amps));
  }

  public Command scoreL1() {
    return coordinatedControl(troughScoreAngle, grabScoreVoltage, isGrabberDone)
        .withName("Score L1");
  }

  public Command scoreL2() {
    return coordinatedControl(branchScoreAngle, grabScoreVoltage, isGrabberDone)
        .withName("Score L2");
  }

  public Command scoreL3() {
    return coordinatedControl(branchScoreAngle, grabScoreVoltage, isGrabberDone)
        .withName("Score L3");
  }

  public Command scoreL4() {
    return coordinatedControl(tipScoreAngle, grabScoreVoltage, isGrabberDone).withName("Score L4");
  }

  public Command stop() {
    return run(() -> {
          io.setGrabVoltage(Volts.of(0));
          io.setWristVoltage(Volts.of(0));
        })
        .withName("Stop coral");
  }

  public Command stow() {
    return coordinatedControl(stowAngle, Volts.zero(), () -> false).withName("Stow Coral Arm");
  }

  public Command intake() {
    return coordinatedControl(intakeAngle, grabIntakeVoltage, isGrabberStalling)
        .withName("Intake coral");
  }

  public Command runSysIdRoutine() {
    return sysIdRoutine
        .dynamic(SysIdRoutine.Direction.kForward)
        .until(atMaxAngle)
        .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(atMinAngle))
        .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(atMaxAngle))
        .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(atMinAngle))
        .withName("Coral Sysid Routine");
  }

  private Command moveWrist(Angle wristAngle) {
    return Commands.startRun(
        () ->
            profiledPIDController.reset(
                io.getWristAngle().in(Radians), io.getWristVelocity().in(RadiansPerSecond)),
        () -> io.setWristVoltage(calculatePIDVoltage(wristAngle)));
  }

  private Command controlGrabber(Voltage voltage) {
    return Commands.run(() -> io.setGrabVoltage(voltage));
  }

  private Command coordinatedControl(
      Angle angle, Voltage grabVoltage, BooleanSupplier endCondition) {
    Command command =
        moveWrist(angle)
            .until(profiledPIDController::atSetpoint)
            .andThen(moveWrist(angle).alongWith(controlGrabber(grabVoltage)).until(endCondition));
    command.addRequirements(this);
    return command;
  }

  private Voltage calculatePIDVoltage(Angle targetAngle) {
    pidVoltage =
        profiledPIDController.calculate(io.getWristAngle().in(Radians), targetAngle.in(Radians));
    feedForwardVoltage = feedforward.calculate(io.getWristAngle().in(Radians), 0);
    return Volts.of(pidVoltage + feedForwardVoltage);
  }
}
