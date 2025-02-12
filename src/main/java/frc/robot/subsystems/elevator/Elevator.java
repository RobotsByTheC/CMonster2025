package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@Logged
public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ProfiledPIDController profiledPIDController;
  private final ElevatorFeedforward feedforward;

  public final Trigger atMinHeight = new Trigger(() -> getHeight().lte(minHeight));
  public final Trigger atMaxHeight = new Trigger(() -> getHeight().gte(maxHeight));
  private final Debouncer stallingDebouncer = new Debouncer(stallDuration.in(Seconds));
  public final Trigger isStalling =
      new Trigger(() -> stallingDebouncer.calculate(getCurrentDraw().gte(stallThreshold)));
  private final SysIdRoutine sysIdRoutine;

  public Elevator(ElevatorIO io) {
    this.io = io;
    feedforward = new ElevatorFeedforward(KS, KG, KV, KA);
    profiledPIDController =
        new ProfiledPIDController(
            KP,
            KI,
            KD,
            new TrapezoidProfile.Constraints(feedforward.maxAchievableVelocity(12.5, 20), 20));
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(4), null),
            new SysIdRoutine.Mechanism(
                io::setVoltage,
                log ->
                    log.motor("Elevator Motor")
                        .current(io.getCurrentDraw())
                        .voltage(io.getAppliedVoltage())
                        .linearPosition(io.getHeight())
                        .linearVelocity(io.getVelocity()),
                this));
  }

  private Distance getHeight() {
    return io.getHeight();
  }

  private Current getCurrentDraw() {
    return io.getCurrentDraw();
  }

  public Command stop() {
    return run(() -> io.setVoltage(Volts.of(0))).withName("Stop");
  }

  @SuppressWarnings("unused")
  public Command applyVoltage(Voltage voltage) {
    return run(() -> io.setVoltage(voltage)).withName("Set Voltage: " + voltage);
  }

  public Command home() {
    return run(() -> io.setVoltage(Volts.of(-2)))
        .until(() -> io.getCurrentDraw().gte(Amps.of(15)))
        .finallyDo(
            (boolean interrupted) -> {
              if (!interrupted) {
                io.resetEncoders();
              }
            })
        .withName("Home");
  }

  public Command goToHeight(Distance targetHeight) {
    return startRun(
            () ->
                profiledPIDController.reset(
                    io.getHeight().in(Meters), io.getVelocity().in(MetersPerSecond)),
            () -> io.setVoltage(calculatePIDVoltage(targetHeight)))
        .until(profiledPIDController::atGoal)
        .withName("Go To Height " + targetHeight.toLongString());
  }

  public Command goToL1Height() {
    return goToHeight(l1).withName("Rising to L1");
  }

  public Command goToL2Height() {
    return goToHeight(l2).withName("Rising to L2");
  }

  public Command goToL3Height() {
    return goToHeight(l3).withName("Rising to L3");
  }

  public Command goToL4Height() {
    return goToHeight(l4).withName("Rising to L4");
  }

  public Command holdCurrentPosition() {
    MutDistance startingHeight = Meters.mutable(0);
    return startRun(
        () -> {
          startingHeight.mut_replace(io.getHeight());
          profiledPIDController.reset(
              io.getHeight().in(Meters), io.getVelocity().in(MetersPerSecond));
        },
        () -> io.setVoltage(calculatePIDVoltage(startingHeight)));
  }

  public Command runSysIdRoutine() {
    return sysIdRoutine
        .dynamic(SysIdRoutine.Direction.kForward)
        .until(atMaxHeight)
        .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(atMinHeight))
        .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(atMaxHeight))
        .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(atMinHeight))
        .withName("Elevator Sysid Routine");
  }

  private Voltage calculatePIDVoltage(Distance targetHeight) {
    double pidVoltage =
        profiledPIDController.calculate(io.getHeight().in(Meters), targetHeight.in(Meters));
    double feedForwardVoltage = feedforward.calculate(0);
    return Volts.of(pidVoltage + feedForwardVoltage);
  }
}
