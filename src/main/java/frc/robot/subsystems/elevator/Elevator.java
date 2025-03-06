package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.KA;
import static frc.robot.Constants.ElevatorConstants.KD;
import static frc.robot.Constants.ElevatorConstants.KG;
import static frc.robot.Constants.ElevatorConstants.KI;
import static frc.robot.Constants.ElevatorConstants.KP;
import static frc.robot.Constants.ElevatorConstants.KS;
import static frc.robot.Constants.ElevatorConstants.KV;
import static frc.robot.Constants.ElevatorConstants.algaeIntake;
import static frc.robot.Constants.ElevatorConstants.intake;
import static frc.robot.Constants.ElevatorConstants.l1;
import static frc.robot.Constants.ElevatorConstants.l2;
import static frc.robot.Constants.ElevatorConstants.l3;
import static frc.robot.Constants.ElevatorConstants.l4;
import static frc.robot.Constants.ElevatorConstants.maxHeight;
import static frc.robot.Constants.ElevatorConstants.minHeight;
import static frc.robot.Constants.ElevatorConstants.stallDuration;
import static frc.robot.Constants.ElevatorConstants.stallThreshold;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutVoltage;
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
  @NotLogged private final Debouncer stallingDebouncer = new Debouncer(stallDuration.in(Seconds));

  public final Trigger isStalling =
      new Trigger(() -> stallingDebouncer.calculate(getCurrentDraw().gte(stallThreshold)));

  @NotLogged private final SysIdRoutine sysIdRoutine;

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
            new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(5), null),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  if (voltage.magnitude() < 0) {
                    io.setVoltage(voltage.div(3));
                  } else {
                    io.setVoltage(voltage);
                  }
                },
                log ->
                    log.motor("Elevator Motor")
                        .current(io.getCurrentDraw())
                        .voltage(io.getAppliedVoltage())
                        .linearPosition(io.getHeight())
                        .linearVelocity(io.getVelocity()),
                this));
    io.resetEncoders();
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
        .until(
            new Trigger(
                    () ->
                        io.getCurrentDraw().gte(Amps.of(15))
                            && (io.getVelocity().abs(InchesPerSecond)) < .1)
                .debounce(.15))
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
            () -> {
              Voltage calc = calculatePIDVoltage(targetHeight);
      
                if (calc.magnitude() < 0) {
                  io.setVoltage(calc.div(2));
                } else {
                  io.setVoltage(calc);
                }
            })
        .until(profiledPIDController::atGoal)
        .withName("Go To Height " + targetHeight.toLongString());
  }

  public Command goToBottom() {
    return goToHeight(minHeight.plus(Inches.of(1.5))).withName("Dropping to Min height").andThen(home());
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

  public Command goToIntakeHeight() {
    return goToHeight(intake).withName("Rising to Inatke");
  }

  public Command goToAlgaeIntakeHeight() {
    return goToHeight(algaeIntake).withName("Rising to Algae");
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

  public Command findFeedforwardTerms() {
    MutVoltage appliedVoltage = Volts.mutable(1);
    MutVoltage riseVoltage = Volts.mutable(0);
    MutVoltage fallVoltage = Volts.mutable(0);

    Voltage increment = Volts.of(0.01);
    LinearVelocity cutoffVelocity = InchesPerSecond.of(1);

    Trigger triggerUp = new Trigger(() -> io.getVelocity().gte(cutoffVelocity)).debounce(0.5);
    Trigger triggerDown =
        new Trigger(() -> io.getVelocity().lte(cutoffVelocity.unaryMinus())).debounce(0.5);

    Command up =
        startRun(
                () -> appliedVoltage.mut_setBaseUnitMagnitude(1),
                () -> {
                  appliedVoltage.mut_plus(increment);
                  io.setVoltage(appliedVoltage);
                })
            .until(triggerUp)
            .finallyDo(() -> riseVoltage.mut_replace(appliedVoltage));

    Command down =
        run(() -> {
              appliedVoltage.mut_minus(increment);
              io.setVoltage(appliedVoltage);
            })
            .until(triggerDown)
            .finallyDo(() -> fallVoltage.mut_replace(appliedVoltage));

    return up.andThen(down)
        .finallyDo(
            () -> {
              double riseVolts = riseVoltage.in(Volts);
              double fallVolts = fallVoltage.in(Volts);

              double kg = (riseVolts + fallVolts) / 2;
              double ks = (riseVolts - fallVolts) / 2;

              System.out.printf("kG = %.3f%n", kg);
              System.out.printf("kS = %.3f%n", ks);
            })
        .withName("Find Elevator Feedforward Terms");
  }

  private Voltage calculatePIDVoltage(Distance targetHeight) {
    double pidVoltage =
        profiledPIDController.calculate(io.getHeight().in(Meters), targetHeight.in(Meters));
    double feedForwardVoltage = feedforward.calculate(0);
    return Volts.of(pidVoltage + feedForwardVoltage);
  }
}
