package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.AlgaeConstants.KA;
import static frc.robot.Constants.AlgaeConstants.KD;
import static frc.robot.Constants.AlgaeConstants.KG;
import static frc.robot.Constants.AlgaeConstants.KI;
import static frc.robot.Constants.AlgaeConstants.KP;
import static frc.robot.Constants.AlgaeConstants.KS;
import static frc.robot.Constants.AlgaeConstants.KV;
import static frc.robot.Constants.AlgaeConstants.grabIntakeVoltage;
import static frc.robot.Constants.AlgaeConstants.grabScoreVoltage;
import static frc.robot.Constants.AlgaeConstants.groundIntakeAngle;
import static frc.robot.Constants.AlgaeConstants.maxWristAngle;
import static frc.robot.Constants.AlgaeConstants.minWristAngle;
import static frc.robot.Constants.AlgaeConstants.processorScoreAngle;
import static frc.robot.Constants.AlgaeConstants.reefIntakeAngle;
import static frc.robot.Constants.AlgaeConstants.stowAngle;
import static frc.robot.Constants.AlgaeConstants.wristTolerance;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.BooleanSupplier;

@Logged
public class Algae extends SubsystemBase {
  private AlgaeIO io;
  private final ProfiledPIDController profiledPIDController;
  private final ArmFeedforward feedForward;

  private double pidVoltage;
  private double feedForwardVoltage;

  @NotLogged private final SysIdRoutine sysIdRoutine;
  public final Trigger atMaxAngle = new Trigger(() -> io.getWristAngle().gte(maxWristAngle));
  public final Trigger atMinAngle = new Trigger(() -> io.getWristAngle().lte(minWristAngle));

  public Algae(AlgaeIO io) {
    this.io = io;
    feedForward = new ArmFeedforward(KS, KG, KV, KA);
    profiledPIDController =
        new ProfiledPIDController(
            KP,
            KI,
            KD,
            new TrapezoidProfile.Constraints(feedForward.maxAchievableVelocity(12.5, 0, 20), 20));
    profiledPIDController.setTolerance(wristTolerance.in(Radians));
    profiledPIDController.enableContinuousInput(0, 2 * Math.PI);
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(3), null),
            new SysIdRoutine.Mechanism(io::setWristVoltage, null, this));
  }

  public Command intakeGround() {
    return coordinatedControl(groundIntakeAngle, grabIntakeVoltage, () -> io.hasAlgae())
        .withName("Algae Ground Intake");
  }

  public Command intakeReef() {
    return coordinatedControl(reefIntakeAngle, grabIntakeVoltage, () -> io.hasAlgae())
        .withName("Algae Reef Intake");
  }

  public Command scoreProcessor() {
    return coordinatedControl(processorScoreAngle, grabScoreVoltage, () -> !io.hasAlgae())
        .withName("Algae Score Processor");
  }

  public Command stow() {
    return coordinatedControl(stowAngle, Volts.zero(), () -> false).withName("Stow Algae Arm");
  }

  public Command stop() {
    return run(() -> {
      io.setGrabVoltage(Volts.of(0));
      io.setWristVoltage(Volts.of(0));
    }).withName("Stop Algae");
  }

  public Command runSysIdRoutine() {
    return sysIdRoutine
        .dynamic(SysIdRoutine.Direction.kReverse)
        .until(atMinAngle)
        .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(atMaxAngle))
        .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(atMinAngle))
        .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(atMaxAngle))
        .withName("Algae Sysid Routine");
  }

  private Command coordinatedControl(
      Angle angle, Voltage grabVoltage, BooleanSupplier endCondition) {
    Command command =
        moveWrist(angle)
            .until(profiledPIDController::atGoal)
            .andThen(moveWrist(angle).alongWith(controlGrabber(grabVoltage)).until(endCondition));
    command.addRequirements(this);
    return command;
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

  private Voltage calculatePIDVoltage(Angle targetAngle) {
    pidVoltage =
        profiledPIDController.calculate(io.getWristAngle().in(Radians), targetAngle.in(Radians));
    feedForwardVoltage = feedForward.calculate(io.getWristAngle().in(Radians), 0);
    return Volts.of(pidVoltage + feedForwardVoltage);
  }
}
