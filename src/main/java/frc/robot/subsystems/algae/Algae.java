package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.AlgaeConstants.*;
import static frc.robot.Constants.CoralConstants.maxWristAngle;
import static frc.robot.Constants.CoralConstants.minWristAngle;

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

public class Algae extends SubsystemBase {
  private AlgaeIO io;
  private final ProfiledPIDController profiledPIDController;
  private final ArmFeedforward feedForward;
  private final SysIdRoutine sysIdRoutine;
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
            new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(io::setWristVoltage, null, this));
  }

  public Command intakeGround() {
    return coordinatedControl(groundIntakeAngle, grabIntakeVoltage, () -> io.hasAlgae());
  }

  public Command intakeReef() {
    return coordinatedControl(reefIntakeAngle, grabIntakeVoltage, () -> io.hasAlgae());
  }

  public Command scoreProcessor() {
    return coordinatedControl(processorScoreAngle, grabScoreVoltage, () -> !io.hasAlgae())
        .withName("Algae Score Processor");
  }
  public Command stow() {
    return coordinatedControl(stowAngle, Volts.zero(), () -> false).withName("Stow Algae Arm");
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
    double pidVoltage =
        profiledPIDController.calculate(io.getWristAngle().in(Radians), targetAngle.in(Radians));
    double feedForwardVoltage = feedForward.calculate(targetAngle.in(Radians), 0);
    return Volts.of(pidVoltage + feedForwardVoltage);
  }
}
