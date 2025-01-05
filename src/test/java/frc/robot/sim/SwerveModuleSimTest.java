package frc.robot.sim;

import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Note: all tests run with a constant 12V battery input. */
class SwerveModuleSimTest {
  @BeforeAll
  static void wpilibSetup() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData(); // "enable" the robot so motor controllers will work
  }

  @AfterEach
  void teardown() {
    SimulationContext.getDefault().removeAll();
  }

  @Test
  void updatesPositiveVelocity() {
    var sim = new MAXSwerveModuleSim();
    sim.setDriveVoltage(Volts.of(12));
    sim.update(0.02);
    assertTrue(
        sim.getWheelVelocity().magnitude() > 0, sim.getWheelVelocity() + " should be positive");
  }

  @Test
  void updatesNegativeVelocity() {
    var sim = new MAXSwerveModuleSim();
    sim.setDriveVoltage(Volts.of(-12));
    sim.update(0.02);
    assertTrue(
        sim.getWheelVelocity().magnitude() < 0, sim.getWheelVelocity() + " should be negative");
  }

  @Test
  void updatesPositiveAngle() {
    var sim = new MAXSwerveModuleSim();
    sim.setTurnVoltage(Volts.of(12));
    sim.update(0.02);
    assertTrue(
        sim.getTurnVelocity().magnitude() > 0, sim.getTurnVelocity() + " should be positive");
  }

  @Test
  void updatesNegativeAngle() {
    var sim = new MAXSwerveModuleSim();
    sim.setTurnVoltage(Volts.of(-12));
    sim.update(0.02);
    assertTrue(
        sim.getTurnVelocity().magnitude() < 0, sim.getTurnVelocity() + " should be positive");
  }
}
