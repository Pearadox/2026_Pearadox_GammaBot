package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class EnergyTracker {
  public enum Compeartment {
    DRIVE_MOTORS,
    TURN_MOTORS,
    FEEDER,
    INTAKE_ROLLERS,
    INTAKE_PIVOT,
    LAUNCHER,
    TURRET,
    SPINDEXER,
    CLIMBER
  }

  private static double totalChargeConsumedAh = 0.0;
  private static double totalEnergyConsumedWh = 0.0;

  private static Map<Compeartment, Double> subsystemCharges = new HashMap<>();
  private static Map<Compeartment, Double> subsystemEnergies = new HashMap<>();

  public static void reportCurrentUsage(
      double deltaHours, Compeartment subsystem, double... supplyCurrentDrawAmps) {
    double totalAmps = 0.0;
    for (double amp : supplyCurrentDrawAmps) totalAmps += amp;

    if (deltaHours > 0) {
      // 3 600 000 000 microseconds per hour
      double deltaAmpHours = totalAmps * deltaHours;
      double deltaWattHours = deltaAmpHours * RobotController.getBatteryVoltage();

      totalChargeConsumedAh += deltaAmpHours;
      totalEnergyConsumedWh += deltaWattHours;

      subsystemCharges.merge(subsystem, deltaAmpHours, Double::sum);
      subsystemEnergies.merge(subsystem, deltaWattHours, Double::sum);
    }
  }

  public static void periodic() {
    Logger.recordOutput("EnergyTracker/Total Charge", totalChargeConsumedAh, "amp hours");
    Logger.recordOutput("EnergyTracker/Total Energy", totalEnergyConsumedWh, "watt hours");

    for (var entry : subsystemCharges.entrySet()) {
      Logger.recordOutput(
          "EnergyTracker/Charges/" + entry.getKey().toString(), entry.getValue(), "amps hours");
    }

    for (var entry : subsystemEnergies.entrySet()) {
      Logger.recordOutput(
          "EnergyTracker/Energies/" + entry.getKey().toString(), entry.getValue(), "watt hours");
    }
  }
}
