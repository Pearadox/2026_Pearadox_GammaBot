package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Estimates how fast fuel is moving at each handoff in the feed chain, and warns when a stage is
 * pushing fuel into a slower one (which is how the chain jams).
 *
 * <p>This only observes and reports. It does not cap any stage's command: a governor that actually
 * enforces the ordering is worth building only after the geometry in {@link FeedChainConstants} has
 * been measured and this monitor has been watched through a practice session.
 *
 * <p>The one thing it does drive is the master throughput slider, which scales the feeder voltage
 * and the spindexer current together so a driver can back the whole chain off (or push it) without
 * re-deriving each stage. The launcher is deliberately not scaled: its speed belongs to the shot
 * solution.
 *
 * <p>Caveat worth remembering when reading the numbers: scaling spindexer torque current scales
 * TORQUE, and its speed only follows through whatever friction load the hopper happens to have. The
 * measured speeds below, not the slider position, are the ground truth for whether ordering holds.
 */
public class FeedChain {
  private static final LoggedTunableNumber masterThroughput =
      new LoggedTunableNumber("FeedChain/Master Throughput", 1.0);

  private static final LoggedTunableNumber hopperRollFactor =
      new LoggedTunableNumber(
          "FeedChain/Hopper Roll Factor", FeedChainConstants.HOPPER_FUEL_ROLL_FACTOR);
  private static final LoggedTunableNumber orderingMargin =
      new LoggedTunableNumber("FeedChain/Ordering Margin", FeedChainConstants.ORDERING_MARGIN);

  /**
   * Scales the feeder and spindexer commands together. Clamped, so a fat-fingered dashboard entry
   * cannot stall the chain or drive it past what the mechanisms tolerate.
   */
  public static double getMasterThroughput() {
    return MathUtil.clamp(
        masterThroughput.get(),
        FeedChainConstants.MIN_THROUGHPUT,
        FeedChainConstants.MAX_THROUGHPUT);
  }

  private final DoubleSupplier spindexerRotorRPS;
  private final DoubleSupplier feederRotorRPS;
  private final DoubleSupplier launcherRPS;
  private final BooleanSupplier feederRunning;

  private final Alert orderingAlert =
      new Alert(
          "Feed chain ordering violated: a stage is feeding a slower one.", AlertType.kWarning);
  private final Alert geometryAlert =
      new Alert(
          "Feed chain geometry is unmeasured (FeedChainConstants.GEOMETRY_MEASURED is false), so"
              + " the ordering alert is suppressed.",
          AlertType.kInfo);

  public FeedChain(
      DoubleSupplier spindexerRotorRPS,
      DoubleSupplier feederRotorRPS,
      DoubleSupplier launcherRPS,
      BooleanSupplier feederRunning) {
    this.spindexerRotorRPS = spindexerRotorRPS;
    this.feederRotorRPS = feederRotorRPS;
    this.launcherRPS = launcherRPS;
    this.feederRunning = feederRunning;

    geometryAlert.set(!FeedChainConstants.GEOMETRY_MEASURED);
  }

  /** Surface speed of a roller, in meters per second, from its motor's rotor velocity. */
  private static double surfaceSpeed(double rotorRPS, double gearing, double radiusMeters) {
    return Math.abs(rotorRPS) * gearing * 2.0 * Math.PI * radiusMeters;
  }

  public void update() {
    double hopperFuelSpeed =
        surfaceSpeed(
                spindexerRotorRPS.getAsDouble(),
                FeedChainConstants.HOPPER_GEARING,
                FeedChainConstants.HOPPER_FLOOR_EXIT_RADIUS_METERS)
            * hopperRollFactor.get();

    double feederFuelSpeed =
        surfaceSpeed(
                feederRotorRPS.getAsDouble(),
                FeedChainConstants.FEEDER_GEARING,
                FeedChainConstants.FEEDER_WHEEL_RADIUS_METERS)
            * FeedChainConstants.FEEDER_FUEL_ROLL_FACTOR;

    double launcherSurfaceSpeed =
        surfaceSpeed(
            launcherRPS.getAsDouble(),
            FeedChainConstants.LAUNCHER_GEARING,
            FeedChainConstants.LAUNCHER_WHEEL_RADIUS_METERS);

    double launcherEntryCapability =
        launcherSurfaceSpeed * FeedChainConstants.LAUNCHER_ENTRY_FACTOR;
    double launcherExitSpeed = launcherSurfaceSpeed * FeedChainConstants.LAUNCHER_EXIT_FACTOR;

    double margin = orderingMargin.get();
    boolean hopperIntoFeederOK = hopperFuelSpeed * margin <= feederFuelSpeed;
    boolean feederIntoLauncherOK = feederFuelSpeed * margin <= launcherEntryCapability;
    boolean orderingOK = hopperIntoFeederOK && feederIntoLauncherOK;

    Logger.recordOutput("FeedChain/HopperFuelSpeed", hopperFuelSpeed);
    Logger.recordOutput("FeedChain/FeederFuelSpeed", feederFuelSpeed);
    Logger.recordOutput("FeedChain/LauncherEntryCapability", launcherEntryCapability);
    Logger.recordOutput("FeedChain/LauncherExitSpeed", launcherExitSpeed);
    Logger.recordOutput("FeedChain/OrderingOK", orderingOK);
    Logger.recordOutput("FeedChain/MasterThroughput", getMasterThroughput());

    // Ratios above 1.0 mean that handoff has the headroom it wants.
    Logger.recordOutput("FeedChain/FeederOverHopperRatio", ratio(feederFuelSpeed, hopperFuelSpeed));
    Logger.recordOutput(
        "FeedChain/LauncherOverFeederRatio", ratio(launcherEntryCapability, feederFuelSpeed));

    // A violation only means anything while fuel is actually being pushed through.
    orderingAlert.set(
        FeedChainConstants.GEOMETRY_MEASURED && feederRunning.getAsBoolean() && !orderingOK);
  }

  private static double ratio(double downstream, double upstream) {
    return upstream > 1e-6 ? downstream / upstream : Double.POSITIVE_INFINITY;
  }
}
