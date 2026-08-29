package frc.robot.util;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.feeder.FeederConstants;
import frc.robot.subsystems.launcher.LauncherConstants;
import frc.robot.subsystems.spindexer.SpindexerConstants;

/**
 * Geometry for estimating how fast fuel actually moves through hopper, feeder, and launcher.
 *
 * <p>The no-jam rule is that fuel must never be pushed into a stage slower than the one it is
 * leaving: hopper fuel speed &lt; feeder fuel speed &lt; launcher entry speed. Motor commands
 * cannot be compared directly (the launcher is closed-loop velocity, the feeder is open-loop
 * voltage, and the spindexer is torque current, which commands torque rather than speed), so {@link
 * FeedChain} converts each stage's MEASURED motor velocity into a fuel speed using the constants
 * below.
 *
 * <p>Each value is marked MEASURED, ESTIMATE, or PLACEHOLDER. Until every PLACEHOLDER is replaced
 * with a real measurement and {@link #GEOMETRY_MEASURED} is flipped to true, the monitor logs its
 * numbers but does not raise the ordering alert, because an alert computed from invented radii
 * would be worse than no alert.
 */
public class FeedChainConstants {

  /**
   * Flip to true once every PLACEHOLDER below has been replaced by a measurement. Gates the
   * ordering alert only; the logged speeds are always published.
   */
  public static final boolean GEOMETRY_MEASURED = false;

  // ---- Hopper (spindexer) ----

  /**
   * PLACEHOLDER. Radius at which fuel leaves the rotating floor, which is not necessarily the
   * SPINDEXER_RADIUS_METERS used for the sim moment of inertia. Measure to the handoff point.
   */
  public static final double HOPPER_FLOOR_EXIT_RADIUS_METERS = Units.inchesToMeters(4.0);

  /**
   * PLACEHOLDER. The source disagrees with itself: SPINDEXER_GEARING is 1/3, its own comment says
   * "technically 1/18 (according to Hitesh)", and an earlier value was 1/15. Count floor
   * revolutions against motor rotations on video before trusting the derived speeds.
   */
  public static final double HOPPER_GEARING = SpindexerConstants.SPINDEXER_GEARING;

  /**
   * ESTIMATE, tunable at runtime. Fuel rests on the moving floor and is free to roll, so it does
   * not travel at the floor's surface speed. Somewhere in [0.5, 1.0]; 1.0 would mean no slip at
   * all.
   */
  public static final double HOPPER_FUEL_ROLL_FACTOR = 0.7;

  // ---- Feeder ----

  /** PLACEHOLDER. Not recorded anywhere in the code; take it off the CAD or the real wheel. */
  public static final double FEEDER_WHEEL_RADIUS_METERS = Units.inchesToMeters(1.0);

  /** From FeederConstants: motor teeth over pulley teeth. */
  public static final double FEEDER_GEARING = FeederConstants.FEEDER_GEARING;

  /**
   * PHYSICS. The feeder wheel contacts fuel on one side only, so the fuel rolls against the
   * opposing surface and its center advances at half the wheel's surface speed.
   */
  public static final double FEEDER_FUEL_ROLL_FACTOR = 0.5;

  // ---- Launcher ----

  /** ESTIMATE. Taken from the sim roller radius; confirm against the real flywheel. */
  public static final double LAUNCHER_WHEEL_RADIUS_METERS = LauncherConstants.ROLLER_RADIUS_METERS;

  public static final double LAUNCHER_GEARING = LauncherConstants.LAUNCHER_GEARING;

  /** PHYSICS. Same one-sided contact as the feeder while fuel is entering the launcher. */
  public static final double LAUNCHER_ENTRY_FACTOR = 0.5;

  /**
   * PHYSICS. The top roller at the exit pinches the fuel, so it leaves at close to the wheel's
   * surface speed rather than half of it.
   */
  public static final double LAUNCHER_EXIT_FACTOR = 1.0;

  // ---- Monitor and master throughput ----

  /** ESTIMATE, tunable. Headroom each stage must have over the stage feeding it. */
  public static final double ORDERING_MARGIN = 1.15;

  public static final double MIN_THROUGHPUT = 0.3;
  public static final double MAX_THROUGHPUT = 1.3;

  private FeedChainConstants() {}
}
