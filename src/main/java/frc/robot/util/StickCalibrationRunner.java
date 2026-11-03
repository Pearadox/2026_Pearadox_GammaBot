package frc.robot.util;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * Runs a stick calibration trace from the dashboard while the robot is Disabled.
 *
 * <p>Two toggles under /Tuning/Drive act like buttons: each resets itself to false once read.
 * "Calibrate Stick" arms a trace; the trace binds to the first drive pad whose stick leaves center,
 * records the farthest reach in every direction, and saves the rim for that pad when the driver
 * lets go. "Clear Stick Calibrations" removes every saved rim. Enabling the robot abandons a trace
 * in progress.
 */
public class StickCalibrationRunner {
  private static final String IDLE = "Idle";
  private static final String ARMED = "Push the stick to its rim in every direction, then let go";

  private final CalibratedStick[] sticks;
  private final LoggedNetworkBoolean calibrate =
      new LoggedNetworkBoolean("/Tuning/Drive/Calibrate Stick", false);
  private final LoggedNetworkBoolean clear =
      new LoggedNetworkBoolean("/Tuning/Drive/Clear Stick Calibrations", false);

  private boolean armed = false;
  private CalibratedStick bound = null;
  private StickCalibration.RimTrace trace = null;
  private String status = IDLE;

  /**
   * @param sticks the drive pads, in priority order when more than one leaves center at once
   */
  public StickCalibrationRunner(CalibratedStick... sticks) {
    this.sticks = sticks;
  }

  /** Abandons any trace in progress and resets both toggles. */
  public void disabledInit() {
    abandon();
    status = IDLE;
    calibrate.set(false);
    clear.set(false);
  }

  /** Call every loop while Disabled. */
  public void disabledPeriodic() {
    for (CalibratedStick stick : sticks) {
      stick.refresh();
    }

    if (clear.get()) {
      clear.set(false);
      abandon();
      int count = CalibratedStick.clearAll();
      for (CalibratedStick stick : sticks) {
        stick.reload();
      }
      status = "Cleared " + count + " saved calibration" + (count == 1 ? "" : "s");
    }

    if (calibrate.get()) {
      calibrate.set(false);
      abandon();
      armed = true;
      trace = new StickCalibration.RimTrace();
      status = ARMED;
    }

    if (armed && bound == null) {
      for (CalibratedStick stick : sticks) {
        if (Math.hypot(stick.rawForward(), stick.rawLeft()) > StickCalibration.RIM_CENTER) {
          bound = stick;
          break;
        }
      }
    }

    if (armed && bound != null) {
      StickCalibration.TraceState state = trace.update(bound.rawForward(), bound.rawLeft());
      if (state == StickCalibration.TraceState.DONE) {
        boolean saved = bound.saveRim(trace.result());
        status =
            (saved ? "Saved for " : "Could not save for ")
                + "port "
                + bound.port()
                + " ("
                + bound.padName()
                + ")";
        abandon();
      } else {
        status =
            "Tracing port "
                + bound.port()
                + " ("
                + bound.padName()
                + "): "
                + trace.seen()
                + " of "
                + StickCalibration.RIM_BINS
                + " directions";
      }
    }

    Logger.recordOutput("Drive/Practice/Calibration/Status", status);
    Logger.recordOutput("Drive/Practice/Calibration/Rim", rimToShow());
  }

  private double[] rimToShow() {
    if (armed && trace != null) {
      return trace.maxes();
    }
    double[] loaded = sticks.length > 0 ? sticks[0].rim() : null;
    return loaded != null ? loaded.clone() : new double[StickCalibration.RIM_BINS];
  }

  private void abandon() {
    armed = false;
    bound = null;
    trace = null;
  }
}
