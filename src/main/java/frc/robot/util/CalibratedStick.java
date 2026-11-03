package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import org.littletonrobotics.junction.Logger;

/**
 * One drive controller's left stick with its saved rim calibration applied.
 *
 * <p>The rim is saved in WPILib Preferences per pad, keyed by joystick port and the name the driver
 * station reports for the pad, so a different pad plugged into the same port never inherits another
 * pad's rim, and the values survive a reboot. The math lives in {@link StickCalibration}.
 */
public class CalibratedStick {
  private static final String KEY_PREFIX = "StickCal_";

  private final GenericHID hid;
  private final int port;
  private String padName = null;
  private double[] rim = null;

  /**
   * @param hid the controller, for example CommandXboxController.getHID()
   * @param port the driver station port the controller is on
   */
  public CalibratedStick(GenericHID hid, int port) {
    this.hid = hid;
    this.port = port;
    refresh();
  }

  /**
   * Reloads the rim when the pad on this port changes and logs whether a rim is loaded. Cheap
   * enough to call every loop.
   */
  public void refresh() {
    String name = DriverStation.getJoystickName(port);
    if (name == null) {
      name = "";
    }
    if (!name.equals(padName)) {
      padName = name;
      load();
    }
    Logger.recordOutput("Drive/Practice/Calibration/Active" + port, rim != null);
  }

  private void load() {
    if (padName == null || padName.isEmpty()) {
      rim = null;
      return;
    }
    rim = StickCalibration.decode(Preferences.getString(key(), ""));
  }

  private String key() {
    return KEY_PREFIX + "p" + port + "_" + padName.replaceAll("[^A-Za-z0-9 _-]", "");
  }

  /** Raw stick forward, -leftY. */
  public double rawForward() {
    return -hid.getRawAxis(XboxController.Axis.kLeftY.value);
  }

  /** Raw stick left, -leftX. */
  public double rawLeft() {
    return -hid.getRawAxis(XboxController.Axis.kLeftX.value);
  }

  /**
   * The calibrated (forward, left) pair for this loop: the raw stick through {@link
   * StickCalibration#apply} with the loaded rim. Both axes come from the same driver station packet
   * within one loop, so calling this once per axis is consistent.
   *
   * @return {forward, left}
   */
  public double[] read() {
    refresh();
    double forward = rawForward();
    double left = rawLeft();
    Logger.recordOutput("Drive/Practice/StickMagnitudeRaw", Math.hypot(forward, left));
    return StickCalibration.apply(rim, forward, left);
  }

  /**
   * Saves a measured rim for the pad on this port and reloads it.
   *
   * @param newRim the measured rim
   * @return false if there is no pad on the port or the rim is not valid
   */
  public boolean saveRim(double[] newRim) {
    refresh();
    double[] clean = StickCalibration.sanitize(newRim);
    if (padName.isEmpty() || clean == null) {
      return false;
    }
    Preferences.setString(key(), StickCalibration.encode(clean));
    load();
    return rim != null;
  }

  /** Re-reads the saved rim for the current pad, for example after the saved ones are cleared. */
  public void reload() {
    padName = null;
    refresh();
  }

  /** The loaded rim, or null when none is saved for this pad. */
  public double[] rim() {
    return rim;
  }

  /** The driver station port. */
  public int port() {
    return port;
  }

  /** The name the driver station reports for the pad on this port, empty when none. */
  public String padName() {
    return padName == null ? "" : padName;
  }

  /**
   * Removes every saved stick calibration.
   *
   * @return how many were removed
   */
  public static int clearAll() {
    int count = 0;
    for (String key : Preferences.getKeys()) {
      if (key.startsWith(KEY_PREFIX)) {
        Preferences.remove(key);
        count++;
      }
    }
    return count;
  }
}
