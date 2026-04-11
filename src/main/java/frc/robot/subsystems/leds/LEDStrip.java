package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Robot;

public class LEDStrip extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;

  public static final LEDStrip LEDSTRIP = new LEDStrip();

  public static LEDStrip getInstance() {
    return LEDSTRIP;
  }

  public LEDStrip() {
    led = new AddressableLED(LEDConstants.LED_PORT);
    ledBuffer = new AddressableLEDBuffer(LEDConstants.NUM_LEDS);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  public void periodic() {
    if (DriverStation.isDisabled()) {
      setGradient(Color.kGreen, Color.kGold);
    }
    led.setData(ledBuffer);
  }

  public void isHubActive() {
    if (Robot.isHubCurrentlyActive() && Robot.getAlliance() == Alliance.Blue) {
      setSolid(Color.kFirstBlue);
    } else if (Robot.isHubCurrentlyActive() && Robot.getAlliance() == Alliance.Red) {
    } else if (Robot.isHubCurrentlyActive() && Robot.getAlliance() == Alliance.Red) {
      setSolid(Color.kFirstRed);
    } else if (!Robot.isHubCurrentlyActive()) {
      setGradient(Color.kGreen, Color.kGold);
    }

    if (DriverStation.getMatchTime() > 30
        && DriverStation.getMatchTime() % 25 <= 5
        && Robot.getAlliance() == Alliance.Blue) {
      setBreathingWithBreathePeriod(Color.kFirstRed,1);
    } else if (DriverStation.getMatchTime() > 30
        && DriverStation.getMatchTime() % 25 <= 5
        && Robot.getAlliance() == Alliance.Red) {
      setBreathingWithBreathePeriod(Color.kFirstBlue, 1);
    }
  }

  private void setSolid(Color color) {
    LEDPattern solidPattern = LEDPattern.solid(color);
    solidPattern.applyTo(ledBuffer);
  }

  private void setGradient(Color... colors) {
    LEDPattern gradient =
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colors)
            .scrollAtRelativeSpeed(LEDConstants.SCROLL_FREQ);
    gradient.applyTo(ledBuffer);
  }

  private void setBlinking(Color color) {
    LEDPattern blinkPattern =
        LEDPattern.solid(color)
            .blink(LEDConstants.BLINK_PERIOD.div(2.0))
            .atBrightness(LEDConstants.BLINK_BRIGHTNESS);
    blinkPattern.applyTo(ledBuffer);
  }

  private void setBreathing(Color color) {
    LEDPattern breathePattern = LEDPattern.solid(color).breathe(LEDConstants.BREATHE_PERIOD);
    breathePattern.applyTo(ledBuffer);
  }

  private void setBreathingWithBreathePeriod(Color color, double frequency) {
    LEDPattern breathePattern = LEDPattern.solid(color).breathe(Seconds.of(frequency));
    breathePattern.applyTo(ledBuffer);
  }
}
