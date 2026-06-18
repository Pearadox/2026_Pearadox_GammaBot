package frc.robot.subsystems.lisaintake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.util.Units;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.EnergyTracker.Compeartment;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;

public class LisaIntake {
  private TalonFXConfiguration pivotConfiguration;
  private TalonFXConfiguration rollerConfiguration;
  protected final PositionVoltage pivotPositionVoltage; 


  public static final double INTAKE_STOWED_DEGS = 60; // TODO: get degrees from pheonix tuner client
  public static final double INTAKE_DEPLOYED_DEGS = 120;
  public static final int INTAKE_MOTOR_PIVOT1_ID = 31; 
  public static final int INTAKE_MOTOR_PIVOT2_ID = 32;
  public static final int INTAKE_ROLLER1_ID = 33;
  public static final int INTAKE_ROLLER2_ID = 34;
  public static final double INTAKE_ROLLER_VOLTS= 3.0; // TODO: tune later
  public static final double OUTTAKE_ROLLER_VOLTS = -3.0;
  public static final double PIVOT_VOLTS = 3.0;

  protected final MotionMagicDutyCycle motionMagicDutyCycle;

  protected final PearadoxTalonFX pivotMotor1;
  protected final PearadoxTalonFX pivotMotor2;
  protected final PearadoxTalonFX rollerMotor1;
  protected final PearadoxTalonFX rollerMotor2;

  public LisaIntake(LisaIntake io) {
    pivotMotor1 =
        new PearadoxTalonFX(INTAKE_MOTOR_PIVOT1_ID, pivotConfiguration, Compeartment.INTAKE_PIVOT);
    pivotMotor2 =
        new PearadoxTalonFX(INTAKE_MOTOR_PIVOT2_ID, pivotConfiguration, Compeartment.INTAKE_PIVOT);
    rollerMotor1 =
        new PearadoxTalonFX(INTAKE_ROLLER1_ID, rollerConfiguration, Compeartment.INTAKE_ROLLERS);
    rollerMotor2 =
        new PearadoxTalonFX(INTAKE_ROLLER2_ID, rollerConfiguration, Compeartment.INTAKE_ROLLERS);
    pivotPositionVoltage = new PositionVoltage(0);
    motionMagicDutyCycle = new MotionMagicDutyCycle(0);
  }
  
  public void rollerForward(){
    rollerMotor1.setVoltage(INTAKE_ROLLER_VOLTS);
    rollerMotor2.setVoltage(-INTAKE_ROLLER_VOLTS);
  }

  public void rollerBackwards(){
    rollerMotor1.setVoltage(OUTTAKE_ROLLER_VOLTS);
    rollerMotor2.setVoltage(-OUTTAKE_ROLLER_VOLTS);
  }

  public void pivotUp(){
    pivotMotor1.setControl( motionMagicDutyCycle.withPosition(
            Units.degreesToRotations(INTAKE_STOWED_DEGS) * IntakeConstants.GEARING));
    pivotMotor2.setControl(motionMagicDutyCycle.withPosition(
            Units.degreesToRotations(INTAKE_STOWED_DEGS) * IntakeConstants.GEARING));
  }

  public void pivotDown(){
    pivotMotor1.setControl(motionMagicDutyCycle.withPosition(
            Units.degreesToRotations(INTAKE_DEPLOYED_DEGS) * IntakeConstants.GEARING));
    pivotMotor2.setControl(motionMagicDutyCycle.withPosition(
            Units.degreesToRotations(INTAKE_DEPLOYED_DEGS) * IntakeConstants.GEARING));
  }
  


}
