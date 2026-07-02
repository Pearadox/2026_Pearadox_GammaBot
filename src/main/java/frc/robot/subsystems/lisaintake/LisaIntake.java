package frc.robot.subsystems.lisaintake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.EnergyTracker.Compeartment;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;

public class LisaIntake extends SubsystemBase{
  private TalonFXConfiguration pivotConfigurationLeader;
  private TalonFXConfiguration pivotConfigurationFollower;
  private TalonFXConfiguration rollerConfiguration;
  protected final PositionVoltage pivotPositionVoltage;
  private final SingleJointedArmSim physicsSim = 
      new SingleJointedArmSim(
        DCMotor.getKrakenX44(2), 
        PIVOT_GEARING, 
        SingleJointedArmSim.estimateMOI(LEN_INTAKE_METERS, MASS_OF_INTAKE_KG), 
        LEN_INTAKE_METERS, 
        Double.NEGATIVE_INFINITY, 
        Double.POSITIVE_INFINITY, 
        false, 
        SIM_STARTING_ANGLE_RADS);
  private final TalonFXSimState pivotSimState;
  

  public static final double INTAKE_STOWED_DEGS = 60; // TODO: get degrees from pheonix tuner client
  public static final double INTAKE_DEPLOYED_DEGS = 120;
  public static final int INTAKE_MOTOR_PIVOT1_ID = 31; 
  public static final int INTAKE_MOTOR_PIVOT2_ID = 32;
  public static final int INTAKE_ROLLER1_ID = 33;
  public static final int INTAKE_ROLLER2_ID = 34;
  public static final double INTAKE_ROLLER_VOLTS= 3.0; // TODO: tune later
  public static final double OUTTAKE_ROLLER_VOLTS = -3.0;
  public static final double PIVOT_VOLTS = 3.0;
  public static final double PIVOT_GEARING =  (38.0 / 12.0) * (50.0 / 16.0) * (44.0 / 12.0);
  public static final double MASS_OF_INTAKE_KG = 11.246; 
  public static final double LEN_INTAKE_METERS = Units.inchesToMeters(15.114);
  public static final double SIM_STARTING_ANGLE_RADS = Units.degreesToRadians(0);
  
  protected final MotionMagicDutyCycle motionMagicDutyCycle;

  protected final PearadoxTalonFX pivotMotor1;
  protected final PearadoxTalonFX pivotMotor2;
  protected final PearadoxTalonFX rollerMotor1;
  protected final PearadoxTalonFX rollerMotor2;

  public LisaIntake() {
    pivotConfigurationLeader.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotConfigurationFollower.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotMotor1 =
        new PearadoxTalonFX(INTAKE_MOTOR_PIVOT1_ID, pivotConfigurationLeader, Compeartment.INTAKE_PIVOT);
    pivotMotor2 =
        new PearadoxTalonFX(INTAKE_MOTOR_PIVOT2_ID, pivotConfigurationFollower, Compeartment.INTAKE_PIVOT);
    rollerMotor1 =
        new PearadoxTalonFX(INTAKE_ROLLER1_ID, rollerConfiguration, Compeartment.INTAKE_ROLLERS);
    rollerMotor2 =
        new PearadoxTalonFX(INTAKE_ROLLER2_ID, rollerConfiguration, Compeartment.INTAKE_ROLLERS);
    pivotPositionVoltage = new PositionVoltage(0);
    motionMagicDutyCycle = new MotionMagicDutyCycle(0);
    pivotSimState = pivotMotor1.getSimState();
  }
  
  public void periodic(){
    pivotSimState.setSupplyVoltage(Constants.NOMINAL_VOLTAGE);
    
    physicsSim.setInput(pivotSimState.getMotorVoltage());
    physicsSim.update(Constants.LOOP_PERIOD);


    pivotSimState.setRawRotorPosition(
        Units.radiansToRotations(physicsSim.getAngleRads()) * PIVOT_GEARING);
    pivotSimState.setRotorVelocity(
        Units.radiansToRotations(physicsSim.getVelocityRadPerSec()) * PIVOT_GEARING);
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
