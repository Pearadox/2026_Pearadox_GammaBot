// package frc.robot.subsystems.intake;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.util.Color8Bit;
// import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
// import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
// import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

// public class MechVisualizer {

//   private final LoggedMechanism2d mech2d = new LoggedMechanism2d(3, 3);

//   private LoggedMechanismRoot2d intakeRoot = mech2d.getRoot("Intake Root", 1.5, 0.5);
//   private LoggedMechanismLigament2d intake =
//       intakeRoot.append(
//           new LoggedMechanismLigament2d(
//               "Intake", IntakeConstants.LENGTH_METERS, 0, 3, new Color8Bit(Color.kBeige)));

//   private double pivotAngleDeg = 0.0;

//   private static final MechVisualizer instance = new MechVisualizer();

//   public static MechVisualizer getInstance() {
//     return instance;
//   }

//   private MechVisualizer() {}

//   public void periodic() {
//     SmartDashboard.putData("Intake Sim", mech2d);
//   }

//   public void updatePositionDegrees(double degrees) {
//     this.pivotAngleDeg = degrees;
//     intake.setAngle(pivotAngleDeg);
//   }
// }
