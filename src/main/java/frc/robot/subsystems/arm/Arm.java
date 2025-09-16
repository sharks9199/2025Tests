package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private MutAngle setpoint = Degrees.mutable(0);

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/Gains/kP", 0);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/Gains/kI", 0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/Gains/kD", 0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/Gains/kS", 0);
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/Gains/kG", 0);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/Gains/kV", 0);
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Arm/Gains/kA", 0);



  private final Distance MIN_HEIGHT = Centimeters.of(0);
  private final Distance MAX_HEIGHT = Centimeters.of(85);

  public Arm(ArmIO io) {
    this.io = io;
    io.setPID(kP.get(), kI.get(), kD.get());
  }

  @Override
  public void periodic() {
    this.io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    this.io.runSetpoint(setpoint);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setPID(kP.get(), kI.get(), kD.get()),
        kP, kI, kD);
    
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setFF(kS.get(), kG.get(), kV.get(), kA.get()), kS, kG, kV, kA);

    RobotState.instance.updateArmPosition(inputs.position);
  }

  public Command setSetpoint(Angle newSetpoint) {
    return runOnce(
        () -> {
          // 1. Converta todos os valores para um double (ex: metros)
          double targetMeters = newSetpoint.in(Radians);
          double minMeters = MIN_HEIGHT.in(Meters);
          double maxMeters = MAX_HEIGHT.in(Meters);

          // 2. Use MathUtil.clamp() para limitar o valor double
          double clampedMeters = MathUtil.clamp(targetMeters, minMeters, maxMeters);

          // 3. Converta o valor limitado de volta para um objeto Distance e atualize o setpoint
          this.setpoint.mut_replace(clampedMeters, Meters);
        });
  }
}
