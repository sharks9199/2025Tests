package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private MutDistance setpoint = Inches.mutable(0);

  private final Distance MIN_HEIGHT = Centimeters.of(0);
  private final Distance MAX_HEIGHT = Centimeters.of(85);

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    this.io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    this.io.runSetpoint(setpoint);
    RobotState.instance.updateElevatorPosition(inputs.position);
  }

  public Command setSetpoint(Distance newSetpoint) {
    return runOnce(
        () -> {
          // 1. Converta todos os valores para um double (ex: metros)
          double targetMeters = newSetpoint.in(Meters);
          double minMeters = MIN_HEIGHT.in(Meters);
          double maxMeters = MAX_HEIGHT.in(Meters);

          // 2. Use MathUtil.clamp() para limitar o valor double
          double clampedMeters = MathUtil.clamp(targetMeters, minMeters, maxMeters);

          // 3. Converta o valor limitado de volta para um objeto Distance e atualize o setpoint
          this.setpoint.mut_replace(clampedMeters, Meters);
        });
  }
}
