package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import frc.robot.subsystems.elevator.ElevatorVisualizer;

public class RobotState {

  public static RobotState instance = new RobotState();

  public MutDistance elevatorPosition;

  public MutAngle armPosition;

  private final ElevatorVisualizer elevatorVisualizer = new ElevatorVisualizer();

  private RobotState() {

    elevatorPosition = Inches.mutable(0);

    armPosition = Degrees.mutable(0);
  }

  public void updateElevatorPosition(Distance position) {

    elevatorPosition.mut_replace(position);

    elevatorVisualizer.update(position);
  }

  public void updateArmPosition(MutAngle position) {

    armPosition.mut_replace(position);
  }
}
