package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class ElevatorVisualizer {

  private final Mechanism2d panel;
  private final MechanismRoot2d root;
  private final MechanismLigament2d elevator;

  public ElevatorVisualizer() {
    this.panel = new Mechanism2d(Inches.of(100).in(Meters), Inches.of(100).in(Meters));
    this.root = panel.getRoot("elevator", Inches.of(7).in(Meters), Inches.of(7).in(Meters));
    this.elevator =
        root.append(
            new MechanismLigament2d(
                "Elevator", Inches.of(0).in(Meters), 90, 10, new Color8Bit(Color.kGreen)));
  }

  public void update(Distance position) {
    // Apenas atualize o comprimento do ligamento. A visualização se atualiza sozinha.
    elevator.setLength(position.in(Meters));
    SmartDashboard.putData("Elevator/Mechanism2d", this.panel);

    // Crie o objeto Pose3d para a visualização no campo 3D
    Pose3d elevator3d =
        new Pose3d(
            0, // Posição X é 0
            0, // Posição Y é 0
            position.in(Meters), // Posição Z (altura) é a posição do elevador
            new Rotation3d() // Rotação é 0
            );

    // CORRETO: Publique o objeto 'elevator3d' (Pose3d) para a chave do Mechanism3d.
    Logger.recordOutput("Elevator/Mechanism3d", elevator3d);
  }
}
