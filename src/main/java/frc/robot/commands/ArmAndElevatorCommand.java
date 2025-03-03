package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ElevatorTrapezoidProfile;
import frc.robot.subsystems.ElevatorTrapezoidProfile.ElevatorTrapezoidState;
import frc.robot.subsystems.Arm.ArmState;

public class ArmAndElevatorCommand extends SequentialCommandGroup {
    public ArmAndElevatorCommand(Arm arm, ArmState arm_state, ElevatorTrapezoidProfile elevator, ElevatorTrapezoidState elevator_state) {
        addCommands(
            // Önce Arm'ı istedigi pozisyonuna getir
            new ArmCommand(arm, arm_state),

            // Arm pozisyonuna geldikten sonra Elevator'ü L4'e çıkar
            new ElevatorTrapezoidCommand(elevator, elevator_state)
        );
    }
}
