package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ElevatorTrapezoidProfile;
import frc.robot.subsystems.ElevatorTrapezoidProfile.ElevatorTrapezoidState;
import frc.robot.subsystems.Arm.ArmState;
//Sequential da olabilir
public class IntakePositionCommand extends SequentialCommandGroup {
    public IntakePositionCommand(Arm arm, ArmState arm_state, ElevatorTrapezoidProfile elevator, ElevatorTrapezoidState elevator_state) {
        addCommands(
            
        new ElevatorTrapezoidCommand(elevator, elevator_state),


            new ArmCommand(arm, arm_state)
        );
    }
}
