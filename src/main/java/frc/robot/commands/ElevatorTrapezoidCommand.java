package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.ElevatorTrapezoidProfile;
import frc.robot.subsystems.ElevatorTrapezoidProfile.ElevatorTrapezoidState;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ElevatorTrapezoidCommand extends Command {
    private final ElevatorTrapezoidProfile m_elevator;
    private final ElevatorTrapezoidState m_state;



    public ElevatorTrapezoidCommand(ElevatorTrapezoidProfile elevator, ElevatorTrapezoidState state) {
        this.m_elevator = elevator;
        this.m_state = state;
        
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        m_elevator.setState(m_state);
        System.out.println("ElevatorTrapezoid: " + m_state);
    }
    @Override
    public void end(boolean interrupted){  
        System.out.println("Elevator Duruyor");
        m_elevator.stop();

    }

    @Override
    public boolean isFinished() {
        if (Robot.isSimulation()){
            return 
            Math.abs(m_elevator.getCurrentPosition() - m_state.getTargetHeight()) < 0.3;}
        return Math.abs(m_elevator.getCurrentPosition() - m_state.getTargetHeight()) < 3;
}}
