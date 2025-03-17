package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.ElevatorTrapezoidProfile;
import frc.robot.subsystems.ElevatorTrapezoidProfile.ElevatorTrapezoidState;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorEncoderCommand extends Command {
    private final ElevatorTrapezoidProfile m_elevator;
    private ElevatorTrapezoidState target;
    private double targettick;
    private double hiz;



    public ElevatorEncoderCommand(ElevatorTrapezoidProfile elevator, ElevatorTrapezoidState target) {
        this.m_elevator = elevator;
        this.target = target;
        targettick = target.getTargetHeight();
        addRequirements(elevator);
    }
    @Override
    public void initialize(){
        m_elevator.stop();
        if (targettick > m_elevator.getCurrentPosition()){
            m_elevator.calistir(0.2);
            hiz= 0.2;
          }
          else if (targettick < m_elevator.getCurrentPosition()){
          m_elevator.calistir(-0.4);
            hiz=-0.4;}
    }

    @Override
    public void execute() {
        if (Math.abs(m_elevator.getCurrentPosition()-targettick) < 3){
            m_elevator.stop();
            m_elevator.calistir(hiz/3);
        };
      
    }
    @Override
    public void end(boolean interrupted){  
        System.out.println("Elevator Duruyor");
        m_elevator.stop();
        if (targettick < -3){
        m_elevator.calistir(-0.05);}

    }

    @Override
    public boolean isFinished() {
        /*if (Robot.isSimulation()){
            return 
            Math.abs(m_elevator.getCurrentPosition() - m_state.getTargetHeight()) < 0.3;}*/
        return Math.abs(m_elevator.getCurrentPosition()-targettick) < 0.2; /*Math.abs(m_elevator.getCurrentPosition() - m_state.getTargetHeight()) < 1.0;*/
}}
