// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArmAndElevatorCommand;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmManuelCommand;
import frc.robot.commands.AutonIntakeCommand;
import frc.robot.commands.BumperIntakeManuelCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.ElevatorTrapezoidCommand;
import frc.robot.commands.ElevatorEncoderCommand;
import frc.robot.commands.IntakeManuelCommand;
import frc.robot.commands.IntakePositionCommand;
import frc.robot.commands.ReefAlignCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorTrapezoidProfile;
import frc.robot.subsystems.ElevatorTrapezoidProfile.ElevatorTrapezoidState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.BumperIntake;
import frc.robot.commands.StartIntake;
import frc.robot.commands.StopIntake;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.button.POVButton;


public class RobotContainer {
    private final ElevatorTrapezoidProfile elevatortrapezoid;
    private final Intake intake;
    private final Arm arm;
    private final BumperIntake bumperIntake;
    
    private final LED m_led = new LED(2, 3, 4);

    
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband degistirildiiiiiiiii
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
;

 

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    
    private final CommandXboxController controlpanel = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser;
    

    public RobotContainer() {
        elevatortrapezoid = new ElevatorTrapezoidProfile();
        intake = new Intake(m_led);
        arm = new Arm();
        bumperIntake = new BumperIntake();
        
        SmartDashboard.putNumber("Bumper Intake Angle Speed", 0.5);


        NamedCommands.registerCommand("Intake", new AutonIntakeCommand(intake));
        NamedCommands.registerCommand("StartIntake", new StartIntake(intake));
        NamedCommands.registerCommand("StopIntake", new StopIntake(intake));


        /*new EventTrigger("notrElevator").onTrue(new ElevatorCommand(elevator, ElevatorState.NOTRELEVATOR));
        new EventTrigger("l1elevator").onTrue(new ElevatorCommand(elevator, ElevatorState.L1ELEVATOR));
        new EventTrigger("l2elevator").onTrue(new ElevatorCommand(elevator, ElevatorState.L2ELEVATOR));
        new EventTrigger("l3elevator").onTrue(new ElevatorCommand(elevator, ElevatorState.L3ELEVATOR));
        new EventTrigger("l4elevator").onTrue(new ElevatorCommand(elevator, ElevatorState.L4ELEVATOR));
        new EventTrigger("l4elevatortrapezoid").onTrue(new ElevatorTrapezoidCommand(elevatortrapezoid, ElevatorTrapezoidState.L4));
        new EventTrigger("l3elevatortrapezoid").onTrue(new ElevatorTrapezoidCommand(elevatortrapezoid, ElevatorTrapezoidState.L3));
        new EventTrigger("l2elevatortrapezoid").onTrue(new ElevatorTrapezoidCommand(elevatortrapezoid, ElevatorTrapezoidState.L2));
        new EventTrigger("l1elevatortrapezoid").onTrue(new ElevatorTrapezoidCommand(elevatortrapezoid, ElevatorTrapezoidState.NOTRL1));
        new EventTrigger("notrelevatortrapezoid").onTrue(new ElevatorTrapezoidCommand(elevatortrapezoid, ElevatorTrapezoidState.NOTRL1));*/
        new EventTrigger("L1").onTrue(new ArmAndElevatorCommand(arm, ArmState.L1_L2_L3ARM , elevatortrapezoid , ElevatorTrapezoidState.NOTRL1));
        new EventTrigger("L2").onTrue(new ArmAndElevatorCommand(arm, ArmState.L1_L2_L3ARM , elevatortrapezoid , ElevatorTrapezoidState.L2));
        new EventTrigger("L3").onTrue(new ArmAndElevatorCommand(arm, ArmState.L1_L2_L3ARM , elevatortrapezoid , ElevatorTrapezoidState.L3));
        new EventTrigger("L4").onTrue(new ArmAndElevatorCommand(arm, ArmState.L4ARM , elevatortrapezoid , ElevatorTrapezoidState.L4));
        new EventTrigger("intake").onTrue(new IntakePositionCommand(arm, ArmState.NOTRINTAKEARM , elevatortrapezoid , ElevatorTrapezoidState.NOTRL1));
        new EventTrigger("intakearm").onTrue(new ArmCommand(arm, ArmState.NOTRINTAKEARM));
        new EventTrigger("l1l2l3arm").onTrue(new ArmCommand(arm, ArmState.L1_L2_L3ARM));
        new EventTrigger("l4arm").onTrue(new ArmCommand(arm, ArmState.L4ARM));




        configureBindings();


        


        autoChooser = AutoBuilder.buildAutoChooser("intakedenemesi");
        SmartDashboard.putData("Auto Mode", autoChooser);
        
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        if (!joystick.leftTrigger().getAsBoolean()){
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getRawAxis(1)/*getLeftY()*/ * MaxSpeed*2/3) // Drive forward with negative Y (forward) (MaxSpeed / 2 yaptik yani degistiridik)
                    .withVelocityY(-joystick.getRawAxis(0)/*getLeftX() */ * MaxSpeed*2/3) // Drive left with negative X (left) (MaxSpeed / 2 yaptik yani degistiridik)
                    .withRotationalRate(-joystick.getRawAxis(2)/*getRightX()*/ * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        }



        //ELEVATOR TRAPEZOİD
        //joystick.a().toggleOnTrue(new ElevatorTrapezoidCommand(elevatortrapezoid, ElevatorTrapezoidState.NOTR));
        //joystick.b().toggleOnTrue(new ElevatorTrapezoidCommand(elevatortrapezoid, ElevatorTrapezoidState.L3));
        //joystick.x().toggleOnTrue(new ElevatorTrapezoidCommand(elevatortrapezoid, ElevatorTrapezoidState.L1));
        //joystick.y().toggleOnTrue(new ElevatorTrapezoidCommand(elevatortrapezoid, ElevatorTrapezoidState.L2));
        //L4 ve NET de atanacak


        //ARMVEELEVATORBİRLİKTE
        /*
        joystick2.povUp().toggleOnTrue(new ArmAndElevatorCommand(arm, ArmState.NETARM ,elevatortrapezoid, ElevatorTrapezoidState.L4));
        joystick2.povLeft().toggleOnTrue(new ArmAndElevatorCommand(arm, ArmState.PROCESSORARM ,elevatortrapezoid, ElevatorTrapezoidState.NOTR));
*/
        
        //joystick.povLeft().whileTrue(new FeedForwardTest(elevatortrapezoid));

        //INTAKE
        joystick.button(8).whileTrue(new IntakeManuelCommand(intake, 0.7,false));
        joystick.button(6).whileTrue(new IntakeManuelCommand(intake, 0.7,true));
        joystick.button(3).toggleOnTrue(new CoralIntakeCommand(intake));

        //Bumper Intake
        //joystick.povRight().whileTrue(new BumperIntakeManuelCommand(bumperIntake));

        joystick.povDown().onTrue(new ElevatorEncoderCommand(elevatortrapezoid, ElevatorTrapezoidState.NOTRL1));
        joystick.povLeft().onTrue(new ElevatorEncoderCommand(elevatortrapezoid, ElevatorTrapezoidState.L2));
        joystick.povUp().onTrue(new ElevatorEncoderCommand(elevatortrapezoid, ElevatorTrapezoidState.L3));
        joystick.povRight().onTrue(new ElevatorEncoderCommand(elevatortrapezoid, ElevatorTrapezoidState.L4));


        //ARM
        joystick.button(2).whileTrue(new ArmManuelCommand(arm, -0.25));
        joystick.button(4).whileTrue(new ArmManuelCommand(arm, 0.35));
        /*joystick.pov(0).onTrue(new ArmCommand(arm, Arm.POSITION_L4));
        joystick.pov(180).onTrue(new ArmCommand(arm, Arm.POSITION_INTAKE_AND_L0));
        joystick.pov(90).onTrue(new ArmCommand(arm, Arm.POSITION_L1_AND_L2));
        joystick.pov(270).onTrue(new ArmCommand(arm, Arm.POSITION_NOTR));*/


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));*/

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //joystick.leftTrigger().onTrue(); SAG DON SOLA DON YAPCAZ VB



        //CONTROL PANEL AYARLARI
        controlpanel.button(8).onTrue(new ReefAlignCommand(1, drivetrain, null));
        controlpanel.button(17).onTrue(new ReefAlignCommand(2, drivetrain, null));
        controlpanel.button(16).onTrue(new ReefAlignCommand(3, drivetrain, null));
        controlpanel.button(18).onTrue(new ReefAlignCommand(4, drivetrain, null));
        controlpanel.button(19).onTrue(new ReefAlignCommand(5, drivetrain, null));
        controlpanel.button(13).onTrue(new ReefAlignCommand(6, drivetrain, null));
        controlpanel.button(12).onTrue(new ReefAlignCommand(7, drivetrain, null));
        controlpanel.button(11).onTrue(new ReefAlignCommand(8, drivetrain, null));
        controlpanel.button(5).onTrue(new ReefAlignCommand(9, drivetrain, null));
        controlpanel.button(7).onTrue(new ReefAlignCommand(10, drivetrain, null));
        controlpanel.button(10).onTrue(new ReefAlignCommand(11, drivetrain, null));
        controlpanel.button(9).onTrue(new ReefAlignCommand(12, drivetrain, null));

        controlpanel.button(0).onTrue(new ElevatorEncoderCommand(elevatortrapezoid, ElevatorTrapezoidState.NOTRL1)/*new ArmAndElevatorCommand(arm, ArmState.L1_L2_L3ARM ,elevatortrapezoid, ElevatorTrapezoidState.NOTRL1)*/);
        controlpanel.button(14).onTrue(new ElevatorEncoderCommand(elevatortrapezoid, ElevatorTrapezoidState.L2)/*new ArmAndElevatorCommand(arm, ArmState.L1_L2_L3ARM ,elevatortrapezoid, ElevatorTrapezoidState.L2)*/);
        controlpanel.button(0).onTrue(new ElevatorEncoderCommand(elevatortrapezoid, ElevatorTrapezoidState.L3)/*new ArmAndElevatorCommand(arm, ArmState.L1_L2_L3ARM ,elevatortrapezoid, ElevatorTrapezoidState.L3)*/);
        controlpanel.button(15).onTrue(new ElevatorEncoderCommand(elevatortrapezoid, ElevatorTrapezoidState.L4)/*new ArmAndElevatorCommand(arm, ArmState.L4ARM ,elevatortrapezoid, ElevatorTrapezoidState.L4)*/);
        controlpanel.button(20).onTrue(new ElevatorEncoderCommand(elevatortrapezoid, ElevatorTrapezoidState.NOTRL1)/*new IntakePositionCommand(arm, ArmState.NOTRINTAKEARM ,elevatortrapezoid, ElevatorTrapezoidState.NOTRL1)*/);




        drivetrain.registerTelemetry(logger::telemeterize);
    }


    

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
    
    
}
