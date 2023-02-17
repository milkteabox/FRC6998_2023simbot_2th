package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LadderSubsystem;

import static frc.robot.Constants.*;


public class IntakeCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;
    private boolean rotateIN;
    private boolean rotateOUT;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, boolean rotateIN, boolean rotateOUT){
        this.intakeSubsystem = intakeSubsystem;
        this.rotateIN = rotateIN;
        this.rotateOUT = rotateOUT;
    }
    @Override
    public void execute() {
        if(rotateIN){
            intakeSubsystem.setIntakeRPM(IntakeRPM);
        }else if(rotateOUT){
            intakeSubsystem.setIntakeRPM(-IntakeRPM);
        }else {
            intakeSubsystem.stopIntake();
        }
    }
}