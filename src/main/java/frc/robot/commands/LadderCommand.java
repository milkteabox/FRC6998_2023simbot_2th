package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LadderSubsystem;

import static frc.robot.Constants.*;


public class LadderCommand extends CommandBase {
    private LadderSubsystem ladderSubsystem;
    private int ladderFloor = 1;
    private double error_fix = 0.0;
    public LadderCommand(LadderSubsystem ladderSubsystem, double error_plus, double eror) {
        this.ladderSubsystem = ladderSubsystem;
        if(ladderFloor==3){
            ladderFloor=1;
        } else {
            ladderFloor++;
        }
    }

    @Override
    public void execute() {
        if(ladderFloor==2){
            ladderSubsystem.setLadder(SECOND_FLOOR_Length+error_fix);
        }else if(ladderFloor==3){
            ladderSubsystem.setLadder(THIRD_FLOOR_Length+error_fix);
        }else{
            ladderSubsystem.setLadder(0);
        }
    }
}