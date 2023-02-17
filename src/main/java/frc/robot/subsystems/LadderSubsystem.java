// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
import static frc.robot.RobotMap.*;


public class LadderSubsystem extends SubsystemBase
{
    private final CANSparkMax ladder_Motor_LL = new CANSparkMax(LADDER_LOWER_LEFT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax ladder_Motor_LR = new CANSparkMax(LADDER_LOWER_RIGHT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax ladder_Motor_UL = new CANSparkMax(LADDER_UPPER_LEFT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax ladder_Motor_UR = new CANSparkMax(LADDER_UPPER_RIGHT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    public LadderSubsystem(){
        configUPPERMotor();
        configLowerMotor();
    }

    public void setLadder(double length){
        ladder_Motor_LL.getPIDController().setReference(length, CANSparkMax.ControlType.kPosition);
    }
//    public void setLadder(double speed){
//        ladder_Motor_LL.getPIDController().setReference(speed, CANSparkMax.ControlType.kVelocity);
//    }
    public void stopLadder(){
        ladder_Motor_LL.stopMotor();
    }

    @Override
    public void periodic()
    {
    }
    private void configLowerMotor(){
        ladder_Motor_LL.restoreFactoryDefaults();
        ladder_Motor_LL.setInverted(LADDER_LOWERLEFT_INVERTED);
        ladder_Motor_LL.setSmartCurrentLimit(LADDER_LOWER_CURRENTLIMIT);
        ladder_Motor_LL.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,LADDER_LOWER_LIMIT);
        ladder_Motor_LL.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        ladder_Motor_LL.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,true);
        ladder_Motor_LL.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,true);
        ladder_Motor_LL.enableVoltageCompensation(VOLTAGE_COMPENSATION);
        ladder_Motor_LL.setIdleMode(CANSparkMax.IdleMode.kCoast);
        ladder_Motor_LL.getEncoder().setPositionConversionFactor(1/21);
        ladder_Motor_LL.getPIDController().setP(LADDER_LOWER_KP);
        ladder_Motor_LL.getPIDController().setI(LADDER_LOWER_KI);
        ladder_Motor_LL.getPIDController().setD(LADDER_LOWER_KD);
        ladder_Motor_LL.getPIDController().setFF(LADDER_LOWER_KF);
        ladder_Motor_LL.burnFlash();

        ladder_Motor_LR.restoreFactoryDefaults();
        ladder_Motor_LR.setSmartCurrentLimit(LADDER_LOWER_CURRENTLIMIT);
        ladder_Motor_LR.setInverted(LADDER_LOWERRIGHT_INVERTED);
        ladder_Motor_LR.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,LADDER_LOWER_LIMIT);
        ladder_Motor_LR.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        ladder_Motor_LR.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,true);
        ladder_Motor_LR.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,true);
        ladder_Motor_LR.enableVoltageCompensation(VOLTAGE_COMPENSATION);
        ladder_Motor_LR.setIdleMode(CANSparkMax.IdleMode.kCoast);
        ladder_Motor_LR.getEncoder().setPositionConversionFactor(1/21);
        ladder_Motor_LR.getPIDController().setP(LADDER_LOWER_KP);
        ladder_Motor_LR.getPIDController().setI(LADDER_LOWER_KI);
        ladder_Motor_LR.getPIDController().setD(LADDER_LOWER_KD);
        ladder_Motor_LR.getPIDController().setFF(LADDER_LOWER_KF);
        ladder_Motor_LR.follow(ladder_Motor_LL, true);
        ladder_Motor_LR.burnFlash();
    }

    private void configUPPERMotor(){
        ladder_Motor_UL.restoreFactoryDefaults();
        ladder_Motor_UL.setSmartCurrentLimit(LADDER_LOWER_CURRENTLIMIT);
        ladder_Motor_UL.setInverted(LADDER_UPPERLEFT_INVERTED);
        ladder_Motor_UL.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, LADDER_UPPER_LIMIT);
        ladder_Motor_UL.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        ladder_Motor_UL.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,true);
        ladder_Motor_UL.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,true);
        ladder_Motor_UL.enableVoltageCompensation(VOLTAGE_COMPENSATION);
        ladder_Motor_UL.setIdleMode(CANSparkMax.IdleMode.kCoast);
        ladder_Motor_UL.getEncoder().setPositionConversionFactor(1/21);
        ladder_Motor_UL.getPIDController().setP(LADDER_UPPER_KP);
        ladder_Motor_UL.getPIDController().setI(LADDER_UPPER_KI);
        ladder_Motor_UL.getPIDController().setD(LADDER_UPPER_KD);
        ladder_Motor_UL.getPIDController().setFF(LADDER_UPPER_KF);
        ladder_Motor_UL.burnFlash();

        ladder_Motor_UR.restoreFactoryDefaults();
        ladder_Motor_UR.setSmartCurrentLimit(LADDER_UPPER_CURRENTLIMIT);
        ladder_Motor_UR.setInverted(LADDER_UPPERRIGHT_INVERTED);
        ladder_Motor_UR.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, LADDER_UPPER_LIMIT);
        ladder_Motor_UR.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        ladder_Motor_UR.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,true);
        ladder_Motor_UR.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,true);
        ladder_Motor_UR.enableVoltageCompensation(VOLTAGE_COMPENSATION);
        ladder_Motor_UR.setIdleMode(CANSparkMax.IdleMode.kCoast);
        ladder_Motor_UR.getEncoder().setPositionConversionFactor(1/21);
        ladder_Motor_UR.getPIDController().setP(LADDER_UPPER_KP);
        ladder_Motor_UR.getPIDController().setI(LADDER_UPPER_KI);
        ladder_Motor_UR.getPIDController().setD(LADDER_UPPER_KD);
        ladder_Motor_UR.getPIDController().setFF(LADDER_UPPER_KF);
        ladder_Motor_UR.follow(ladder_Motor_UL, true);
        ladder_Motor_UR.burnFlash();
    }
}
