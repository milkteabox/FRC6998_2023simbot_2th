// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
import static frc.robot.RobotMap.*;

public class IntakeSubsystem extends SubsystemBase
{
    private final CANSparkMax intake_Motor = new CANSparkMax(INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax intake_Angle_L_Motor = new CANSparkMax(INTAKE_ANGLE_L, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax intake_Angle_R_Motor = new CANSparkMax(INTAKE_ANGLE_R, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final DutyCycleEncoder intake_Angle_L_Encoder = new DutyCycleEncoder(INTAKE_ANGLE_ENCODER_L);
    private final DutyCycleEncoder intake_Angle_R_Encoder = new DutyCycleEncoder(INTAKE_ANGLE_ENCODER_R);

    public IntakeSubsystem(){
        configIntakeMotor();
        configAngleMotor();
        Timer.delay(0.5);
        intake_Angle_L_Motor.getEncoder().setPosition(intake_Angle_L_Encoder.getAbsolutePosition()*360);
        intake_Angle_R_Motor.getEncoder().setPosition(intake_Angle_L_Encoder.getAbsolutePosition()*360);
    }
    public void intakeUP(boolean UP){
        if(UP){
            intake_Angle_L_Motor.set(1);
        }else {
            intake_Angle_L_Motor.stopMotor();
        }
    }
    public void setIntakeRPM(double RPM){
        intake_Motor.getPIDController().setReference(RPM, CANSparkMax.ControlType.kVelocity);
    }
    public void stopIntake(){
        intake_Motor.stopMotor();
    }
    public void setIntakeAngle(double degree){
        intake_Angle_L_Motor.getPIDController().setReference(degree, CANSparkMax.ControlType.kPosition);
    }
    private void configIntakeMotor(){
        intake_Motor.restoreFactoryDefaults();
        intake_Motor.setSmartCurrentLimit(INTAKE_CURRENTLIMIT);
        intake_Motor.setInverted(INTAKE_INVERTED);
        intake_Motor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
        intake_Motor.getEncoder().setVelocityConversionFactor(1/3);
        intake_Motor.getPIDController().setP(INTAKE_KP);
        intake_Motor.getPIDController().setI(INTAKE_KI);
        intake_Motor.getPIDController().setD(INTAKE_KD);
        intake_Motor.getPIDController().setFF(INTAKE_KF);
        intake_Motor.burnFlash();
    }
    private void configAngleMotor(){
        intake_Angle_L_Motor.restoreFactoryDefaults();
        intake_Angle_L_Motor.setInverted(INTAKE_ANGLE_L_INVERTED);
        intake_Angle_L_Motor.setSmartCurrentLimit(INTAKE_ANGLE_CURRENTLIMIT);
        intake_Angle_L_Motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,INTAKE_ANGLE_UP_LIMIT);
        intake_Angle_L_Motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, INTAKE_ANGLE_DOWN_LIMIT);
        intake_Angle_L_Motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,true);
        intake_Angle_L_Motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,true);
        intake_Angle_L_Motor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
        intake_Angle_L_Motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intake_Angle_L_Motor.getEncoder().setPositionConversionFactor(360/30);
        intake_Angle_L_Motor.getPIDController().setP(INTAKE_ANGLE_KP);
        intake_Angle_L_Motor.getPIDController().setI(INTAKE_ANGLE_KI);
        intake_Angle_L_Motor.getPIDController().setD(INTAKE_ANGLE_KD);
        intake_Angle_L_Motor.getPIDController().setFF(INTAKE_ANGLE_KF);
        intake_Angle_L_Motor.burnFlash();

        intake_Angle_R_Motor.restoreFactoryDefaults();
        intake_Angle_R_Motor.setInverted(INTAKE_ANGLE_R_INVERTED);
        intake_Angle_R_Motor.setSmartCurrentLimit(INTAKE_ANGLE_CURRENTLIMIT);
        intake_Angle_R_Motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,INTAKE_ANGLE_UP_LIMIT);
        intake_Angle_R_Motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, INTAKE_ANGLE_DOWN_LIMIT);
        intake_Angle_R_Motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,true);
        intake_Angle_R_Motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,true);
        intake_Angle_R_Motor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
        intake_Angle_R_Motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intake_Angle_R_Motor.getEncoder().setPositionConversionFactor(360/30);
        intake_Angle_R_Motor.getPIDController().setP(INTAKE_ANGLE_KP);
        intake_Angle_R_Motor.getPIDController().setI(INTAKE_ANGLE_KI);
        intake_Angle_R_Motor.getPIDController().setD(INTAKE_ANGLE_KD);
        intake_Angle_R_Motor.getPIDController().setFF(INTAKE_ANGLE_KF);
        intake_Angle_R_Motor.follow(intake_Angle_L_Motor, true);
        intake_Angle_R_Motor.burnFlash();
    }

    @Override
    public void periodic()
    {
    }
}
