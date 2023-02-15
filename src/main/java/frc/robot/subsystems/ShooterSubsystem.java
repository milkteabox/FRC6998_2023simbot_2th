// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
import static frc.robot.RobotMap.*;


public class ShooterSubsystem extends SubsystemBase
{
    private final CANSparkMax shoot_Motor = new CANSparkMax(SHOOTER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    private void configShooterMotor(){
        shoot_Motor.restoreFactoryDefaults();
        shoot_Motor.setSmartCurrentLimit(SHOOTER_CURRENTLIMIT);
        shoot_Motor.setInverted(SHOOTER_INVERTED);
        shoot_Motor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
        shoot_Motor.getEncoder().setVelocityConversionFactor(1/3);
        shoot_Motor.getPIDController().setP(SHOOTER_KP);
        shoot_Motor.getPIDController().setI(SHOOTER_KI);
        shoot_Motor.getPIDController().setD(SHOOTER_KD);
        shoot_Motor.getPIDController().setFF(SHOOTER_KF);
        shoot_Motor.burnFlash();
    }

    private void setShooterSpeed(double RPM){
        shoot_Motor.getPIDController().setReference(RPM, CANSparkMax.ControlType.kVelocity);
    }
    @Override
    public void periodic()
    {
    }
}
