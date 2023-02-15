// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants
{
    public static final Rotation2d SWERVE_LEFTFRONT_OFFSET = Rotation2d.fromDegrees(227.374047);
    public static final Rotation2d SWERVE_LEFTREAR_OFFSET = Rotation2d.fromDegrees(11.601563);
    public static final Rotation2d SWERVE_RIGHTFRONT_OFFSET = Rotation2d.fromDegrees(71.806641);
    public static final Rotation2d SWERVE_RIGHTREAR_OFFSET = Rotation2d.fromDegrees(175.605469);

    public static final double SWERVE_CHASSIS_TRACKWIDTH_METERS = 0.62865;
    public static final double SWERVE_CHASSIS_WHEELBASE_METERS = 0.62865;
    public static final double SWERVE_WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI;
    public static final double MAX_VOTAGE = 12.0;
    public static final  double SWERVE_MAX_SPEED = 4.5;//Wait for test.
    public static final  double SWERVE_MAX_ANGULAR_VELOCITY = 4.5;//Wait for test.


    public static final double VOLTAGE_COMPENSATION = 12.0;
    //Swerve angle falcon current limit.
    public static final int SWERVE_ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
    public static final int SWERVE_ANGLE_PEAK_CURRENT_LIMIT = 40;
    public static final double SWERVE_ANGLE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean SWERVE_ANGLE_CURRENT_LIMIT_ENABLE = true;

    public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;

    //Swerve drive falcon current limit.
    public static final int SWERVE_DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
    public static final int SWERVE_DRIVE_PEAK_CURRENT_LIMIT = 60;
    public static final double SWERVE_DRIVE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean SWERVE_DRIVE_CURRENT_LIMIT_ENABLE = true;

    public static final double SWERVE_DRIVE_MOTOR_KP = 0.05;//Wait to test.
    public static final double SWERVE_DRIVE_MOTOR_KI = 0.0;
    public static final double SWERVE_DRIVE_MOTOR_KD = 0.0;
    public static final double SWERVE_DRIVE_MOTOR_KF = 0.0;
    public static final double SWERVE_DRIVE_MOTOR_OPENLOOPRAMP = 0.25;
    public static final double SWERVE_DRIVE_MOTOR_CLOSELOOPRAMP = 0.0;
    public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

    public static final double SWERVE_DRIVE_KS = (0.32 / MAX_VOTAGE);//Wait to test by SYSID.
    public static final double SWERVE_DRIVE_KV = (1.51 / MAX_VOTAGE);
    public static final double SWERVE_DRIVE_KA = (0.27 / MAX_VOTAGE);

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */

    public static final boolean NAVX_INVERTED = true;

    public static final double SWERVE_DRIVE_JOYSTICK_DEADBAND = 0.05;

    public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(SWERVE_CHASSIS_TRACKWIDTH_METERS / 2.0,SWERVE_CHASSIS_WHEELBASE_METERS / 2.0),
            new Translation2d(-SWERVE_CHASSIS_TRACKWIDTH_METERS / 2.0,SWERVE_CHASSIS_WHEELBASE_METERS / 2.0),
            new Translation2d(SWERVE_CHASSIS_TRACKWIDTH_METERS / 2.0,-SWERVE_CHASSIS_WHEELBASE_METERS / 2.0),
            new Translation2d(-SWERVE_CHASSIS_TRACKWIDTH_METERS / 2.0,-SWERVE_CHASSIS_WHEELBASE_METERS / 2.0)
    );

    public static final double SWERVE_AUTO_XY_KP = 5.0;
    public static final double SWERVE_AUTO_XY_KI = 0.0;
    public static final double SWERVE_AUTO_XY_KD = 0.0;

    public static final double SWERVE_AUTO_Z_KP = 0.0;
    public static final double SWERVE_AUTO_Z_KI = 0.0;
    public static final double SWERVE_AUTO_Z_KD = 0.0;

    public static final double SECOND_FLOOR_Length = 10;
    public static final double THIRD_FLOOR_Length = 10;

    public static final boolean LADDER_LOWERLEFT_INVERTED = false;
    public static final boolean LADDER_LOWERRIGHT_INVERTED = false;
    public static final int LADDER_LOWER_CURRENTLIMIT = 35;
    public static final double LADDER_LOWER_KP = 0.0;
    public static final double LADDER_LOWER_KI = 0.0;
    public static final double LADDER_LOWER_KD = 0.0;
    public static final double LADDER_LOWER_KF = 0.0;

    public static final float LADDER_LOWER_LIMIT = 100;


    public static final boolean LADDER_UPPERLEFT_INVERTED = false;
    public static final boolean LADDER_UPPERRIGHT_INVERTED = false;
    public static final int LADDER_UPPER_CURRENTLIMIT = 35;
    public static final double LADDER_UPPER_KP = 0.0;
    public static final double LADDER_UPPER_KI = 0.0;
    public static final double LADDER_UPPER_KD = 0.0;
    public static final double LADDER_UPPER_KF = 0.0;

    public static final float LADDER_UPPER_LIMIT = 100;

    public static final boolean INTAKE_INVERTED = false;
    public static final int INTAKE_CURRENTLIMIT = 35;
    public static final double INTAKE_KP = 0.0;
    public static final double INTAKE_KI = 0.0;
    public static final double INTAKE_KD = 0.0;
    public static final double INTAKE_KF = 0.0;

    public static final boolean SHOOTER_INVERTED = false;
    public static final int SHOOTER_CURRENTLIMIT = 35;
    public static final double SHOOTER_KP = 0.0;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.0;
    public static final double SHOOTER_KF = 0.0;

    public static final double SHOOT_FIRST_RPM = 1000;
    public static final double SHOOT_INTAKE_FIRST_RPM = 1000;

    public static final double SHOOT_SECOND_RPM = 1000;
    public static final double SHOOT_INTAKE_SECOND_RPM = 1000;

    public static final double SHOOT_THIRD_RPM = 2000;
    public static final double SHOOT_INTAKE_THIRD_RPM = 2000;

    public static final boolean INTAKE_ANGLE_L_INVERTED = false;
    public static final boolean INTAKE_ANGLE_R_INVERTED = false;
    public static final int INTAKE_ANGLE_CURRENTLIMIT = 35;
    public static final float INTAKE_ANGLE_UP_LIMIT = 0;
    public static final float INTAKE_ANGLE_DOWN_LIMIT = 0;
    public static final double INTAKE_ANGLE_KP = 0.0;
    public static final double INTAKE_ANGLE_KI = 0.0;
    public static final double INTAKE_ANGLE_KD = 0.0;
    public static final double INTAKE_ANGLE_KF = 0.0;
}
