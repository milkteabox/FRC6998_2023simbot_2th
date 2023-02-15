package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.ModuleState;
import frc.lib.util.SwerveTypeConstants;

import static frc.robot.Constants.*;
import static frc.robot.RobotMap.SWERVE_CANBUS_TYPE;

public class SwerveModule {
    public int moduleNum;
    private SwerveTypeConstants swerveTypeConstants;
    private Rotation2d angleOffSet;
    private Rotation2d lastAngle;
    private TalonFX driveFalcon;
    private TalonFX angleFalcon;
    private CANCoder angleCANCoder;

    SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(SWERVE_DRIVE_KS,SWERVE_DRIVE_KV,SWERVE_DRIVE_KA);

    public SwerveModule(
            int moduleNum,
            SwerveTypeConstants swerveTypeConstants,
            int driveMotorID, int angleMotorID, int canCoderID,
            Rotation2d angleOffSet){
        this.swerveTypeConstants = swerveTypeConstants;
        this.moduleNum = moduleNum;
        this.angleOffSet = angleOffSet;

        angleCANCoder = new CANCoder(canCoderID, SWERVE_CANBUS_TYPE);
        configAngleCanCoder();

        driveFalcon = new TalonFX(driveMotorID, SWERVE_CANBUS_TYPE);
        configDriveMotor();

        angleFalcon = new TalonFX(angleMotorID, SWERVE_CANBUS_TYPE);
        configAngleMotor();

        lastAngle = getState().angle;
    }
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = ModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / SWERVE_MAX_SPEED;
            driveFalcon.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, SWERVE_WHEEL_CIRCUMFERENCE, swerveTypeConstants.driveGearRatio);
            driveFalcon.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SWERVE_MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        angleFalcon.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), swerveTypeConstants.angleGearRatio));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(angleFalcon.getSelectedSensorPosition(), swerveTypeConstants.angleGearRatio));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleCANCoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffSet.getDegrees(), swerveTypeConstants.angleGearRatio);
        angleFalcon.setSelectedSensorPosition(absolutePosition);
    }


    private void configDriveMotor(){
        //Set drive motor config.
        TalonFXConfiguration driveFalconConfiguration = new TalonFXConfiguration();
        driveFalconConfiguration.slot0.kP = SWERVE_DRIVE_MOTOR_KP;
        driveFalconConfiguration.slot0.kI = SWERVE_DRIVE_MOTOR_KI;
        driveFalconConfiguration.slot0.kD = SWERVE_DRIVE_MOTOR_KD;
        driveFalconConfiguration.slot0.kF = SWERVE_DRIVE_MOTOR_KF;
        driveFalconConfiguration.voltageCompSaturation = VOLTAGE_COMPENSATION;
        driveFalconConfiguration.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                SWERVE_DRIVE_CURRENT_LIMIT_ENABLE,
                SWERVE_DRIVE_CONTINUOUS_CURRENT_LIMIT,
                SWERVE_DRIVE_PEAK_CURRENT_LIMIT,
                SWERVE_DRIVE_PEAK_CURRENT_DURATION);
        driveFalconConfiguration.openloopRamp = SWERVE_DRIVE_MOTOR_OPENLOOPRAMP;
        driveFalconConfiguration.closedloopRamp = SWERVE_DRIVE_MOTOR_CLOSELOOPRAMP;

        driveFalcon.configFactoryDefault();
        driveFalcon.configAllSettings(driveFalconConfiguration);
        driveFalcon.setInverted(swerveTypeConstants.driveMotorInvert);
        driveFalcon.setNeutralMode(DRIVE_NEUTRAL_MODE);
        driveFalcon.setSelectedSensorPosition(0);
    }

    private void configAngleMotor(){
        TalonFXConfiguration angleFalconConfiguration = new TalonFXConfiguration();
        //Set angle motor config.
        angleFalconConfiguration.slot0.kP = swerveTypeConstants.angleKP;
        angleFalconConfiguration.slot0.kI = swerveTypeConstants.angleKI;
        angleFalconConfiguration.slot0.kD = swerveTypeConstants.angleKD;
        angleFalconConfiguration.slot0.kF = swerveTypeConstants.angleKF;
        angleFalconConfiguration.voltageCompSaturation = VOLTAGE_COMPENSATION;
        angleFalconConfiguration.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                SWERVE_ANGLE_CURRENT_LIMIT_ENABLE,
                SWERVE_ANGLE_CONTINUOUS_CURRENT_LIMIT,
                SWERVE_ANGLE_PEAK_CURRENT_LIMIT,
                SWERVE_ANGLE_PEAK_CURRENT_DURATION);

        angleFalcon.configFactoryDefault();
        angleFalcon.configAllSettings(angleFalconConfiguration);
        angleFalcon.setInverted(swerveTypeConstants.angleMotorInvert);
        angleFalcon.setNeutralMode(ANGLE_NEUTRAL_MODE);
        resetToAbsolute();
    }

    private void configAngleCanCoder(){
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfiguration.sensorDirection = swerveTypeConstants.canCoderInvert;
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;

        angleCANCoder.configFactoryDefault();
        angleCANCoder.configAllSettings(canCoderConfiguration);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
                Conversions.falconToMPS(driveFalcon.getSelectedSensorVelocity(), SWERVE_WHEEL_CIRCUMFERENCE, swerveTypeConstants.driveGearRatio),
                getAngle()
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
                Conversions.falconToMeters(driveFalcon.getSelectedSensorPosition(), SWERVE_WHEEL_CIRCUMFERENCE, swerveTypeConstants.driveGearRatio),
                getAngle()
        );
    }
}
