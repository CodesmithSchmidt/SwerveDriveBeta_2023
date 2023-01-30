package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.Constants.*;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final TalonFXSensorCollection driveEncoder;
    private final TalonFXSensorCollection turningEncoder;
    
    private final PIDController turningPIDController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, 
            int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderID);

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new TalonFX(turningMotorId);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getSensorCollection();
        turningEncoder = turningMotor.getSensorCollection();


        //  **************  FIX ME FOR FALCONS ***************  
     //   driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
     //   driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MetersPerSec);
     //   turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
     //   turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPIDController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    //Adjust for Falcon500s***

    public double getDrivePosition() {
        return driveEncoder.getIntegratedSensorPosition();
    }

    public double getTurningPosition(){
        return turningEncoder.getIntegratedSensorPosition();
    }

/*********************************** */

    public double getDriveVelocity() {
        return driveEncoder.getIntegratedSensorVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getIntegratedSensorVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);

    }

    public void resetEncoders() {
        driveEncoder.setIntegratedSensorPosition(0, 0);
        turningEncoder.setIntegratedSensorPosition(getAbsoluteEncoderRad(), 0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.01){
            stop();
            return;
        }
        
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(TalonFXControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(TalonFXControlMode.Position, turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard .putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());   
     

    }

    public void stop() {
        driveMotor.set(TalonFXControlMode.PercentOutput, 0);
        turningMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

}
