package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

public class SwerveModule {

    //instance variables
    private CANCoderConfiguration _canconfig;
    private int moduleNumber;
    private TalonFX driveMotor;
    private TalonFX angleMotor;
    private Rotation2d angleOffset;
    private WPI_CANCoder angleEncoder;
    private final PIDController driveController;
    private final SimpleMotorFeedforward feedforward= new SimpleMotorFeedforward(Constants.FEEDKS, Constants.kv, Constants.ka);
    

    //constructor
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber= moduleNumber;
        angleOffset= moduleConstants.angleOffset;
        
        //angle encoder config
        _canconfig=new CANCoderConfiguration(); 
        angleEncoder= new WPI_CANCoder(moduleConstants.cancoderID,"3925");//canCoderID
        angleEncoder.configAllSettings(_canconfig);
        angleEncoder.configMagnetOffset(angleOffset.getDegrees());

        //angle motor config
        angleMotor= new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();
    }


    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
    
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;
    
        desiredState.angle = angle;
    
        double error = getState().angle.getDegrees() - desiredState.angle.getDegrees();
        double constrainedError = MathUtility.constrainAngleDegrees(error);
        double rotorOutput = rotorPID.calculate(constrainedError);
        rotorOutput = MathUtility.clamp(rotorOutput, -1, 1);
        angleMotor.set(rotorOutput);
        lastAngle = angle;
    
    
        if (isOpenLoop) {
          double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
          percentOutput = MathUtility.clamp(percentOutput, -1, 1);
          driveMotor.set(percentOutput);
        } else {
          driveController.setReference(
              desiredState.speedMetersPerSecond,
              ControlType.kVelocity,
              0,
              feedforward.calculate(desiredState.speedMetersPerSecond));
        }
      }
    
    
      private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kSensorDataOnly);
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
      }
    
      private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kVelocityOnly);
        angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        angleMotor.setInverted(Constants.Swerve.angleInvert);
        angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
        rotorPID = new PID(Constants.Swerve.angleKP, 0, Constants.Swerve.angleKD, 0, 0);
        angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
      }
    
      private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        driveMotor.setInverted(Constants.Swerve.driveInvert);
        driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
        driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
        driveController.setP(Constants.Swerve.driveKP);
        driveController.setI(Constants.Swerve.driveKI);
        driveController.setD(Constants.Swerve.driveKD);
        driveController.setFF(Constants.Swerve.driveKFF);
        driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        driveEncoder.setPosition(0.0);
      }
    
      public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
      }
    
      public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
          driveEncoder.getPosition(), 
          getAngle());
      }
    
      public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
      }

    public void configAngleMotor(){
        angleMotor.configFactoryDefault();

    }
}
