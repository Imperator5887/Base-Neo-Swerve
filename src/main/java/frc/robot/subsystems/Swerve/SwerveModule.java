package frc.robot.subsystems.Swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax giroMotor;

    private final RelativeEncoder avanceEncoder;
    private final RelativeEncoder giroEncoder;

    private final PIDController giroPIDController;

    private final int moduleNumber;

    private final AbsoluteEncoder absoluteEncoder;
   private final SparkMaxAbsoluteEncoder.Type encodeerabstype = SparkMaxAbsoluteEncoder.Type.kDutyCycle;


    public SwerveModule(SwerveModuleConstants constants, int moduleNumber) {

        this.moduleNumber = moduleNumber;

        driveMotor = new CANSparkMax(constants.driveMotorID, MotorType.kBrushless);
        giroMotor = new CANSparkMax(constants.turnMotorID, MotorType.kBrushless);

        driveMotor.setInverted(constants.driveMotorInverted);
        giroMotor.setInverted(constants.turnMotorInverted);

        driveMotor.setIdleMode(IdleMode.kBrake);
        giroMotor.setIdleMode(IdleMode.kBrake);

        avanceEncoder = driveMotor.getEncoder();
        giroEncoder = giroMotor.getEncoder();

        absoluteEncoder = giroMotor.getAbsoluteEncoder(encodeerabstype);
        absoluteEncoder.setPositionConversionFactor(2 * Math.PI);
        absoluteEncoder.setInverted(constants.absoluteEncoderReversed);
        absoluteEncoder.setZeroOffset(constants.absoluteEncoderOffsetRad);

        giroMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 15);

        avanceEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        avanceEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        giroEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        giroEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        giroPIDController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        giroPIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getAvancePosicion() {
        return avanceEncoder.getPosition();
    }

    public double getGiroPosicion() {
        return giroEncoder.getPosition();
    }

    public double getAvanceVelocidad() {
        return avanceEncoder.getVelocity();
    }

    public double getGiroVelocidad() {
        return giroEncoder.getVelocity();
    }

    public double getAbsoluteEncoder(){
        return absoluteEncoder.getPosition();
    }

    public Rotation2d getPosicionGiro2D(){
        return Rotation2d.fromRadians(getGiroPosicion());
    }

    public void resetEncoders() {
        avanceEncoder.setPosition(0);
        giroEncoder.setPosition(getAbsoluteEncoder());
    }

    public double getPotenciaAvance(){
        return driveMotor.getAppliedOutput();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getAvanceVelocidad(), new Rotation2d(getGiroPosicion()));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getAvancePosicion(), getPosicionGiro2D());
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.005) {
            parar();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kVelocidadTeoricaMaximaMPS);
        giroMotor.set(giroPIDController.calculate(getGiroPosicion(), state.angle.getRadians()));
        SmartDashboard.putString("Modulo numero" + moduleNumber + "state", state.toString());
    }

    public void parar() {
        driveMotor.set(0);
        giroMotor.set(0);
    }
}
