package frc.hawklibraries.vendorRewrites.wpilib;

import edu.wpi.first.math.MathUtil;
import frc.hawklibraries.utilities.PIDConstants;
import frc.hawklibraries.vendorRewrites.rev.CANSparkMax;
import frc.hawklibraries.vendorRewrites.rev.CANSparkMax.SetpointType;

public class PIDController extends edu.wpi.first.math.controller.PIDController {

    private PIDConstants pid;

    private double outputMin = -1;
    private double outputMax = 1;

    private double posWrapMin;
    private double posWrapMax;

    private CANSparkMax sparkMax;
    private SetpointType setpointType;

    public PIDController(PIDConstants pid) {
        super(pid.getkP(), pid.getkI(), pid.getkD());
        this.pid = pid;
    }

    public PIDController withPositionWrapping(double min, double max) {
        this.posWrapMin = min;
        this.posWrapMax = max;
        this.enableContinuousInput(this.posWrapMin, this.posWrapMax);
        return this;
    }
    
    public void applyToSparkMax(CANSparkMax sparkMax, SetpointType setpointType) {
        this.sparkMax = sparkMax;
        this.sparkMax.setPID(this.pid);
        if (this.isContinuousInputEnabled()) {
            this.sparkMax.enablePositionWrapping(posWrapMin, posWrapMax);
        } else {
            this.sparkMax.disablePositionWrapping();
        }
        this.sparkMax.setOutputRange(this.pid.getkOutputMin(), this.pid.getkOutputMax());
    }

    @Override
    public void setSetpoint(double setpoint) {
        super.setSetpoint(setpoint);
        if (this.sparkMax != null) {
            this.sparkMax.setSetpoint(setpoint, setpointType);
        }
    }

    @Override
    public boolean atSetpoint() {
        if (this.sparkMax != null) {
            if (this.setpointType == SetpointType.kPosition) {
                this.calculate(this.sparkMax.getPosition());
            }
            if (this.setpointType == SetpointType.kVelocity) {
                this.calculate(this.sparkMax.getVelocity());
            }
        }
        return super.atSetpoint();
    }

    @Override
    public double calculate(double measurement) {
        return MathUtil.clamp(
            super.calculate(measurement),
            outputMin,
            outputMax
        );
    }

    @Override
    public double calculate(double measurement,double setpoint) {
        return MathUtil.clamp(
            super.calculate(measurement, setpoint),
            outputMin,
            outputMax
        );
    }


}