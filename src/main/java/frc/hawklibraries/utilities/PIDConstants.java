package frc.hawklibraries.utilities;

public class PIDConstants {
    
    private double kP;
    private double kI;
    private double kD;
    private double kFF;
    private double kOutputMin = -1;
    private double kOutputMax = 1;

    public PIDConstants(double kP) {
        this.kP = kP;
    }

    public PIDConstants(double kP, double kI) {
        this.kP = kP;
        this.kI = kI;
    }

    public PIDConstants(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public PIDConstants(double kP, double kI, double kD, double kFF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kFF = kFF;
    }

    public PIDConstants(double kP, double kI, double kD, double kFF, double kOutputMin, double kOutputMax) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kFF = kFF;
        this.kOutputMin = kOutputMin;
        this.kOutputMax = kOutputMax;
    }

    public double getkP() {return this.kP;}
    public double getkI() {return this.kI;}
    public double getkD() {return this.kD;}
    public double getkFF() {return this.kFF;}
    public double getkOutputMin() {return this.kOutputMin;}
    public double getkOutputMax() {return this.kOutputMax;}
    public double[] getAsArray() {return new double[] {
        this.kP,
        this.kI,
        this.kD,
        this.kFF
    };}

}