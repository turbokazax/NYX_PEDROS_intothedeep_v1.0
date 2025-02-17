package yori.utils;

public class PIDConstants {
    public double KP;
    public double KI;
    public double KD;
    public double KF;

    public PIDConstants(double KP, double KI, double KD) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.KF = 0;
    }

    public PIDConstants(double KP, double KI, double KD, double KF) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.KF = KF;
    }

    public double p(){
        return KP;
    }

    public double i(){
        return KI;
    }

    public double d(){
        return KD;
    }

    public double f(){
        return KF;
    }
}
