package yori.utils;

public class LowVoltageAdapter {
    private static double SOME_RANDOM_MAGIC_COEFFICIENT = 1.5;
    public static double adapt(double value, double voltage){
        return value * 12.0 / voltage * SOME_RANDOM_MAGIC_COEFFICIENT;
    }

    public void updateMagicCoefficient(double SOME_RANDOM_MAGIC_COEFFICIENT){
        LowVoltageAdapter.SOME_RANDOM_MAGIC_COEFFICIENT = SOME_RANDOM_MAGIC_COEFFICIENT;
    }
}
