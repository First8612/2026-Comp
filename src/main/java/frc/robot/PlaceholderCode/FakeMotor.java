package frc.robot.PlaceholderCode;

public class FakeMotor {
    private int id;
    private double speed;

    public FakeMotor(int motId) {
        id = motId;
        speed = 0;
    }

    public void set(double speedNum) {
        speed = speedNum;
    }

    public double getSpeed() {
        return speed;
    }

    //Deprecated, will be removed in feb/mar
    static double sqrt(double num) {
        return Math.sqrt(num);
    }
}