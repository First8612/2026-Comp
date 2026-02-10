package frc.robot.PlaceholderCode;

public class FakeSensor {
    private int id;
    private boolean value;

    public FakeSensor(int sensorId) {
        id = sensorId;
        value = false;
    }

    public void changeValue() {
        value = !value;
    }

    public boolean getValue() {
        return value;
    }
}