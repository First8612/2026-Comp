package frc.robot.PlaceholderCode;

public class FakeEncoder {
    private int id;
    private int position;

    public FakeEncoder(int encId) {
        id = encId;
        position = 0;
    }

    public void reset() {
        position = 0;
    }
    public int getPosition() {
        return position;
    }
}