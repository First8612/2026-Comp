package frc.robot.utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class InterpolatingArrayTreeMap {
    private InterpolatingDoubleTreeMap[] treeMaps;
    private int length;

    public InterpolatingArrayTreeMap(int numValues) {
        length = Math.max(numValues,1);
        treeMaps = new InterpolatingDoubleTreeMap[length];
        for(int i = 0; i < length; i++) {
            treeMaps[i] = new InterpolatingDoubleTreeMap();
        }
    }

    public void put(double key, double[] values) {
        if(values.length != length) {
            System.out.println("Wrong array length in InterpolatingArrayTreeMap put()");
            return;
        }
        for(int i = 0; i < length; i++) {
            treeMaps[i].put(key, values[i]);
        }
    }

    public double get(double key, int index) {
        if(index < 0 || index >= length) {
            throw new IndexOutOfBoundsException("Index" + index + "out of bounds");
        }
        return treeMaps[index].get(key);
    }

    public double[] get(double key) {
        double[] values = new double[length];
        for(int i = 0; i < length; i++) {
            values[i] = treeMaps[i].get(key);
        }
        return values;
    }
} 