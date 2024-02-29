package frc.utils;

public class LookupTable {
    private double[] angles; // in radians
    private double[] distances;

    public LookupTable(double[] angles, double[] distances) {
        this.angles = angles;
        this.distances = distances;
    }

    // Linear Interpolation (lerp) function to calculate the angle
    public double getAngleFromDistance(double distance) {
        // Find the two closest distances in the array
        int index1 = 0;
        int index2 = 1;
        for (int i = 1; i < distances.length; i++) {
            if (Math.abs(distances[i] - distance) < Math.abs(distances[index1] - distance)) {
                index2 = index1;
                index1 = i;
            } else if (Math.abs(distances[i] - distance) < Math.abs(distances[index2] - distance)) {
                index2 = i;
            }
        }
        // Linear interpolation
        double x1 = distances[index1];
        double x2 = distances[index2];
        double y1 = angles[index1];
        double y2 = angles[index2];
        // Avoid division by zero
        if (x1 == x2) return y1;
        // Calculate the interpolated angle
        else return y1 + ((distance - x1) / (x2 - x1)) * (y2 - y1);
    }
}
