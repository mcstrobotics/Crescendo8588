package frc.utils;

public class LookupTable {
    private float[] angles; // in radians
    private float[] distances;

    public LookupTable(float[] angles, float[] distances) {
        this.angles = angles;
        this.distances = distances;
    }

    // Linear Interpolation (lerp) function to calculate the angle
    public float getAngleFromDistance(float distance) {
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
        float x1 = distances[index1];
        float x2 = distances[index2];
        float y1 = angles[index1];
        float y2 = angles[index2];
        // Avoid division by zero
        if (x1 == x2) return y1;
        // Calculate the interpolated angle
        else return y1 + ((distance - x1) / (x2 - x1)) * (y2 - y1);
    }
}
