package frc.lib;
import edu.wpi.first.math.interpolation.Interpolatable;

public class InterpolatingDouble implements Interpolatable<InterpolatingDouble>{
    public Double num = 0.0;

    public InterpolatingDouble(Double num) {
        this.num = num;
    }

    @Override
    public InterpolatingDouble interpolate(InterpolatingDouble other, double ratio) {
        double change = other.num - num;
        return new InterpolatingDouble((ratio * change) + num);
    }
}