package at.jku.icg.aos_dji_sdkv5.liveinfo;

/* JADX INFO: loaded from: classes.dex */
public class LiveInfo {
    private double latitude = 0.0d;
    private double longitude = 0.0d;
    private double altitude = 0.0d;
    private double compass = 0.0d;
    private double gimbalTilt = 0.0d;
    private double gimbalPan = 0.0d;
    private double gimbalYaw = 0.0d;
    private double velocity_x = 0.0d;
    private double velocity_y = 0.0d;
    private double velocity_z = 0.0d;
    private double combined_velocity = 0.0d;
    private boolean usable = false;

    public double getLatitude() {
        return this.latitude;
    }

    public void setLatitude(double d) {
        this.latitude = d;
    }

    public double getLongitude() {
        return this.longitude;
    }

    public void setLongitude(double d) {
        this.longitude = d;
    }

    public double getAltitude() {
        return this.altitude;
    }

    public void setAltitude(double d) {
        this.altitude = d;
    }

    public double getCompass() {
        return this.compass;
    }

    public void setCompass(double d) {
        this.compass = d;
    }

    public double getGimbalTilt() {
        return this.gimbalTilt;
    }

    public void setGimbalTilt(double d) {
        this.gimbalTilt = d;
    }

    public double getGimbalPan() {
        return this.gimbalPan;
    }

    public void setGimbalPan(double d) {
        this.gimbalPan = d;
    }

    public double getGimbalYaw() {
        return this.gimbalYaw;
    }

    public void setGimbalYaw(double d) {
        this.gimbalYaw = d;
    }

    public double getVelocity_x() {
        return this.velocity_x;
    }

    public void setVelocity_x(double d) {
        this.velocity_x = d;
    }

    public double getVelocity_y() {
        return this.velocity_y;
    }

    public void setVelocity_y(double d) {
        this.velocity_y = d;
    }

    public double getVelocity_z() {
        return this.velocity_z;
    }

    public void setVelocity_z(double d) {
        this.velocity_z = d;
    }

    public double getCombined_velocity() {
        return this.combined_velocity;
    }

    public void setCombined_velocity(double d) {
        this.combined_velocity = d;
    }

    public boolean getUsable() {
        return this.usable;
    }

    public void setUsable(boolean z) {
        this.usable = z;
    }
}
