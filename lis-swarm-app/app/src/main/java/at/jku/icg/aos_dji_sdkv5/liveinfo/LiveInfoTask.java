package at.jku.icg.aos_dji_sdkv5.liveinfo;

import at.jku.icg.aos_dji_sdkv5.core.AOSActivity;
import at.jku.icg.aos_dji_sdkv5.core.AOSManager;
import dji.sdk.keyvalue.key.FlightControllerKey;
import dji.sdk.keyvalue.key.GimbalKey;
import dji.sdk.keyvalue.key.KeyTools;
import dji.sdk.keyvalue.value.common.Attitude;
import dji.sdk.keyvalue.value.common.LocationCoordinate3D;
import dji.sdk.keyvalue.value.common.Velocity3D;
import dji.v5.common.callback.CommonCallbacks;
import dji.v5.common.error.IDJIError;
import dji.v5.manager.KeyManager;
import dji.v5.manager.aircraft.rtk.RTKCenter;
import dji.v5.manager.aircraft.rtk.RTKLocationInfo;
import dji.v5.manager.aircraft.rtk.RTKLocationInfoListener;
import dji.v5.utils.common.ContextUtil;
import dji.v5.utils.common.DiskUtil;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.TimerTask;

/* JADX INFO: loaded from: classes.dex */
public class LiveInfoTask extends TimerTask {
    private final AOSManager aosManager;
    private static String PATH = "/DJI_ScreenShot";
    private static String folder_path = DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), PATH);
    public static final File baseDir = new File(folder_path);
    private boolean simulation_mode = false;
    private boolean parameter_selection_mode = false;
    private Attitude aircraft_attitude1 = null;
    private List<Double> latitude_list = new ArrayList();
    private List<Double> longitude_list = new ArrayList();
    private List<Double> altitude_list = new ArrayList();
    private List<Double> compass_list = new ArrayList();
    private int location_list_count = 0;
    private LocationCoordinate3D curr_loc = null;
    private LocationCoordinate3D curr_loc1 = null;
    private Attitude gimbal_attitude = null;
    private Velocity3D curr_velocity = null;
    private Double curr_compass = Double.valueOf(0.0d);
    private Boolean location_received = false;
    private Boolean compass_received = false;
    private Boolean rtk_mode = false;
    private RTKLocationInfo rtk_location_info = new RTKLocationInfo();
    private double dummy_lat = 47.2234087d;
    private double dummy_lon = 14.1234087d;
    private RTKLocationInfoListener live_rtkLocationInfoListener = new RTKLocationInfoListener() { // from class: at.jku.icg.aos_dji_sdkv5.liveinfo.LiveInfoTask.1
        /* JADX DEBUG: Method merged with bridge method: onUpdate(Ljava/lang/Object;)V */
        public void onUpdate(RTKLocationInfo rTKLocationInfo) {
            if (rTKLocationInfo != null) {
                if (rTKLocationInfo.getReal3DLocation() != null && LiveInfoTask.this.rtk_mode.booleanValue()) {
                    LiveInfoTask.this.curr_loc = rTKLocationInfo.getRtkLocation().getMobileStationLocation();
                    LiveInfoTask.this.curr_loc1 = rTKLocationInfo.getReal3DLocation();
                    LiveInfoTask.this.location_received = true;
                }
                if (rTKLocationInfo.getRealHeading() == null || !LiveInfoTask.this.rtk_mode.booleanValue()) {
                    return;
                }
                KeyManager.getInstance().getValue(KeyTools.createKey(FlightControllerKey.KeyAircraftAttitude), new CommonCallbacks.CompletionCallbackWithParam<Attitude>() { // from class: at.jku.icg.aos_dji_sdkv5.liveinfo.LiveInfoTask.1.1
                    public void onFailure(IDJIError iDJIError) {
                    }

                    /* JADX DEBUG: Method merged with bridge method: onSuccess(Ljava/lang/Object;)V */
                    public void onSuccess(Attitude attitude) {
                        LiveInfoTask.this.aircraft_attitude1 = attitude;
                        LiveInfoTask.this.curr_compass = LiveInfoTask.this.aircraft_attitude1.getYaw();
                        LiveInfoTask.this.compass_received = true;
                    }
                });
            }
        }
    };
    private final List<LiveInfoListener> listeners = new ArrayList();

    public LiveInfoTask(AOSManager aOSManager) {
        this.aosManager = aOSManager;
    }

    public void Read_location_info() throws IOException {
        BufferedReader bufferedReader = new BufferedReader(new FileReader(new File(AOSActivity.baseDir, "latitude_list.txt")));
        while (true) {
            String line = bufferedReader.readLine();
            if (line == null) {
                break;
            } else {
                this.latitude_list.add(Double.valueOf(line));
            }
        }
        bufferedReader.close();
        BufferedReader bufferedReader2 = new BufferedReader(new FileReader(new File(AOSActivity.baseDir, "longitude_list.txt")));
        while (true) {
            String line2 = bufferedReader2.readLine();
            if (line2 == null) {
                break;
            } else {
                this.longitude_list.add(Double.valueOf(line2));
            }
        }
        bufferedReader2.close();
        BufferedReader bufferedReader3 = new BufferedReader(new FileReader(new File(AOSActivity.baseDir, "altitude_list.txt")));
        while (true) {
            String line3 = bufferedReader3.readLine();
            if (line3 == null) {
                break;
            } else {
                this.altitude_list.add(Double.valueOf(line3));
            }
        }
        bufferedReader3.close();
        BufferedReader bufferedReader4 = new BufferedReader(new FileReader(new File(AOSActivity.baseDir, "compass_list.txt")));
        while (true) {
            String line4 = bufferedReader4.readLine();
            if (line4 != null) {
                this.compass_list.add(Double.valueOf(line4));
            } else {
                bufferedReader4.close();
                return;
            }
        }
    }

    @Override // java.util.TimerTask, java.lang.Runnable
    public void run() {
        double dDoubleValue;
        double dDoubleValue2;
        double dDoubleValue3;
        double d;
        double d2;
        double d3;
        double d4;
        double d5;
        double d6;
        double dDoubleValue4;
        double dDoubleValue5;
        double dDoubleValue6;
        double dDoubleValue7;
        double dDoubleValue8;
        double dDoubleValue9;
        if (this.aosManager.isRunning()) {
            this.location_received = false;
            this.compass_received = false;
            LiveInfo liveInfo = new LiveInfo();
            if (!this.rtk_mode.booleanValue()) {
                KeyManager.getInstance().getValue(KeyTools.createKey(FlightControllerKey.KeyAircraftLocation3D), new CommonCallbacks.CompletionCallbackWithParam<LocationCoordinate3D>() { // from class: at.jku.icg.aos_dji_sdkv5.liveinfo.LiveInfoTask.2
                    /* JADX DEBUG: Method merged with bridge method: onSuccess(Ljava/lang/Object;)V */
                    public void onSuccess(LocationCoordinate3D locationCoordinate3D) {
                        LiveInfoTask.this.curr_loc = locationCoordinate3D;
                        LiveInfoTask.this.location_received = true;
                    }

                    public void onFailure(IDJIError iDJIError) {
                        LiveInfoTask.this.location_received = true;
                    }
                });
                KeyManager.getInstance().getValue(KeyTools.createKey(FlightControllerKey.KeyCompassHeading), new CommonCallbacks.CompletionCallbackWithParam<Double>() { // from class: at.jku.icg.aos_dji_sdkv5.liveinfo.LiveInfoTask.3
                    /* JADX DEBUG: Method merged with bridge method: onSuccess(Ljava/lang/Object;)V */
                    public void onSuccess(Double d7) {
                        LiveInfoTask.this.curr_compass = d7;
                        LiveInfoTask.this.compass_received = true;
                    }

                    public void onFailure(IDJIError iDJIError) {
                        LiveInfoTask.this.compass_received = true;
                    }
                });
            }
            KeyManager.getInstance().getValue(KeyTools.createKey(GimbalKey.KeyGimbalAttitude), new CommonCallbacks.CompletionCallbackWithParam<Attitude>() { // from class: at.jku.icg.aos_dji_sdkv5.liveinfo.LiveInfoTask.4
                public void onFailure(IDJIError iDJIError) {
                }

                /* JADX DEBUG: Method merged with bridge method: onSuccess(Ljava/lang/Object;)V */
                public void onSuccess(Attitude attitude) {
                    LiveInfoTask.this.gimbal_attitude = attitude;
                }
            });
            double dSqrt = Math.sqrt(0.0d);
            LocationCoordinate3D locationCoordinate3D = this.curr_loc;
            if (locationCoordinate3D != null) {
                dDoubleValue = locationCoordinate3D.getLatitude().doubleValue();
                dDoubleValue2 = this.curr_loc.getLongitude().doubleValue();
                if (this.rtk_mode.booleanValue()) {
                    dDoubleValue3 = this.curr_loc1.getAltitude().doubleValue();
                } else {
                    dDoubleValue3 = this.curr_loc.getAltitude().doubleValue();
                }
                Attitude attitude = this.gimbal_attitude;
                if (attitude != null) {
                    dDoubleValue4 = attitude.getPitch().doubleValue();
                    dDoubleValue5 = this.gimbal_attitude.getRoll().doubleValue();
                    dDoubleValue6 = this.gimbal_attitude.getYaw().doubleValue();
                } else {
                    dDoubleValue4 = 0.0d;
                    dDoubleValue5 = 0.0d;
                    dDoubleValue6 = 0.0d;
                }
                Velocity3D velocity3D = this.curr_velocity;
                if (velocity3D != null) {
                    dDoubleValue7 = velocity3D.getX().doubleValue();
                    dDoubleValue8 = this.curr_velocity.getY().doubleValue();
                    dDoubleValue9 = this.curr_velocity.getZ().doubleValue();
                } else {
                    dDoubleValue7 = 0.0d;
                    dDoubleValue8 = 0.0d;
                    dDoubleValue9 = 0.0d;
                }
                liveInfo.setUsable(true);
                d = dDoubleValue4;
                d2 = dDoubleValue5;
                d3 = dDoubleValue6;
                d4 = dDoubleValue7;
                d5 = dDoubleValue8;
                d6 = dDoubleValue9;
            } else {
                dDoubleValue = this.dummy_lat + 9.999999747378752E-5d;
                this.dummy_lat = dDoubleValue;
                dDoubleValue2 = 9.999999747378752E-5d + this.dummy_lon;
                this.dummy_lon = dDoubleValue2;
                dDoubleValue3 = 30.0d;
                liveInfo.setUsable(true);
                d = 0.0d;
                d2 = 0.0d;
                d3 = 0.0d;
                d4 = 0.0d;
                d5 = 0.0d;
                d6 = 0.0d;
            }
            if (this.simulation_mode) {
                double dDoubleValue10 = this.latitude_list.get(this.location_list_count).doubleValue();
                double dDoubleValue11 = this.longitude_list.get(this.location_list_count).doubleValue();
                double dDoubleValue12 = this.altitude_list.get(this.location_list_count).doubleValue();
                this.curr_compass = this.compass_list.get(this.location_list_count);
                if (this.parameter_selection_mode) {
                    this.location_list_count = this.location_list_count;
                } else {
                    this.location_list_count++;
                }
                liveInfo.setLatitude(dDoubleValue10);
                liveInfo.setLongitude(dDoubleValue11);
                liveInfo.setAltitude(dDoubleValue12);
                liveInfo.setCompass(this.curr_compass.doubleValue());
                liveInfo.setVelocity_x(0.0d);
                liveInfo.setVelocity_y(0.0d);
                liveInfo.setVelocity_z(0.0d);
                liveInfo.setGimbalTilt(d);
                liveInfo.setGimbalPan(d2);
                liveInfo.setGimbalYaw(d3);
                liveInfo.setCombined_velocity(1.0d);
                liveInfo.setGimbalTilt(0.0d);
                liveInfo.setUsable(true);
            } else {
                liveInfo.setLatitude(dDoubleValue);
                liveInfo.setLongitude(dDoubleValue2);
                liveInfo.setAltitude(dDoubleValue3);
                liveInfo.setCompass(this.curr_compass.doubleValue());
                liveInfo.setVelocity_x(d4);
                liveInfo.setVelocity_y(d5);
                liveInfo.setVelocity_z(d6);
                liveInfo.setGimbalTilt(d);
                liveInfo.setGimbalPan(d2);
                liveInfo.setGimbalYaw(d3);
                liveInfo.setCombined_velocity(dSqrt);
                if (this.parameter_selection_mode) {
                    liveInfo.setUsable(false);
                }
            }
            fire(liveInfo);
            new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.getDefault()).format(new Date());
            File file = new File(AOSActivity.baseDir, "g_waypoint");
            if (file.exists()) {
                return;
            }
            file.mkdirs();
        }
    }

    public void addListener(LiveInfoListener liveInfoListener) {
        this.listeners.add(liveInfoListener);
    }

    public void removeListener(LiveInfoListener liveInfoListener) {
        this.listeners.remove(liveInfoListener);
    }

    private void fire(LiveInfo liveInfo) {
        Iterator<LiveInfoListener> it = this.listeners.iterator();
        while (it.hasNext()) {
            it.next().onReceive(liveInfo);
        }
    }

    public void setSimulation_mode(boolean z) {
        this.simulation_mode = z;
    }

    public boolean getSimulation_mode() {
        return this.simulation_mode;
    }

    public void setParameter_selection_mode(boolean z) {
        this.parameter_selection_mode = z;
    }

    public boolean Parameter_selection_mode() {
        return this.parameter_selection_mode;
    }

    public void next_data_simulation() {
        this.location_list_count++;
    }

    public void setRtk_mode(boolean z) {
        this.rtk_mode = Boolean.valueOf(z);
    }

    public boolean getRtk_mode() {
        return this.rtk_mode.booleanValue();
    }

    public void addRTKLocationInfoListener() {
        RTKCenter.getInstance().addRTKLocationInfoListener(this.live_rtkLocationInfoListener);
    }

    public void removeRTKLocationInfoListener() {
        RTKCenter.getInstance().removeRTKLocationInfoListener(this.live_rtkLocationInfoListener);
    }
}
