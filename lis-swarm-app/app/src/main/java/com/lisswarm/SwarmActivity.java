package com.lisswarm;

import android.app.Activity;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.widget.Button;
import android.widget.TextView;

import at.jku.icg.aos_dji_sdkv5.core.AOSManager;
import at.jku.icg.aos_dji_sdkv5.core.DroneSwarmStreamData;
import at.jku.icg.aos_dji_sdkv5.core.MQTTEmbedded;
import at.jku.icg.aos_dji_sdkv5.dji.DJIManager;

import dji.sdk.keyvalue.key.FlightControllerKey;
import dji.sdk.keyvalue.key.GimbalKey;
import dji.sdk.keyvalue.key.KeyTools;
import dji.sdk.keyvalue.value.common.Attitude;
import dji.sdk.keyvalue.value.common.EmptyMsg;
import dji.sdk.keyvalue.value.common.LocationCoordinate3D;
import dji.sdk.keyvalue.value.common.Velocity3D;
import dji.sdk.keyvalue.value.flightcontroller.FlightControlAuthorityChangeReason;
import dji.sdk.keyvalue.value.flightcontroller.FlightCoordinateSystem;
import dji.sdk.keyvalue.value.flightcontroller.RollPitchControlMode;
import dji.sdk.keyvalue.value.flightcontroller.VerticalControlMode;
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam;
import dji.sdk.keyvalue.value.flightcontroller.YawControlMode;
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotation;
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotationMode;
import dji.v5.common.callback.CommonCallbacks;
import dji.v5.common.error.IDJIError;
import dji.v5.common.video.channel.VideoChannelType;
import dji.v5.common.video.decoder.DecoderOutputMode;
import dji.v5.common.video.decoder.VideoDecoder;
import dji.v5.manager.KeyManager;
import dji.v5.manager.aircraft.virtualstick.VirtualStickManager;
import dji.v5.manager.aircraft.virtualstick.VirtualStickState;
import dji.v5.manager.aircraft.virtualstick.VirtualStickStateListener;

import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Collections;
import java.util.Enumeration;
import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;

/**
 * Main activity for LIS_Swarm. Streams video/telemetry to PC via RTSP (native
 * libs) and receives joystick commands via embedded MQTT broker.
 *
 * Command protocol (received via MQTT):
 *   "VS:pitch:roll:yaw:throttle:gimbal_pitch:gimbal_yaw"
 *   "ENABLE_VS" / "DISABLE_VS"
 *   "TAKEOFF" / "LAND"
 */
public class SwarmActivity extends Activity {

    private static final String TAG = "LIS_Swarm";
    private static final long VS_SEND_INTERVAL_MS = 50;   // 20 Hz
    private static final long TELEM_UI_INTERVAL_MS = 500;  // 2 Hz UI refresh

    // Existing video/telemetry pipeline
    private DJIManager djiManager;
    private AOSManager aosManager;
    public DroneSwarmStreamData droneSwarmStreamData;
    private MQTTEmbedded mqttEmbedded;

    // Virtual stick state
    private volatile double vsPitch = 0;
    private volatile double vsRoll = 0;
    private volatile double vsYaw = 0;
    private volatile double vsThrottle = 0;
    private volatile double cmdGimbalPitch = -90;
    private volatile double cmdGimbalYaw = 0;
    private volatile boolean vsActive = false;

    private VirtualStickState currentVsState;
    private Timer vsSendTimer;
    private Timer telemUiTimer;

    // Telemetry values (updated by KeyManager callbacks)
    private volatile double telemLat = 0, telemLon = 0, telemAlt = 0;
    private volatile double telemHeading = 0;
    private volatile double telemPitch = 0, telemRoll = 0, telemYaw = 0;
    private volatile double telemGimbalPitch = 0, telemGimbalRoll = 0, telemGimbalYaw = 0;
    private volatile double telemVx = 0, telemVy = 0, telemVz = 0;
    private volatile int telemSatCount = 0;

    // Video
    private SurfaceView surfaceVideo;

    // UI
    private TextView tvStatus;
    private TextView tvTelemetry;
    private TextView tvIp;
    private TextView tvVsState;
    private TextView tvTelemGps;
    private TextView tvTelemAttitude;
    private TextView tvTelemGimbal;
    private TextView tvTelemVelocity;
    private Button btnStartRtsp;
    private Button btnEnableVs;
    private Button btnDisableVs;
    private Handler uiHandler;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_swarm);

        uiHandler = new Handler(Looper.getMainLooper());
        initUI();
        initVideoAndTelemetry();
        initMQTT();
        initVirtualStick();
        startVsSendLoop();
        startTelemetryListeners();
        startTelemUiLoop();
    }

    private void initUI() {
        tvStatus = findViewById(R.id.tv_status);
        tvTelemetry = findViewById(R.id.tv_telemetry);
        tvIp = findViewById(R.id.tv_ip);
        tvVsState = findViewById(R.id.tv_vs_state);
        tvTelemGps = findViewById(R.id.tv_telem_gps);
        tvTelemAttitude = findViewById(R.id.tv_telem_attitude);
        tvTelemGimbal = findViewById(R.id.tv_telem_gimbal);
        tvTelemVelocity = findViewById(R.id.tv_telem_velocity);
        btnStartRtsp = findViewById(R.id.btn_start_rtsp);
        btnEnableVs = findViewById(R.id.btn_enable_vs);
        btnDisableVs = findViewById(R.id.btn_disable_vs);
        surfaceVideo = findViewById(R.id.surface_video);

        btnStartRtsp.setOnClickListener(v -> onStartRtspClicked());
        btnEnableVs.setOnClickListener(v -> enableVirtualStick());
        btnDisableVs.setOnClickListener(v -> disableVirtualStick());

        // Create DJI VideoDecoder once the surface is available
        surfaceVideo.getHolder().addCallback(new SurfaceHolder.Callback() {
            @Override
            public void surfaceCreated(SurfaceHolder holder) {
                Log.i(TAG, "Surface created, starting VideoDecoder");
                if (djiManager != null && djiManager.videodecoder != null) {
                    djiManager.videodecoder.onPause();
                    djiManager.videodecoder.destroy();
                }
                if (djiManager != null) {
                    djiManager.videodecoder = new VideoDecoder(
                        getApplicationContext(),
                        VideoChannelType.PRIMARY_STREAM_CHANNEL,
                        DecoderOutputMode.SURFACE_MODE,
                        holder,
                        surfaceVideo.getWidth(),
                        surfaceVideo.getHeight(),
                        true);
                    Log.i(TAG, "VideoDecoder created");
                }
            }

            @Override
            public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
            }

            @Override
            public void surfaceDestroyed(SurfaceHolder holder) {
                Log.i(TAG, "Surface destroyed");
                if (djiManager != null && djiManager.videodecoder != null) {
                    djiManager.videodecoder.onPause();
                    djiManager.videodecoder.destroy();
                    djiManager.videodecoder = null;
                }
            }
        });

        String ethIp = getEth0Ip();
        tvIp.setText("eth0: " + ethIp);
        tvStatus.setText("Initializing... RTSP not started yet");
    }

    // ========== RTSP Control ==========

    /**
     * User presses "Start RTSP" once video is confirmed working on screen.
     * This starts the native RTSP server with the correct detected video format,
     * and enables frame feeding.
     */
    private void onStartRtspClicked() {
        if (droneSwarmStreamData == null) {
            updateStatus("Video pipeline not ready");
            return;
        }
        boolean ok = droneSwarmStreamData.startRtspServer();
        String ethIp = getEth0Ip();
        if (ok) {
            btnStartRtsp.setEnabled(false);
            btnStartRtsp.setText("RTSP ON");
            updateStatus("RTSP server started on " + ethIp + ":8554");
            tvIp.setText("RTSP: rtsp://video:video@" + ethIp + ":8554");
        } else {
            updateStatus("RTSP start FAILED — check logcat");
        }
    }

    // ========== Video & Telemetry (existing pipeline) ==========

    private void initVideoAndTelemetry() {
        aosManager = new AOSManager();
        aosManager.aosActivity = null;

        // Mark AOSManager as running so getTelemetryData() actually collects data.
        aosManager.setRunning(true);

        djiManager = new DJIManager(this);

        droneSwarmStreamData = new DroneSwarmStreamData(djiManager);
        droneSwarmStreamData.aosManager = aosManager;
        // Start with RTSP feed DISABLED — user enables it via the button
        droneSwarmStreamData.EnableRtspStream = false;
        djiManager.droneSwarmStreamData = droneSwarmStreamData;

        try {
            droneSwarmStreamData.start();
            updateStatus("Video pipeline started. Press 'Start RTSP' to stream.");
        } catch (Exception e) {
            Log.e(TAG, "Failed to start video pipeline", e);
            updateStatus("Video start failed: " + e.getMessage());
        }

        aosManager.start();
    }

    // ========== MQTT ==========

    private void initMQTT() {
        mqttEmbedded = new MQTTEmbedded(aosManager, getFilesDir().getAbsolutePath());
        mqttEmbedded.setCommandListener(this::onCommandReceived);
        try {
            mqttEmbedded.run();
            updateStatus("MQTT broker started on :1883");
        } catch (Exception e) {
            Log.e(TAG, "Failed to start MQTT broker", e);
            updateStatus("MQTT FAILED: " + e.getMessage());
        }
    }

    public void onCommandReceived(String command) {
        // DIAGNOSTIC: log every MQTT command landing on the app
        Log.v(TAG, "MQTT recv: " + command + "  ts=" + System.currentTimeMillis());
        if (command == null || command.isEmpty()) return;

        if (command.startsWith("VS:")) {
            String[] parts = command.substring(3).split(":");
            if (parts.length >= 4) {
                vsPitch = Double.parseDouble(parts[0]);
                vsRoll = Double.parseDouble(parts[1]);
                vsYaw = Double.parseDouble(parts[2]);
                vsThrottle = Double.parseDouble(parts[3]);
                if (parts.length >= 5) cmdGimbalPitch = Double.parseDouble(parts[4]);
                if (parts.length >= 6) cmdGimbalYaw = Double.parseDouble(parts[5]);
            }
        } else if (command.equals("ENABLE_VS")) {
            enableVirtualStick();
        } else if (command.equals("DISABLE_VS")) {
            disableVirtualStick();
        } else if (command.equals("TAKEOFF")) {
            performTakeoff();
        } else if (command.equals("LAND")) {
            performLanding();
        }
    }

    // ========== Telemetry Display ==========

    /**
     * Register KeyManager listeners to continuously receive drone telemetry.
     * These update volatile fields that the UI timer reads.
     */
    private void startTelemetryListeners() {
        // GPS location
        KeyManager.getInstance().listen(
            KeyTools.createKey(FlightControllerKey.KeyAircraftLocation3D), this,
            (LocationCoordinate3D oldVal, LocationCoordinate3D newVal) -> {
                if (newVal != null) {
                    telemLat = newVal.getLatitude();
                    telemLon = newVal.getLongitude();
                    telemAlt = newVal.getAltitude();
                }
            });

        // Compass heading
        KeyManager.getInstance().listen(
            KeyTools.createKey(FlightControllerKey.KeyCompassHeading), this,
            (Double oldVal, Double newVal) -> {
                if (newVal != null) telemHeading = newVal;
            });

        // Aircraft attitude
        KeyManager.getInstance().listen(
            KeyTools.createKey(FlightControllerKey.KeyAircraftAttitude), this,
            (Attitude oldVal, Attitude newVal) -> {
                if (newVal != null) {
                    telemPitch = newVal.getPitch();
                    telemRoll = newVal.getRoll();
                    telemYaw = newVal.getYaw();
                }
            });

        // Gimbal attitude
        KeyManager.getInstance().listen(
            KeyTools.createKey(GimbalKey.KeyGimbalAttitude), this,
            (Attitude oldVal, Attitude newVal) -> {
                if (newVal != null) {
                    telemGimbalPitch = newVal.getPitch();
                    telemGimbalRoll = newVal.getRoll();
                    telemGimbalYaw = newVal.getYaw();
                }
            });

        // Velocity
        KeyManager.getInstance().listen(
            KeyTools.createKey(FlightControllerKey.KeyAircraftVelocity), this,
            (Velocity3D oldVal, Velocity3D newVal) -> {
                if (newVal != null) {
                    telemVx = newVal.getX();
                    telemVy = newVal.getY();
                    telemVz = newVal.getZ();
                }
            });

        // Satellite count
        KeyManager.getInstance().listen(
            KeyTools.createKey(FlightControllerKey.KeyGPSSatelliteCount), this,
            (Integer oldVal, Integer newVal) -> {
                if (newVal != null) telemSatCount = newVal;
            });
    }

    /**
     * Periodic UI update showing live drone telemetry on screen.
     */
    private void startTelemUiLoop() {
        telemUiTimer = new Timer("TelemUI");
        telemUiTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                uiHandler.post(() -> {
                    tvTelemGps.setText(String.format(Locale.US,
                        "GPS: %.6f, %.6f  Alt:%.1fm\nHdg:%.1f  Sat:%d",
                        telemLat, telemLon, telemAlt, telemHeading, telemSatCount));

                    tvTelemAttitude.setText(String.format(Locale.US,
                        "ATT: P:%.1f R:%.1f Y:%.1f",
                        telemPitch, telemRoll, telemYaw));

                    tvTelemGimbal.setText(String.format(Locale.US,
                        "GMB: P:%.1f R:%.1f Y:%.1f",
                        telemGimbalPitch, telemGimbalRoll, telemGimbalYaw));

                    tvTelemVelocity.setText(String.format(Locale.US,
                        "VEL: X:%.2f Y:%.2f Z:%.2f\nRTSP: %s  Frames: %d",
                        telemVx, telemVy, telemVz,
                        droneSwarmStreamData != null && droneSwarmStreamData.RtspIsRunning ? "ON" : "OFF",
                        droneSwarmStreamData != null ? droneSwarmStreamData.getRtspFramesFed() : 0));
                });
            }
        }, 500, TELEM_UI_INTERVAL_MS);
    }

    // ========== Virtual Stick ==========

    private void initVirtualStick() {
        VirtualStickManager.getInstance().setVirtualStickStateListener(
            new VirtualStickStateListener() {
                @Override
                public void onVirtualStickStateUpdate(VirtualStickState state) {
                    currentVsState = state;
                    vsActive = state.isVirtualStickEnable();
                    droneSwarmStreamData.virtualstickonoff = vsActive ? 1.0 : 0.0;
                    uiHandler.post(() -> tvVsState.setText(String.format("VS: %s",
                        vsActive ? "ENABLED" : "DISABLED")));
                }

                @Override
                public void onChangeReasonUpdate(FlightControlAuthorityChangeReason reason) {
                    Log.i(TAG, "Flight authority change: " + reason.name());
                }
            });
    }

    private void enableVirtualStick() {
        VirtualStickManager.getInstance().enableVirtualStick(
            new CommonCallbacks.CompletionCallback() {
                @Override
                public void onSuccess() {
                    Log.i(TAG, "Virtual stick enabled");
                    VirtualStickManager.getInstance().setVirtualStickAdvancedModeEnabled(true);
                    updateStatus("Virtual stick ENABLED");
                }

                @Override
                public void onFailure(IDJIError error) {
                    Log.e(TAG, "Enable VS failed: " + error.description());
                    updateStatus("VS enable failed: " + error.description());
                }
            });
    }

    private void disableVirtualStick() {
        vsPitch = 0;
        vsRoll = 0;
        vsYaw = 0;

        VirtualStickManager.getInstance().disableVirtualStick(
            new CommonCallbacks.CompletionCallback() {
                @Override
                public void onSuccess() {
                    Log.i(TAG, "Virtual stick disabled");
                    updateStatus("Virtual stick DISABLED");
                }

                @Override
                public void onFailure(IDJIError error) {
                    Log.e(TAG, "Disable VS failed: " + error.description());
                }
            });
    }

    private void startVsSendLoop() {
        vsSendTimer = new Timer("VS_Send");
        vsSendTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                if (!vsActive) return;

                VirtualStickFlightControlParam param = new VirtualStickFlightControlParam();
                param.setRollPitchCoordinateSystem(FlightCoordinateSystem.GROUND);
                param.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
                param.setYawControlMode(YawControlMode.ANGLE);
                param.setVerticalControlMode(VerticalControlMode.POSITION);
                param.setPitch(vsPitch);
                param.setRoll(vsRoll);
                param.setYaw(vsYaw);
                param.setVerticalThrottle(vsThrottle);

                VirtualStickManager.getInstance().sendVirtualStickAdvancedParam(param);
                sendGimbalCommand(cmdGimbalPitch, cmdGimbalYaw);

                uiHandler.post(() -> tvTelemetry.setText(String.format(Locale.US,
                    "Cmd: P=%.1f R=%.1f Y=%.1f T=%.1f  Gimbal: P=%.1f Y=%.1f",
                    vsPitch, vsRoll, vsYaw, vsThrottle, cmdGimbalPitch, cmdGimbalYaw)));
            }
        }, 0, VS_SEND_INTERVAL_MS);
    }

    private void sendGimbalCommand(double pitch, double yaw) {
        GimbalAngleRotation rotation = new GimbalAngleRotation();
        rotation.setMode(GimbalAngleRotationMode.ABSOLUTE_ANGLE);
        rotation.setPitch(pitch);
        rotation.setYaw(yaw);

        KeyManager.getInstance().performAction(
            KeyTools.createKey(GimbalKey.KeyRotateByAngle),
            rotation,
            new CommonCallbacks.CompletionCallbackWithParam<EmptyMsg>() {
                @Override public void onSuccess(EmptyMsg msg) {}
                @Override public void onFailure(IDJIError error) {}
            });
    }

    private void performTakeoff() {
        KeyManager.getInstance().performAction(
            KeyTools.createKey(FlightControllerKey.KeyStartTakeoff),
            null,
            new CommonCallbacks.CompletionCallbackWithParam<EmptyMsg>() {
                @Override
                public void onSuccess(EmptyMsg msg) { updateStatus("Takeoff initiated"); }
                @Override
                public void onFailure(IDJIError error) {
                    updateStatus("Takeoff failed: " + error.description());
                }
            });
    }

    private void performLanding() {
        KeyManager.getInstance().performAction(
            KeyTools.createKey(FlightControllerKey.KeyStartAutoLanding),
            null,
            new CommonCallbacks.CompletionCallbackWithParam<EmptyMsg>() {
                @Override
                public void onSuccess(EmptyMsg msg) { updateStatus("Landing initiated"); }
                @Override
                public void onFailure(IDJIError error) {
                    updateStatus("Landing failed: " + error.description());
                }
            });
    }

    // ========== Helpers ==========

    private void updateStatus(String msg) {
        Log.i(TAG, msg);
        uiHandler.post(() -> tvStatus.setText(msg));
    }

    /**
     * Returns the IPv4 address of eth0 (ethernet). Falls back to any
     * non-loopback IPv4 if eth0 has no address.
     */
    private String getEth0Ip() {
        String fallbackIp = null;
        try {
            Enumeration<NetworkInterface> interfaces = NetworkInterface.getNetworkInterfaces();
            if (interfaces == null) return "no interfaces";
            for (NetworkInterface ni : Collections.list(interfaces)) {
                for (InetAddress addr : Collections.list(ni.getInetAddresses())) {
                    if (addr.isLoopbackAddress() || !(addr instanceof Inet4Address)) continue;
                    String ip = addr.getHostAddress();
                    // Prefer eth0 (wired ethernet on RC Pro)
                    if (ni.getName().startsWith("eth")) {
                        return ip;
                    }
                    if (fallbackIp == null) {
                        fallbackIp = ip;
                    }
                }
            }
        } catch (SocketException e) {
            Log.e(TAG, "Failed to get IP", e);
        }
        return fallbackIp != null ? fallbackIp : "no IP";
    }

    // ========== Lifecycle ==========

    @Override
    protected void onDestroy() {
        if (vsSendTimer != null) vsSendTimer.cancel();
        if (telemUiTimer != null) telemUiTimer.cancel();

        if (vsActive) {
            vsPitch = 0; vsRoll = 0;
            VirtualStickManager.getInstance().disableVirtualStick(
                new CommonCallbacks.CompletionCallback() {
                    @Override public void onSuccess() {}
                    @Override public void onFailure(IDJIError e) {}
                });
        }

        if (droneSwarmStreamData != null) droneSwarmStreamData.stop();
        if (aosManager != null) {
            aosManager.stop();
            aosManager.destroy();
        }
        if (djiManager != null) djiManager.onDestroy();
        if (mqttEmbedded != null) mqttEmbedded.stop();

        KeyManager.getInstance().cancelListen(this);
        super.onDestroy();
    }
}
