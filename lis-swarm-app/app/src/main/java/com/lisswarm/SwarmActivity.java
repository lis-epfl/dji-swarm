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

import dji.sdk.keyvalue.key.AirLinkKey;
import dji.sdk.keyvalue.key.BatteryKey;
import dji.sdk.keyvalue.key.DJIKeyInfo;
import dji.sdk.keyvalue.key.FlightControllerKey;
import dji.sdk.keyvalue.key.GimbalKey;
import dji.sdk.keyvalue.key.KeyTools;
import dji.sdk.keyvalue.value.airlink.ChannelSelectionMode;
import dji.sdk.keyvalue.value.airlink.FrequencyBand;
import dji.sdk.keyvalue.value.camera.VideoFrameRate;
import dji.sdk.keyvalue.value.camera.VideoResolution;
import dji.sdk.keyvalue.value.camera.VideoResolutionFrameRate;
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
 *     pitch/roll = world N/E velocity m/s, yaw = yaw RATE deg/s,
 *     throttle = absolute altitude m, gimbal angles = absolute deg.
 *   "ENABLE_VS" / "DISABLE_VS"
 *   "TAKEOFF" / "LAND"
 *   "AIRLINK:band:channel:res@fps" — one-shot radio/camera-stream setup
 *     (see applyAirlinkSettings; sent by the PC after its command channel
 *     connects so per-drone radio config lives in the PC's flocking config)
 *
 * Operator lockout: the on-screen Disable VS button latches out ALL PC motion
 * commands (VS:, ENABLE_VS, TAKEOFF, LAND) until the on-screen Enable VS
 * button is pressed. Only AIRLINK: and DISABLE_VS pass through while latched.
 * PC-sent DISABLE_VS does NOT latch (GUI Stop→Start keeps working).
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
    private volatile double vsYaw = 0;          // yaw RATE, deg/s (ANGULAR_VELOCITY mode)
    private volatile double vsThrottle = 0;
    private volatile double cmdGimbalPitch = -90;
    private volatile double cmdGimbalYaw = 0;
    private volatile boolean vsActive = false;

    // Operator lockout latch: set by the local Disable VS button, cleared ONLY
    // by the local Enable VS button. While set, PC motion commands (VS:,
    // ENABLE_VS, TAKEOFF, LAND) are ignored — the PC streams VS: at 20 Hz for
    // the life of its controller and its QoS-1 one-shots can arrive late after
    // a link blip, so a plain disableVirtualStick() call is not an operator
    // override (the next incoming message re-fills the command cache within
    // 50 ms, and a queued ENABLE_VS can silently re-arm). PC-initiated
    // DISABLE_VS never sets this latch, so normal GUI Stop→Start is unaffected.
    private volatile boolean pcLockout = false;
    private long lockoutDropCount = 0;              // MQTT thread ++, UI thread resets; log-only
    private volatile long lastLockoutLogMs = 0;
    private volatile long lastLockoutDisableRetryMs = 0;
    private static final long LOCKOUT_LOG_INTERVAL_MS = 2000;
    private static final long LOCKOUT_DISABLE_RETRY_MS = 1000;

    // volatile: written by DJI's state listener, read by the VS send timer
    // (the lockout retry checks it cross-thread)
    private volatile VirtualStickState currentVsState;
    private Timer vsSendTimer;
    private Timer telemUiTimer;

    // Telemetry values (updated by KeyManager callbacks)
    private volatile double telemLat = 0, telemLon = 0, telemAlt = 0;
    private volatile double telemHeading = 0;
    private volatile double telemPitch = 0, telemRoll = 0, telemYaw = 0;
    private volatile double telemGimbalPitch = 0, telemGimbalRoll = 0, telemGimbalYaw = 0;
    private volatile double telemVx = 0, telemVy = 0, telemVz = 0;
    private volatile int telemSatCount = 0;
    private volatile int telemBatteryPercent = -1;   // -1 = not yet reported
    private volatile int telemSignalQuality = -1;     // 0-100, -1 = not yet reported

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
    private TextView tvBattery;
    private TextView tvLink;
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
        tvBattery = findViewById(R.id.tv_battery);
        tvLink = findViewById(R.id.tv_link);
        btnStartRtsp = findViewById(R.id.btn_start_rtsp);
        btnEnableVs = findViewById(R.id.btn_enable_vs);
        btnDisableVs = findViewById(R.id.btn_disable_vs);
        surfaceVideo = findViewById(R.id.surface_video);

        btnStartRtsp.setOnClickListener(v -> onStartRtspClicked());
        btnEnableVs.setOnClickListener(v -> onLocalEnableClicked());
        btnDisableVs.setOnClickListener(v -> onLocalDisableClicked());

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

        // Operator lockout: drop every motion command from the PC (VS: stream,
        // ENABLE_VS, TAKEOFF, LAND). DISABLE_VS stays honored (redundant but
        // harmless, and it never clears the latch); AIRLINK: stays honored
        // (radio/camera config, no motion).
        if (pcLockout && (command.startsWith("VS:") || command.equals("ENABLE_VS")
                || command.equals("TAKEOFF") || command.equals("LAND"))) {
            lockoutDropCount++;
            long now = System.currentTimeMillis();
            if (now - lastLockoutLogMs >= LOCKOUT_LOG_INTERVAL_MS) {
                lastLockoutLogMs = now;
                Log.i(TAG, "PC lockout: dropped " + lockoutDropCount
                    + " PC motion commands (latest: " + command + ")");
            }
            return;
        }

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
        } else if (command.startsWith("AIRLINK:")) {
            applyAirlinkSettings(command.substring("AIRLINK:".length()));
        }
    }

    // ========== AirLink management ==========

    /**
     * "AIRLINK:&lt;band&gt;:&lt;channel&gt;:&lt;res@fps&gt;" — with ten aircraft/RC links
     * sharing the spectrum, DJI's per-link auto channel selection has no view
     * of the whole fleet, so the PC assigns bands/channels deterministically.
     * Fields ('-' or empty = leave unchanged):
     *   band     2G4 | 5G8 | DUAL (firmware picks per packet)
     *   channel  &gt;=0 = ChannelSelectionMode MANUAL + that channel number;
     *            -1  = back to ChannelSelectionMode AUTO
     *   res@fps  camera stream cap, e.g. 1920x1080@24 — lowers the encoded
     *            bitrate and thus the per-link airtime
     * Every set is read back and surfaced via updateStatus/logcat, so firmware
     * that locks a key (likely for manual channels on consumer aircraft) is
     * visible on the RC screen during a bench test; a rejected MANUAL channel
     * falls back to AUTO rather than leaving the link half-configured.
     */
    private void applyAirlinkSettings(String spec) {
        try {
            String[] parts = spec.split(":");
            String bandTok = parts.length >= 1 ? parts[0].trim() : "";
            String chanTok = parts.length >= 2 ? parts[1].trim() : "";
            String resTok = parts.length >= 3 ? parts[2].trim() : "";

            if (!bandTok.isEmpty() && !bandTok.equals("-")) {
                FrequencyBand band =
                    bandTok.equals("2G4") ? FrequencyBand.BAND_2_DOT_4G :
                    bandTok.equals("5G8") ? FrequencyBand.BAND_5_DOT_8G :
                    bandTok.equals("DUAL") ? FrequencyBand.BAND_DUAL : null;
                if (band == null) {
                    updateStatus("AIRLINK: unknown band '" + bandTok + "'");
                } else {
                    setAirlinkKeyAndVerify("band", AirLinkKey.KeyFrequencyBand, band);
                }
            }

            if (!chanTok.isEmpty() && !chanTok.equals("-")) {
                int channel = Integer.parseInt(chanTok);
                if (channel < 0) {
                    setAirlinkKeyAndVerify("channel mode",
                        AirLinkKey.KeyChannelSelectionMode, ChannelSelectionMode.AUTO);
                } else {
                    setManualChannel(channel);
                }
            }

            if (!resTok.isEmpty() && !resTok.equals("-")) {
                int at = resTok.indexOf('@');
                VideoResolution res = VideoResolution.valueOf(
                    "RESOLUTION_" + resTok.substring(0, at));
                VideoFrameRate rate = VideoFrameRate.valueOf(
                    "RATE_" + resTok.substring(at + 1) + "FPS");
                droneSwarmStreamData.setVideoResolution(
                    new VideoResolutionFrameRate(res, rate));
                updateStatus("AIRLINK video -> " + resTok);
            }
        } catch (Exception e) {
            // Never let a malformed one-shot kill the command listener
            updateStatus("AIRLINK parse failed for '" + spec + "': " + e);
        }
    }

    /** MANUAL selection mode first, then the channel number; either rejection
     *  reverts to AUTO so the link is never left half-configured. */
    private void setManualChannel(int channel) {
        KeyManager.getInstance().setValue(
            KeyTools.createKey(AirLinkKey.KeyChannelSelectionMode),
            ChannelSelectionMode.MANUAL,
            new CommonCallbacks.CompletionCallback() {
                @Override
                public void onSuccess() {
                    KeyManager.getInstance().setValue(
                        KeyTools.createKey(AirLinkKey.KeyChannelNumber),
                        channel,
                        new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onSuccess() {
                                readBackAirlinkKey("channel", AirLinkKey.KeyChannelNumber);
                            }

                            @Override
                            public void onFailure(IDJIError error) {
                                updateStatus("AIRLINK channel=" + channel
                                    + " REJECTED (" + error.description()
                                    + ") — reverting to AUTO");
                                setAirlinkKeyAndVerify("channel mode",
                                    AirLinkKey.KeyChannelSelectionMode,
                                    ChannelSelectionMode.AUTO);
                            }
                        });
                }

                @Override
                public void onFailure(IDJIError error) {
                    updateStatus("AIRLINK MANUAL mode REJECTED ("
                        + error.description() + ") — staying AUTO");
                }
            });
    }

    private <T> void setAirlinkKeyAndVerify(String what, DJIKeyInfo<T> keyInfo, T value) {
        KeyManager.getInstance().setValue(
            KeyTools.createKey(keyInfo), value,
            new CommonCallbacks.CompletionCallback() {
                @Override
                public void onSuccess() {
                    readBackAirlinkKey(what, keyInfo);
                }

                @Override
                public void onFailure(IDJIError error) {
                    updateStatus("AIRLINK " + what + "=" + value
                        + " REJECTED: " + error.description());
                }
            });
    }

    private <T> void readBackAirlinkKey(String what, DJIKeyInfo<T> keyInfo) {
        KeyManager.getInstance().getValue(
            KeyTools.createKey(keyInfo),
            new CommonCallbacks.CompletionCallbackWithParam<T>() {
                @Override
                public void onSuccess(T readBack) {
                    updateStatus("AIRLINK " + what + " -> " + readBack);
                }

                @Override
                public void onFailure(IDJIError error) {
                    updateStatus("AIRLINK " + what
                        + " set OK but read-back failed: " + error.description());
                }
            });
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

        // Battery charge remaining (percent)
        KeyManager.getInstance().listen(
            KeyTools.createKey(BatteryKey.KeyChargeRemainingInPercent), this,
            (Integer oldVal, Integer newVal) -> {
                if (newVal != null) telemBatteryPercent = newVal;
            });

        // Air link signal quality (0-100), i.e. connection quality to the RC
        KeyManager.getInstance().listen(
            KeyTools.createKey(AirLinkKey.KeySignalQuality), this,
            (Integer oldVal, Integer newVal) -> {
                if (newVal != null) telemSignalQuality = newVal;
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
                    tvBattery.setText(telemBatteryPercent < 0
                        ? "BAT: --%"
                        : String.format(Locale.US, "BAT: %d%%", telemBatteryPercent));
                    tvBattery.setTextColor(batteryColor(telemBatteryPercent));

                    tvLink.setText(telemSignalQuality < 0
                        ? "LINK: --"
                        : String.format(Locale.US, "LINK: %d%%", telemSignalQuality));
                    tvLink.setTextColor(signalColor(telemSignalQuality));

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
                    refreshVsStateLabel();
                }

                @Override
                public void onChangeReasonUpdate(FlightControlAuthorityChangeReason reason) {
                    Log.i(TAG, "Flight authority change: " + reason.name());
                }
            });
    }

    // ========== Local operator buttons (RC screen) ==========

    /**
     * Disable VS button: engage the PC lockout latch, close the send-loop gate
     * immediately (waiting for DJI's async state listener would let PC commands
     * keep reaching the FC for a beat after the press), then disable VS. The
     * latch stays on until the local Enable VS button is pressed — a PC GUI
     * Start will NOT re-arm this aircraft while latched.
     */
    private void onLocalDisableClicked() {
        pcLockout = true;
        lockoutDropCount = 0;
        vsActive = false;   // eager; onVirtualStickStateUpdate stays the source of truth
        disableVirtualStick();
        updateStatus("PC LOCKOUT ON — ignoring PC motion commands until Enable VS");
        refreshVsStateLabel();
    }

    /** Enable VS button: clear the lockout latch and re-enable VS. */
    private void onLocalEnableClicked() {
        pcLockout = false;
        refreshVsStateLabel();
        enableVirtualStick();
    }

    private void refreshVsStateLabel() {
        final String text = String.format("VS: %s%s",
            vsActive ? "ENABLED" : "DISABLED",
            pcLockout ? " | PC LOCKED OUT" : "");
        uiHandler.post(() -> tvVsState.setText(text));
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
                    updateStatus("VS disable FAILED: " + error.description());
                }
            });
    }

    private void startVsSendLoop() {
        vsSendTimer = new Timer("VS_Send");
        vsSendTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                if (pcLockout) {
                    // Belt-and-braces while latched: if VS somehow reports
                    // enabled again (initial disable failed silently, or a
                    // stray external re-enable), keep re-issuing the disable
                    // at ~1 Hz until the FC confirms it is off.
                    VirtualStickState s = currentVsState;
                    if (s != null && s.isVirtualStickEnable()) {
                        long now = System.currentTimeMillis();
                        if (now - lastLockoutDisableRetryMs >= LOCKOUT_DISABLE_RETRY_MS) {
                            lastLockoutDisableRetryMs = now;
                            Log.w(TAG, "PC lockout: VS still enabled, retrying disable");
                            disableVirtualStick();
                        }
                    }
                    return;
                }
                if (!vsActive) return;

                VirtualStickFlightControlParam param = new VirtualStickFlightControlParam();
                param.setRollPitchCoordinateSystem(FlightCoordinateSystem.GROUND);
                param.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
                // Yaw is RATE-controlled (deg/s), not absolute angle. ANGLE mode
                // makes the FC's position controller chase a stepped heading
                // setpoint, which is visibly jerky both while turning and while
                // holding. The PC side runs a heading-hold P controller and sends
                // a smooth yaw rate; vsYaw is deg/s. (See the yaw note in CLAUDE.md.)
                param.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
                param.setVerticalControlMode(VerticalControlMode.POSITION);
                // AXIS NOTE: in GROUND + VELOCITY on this Mini 3 Pro / MSDK v5,
                // the DJI pitch axis moves the aircraft EAST and the roll axis
                // moves it NORTH — the transpose of the naive assumption. Verified
                // by the 2026-07-05 rotation-check: with pitch<-north/roll<-east
                // every drone flew east on a north command and north on an east
                // command, identically at west/north/south headings (heading-
                // independent => world frame, but N/E swapped). So feed the
                // protocol's EAST field to setPitch and its NORTH field to setRoll.
                param.setPitch(vsRoll);   // vsRoll  = protocol EAST  field -> DJI pitch(E)
                param.setRoll(vsPitch);   // vsPitch = protocol NORTH field -> DJI roll(N)
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

    /** Green >50%, amber 20-50%, red <20%, grey if unknown. */
    private static int batteryColor(int percent) {
        if (percent < 0) return 0xFFCCCCCC;
        if (percent < 20) return 0xFFFF4444;
        if (percent < 50) return 0xFFFFCC00;
        return 0xFF00FF00;
    }

    /** Green >70, amber 40-70, red <40 (0-100 signal quality), grey if unknown. */
    private static int signalColor(int quality) {
        if (quality < 0) return 0xFFCCCCCC;
        if (quality < 40) return 0xFFFF4444;
        if (quality < 70) return 0xFFFFCC00;
        return 0xFF00FF00;
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
