package com.liscontroller;

import android.Manifest;
import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.hardware.input.InputManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.util.TypedValue;
import android.view.Gravity;
import android.view.InputDevice;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.widget.LinearLayout;
import android.widget.TextView;

import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Render-only. It grants permissions, starts StreamService, and draws what the
 * service's reader sees at 10 Hz. Everything that must keep running lives in the
 * service.
 *
 * The screen is the operator's: one status line with the Ethernet address, the PC link and
 * the battery; the sticks and dials; the stick source and the gamepad's mode; C1/C2; and
 * alerts only when something needs doing. Every diagnostic the bench and DEBUGGING.md use
 * is in the Details panel over it.
 *
 * It also feeds GamepadInput: the RC's built-in gamepad is the fast (~70 Hz) source for the
 * sticks and dials, and Android delivers joystick events only to the FOCUSED window, which
 * is why this lives here and not in the service. The events are consumed here, so Android
 * never turns stick motion into D-pad focus navigation of this screen. While the activity
 * is not in front, the stream carries MSDK's (~10 Hz) values instead.
 *
 * SAFE MODE (rc-joystick/android/DEBUGGING.md) is entered automatically when the last
 * run left a crash report, or on request (adb shell am start -n
 * com.liscontroller/.MainActivity --ez safe true). Nothing starts by itself: Details opens
 * with the report and the previous run's last steps, and its stage buttons start the service
 * one piece at a time (1 stream only, 2 + MSDK, 3 + key reading), which bisects the crash.
 */
public class MainActivity extends Activity implements InputManager.InputDeviceListener {
    static final String EXTRA_SAFE = "safe";
    /** The RC Pro's built-in gamepad, "DJI embedded joystick" (README). */
    static final int GAMEPAD_VENDOR = 0x2ca3, GAMEPAD_PRODUCT = 0x1501;
    private static final int REQ_PERMS = 7;
    // res/values/colors.xml, as ints for the code that colours at runtime.
    private static final int C_OK = 0xFF3FB950, C_WARN = 0xFFD29922, C_BAD = 0xFFF85149,
            C_TEXT = 0xFFE6EDF3, C_DIM = 0xFF8B949E, C_FAINT = 0xFF545D68,
            C_SURFACE_HI = 0xFF1A2029, C_ACCENT = 0xFF58A6FF;
    /** Battery below this is an alert; the bridge warns at the same level. */
    private static final int BATTERY_LOW_PCT = 25;
    /** The C1/C2 pills stay lit this long after a press, so a quick tap is seen. */
    private static final long PRESS_FLASH_MS = 350;
    /** Protocol.BUTTONS indexes of the two buttons the PC uses (bridge: C1 panorama, C2 reset). */
    private static final int[] PILL_BUTTONS = {0, 1};

    private final Handler ui = new Handler(Looper.getMainLooper());
    private TextView statusWord, statusDetail, chipNet, chipPc, chipBatt;
    private TextView srcLabel, srcMode, srcRate, diag, detailsTitle;
    private View details, safeRow;
    private LinearLayout alerts;
    private StickPadView padL, padR;
    private DialBarView dialL, dialR;
    private TextView retry, grant;
    private final GradientDrawable dot = new GradientDrawable();
    private final TextView[] pills = new TextView[PILL_BUTTONS.length];
    private final GradientDrawable[] pillBg = new GradientDrawable[PILL_BUTTONS.length];
    private final int[] lastPresses = new int[PILL_BUTTONS.length];
    private final boolean[] pressesSeen = new boolean[PILL_BUTTONS.length];
    private final long[] flashUntil = new long[PILL_BUTTONS.length];
    private String alertsKey = "";
    private float density;
    private InputManager inputManager;
    private boolean safe;
    private int pendingStage;              // stage to start once permissions allow; 0 = none
    private String version = "?";
    private String crashReport, prevTrace;
    private String lastUiError;

    private final ArrayDeque<String> events = new ArrayDeque<>();
    private final SimpleDateFormat clock = new SimpleDateFormat("HH:mm:ss.SSS", Locale.US);
    private String devicesText = "";
    private String tracedDevices;
    // Gamepad reports per second, traced while a control moves: the fast side of the
    // gamepad-vs-MSDK comparison (RcInputReader.logRates traces the MSDK side).
    private int gamepadReports, foreignReports, gamepadLinesTraced;
    private long lastGamepadTick;
    private static final String[] MOTION_KEYS = {"lh", "lv", "rh", "rv", "l", "r"};  // MSDK stat names
    private long lastMotionLog, lastDiag, lastRateAt;
    private final Map<String, Long> prevCounts = new HashMap<>();
    private final Map<String, Double> rates = new HashMap<>();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        safe = getIntent().getBooleanExtra(EXTRA_SAFE, false) || CrashLog.hasCrash(this);
        CrashLog.step("MainActivity.onCreate" + (safe ? " - SAFE MODE, nothing auto-starts" : ""));
        try {
            version = getPackageManager().getPackageInfo(getPackageName(), 0).versionName;
        } catch (PackageManager.NameNotFoundException ignored) {
        }
        setContentView(R.layout.activity_main);
        density = getResources().getDisplayMetrics().density;
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        // Not a keyboard (IME) target. On Android 10, joystick MotionEvents are not pointer
        // events, so ViewRootImpl offers them to a bound IME (Gboard on this RC) before the
        // view hierarchy. That was NOT what blocked the gamepad (DJI's framework was, see
        // GamepadInput), but this app has no text input, so the IME is kept out of the path.
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_ALT_FOCUSABLE_IM);
        dot.setShape(GradientDrawable.OVAL);
        dot.setColor(C_FAINT);
        findViewById(R.id.status_dot).setBackground(dot);
        statusWord = findViewById(R.id.status_word);
        statusDetail = findViewById(R.id.status_detail);
        chipNet = findViewById(R.id.chip_net);
        chipPc = findViewById(R.id.chip_pc);
        chipBatt = findViewById(R.id.chip_batt);
        alerts = findViewById(R.id.alerts);
        srcLabel = findViewById(R.id.src_label);
        srcMode = findViewById(R.id.src_mode);
        srcRate = findViewById(R.id.src_rate);
        padL = findViewById(R.id.pad_left);
        padR = findViewById(R.id.pad_right);
        dialL = findViewById(R.id.dial_left);
        dialR = findViewById(R.id.dial_right);
        details = findViewById(R.id.details);
        detailsTitle = findViewById(R.id.details_title);
        diag = findViewById(R.id.diag);
        retry = findViewById(R.id.btn_retry);
        grant = findViewById(R.id.btn_grant);
        safeRow = findViewById(R.id.safe_row);
        buildPills();

        findViewById(R.id.btn_stop).setOnClickListener(v -> {
            stopService(new Intent(this, StreamService.class));   // sends bye
            finishAndRemoveTask();
        });
        findViewById(R.id.btn_details).setOnClickListener(v -> showDetails(true));
        findViewById(R.id.btn_close).setOnClickListener(v -> showDetails(false));
        retry.setOnClickListener(v -> {
            StreamService s = StreamService.instance;
            if (s != null) s.retryRegistration();
        });
        grant.setOnClickListener(v -> ensurePermissionsThenStart());
        findViewById(R.id.btn_stage1).setOnClickListener(v -> requestStage(StreamService.STAGE_SERVICE));
        findViewById(R.id.btn_stage2).setOnClickListener(v -> requestStage(StreamService.STAGE_SDK));
        findViewById(R.id.btn_stage3).setOnClickListener(v -> requestStage(StreamService.STAGE_FULL));
        findViewById(R.id.btn_clear).setOnClickListener(v -> {
            CrashLog.clearCrash(this);
            safe = false;
            loadReports();
            requestStage(StreamService.STAGE_FULL);
        });
        inputManager = (InputManager) getSystemService(INPUT_SERVICE);
        loadReports();
        showDetails(safe);                      // safe mode: the report is the point
        if (safe) {
            ensurePermissionsThenStart();       // asks for permissions, starts nothing
        } else {
            requestStage(StreamService.STAGE_FULL);
        }
    }

    private void loadReports() {
        crashReport = CrashLog.readCrash(this);
        prevTrace = CrashLog.readPreviousTrace(this);
    }

    @Override
    protected void onResume() {
        super.onResume();
        inputManager.registerInputDeviceListener(this, ui);
        refreshDevices();
        ui.post(refresh);
    }

    @Override
    protected void onPause() {
        ui.removeCallbacks(refresh);
        inputManager.unregisterInputDeviceListener(this);
        GamepadInput.INSTANCE.focus(false);     // the gamepad goes elsewhere now: back to MSDK
        super.onPause();
    }

    @Override
    public void onBackPressed() {
        if (details.getVisibility() == View.VISIBLE) {
            showDetails(false);
            return;
        }
        super.onBackPressed();
    }

    // --- permissions + service ------------------------------------------------------

    /** The runtime ("dangerous") subset of lis-swarm-app's list; the rest are install-time. */
    private static List<String> runtimePermissions() {
        List<String> p = new ArrayList<>(Arrays.asList(
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.READ_PHONE_STATE));
        if (Build.VERSION.SDK_INT < 30) p.add(Manifest.permission.WRITE_EXTERNAL_STORAGE);
        if (Build.VERSION.SDK_INT < 33) p.add(Manifest.permission.READ_EXTERNAL_STORAGE);
        return p;
    }

    private List<String> missing() {
        List<String> out = new ArrayList<>();
        for (String p : runtimePermissions()) {
            if (ContextCompat.checkSelfPermission(this, p) != PackageManager.PERMISSION_GRANTED) {
                out.add(p.substring(p.lastIndexOf('.') + 1));
            }
        }
        return out;
    }

    /** Start the service up to `stage` (StreamService.STAGE_*), after any permission prompt. */
    private void requestStage(int stage) {
        pendingStage = stage;
        ensurePermissionsThenStart();
    }

    private void ensurePermissionsThenStart() {
        List<String> need = new ArrayList<>();
        for (String p : runtimePermissions()) {
            if (ContextCompat.checkSelfPermission(this, p) != PackageManager.PERMISSION_GRANTED) need.add(p);
        }
        if (need.isEmpty()) {
            if (pendingStage > 0) startStage(pendingStage);
        } else {
            CrashLog.step("requesting permissions: " + need.size());
            ActivityCompat.requestPermissions(this, need.toArray(new String[0]), REQ_PERMS);
        }
    }

    @Override
    public void onRequestPermissionsResult(int code, String[] perms, int[] results) {
        super.onRequestPermissionsResult(code, perms, results);
        List<String> miss = missing();
        CrashLog.step("permissions result, still missing: " + miss);
        if (code == REQ_PERMS && miss.isEmpty() && pendingStage > 0) startStage(pendingStage);
    }

    private void startStage(int stage) {
        CrashLog.step("starting the stream service, stage " + stage);
        startForegroundService(new Intent(this, StreamService.class)
                .putExtra(StreamService.EXTRA_STAGE, stage));
    }

    // --- rendering ------------------------------------------------------------------

    private int dp(float v) {
        return Math.round(v * density);
    }

    private void buildPills() {
        LinearLayout row = findViewById(R.id.buttons);
        for (int i = 0; i < pills.length; i++) {
            TextView t = new TextView(this);
            t.setText(Protocol.BUTTONS[PILL_BUTTONS[i]].toUpperCase(Locale.US));
            t.setTextSize(TypedValue.COMPLEX_UNIT_SP, 13);
            t.setTypeface(Typeface.create("sans-serif-medium", Typeface.NORMAL));
            t.setLetterSpacing(0.08f);
            t.setGravity(Gravity.CENTER);
            t.setMinWidth(dp(52));
            t.setPadding(dp(14), dp(6), dp(14), dp(6));
            GradientDrawable bg = new GradientDrawable();
            bg.setCornerRadius(dp(14));
            t.setBackground(bg);
            LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(
                    LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT);
            lp.setMargins(dp(5), 0, dp(5), 0);
            row.addView(t, lp);
            pills[i] = t;
            pillBg[i] = bg;
            stylePill(i, false);
        }
    }

    private void stylePill(int i, boolean lit) {
        pillBg[i].setColor(lit ? C_ACCENT : C_SURFACE_HI);
        pills[i].setTextColor(lit ? 0xFF0B0E13 : C_DIM);
    }

    private void showDetails(boolean show) {
        details.setVisibility(show ? View.VISIBLE : View.GONE);
        if (show) lastDiag = 0;                 // fill it on the next tick, not in 0.5 s
    }

    private final Runnable refresh = new Runnable() {
        @Override
        public void run() {
            GamepadInput.INSTANCE.uiTick();     // the gamepad path's "this screen is live"
            tickGamepadRate();
            renderSafely();
            ui.postDelayed(this, 100);
        }
    };

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        GamepadInput.INSTANCE.focus(hasFocus);
        CrashLog.step("window focus " + (hasFocus ? "gained" : "lost"));
        if (hasFocus) immersive();
    }

    /**
     * Hide the system bars, sticky: an edge swipe only peeks at them, so a stray swipe does
     * not take focus (and with it the gamepad) away from this screen.
     */
    private void immersive() {
        getWindow().getDecorView().setSystemUiVisibility(View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                | View.SYSTEM_UI_FLAG_FULLSCREEN | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_LAYOUT_STABLE | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION);
    }

    /**
     * Once a second, while a control moves: gamepad reports/s and the stick source (UI
     * thread only). Also traced while MSDK reports motion, so "the gamepad delivered
     * nothing" is an explicit zero in the trace, not an absence.
     */
    private void tickGamepadRate() {
        long now = SystemClock.elapsedRealtime();
        if (now - lastGamepadTick < 1000) return;
        boolean msdkMoving = false;
        for (String k : MOTION_KEYS) {
            Double r = rates.get(k);
            if (r != null && r > 0) msdkMoving = true;
        }
        if ((gamepadReports > 0 || foreignReports > 0 || msdkMoving) && gamepadLinesTraced < 300) {
            gamepadLinesTraced++;
            StreamService svc = StreamService.instance;
            RcInputReader.Snapshot s = svc == null ? null : svc.snapshot();
            CrashLog.step("gamepad reports/s: " + gamepadReports
                    + (s == null || s.gamepad == null ? "" : ", " + gapText(s.gamepad.gapMs))
                    + " (stick source " + (s == null ? "-" : s.stickSrc) + ")"
                    + (foreignReports > 0 ? ", " + foreignReports + " from another joystick (ignored)" : ""));
        }
        gamepadReports = 0;
        foreignReports = 0;
        lastGamepadTick = now;
    }

    /** A display bug must never take the joystick down: it is shown and logged instead. */
    private void renderSafely() {
        try {
            render();
        } catch (RuntimeException e) {
            String msg = e.toString();
            if (!msg.equals(lastUiError)) {
                lastUiError = msg;
                CrashLog.error("UI render", e);
            }
            state(C_BAD, "UI ERROR", msg);
        }
    }

    private void render() {
        long now = SystemClock.elapsedRealtime();
        detailsTitle.setText(getString(R.string.app_name) + " " + version
                + (safe ? "  ·  SAFE MODE" : "") + "  ·  diagnostics");
        safeRow.setVisibility(safe ? View.VISIBLE : View.GONE);
        boolean detailsDue = details.getVisibility() == View.VISIBLE && now - lastDiag >= 500;
        StreamService svc = StreamService.instance;
        if (svc == null) {
            List<String> miss = missing();
            if (!miss.isEmpty()) state(C_WARN, "PERMISSIONS", "missing: " + join(miss));
            else if (safe) state(C_WARN, "SAFE MODE", "nothing started · the stage buttons are in Details");
            else state(C_FAINT, "STARTING", "starting the stream service");
            grant.setVisibility(miss.isEmpty() ? View.GONE : View.VISIBLE);
            retry.setVisibility(View.GONE);
            chips(null, Collections.<UdpStreamServer.Subscriber>emptyList(), null);
            source(null);
            padL.set("LEFT", null, null);
            padR.set("RIGHT", null, null);
            dialL.set(null);
            dialR.set(null);
            for (int i = 0; i < pills.length; i++) pills[i].setVisibility(View.GONE);
            setAlerts(Collections.<Alert>emptyList());
            if (detailsDue) {
                lastDiag = now;
                diag.setText(reportText() + "(the stream service is not running)\n\n"
                        + "THIS RUN, last steps\n" + CrashLog.recent(15));
            }
            return;
        }
        grant.setVisibility(View.GONE);
        RcInputReader.Snapshot s = svc.snapshot();
        UdpStreamServer srv = svc.server();
        List<UdpStreamServer.Subscriber> subs = srv != null ? srv.subscribers()
                : Collections.<UdpStreamServer.Subscriber>emptyList();
        GamepadInput.Reading g = s.gamepad;

        padL.set("LEFT", s.sticks[0], s.sticks[1]);
        padR.set("RIGHT", s.sticks[2], s.sticks[3]);
        dialL.set(s.dials[0]);
        dialR.set(s.dials[1]);
        source(s);
        pills(s, now);

        String reason = s.notOkReason();
        if (svc.serverError != null) {
            state(C_BAD, "ERROR", svc.serverError);
        } else if (!svc.sdkRegistered) {
            state(svc.sdkFailed ? C_BAD : C_WARN, svc.sdkFailed ? "SDK FAILED" : "STARTING", svc.sdkStatus);
        } else if (reason != null) {
            state(C_BAD, s.aircraftLinked ? "BLOCKED" : "NOT OK", reason);
        } else if (subs.isEmpty()) {
            state(C_WARN, "READY", "waiting for the PC");
        } else if (s.slowReason() != null) {
            state(C_WARN, "STREAMING", "sticks via MSDK, ~10 Hz · " + s.slowReason());
        } else if (slowMode(g)) {
            state(C_WARN, "STREAMING", "gamepad in slow mode (~10 reports/s) · reboot the RC");
        } else {
            state(C_OK, "STREAMING", srv.currentRate + " Hz to "
                    + (subs.size() == 1 ? "1 PC" : subs.size() + " PCs"));
        }
        retry.setVisibility(svc.sdkFailed ? View.VISIBLE : View.GONE);
        chips(svc.ethIp(), subs, s.batteryPct);
        setAlerts(alertsFor(svc, s, g));

        if (now - lastRateAt >= 1000) {
            double dt = lastRateAt == 0 ? 0 : (now - lastRateAt) / 1000.0;
            for (RcInputReader.KeyStat k : s.stats) {
                Long prev = prevCounts.get(k.name);
                if (prev != null && dt > 0) rates.put(k.name, (k.count - prev) / dt);
                prevCounts.put(k.name, k.count);
            }
            lastRateAt = now;
        }
        if (detailsDue) {
            lastDiag = now;
            diag.setText(reportText() + diagText(svc, s, srv, subs, now));
        }
    }

    private void state(int color, String word, String detail) {
        dot.setColor(color);
        statusWord.setText(word);
        statusWord.setTextColor(color == C_FAINT ? C_DIM : color);
        statusDetail.setText(detail == null ? "" : detail);
    }

    private void chips(String eth, List<UdpStreamServer.Subscriber> subs, Integer battery) {
        chipNet.setText(eth == null ? "No Ethernet" : eth);
        chipNet.setTextColor(eth == null ? C_BAD : C_DIM);
        chipPc.setText(subs.isEmpty() ? "No PC" : subs.size() == 1 ? "1 PC" : subs.size() + " PCs");
        chipPc.setTextColor(subs.isEmpty() ? C_FAINT : C_OK);
        chipBatt.setVisibility(battery == null ? View.GONE : View.VISIBLE);
        if (battery != null) {
            chipBatt.setText(battery + "%");
            chipBatt.setTextColor(battery < BATTERY_LOW_PCT ? C_BAD : C_DIM);
        }
    }

    /** Which source the sticks come from, and for the gamepad, its report mode and rate. */
    private void source(RcInputReader.Snapshot s) {
        if (s == null) {
            srcLabel.setText("");
            srcMode.setText("—");
            srcMode.setTextColor(C_FAINT);
            srcRate.setText("");
            return;
        }
        GamepadInput.Reading g = s.gamepad;
        if ("gamepad".equals(s.stickSrc) && g != null) {
            srcLabel.setText("GAMEPAD");
            if (g.gapMs >= 0 && g.gapMs <= 30) {
                srcMode.setText("FAST");
                srcMode.setTextColor(C_OK);
            } else if (g.gapMs >= 70) {
                srcMode.setText("SLOW");
                srcMode.setTextColor(C_WARN);
            } else {
                srcMode.setText("—");               // no moving gaps measured yet
                srcMode.setTextColor(C_DIM);
            }
            srcRate.setText(g.hz > 0 ? g.hz + " reports/s" : "idle");
        } else {
            srcLabel.setText("MSDK");
            srcMode.setText("10 Hz");
            srcMode.setTextColor(C_WARN);
            String why = s.slowReason();
            srcRate.setText(why == null ? "" : why.equals("no gamepad report yet")
                    ? "move a stick for the gamepad" : why);
        }
    }

    /** The gamepad path is live but the virtual joystick reports ~100 ms apart (README). */
    private static boolean slowMode(GamepadInput.Reading g) {
        return g != null && g.usable && g.gapMs >= 70;
    }

    private void pills(RcInputReader.Snapshot s, long now) {
        for (int i = 0; i < pills.length; i++) {
            int b = PILL_BUTTONS[i];
            Boolean down = s.buttons[b];
            if (down == null) {                 // not served by this RC
                pills[i].setVisibility(View.GONE);
                continue;
            }
            pills[i].setVisibility(View.VISIBLE);
            int presses = s.presses[b];
            if (pressesSeen[i] && presses > lastPresses[i]) flashUntil[i] = now + PRESS_FLASH_MS;
            lastPresses[i] = presses;
            pressesSeen[i] = true;
            stylePill(i, down || now < flashUntil[i]);
        }
    }

    // --- alerts ---------------------------------------------------------------------

    private static final class Alert {
        final int color;
        final boolean big;
        final String text;

        Alert(int color, boolean big, String text) {
            this.color = color;
            this.big = big;
            this.text = text;
        }
    }

    /** What the operator must act on; empty in normal operation. */
    private List<Alert> alertsFor(StreamService svc, RcInputReader.Snapshot s, GamepadInput.Reading g) {
        List<Alert> out = new ArrayList<>();
        if (s.aircraftLinked) {
            out.add(new Alert(C_BAD, true, getString(R.string.linked_banner, s.linkReason)));
        }
        List<String> rf = svc.rf().warnings();
        if (!rf.isEmpty()) {
            out.add(new Alert(C_BAD, false, "Radio on: " + join(rf)
                    + " · switch off in Settings (and Location → Wi-Fi and Bluetooth scanning)"));
        }
        // 2026-09-28: DJI Fly, started by the home screen at boot, took the RC link from
        // MSDK every few seconds. Swiping it away restarts it; a force-stop keeps it down.
        if (svc.sdkRegistered && s.running && !s.aircraftLinked && !Boolean.TRUE.equals(s.rcConnected)) {
            out.add(new Alert(C_WARN, false, "MSDK has lost the RC. If DJI Fly is running, force-stop it:"
                    + " Settings → Apps → DJI Fly → Force stop (swiping it away restarts it)."));
        }
        if (g != null && Boolean.FALSE.equals(Protocol.gate(g))) {
            out.add(new Alert(C_WARN, false, "DJI's gamepad gate is closed · run enable-gamepad.ps1"
                    + " (until then the sticks come from MSDK, ~10 Hz)"));
        }
        if (g != null) {
            StringBuilder inv = new StringBuilder();
            for (int i = 0; i < GamepadInput.AXES.length; i++) {
                if (g.check[i] == GamepadInput.CHECK_INVERTED) {
                    inv.append(inv.length() > 0 ? ", " : "").append(GamepadInput.AXES[i]);
                }
            }
            if (inv.length() > 0) {
                out.add(new Alert(C_BAD, false, "MSDK reads " + inv + " inverted against the gamepad"
                        + " · withheld on the MSDK path"));
            }
        }
        if (s.batteryPct != null && s.batteryPct < BATTERY_LOW_PCT) {
            out.add(new Alert(C_WARN, false, "RC battery " + s.batteryPct + "%"));
        }
        return out;
    }

    private void setAlerts(List<Alert> list) {
        StringBuilder key = new StringBuilder();
        for (Alert a : list) key.append(a.color).append(a.text).append('\n');
        if (key.toString().equals(alertsKey)) return;
        alertsKey = key.toString();
        alerts.removeAllViews();
        for (Alert a : list) {
            TextView t = new TextView(this);
            t.setText(a.text);
            t.setTextColor(a.color);
            t.setTextSize(TypedValue.COMPLEX_UNIT_SP, a.big ? 15 : 13);
            if (a.big) t.setTypeface(Typeface.create("sans-serif-medium", Typeface.NORMAL));
            t.setPadding(dp(14), dp(a.big ? 10 : 7), dp(14), dp(a.big ? 10 : 7));
            GradientDrawable bg = new GradientDrawable();
            bg.setCornerRadius(dp(12));
            bg.setColor((a.color & 0x00FFFFFF) | 0x1F000000);
            bg.setStroke(dp(1), (a.color & 0x00FFFFFF) | 0x59000000);
            t.setBackground(bg);
            LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(
                    LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);
            lp.bottomMargin = dp(6);
            alerts.addView(t, lp);
        }
        alerts.setVisibility(list.isEmpty() ? View.GONE : View.VISIBLE);
    }

    // --- details ---------------------------------------------------------------------

    /** The crash report and, in safe mode, where the previous run stopped. Empty when clean. */
    private String reportText() {
        StringBuilder b = new StringBuilder();
        if (crashReport != null) {
            b.append("=== LAST CRASH (crash.txt) - press 'Clear report' once read ===\n")
                    .append(crashReport).append('\n');
        }
        if (safe && prevTrace != null) {
            b.append("=== PREVIOUS RUN, last steps (trace.prev.txt) ===\n")
                    .append(tail(prevTrace, 40)).append('\n');
        }
        return b.toString();
    }

    private static String tail(String text, int lines) {
        String[] all = text.split("\n");
        StringBuilder b = new StringBuilder();
        for (int i = Math.max(0, all.length - lines); i < all.length; i++) {
            b.append("  ").append(all[i]).append('\n');
        }
        return b.toString();
    }

    private String diagText(StreamService svc, RcInputReader.Snapshot s, UdpStreamServer srv,
                            List<UdpStreamServer.Subscriber> subs, long now) {
        StringBuilder b = new StringBuilder();
        b.append(svc.sdkStatus).append("   (service stage ").append(svc.stage).append(")\n");
        b.append(String.format(Locale.US, "RC      %s  sn %s  fw %s  battery %s%%  stick %s  pairing %s%n",
                s.rcType, s.sn, s.fw, s.batteryPct, s.stickMode, s.pairing));
        b.append(String.format(Locale.US, "LINK    rc_conn %s  product %s (type %s)  fc %s  still_linked %s%n",
                s.rcConnected, s.productConnected, s.productType, s.fcConnected, s.stillLinked));
        b.append(String.format(Locale.US, "AIRCRAFT %s%n", s.aircraftLinked
                ? "LINKED: " + s.linkReason
                : "not linked (product connected alone is the RC itself on an RC Pro)"));
        b.append(String.format(Locale.US, "rc_ok   %s%s%n", s.rcOk,
                s.rcOk ? "" : "  (" + s.notOkReason() + ")"));
        b.append("WIRE    ").append(wireText(s.sticks, s.dials)).append("   (what the PC gets)\n");
        b.append("BUTTONS ").append(buttonsText(s)).append('\n');
        GamepadInput.Reading g = s.gamepad;
        if (g != null) {
            b.append(String.format(Locale.US, "STICKS  from %s | gamepad %d reports/s, %s, last %s | focus %s | DJI gate: %s%n",
                    sourceText(s), g.hz, gapText(g.gapMs), g.ageMs < 0 ? "never" : g.ageMs + " ms ago",
                    g.focused ? "yes" : "NO", gateText(g)));
            b.append("        gamepad ").append(wireText(g.sticks, g.dials))
                    .append("   msdk ").append(wireText(s.msdkSticks, s.msdkDials)).append('\n');
            b.append("        cross-check vs MSDK (both still): ").append(checkText(g)).append('\n');
        }
        b.append(String.format(Locale.US, "PROBE   liveness read of %s: ok %d fail %d, last %d ms, age %d ms%s%n",
                s.probeKey, s.probeOk, s.probeFail, s.probeLatencyMs, s.probeAgeMs,
                s.probeError == null ? "" : ", last err " + s.probeError));
        if (s.probeUnsupported) {
            b.append("        no async read answers on this RC: rc_ok uses KeyConnection alone\n");
        }
        b.append(String.format(Locale.US, "        stick read: ok %d fail %d, last %d ms%s%n",
                s.stickProbeOk, s.stickProbeFail, s.stickProbeLatencyMs,
                s.stickProbeError == null ? "" : ", err " + s.stickProbeError));
        RfStatus rf = svc.rf();
        b.append(String.format(Locale.US, "RF      wifi %s  wifi_scan %s  bt %s  ble_scan %s  airplane %s%n",
                rf.wifi, rf.wifiScan, rf.bt, rf.bleScan, rf.airplane));
        if (srv != null) {
            b.append(String.format(Locale.US, "STREAM  %s:%d  rx %d  tx %d  rejected %d  rate %d Hz%s%n",
                    svc.ethIp(), Protocol.PORT, srv.rxPackets, srv.txPackets, srv.rejected, srv.currentRate,
                    srv.lastError == null ? "" : "  last error: " + srv.lastError));
        }
        b.append("SUBSCRIBERS").append(subs.isEmpty() ? "  none\n" : "\n");
        for (UdpStreamServer.Subscriber sub : subs) {
            b.append(String.format(Locale.US, "  %s  %s  %d Hz  for %d s, seen %.1f s ago%n",
                    sub.addr, sub.client, sub.rate, (now - sub.since) / 1000,
                    (now - sub.lastSeen) / 1000.0));
        }
        b.append("\nKEYS  name          group   callbacks  /s   age    seed  value\n");
        for (RcInputReader.KeyStat k : s.stats) {
            Double r = rates.get(k.name);
            b.append(String.format(Locale.US, "  %-13s %-7s %9d %5s %6s  %-4s  %s%n",
                    k.name, k.group, k.count, r == null ? "-" : String.format(Locale.US, "%.0f", r),
                    k.lastMs == 0 ? "never" : (now - k.lastMs) + "ms", k.seeded ? "yes" : "",
                    k.value));
        }
        b.append("\nANDROID INPUT DEVICES (a gamepad here = a no-MSDK fallback)\n").append(devicesText);
        b.append("\nLAST INPUT EVENTS\n");
        for (String e : events) b.append("  ").append(e).append('\n');
        b.append("\nTHIS RUN, last steps (trace.txt)\n").append(CrashLog.recent(15));
        String err = CrashLog.lastError();
        if (err != null) b.append("last non-fatal error: ").append(err).append('\n');
        b.append(CrashLog.build()).append('\n');
        return b.toString();
    }

    /** Every button: level and press count, n/a when this RC does not serve it. */
    private static String buttonsText(RcInputReader.Snapshot s) {
        StringBuilder b = new StringBuilder();
        for (int i = 0; i < Protocol.BUTTONS.length; i++) {
            Boolean down = s.buttons[i];
            b.append(Protocol.BUTTONS[i]).append(' ')
                    .append(down == null ? "n/a" : (down ? "DOWN" : "up") + "/" + s.presses[i]).append("  ");
        }
        return b.toString().trim();
    }

    private static String sourceText(RcInputReader.Snapshot s) {
        String slow = s.slowReason();
        return slow == null ? "sticks via GAMEPAD" : "sticks via MSDK, ~10 Hz (" + slow + ")";
    }

    /** The virtual joystick's mode, from its median report gap (GamepadInput.MOVING_GAP_MS). */
    static String gapText(int gapMs) {
        if (gapMs < 0) return "report gap unknown yet";
        return "median gap " + gapMs + " ms (" + (gapMs <= 30 ? "fast mode" : gapMs >= 70 ? "SLOW mode" : "?") + ")";
    }

    private static String gateText(GamepadInput.Reading g) {
        return "game mode " + onOff(g.gameMode) + ", left stick " + onOff(g.motionLeft)
                + ", right stick " + onOff(g.motionRight);
    }

    private static String onOff(Boolean b) {
        return b == null ? "?" : b ? "on" : "OFF";
    }

    private static String wireText(int[] sticks, int[] dials) {
        Integer[] s = new Integer[sticks.length];
        Integer[] d = new Integer[dials.length];
        for (int i = 0; i < s.length; i++) s[i] = sticks[i];
        for (int i = 0; i < d.length; i++) d[i] = dials[i];
        return wireText(s, d);
    }

    private static String wireText(Integer[] sticks, Integer[] dials) {
        StringBuilder b = new StringBuilder();
        for (int i = 0; i < sticks.length; i++) {
            b.append(Protocol.STICKS[i]).append(' ').append(sticks[i] == null ? "null"
                    : String.format(Locale.US, "%+d", sticks[i])).append(' ');
        }
        for (int i = 0; i < dials.length; i++) {
            b.append(Protocol.DIALS[i]).append(' ').append(dials[i] == null ? "null"
                    : String.format(Locale.US, "%+d", dials[i])).append(' ');
        }
        return b.toString().trim();
    }

    private static String checkText(GamepadInput.Reading g) {
        StringBuilder b = new StringBuilder();
        for (int i = 0; i < GamepadInput.AXES.length; i++) {
            String v;
            switch (g.check[i]) {
                case GamepadInput.CHECK_OK:
                    v = "ok";
                    break;
                case GamepadInput.CHECK_CENTRE:
                    v = "centre";
                    break;
                case GamepadInput.CHECK_INVERTED:
                    v = "INVERTED";
                    break;
                default:
                    v = "-";
            }
            b.append(GamepadInput.AXES[i]).append(' ').append(v);
            if (g.check[i] != GamepadInput.CHECK_UNTESTED) b.append(" (max diff ").append(g.checkDiff[i]).append(')');
            b.append("  ");
        }
        return b.toString().trim();
    }

    private static String join(List<String> parts) {
        StringBuilder b = new StringBuilder();
        for (String p : parts) {
            if (b.length() > 0) b.append(", ");
            b.append(p);
        }
        return b.toString();
    }

    // --- Android input diagnostics (spike S2/S4) --------------------------------------

    private void refreshDevices() {
        StringBuilder b = new StringBuilder();
        for (int id : InputDevice.getDeviceIds()) {
            InputDevice d = InputDevice.getDevice(id);
            if (d == null) continue;
            int src = d.getSources();
            b.append(String.format(Locale.US, "  #%d %s  src=0x%x%s%s%s%s%n", id, d.getName(), src,
                    has(src, InputDevice.SOURCE_JOYSTICK) ? " JOYSTICK" : "",
                    has(src, InputDevice.SOURCE_GAMEPAD) ? " GAMEPAD" : "",
                    has(src, InputDevice.SOURCE_DPAD) ? " DPAD" : "",
                    has(src, InputDevice.SOURCE_KEYBOARD) ? " KEYBOARD" : ""));
            List<InputDevice.MotionRange> ranges = d.getMotionRanges();
            if (!ranges.isEmpty()) {
                b.append("     axes:");
                for (InputDevice.MotionRange r : ranges) {
                    b.append(' ').append(MotionEvent.axisToString(r.getAxis()).replace("AXIS_", ""));
                }
                b.append('\n');
            }
        }
        devicesText = b.length() == 0 ? "  none\n" : b.toString();
        // Into trace.txt too, when it changes: if MSDK serves no RC keys without an aircraft
        // (REQUEST_HANDLER_NOT_FOUND for component 4 on product 0, 2026-09-25), an Android
        // gamepad exposing the sticks is the no-MSDK fallback, and this is the evidence.
        if (!devicesText.equals(tracedDevices)) {
            tracedDevices = devicesText;
            CrashLog.step("android input devices:\n" + devicesText.replaceAll("(?m)\\s+$", ""));
        }
    }

    private static boolean has(int sources, int source) {
        return (sources & source) == source;
    }

    private void addEvent(String e) {
        events.addFirst(clock.format(new Date()) + " " + e);
        while (events.size() > 12) events.removeLast();
    }

    private static String devName(int deviceId) {
        InputDevice d = InputDevice.getDevice(deviceId);
        return d == null ? String.valueOf(deviceId) : d.getName();
    }

    @Override
    public boolean dispatchKeyEvent(KeyEvent e) {
        String action = e.getAction() == KeyEvent.ACTION_DOWN ? "down"
                : e.getAction() == KeyEvent.ACTION_UP ? "up" : "multi";
        addEvent("key " + KeyEvent.keyCodeToString(e.getKeyCode()) + " " + action
                + " dev=" + devName(e.getDeviceId()));
        // D-pad keys from a joystick (Android synthesizes them from unconsumed stick motion)
        // must never move focus around this screen: swallowed.
        if (isFromJoystick(e.getSource()) && isDpad(e.getKeyCode())) return true;
        return super.dispatchKeyEvent(e);
    }

    private static boolean isFromJoystick(int source) {
        return has(source, InputDevice.SOURCE_JOYSTICK) || has(source, InputDevice.SOURCE_GAMEPAD);
    }

    private static boolean isDpad(int k) {
        return k == KeyEvent.KEYCODE_DPAD_UP || k == KeyEvent.KEYCODE_DPAD_DOWN
                || k == KeyEvent.KEYCODE_DPAD_LEFT || k == KeyEvent.KEYCODE_DPAD_RIGHT
                || k == KeyEvent.KEYCODE_DPAD_CENTER;
    }

    /**
     * The RC's gamepad: every report goes to GamepadInput, and the event is CONSUMED, so
     * Android never synthesizes D-pad navigation from it. Another joystick (a USB gamepad
     * plugged into the RC, say) is consumed too, and drives nothing.
     */
    @Override
    public boolean dispatchGenericMotionEvent(MotionEvent e) {
        if (!has(e.getSource(), InputDevice.SOURCE_JOYSTICK)) return super.dispatchGenericMotionEvent(e);
        InputDevice d = e.getDevice();
        int samples = 1 + e.getHistorySize();           // batched samples are reports too
        if (d != null && d.getVendorId() == GAMEPAD_VENDOR && d.getProductId() == GAMEPAD_PRODUCT) {
            gamepadReports += samples;
            long[] times = new long[samples];           // the kernel's timestamps, oldest first
            for (int h = 0; h < samples - 1; h++) times[h] = e.getHistoricalEventTime(h);
            times[samples - 1] = e.getEventTime();
            GamepadInput.INSTANCE.report(e.getAxisValue(MotionEvent.AXIS_X), e.getAxisValue(MotionEvent.AXIS_Y),
                    e.getAxisValue(MotionEvent.AXIS_RX), e.getAxisValue(MotionEvent.AXIS_RY),
                    e.getAxisValue(MotionEvent.AXIS_Z), e.getAxisValue(MotionEvent.AXIS_RZ), times);
        } else {
            foreignReports += samples;
        }
        logMotion(e);
        return true;
    }

    private void logMotion(MotionEvent e) {
        long now = SystemClock.uptimeMillis();
        if (now - lastMotionLog >= 200) {           // at most 5 lines a second
            lastMotionLog = now;
            StringBuilder b = new StringBuilder("motion dev=" + devName(e.getDeviceId())
                    + " src=0x" + Integer.toHexString(e.getSource()));
            InputDevice d = e.getDevice();
            if (d != null) {
                for (InputDevice.MotionRange r : d.getMotionRanges()) {
                    float v = e.getAxisValue(r.getAxis());
                    if (Math.abs(v) > 0.01f) {
                        b.append(' ').append(MotionEvent.axisToString(r.getAxis()).replace("AXIS_", ""))
                                .append('=').append(String.format(Locale.US, "%.2f", v));
                    }
                }
            }
            addEvent(b.toString());
        }
    }

    @Override
    public void onInputDeviceAdded(int deviceId) {
        addEvent("device added: " + devName(deviceId));
        refreshDevices();
    }

    @Override
    public void onInputDeviceRemoved(int deviceId) {
        addEvent("device removed: #" + deviceId);
        refreshDevices();
    }

    @Override
    public void onInputDeviceChanged(int deviceId) {
        refreshDevices();
    }
}
