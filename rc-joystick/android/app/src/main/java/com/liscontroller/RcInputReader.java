package com.liscontroller;

import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.util.Log;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import dji.sdk.keyvalue.key.DJIKey;
import dji.sdk.keyvalue.key.DJIKeyInfo;
import dji.sdk.keyvalue.key.FlightControllerKey;
import dji.sdk.keyvalue.key.KeyTools;
import dji.sdk.keyvalue.key.ProductKey;
import dji.sdk.keyvalue.key.RemoteControllerKey;
import dji.sdk.keyvalue.value.remotecontroller.BatteryInfo;
import dji.sdk.keyvalue.value.remotecontroller.ControlMode;
import dji.sdk.keyvalue.value.remotecontroller.FiveDimensionPressedStatus;
import dji.sdk.keyvalue.value.remotecontroller.RCFlightModeSwitch;
import dji.sdk.keyvalue.value.remotecontroller.RcCustomButtonHardwareStatus;
import dji.v5.common.callback.CommonCallbacks;
import dji.v5.common.error.IDJIError;
import dji.v5.manager.KeyManager;

/**
 * Reads the RC's sticks, dials and buttons through MSDK. It is READ-ONLY by
 * construction: only listen() and getValue() are ever called, never setValue() or
 * performAction(), so this app cannot change the RC's radio, pairing or anything else.
 *
 * Listener callbacks arrive on the main thread and only update fields under `lock`.
 * The stream's tx thread and the UI take copies with snapshot(). No I/O happens here.
 *
 * snapshot() also merges in the RC's built-in gamepad (GamepadInput): while it is usable,
 * the sticks and dials come from it at ~70 Hz instead of MSDK's ~10 Hz. MSDK keeps
 * everything else, and its stick changes are the witness that the gamepad still delivers.
 */
final class RcInputReader {
    static final String TAG = "LIS_CONTROLLER";

    /**
     * Sign applied to each raw value so the wire convention holds: + = up / right /
     * clockwise (PROTOCOL.md). All +1 until the hardware spike (S2) shows otherwise;
     * the PC never flips a sign, so a correction belongs here.
     */
    private static final int[] STICK_SIGN = {1, 1, 1, 1};
    private static final int[] DIAL_SIGN = {1, 1};

    static final long PROBE_PERIOD_MS = 200;
    /** rc_ok needs a successful liveness round-trip this recent. */
    static final long PROBE_FRESH_MS = 500;
    /** An MSDK stick/dial change this big (wire units, ~1.5%) is real motion, not ADC noise. */
    static final int WITNESS_MIN_DELTA = 10;

    interface Waker {
        void wake();
    }

    /** Per-key counters for the diagnostics table and info.keys. */
    static final class KeyStat {
        final String name;
        final String group;
        final boolean wire;         // reported in info.keys
        long count;                 // listener callbacks since start
        long lastMs;                // elapsedRealtime of the last callback, 0 = never
        boolean seeded;             // a cached value existed at start
        boolean errored;            // a callback failed (logged once)
        boolean everSet;            // any value yet (the first one is traced)
        boolean traceChanges;       // trace every change (link/status keys: rare, decisive)
        String value = "-";

        KeyStat(String name, String group, boolean wire) {
            this.name = name;
            this.group = group;
            this.wire = wire;
        }

        KeyStat copy() {
            KeyStat k = new KeyStat(name, group, wire);
            k.count = count;
            k.lastMs = lastMs;
            k.seeded = seeded;
            k.value = value;
            return k;
        }
    }

    /** An immutable copy of everything the stream and the UI need. */
    static final class Snapshot {
        final Integer[] sticks = new Integer[4];    // what goes on the wire
        final Integer[] dials = new Integer[2];
        final Integer[] msdkSticks = new Integer[4];  // MSDK's own values, for the display
        final Integer[] msdkDials = new Integer[2];
        String stickSrc = "msdk";                   // "gamepad" or "msdk"
        GamepadInput.Reading gamepad;
        final Boolean[] buttons = new Boolean[Protocol.BUTTONS.length];
        final int[] presses = new int[Protocol.BUTTONS.length];
        String modeSwitch;
        boolean running;
        Boolean rcConnected, productConnected, fcConnected, stillLinked;
        boolean aircraftLinked, productIsAircraft, probeFresh, probeUnsupported, rcOk;
        String linkReason;          // what made aircraftLinked true, in words
        String rcType, sn, fw, stickMode, pairing, productType;
        Integer batteryPct;
        String probeKey;
        long probeOk, probeFail, probeLatencyMs = -1, probeAgeMs = -1;
        String probeError;
        long stickProbeOk, stickProbeFail, stickProbeLatencyMs = -1;
        String stickProbeError;
        List<KeyStat> stats = new ArrayList<>();

        /** Why rc_ok is false, in words; null when it is true. */
        String notOkReason() {
            if (!running) return "MSDK not registered yet";
            if (aircraftLinked) return "AIRCRAFT LINKED (" + linkReason + ")";
            if (!Boolean.TRUE.equals(rcConnected)) return "MSDK reports the RC not connected";
            if (!probeUnsupported && !probeFresh) {
                return "liveness probe failing" + (probeError != null ? " (" + probeError + ")" : "");
            }
            return null;
        }

        /** Why the sticks are not on the fast (gamepad) path, in words; null when they are. */
        String slowReason() {
            if ("gamepad".equals(stickSrc)) return null;
            return gamepad == null || gamepad.whyNot == null ? "gamepad not started" : gamepad.whyNot;
        }
    }

    /**
     * Whether MSDK's product type names an actual aircraft. On an RC Pro with NO aircraft,
     * ProductKey.KeyConnection is TRUE anyway: MSDK counts the RC's own link as product
     * UNRECOGNIZED (onProductConnect(0), seen 2026-09-25 on an RM510). So "product
     * connected" alone must never mean "aircraft linked".
     */
    static boolean isAircraftType(String t) {
        return t != null && !t.equals("UNRECOGNIZED") && !t.equals("UNKNOWN")
                && !t.startsWith("NOT_SUPPORTED") && !t.equals("DRTK_2");
    }

    private interface Sink<T> {
        void set(T v);
    }

    private final Object lock = new Object();
    private final Handler main = new Handler(Looper.getMainLooper());
    private final Waker waker;
    private final Map<String, KeyStat> stats = new LinkedHashMap<>();
    private boolean running;
    private boolean pendingWake;
    private int probeTicks;
    private int failedKeys;
    private final Map<String, Long> lastLoggedCounts = new LinkedHashMap<>();

    // Liveness probe candidates, tried in order until one answers (see `probe`). All
    // guarded by lock: replies arrive on MSDK's threads.
    private final List<DJIKey<?>> probeKeys = new ArrayList<>();
    private final List<String> probeNames = new ArrayList<>();
    private int probeIdx;
    private int candidateFails, candidateSent, candidateReplied;
    private boolean probeEverOk, probeUnsupported;
    private boolean stickProbeTraced;

    // Latest values, guarded by lock. Sticks/dials are already in the wire's sign convention.
    private final Integer[] sticks = new Integer[4];
    private final Integer[] dials = new Integer[2];
    private final long[] axisChangeMs = new long[6];   // last MSDK change: lh lv rh rv l r
    private final Boolean[] buttons = new Boolean[Protocol.BUTTONS.length];
    private final int[] presses = new int[Protocol.BUTTONS.length];
    private String modeSwitch;
    private Boolean rcConnected, productConnected, fcConnected, stillLinked;
    private String rcType, sn, fw, stickMode, pairing, productType;
    private Integer batteryPct;
    private long probeOkAt, probeOk, probeFail, probeLatencyMs = -1;
    private String probeError;
    private long stickProbeOk, stickProbeFail, stickProbeLatencyMs = -1;
    private String stickProbeError;

    RcInputReader(Waker waker) {
        this.waker = waker;
    }

    /** Main thread, after MSDK registration. */
    void start() {
        if (running) return;
        running = true;
        CrashLog.step("reader: registering read-only key listeners");

        // --- what the wire carries -------------------------------------------------
        watch("lh", "stick", true, RemoteControllerKey.KeyStickLeftHorizontal, v -> axis(0, signed(v, STICK_SIGN[0])));
        watch("lv", "stick", true, RemoteControllerKey.KeyStickLeftVertical, v -> axis(1, signed(v, STICK_SIGN[1])));
        watch("rh", "stick", true, RemoteControllerKey.KeyStickRightHorizontal, v -> axis(2, signed(v, STICK_SIGN[2])));
        watch("rv", "stick", true, RemoteControllerKey.KeyStickRightVertical, v -> axis(3, signed(v, STICK_SIGN[3])));
        watch("l", "dial", true, RemoteControllerKey.KeyLeftDial, v -> axis(4, signed(v, DIAL_SIGN[0])));
        watch("r", "dial", true, RemoteControllerKey.KeyRightDial, v -> axis(5, signed(v, DIAL_SIGN[1])));
        watch("c1", "button", true, RemoteControllerKey.KeyCustomButton1Down, v -> button(0, v));
        watch("c2", "button", true, RemoteControllerKey.KeyCustomButton2Down, v -> button(1, v));
        watch("c3", "button", true, RemoteControllerKey.KeyCustomButton3Down, v -> button(2, v));
        watch("shutter", "button", true, RemoteControllerKey.KeyShutterButtonDown, v -> button(3, v));
        watch("record", "button", true, RemoteControllerKey.KeyRecordButtonDown, v -> button(4, v));
        watch("rth", "button", true, RemoteControllerKey.KeyGoHomeButtonDown, v -> button(5, v));
        watch("pause", "button", true, RemoteControllerKey.KeyPauseButtonDown, v -> button(6, v));
        watch("rc_switch", "button", true, RemoteControllerKey.KeyRCSwitchButtonDown, v -> button(7, v));
        watch("5d", "button", true, RemoteControllerKey.KeyFiveDimensionPressedStatus, this::fiveD);
        watch("mode_switch", "switch", true, RemoteControllerKey.KeyFlightModeSwitchState,
                v -> modeSwitch = modeName(v));
        watch("rc_conn", "status", true, RemoteControllerKey.KeyConnection, v -> rcConnected = v);

        // --- status / info ---------------------------------------------------------
        watch("rc_type", "info", false, RemoteControllerKey.KeyRemoteControllerType,
                v -> rcType = v == null ? null : v.name());
        watch("sn", "info", false, RemoteControllerKey.KeySerialNumber, v -> sn = v);
        watch("fw", "info", false, RemoteControllerKey.KeyFirmwareVersion, v -> fw = v);
        watch("battery", "info", false, RemoteControllerKey.KeyBatteryInfo,
                v -> batteryPct = v == null ? null : v.getBatteryPercent());
        watch("stick_mode", "info", false, RemoteControllerKey.KeyControlMode,
                v -> stickMode = stickModeName(v));
        watch("pairing", "link", false, RemoteControllerKey.KeyPairingStatus,
                v -> pairing = v == null ? null : v.name());
        // The interlock: either one true means an aircraft is linked to THIS RC.
        watch("product_conn", "link", false, ProductKey.KeyConnection, v -> productConnected = v);
        watch("fc_conn", "link", false, FlightControllerKey.KeyConnection, v -> fcConnected = v);
        watch("product_type", "link", false, ProductKey.KeyProductType,
                v -> productType = v == null ? null : v.name());

        // --- hardware-spike diagnostics: shown on screen, NOT on the wire. -----------
        // "wheel" is reported in info.keys, because whether the RC Pro's right-hand
        // control reports as KeyRightDial or KeyScrollWheel is an open question (S2).
        watch("wheel", "spike", true, RemoteControllerKey.KeyScrollWheel, v -> { });
        watch("still_linked", "spike", false, RemoteControllerKey.KeyIsStillLinked, v -> stillLinked = v);
        watch("lh_normal", "spike", false, RemoteControllerKey.KeyIsLeftHorizontalStickNormal, v -> { });
        watch("lv_normal", "spike", false, RemoteControllerKey.KeyIsLeftVerticalStickNormal, v -> { });
        watch("rh_normal", "spike", false, RemoteControllerKey.KeyIsRightHorizontalStickNormal, v -> { });
        watch("rv_normal", "spike", false, RemoteControllerKey.KeyIsRightVerticalStickNormal, v -> { });
        watch("hw_state", "spike", false, RemoteControllerKey.KeyRcHardwareState, v -> { });
        watch("c4", "spike", false, RemoteControllerKey.KeyRCCustomButton4Down, v -> { });
        watch("playback", "spike", false, RemoteControllerKey.KeyRCPlaybackButtonDown, v -> { });
        watch("rwheel_btn", "spike", false, RemoteControllerKey.KeyRCRightWheelButtonDown, v -> { });
        watch("shutter_long", "spike", false, RemoteControllerKey.KeyRCShutterButtonLongPress, v -> { });
        watch("rc_mode", "spike", false, RemoteControllerKey.KeyRcMachineMode, v -> { });
        watch("usb_mode", "spike", false, RemoteControllerKey.KeyRCUSBMode, v -> { });

        int n;
        synchronized (lock) {
            n = stats.size();
        }
        CrashLog.step("reader: " + n + " keys registered, " + failedKeys + " failed");

        // Liveness candidates, cheapest first. KeyConnection's async read was rejected on
        // the first RC Pro run (2026-09-25), so the others are there to find one this
        // firmware answers. If none ever does, rc_ok falls back to KeyConnection alone.
        addProbe("KeyConnection", RemoteControllerKey.KeyConnection);
        addProbe("KeySerialNumber", RemoteControllerKey.KeySerialNumber);
        addProbe("KeyBatteryInfo", RemoteControllerKey.KeyBatteryInfo);
        addProbe("KeyStickLeftHorizontal", RemoteControllerKey.KeyStickLeftHorizontal);
        main.post(probe);
        main.postDelayed(summary, 5000);
    }

    private void addProbe(String name, DJIKeyInfo<?> info) {
        try {
            DJIKey<?> k = KeyTools.createKey(info);
            synchronized (lock) {
                probeKeys.add(k);
                probeNames.add(name);
            }
        } catch (RuntimeException | LinkageError e) {
            CrashLog.error("probe key " + name, e);
        }
    }

    /** Main thread. */
    void stop() {
        if (!running) return;
        running = false;
        main.removeCallbacks(probe);
        main.removeCallbacks(summary);
        try {
            KeyManager.getInstance().cancelListen(this);
        } catch (RuntimeException | LinkageError e) {
            CrashLog.error("reader cancelListen", e);
        }
        CrashLog.step("reader: listeners cancelled");
    }

    Snapshot snapshot(boolean withStats) {
        long now = SystemClock.elapsedRealtime();
        Snapshot s = new Snapshot();
        long[] changes = new long[axisChangeMs.length];
        synchronized (lock) {
            System.arraycopy(sticks, 0, s.sticks, 0, sticks.length);
            System.arraycopy(dials, 0, s.dials, 0, dials.length);
            System.arraycopy(axisChangeMs, 0, changes, 0, changes.length);
            System.arraycopy(buttons, 0, s.buttons, 0, buttons.length);
            System.arraycopy(presses, 0, s.presses, 0, presses.length);
            s.modeSwitch = modeSwitch;
            s.running = running;
            s.rcConnected = rcConnected;
            s.productConnected = productConnected;
            s.fcConnected = fcConnected;
            s.stillLinked = stillLinked;
            s.rcType = rcType;
            s.sn = sn;
            s.fw = fw;
            s.stickMode = stickMode;
            s.pairing = pairing;
            s.productType = productType;
            s.batteryPct = batteryPct;
            s.probeOk = probeOk;
            s.probeFail = probeFail;
            s.probeLatencyMs = probeLatencyMs;
            s.probeAgeMs = probeOkAt > 0 ? now - probeOkAt : -1;
            s.probeError = probeError;
            s.probeUnsupported = probeUnsupported;
            s.probeKey = probeUnsupported ? "none answers"
                    : probeIdx < probeNames.size() ? probeNames.get(probeIdx) : "-";
            s.stickProbeOk = stickProbeOk;
            s.stickProbeFail = stickProbeFail;
            s.stickProbeLatencyMs = stickProbeLatencyMs;
            s.stickProbeError = stickProbeError;
            if (withStats) {
                for (KeyStat k : stats.values()) s.stats.add(k.copy());
            }
        }
        // The interlock: this RC's sticks are flying an aircraft. The flight controller's
        // connection is the aircraft's own link; a product type naming an aircraft is the
        // second, independent sign. NOT ProductKey.KeyConnection alone: see isAircraftType.
        s.productIsAircraft = Boolean.TRUE.equals(s.productConnected) && isAircraftType(s.productType);
        boolean fc = Boolean.TRUE.equals(s.fcConnected);
        s.aircraftLinked = fc || s.productIsAircraft;
        s.linkReason = fc && s.productIsAircraft ? "flight controller connected, product " + s.productType
                : fc ? "flight controller connected"
                : s.productIsAircraft ? "product type " + s.productType : null;
        // Liveness: a fresh async read of the RC, unless this firmware answers none of the
        // candidates, in which case MSDK's own RC connection state is all there is.
        s.probeFresh = s.probeAgeMs >= 0 && s.probeAgeMs < PROBE_FRESH_MS;
        boolean live = s.probeUnsupported || s.probeFresh;
        s.rcOk = s.running && Boolean.TRUE.equals(s.rcConnected) && live && !s.aircraftLinked;
        mergeGamepad(s, changes, now);
        return s;
    }

    /**
     * The sticks and dials from the gamepad while it is usable, else MSDK's. On the MSDK
     * path an axis the cross-check found INVERTED is withheld (null: the PC then has no
     * joystick) rather than flown backwards. Outside `lock`: GamepadInput has its own.
     */
    private static void mergeGamepad(Snapshot s, long[] msdkChangeMs, long now) {
        System.arraycopy(s.sticks, 0, s.msdkSticks, 0, s.sticks.length);
        System.arraycopy(s.dials, 0, s.msdkDials, 0, s.dials.length);
        GamepadInput g = GamepadInput.INSTANCE;
        g.crossCheck(s.msdkSticks, s.msdkDials, msdkChangeMs, now);
        GamepadInput.Reading r = g.read(now);
        s.gamepad = r;
        if (r.usable) {
            s.stickSrc = "gamepad";
            for (int i = 0; i < s.sticks.length; i++) s.sticks[i] = r.sticks[i];
            for (int i = 0; i < s.dials.length; i++) s.dials[i] = r.dials[i];
        } else {
            s.stickSrc = "msdk";
            for (int i = 0; i < s.sticks.length; i++) {
                if (r.check[i] == GamepadInput.CHECK_INVERTED) s.sticks[i] = null;
            }
            for (int i = 0; i < s.dials.length; i++) {
                if (r.check[4 + i] == GamepadInput.CHECK_INVERTED) s.dials[i] = null;
            }
        }
    }

    // --- internals -----------------------------------------------------------------

    private <T> void watch(String name, String group, boolean wire, DJIKeyInfo<T> info,
                           final Sink<T> sink) {
        final KeyStat st = new KeyStat(name, group, wire);
        st.traceChanges = group.equals("link") || group.equals("status") || name.equals("still_linked");
        synchronized (lock) {
            stats.put(name, st);
        }
        // One key this firmware lacks, or that MSDK chokes on, must not abort every
        // registration after it, and certainly not the app: it is recorded and shown.
        try {
            final DJIKey<T> key = KeyTools.createKey(info);
            // Seed from MSDK's cache FIRST, then listen: a listener callback can then
            // never be overwritten by an older cached value.
            try {
                T seed = KeyManager.getInstance().getValue(key);
                if (seed != null) apply(st, sink, seed, false);
            } catch (RuntimeException e) {
                Log.w(TAG, "seed " + name + ": " + e);
            }
            KeyManager.getInstance().listen(key, this, new CommonCallbacks.KeyListener<T>() {
                @Override
                public void onValueChange(T oldValue, T newValue) {
                    apply(st, sink, newValue, true);
                }
            });
        } catch (RuntimeException | LinkageError e) {
            failedKeys++;
            CrashLog.error("key " + name, e);
            synchronized (lock) {
                st.value = "FAILED: " + e.getClass().getSimpleName();
            }
        }
    }

    private <T> void apply(KeyStat st, Sink<T> sink, T v, boolean fromListener) {
        boolean wake, first;
        String before, after;
        RuntimeException failure = null;
        synchronized (lock) {
            if (fromListener) {
                st.count++;
                st.lastMs = SystemClock.elapsedRealtime();
            } else {
                st.seeded = true;
            }
            before = st.value;
            first = !st.everSet;
            st.everSet = true;
            try {
                st.value = describe(v);
                sink.set(v);
            } catch (RuntimeException e) {
                // A bug in handling one value must not crash a joystick in flight: the
                // key shows ERR, and the first failure per key is logged with its stack.
                st.value = "ERR " + e.getClass().getSimpleName();
                if (!st.errored) {
                    st.errored = true;
                    failure = e;
                }
            }
            after = st.value;
            wake = pendingWake;
            pendingWake = false;
        }
        // Into trace.txt: every key's first value (is it served at all?) and every change
        // of a link/status key (rare, and exactly what decides the interlock).
        if (first) {
            CrashLog.step("key " + st.name + " first value" + (fromListener ? "" : " (MSDK cache)")
                    + ": " + after);
        } else if (st.traceChanges && !after.equals(before)) {
            CrashLog.step("key " + st.name + ": " + before + " -> " + after);
        }
        if (failure != null) CrashLog.error("key " + st.name + " callback", failure);
        // Outside the lock: the waker takes the stream's own lock.
        if (wake && waker != null) waker.wake();
    }

    /**
     * Under lock. A stick (0-3) or dial (4-5) value from MSDK. Its change time feeds the
     * gamepad cross-check, and a real change is the witness GamepadInput needs to notice a
     * gamepad that stopped delivering.
     */
    private void axis(int i, Integer v) {
        Integer prev = i < 4 ? sticks[i] : dials[i - 4];
        if (i < 4) sticks[i] = v;
        else dials[i - 4] = v;
        if (v != null && prev != null && !v.equals(prev)) {
            long now = SystemClock.elapsedRealtime();
            axisChangeMs[i] = now;
            if (Math.abs(v - prev) >= WITNESS_MIN_DELTA) GamepadInput.INSTANCE.msdkMotion(now);
        }
    }

    /** Under lock. A rising edge bumps the press count and asks the stream to send early. */
    private void button(int i, Boolean v) {
        Boolean prev = buttons[i];
        buttons[i] = v;
        if (Boolean.TRUE.equals(v) && !Boolean.TRUE.equals(prev)) {
            presses[i]++;
            pendingWake = true;
        }
    }

    private void fiveD(FiveDimensionPressedStatus v) {
        if (v == null) {
            for (int i = 8; i <= 12; i++) buttons[i] = null;
            return;
        }
        button(8, bool(v.getUpwards()));
        button(9, bool(v.getDownwards()));
        button(10, bool(v.getLeftwards()));
        button(11, bool(v.getRightwards()));
        button(12, bool(v.getMiddlePressed()));
    }

    private static Boolean bool(Boolean b) {
        return b == null ? Boolean.FALSE : b;
    }

    private static Integer signed(Integer v, int sign) {
        return v == null ? null : v * sign;
    }

    static String modeName(RCFlightModeSwitch v) {
        if (v == null) return null;
        switch (v) {
            case SWITCH_ONE:
                return "ONE";
            case SWITCH_TWO:
                return "TWO";
            case SWITCH_THREE:
                return "THREE";
            default:
                return null;
        }
    }

    /** DJI names the stick modes after hands: JP = Mode 1, USA = Mode 2, CH = Mode 3. */
    static String stickModeName(ControlMode m) {
        if (m == null) return null;
        switch (m) {
            case JP:
                return "MODE_1";
            case USA:
                return "MODE_2";
            case CH:
                return "MODE_3";
            case CUSTOM:
                return "CUSTOM";
            default:
                return null;
        }
    }

    private static String describe(Object v) {
        if (v == null) return "null";
        if (v instanceof FiveDimensionPressedStatus) {
            FiveDimensionPressedStatus f = (FiveDimensionPressedStatus) v;
            return flag("U", f.getUpwards()) + flag("D", f.getDownwards())
                    + flag("L", f.getLeftwards()) + flag("R", f.getRightwards())
                    + flag("M", f.getMiddlePressed());
        }
        if (v instanceof RcCustomButtonHardwareStatus) {
            RcCustomButtonHardwareStatus h = (RcCustomButtonHardwareStatus) v;
            return flag("1", h.getIsC1Click()) + flag("2", h.getIsC2Click())
                    + flag("3", h.getIsC3Click()) + flag("4", h.getIsC4Click())
                    + " 5d:" + describe(h.getFiveDimensionPressStatus());
        }
        if (v instanceof BatteryInfo) {
            BatteryInfo b = (BatteryInfo) v;
            return b.getBatteryPercent() + "%";
        }
        String s = String.valueOf(v);
        return s.length() > 40 ? s.substring(0, 40) + "..." : s;
    }

    private static String flag(String name, Boolean b) {
        return b == null ? "?" : (b ? name : ".");
    }

    /** Every field of an MSDK error: description() alone came back null on the RC Pro. */
    static String err(IDJIError e) {
        if (e == null) return "null error";
        StringBuilder b = new StringBuilder();
        b.append(e.errorType()).append('/').append(e.errorCode());
        if (e.innerCode() != null) b.append('/').append(e.innerCode());
        if (e.hint() != null) b.append(" hint=").append(e.hint());
        if (e.description() != null) b.append(" \"").append(e.description()).append('"');
        return b.toString();
    }

    /** A candidate that never answers is abandoned after this many failures... */
    private static final int PROBE_GIVE_UP_FAILS = 5;
    /** ...or after this many requests with no reply at all (2 s at 5 Hz). */
    private static final int PROBE_GIVE_UP_UNANSWERED = 10;

    /**
     * Liveness: change-driven listeners are silent while a stick is held still, so
     * silence cannot tell "held" from "wedged". rc_ok therefore also needs a fresh async
     * getValue round-trip to the RC.
     *
     * Which keys this firmware answers asynchronously is not documented (KeyConnection's
     * read was rejected on the first RC Pro run). So the candidates are tried in order: one
     * that has never succeeded is dropped after PROBE_GIVE_UP_FAILS failures or
     * PROBE_GIVE_UP_UNANSWERED unanswered requests, and the first that succeeds is kept
     * for good, after which a failing read DOES block rc_ok (a real wedge). If every
     * candidate is dropped, the probe is marked unsupported and rc_ok falls back to MSDK's
     * RC connection state. Every step is traced.
     *
     * Once a second a STICK is also read the same way, purely as a spike diagnostic (a
     * device round-trip, or a cache read?), and a per-key callback-rate line goes to logcat.
     */
    private final Runnable probe = new Runnable() {
        @Override
        public void run() {
            if (!running) return;
            try {
                DJIKey<?> key = null;
                int idx;
                String drop = null;
                synchronized (lock) {
                    idx = probeIdx;
                    if (!probeUnsupported && idx < probeKeys.size()) {
                        if (!probeEverOk && candidateSent - candidateReplied >= PROBE_GIVE_UP_UNANSWERED) {
                            drop = probeNames.get(idx) + ": " + (candidateSent - candidateReplied)
                                    + " requests without any reply";
                        } else {
                            key = probeKeys.get(idx);
                            candidateSent++;
                        }
                    }
                }
                if (drop != null) {
                    nextCandidate(idx, drop);
                } else if (key != null) {
                    probeOnce(key, idx, SystemClock.elapsedRealtime());
                }
                if (++probeTicks % 5 == 0) {
                    stickProbe();
                    logRates();
                }
            } catch (RuntimeException | LinkageError e) {
                synchronized (lock) {
                    probeFail++;
                    probeError = e.toString();
                }
                CrashLog.error("liveness probe", e);
            } finally {
                if (running) main.postDelayed(this, PROBE_PERIOD_MS);
            }
        }
    };

    private <T> void probeOnce(DJIKey<T> key, final int idx, final long sent) {
        KeyManager.getInstance().getValue(key, new CommonCallbacks.CompletionCallbackWithParam<T>() {
            @Override
            public void onSuccess(T v) {
                long now = SystemClock.elapsedRealtime();
                boolean firstEver;
                String name;
                synchronized (lock) {
                    if (idx == probeIdx) candidateReplied++;
                    probeOkAt = now;
                    probeLatencyMs = now - sent;
                    probeOk++;
                    firstEver = !probeEverOk;
                    probeEverOk = true;
                    name = idx < probeNames.size() ? probeNames.get(idx) : "?";
                    if (idx == 0 && v instanceof Boolean) rcConnected = (Boolean) v;
                }
                if (firstEver) {
                    CrashLog.step("liveness probe: " + name + " answers (" + (now - sent)
                            + " ms) - kept as the liveness read");
                }
            }

            @Override
            public void onFailure(IDJIError e) {
                String why = err(e);
                boolean giveUp = false, first;
                synchronized (lock) {
                    if (idx == probeIdx) candidateReplied++;
                    probeFail++;
                    first = !why.equals(probeError);
                    probeError = why;
                    if (idx == probeIdx && !probeEverOk && ++candidateFails >= PROBE_GIVE_UP_FAILS) {
                        giveUp = true;
                    }
                }
                if (giveUp) {
                    nextCandidate(idx, why);
                } else if (first) {
                    CrashLog.step("liveness probe failure: " + why);
                }
            }
        });
    }

    /** Drop candidate `idx` (if still current) and move on; mark unsupported after the last. */
    private void nextCandidate(int idx, String why) {
        String msg;
        synchronized (lock) {
            if (idx != probeIdx || probeEverOk) return;
            String name = probeNames.get(idx);
            probeIdx++;
            candidateFails = candidateSent = candidateReplied = 0;
            if (probeIdx >= probeKeys.size()) {
                probeUnsupported = true;
                msg = "liveness probe: " + name + " never answered (" + why + "). No candidate "
                        + "answers on this RC - rc_ok now uses MSDK's RC connection state alone";
            } else {
                msg = "liveness probe: " + name + " never answered (" + why + ") - trying "
                        + probeNames.get(probeIdx);
            }
        }
        CrashLog.step(msg);
    }

    private void stickProbe() {
        final long sent = SystemClock.elapsedRealtime();
        KeyManager.getInstance().getValue(KeyTools.createKey(RemoteControllerKey.KeyStickLeftHorizontal),
                new CommonCallbacks.CompletionCallbackWithParam<Integer>() {
                    @Override
                    public void onSuccess(Integer v) {
                        long ms = SystemClock.elapsedRealtime() - sent;
                        boolean first;
                        synchronized (lock) {
                            stickProbeOk++;
                            stickProbeLatencyMs = ms;
                            first = !stickProbeTraced;
                            stickProbeTraced = true;
                        }
                        if (first) CrashLog.step("stick read (async): ok, " + ms + " ms, value " + v);
                    }

                    @Override
                    public void onFailure(IDJIError e) {
                        String why = err(e);
                        boolean first;
                        synchronized (lock) {
                            stickProbeFail++;
                            stickProbeError = why;
                            first = !stickProbeTraced;
                            stickProbeTraced = true;
                        }
                        if (first) CrashLog.step("stick read (async): FAILED " + why);
                    }
                });
    }

    /**
     * Five seconds in, one trace line with every key's state, so trace.txt alone answers
     * "which keys does this RC serve with no aircraft?" (hardware spike S2/S3).
     */
    private final Runnable summary = new Runnable() {
        @Override
        public void run() {
            if (!running) return;
            StringBuilder b = new StringBuilder("keys at +5 s (name=value/callbacks):");
            synchronized (lock) {
                for (KeyStat k : stats.values()) {
                    b.append(' ').append(k.name).append('=').append(k.value).append('/').append(k.count);
                }
            }
            CrashLog.step(b.toString());
            Snapshot s = snapshot(false);
            CrashLog.step("state at +5 s: rc_ok=" + s.rcOk + (s.rcOk ? "" : " (" + s.notOkReason() + ")")
                    + ", aircraft_linked=" + s.aircraftLinked + ", liveness key=" + s.probeKey);
        }
    };

    /** One logcat line a second: callbacks/s per wire key (adb logcat -s LIS_CONTROLLER). */
    private void logRates() {
        StringBuilder sb = new StringBuilder("rates/s");
        StringBuilder moving = new StringBuilder();
        synchronized (lock) {
            for (KeyStat k : stats.values()) {
                if (!k.wire) continue;
                Long prev = lastLoggedCounts.get(k.name);
                long d = k.count - (prev == null ? 0 : prev);
                sb.append(' ').append(k.name).append('=').append(d);
                lastLoggedCounts.put(k.name, k.count);
                if (d > 0 && (k.group.equals("stick") || k.group.equals("dial") || k.name.equals("wheel"))) {
                    moving.append(' ').append(k.name).append('=').append(d);
                }
            }
            sb.append("  probe ").append(probeLatencyMs).append("ms ok/fail ")
                    .append(probeOk).append('/').append(probeFail)
                    .append("  stick-probe ").append(stickProbeLatencyMs).append("ms ok/fail ")
                    .append(stickProbeOk).append('/').append(stickProbeFail);
        }
        Log.i(TAG, sb.toString());
        // Into trace.txt as well, but only while a stick or dial is reporting: the MSDK side
        // of the rate comparison (MainActivity traces the gamepad side). Capped per run.
        if (moving.length() > 0 && rateLinesTraced < MAX_RATE_LINES) {
            rateLinesTraced++;
            CrashLog.step("msdk callbacks/s:" + moving);
        }
    }

    private static final int MAX_RATE_LINES = 300;
    private int rateLinesTraced;
}
