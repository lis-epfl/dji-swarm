package com.liscontroller;

import android.os.SystemClock;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/**
 * Wire protocol v1: the contract is rc-joystick/PROTOCOL.md, and the PC side is
 * rc-joystick/pc/rcjoy/protocol.py. Keep all three in step.
 *
 * org.json drops a key whose value is Java null, so every "not served" value goes
 * out as JSONObject.NULL. On the wire, null means "not served", which is different
 * from zero.
 */
final class Protocol {
    static final int VERSION = 1;
    static final int PORT = 5070;
    static final int RATE_MIN_HZ = 10;
    static final int RATE_MAX_HZ = 100;
    static final int RATE_DEFAULT_HZ = 50;
    static final long SUBSCRIPTION_TTL_MS = 3000;
    static final int MAX_INFO_BYTES = 1200;

    // Wire field names. The index is the slot in RcInputReader's arrays.
    static final String[] STICKS = {"lh", "lv", "rh", "rv"};
    static final String[] DIALS = {"l", "r"};
    static final String[] BUTTONS = {"c1", "c2", "c3", "shutter", "record", "rth", "pause",
            "rc_switch", "5d_up", "5d_down", "5d_left", "5d_right", "5d_press"};

    private Protocol() {
    }

    static int clampRate(int hz) {
        return Math.max(RATE_MIN_HZ, Math.min(RATE_MAX_HZ, hz));
    }

    private static Object orNull(Object v) {
        return v == null ? JSONObject.NULL : v;
    }

    static JSONObject state(long seq, RcInputReader.Snapshot s) throws JSONException {
        JSONObject m = new JSONObject();
        m.put("type", "state");
        m.put("v", VERSION);
        m.put("seq", seq);
        m.put("t_ms", SystemClock.elapsedRealtime());
        m.put("rc_ok", s.rcOk);
        m.put("aircraft_linked", s.aircraftLinked);
        // Where this packet's sticks and dials came from: "gamepad" (~70 Hz) or "msdk" (~10 Hz).
        m.put("stick_src", s.stickSrc);
        JSONObject sticks = new JSONObject();
        for (int i = 0; i < STICKS.length; i++) sticks.put(STICKS[i], orNull(s.sticks[i]));
        m.put("sticks", sticks);
        JSONObject dials = new JSONObject();
        for (int i = 0; i < DIALS.length; i++) dials.put(DIALS[i], orNull(s.dials[i]));
        m.put("dials", dials);
        JSONObject buttons = new JSONObject();
        JSONObject presses = new JSONObject();
        for (int i = 0; i < BUTTONS.length; i++) {
            buttons.put(BUTTONS[i], orNull(s.buttons[i]));
            // Counts only for buttons the RC actually serves; the PC baselines a
            // button the first time it appears.
            if (s.buttons[i] != null) presses.put(BUTTONS[i], s.presses[i]);
        }
        m.put("buttons", buttons);
        m.put("presses", presses);
        m.put("mode_switch", orNull(s.modeSwitch));
        return m;
    }

    static JSONObject info(Object echoT, RcInputReader.Snapshot s, RfStatus rf, String ethIp,
                           String appVersion, int rateHz, int subscribers, boolean withKeys)
            throws JSONException {
        JSONObject m = new JSONObject();
        m.put("type", "info");
        m.put("v", VERSION);
        m.put("echo_t", orNull(echoT));
        m.put("app", appVersion);
        m.put("rc_type", orNull(s.rcType));
        m.put("sn", orNull(s.sn));
        m.put("fw", orNull(s.fw));
        m.put("battery", orNull(s.batteryPct));
        m.put("stick_mode", orNull(s.stickMode));
        m.put("eth_ip", orNull(ethIp));
        m.put("port", PORT);
        m.put("rate_hz", rateHz);
        m.put("subscribers", subscribers);
        if (withKeys) {
            long now = SystemClock.elapsedRealtime();
            JSONObject keys = new JSONObject();
            for (RcInputReader.KeyStat k : s.stats) {
                if (!k.wire) continue;
                JSONObject e = new JSONObject();
                e.put("n", k.count);
                e.put("age_ms", k.lastMs > 0 ? (Object) (now - k.lastMs) : JSONObject.NULL);
                keys.put(k.name, e);
            }
            m.put("keys", keys);
        }
        JSONObject r = new JSONObject();
        r.put("wifi", orNull(rf.wifi));
        r.put("wifi_scan", orNull(rf.wifiScan));
        r.put("bt", orNull(rf.bt));
        r.put("ble_scan", orNull(rf.bleScan));
        r.put("airplane", rf.airplane);
        r.put("aircraft_linked", s.aircraftLinked);
        m.put("rf", r);
        GamepadInput.Reading g = s.gamepad;
        if (g != null) {
            JSONObject gp = new JSONObject();
            gp.put("hz", g.hz);
            gp.put("gap_ms", g.gapMs < 0 ? JSONObject.NULL : (Object) g.gapMs);
            gp.put("why", orNull(g.whyNot));
            gp.put("gate", orNull(gate(g)));
            JSONArray verified = new JSONArray();
            JSONArray inverted = new JSONArray();
            for (int i = 0; i < GamepadInput.AXES.length; i++) {
                if (g.check[i] == GamepadInput.CHECK_OK) verified.put(GamepadInput.AXES[i]);
                if (g.check[i] == GamepadInput.CHECK_INVERTED) inverted.put(GamepadInput.AXES[i]);
            }
            gp.put("verified", verified);
            gp.put("inverted", inverted);
            m.put("gamepad", gp);
        }
        return m;
    }

    /** DJI's gamepad gate: true = all three settings on, false = one is off, null = unread. */
    static Boolean gate(GamepadInput.Reading g) {
        if (Boolean.FALSE.equals(g.gameMode) || Boolean.FALSE.equals(g.motionLeft)
                || Boolean.FALSE.equals(g.motionRight)) {
            return Boolean.FALSE;
        }
        if (g.gameMode == null || g.motionLeft == null || g.motionRight == null) return null;
        return Boolean.TRUE;
    }

    static JSONObject bye(String reason) throws JSONException {
        JSONObject m = new JSONObject();
        m.put("type", "bye");
        m.put("v", VERSION);
        m.put("reason", reason);
        return m;
    }
}
