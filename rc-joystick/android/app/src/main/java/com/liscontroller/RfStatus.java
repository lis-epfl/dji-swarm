package com.liscontroller;

import android.bluetooth.BluetoothAdapter;
import android.content.ContentResolver;
import android.content.Context;
import android.net.wifi.WifiManager;
import android.provider.Settings;

import java.util.ArrayList;
import java.util.List;

/**
 * The RC's Android radios, READ-ONLY. It never switches anything.
 *
 * Only Android's radios are visible here. The RC's OcuSync link to aircraft is
 * run by its own firmware, and airplane mode does not cover it. Whether an
 * unlinked RC stays quiet on 2.4/5.8 GHz is MEASURED (rc-joystick/README.md, Z1).
 * The scanning flags matter because "Wi-Fi off" with "Wi-Fi scanning" on still
 * lets the radio transmit probe requests.
 */
final class RfStatus {
    static final RfStatus UNKNOWN = new RfStatus(null, null, null, null, false);

    final Boolean wifi;
    final Boolean wifiScan;
    final Boolean bt;
    final Boolean bleScan;
    final boolean airplane;

    private RfStatus(Boolean wifi, Boolean wifiScan, Boolean bt, Boolean bleScan, boolean airplane) {
        this.wifi = wifi;
        this.wifiScan = wifiScan;
        this.bt = bt;
        this.bleScan = bleScan;
        this.airplane = airplane;
    }

    static RfStatus read(Context ctx) {
        ContentResolver cr = ctx.getContentResolver();
        Boolean wifi = null, wifiScan = null, bt = null, bleScan = null;
        boolean airplane = false;
        try {
            WifiManager wm = (WifiManager) ctx.getApplicationContext()
                    .getSystemService(Context.WIFI_SERVICE);
            if (wm != null) {
                wifi = wm.isWifiEnabled();
                wifiScan = wm.isScanAlwaysAvailable();
            }
        } catch (RuntimeException ignored) {
        }
        try {
            BluetoothAdapter ad = BluetoothAdapter.getDefaultAdapter();
            bt = ad != null && ad.isEnabled();
        } catch (RuntimeException ignored) {    // SecurityException on newer Android
        }
        try {
            bleScan = Settings.Global.getInt(cr, "ble_scan_always_enabled", 0) != 0;
        } catch (RuntimeException ignored) {
        }
        try {
            airplane = Settings.Global.getInt(cr, Settings.Global.AIRPLANE_MODE_ON, 0) != 0;
        } catch (RuntimeException ignored) {
        }
        return new RfStatus(wifi, wifiScan, bt, bleScan, airplane);
    }

    /** What the RC screen and the PC shout about. Empty = nothing Android-side is on. */
    List<String> warnings() {
        List<String> out = new ArrayList<>();
        if (Boolean.TRUE.equals(wifi)) out.add("Wi-Fi ON");
        if (Boolean.TRUE.equals(wifiScan)) out.add("Wi-Fi scanning ON");
        if (Boolean.TRUE.equals(bt)) out.add("Bluetooth ON");
        if (Boolean.TRUE.equals(bleScan)) out.add("Bluetooth scanning ON");
        return out;
    }
}
