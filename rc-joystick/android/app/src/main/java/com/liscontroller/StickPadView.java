package com.liscontroller;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.util.AttributeSet;
import android.view.View;

import java.util.Objects;

/**
 * One stick as a round gate: + x = right, + y = up (the wire convention). A stick the RC
 * does not serve (null) is drawn dashed and empty, because a missing stick is not a
 * centred one. The numbers are in Details; this view only shows where the stick is.
 */
public class StickPadView extends View {
    private static final int SURFACE = 0xFF12161D, LINE = 0xFF262D38, GUIDE = 0xFF1B2129,
            FAINT = 0xFF545D68, ACCENT = 0xFF58A6FF;
    private final Paint fill = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint ring = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint dashed = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint guide = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint centre = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint stem = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint halo = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint knob = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint label = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final float d;
    private String name = "";
    private Integer rawX, rawY;

    public StickPadView(Context c, AttributeSet a) {
        super(c, a);
        d = getResources().getDisplayMetrics().density;
        fill.setColor(SURFACE);
        ring.setStyle(Paint.Style.STROKE);
        ring.setStrokeWidth(1.5f * d);
        ring.setColor(LINE);
        dashed.setStyle(Paint.Style.STROKE);
        dashed.setStrokeWidth(1.5f * d);
        dashed.setColor(FAINT);
        dashed.setPathEffect(new DashPathEffect(new float[]{6 * d, 6 * d}, 0));
        guide.setStyle(Paint.Style.STROKE);
        guide.setStrokeWidth(1 * d);
        guide.setColor(GUIDE);
        centre.setColor(LINE);
        stem.setStrokeWidth(2 * d);
        stem.setStrokeCap(Paint.Cap.ROUND);
        stem.setColor(0x6658A6FF);
        halo.setColor(0x2E58A6FF);
        knob.setColor(ACCENT);
        label.setColor(FAINT);
        label.setTextSize(10 * d);
        label.setTextAlign(Paint.Align.CENTER);
        label.setLetterSpacing(0.18f);
        label.setTypeface(Typeface.create("sans-serif-medium", Typeface.NORMAL));
    }

    void set(String name, Integer rawX, Integer rawY) {
        if (name.equals(this.name) && Objects.equals(rawX, this.rawX) && Objects.equals(rawY, this.rawY)) {
            return;
        }
        this.name = name;
        this.rawX = rawX;
        this.rawY = rawY;
        invalidate();
    }

    private static float norm(Integer raw) {
        return Math.max(-1f, Math.min(1f, raw / 660f));
    }

    @Override
    protected void onDraw(Canvas c) {
        float labelSpace = 18 * d;
        float w = getWidth(), h = getHeight() - labelSpace;
        float r = Math.min(w, h) / 2f - 4 * d;
        if (r <= 12 * d) return;
        float cx = w / 2f, cy = h / 2f;
        boolean served = rawX != null && rawY != null;

        c.drawCircle(cx, cy, r, fill);
        c.drawCircle(cx, cy, r * 0.5f, guide);
        c.drawLine(cx - r, cy, cx + r, cy, guide);
        c.drawLine(cx, cy - r, cx, cy + r, guide);
        c.drawCircle(cx, cy, r, served ? ring : dashed);
        c.drawText(name, cx, getHeight() - 4 * d, label);
        if (!served) return;

        c.drawCircle(cx, cy, 2.5f * d, centre);
        // The two axes throw a square; the gate is round. The elliptical grid mapping puts
        // the square's corners on the rim while leaving a single-axis deflection exact.
        float x = norm(rawX), y = norm(rawY);
        float u = x * (float) Math.sqrt(1 - y * y / 2), v = y * (float) Math.sqrt(1 - x * x / 2);
        float travel = r - 12 * d;
        float kx = cx + u * travel, ky = cy - v * travel;
        c.drawLine(cx, cy, kx, ky, stem);
        c.drawCircle(kx, ky, 19 * d, halo);
        c.drawCircle(kx, ky, 9 * d, knob);
    }
}
