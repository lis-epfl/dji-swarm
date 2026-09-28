package com.liscontroller;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.View;

import java.util.Objects;

/**
 * One dial as a centre-zero slider: + = turned right / clockwise (the wire convention).
 * A dial the RC does not serve (null) is a dashed, empty track.
 */
public class DialBarView extends View {
    private static final int LINE = 0xFF262D38, FAINT = 0xFF545D68, ACCENT = 0xFF58A6FF;
    private final Paint track = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint dashed = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint tick = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint fill = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint knob = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final float d;
    private boolean drawn;
    private Integer raw;

    public DialBarView(Context c, AttributeSet a) {
        super(c, a);
        d = getResources().getDisplayMetrics().density;
        track.setStrokeWidth(4 * d);
        track.setStrokeCap(Paint.Cap.ROUND);
        track.setColor(LINE);
        dashed.setStyle(Paint.Style.STROKE);
        dashed.setStrokeWidth(2 * d);
        dashed.setColor(FAINT);
        dashed.setPathEffect(new DashPathEffect(new float[]{5 * d, 5 * d}, 0));
        tick.setStrokeWidth(1.5f * d);
        tick.setStrokeCap(Paint.Cap.ROUND);
        tick.setColor(FAINT);
        fill.setStrokeWidth(4 * d);
        fill.setStrokeCap(Paint.Cap.ROUND);
        fill.setColor(ACCENT);
        knob.setColor(ACCENT);
    }

    void set(Integer raw) {
        if (drawn && Objects.equals(raw, this.raw)) return;
        drawn = true;
        this.raw = raw;
        invalidate();
    }

    @Override
    protected void onDraw(Canvas c) {
        float w = getWidth(), cy = getHeight() / 2f;
        float x0 = 9 * d, x1 = w - 9 * d, cx = (x0 + x1) / 2f;
        if (x1 <= x0) return;
        if (raw == null) {
            c.drawLine(x0, cy, x1, cy, dashed);
            return;
        }
        c.drawLine(x0, cy, x1, cy, track);
        c.drawLine(cx, cy - 7 * d, cx, cy + 7 * d, tick);
        float v = Math.max(-1f, Math.min(1f, raw / 660f));
        float end = cx + v * (x1 - cx);
        if (Math.abs(end - cx) > 0.5f) c.drawLine(cx, cy, end, cy, fill);
        c.drawCircle(end, cy, 7 * d, knob);
    }
}
