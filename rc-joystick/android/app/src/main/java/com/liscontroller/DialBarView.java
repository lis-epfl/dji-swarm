package com.liscontroller;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.RectF;
import android.util.AttributeSet;
import android.view.View;

import java.util.Locale;

/** One dial as a centre-zero bar: + = turned right / clockwise (the wire convention). */
public class DialBarView extends View {
    private final Paint frame = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint fill = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint text = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final RectF r = new RectF();
    private String label = "";
    private Integer raw;

    public DialBarView(Context c, AttributeSet a) {
        super(c, a);
        float d = getResources().getDisplayMetrics().density;
        frame.setStyle(Paint.Style.STROKE);
        frame.setStrokeWidth(2 * d);
        frame.setColor(0xFF90A4AE);
        fill.setColor(0xFF4FC3F7);
        text.setColor(0xFFECEFF1);
        text.setTextSize(12 * d);
    }

    void set(String label, Integer raw) {
        this.label = label;
        this.raw = raw;
        invalidate();
    }

    @Override
    protected void onDraw(Canvas c) {
        float d = getResources().getDisplayMetrics().density;
        float w = getWidth(), h = getHeight();
        float cx = w / 2f;
        r.set(2 * d, 2 * d, w - 2 * d, h - 2 * d);
        if (raw != null) {
            float v = Math.max(-1f, Math.min(1f, raw / 660f));
            float end = cx + v * (w / 2f - 2 * d);
            c.drawRect(Math.min(cx, end), r.top, Math.max(cx, end), r.bottom, fill);
        }
        c.drawRect(r, frame);
        c.drawLine(cx, r.top, cx, r.bottom, frame);
        String t = raw == null ? label + ": not served" : String.format(Locale.US, "%s %+4d", label, raw);
        c.drawText(t, 6 * d, h / 2f + 4 * d, text);
    }
}
