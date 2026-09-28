package com.liscontroller;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.RectF;
import android.util.AttributeSet;
import android.view.View;

import java.util.Locale;

/** One stick as a square pad: + x = right, + y = up (the wire convention). */
public class StickPadView extends View {
    private final Paint frame = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint cross = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint dot = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint text = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final RectF box = new RectF();
    private String label = "";
    private Integer rawX, rawY;

    public StickPadView(Context c, AttributeSet a) {
        super(c, a);
        float d = getResources().getDisplayMetrics().density;
        frame.setStyle(Paint.Style.STROKE);
        frame.setStrokeWidth(2 * d);
        frame.setColor(0xFF90A4AE);
        cross.setStrokeWidth(1 * d);
        cross.setColor(0xFF455A64);
        dot.setColor(0xFF4FC3F7);
        text.setColor(0xFFECEFF1);
        text.setTextSize(12 * d);
    }

    void set(String label, Integer rawX, Integer rawY) {
        this.label = label;
        this.rawX = rawX;
        this.rawY = rawY;
        invalidate();
    }

    private static float norm(Integer raw) {
        return Math.max(-1f, Math.min(1f, raw / 660f));
    }

    @Override
    protected void onDraw(Canvas c) {
        float d = getResources().getDisplayMetrics().density;
        float side = Math.min(getWidth(), getHeight()) - 4 * d;
        float left = (getWidth() - side) / 2f, top = (getHeight() - side) / 2f;
        box.set(left, top, left + side, top + side);
        float cx = box.centerX(), cy = box.centerY(), r = side / 2f;
        c.drawRoundRect(box, 8 * d, 8 * d, frame);
        c.drawLine(cx, box.top, cx, box.bottom, cross);
        c.drawLine(box.left, cy, box.right, cy, cross);
        c.drawText(label, box.left + 6 * d, box.top + 16 * d, text);
        if (rawX == null || rawY == null) {
            text.setColor(0xFFEF5350);
            c.drawText("not served", cx - 30 * d, cy - 8 * d, text);
            text.setColor(0xFFECEFF1);
        } else {
            float x = norm(rawX), y = norm(rawY);
            c.drawCircle(cx + x * (r - 10 * d), cy - y * (r - 10 * d), 9 * d, dot);
            c.drawText(String.format(Locale.US, "%+4d %+4d", rawX, rawY),
                    box.left + 6 * d, box.bottom - 8 * d, text);
        }
    }
}
