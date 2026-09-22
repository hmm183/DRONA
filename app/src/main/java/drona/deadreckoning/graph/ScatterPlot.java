package drona.deadreckoning.graph;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.view.View;

import java.util.ArrayList;

public class ScatterPlot {

    private String seriesName;
    private ArrayList<Double> xList;
    private ArrayList<Double> yList;

    public ScatterPlot (String seriesName) {
        this.seriesName = seriesName;
        xList = new ArrayList<>();
        yList = new ArrayList<>();
    }

    public View getGraphView(Context context) {
        return new ScatterPlotView(context);
    }

    public void addPoint(double x, double y) {
        xList.add(x);
        yList.add(y);
    }

    public float getLastXPoint() {
        if (xList.isEmpty()) return 0f;
        return xList.get(xList.size() - 1).floatValue();
    }

    public float getLastYPoint() {
        if (yList.isEmpty()) return 0f;
        return yList.get(yList.size() - 1).floatValue();
    }

    public void clearSet() {
        xList.clear();
        yList.clear();
    }

    private double getMaxBound() {
        double max = 0;
        for (double num : xList)
            if (max < Math.abs(num))
                max = Math.abs(num);
        for (double num : yList)
            if (max < Math.abs(num))
                max = Math.abs(num);
        return Math.max(100.0, (Math.abs(max) / 100) * 100 + 100);
    }

    private class ScatterPlotView extends View {
        private Paint pointPaint;
        private Paint axisPaint;

        public ScatterPlotView(Context context) {
            super(context);
            pointPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
            pointPaint.setColor(Color.parseColor("#ff0099ff"));
            pointPaint.setStyle(Paint.Style.FILL);

            axisPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
            axisPaint.setColor(Color.LTGRAY);
            axisPaint.setStrokeWidth(3f);
        }

        @Override
        protected void onDraw(Canvas canvas) {
            super.onDraw(canvas);
            int width = getWidth();
            int height = getHeight();
            if (width == 0 || height == 0) return;

            float centerX = width / 2f;
            float centerY = height / 2f;
            double bound = getMaxBound();

            // Draw axis
            canvas.drawLine(0, centerY, width, centerY, axisPaint);
            canvas.drawLine(centerX, 0, centerX, height, axisPaint);

            float scale = (float) (Math.min(width, height) / (2.0 * bound));

            // Draw points
            for (int i = 0; i < xList.size(); i++) {
                float px = centerX + (float) (xList.get(i) * scale);
                float py = centerY - (float) (yList.get(i) * scale);
                canvas.drawCircle(px, py, 12f, pointPaint);
            }
        }
    }
}
