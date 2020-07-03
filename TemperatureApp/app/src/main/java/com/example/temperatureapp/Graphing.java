package com.example.temperatureapp;

import android.graphics.Color;

import com.github.mikephil.charting.charts.LineChart;
import com.github.mikephil.charting.components.Description;
import com.github.mikephil.charting.components.Legend;
import com.github.mikephil.charting.components.LegendEntry;
import com.github.mikephil.charting.data.LineData;
import com.github.mikephil.charting.data.LineDataSet;
import com.github.mikephil.charting.interfaces.datasets.ILineDataSet;

import java.util.ArrayList;

public class Graphing {
    LineChart mpLineChart;

    public Graphing(LineChart mpLineChart){
        this.mpLineChart = mpLineChart;
        this.mpLineChart.setDrawGridBackground(true);
        this.mpLineChart.setDrawBorders(true);
        Description description = new Description();
        description.setText("Temperature Readings");
        description.setTextColor(Color.BLUE);
        description.setTextSize(14);
        mpLineChart.setDescription(description);

        Legend legend = mpLineChart.getLegend();
        legend.setTextColor(Color.RED);
        legend.setTextSize(13);
        LegendEntry[] legendEntries = new LegendEntry[1];

        int[] colorClassArray = new int[] {Color.BLUE};
        String[] legendName = {"Temp"};
        for (int i=0; i<legendEntries.length; i++)
        {
            LegendEntry entry = new LegendEntry();
            entry.formColor = colorClassArray[i];
            entry.label = String.valueOf(legendName[i]);
            legendEntries[i] = entry;
        }

        legend.setCustom(legendEntries);
    }

    public void updateGraph(ArrayList numbers) {
        LineDataSet lineDataSet1 = new LineDataSet(numbers, "Data Set 1");
        ArrayList<ILineDataSet> dataSets = new ArrayList<>();
        dataSets.add(lineDataSet1);
        LineData data = new LineData(dataSets);
        mpLineChart.setData(data);
        mpLineChart.invalidate();
    }
}
