package com.example.temperatureapp;

import androidx.appcompat.app.AppCompatActivity;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.graphics.Color;
import android.os.Bundle;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import com.example.temperatureapp.R;
import com.github.mikephil.charting.charts.LineChart;
import com.github.mikephil.charting.components.Description;
import com.github.mikephil.charting.components.Legend;
import com.github.mikephil.charting.components.LegendEntry;
import com.github.mikephil.charting.data.Entry;
import com.github.mikephil.charting.data.LineData;
import com.github.mikephil.charting.data.LineDataSet;
import com.github.mikephil.charting.interfaces.datasets.ILineDataSet;
import com.example.temperatureapp.Graphing;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.UUID;



public class MainActivity extends AppCompatActivity {
    Graphing myGraph;
    static final UUID mUUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    BluetoothSocket btSocket = null;
    public int count = 0;
    ArrayList<Entry> numbers = new ArrayList<>();

    public void handleBluetooth(){
        BluetoothAdapter btAdapter = BluetoothAdapter.getDefaultAdapter();
        System.out.println(btAdapter.getBondedDevices());
        Object macAddress = btAdapter.getBondedDevices();
        String address = macAddress.toString();
        System.out.println(address);
        String newAddress1 = address.replace("[", "");
        System.out.println(newAddress1);
        String newAddress2 = newAddress1.replace("]", "");
        System.out.println(newAddress2);
        BluetoothDevice hc05 = btAdapter.getRemoteDevice(newAddress2);
        System.out.println(hc05.getName());
        try {
            btSocket = hc05.createRfcommSocketToServiceRecord(mUUID);
            System.out.println(btSocket);
            btSocket.connect();
            System.out.println(btSocket.isConnected());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        myGraph = new Graphing((LineChart) findViewById(R.id.line_chart));


        final TextView temp = findViewById(R.id.temp);

        this.handleBluetooth();

        Thread t = new Thread(){
            @Override
            public void  run(){
                while(count<96){
                    try{
                        Thread.sleep(1000);
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                InputStream inputStream = null;
                                try {
                                    inputStream = btSocket.getInputStream();
                                    inputStream.skip(inputStream.available());
                                    byte b = (byte) inputStream.read();
                                    int temperature = (int) b;
                                    System.out.println(temperature);
                                    temp.setText(String.valueOf(temperature));
                                    numbers.add(new Entry(count,temperature));
                                    System.out.println(numbers);
                                    System.out.println(count);

                                } catch (IOException e) {
                                    e.printStackTrace();
                                }
                                count++;

                            }

                        });
                        myGraph.updateGraph(numbers);
                    } catch (InterruptedException e){
                        e.printStackTrace();
                    }
                }


            }
        };

        t.start();






    }





}
