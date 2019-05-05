/*
 * Copyright (C) 2014 Oliver Degener.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ollide.rosandroid;

import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Address;
import android.location.Geocoder;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.os.Handler;
import android.provider.Settings;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.text.Html;
import android.text.Spannable;
import android.text.SpannableString;
import android.text.TextUtils;
import android.text.style.ForegroundColorSpan;
import android.text.style.RelativeSizeSpan;
import android.text.style.SuperscriptSpan;
import android.util.Log;
import android.view.View;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.message.MessageListener;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import java.io.IOException;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

import android.support.v7.app.AppCompatActivity;
import static android.content.ContentValues.TAG;
import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;
import static java.lang.Math.abs;
import static java.util.Objects.isNull;

import com.google.android.gms.common.api.GoogleApiClient;
import com.google.android.gms.location.LocationServices;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.MapFragment;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;

import android.support.v4.app.Fragment;

import std_msgs.Float64MultiArray;

public class MainActivity extends RosActivity implements LocationListener, SensorEventListener, StepListener, OnMapReadyCallback, MessageListener<Float64MultiArray> {

    public MainActivity() {
        super("RosAndroidExample", "RosAndroidExample");
    }
    private SimplePublisherNode node_sp;
    private static final int PERMISSIONS_REQUEST_ACCESS_FINE_LOCATION = 1;
    private boolean mLocationPermissionGranted;
    private final float[] mAccelerometerReading = new float[3];
    private final float[] mMagnetometerReading = new float[3];
    private Sensor mAccelerometer;
    private Sensor mMagnetometer;
    private SensorManager mSensorManager;
    private StepDetector simpleStepDetector;
    private Handler mRouteHandler;
    private int steps=0;
    private int artf_steps=0;
    private int prev_steps=0;
    private int final_steps=0;
    private double prev_d=0;
    private double step_len=0.1;
    private int mInterval = 2000;
    private int count=5;
    private boolean source_gps_rcvd = FALSE;
    private boolean target_coords_rcvd = FALSE;
    private GoogleMap googleMap_g;
    private LatLng startLatLon;
    private LatLng previousLatLon;
    private GoogleApiClient mGoogleApiClient;
    private Marker marker;
    private Marker inital_marker;
    private float[] orientationAngles = {0,0,0};
    private double[] message_data ={0};
    private int avg = 0;


    @Override
    public void onNewMessage(final std_msgs.Float64MultiArray message) {

        message_data=message.getData();
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        if (!(ContextCompat.checkSelfPermission(this.getApplicationContext(),
                android.Manifest.permission.ACCESS_FINE_LOCATION)
                == PackageManager.PERMISSION_GRANTED)) {
            Log.i(TAG,"Permissions Not Granted");
            final TextView helloTextView = findViewById(R.id.textView3);
            helloTextView.setText("Permissions Not Granted");

            getLocationPermission();

        }
        LocationManager locationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
        locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, this);

        mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        mSensorManager.registerListener(this, mAccelerometer,
                SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(this, mMagnetometer,
                SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);

        mAccelerometer =   mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mMagnetometer =   mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        simpleStepDetector = new StepDetector();
        simpleStepDetector.registerListener(this);

//        initializeMap();
        MapFragment mapFragment = (MapFragment) getFragmentManager().findFragmentById(R.id.map);
//        mapFragment.getMapAsync(this);
        mapFragment.getMapAsync(this);
        mRouteHandler = new Handler();

//        helloTextView.setTextColor(Color.WHITE);
//        helloTextView.setText(Html.fromHtml("<html><body><font size=5 color=red>Hello </font> World </body><html>"));

    }


//    private void initializeMap() {
//        if (googleMap_g == null) {
//            MapFragment mapFragment = (MapFragment) getFragmentManager().findFragmentById(R.id.map);
//            if(mapFragment!=null){
//                mapFragment.getMapAsync(this);
//            }
//
//            // check if map is created successfully or not
//            if (null== googleMap_g) {
//                Toast.makeText(getApplicationContext(), "Sorry! unable to create maps", Toast.LENGTH_SHORT).show();
//            }
//        }
//    }

    @Override
    protected void onResume() {
        super.onResume();

        // Get updates from the accelerometer and magnetometer at a constant rate.
        // To make batch operations more efficient and reduce power consumption,
        // provide support for delaying updates to the application.
        //
        // In this example, the sensor reporting delay is small enough such that
        // the application receives an update before the system checks the sensor
        // readings again.
        mSensorManager.registerListener(this, mAccelerometer,
                SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(this, mMagnetometer,
                SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);
        simpleStepDetector.registerListener(this);

    }

    @Override
    protected void onPause() {
        super.onPause();

        // Don't receive any more updates from either sensor.
        mSensorManager.unregisterListener(this);
    }

    public void increment(View v)
    {
        artf_steps=artf_steps+1;
    }

    public void decrement(View v)
    {
        artf_steps=artf_steps-1;
    }
    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        node_sp = new SimplePublisherNode();


        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getROS_IP());

        //nodeConfiguration.setTcpRosBindAddress()

        nodeConfiguration.setMasterUri(getMasterUri());

        nodeMainExecutor.execute(node_sp, nodeConfiguration);
    }

    @Override
    public void onLocationChanged(Location location) {
        // TODO Auto-generated method stub

        double latitude =  location.getLatitude();
        double longitude = location.getLongitude();

        Log.i("Geo_Location", "Latitude: " + latitude + ", Longitude: " + longitude);
        final TextView helloTextView = findViewById(R.id.textView2);
        helloTextView.setTextColor(Color.WHITE);
        helloTextView.setText(Html.fromHtml("<html><body><font size=5 color=red>Hello </font> World </body><html>"));
//        helloTextView.setText("Latitude: " + latitude + ", Longitude: " + longitude);
        String msg="Latitude: " + latitude + ", Longitude: " + longitude;
        if(node_sp!=null){
            if(node_sp.GPS_arr!=null){
        node_sp.GPS_arr[0]=latitude;
        node_sp.GPS_arr[1]=longitude;
        node_sp.GPS_updated+=1;
            }

        }
    }

    @Override
    public void onProviderDisabled(String provider) {
        // TODO Auto-generated method stub

    }

    @Override
    public void onProviderEnabled(String provider) {
        // TODO Auto-generated method stub

    }

    @Override
    public void onStatusChanged(String provider, int status, Bundle extras) {
        // TODO Auto-generated method stub

    }
    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Do something here if sensor accuracy changes.
        // You must implement this callback in your code.
    }

    private void getLocationPermission() {
        /*
         * Request location permission, so that we can get the location of the
         * device. The result of the permission request is handled by a callback,
         * onRequestPermissionsResult.
         */
        if (ContextCompat.checkSelfPermission(this.getApplicationContext(),
                android.Manifest.permission.ACCESS_FINE_LOCATION)
                == PackageManager.PERMISSION_GRANTED) {
            mLocationPermissionGranted = true;
        } else {
            ActivityCompat.requestPermissions(this,
                    new String[]{android.Manifest.permission.ACCESS_FINE_LOCATION},
                    PERMISSIONS_REQUEST_ACCESS_FINE_LOCATION);
        }
    }
    @Override
    public void step(long timeNs) {
        steps++;

    }

    // Get readings from accelerometer and magnetometer. To simplify calculations,
    // consider storing these readings as unit vectors.
    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            System.arraycopy(event.values, 0, mAccelerometerReading,
                    0, mAccelerometerReading.length);
            simpleStepDetector.updateAccel(
                    event.timestamp, event.values[0], event.values[1], event.values[2]);

        }
        else if (event.sensor == mMagnetometer) {
            System.arraycopy(event.values, 0, mMagnetometerReading,
                    0, mMagnetometerReading.length);
        }

        // --------------------------------------------------------------------
        // Orientation Sensor
        // Rotation matrix based on current readings from accelerometer and magnetometer.
        float[] rotationMatrix = new float[9];
        mSensorManager.getRotationMatrix(rotationMatrix, null,
                mAccelerometerReading, mMagnetometerReading);

        orientationAngles[0]= (float)Math.atan2(rotationMatrix[7],rotationMatrix[8]);
        orientationAngles[1]= (float)Math.atan2(-rotationMatrix[6],Math.sqrt(Math.pow(rotationMatrix[7],2)+Math.pow(rotationMatrix[8],2)));
        orientationAngles[2]= (float)Math.atan2(rotationMatrix[3],rotationMatrix[0]);

        final TextView helloTextView = findViewById(R.id.textView3);
        helloTextView.setTextColor(Color.BLACK);
        if(node_sp!=null) {
            String ang = Integer.toString((int) (orientationAngles[2] * 180 / 3.14)) + "o";
            SpannableString angle =  new SpannableString(ang);
            angle.setSpan(new RelativeSizeSpan(4f), 0,ang.length()-1, 0); // set size
            angle.setSpan(new RelativeSizeSpan(3f), ang.length()-1,ang.length(), 0); // set size
            angle.setSpan(new SuperscriptSpan(), ang.length()-1, ang.length(), Spannable.SPAN_EXCLUSIVE_EXCLUSIVE);
            helloTextView.setText(angle);

            if (node_sp.toggle) {
                node_sp.Orientation_arr=orientationAngles.clone();
                node_sp.toggle = FALSE;
                avg=1;
            } else {
                for(int i=0;i<3;i++) {
                    node_sp.Orientation_arr[i] = (node_sp.Orientation_arr[i]*avg + orientationAngles[i])/(avg+1);
                }
                avg=avg+1;
            }
        }

    }



    @Override
    public void onMapReady(final GoogleMap googleMap) {
        // Add a marker in Sydney, Australia,
        // and move the map's camera to the same location.
        LatLng inital_LatLon = new LatLng(19.1306407, 72.9185153);
        inital_marker = googleMap.addMarker(new MarkerOptions().position(inital_LatLon)
                .title("Initial Marker"));
        googleMap.moveCamera(CameraUpdateFactory.newLatLng(inital_LatLon));
        System.out.println("before map");
        if(googleMap!=null) {
            googleMap.animateCamera(CameraUpdateFactory.zoomTo(18.0f));
            googleMap.setMapType(GoogleMap.MAP_TYPE_NORMAL);

            googleMap.setOnMapClickListener(new GoogleMap.OnMapClickListener() {
                @Override
                public void onMapClick(LatLng point) {
                    if(googleMap!=null) {
                        MarkerOptions marker = new MarkerOptions()
                                .position(new LatLng(point.latitude, point.longitude))
                                .title("New Marker")
                                .icon(BitmapDescriptorFactory.fromResource(R.drawable.target_small));

                        googleMap.addMarker(marker);

                        node_sp.target_cords[0] = point.latitude;
                        node_sp.target_cords[1] = point.longitude;

                        Polyline target_line = googleMap.addPolyline(new PolylineOptions()
                                .add(startLatLon, point)
                                .width(5)
                                .color(Color.BLUE));
                    }
                }
            });
            googleMap_g = googleMap;
            startRepeatingTask();
        }
        else{
            System.out.println("map is null");
        }

    }

    void startRepeatingTask() {
        mRouteDraw.run();
    }


    //Map Route -> Add marker, arrow for heading

    Runnable mRouteDraw = new Runnable() {
        @Override
        public void run() {
        try {
            if(node_sp!=null && googleMap_g!=null) {
                if(!target_coords_rcvd ){
                    if(node_sp.tg_obj.msg_data[0] != 0) {
                        target_coords_rcvd = TRUE;
                        for (int i = 0; i < 10; i += 2) {
                            if (node_sp.tg_obj.msg_data[i] != 0) {
                                MarkerOptions marker = new MarkerOptions()
                                        .position(new LatLng(node_sp.tg_obj.msg_data[i], node_sp.tg_obj.msg_data[i + 1]))
                                        .title("Marker")
                                        .icon(BitmapDescriptorFactory.fromResource(R.drawable.target_small));
                                googleMap_g.addMarker(marker);
                            }
                        }
                    }
                }

//                if(abs(node_sp.message_data[0])>0.1 && abs(node_sp.message_data[1])>0.1 && !source_gps_rcvd)
//                {
//                    startLatLon = new LatLng(node_sp.message_data[0], node_sp.message_data[1]);
//                    previousLatLon = startLatLon;
//
//                    inital_marker.remove();
//
//                    googleMap_g.addMarker(new MarkerOptions().position(startLatLon)
//                            .title("Initial Marker"));
//                    googleMap_g.moveCamera(CameraUpdateFactory.newLatLng(startLatLon));
//                    source_gps_rcvd = true;
//                }
//                else if( source_gps_rcvd && abs(node_sp.target_cords[0])>0.1 && abs(node_sp.target_cords[1])>0.1){
//                    System.out.println("previousLatLon   "+previousLatLon.latitude+"---"+ previousLatLon.longitude);
//                    System.out.println("message data   "+node_sp.message_data[0]+"---"+ node_sp.message_data[1]);
//                    if (abs(node_sp.message_data[0] - previousLatLon.latitude) > 0.000003 || abs(node_sp.message_data[1] - previousLatLon.longitude) > 0.000003) {
//                        Polyline line = googleMap_g.addPolyline(new PolylineOptions()
//                                .add(previousLatLon, new LatLng(node_sp.message_data[0], node_sp.message_data[1]))
//                                .width(5)
//                                .color(Color.RED));
//
//
//                        if(marker!=null)
//                            marker.remove();
//
//                        MarkerOptions marker_options = new MarkerOptions()
//                                .position(new LatLng(node_sp.message_data[0], node_sp.message_data[1]))
//                                .icon(BitmapDescriptorFactory.fromResource(R.drawable.arrow_small))
//                                .anchor(0.5f, 0.5f)
//                                .rotation((float) node_sp.message_data[2]);
//
//
//
//                        marker = googleMap_g.addMarker(marker_options);
//                        previousLatLon = new LatLng(node_sp.message_data[0], node_sp.message_data[1]);
////                        final TextView helloTextView = findViewById(R.id.textView3);
////                        helloTextView.setTextColor(Color.WHITE);
////                        helloTextView.setText("Steps:" + steps + " Message: Lat : " + node_sp.message_data[0] + " Lon : " + node_sp.message_data[1]);
//                    }
//                }
            }
        }finally {
                // 100% guarantee that this always happens, even if
                // your update method throws an exception
                mRouteHandler.postDelayed(mRouteDraw, mInterval);
            }
        }
    };
}




/// node_tl.targets_coords