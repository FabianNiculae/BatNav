package com.example.batnav;

import android.Manifest;
import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.location.Criteria;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.media.MediaPlayer;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.os.Looper;
import android.provider.Settings;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import com.google.android.gms.common.api.GoogleApiClient;
import com.google.android.gms.location.FusedLocationProviderClient;
import com.google.android.gms.location.LocationCallback;
import com.google.android.gms.location.LocationRequest;
import com.google.android.gms.location.LocationResult;
import com.google.android.gms.location.LocationServices;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.tasks.OnCompleteListener;
import com.google.android.gms.tasks.OnSuccessListener;
import com.google.android.gms.tasks.Task;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;
import java.io.InputStream;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.TimeUnit;

import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.os.AsyncTask;
import android.os.Bundle;
import android.util.Log;
import android.widget.Toast;

import java.io.IOException;
import java.util.UUID;

public class MainActivity extends AppCompatActivity {
    /*
        Buttons and Outdoor Localization

     */

    Button start, stop, swSound, plus, minus, swGPS;
    boolean startClicked = false;
    boolean startClickedSound = true;
    private FusedLocationProviderClient fusedLocationProviderClient;
    LocationRequest locationRequest;
    private long lastTouchTime = 0;
    private long currentTouchTime = 0;

    private static final String MAC_ADDR = "98:DA:60:00:BE:A6";
    static final UUID myUUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    private ProgressDialog progress;
    BluetoothAdapter myBluetooth = null;
    BluetoothSocket btSocket = null;
    private boolean isBtConnected = false;


    /*
        Indoor Localization

     */
    // TAG for log debug
    private static final String TAG = "utwente";

    // JSON File with predefined beacons in Ravelijn building
    //private static final String FILE_NAME = "beacons.json";

    // Predefined final values
    private static final int REQUEST_CODE = 101;
    private static final double MS_POWER = -69;
    private static final double N = 2;

    // Delay time between scans - Turned OFF now
    private static final long SCAN_PERIOD = 10000; //ms
    // Var that holds one MAC for a simulated beacon on mobile phones - DEBUG
    private static final String beaconSimMac = "78:08:bd:9d:e0:d1";
    // Bluetooth stuff
    BluetoothManager blManager;
    BluetoothAdapter blAdapter = BluetoothAdapter.getDefaultAdapter();


    // the next line crashes the app


    BluetoothLeScanner blLeScanner = blAdapter.getBluetoothLeScanner();


    // Handler for Intent
    Handler mHandler = new Handler();
    // All the beacons info from the JSON file in an Array of JSON Objects
    JSONArray beaconsArray;
    // Device current location
    Location currentLocation;

    // Beacons
    HashMap<String, Double> nearestBeacons = new HashMap<>();
    HashMap<String, Location> posBeacons = new HashMap<>();
    // We can stop the scan at anytime by changing this value to true
    private boolean scanningEnd = false;
    private GoogleMap mMap;
    private boolean posDetected = false;

    private Location locationBeaconA;
    private Location locationBeaconB;
    private Location locationBeaconC;

    private String macBeaconA;
    private String macBeaconB;
    private String macBeaconC;

    private Double distanceBeaconA;
    private Double distanceBeaconB;
    private Double distanceBeaconC;
    private boolean markerAdded = false;
    private boolean beaconMarkersSet = false;

    // Ravelijn building coordinates
    double rLat = 52.2393990;
    double rLng = 6.8556999;

    /**
     * Executes after BT scan has been made
     */
    private final ScanCallback mLeScanCallback = new ScanCallback() {

        /**
         * Executes when Bluetooth Scan comes up with results
         * @logs error code along with the tag
         */
        @SuppressLint("LongLogTag")
        @RequiresApi(api = Build.VERSION_CODES.O)
        @Override
        public void onScanResult(int callbackType, ScanResult result) {
            super.onScanResult(callbackType, result);

            /**
             * MAC Address of the scan result
             */
            String resMac = String.valueOf(result.getDevice()).toLowerCase();

            for (int i = 0; i < beaconsArray.length(); i++) {
                //Log.d(TAG, "Comparing " + resMac + " to " + String.valueOf(i) + " from JSON file");
                try {
                    /**
                     * Getting the information of the beacon from JSON file
                     */
                    JSONObject beacon = beaconsArray.getJSONObject(i);
                    int bID = beacon.getInt("beacon_id");
                    String bAddress = beacon.getString("mac_address");
                    String bDeviceName = beacon.getString("device_name");
                    String bLat = beacon.getString("latitude");
                    String bLng = beacon.getString("longitude");
                    int bFloor = beacon.getInt("floor");

                    /**
                     * Checks for a MATCH with the MAC address of the scanned device
                     */
                    //Log.d(TAG+" -debug", "before mac validity check");
                    if (bAddress.equals(resMac) || resMac.equals(beaconSimMac)) {
                        Log.d(TAG + " MATCH", " Found a matching MAC Address: " + resMac + " on floor: " + bFloor);

                        /**
                         * Only floor 1 of Ravelijn building
                         */
                        //if(bFloor == 1){
                        if (true) {
                            /**
                             * Calculate distance to beacon with formula:
                             * distance = 10 ^ ((MEASURED_POWER - SCAN_RSSI) / (10 * N))
                             * MEASURED_POWER = -69    <-    Calibrated 1 RSSI
                             */
                            double distance = Math.pow(10, ((MS_POWER - result.getRssi()) / (10 * N)));
                            Log.d(TAG + " -DEVICE_DETAILS Scan result device details:",
                                    "\n MAC: " + result.getDevice()
                                            + "\n RSSI: " + result.getRssi()
                                            + "\n Distance to beacon: " + String.valueOf(distance) + "m"
                                            + "\n TX Power: " + result.getTxPower()
                                            + "\n Advertising SID: " + result.getAdvertisingSid()
                                            + "\n Data Status: " + result.getDataStatus()
                                            + "\n Primary: " + result.getPrimaryPhy()
                                            + "\n Secondary: " + result.getSecondaryPhy()
                                            + "\n Timestamps: " + result.getTimestampNanos()
                                            + "\n Lat: " + bLat
                                            + "\n Lng: " + bLng
                                            + "\n AD Interval: " + result.getPeriodicAdvertisingInterval());

                            nearestBeacons.put(bAddress, distance);

                            Location beaconLocation = new Location("");
                            beaconLocation.setLatitude(Double.valueOf(bLat));
                            beaconLocation.setLongitude(Double.valueOf(bLng));

                            posBeacons.put(bAddress, beaconLocation);

                            nearestBeacons = sortMapByValue(nearestBeacons);

                            if (nearestBeacons.size() >= 3) {
                                getInfoNearestBeacons(nearestBeacons, 3);
                                Log.d(TAG + "-debugLocation", "Calling function to get location");

                                Log.d(TAG + "-deubgLocation",
                                        "\n Location Beacon A: " + String.valueOf(locationBeaconA)
                                                + "\n Location Beacon B: " + String.valueOf(locationBeaconB)
                                                + "\n  Location Beacon C: " + String.valueOf(locationBeaconC));

                                currentLocation = getLocationWithBeacons(locationBeaconA, locationBeaconB, locationBeaconC, distanceBeaconA, distanceBeaconB, distanceBeaconC);
                                //setBeaconsMarkers();
                                //beaconMarkersSet = true;
                                Log.d(TAG + "-currentLocation", String.valueOf(currentLocation));
//                                updateDeviceLocation(mMap);
//                                markerAdded = true;
                            }
                        }
                    }
                } catch (JSONException e) {
                    e.printStackTrace();
                }
            }
        }

        /**
         * Executes when Bluetooth Scan fails
         * @logs error code along with the tag
         */
        @Override
        public void onScanFailed(int errorCode) {
            super.onScanFailed(errorCode);
            Log.i(TAG + " BTLE SCAN FAIL", String.valueOf(errorCode));
        }
    };

    /**
     * Sort required to get the 3 closest beacons
     * @return sorted HashMap
     */
    private static HashMap sortMapByValue(HashMap map) {
        List list = new LinkedList(map.entrySet());

        Collections.sort(list, new Comparator() {
            public int compare(Object o1, Object o2) {
                return ((Comparable) ((Map.Entry) (o1)).getValue())
                        .compareTo(((Map.Entry) (o2)).getValue());
            }
        });

        HashMap sortedHashMap = new LinkedHashMap();
        for (Iterator it = list.iterator(); it.hasNext(); ) {
            Map.Entry entry = (Map.Entry) it.next();
            sortedHashMap.put(entry.getKey(), entry.getValue());
        }
        //Log.d(TAG + "-inFuncSorted", String.valueOf(sortedHashMap));
        return sortedHashMap;
    }


    /*
            On create method
     */
    @SuppressLint("MissingPermission")
    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        start = (Button) findViewById(R.id.start);
        stop = (Button) findViewById(R.id.stop);
        swSound = (Button) findViewById(R.id.swSound);
        swGPS = (Button) findViewById(R.id.swGPS);
        plus = (Button) findViewById(R.id.plus);
        minus = (Button) findViewById(R.id.minus);

        final MediaPlayer started = MediaPlayer.create(this, R.raw.started);
        final MediaPlayer stopped = MediaPlayer.create(this, R.raw.stopped);
        final MediaPlayer deviceConnected = MediaPlayer.create(this, R.raw.device_connected);
        final MediaPlayer deviceNotConnected = MediaPlayer.create(this, R.raw.device_not_connected);
        final MediaPlayer distanceDecreased = MediaPlayer.create(this, R.raw.distance_decreased);
        final MediaPlayer distanceIncreased = MediaPlayer.create(this, R.raw.distance_increased);
        final MediaPlayer indoorMode = MediaPlayer.create(this, R.raw.indoor_mode);
        final MediaPlayer outdoorMode = MediaPlayer.create(this, R.raw.outdoor_mode);
        final MediaPlayer objCareful = MediaPlayer.create(this, R.raw.objbecareful);
        final MediaPlayer objPayAttention = MediaPlayer.create(this, R.raw.objpayattention);
        final MediaPlayer objWatchOut = MediaPlayer.create(this, R.raw.objwatchout);
        final MediaPlayer muted = MediaPlayer.create(this, R.raw.muted);
        final MediaPlayer unmuted = MediaPlayer.create(this, R.raw.unmuted);
        int[] sounds = {R.raw.objbecareful, R.raw.objpayattention, R.raw.objwatchout};

        myBluetooth = BluetoothAdapter.getDefaultAdapter();
        if (myBluetooth == null) {
            //Show a mensag. that thedevice has no bluetooth adapter
            Toast.makeText(getApplicationContext(), "Bluetooth Device Not Available", Toast.LENGTH_LONG).show();
            //finish apk
            finish();
        } else {
            if (myBluetooth.isEnabled()) {
                ConnectBT p = new ConnectBT(143);
                p.start();
                Toast.makeText(getApplicationContext(), "Bluetooth Device Available", Toast.LENGTH_LONG).show();
            } else {
                //Ask to the user turn the bluetooth on
                Intent turnBTon = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                startActivityForResult(turnBTon, 1);
            }
        }


        /*
                Indoor localization - bluetooth
         */
        stop.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                if (startClicked == true) {
                    startClicked = false;
                    Log.i("Yes", "Stop clicked " + startClicked);
                    stopped.start();
                }

            }
        });


        start.setOnClickListener(new View.OnClickListener() {
            public void onClick(View view) {
                if (startClicked == false) {
                    startClicked = true;
                    Log.i("Yes", "Start clicked " + startClicked);
                    started.start();






                    /*
                        Indoor
                    /*

                        // Getting BT adapter ready



                    /**
                     * Checks if BT adapter is off, if yes it enables it
                     */

                }


            }
        });


        swGPS.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {


                if (swGPS.getText().equals("Outdoor") && startClicked) {
                    swGPS.setText("Indoor");
                    Log.i("Yes", "Stop clicked " + startClicked);
                    indoorMode.start();
                } else if (swGPS.getText().equals("Indoor") && startClicked) {
                    swGPS.setText("Outdoor");
                    Log.i("Yes", "Stop clicked " + startClicked);
                    outdoorMode.start();

                }


            }


        });
        swGPS.setOnLongClickListener(new View.OnLongClickListener() {
            @Override
            public boolean onLongClick(View v) {
                Random r = new Random();
                int Low = 0;
                int High = 3;
                int rndm = r.nextInt(sounds.length);

                MediaPlayer player = MediaPlayer.create(getApplicationContext(), sounds[rndm]);
                player.start();
                return true;
            }
        });


        plus.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                if (startClicked)
                    distanceIncreased.start();

            }
        });

        minus.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                if (startClicked)
                    distanceDecreased.start();

            }
        });

        swSound.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                if (swSound.getText().equals("Phone") && startClicked) {
                    swSound.setText("Arduino");
                    Log.i("Yes", "Stop clicked " + startClicked);
                    muted.start();
                } else if (swSound.getText().equals("Arduino") && startClicked) {
                    swSound.setText("Phone");
                    Log.i("Yes", "Stop clicked " + startClicked);
                    unmuted.start();

                }


            }
        });
        fusedLocationProviderClient = LocationServices.getFusedLocationProviderClient(this);


        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED
                && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(MainActivity.this, new String[]{Manifest.permission.ACCESS_FINE_LOCATION}, 1);
            ActivityCompat.requestPermissions(MainActivity.this, new String[]{Manifest.permission.ACCESS_COARSE_LOCATION}, 2);
        }


        locationRequest = locationRequest.create();
        locationRequest.setInterval(100);
        locationRequest.setFastestInterval(50);
        locationRequest.setPriority(LocationRequest.PRIORITY_HIGH_ACCURACY);

        //instantiating the LocationCallBack
        LocationCallback locationCallback = new LocationCallback() {
            @Override
            public void onLocationResult(LocationResult locationResult) {
                if (locationResult != null) {
                    if (locationResult == null) {
                        return;
                    }
                    //Showing the latitude, longitude and accuracy on the home screen.
                    for (Location location : locationResult.getLocations()) {
                        Log.i("Location", "Lat + long" + location.getLongitude() + location.getLatitude());
                    }
                }
            }
        };
        fusedLocationProviderClient.requestLocationUpdates(locationRequest, locationCallback, Looper.getMainLooper());
    }


    private class ConnectBT extends Thread  // UI thread
    {
        private boolean ConnectSuccess = true; //if it's here, it's almost connected
        long minTime;

        ConnectBT(long minTime) {
            Log.d("DBG22", "Setting min Time");
            this.minTime = minTime;
        }

        public void run() {
            // compute primes larger than minPrime
            Log.d("DBG22", "Pre-execute THREAD");

            doInBackground();

            if (!ConnectSuccess) {
                msg("Connection Failed. Is it a SPP Bluetooth? Try again.");
                finish();
            } else {
                msg("Connected.");
                isBtConnected = true;
            }
            msgON();
        }

        protected Void doInBackground() //while the progress dialog is shown, the connection is done in background
        {
            try {
                if (btSocket == null || !isBtConnected) {
                    Log.d("DBG22", "Starting connection");
                    myBluetooth = BluetoothAdapter.getDefaultAdapter();//get the mobile bluetooth device
                    BluetoothDevice btDevice = myBluetooth.getRemoteDevice(MAC_ADDR);//connects to the device's address and checks if it's available
                    Log.d("DBG22", "Device");

                    btSocket = btDevice.createInsecureRfcommSocketToServiceRecord(myUUID);//create a RFCOMM (SPP) connection
                    Log.d("DBG22", "UUID SET");

                    BluetoothAdapter.getDefaultAdapter().cancelDiscovery();
                    Log.d("DBG22", "Stopped discovery");

                    btSocket.connect();//start connection
                    Log.d("DBG22", "Connected");
                }
            }
            catch (IOException e)
            {
                Log.d("DBG22", "Failed to connect, check exception");
                ConnectSuccess = false;//if the try failed, you can check the exception here
            }
            return null;
        }
    }
    private void msg(String s)
    {
        Log.d("DBG22", "MSG");
        //Toast.makeText(getApplicationContext(),s,Toast.LENGTH_LONG).show();
    }

    private void Disconnect()
    {
        if (btSocket!=null) //If the btSocket is busy
        {
            try
            {
                btSocket.close(); //close connection
            }
            catch (IOException e)
            { msg("Error");}
        }
        finish(); //return to the first layout
    }

    private void msgOFF()
    {
        if (btSocket!=null)
        {
            try
            {
                btSocket.getOutputStream().write("OFF".toString().getBytes());
            }
            catch (IOException e)
            {
                msg("Error");
            }
        }
    }
    private void msgON() {
        if (btSocket != null) {
            try {
                btSocket.getOutputStream().write("ON".toString().getBytes());
            } catch (IOException e) {
                msg("Error");
            }
        }
    }















    /*
     Function that calculates the distance between two locations
     */

    public double CalculationByDistance(LatLng StartP, LatLng EndP) {
        int Radius = 6371;// radius of earth in Km
        double lat1 = StartP.latitude;
        double lat2 = EndP.latitude;
        double lon1 = StartP.longitude;
        double lon2 = EndP.longitude;
        double dLat = Math.toRadians(lat2 - lat1);
        double dLon = Math.toRadians(lon2 - lon1);
        double a = Math.sin(dLat / 2) * Math.sin(dLat / 2)
                + Math.cos(Math.toRadians(lat1))
                * Math.cos(Math.toRadians(lat2)) * Math.sin(dLon / 2)
                * Math.sin(dLon / 2);
        double c = 2 * Math.asin(Math.sqrt(a));
        double valueResult = Radius * c;
        double km = valueResult / 1;

        DecimalFormat newFormat = new DecimalFormat("####");
        int kmInDec = Integer.valueOf(newFormat.format(km));
        double meter = valueResult % 1000;
        int meterInDec = Integer.valueOf(newFormat.format(meter));
        Log.i("Radius Value", "" + valueResult + "   KM  " + kmInDec
                + " Meter   " + meterInDec);

        return Radius * c;
    }


    /**
     * Checks if scanning is requested to be stopped then it stopps it, otherwise starts scan
     */
    private void scanLeDevice() {
        if (!scanningEnd) {
            Log.d(TAG + " -debug", "start scan");
            if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
                // TODO: Consider calling
                //    ActivityCompat#requestPermissions
                // here to request the missing permissions, and then overriding
                //   public void onRequestPermissionsResult(int requestCode, String[] permissions,
                //                                          int[] grantResults)
                // to handle the case where the user grants the permission. See the documentation
                // for ActivityCompat#requestPermissions for more details.
                return;
            }
            blLeScanner.startScan(mLeScanCallback);
        } else {
            blLeScanner.stopScan(mLeScanCallback);
        }


    }

    /**
     * @param beaconA coordinate location of first beacon
     * @param beaconB coordinate location of second beacon
     * @param beaconC coordinate location of third beacon
     * @param distanceA device distance to second beacon
     * @param distanceB device distance to second beacon
     * @param distanceC device distance to second beacon
     *
     * @return beacons map sorted
     */
    @SuppressLint("LongLogTag")
    public Location getLocationWithBeacons(Location beaconA, Location beaconB, Location beaconC, double distanceA, double distanceB, double distanceC) {
        double METERS_IN_COORDINATE_UNITS_RATIO = 0.00001;

        double cogX = (beaconA.getLatitude() + beaconB.getLatitude() + beaconC.getLatitude()) / 3;
        double cogY = (beaconA.getLongitude() + beaconB.getLongitude() + beaconC.getLongitude()) / 3;
        Location cog = new Location("Cog");
        cog.setLatitude(cogX);
        cog.setLongitude(cogY);

        Location nearestBeacon;
        double shortestDistanceInMeters;
        if (distanceA < distanceB && distanceA < distanceC) {
            nearestBeacon = beaconA;
            shortestDistanceInMeters = distanceA;
        } else if (distanceB < distanceC) {
            nearestBeacon = beaconB;
            shortestDistanceInMeters = distanceB;
        } else {
            nearestBeacon = beaconC;
            shortestDistanceInMeters = distanceC;
        }

        double distanceToCog = Math.sqrt(Math.pow(cog.getLatitude() - nearestBeacon.getLatitude(), 2)
                + Math.pow(cog.getLongitude() - nearestBeacon.getLongitude(), 2));

        double shortestDistanceInCoordinationUnits = shortestDistanceInMeters * METERS_IN_COORDINATE_UNITS_RATIO;

        double t = shortestDistanceInCoordinationUnits / distanceToCog;

        Location pointsDiff = new Location("PointsDiff");
        pointsDiff.setLatitude(cog.getLatitude() - nearestBeacon.getLatitude());
        pointsDiff.setLongitude(cog.getLongitude() - nearestBeacon.getLongitude());

        Location tTimesDiff = new Location("tTimesDiff");
        tTimesDiff.setLatitude(pointsDiff.getLatitude() * t);
        tTimesDiff.setLongitude(pointsDiff.getLongitude() * t);

        Location userLocation = new Location("UserLocation");
        userLocation.setLatitude(nearestBeacon.getLatitude() + tTimesDiff.getLatitude());
        userLocation.setLongitude(nearestBeacon.getLongitude() + tTimesDiff.getLongitude());

        return userLocation;
    }

    /**
     * Logs a HashMap
     * @param mp to be looged
     */
    public void logMap(String customTag, HashMap mp) {
        Iterator it = mp.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry pair = (Map.Entry) it.next();
            Log.d(TAG + customTag, pair.getKey() + " = " + pair.getValue());
        }
    }

    public void getInfoNearestBeacons(HashMap mp, int amount) {
        Log.d(TAG + "-debugMAP SIZE", String.valueOf(mp.size()));
        logMap(TAG + "-debugMAP", mp);
        Log.d(TAG + "-debug", "Starting beacon details getter for this number of beacons: " + String.valueOf(amount));

        Iterator it = mp.entrySet().iterator();
        int k = 0;
        while (k < amount) {
            Map.Entry pair = (Map.Entry) it.next();
            if (k == 0) {
                //Log.d(TAG + "-debug", "Getting info beacon A");
                macBeaconA = String.valueOf(pair.getKey());
                //Log.d(TAG + "-debug", "Getting Location beacon A");
                locationBeaconA = posBeacons.get(macBeaconA);
                //Log.d(TAG + "-debug", "Getting distance beacon A");
                distanceBeaconA = (Double) pair.getValue();
                //Log.d(TAG + "-debug", "VALUE OF K: " + String.valueOf(k));
            }

            if (k == 1) {
                macBeaconB = String.valueOf(pair.getKey());
                //Log.d(TAG + "-debug", "Getting info beacon B:");
                //Log.d(TAG + "-debug", "Getting Location beacon B");
                locationBeaconB = posBeacons.get(macBeaconB);
                //Log.d(TAG + "-debug", "Getting distance beacon B");
                distanceBeaconB = (Double) pair.getValue();
                //Log.d(TAG + "-debug", "VALUE OF K: " + String.valueOf(k));
            }

            if (k == 2) {
                //Log.d(TAG + "-debug", "Getting info beacon C");
                macBeaconC = String.valueOf(pair.getKey());
                //Log.d(TAG + "-debug", "Getting Location beacon C");
                locationBeaconC = posBeacons.get(macBeaconC);
                // Log.d(TAG + "-debug", "Getting distance beacon C");
                distanceBeaconC = (Double) pair.getValue();
                //Log.d(TAG + "-debug", "VALUE OF K: " + String.valueOf(k));
            }
            it.remove();
            k = k + 1;
            //Log.d(TAG + "-debug", "VALUE OF K: " + String.valueOf(k));
        }
    }


    /**
     * Reads a JSON file from /app/assets/ and converts it to a string
     * @param file_name the name and extension of the file in a string to be read
     * @return null/JSON string of the json file
     * @throws IOException ReadBuffer exception for I/O
     */
    private String readJsonFile(String file_name) throws IOException {
        String json = null;
        try {
            InputStream is = getAssets().open(file_name);
            int size = is.available();
            byte[] buffer = new byte[size];
            is.read(buffer);
            is.close();
            json = new String(buffer, "UTF-8");
            return json;
        } catch (IOException ex) {
            ex.printStackTrace();
            return null;
        }
    }

    /**
     * Makes the app sleep for an amount of milliseconds
     * @param milliseconds time to sleep
     */
    private void sleepFor(int milliseconds) {
        try {
            TimeUnit.MILLISECONDS.sleep(milliseconds);
        } catch (Exception e) {
            Log.d("Sleep Function error!", String.valueOf(e));
        }
    }

    public void indoorLoc() {
        blManager = (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        blAdapter = blManager.getAdapter();
        if (!blAdapter.isEnabled()) {
            if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
                // TODO: Consider calling
                //    ActivityCompat#requestPermissions
                // here to request the missing permissions, and then overriding
                //   public void onRequestPermissionsResult(int requestCode, String[] permissions,
                //                                          int[] grantResults)
                // to handle the case where the user grants the permission. See the documentation
                // for ActivityCompat#requestPermissions for more details.
                return;
            }
            blAdapter.enable();
        }

        /**
         * Tries to read the file using created method and puts everything to a JSON ARRAY
         * @throws I/O and JSON exceptions
         */
        try {
            String JSONobjString = readJsonFile("beacons.json");
            try {
                beaconsArray = new JSONArray(JSONobjString);
            } catch (JSONException e) {
                Log.d("JSON", "ERROR EXCEPTION ARRAY");
                e.printStackTrace();
            }
        } catch (IOException e) {
            Log.d("JSON", "ERROR EXCEPTION");
            e.printStackTrace();
        }

        /**
         * Start the scan
         */
        sleepFor(8000);
        scanLeDevice();blManager = (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        blAdapter = blManager.getAdapter();
        if (!blAdapter.isEnabled()) {
            blAdapter.enable();
        }

        /**
         * Tries to read the file using created method and puts everything to a JSON ARRAY
         * @throws I/O and JSON exceptions
         */
        try {
            String JSONobjString = readJsonFile("beacons.json");
            try {
                beaconsArray = new JSONArray(JSONobjString);
            } catch (JSONException e) {
                Log.d("JSON", "ERROR EXCEPTION ARRAY");
                e.printStackTrace();
            }
        } catch (IOException e) {
            Log.d("JSON", "ERROR EXCEPTION");
            e.printStackTrace();
        }

        /**
         * Start the scan
         */
        sleepFor(8000);
        scanLeDevice();
    }




}

