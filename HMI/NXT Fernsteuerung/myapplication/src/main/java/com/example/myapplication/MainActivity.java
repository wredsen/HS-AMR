package com.example.myapplication;
import java.util.ArrayList;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.content.Intent;
import android.graphics.Color;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.util.TypedValue;
import android.view.Menu;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;
import amr.plt.rcParkingRobot.AndroidHmiPLT;
import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.Toolbar;
import androidx.constraintlayout.widget.ConstraintLayout;
import androidx.constraintlayout.widget.ConstraintSet;
import androidx.core.content.ContextCompat;
import android.os.Handler;
import com.google.android.material.button.MaterialButton;
import com.sothree.slidinguppanel.SlidingUpPanelLayout;
import amr.plt.rcParkingRobot.IAndroidHmi;
import parkingRobot.IGuidance;
import parkingRobot.INxtHmi;
import android.view.MenuItem;
import java.lang.Math;

/**
 * Activity for loading layout resources and handling UI<br>
 * App is using SlidingPanel preset from the user umano on Github - https://github.com/umano/AndroidSlidingUpPanel<br>
 * the project is licensed under Apache License 2.0 - http://www.apache.org/licenses/LICENSE-2.0<br>
 */
public class MainActivity extends AppCompatActivity {
    //representing local Bluetooth adapter
    BluetoothAdapter mBtAdapter = null;
    //representing the bluetooth hardware device
    BluetoothDevice btDevice = null;
    //instance handels bluetooth communication to NXT
    AndroidHmiPLT hmiModule = null;
    // bool to check if device already connected to robo
    boolean connected = false;
    // representing Map robo is driving on, used for displaying trail, parking slots and current position
    Map map = null;
    // representing parking slot handling
    Parking parking = null;

    //request code
    final int REQUEST_SETUP_BT_CONNECTION = 1;
    // driving mode (1 = PAUSE, 0 = Continue Previous Mode)
    int drive_mode = 1;
    // bool to dis-/enable trail
    boolean trail_bool = true;

    //handler to handle runnables
    Handler update_handler = new Handler();
    // required for converter
    Context context = null;
    //representing converter to convert from cm to px on used display
    Converter converter = null;
    private Menu menu;


    /**
     * called when Activity gets started <br>
     * handling "toggle_mode" button behaviour <br>
     * handling Sliding Panel behaviour <br>
     * App is using SlidingPanel preset from the user umano on Github - https://github.com/umano/AndroidSlidingUpPanel<br>
     * the project is licensed under Apache License 2.0 - http://www.apache.org/licenses/LICENSE-2.0<br>
     * creates Instant of Converter, cause context is only not null after onCreate has been called<br>
     * @param savedInstanceState saved State of App (e.g. when it is in background)
     */
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Toolbar toolbar = findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);

        //get the BT-Adapter
        mBtAdapter = BluetoothAdapter.getDefaultAdapter();
        //If the adapter is null, then Bluetooth is not supported
        if (mBtAdapter == null) {
            Toast.makeText(this, "Bluetooth is not available", Toast.LENGTH_LONG).show();
            finish();
            return;
        }

        //toggle button allows user to set mode of the NXT device
        final MaterialButton modeButton = findViewById(R.id.toggle_mode);
        modeButton.setEnabled(false);
        //on click change mode
        modeButton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                if (hmiModule != null) {
                    if (drive_mode == 1) {
                        //if previous mode is PAUSE change mode to SCOUT
                        hmiModule.setMode(INxtHmi.Mode.SCOUT);
                        Log.i("Toggle", "Toggled to Scout");
                        modeButton.setText(R.string.pause);
                        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.LOLLIPOP) {
                            modeButton.setIcon(getDrawable(R.drawable.pause));
                        }
                        drive_mode = 0;
                    } else {
                        // otherwise change mode to PAUSE
                        hmiModule.setMode(INxtHmi.Mode.PAUSE);
                        Log.i("Toggle", "Toggled to Pause");
                        modeButton.setText(R.string.Continue);

                        if (android.os.Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
                            modeButton.setIcon(getDrawable(R.drawable.play));
                        }
                        drive_mode = 1;
                    }
                }
            }
        });

        SlidingUpPanelLayout mLayout = findViewById(R.id.panel);
        mLayout.addPanelSlideListener(new SlidingUpPanelLayout.SimplePanelSlideListener() {
            @Override
            public void onPanelSlide(View panel, float slideOffset) {
                ConstraintLayout map_layout = findViewById(R.id.map_layout);
                //int value = (int) (dptoPx(100f)+ slideOffset*dptoPx(90f));
                // resizes map according to panelposition
                map_layout.setScaleY(1f - 0.2f * slideOffset);
                map_layout.setScaleX(1f - 0.2f * slideOffset);
                map_layout.setTranslationY(converter.dptoPx(0f - 52f * slideOffset));
            }
        });
        context = this;
        converter = new Converter(this, context);

    }

    /**
     * inflates the menu and adds its elements to the toolbar<br>
     * @param menu
     * @return always true
     */
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu_main; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        this.menu = menu;
        return true;
    }

    /**
     *  method to bind bluetooth and trail button to action<br>
     *  bluetooth - inflate bluetooth window<br>
     *  settings - enabled/disable robot trail<br>
     *  called when one of the items in the toolbar has been clicked<br>
     * @param item item that has been clicked
     * @return true
     */
    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.bluetooth:
                Intent serverIntent = new Intent(getApplicationContext(), BluetoothActivity.class);
                serverIntent.putExtra("already_connected", connected);
                startActivityForResult(serverIntent, REQUEST_SETUP_BT_CONNECTION);
                return true;
            case R.id.settings:
                if (trail_bool) {
                    item.setTitle("Turn Trail On");
                    trail_bool = false;
                } else {
                    item.setTitle("Turn Trail Off");
                    trail_bool = true;
                }
                return true;
            default:
                // default - unknown action has been recognized
                // let super class handle it
                return super.onOptionsItemSelected(item);
        }
    }

    /**
     * Connect to the chosen device <br>
     * Method was already provided<br>
     * @param data
     */
    private void establishBluetoothConnection(Intent data) {
        //get instance of the chosen bluetooth device
        String address = data.getExtras().getString(BluetoothActivity.EXTRA_DEVICE_ADDRESS);
        btDevice = mBtAdapter.getRemoteDevice(address);
        //get name and address of the device
        String btDeviceAddress = btDevice.getAddress();
        String btDeviceName = btDevice.getName();

        //instantiate client module
        hmiModule = new AndroidHmiPLT(btDeviceName, btDeviceAddress);

        //connect to the specified device
        hmiModule.connect();

        //wait till connection really is established and
        int i = 0;
        while (!hmiModule.isConnected() && i < 100000000) {
            i++;
        }
    }

    /**
     * instantiating AndroidHmiPlt object via establishBluetoothConnection method and display received data(non-Javadoc) <br>
     * Method was already provided and has been changed <br>
     * {@link com.example.myapplication.MainActivity#establishBluetoothConnection(Intent)}<br>
     * establishes Bluetooth connection and instantiates AndroidHmiPlt object<br>
     * @see android.app.Activity #onActivityResult(int, int, android.content.Intent)
     */
    @Override
    public void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        switch (resultCode) {
            //user pressed back button or cancel on bluetooth activity, so return to initial screen
            case Activity.RESULT_CANCELED:
                break;
            //clicked disconnect button
            case Activity.RESULT_FIRST_USER:
                // restart program
                onBackPressed();
                break;
            case Activity.RESULT_OK:
                //connect to chosen NXT
                establishBluetoothConnection(data);
                if (hmiModule.connected) {
                    //enable toggle button
                    connected = true;
                    TextView status = findViewById(R.id.status);
                    status.setText("Bluetooth connected");
                    //sets start state of buttons to be able to control robot properly
                    final MaterialButton modeButton = findViewById(R.id.toggle_mode);
                    modeButton.setEnabled(true);
                    final MaterialButton parkingButton = findViewById(R.id.parking);
                    parkingButton.setEnabled(false);
                    hmiModule.setMode(INxtHmi.Mode.PAUSE);
                    //display received data from NXT
                    displayDataNXT();
                    break;
                } else {
                    Toast.makeText(this, "Bluetooth connection failed!", Toast.LENGTH_SHORT).show();
                    Toast.makeText(this, "Is the selected NXT really present & switched on?", Toast.LENGTH_LONG).show();
                    break;
                }
        }
    }

    /**
     * method responsible to manage runnables to retrieve and display data, parking slots, position of the robot <br>
     * runnables are managed by update_handler<br>
     * runnables:<br>
     * map_update --> responsible for updating and displaying telemetry data, robot position and trail of his path<br>
     * parking_update --> responsible for showing and updating detected parking slots <br>
     * update_runnable --> responsible for status of park button and EXIT status of robot<br>
     */
    private void displayDataNXT() {
        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.LOLLIPOP) {
            menu.getItem(0).setIcon(R.drawable.bluetooth_connected);
        }
        // displays for 1s that Bluetooth has been connected before showing current robot mode in toolbar
        do {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } while (hmiModule == null);
        map = new Map(this, hmiModule, converter);
        parking = new Parking(this, hmiModule, converter, context);


        Runnable map_update = new Runnable() {
            @Override
            public void run() {
                map.positioning(trail_bool);
                // restarting this runnable after 100ms
                update_handler.postDelayed(this, 100);
            }
        };

        Runnable parking_update = new Runnable() {
            @Override
            public void run() {
                parking.update_parking();
                parking.update_parking_data();
                // restarting this runnable after 100ms
                update_handler.postDelayed(this, 500);
            }
        };

        // handle behaviour of parkButton
        Runnable update_runnable = new Runnable() {
            boolean flag = false;
            public void run() {
                MaterialButton parkButton = findViewById(R.id.parking);
                MaterialButton modeButton = findViewById(R.id.toggle_mode);
                if (hmiModule.getCurrentStatus() == IGuidance.CurrentStatus.PARK) {
                    modeButton.setEnabled(false);
                    try {
                        Thread.sleep(600);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    parkButton.setEnabled(true);

                }
                if (hmiModule.getCurrentStatus() == IGuidance.CurrentStatus.PARK_OUT) {
                    flag = true;
                    parkButton.setEnabled(false);
                    modeButton.setEnabled(true);
                }
                if (hmiModule.getCurrentStatus() == IGuidance.CurrentStatus.DRIVING && flag) {
                    parking.parking_flag = false;
                    flag = false;
                    for (int i = 0; i < parking.parking_slots.size(); i++) {
                        parking.parking_slots.get(i).setEnabled(true);
                    }
                }
                if (!parking.parking_flag && hmiModule.getCurrentStatus() == IGuidance.CurrentStatus.DRIVING){
                    if (parking.selected_parking_slot == 0) {
                        parkButton.setEnabled(false);
                    } else {
                        parkButton.setEnabled(true);
                    }
                }
                else if (!parking.parking_flag && hmiModule.getCurrentStatus() == IGuidance.CurrentStatus.INACTIVE){
                    parkButton.setEnabled(false);
                }
                if (hmiModule.getCurrentStatus() == IGuidance.CurrentStatus.EXIT) {
                   terminateBluetoothConnection();
                    restartActivity();
                }
                // restarting this runnable after 100ms
                update_handler.postDelayed(this, 100);
            }
        };
        // starting runnables
        update_handler.post(map_update);
        update_handler.post(parking_update);
        update_handler.post(update_runnable);

    }

    /**
     * frees bluetooth-adapter when closing app<br>
     * method was already provided<br>
     */
    @Override
    public void onDestroy() {
        super.onDestroy();
        if (mBtAdapter != null) {
            mBtAdapter.cancelDiscovery();
        }
    }

    /**
     * Terminate the bluetooth connection to NXT<br>
     * method was already provided<br>
     */
    private void terminateBluetoothConnection() {
        update_handler.removeCallbacksAndMessages(null);
        Toast.makeText(this, "Bluetooth connection was terminated!", Toast.LENGTH_LONG).show();
        hmiModule.setMode(INxtHmi.Mode.DISCONNECT);
        hmiModule.disconnect();
        while (hmiModule.isConnected()) {
            //wait until disconnected
        }
        hmiModule = null;
    }

    /**
     * restarts the activity.<br>
     * method was already provided<br>
     */
    private void restartActivity() {
        Intent restartIntent = new Intent(getApplicationContext(), MainActivity.class);
        startActivity(restartIntent);
        finish();
    }

    /**
     * handling a pressed back button if connected<br>
     * terminates Bluetooth connection and restarts app<br>
     * @see android.app.Activity#onBackPressed()
     */
    public void onBackPressed() {
        super.onBackPressed();
        if (hmiModule != null && hmiModule.connected) {
            //creating new AlertDialog
            Log.e("disconnect", "now");
            try {
                terminateBluetoothConnection();
            }
            catch (Exception e){
                Log.e("onBackpressed", "exception");
                restartActivity();
            }
        }
        restartActivity();
    }
}

/**
 * provides background map of arena, trail dots, telemetry<br>
 * Singleton class<br>
 */
class Map{
    private int trail_counter = 0;
    private ArrayList<ImageView> trail = new ArrayList<>();
    private float old_position_x = 0;
    private float old_position_y = 0;

    private AndroidHmiPLT hmiModule;
    private Converter converter;
    private Activity activity;
    private ImageView robot;
    private TextView status;
    private TextView fld_xPos;
    private TextView fld_yPos;
    private TextView fld_angle;
    private TextView speed_value;
    private ConstraintLayout map_layout;

    /**
     *  constructor<br>
     *  calls speed_updater to start runnable to calculate current speed of robot<br>
     * @param activity main activity of app
     * @param hmiModule hmiModule
     * @param converter converter to convert cm to pixel
     */
    Map(Activity activity, AndroidHmiPLT hmiModule, Converter converter) {
        this.activity = activity;
        this.hmiModule = hmiModule;
        this.converter = converter;
        robot = activity.findViewById(R.id.robot);
        status = activity.findViewById(R.id.status);
        fld_xPos = activity.findViewById(R.id.x_coords);
        fld_angle = activity.findViewById(R.id.angle_value);
        fld_yPos = activity.findViewById(R.id.y_coords);
        speed_value = activity.findViewById(R.id.speed_value);
        map_layout = activity.findViewById(R.id.map_layout);
        speed_updater();
    }

    /**
     * method responsible for calculating a rough speed estimation<br>
     * comparing position 400ms ago with current position, calculates traveled distance --> calculates speed<br>
     */
    private void speed_updater(){
        Log.i("speed_updater", "Called SpeedUpdater method");
        final Handler speed_handler = new Handler();

        final Runnable speed_runnable = new Runnable() {
            float speed_x_old = 0f;
            float speed_y_old = 0f;
            float distance = 0f;
            float deltax = 0f;
            float deltay = 0f;
            float speed = 0f;
            @Override
            public void run() {
                deltax = Math.abs(speed_x_old - hmiModule.getPosition().getY());
                deltay = Math.abs(speed_y_old - hmiModule.getPosition().getX());
                distance = (float) Math.sqrt(Math.pow(deltax, 2f) + Math.pow(deltay, 2f));
                speed = Math.round(distance/ 0.4f);
                speed_value.setText(String.valueOf((speed + " cm/s ")));
                speed_x_old =  hmiModule.getPosition().getY();
                speed_y_old = hmiModule.getPosition().getX();
                speed_handler.postDelayed(this, 400);
            }
        };
        speed_handler.post(speed_runnable);
    }

    /**
     *  method responsible for updating current position and angle of robot and calling method to set trail<br>
     *  updates telemetry shown on panel(coordinates, angle, amount of parking slots, selected ParkId)<br>
     * @param trail_bool bool to disable/enable trail
     */
    void positioning(boolean trail_bool){
            if (trail_counter == 3) {
                if ((old_position_x - hmiModule.getPosition().getX()) != 0 || (old_position_y - hmiModule.getPosition().getY() != 0)) {
                    if (trail_bool){
                        setTrail(converter.xToPx(hmiModule.getPosition().getY()), converter.yToPx(hmiModule.getPosition().getX()));
                        trail_counter = 0;
                        robot.bringToFront();
                    }
                    else{
                        for (int i = 0; i < trail.size(); i++){
                            ImageView trail_dot = trail.get(0);
                            map_layout.removeView(trail_dot);
                        }
                        trail.clear();
                        trail_counter = 0;
                    }
                }
            } else {
                trail_counter++;
            }
            // get Current Position of robot
            old_position_y = hmiModule.getPosition().getY();
            old_position_x = hmiModule.getPosition().getX();

            // display current status
            status.setText(String.valueOf(hmiModule.getCurrentStatus()));

            //display x value
            fld_xPos.setText(String.valueOf("x = " + hmiModule.getPosition().getX() + " cm"));

            //display y value
            fld_yPos.setText(String.valueOf("y = " + hmiModule.getPosition().getY() + " cm"));

            //display angle value
            fld_angle.setText(String.valueOf(hmiModule.getPosition().getAngle() + "°"));

            // set current position (x,y, angle) of robot icon --> displays current position of robot in arena
            robot.setTranslationY(converter.yToPx(hmiModule.getPosition().getX()));
            robot.setTranslationX(converter.xToPx(hmiModule.getPosition().getY()));
            robot.setRotation(-hmiModule.getPosition().getAngle());
        }

    /**
     * method responsible for updating and setting trail path <br>
     * at first it will fill the arrayList with trail dots <br>
     * after array list has reached specified size (var trail_length)<br>
     * @param posx current x_coordinate of robot
     * @param posy current y_coordinate of robot
     */
    private void setTrail(float posx, float posy) {
        int trail_length = 10;
        if (trail.size() != trail_length) {
            ImageView trail_dot = setTrailDot(posx, posy, trail.size());
            trail.add(trail_dot);
        }
        else {
            ImageView trail_dot = trail.get(0);
            map_layout.removeView(trail_dot);
            trail.remove(0);
            trail_dot = setTrailDot(posx, posy, trail_dot.getId());
            trail.add(trail_dot);
            for (int i = 0; i < 6; i++) {
                trail_dot = trail.get(i);
                trail_dot.setImageAlpha(100 + i * 30);
            }
        }
    }

    /**
     * creates an ImageView, adds it to map_layout and sets Constraints<br>
     * @param posx x coordinate of trail dot
     * @param posy y coordinate of trail dot
     * @param id id the trail dot has to have
     * @return trail_dot as imageView
     */
    private ImageView setTrailDot(float posx, float posy, int id) {
        //ConstraintLayout map_layout = (ConstraintLayout) findViewById(R.id.map_layout);
        ConstraintSet constraintSet = new ConstraintSet();
        ImageView trail_dot = new ImageView(activity);
        trail_dot.setImageResource(R.drawable.trail);
        trail_dot.setId(id);
        ConstraintLayout.LayoutParams layoutParams = new ConstraintLayout.LayoutParams((int) converter.dptoPx(15f), (int) converter.dptoPx(15f));
        trail_dot.setLayoutParams(layoutParams);
        map_layout.addView(trail_dot);
        constraintSet.clone(map_layout);
        constraintSet.connect(trail_dot.getId(), ConstraintSet.RIGHT, R.id.map_line, ConstraintSet.RIGHT, 0);
        constraintSet.connect(trail_dot.getId(), ConstraintSet.TOP, R.id.map_line, ConstraintSet.TOP, 0);
        constraintSet.connect(trail_dot.getId(), ConstraintSet.BOTTOM, R.id.map_line, ConstraintSet.BOTTOM, 0);
        constraintSet.connect(trail_dot.getId(), ConstraintSet.LEFT, R.id.map_line, ConstraintSet.LEFT, 0);
        constraintSet.setHorizontalBias(trail_dot.getId(), 0.653f);
        constraintSet.setVerticalBias(trail_dot.getId(), 0.97f);
        constraintSet.applyTo(map_layout);
        trail_dot.setTranslationX(posx);
        trail_dot.setTranslationY(posy);
        return trail_dot;
    }
}


/**
 * class responsible for handling (adding) parking slots and their behaviour<br>
 * Singleton class<br>
 */
class Parking{
    private AndroidHmiPLT hmiModule;
    private Converter converter;
    private Context context;
    private Activity activity;
    private float parking_threshold_x = 20f;
    private float parking_threshold_y = 170;
    ArrayList<Boolean> parking_slot_button = new ArrayList<Boolean>();
    int selected_parking_slot = 0;
    private int no_parking_slots = 0;
    ArrayList<ImageButton> parking_slots = new ArrayList<>();
    private TextView park_slot_number;
    private MaterialButton parkButton;
    private int park_mode = 0;
    private boolean blink = false;
    private int counter = 0;
    boolean parking_flag = false;

    /**
     *  constructor<br>
     *  calling parking_blink to bind parkbutton to action<br>
     * @param activity main acitivity of app
     * @param hmiModule hmiModule
     * @param converter converter object to convert cm to pixel
     * @param context context of activity, used to set color of parking_slot buttons
     */
    Parking(Activity activity, final AndroidHmiPLT hmiModule, Converter converter, Context context) {
        this.activity = activity;
        this.hmiModule = hmiModule;
        this.converter = converter;
        this.context = context;
        park_slot_number = activity.findViewById(R.id.park_no_value);
        parkButton = activity.findViewById(R.id.parking);
        final MaterialButton parkButton = activity.findViewById(R.id.parking);
        parkButton.setEnabled(false);
        parking_blink();
    }

    /**
     *  creates onClickListener for parkButton in panel<br>
     *  if button is clickable and gets clicked it calls setMode to tell guidance to activate PARK_THIS<br>
     *  all parking_slot buttons and the parkButton will be disabled<br>
     *  while parking button will blink periodically (1s) through changing its color (achieved through runnable parking_runnable)<br>
     *  when status has changed to PARK blinking will stop<br>
     */
    private void parking_blink(){
        final Handler park_handler = new Handler();
        parkButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (park_mode == 0) {
                    parking_flag = true;
                    hmiModule.setMode(INxtHmi.Mode.PARK_THIS);
                    parkButton.setText("Ausparken");
                    parkButton.setEnabled(false);
                    park_mode = 1;

                    for (int i = 0; i < parking_slots.size(); i++){
                        parking_slots.get(i).setEnabled(false);
                    }
                    Runnable parking_runnable = new Runnable() {
                        public void run() {
                            int id = selected_parking_slot - 1;
                            if (hmiModule.getCurrentStatus() != IGuidance.CurrentStatus.PARK){
                                if (blink){
                                    parking_slots.get(id).setColorFilter(ContextCompat.getColor(context, R.color.colorLightPrimary));
                                    blink = false;
                                }
                                else{
                                    parking_slots.get(id).setColorFilter(ContextCompat.getColor(context, R.color.colorAccent));
                                    blink = true;
                                }
                                park_handler.postDelayed(this, 1000);
                            }
                            else{
                                parking_slots.get(id).setColorFilter(ContextCompat.getColor(context, R.color.colorLightPrimary));
                            }
                        }
                    };
                    park_handler.postDelayed(parking_runnable, 100);
                } else {
                    hmiModule.setMode(INxtHmi.Mode.PARK_OUT);
                    parkButton.setText("Park");
                    parkButton.setEnabled(false);
                    park_mode = 0;
                    selected_parking_slot = 0;
                }
            }
        });
    }

    /**
     * executed through runnable<br>
     * checks if parking slot has been found and calls method to ads it visually<br>
     * calls method to check if parking slot has been modified, applies modification<br>
     */
    void update_parking() {
        park_slot_number.setText(String.valueOf(hmiModule.getNoOfParkingSlots()));
        if (no_parking_slots < hmiModule.getNoOfParkingSlots()) {
            counter ++;
            no_parking_slots ++;
            addParkingSlot(no_parking_slots);
        }
    }

    /**
     * method responsible for updating positions and status ((not)suitable --> (not)visible) of parking slots if they have changed<br>
     * iterating through all parking slots in list, checking if positions/dimensions have changed<br>
     * if yes, applies those changes<br>
     * lastly checks if status has changed --> differs visibility of parking slot<br>
     */
    void update_parking_data(){
        Log.e("update_parking_data", "entered method");
        for (int id = 0; id < parking_slots.size(); id++){
            IAndroidHmi.ParkingSlot parking_slot_object_map = hmiModule.getParkingSlot(id+1);
            float posx_start = 100f * parking_slot_object_map.getBackBoundaryPosition().y;
            float posy_start = 100f * parking_slot_object_map.getBackBoundaryPosition().x;
            float posx_end = 100f * parking_slot_object_map.getFrontBoundaryPosition().y;
            float posy_end = 100f * parking_slot_object_map.getFrontBoundaryPosition().x;
            ImageButton parking_slot_object_hmi = parking_slots.get(id);
            if (posx_start < parking_threshold_x && posy_start < parking_threshold_y) {
                int x = (int) converter.xToPx(-28.5f);
                int y = (int) converter.yToPx(-(posy_end - posy_start));
                Log.e("update_parking_data", "hmi:" + parking_slot_object_hmi.getLayoutParams().height + " navigation: " + y);
                if ( y != parking_slot_object_hmi.getLayoutParams().height || parking_slot_object_hmi.getTranslationY() != converter.yToPx(posy_start)){
                Log.e("update_parking_data", "right parking slot update");
                ViewGroup.LayoutParams layoutParams = parking_slot_object_hmi.getLayoutParams();
                layoutParams.height = y;
                layoutParams.width = x;
                parking_slot_object_hmi.setLayoutParams(layoutParams);
                parking_slot_object_hmi.setTranslationY(converter.yToPx(posy_start + 10f));
                parking_slot_object_hmi.setTranslationX(converter.xToPx(0f));
               }
            } else if (posx_start > parking_threshold_x && posy_start < parking_threshold_y) {
                int x = (int) converter.xToPx(-30.5f);
                int y = (int) converter.yToPx(-(posy_start - posy_end));
                if ( y != parking_slot_object_hmi.getLayoutParams().height || parking_slot_object_hmi.getTranslationY() != converter.yToPx(posy_end)){
                    Log.e("update_parking_data", "top parking slot update");
                ViewGroup.LayoutParams layoutParams = parking_slot_object_hmi.getLayoutParams();
                layoutParams.height = y;
                layoutParams.width = x;
                parking_slot_object_hmi.setLayoutParams(layoutParams);
                parking_slot_object_hmi.setTranslationY(converter.yToPx(posy_end+ 10f));
                parking_slot_object_hmi.setTranslationX(converter.xToPx(77.9f));
               }
            } else if (posy_start > parking_threshold_y) {
                int x = (int) converter.xToPx(-(posx_end - posx_start));
                int y = (int) converter.yToPx(-36f);
                if ( x != parking_slot_object_hmi.getLayoutParams().width || parking_slot_object_hmi.getTranslationX() != converter.xToPx(posx_start)){
                    Log.e("update_parking_data", "left parking slot update");
                ViewGroup.LayoutParams layoutParams = parking_slot_object_hmi.getLayoutParams();
                layoutParams.height = y;
                layoutParams.width = x;
                parking_slot_object_hmi.setLayoutParams(layoutParams);
                parking_slot_object_hmi.setTranslationY(converter.yToPx(196.2f));
                parking_slot_object_hmi.setTranslationX(converter.xToPx(posx_start + 38.5f));
                }
            }
            parking_slot_object_hmi.bringToFront();
            if (parking_slot_object_map.getParkingSlotStatus() == IAndroidHmi.ParkingSlot.ParkingSlotStatus.SUITABLE_FOR_PARKING){
                parking_slot_object_hmi.setVisibility(View.VISIBLE);
            }
            else{
                parking_slot_object_hmi.setVisibility(View.GONE);
            }
        }
    }

    /**
     *  method to add a parking slot (suitable and non suitable) <br>
     *  all parking slot buttons and their current states are saved in two array lists <br>
     *  retrieves dimensions of parking slot<br>
     *  creates ImageButton and sets id + LayoutParameter of it according to position of parking slot<br>
     *  --> id of button = id of parking slot + 1000<br>
     *  adds button to Constraint Layout and sets constraints<br>
     *  modifies color, appearance and adds button state to array list<br>
     *  adds onClickListener to set behaviour when button pressed <br>
     *  sets parking slot clickable and to non transparent when it is suitable to park<br>
     */
    private void addParkingSlot(int id) {
        Log.e("addParkingSlot", "counter_parkslot: "+ counter);
        IAndroidHmi.ParkingSlot parking_slot_object = hmiModule.getParkingSlot(id);
        float posx_start = 100f * parking_slot_object.getBackBoundaryPosition().y;
        float posy_start = 100f * parking_slot_object.getBackBoundaryPosition().x;
        float posx_end = 100f * parking_slot_object.getFrontBoundaryPosition().y;
        float posy_end = 100f * parking_slot_object.getFrontBoundaryPosition().x;
        Log.e("addParkingSlot", "id" + id);
        Log.e("addParkingSlot", "posx_end: " + posx_end);
        Log.e("addParkingSlot", "posx_start: " + posx_start);
        Log.e("addParkingSlot", "posy_start: " + posy_start);
        Log.e("addParkingSlot", "posy_end: " + posy_end);
        ConstraintLayout map_layout = activity.findViewById(R.id.map_layout);
        ConstraintSet constraintSet = new ConstraintSet();
        ImageButton parking_slot_view = new ImageButton(activity);
        parking_slot_view.setVisibility(View.GONE);
        id = id + 1000;
        parking_slot_view.setId(id);
        parking_slot_view.setEnabled(true);
        parking_slot_view.setImageResource(R.drawable.parking_slot);
        map_layout.addView(parking_slot_view);
        constraintSet.clone(map_layout);
        constraintSet.connect(parking_slot_view.getId(), ConstraintSet.RIGHT, R.id.map_line, ConstraintSet.RIGHT, 0);
        constraintSet.connect(parking_slot_view.getId(), ConstraintSet.BOTTOM, R.id.map_line, ConstraintSet.BOTTOM, 0);
        constraintSet.applyTo(map_layout);
        parking_slot_view.setScaleType(ImageView.ScaleType.FIT_XY);

        if (posx_start < parking_threshold_x && posy_start < parking_threshold_y) {
            int x = (int) converter.xToPx(-28.5f);
            int y = (int) converter.yToPx(-(posy_end - posy_start));

            ViewGroup.LayoutParams layoutParams = parking_slot_view.getLayoutParams();
            layoutParams.height = y;
            layoutParams.width = x;
            parking_slot_view.setLayoutParams(layoutParams);
            parking_slot_view.setTranslationY(converter.yToPx(posy_start + 10f));
            parking_slot_view.setTranslationX(converter.xToPx(0f));
        } else if (posx_start > parking_threshold_x && posy_start < parking_threshold_y) {
            int x = (int) converter.xToPx(-30.5f);
            int y = (int) converter.yToPx(-(posy_start - posy_end));
            ViewGroup.LayoutParams layoutParams = parking_slot_view.getLayoutParams();
            layoutParams.height = y;
            layoutParams.width = x;
            parking_slot_view.setLayoutParams(layoutParams);
            parking_slot_view.setTranslationY(converter.yToPx(posy_end + 10f));
            parking_slot_view.setTranslationX(converter.xToPx(77.9f));
        } else if (posy_start > parking_threshold_y) {
            int x = (int) converter.xToPx(-(posx_end - posx_start));
            int y = (int) converter.yToPx(-36f);
            ViewGroup.LayoutParams layoutParams = parking_slot_view.getLayoutParams();
            layoutParams.height = y;
            layoutParams.width = x;
            parking_slot_view.setLayoutParams(layoutParams);
            parking_slot_view.setTranslationY(converter.yToPx(196.2f));
            parking_slot_view.setTranslationX(converter.xToPx(posx_start + 38.5f));
        }
        parking_slots.add(parking_slot_view);
        parking_slot_view.bringToFront();
        parking_slot_view.setBackgroundColor(Color.TRANSPARENT);
        TypedValue outValue = new TypedValue();
        activity.getTheme().resolveAttribute(android.R.attr.selectableItemBackground, outValue, true);
        parking_slot_view.setBackgroundResource(outValue.resourceId);
        parking_slot_view.setColorFilter(ContextCompat.getColor(context, R.color.colorLightPrimary));
        parking_slot_button.add(false);
        parking_slot_view.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                TextView park_id_value = activity.findViewById(R.id.park_id_value);
                int id = v.getId();
                id = id - 1000;
                for (int i = 0; i < parking_slots.size(); i++){
                    parking_slots.get(i).setColorFilter(ContextCompat.getColor(context, R.color.colorLightPrimary));
                }
                if (id != selected_parking_slot){
                    selected_parking_slot = id;

                    parking_slots.get(id-1).setColorFilter(ContextCompat.getColor(context, R.color.colorAccent));
                }
                else{
                    selected_parking_slot = 0;
                }
                hmiModule.setSelectedParkingSlot(selected_parking_slot);
                park_id_value.setText(String.valueOf(selected_parking_slot));
            }
        });

        if (parking_slot_object.getParkingSlotStatus() == IAndroidHmi.ParkingSlot.ParkingSlotStatus.SUITABLE_FOR_PARKING){
            parking_slot_view.setVisibility(View.VISIBLE);
        }
            Log.e("addParkingSlot", "exit");
    }
}

/**
 * class responsible to convert cm to px dependent on map (screen) size<br>
 * 3 methods to convert x,y dimensions to px and to convert dp to pixel (px)<br>
 * Singleton class
 */
class Converter{
    private Activity activity;
    private Context context;

    /**
     * constructor<br>
     * @param activity activity of the app
     * @param context   context of the app, not really needed as param, could be retrieved from activity
     */
    Converter(Activity activity, Context context){
        this.activity = activity;
        this.context = context;
    }

    /**
     * converting given value/cm to value/px <br>
     * calculation: <br>
     * negative sign necessary to compensate for wrong alignment of coordinate axis <br>
     * 0.555395 is factor determined empirically<br>
     * @param x_position x coordinate as float in cm, param to be converted to px
     * @return converted x_position in pixel
     */
    float xToPx ( float x_position){
        ConstraintLayout mlayout = activity.findViewById(R.id.map_layout);
        float layer_width = mlayout.getWidth();
        return -x_position * layer_width * 0.55395f / 60f;
    }

    /**
     * converting given value/cm to value/px<br>
     * calculation: <br>
     * negative sign necessary to compensate for wrong alignment of coordinate axis <br>
     * 0.78955 is factor determined empirically<br>
     * @param y_position  y coordinate as float in cm, param to be converted to px
     * @return converted y_position in pixel
     */
    float yToPx ( float y_position){
        // get map_layout
        ConstraintLayout mlayout = activity.findViewById(R.id.map_layout);
        // get height of map in pixel
        float layer_height = mlayout.getHeight();
        return (-y_position * layer_height * (0.78955f / 180f));
    }

    /**
     * converting given value/dp to value/px<br>
     * @param dp value dp to be converted to pixel
     * @return  value in pixel
     */
    float dptoPx ( float dp){
        // get pixel density of screen (which is the context)
        float density = context.getResources()
                .getDisplayMetrics()
                .scaledDensity;
        return density * dp;
    }
}