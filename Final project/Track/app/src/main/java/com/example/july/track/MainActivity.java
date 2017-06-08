package com.example.july.track;

import android.Manifest;
import android.app.Activity;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.ScrollView;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.widget.TextView;

import com.hoho.android.usbserial.driver.CdcAcmSerialDriver;
import com.hoho.android.usbserial.driver.ProbeTable;
import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;
import static android.graphics.Color.rgb;


public class MainActivity extends Activity implements TextureView.SurfaceTextureListener {
    private Camera mCamera;
    private TextureView mTextureView;
    private SurfaceView mSurfaceView;
    private SurfaceHolder mSurfaceHolder;
    private Bitmap bmp = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
    private Canvas canvas = new Canvas(bmp);
    private Paint paint1 = new Paint();
    private TextView mTextView;
    private SeekBar R_Bar;
    private SeekBar T_Bar;
    //private TextView R_value;
    //private TextView T_value;
    private UsbManager manager;
    private UsbSerialPort sPort;
    private final ExecutorService mExecutor = Executors.newSingleThreadExecutor();
    private SerialInputOutputManager mSerialIoManager;

    private SeekBar speedControl_bar;
    private TextView speedTextView;
    private Button button;
    private TextView myTextView2;
    private ScrollView myScrollView;
    private TextView myTextView3;


    static long prevtime = 0; // for FPS calculation
    int thresh = 0; // comparison value
    int Range=20;
    int Thresh=20;
    float COM;
    int speedControl_init;
    int speedControl_right;
    int speedControl_left;
    float center_pre;
    float aveCOM_pre;

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON); // keeps the screen from turning off

        mTextView = (TextView) findViewById(R.id.cameraStatus);

        R_Bar = (SeekBar) findViewById(R.id.seekR);
       // R_value = (TextView) findViewById(R.id.R_Text);
        T_Bar = (SeekBar) findViewById(R.id.seekT);
      //  T_value = (TextView) findViewById(R.id.T_Text);
        speedControl_bar = (SeekBar) findViewById(R.id.seek1);
        speedTextView = (TextView) findViewById(R.id.textView01);


        myTextView2 = (TextView) findViewById(R.id.textView02);
        myScrollView = (ScrollView) findViewById(R.id.ScrollView01);
        myTextView3 = (TextView) findViewById(R.id.textView03);
        button = (Button) findViewById(R.id.button1);

        setMyControlListener();

        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                speedControl_init = speedControl_bar.getProgress()*12;
                String sendString = String.valueOf(1) + ' ' +String.valueOf(speedControl_init)+'\n';
                try {
                    sPort.write(sendString.getBytes(), 10); // 10 is the timeout
                } catch (IOException e) { }

                myTextView2.setText("value on click is "+speedControl_init);

            }
        });
        manager = (UsbManager) getSystemService(Context.USB_SERVICE);


        // see if the app has permission to use the camera
        //ActivityCompat.requestPermissions(MainActivity.this, new String[]{Manifest.permission.CAMERA}, 1);
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
            mSurfaceView = (SurfaceView) findViewById(R.id.surfaceview);
            mSurfaceHolder = mSurfaceView.getHolder();

            mTextureView = (TextureView) findViewById(R.id.textureview);
            mTextureView.setSurfaceTextureListener(this);

            // set the paintbrush for writing text on the image
            paint1.setColor(0xffff0000); // red
            paint1.setTextSize(24);

            mTextView.setText("started camera");
        } else {
            mTextView.setText("no camera permissions");
        }

    }

    private void setMyControlListener() {
        R_Bar.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                Range = progress;
               // R_value.setText("Range: "+progress);
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        T_Bar.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                Thresh = progress;
               // T_value.setText("Thresh: "+progress);
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        speedControl_bar.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {

            int progressChanged = 0;

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                progressChanged = progress*12;
                String sendString = String.valueOf(1)+' '+String.valueOf(progressChanged) + '\n';
                try {
                    sPort.write(sendString.getBytes(), 10); // 10 is the timeout
                } catch (IOException e) { }
                speedTextView.setText("PWM: "+progressChanged);
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
    }

    private final SerialInputOutputManager.Listener mListener =
            new SerialInputOutputManager.Listener() {
                @Override
                public void onRunError(Exception e) {

                }

                @Override
                public void onNewData(final byte[] data) {
                    MainActivity.this.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            MainActivity.this.updateReceivedData(data);
                        }
                    });
                }
            };
    @Override
    protected void onPause(){
        super.onPause();
        stopIoManager();
        if(sPort != null){
            try{
                sPort.close();
            } catch (IOException e){ }
            sPort = null;
        }
        finish();
    }
    @Override
    protected void onResume() {
        super.onResume();

        ProbeTable customTable = new ProbeTable();
        customTable.addProduct(0x04D8,0x000A, CdcAcmSerialDriver.class);
        UsbSerialProber prober = new UsbSerialProber(customTable);

        final List<UsbSerialDriver> availableDrivers = prober.findAllDrivers(manager);

        if(availableDrivers.isEmpty()) {
            //check
            return;
        }

        UsbSerialDriver driver = availableDrivers.get(0);
        sPort = driver.getPorts().get(0);

        if (sPort == null){
            //check
        }else{
            final UsbManager usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
            UsbDeviceConnection connection = usbManager.openDevice(driver.getDevice());
            if (connection == null){
                //check
                PendingIntent pi = PendingIntent.getBroadcast(this, 0, new Intent("com.android.example.USB_PERMISSION"), 0);
                usbManager.requestPermission(driver.getDevice(), pi);
                return;
            }

            try {
                sPort.open(connection);
                sPort.setParameters(9600, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);

            }catch (IOException e) {
                //check
                try{
                    sPort.close();
                } catch (IOException e1) { }
                sPort = null;
                return;
            }
        }
        onDeviceStateChange();
    }

    private void stopIoManager(){
        if(mSerialIoManager != null) {
            mSerialIoManager.stop();
            mSerialIoManager = null;
        }
    }

    private void startIoManager() {
        if(sPort != null){
            mSerialIoManager = new SerialInputOutputManager(sPort, mListener);
            mExecutor.submit(mSerialIoManager);
        }
    }

    private void onDeviceStateChange(){
        stopIoManager();
        startIoManager();
    }

    private void updateReceivedData(byte[] data) {
        //do something with received data

        //for displaying:
        String rxString = null;
        try {
            rxString = new String(data, "UTF-8"); // put the data you got into a string
            myTextView3.append(rxString);
            myScrollView.fullScroll(View.FOCUS_DOWN);
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
        }
    }

    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        mCamera = Camera.open();
        Camera.Parameters parameters = mCamera.getParameters();
        parameters.setPreviewSize(640, 480);
        parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_INFINITY); // no autofocusing
        parameters.setAutoExposureLock(true); // keep the white balance constant
        parameters.setAutoWhiteBalanceLock((true)); //set white balance off
        mCamera.setParameters(parameters);
        mCamera.setDisplayOrientation(90); // rotate to portrait mode

        try {
            mCamera.setPreviewTexture(surface);
            mCamera.startPreview();
        } catch (IOException ioe) {
            // Something bad happened
        }
    }

    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        // Ignored, Camera does all the work for us
    }

    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        mCamera.stopPreview();
        mCamera.release();
        return true;
    }

//    public boolean checkGreen(int i){
//        if ((green(i) - red(i)) > thresh){
//            return true;
//        }
//        else {
//            return false;
//        }
//    }

//    public boolean checkGray(int i){
//        if (((green(i) - red(i)) > -Range)&&((green(i) - red(i)) < Range)&&(green(i) > Thresh)&&(red(i)>Thresh)) {
//            if(red(i)*65536+green(i)*256+blue(i)>8421504){
//                return true;
//            }
//            else{
//                return false;
//            }
//        }
//        else {
//            return false;
//        }
//    }
        public boolean checkGray(int i){
        if (((green(i) - red(i)) > -Range)&&((green(i) - red(i)) < Range)&&(green(i) > (Thresh*2))&&(red(i)>(Thresh*2))) {
            if(((green(i)-blue(i)>-Range)&&((green(i)-blue(i)) < Range))){
                return true;
            }
            else{
                return false;
            }
        }
        else {
            return false;
        }
    }

//    public float read_masscenter(int row){
//        int[] pixels = new int[bmp.getWidth()];
//        float center;
//        bmp.getPixels(pixels, 0, bmp.getWidth(), 0, row, bmp.getWidth(), 1);
//        int sum_mr = 0; // the sum of the mass times the radius
//        int sum_m = 0; // the sum of the masses
//        for (int i = 0; i < bmp.getWidth(); i++) {
//            if (checkGray(pixels[i])) {
//                pixels[i] = rgb(1, 1, 1); // set the pixel to almost 100% black
//
//                sum_m = sum_m + green(pixels[i])+red(pixels[i])+blue(pixels[i]);
//                sum_mr = sum_mr + (green(pixels[i])+red(pixels[i])+blue(pixels[i]))*i;
//            }
//        }
//        // only use the data if there were a few pixels identified, otherwise you might get a divide by 0 error
//        if(sum_m>5){
//            center = sum_mr / sum_m;
//        }
//        else{
//            center = 0;
//        }
//        return center;
//    }

    public float read_masscenter(int row){
        int[] pixels = new int[bmp.getWidth()];
        float center;
        bmp.getPixels(pixels, 0, bmp.getWidth(), 0, row, bmp.getWidth(), 1);
        int sum_mr = 0; // the sum of the mass times the radius
        int sum_m = 0; // the sum of the masses
        for (int i = 0; i < bmp.getWidth(); i++) {
            if (checkGray(pixels[i])) {
                pixels[i] = rgb(1, 1, 1); // set the pixel to almost 100% black

                sum_m = sum_m + green(pixels[i])+red(pixels[i])+blue(pixels[i]);
                sum_mr = sum_mr + (green(pixels[i])+red(pixels[i])+blue(pixels[i]))*i;
            }
        }
        // only use the data if there were a few pixels identified, otherwise you might get a divide by 0 error
        if(sum_m>5){
            center = sum_mr / sum_m;
        }
        else{
            center = 0;
        }
        center_pre = center;
        return center;
    }



    // the important function
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        // every time there is a new Camera preview frame
        setMyControlListener();
        mTextureView.getBitmap(bmp);

        final Canvas c = mSurfaceHolder.lockCanvas();
        if (c != null) {
            int[] pixels = new int[bmp.getWidth()]; // pixels[] is the RGBA data
            int count = 0;
            int checkdraw = 0;
            float sumCOM = 0;
            float aveCOM;
            int startY = 240; // which row in the bitmap to analyze to read
            float COM1;
            float COM2;
            float COM3;
            float COM4;
            float COM5;
            float COM6;
            int y1 = 100;
            int y2 = y1+2;
            int y3 = y1+4;
            int y4 = y1+6;
            int y5 = y1+8;
            int y6 = y1+10;


/*read rows*/
//            for ( int row = 220; row < 260; row=row+4){
//                bmp.getPixels(pixels, 0, bmp.getWidth(), 0, row, bmp.getWidth(), 1);
//
//                // in the row, see if there is more green than red
////                for (int i = 0; i < bmp.getWidth(); i++) {
////                    if ((green(pixels[i]) - red(pixels[i])) > thresh) {
////                        pixels[i] = rgb(0, 255, 0); // over write the pixel with pure green
////                    }
////                }
//
//                // in the row, see if the pix is gray
//                int sum_mr = 0; // the sum of the mass times the radius
//                int sum_m = 0; // the sum of the masses
//                for (int i = 0; i < bmp.getWidth(); i++) {
//                    if (checkGray(pixels[i])) {
//                        pixels[i] = rgb(1, 1, 1); // set the pixel to almost 100% black
//
//                        sum_m = sum_m + green(pixels[i])+red(pixels[i])+blue(pixels[i]);
//                        sum_mr = sum_mr + (green(pixels[i])+red(pixels[i])+blue(pixels[i]))*i;
//                    }
//                }
//                // only use the data if there were a few pixels identified, otherwise you might get a divide by 0 error
//                if(sum_m>5){
//                    COM = sum_mr / sum_m;
//                }
//                else{
//                    COM = 0;
//                }
//                sumCOM = sumCOM+COM;
//                checkdraw++;
////                if(checkdraw == ){
//                    canvas.drawCircle(COM,row,3,paint1);
////                    checkdraw = 0;
////                }
//                count++;
//                // update the row
//                bmp.setPixels(pixels, 0, bmp.getWidth(), 0, row, bmp.getWidth(), 1);
//
//            }
            //            aveCOM = sumCOM/count;
/*stop here*/
/**read one row   worked*/
//            bmp.getPixels(pixels, 0, bmp.getWidth(), 0, startY, bmp.getWidth(), 1);
//
//            // in the row, see if there is more green than red
////                for (int i = 0; i < bmp.getWidth(); i++) {
////                    if ((green(pixels[i]) - red(pixels[i])) > thresh) {
////                        pixels[i] = rgb(0, 255, 0); // over write the pixel with pure green
////                    }
////                }
//
//            // in the row, see if the pix is gray
//            int sum_mr = 0; // the sum of the mass times the radius
//            int sum_m = 0; // the sum of the masses
//            for (int i = 0; i < bmp.getWidth(); i++) {
//                if (checkGray(pixels[i])) {
//                    pixels[i] = rgb(1, 1, 1); // set the pixel to almost 100% black
//
//                    sum_m = sum_m + green(pixels[i])+red(pixels[i])+blue(pixels[i]);
//                    sum_mr = sum_mr + (green(pixels[i])+red(pixels[i])+blue(pixels[i]))*i;
//                }
//            }
//            // only use the data if there were a few pixels identified, otherwise you might get a divide by 0 error
//            if(sum_m>5){
//                COM = sum_mr / sum_m;
//            }
//            else{
//                COM = 0;
//            }
//            //send calculate pos to pic
//            String sendString = String.valueOf(0)+' '+String.valueOf((int)COM) + '\n';
//            try {
//                sPort.write(sendString.getBytes(), 10); // 10 is the timeout
//            } catch (IOException e) { }
//            canvas.drawCircle(COM,startY,5,paint1);
//            aveCOM = COM;
//
//            // update the row
//            bmp.setPixels(pixels, 0, bmp.getWidth(), 0, startY, bmp.getWidth(), 1);
/*stop here*/

/*get mass center in different rows*/
//            COM1 = read_masscenter(y1);
//            COM2 = read_masscenter(y2);
//            COM3 = read_masscenter(y3);
//            COM4 = read_masscenter(y4);
//            COM5 = read_masscenter(y5);
//            COM6 = read_masscenter(y6);
//            aveCOM = (COM1+COM2+COM3+COM4+COM5+COM6)/6;
//            //send data
//            String sendString = String.valueOf(0)+' '+String.valueOf((int)aveCOM) + '\n';
//            try {
//                sPort.write(sendString.getBytes(), 10); // 10 is the timeout
//            } catch (IOException e) { }
//
//            canvas.drawCircle(COM1,y1,5,paint1);
//            canvas.drawCircle(COM2,y2,5,paint1);
//            canvas.drawCircle(COM3,y3,5,paint1);
//            canvas.drawCircle(COM4,y4,5,paint1);
//            canvas.drawCircle(COM5,y5,5,paint1);
//            canvas.drawCircle(COM6,y6,5,paint1);
/*stop here*/
/*test1 nearlly worked*/
            sumCOM = 0;
            count = 0;
            for(int row = 240;row<=250;row=row+5){
                   float COMtemp = read_masscenter(row);
                if(COMtemp!=0){
                    sumCOM = sumCOM+COMtemp;
                    count++;
                }
                canvas.drawCircle(COMtemp,row,5,paint1);
            }
            if(count == 0){
                aveCOM = 0;
            }else{
                aveCOM = sumCOM/count;
            }

            if(aveCOM != 0){
                String sendString = String.valueOf(0)+' '+String.valueOf((int)aveCOM) + '\n';
                try {
                    sPort.write(sendString.getBytes(), 10); // 10 is the timeout
                } catch (IOException e) { }
            }

/*test1 stop*/

/*test2*/
//            sumCOM = 0;
//            count = 0;
//            for (int row = 50;row<=60;row=row+5){
//                bmp.getPixels(pixels, 0, bmp.getWidth(), 0, row, bmp.getWidth(), 1);
//                int sum_mr = 0;
//                int sum_m = 0;
//                for (int j = 0; j < bmp.getWidth(); j++) {
//                    if (-437.5317 + 1.67219 * (float) red(pixels[j]) + 0.083888 * (float) green(pixels[j]) + 0.83757 * (float) blue(pixels[j]) > 0) {
//                        pixels[j] = rgb(1, 1, 1);
//                        sum_m = sum_m + 1;
//                        sum_mr = sum_mr + j;
//                        }
//                    }
//                    if (sum_m > 5) {
//                        COM = sum_mr / sum_m;
//                    }
//                sumCOM = sumCOM + COM;
//                canvas.drawCircle(COM, row, 5, paint1); // x position, y position, diameter, color
//                bmp.setPixels(pixels, 0, bmp.getWidth(), 0, row, bmp.getWidth(), 1);
//                count++;
//            }
//            aveCOM = sumCOM/count;
//            //send data
//            String sendString = String.valueOf(0)+' '+String.valueOf((int)aveCOM) + '\n';
//            try {
//                sPort.write(sendString.getBytes(), 10); // 10 is the timeout
//            } catch (IOException e) { }
//
//            canvas.drawCircle(aveCOM, 240, 5, paint1);


/*test2 stop*/

            //send calculate pos to pic
            //show middle point, Thresh, Range
            canvas.drawText("pos: " +aveCOM, 10, 100, paint1); //show the position of middle point
            canvas.drawText("Thresh: " +Thresh, 10, 200, paint1);
            canvas.drawText("Range:"+Range,10,300,paint1);

        }



        c.drawBitmap(bmp, 0, 0, null);
        mSurfaceHolder.unlockCanvasAndPost(c);

        // calculate the FPS to see how fast the code is running
        long nowtime = System.currentTimeMillis();
        long diff = nowtime - prevtime;
        mTextView.setText("FPS " + 1000 / diff);
        prevtime = nowtime;
    }
}


