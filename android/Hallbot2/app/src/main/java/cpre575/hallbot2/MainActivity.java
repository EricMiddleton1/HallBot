package cpre575.hallbot2;

import android.Manifest;
import android.content.Intent;
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.view.Gravity;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import org.opencv.android.OpenCVLoader;

public class MainActivity extends AppCompatActivity {

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
        System.loadLibrary("opencv_java3");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Camera permission
        ActivityCompat.requestPermissions(MainActivity.this,
                new String[]{Manifest.permission.CAMERA},
                1);

        // Example of a call to a native method
        TextView textView = (TextView) findViewById(R.id.startup_text);
        textView.setText(stringFromJNI());

        if (!OpenCVLoader.initDebug()) {
            textView.setText(textView.getText() + "\nOpenCVLoader.initDebug(), not working");
        } else {
            textView.setText(textView.getText() + "\nOpenCVLoader.initDebug(), working!!!!");
            textView.setText(textView.getText() + "\n" + validate(0L,0L));

        }
        textView.setAllCaps(true);
        textView.setPadding(10,100,10,100);
        textView.setGravity(Gravity.CENTER_HORIZONTAL); // move text to center


        Button b = (Button) findViewById(R.id.opencv_test_0);
        b.setText("OpenCV Canny Edge");
        Button b2 = (Button) findViewById(R.id.opencv_hough);
        b2.setText("OpenCV Vanishing Point");

    }

    public void startOpenCVTest0(View view) {
        Intent intent = new Intent(this, OpenCVTest0.class);
        startActivity(intent);
    }

    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     */
    public native String stringFromJNI();

    public native String validate(long matAddrGr, long matAddrRgba);
}
