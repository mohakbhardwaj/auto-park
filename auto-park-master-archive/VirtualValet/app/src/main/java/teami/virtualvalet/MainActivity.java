package teami.virtualvalet;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;
import java.util.Set;

public class MainActivity extends AppCompatActivity
{
    String message = "";
    TextView bluetoothStatus;
    TextView messageToUser;

    BluetoothAdapter mBluetoothAdapter;
    BluetoothDevice mBluetoothDevice;
    private BluetoothChatService mChatService = null;

    Handler mHandler = new Handler();

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        messageToUser = (TextView) findViewById(R.id.messageTextView);
        bluetoothStatus = (TextView) findViewById(R.id.bluetoothStatusText);

        // create chat service and adapter
        mChatService = new BluetoothChatService(this, mHandler);
        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        // update text on display to indicate if bluetooth is available
        if(mBluetoothAdapter == null)
        {
            bluetoothStatus.setText("Bluetooth not supported");
        }
        else
        {
            bluetoothStatus.setText("Bluetooth supported");
        }

        // enable bluetooth adapter
        if(!mBluetoothAdapter.isEnabled())
        {
            Intent enableBluetooth = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableBluetooth, 1);
        }

        // Find and save device that matches DEVICE_NAME, saved in Constants.Java
        // This is currently the name of the laptop, will be the name of the mobile platform
        // Todo: implement way for user to set device_name so that user can decide what to connect to
        Set <BluetoothDevice> pairedDevices = mBluetoothAdapter.getBondedDevices();
        if(pairedDevices.size() > 0)
        {
            for(BluetoothDevice device : pairedDevices)
            {
                if (device.getName().equals(Constants.DEVICE_NAME))
                {
                    mBluetoothDevice = device;
                }
            }
        }
        // start chat service and connect to bluetooth
        mChatService.start();
        Toast.makeText(this, "start mChatService", Toast.LENGTH_SHORT).show();
        mChatService.connect(mBluetoothDevice, false);
        Toast.makeText(this, "connect mChatService", Toast.LENGTH_SHORT).show();
    }

    public void parkCar(View view)
    {
        TextView status = (TextView) findViewById(R.id.statusText);
        TextView ETA = (TextView) findViewById(R.id.ETText);
        TextView ETALabel = (TextView) findViewById(R.id.justETALabel);
        Button parkButton = (Button) findViewById(R.id.parkButton);
        Button returnButton = (Button) findViewById(R.id.returnButton);

        // update message to user, debugging only
        // todo: remove this once app is functional
        messageToUser.setText("Car is parking");

        // update status and ETA.  enable Return and disable Park buttons
        status.setText("Parking");
        ETA.setText("May 2016");
        ETALabel.setText("ETP");
        returnButton.setEnabled(true);
        parkButton.setEnabled(false);
        message = "Park";

        // send park message to chat service
        mChatService.write(message.getBytes());
    }

    public void returnCar(View view)
    {
        TextView status = (TextView) findViewById(R.id.statusText);
        TextView ETA = (TextView) findViewById(R.id.ETText);
        TextView ETALabel = (TextView) findViewById(R.id.justETALabel);
        Button parkButton = (Button) findViewById(R.id.parkButton);
        Button returnButton = (Button) findViewById(R.id.returnButton);

        // update message to user, debugging only
        // todo: remove this once app is functional
        messageToUser.setText("Car is returning");

        // update status and ETA.  enable Park and disable Return buttons
        status.setText("Returning");
        ETA.setText("One Month");
        ETALabel.setText("ETA");
        parkButton.setEnabled(true);
        returnButton.setEnabled(false);
        message = "Return";

        // send return message to chat service
        mChatService.write(message.getBytes());
    }
}