package teami.virtualvalet;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.support.v4.app.Fragment;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;
import android.app.Activity;


import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Set;
import java.util.UUID;

public class MainActivity extends AppCompatActivity {
    String message = "";
    TextView bluetoothStatus;
    TextView messageToUser;
    ConnectedThread mConnectedThread;

    BluetoothAdapter mBluetoothAdapter;
    BluetoothDevice mBluetoothDevice;
    ConnectThread mConnectThread;
    private BluetoothChatService mChatService = null;

    Handler mHandler = new Handler();
    /*Handler mHandler = new Handler()
    {
        @Override
        public void handleMessage(Message msg)
        {
            byte[] writeBuf = (byte[]) msg.obj;
            int begin = (int)msg.arg1;
            int end = (int)msg.arg2;

            switch(msg.what)
            {
                case 1:
                    String writeMessage = new String(writeBuf);
                    writeMessage = writeMessage.substring(begin, end);
                    break;
            }
        }
    };*/

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mChatService = new BluetoothChatService(this, mHandler);

        messageToUser = (TextView) findViewById(R.id.messageTextView);
        bluetoothStatus = (TextView) findViewById(R.id.bluetoothStatusText);


        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        if(mBluetoothAdapter == null)
        {
            bluetoothStatus.setText("Bluetooth not supported");
        }
        else
        {
            bluetoothStatus.setText("Bluetooth supported");
        }

        if(!mBluetoothAdapter.isEnabled())
        {
            Intent enableBluetooth = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableBluetooth, 1);

            Toast.makeText(this, "starting activity", Toast.LENGTH_SHORT).show();
        }
        Set <BluetoothDevice> pairedDevices = mBluetoothAdapter.getBondedDevices();
        if(pairedDevices.size() > 0)
        {
            for(BluetoothDevice device : pairedDevices)
            {
                if (device.getName().equals("ubuntu-0"))
                {
                    mBluetoothDevice = device;

                    Toast.makeText(this, "device is found", Toast.LENGTH_SHORT).show();
                }
            }
        }

        mChatService.start();

        Toast.makeText(this, "start mChatService", Toast.LENGTH_SHORT).show();
        mChatService.connect(mBluetoothDevice, false);
        Toast.makeText(this, "connect mChatService", Toast.LENGTH_SHORT).show();

    }

    public void parkCar(View view)
    {

        Toast.makeText(this, "parking car brah", Toast.LENGTH_SHORT).show();

        TextView status = (TextView) findViewById(R.id.statusText);
        TextView ETA = (TextView) findViewById(R.id.ETText);
        TextView ETALabel = (TextView) findViewById(R.id.justETALabel);
        Button parkButton = (Button) findViewById(R.id.parkButton);
        Button returnButton = (Button) findViewById(R.id.returnButton);

        if (messageToUser.getText().equals("No message sent")|| messageToUser.getText().equals("Car is returning") || messageToUser.getText().equals("It's still returning, calm down"))
        {

            messageToUser.setText("Car is parking");
            Toast.makeText(this, "should have changed the message", Toast.LENGTH_SHORT).show();

            status.setText("Parking");
            ETA.setText("May 2016");
            ETALabel.setText("ETP");
            returnButton.setEnabled(true);
            message = "Park";

            mChatService.write(message.getBytes());
        }
        else if (messageToUser.getText().equals("Car is parking"))
        {
            messageToUser.setText("It's still parking, calm down");
        }
        else if (messageToUser.getText().equals("It's still parking, calm down"))
        {
            parkButton.setEnabled(false);
        }
    }

    public void returnCar(View view) throws IOException {
        TextView status = (TextView) findViewById(R.id.statusText);
        TextView ETA = (TextView) findViewById(R.id.ETText);
        TextView ETALabel = (TextView) findViewById(R.id.justETALabel);
        Button parkButton = (Button) findViewById(R.id.parkButton);
        Button returnButton = (Button) findViewById(R.id.returnButton);

        if (messageToUser.getText().equals("No message sent")|| messageToUser.getText().equals("Car is parking") || messageToUser.getText().equals("It's still parking, calm down"))
        {
            messageToUser.setText("Car is returning");
            status.setText("Returning");
            ETA.setText("One Month");
            ETALabel.setText("ETA");
            parkButton.setEnabled(true);
            message = "Return";
            mChatService.write(message.getBytes());
        }
        else if (messageToUser.getText().equals("Car is returning"))
        {
            messageToUser.setText("It's still returning, calm down");
        }
        else if (messageToUser.getText().equals("It's still returning, calm down"))
        {
            returnButton.setEnabled(false);
        }
    }



    private class ConnectThread extends Thread
    {
        private final BluetoothSocket mSocket;
        private final BluetoothDevice mDevice;
        private final UUID mUUID = UUID.fromString("94f39d29-7d6d-437d-973b-fba39e49d4ee");


        public ConnectThread(BluetoothDevice device)
        {
            BluetoothSocket temp = null;
            mDevice = device;
            try
            {
                temp = device.createRfcommSocketToServiceRecord(mUUID);
                System.out.println("zzzYeah, that socket totes work");
            }
            catch (IOException e)
            {
                System.out.println("zzzYeah, that socket didn't work");
            }
            mSocket = temp;
        }

        public void run()
        {
            mBluetoothAdapter.cancelDiscovery();
            try
            {
                mSocket.connect();
            }
            catch (IOException e)
            {
                try
                {
                    mSocket.close();

                }
                catch (IOException closeException)
                {

                }
                return;
            }

            mConnectedThread = new ConnectedThread(mSocket);
            mConnectedThread.start();
            mConnectedThread.run();
        }

        public void cancel()
        {
            try
            {
                mSocket.close();
            }
            catch (IOException e)
            {

            }
        }

        public void write(String msg)
        {
            System.out.println("zzzgot to first write");
            mConnectedThread.write(msg);
        }
    }

    private class ConnectedThread extends Thread
    {
        private final BluetoothSocket mSocket;
        private final InputStream mInStream;
        private final OutputStream mOutStream;

        public ConnectedThread(BluetoothSocket socket)
        {
            mSocket = socket;
            InputStream tmpIn = null;
            OutputStream tmpOut = null;

            try
            {
                tmpIn = socket.getInputStream();
                tmpOut = socket.getOutputStream();
            }
            catch (IOException e)
            {
                System.out.println("No, no streams working");
            }

            mInStream = tmpIn;
            mOutStream = tmpOut;
        }

        public void run()
        {
            byte[] buffer = new byte[1024];
            int begin = 0;
            int bytes = 0;

            while(true)
            {
                try
                {
                    bytes += mInStream.read(buffer, bytes, buffer.length - bytes);
                    for(int i = begin; i < bytes; i++)
                    {
                        if(buffer[1] == "#".getBytes()[0])
                        {
                            mHandler.obtainMessage(1, begin, i, buffer).sendToTarget();
                            begin = i + 1;
                            if(i == bytes - 1)
                            {
                                bytes = 0;
                                begin = 0;
                            }
                        }
                    }
                }
                catch (IOException e)
                {
                    break;
                }
            }
        }

        public void write(String msg)
        {
            byte[] bytes = msg.getBytes();
            System.out.println("zzzTrying to write");

            mChatService.write(bytes);

        }

        public void cancel()
        {
            try
            {
                mSocket.close();
            }
            catch (IOException e)
            {

            }
        }
    }



    /*protected void onActivityResult(int requestCode, int resultCode, Intent data)
    {


        UUID uuid = UUID.fromString("94f39d29-7d6d-437d-973b-fba39e49d4ee"); //Standard SerialPortService ID
        try
        {
            mSocket = mBluetoothDevice.createRfcommSocketToServiceRecord(uuid);

            mSocket.connect();
            mOutputStream = mSocket.getOutputStream();
            mInputStream = mSocket.getInputStream();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    protected void sendMessage(String msg)
    {
        try
        {
            mOutputStream.write(msg.getBytes());
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }*/


    /*void openBT()
    {
        try
        {
            UUID uuid = UUID.fromString("00001101-0000-1000-8000-00805f9b34fb");
            mSocket = mDevice.createRfcommSocketToServiceRecord(uuid);
            mSocket.connect();
            mOutputStream = mSocket.getOutputStream();
            mInputStream = mSocket.getInputStream();

            beginListForData();

            bluetoothStatus.setText("Bluetooth Opened");
        }
        catch (NullPointerException e)
        {
            e.printStackTrace();
        }
        catch(Exception e)
        {
            e.printStackTrace();
        }
    }*/



    /*void beginListForData()
    {
        //todo later
    }*/

    /*void sendData(String msg)
    {
        try
        {

            Toast.makeText(this, "I'm trying", Toast.LENGTH_SHORT).show();
            mOutputStream.write(msg.getBytes());

            bluetoothStatus.setText("Sent message");
        }
        catch (NullPointerException e)
        {

            Toast.makeText(this, "null pointer exception", Toast.LENGTH_SHORT).show();
            e.printStackTrace();
        }
        catch(Exception e)
        {
            Toast.makeText(this, "other exception", Toast.LENGTH_SHORT).show();
            e.printStackTrace();
        }
    }*/

    /*void closeBT()
    {
        try
        {
            stopWorker = true;
            mOutputStream.close();
            mInputStream.close();
            mSocket.close();
            bluetoothStatus.setText("Bluetooth Closed");
        }
        catch (NullPointerException e)
        {
            e.printStackTrace();
        }
        catch(Exception e)
        {
            e.printStackTrace();
        }
    }*/

    /*public void enableBluetooth()
    {
        Intent discoveryIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE);

        //discoveryIntent.putExtra(BluetoothAdapter.EXTRA_DISCOVERABLE_DURATION, 300);

        startActivityForResult(discoveryIntent, 1);



    }*/

    /*protected void onActivityResult(int requestCode, int resultCode, Intent data)
    {

        try
        {
            final BluetoothServerSocket mServer = mBluetoothAdapter
        }

        if(resultCode == 300 && requestCode == 1)
        {
            Intent intent = new Intent();
            intent.setAction(Intent.ACTION_SEND);
            intent.setType("text/plain");
            try {

                File root = new File(Environment.getExternalStorageDirectory(), "Notes");
                if (!root.exists()) {
                    root.mkdir();
                }
                File gpxFile = new File(root, "Message.txt");

                FileWriter writer = new FileWriter(gpxFile);
                writer.append(message);
                writer.flush();
                writer.close();

                intent.putExtra(Intent.EXTRA_STREAM, Uri.fromFile(gpxFile));

                Toast.makeText(this, "Sent!", Toast.LENGTH_SHORT).show();
            }
            catch (IOException e)
            {
                e.printStackTrace();
            }

            PackageManager pm = getPackageManager();
            List <ResolveInfo> appsList = pm.queryIntentActivities( intent, 0);
            if(appsList.size() > 0)
            {
                String packageName = null;
                String className = null;
                boolean found = false;

                for(ResolveInfo info: appsList)
                {
                    packageName = info.activityInfo.packageName;
                    if(packageName.equals("com.android.bluetooth"))
                    {
                        className = info.activityInfo.name;
                        found = true;
                        break;
                    }
                }

                if(!found)
                {
                    TextView bluetoothStatus = (TextView) findViewById(R.id.bluetoothStatusText);
                    bluetoothStatus.setText("Bluetooth not found!!");
                }
                else
                {
                    intent.setClassName(packageName, className);
                    startActivity(intent);
                }
            }

            else
            {
                TextView bluetoothStatus = (TextView) findViewById(R.id.bluetoothStatusText);
                bluetoothStatus.setText("Bluetooth is cancelled.");
            }
        }
    }*/
}