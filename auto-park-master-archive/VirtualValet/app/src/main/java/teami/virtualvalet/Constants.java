package teami.virtualvalet;

import java.util.UUID;

/**
 * Created by dorothy.kirlew on 11/9/2015.
 */
public interface Constants
{
    // Message types sent from the BluetoothChatService Handler
    public static final int MESSAGE_STATE_CHANGE = 1;
    public static final int MESSAGE_READ = 2;
    public static final int MESSAGE_WRITE = 3;
    public static final int MESSAGE_DEVICE_NAME = 4;
    public static final int MESSAGE_TOAST = 5;

    // Unique UUID for this application
    public static final UUID MY_UUID_SECURE = UUID.fromString("94f39d29-7d6d-437d-973b-fba39e49d4ee");
    public static final UUID MY_UUID_INSECURE = UUID.fromString("94f39d29-7d6d-437d-973b-fba39e49d4ee");


    // Key names received from the BluetoothChatService Handler
    public static final String DEVICE_NAME = "ubuntu-0";
    public static final String TOAST = "toast";
}
