package frc.spectrumLib.util;

import java.net.*;

/** Common Network Utilties */
public class Network {

    /**
     * Gets the MAC address of the robot
     *
     * @return the MAC address of the robot
     */
    public static String getMACaddress() {
        InetAddress localHost;
        NetworkInterface ni;
        byte[] hardwareAddress;
        String MAC = "";
        int i = 0;
        while (i < 10) {
            try {
                localHost = InetAddress.getLocalHost();
                ni = NetworkInterface.getByInetAddress(localHost);
                hardwareAddress = ni.getHardwareAddress();
                String[] hexadecimal = new String[hardwareAddress.length];
                for (int j = 0; j < hardwareAddress.length; j++) {
                    hexadecimal[j] = String.format("%02X", hardwareAddress[j]);
                }
                MAC = String.join(":", hexadecimal);
                i++;
                return MAC;
            } catch (UnknownHostException | SocketException | NullPointerException e) {
            }
        }
        return "UNKNOWN";
    }

    /**
     * Gets the IP address of the robot
     *
     * @return the IP address of the robot
     */
    public static String getIPaddress() {
        InetAddress localHost;
        String IP = "";
        int i = 0;
        while (i < 10) {
            try {
                localHost = InetAddress.getLocalHost();
                IP = localHost.getHostAddress();
                i++;
                return IP;
            } catch (UnknownHostException e) {
            }
        }
        return "UNKNOWN";
    }

    /**
     * Gets the IP Address of the device at the address such as "limelight.local"
     *
     * @return the IP Address of the device
     */
    public static String getIPaddress(String deviceNameAddress) {
        InetAddress localHost;
        String IP = "";
        int i = 0;
        while (i < 10) {
            try {
                localHost = InetAddress.getByName(deviceNameAddress);
                IP = localHost.getHostAddress();
                i++;
                return IP;
            } catch (UnknownHostException e) {
            }
        }
        return "UNKNOWN";
    }
}
