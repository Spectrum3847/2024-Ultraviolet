package frc.spectrumLib.util.exceptions;

public class KillRobotException extends RuntimeException {

    /*
     * Required when we want to add a custom message when throwing the exception
     * as throw new CustomUncheckedException(" Custom Unchecked Exception ");
     */
    public KillRobotException(String message) {
        // calling super invokes the constructors of all super classes
        // which helps to create the complete stacktrace.
        super(message);
    }

    /*
     * Required when we want to wrap the exception generated inside the catch block and rethrow it
     * as catch(ArrayIndexOutOfBoundsException e) {
     * throw new CustomUncheckedException(e);
     * }
     */
    public KillRobotException(Throwable cause) {
        // call appropriate parent constructor
        super(cause);
    }
    /*
     * Required when we want both the above
     * as catch(ArrayIndexOutOfBoundsException e) {
     * throw new CustomUncheckedException(e, "File not found");
     * }
     */
    public KillRobotException(String message, Throwable throwable) {
        // call appropriate parent constructor
        super(message, throwable);
    }
}
