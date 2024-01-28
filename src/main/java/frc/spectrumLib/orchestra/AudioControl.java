package frc.spectrumLib.orchestra;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.spectrumLib.mechanism.Mechanism;
import java.util.Arrays;
import java.util.Collection;
import java.util.stream.Collectors;
import java.util.stream.Stream;


//TODO: separate tracks and SFX, implement audio control in robot control & gamepads
public class AudioControl implements Subsystem {
    public Orchestra orchestra;
    public Collection<ParentDevice> instruments;

    private final String[] playlist = {
        "track1.chrp",
        "track2.chrp",
        "track3.chrp",
        "track4.chrp",
        "track5.chrp",
        "track6.chrp",
        "track7.chrp",
        "track8.chrp",
        "track9.chrp",
        "track10.chrp",
        "track11.chrp",
        "track12.chrp",
        "track13.chrp",
        "track14.chrp",
        "track15.chrp",
        "track16.chrp",
        "track17.chrp",
        "track18.chrp",
        "track19.chrp",
        "track20.chrp"
    };
    private int currentTrackNumber = 0;
    private String currentTrack = playlist[currentTrackNumber];

    public AudioControl() {
        Stream<frc.spectrumLib.swerve.Module> swerveModules =
                Arrays.stream(Robot.swerve.getModules()); // retrieve swerve modules
        instruments =
                Mechanism.getInstances().stream()
                        .map(Mechanism::getMotor)
                        .collect(Collectors.toList()); // add mechanism motors
        instruments.addAll(
                swerveModules
                        .flatMap(
                                module -> Stream.of(module.getDriveMotor(), module.getSteerMotor()))
                        .collect(Collectors.toList())); // add swerve drive + angle motors
        orchestra = new Orchestra(instruments);
        loadMusic(currentTrack); //queue up first song
    }

    /* Custom */
    public void skipTrack() {
        currentTrack = playlist[++currentTrackNumber];
    }

    /* Commands */
    public Command runPlay() {
        return runOnce(() -> play()).withName("AudioControl.play").ignoringDisable(true);
    }

    public Command runPause() {
        return runOnce(() -> pause()).withName("AudioControl.pause").ignoringDisable(true);
    }

    public Command runSkip() {
        return runOnce(() -> skipTrack()).withName("AudioControl.skip").ignoringDisable(true);
    }


    /* Wrapper */

    public StatusCode addInstrument(ParentDevice instrument) {
        return orchestra.addInstrument(instrument);
    }

    public StatusCode addInstrument(ParentDevice instrument, int trackNumber) {
        return orchestra.addInstrument(instrument, trackNumber);
    }

    public StatusCode clearInstruments() {
        return orchestra.clearInstruments();
    }

    public void close() {
        orchestra.close();
    }

    public double getCurrentTime() {
        return orchestra.getCurrentTime();
    }

    public boolean isPlaying() {
        return orchestra.isPlaying();
    }

    public StatusCode loadMusic(String filepath) {
        return orchestra.loadMusic("chirps/" + filepath);
    }

    public StatusCode pause() {
        return orchestra.pause();
    }

    public StatusCode play() {
        return orchestra.play();
    }

    public StatusCode stop() {
        return orchestra.stop();
    }
}
