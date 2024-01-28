package frc.spectrumLib.orchestra;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.wpilibj.DriverStation;
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
    public Orchestra mixer;
    public Orchestra sfxControl;
    public Collection<ParentDevice> trackInstruments;
    public Collection<ParentDevice> sfxInstruments;

    private boolean runWhileEnabled = false;
    private final String[] sfx = { "sfx1.chrp"};
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
        "track19.chrp"
    };
    private int currentTrackNumber = 0;
    private String currentTrack = playlist[currentTrackNumber];

    /**
     * 
     * @param runWhileEnabled whether tracks (not SFX) run in enabled
     */
    public AudioControl(boolean runWhileEnabled) {
        this.runWhileEnabled = runWhileEnabled;
        // retrieve swerve modules
        Stream<frc.spectrumLib.swerve.Module> swerveModules =
                Arrays.stream(Robot.swerve.getModules()); 

        // create sfx instruments
        sfxInstruments =
                Mechanism.getInstances().stream()
                        .map(Mechanism::getMotor)
                        .collect(Collectors.toList()); // only mechanism motors

        //create track instruments
        trackInstruments.addAll(sfxInstruments); // include mechanism motors in track instruments
        trackInstruments.addAll(
                swerveModules
                        .flatMap(
                                module -> Stream.of(module.getDriveMotor(), module.getSteerMotor()))
                        .collect(Collectors.toList())); // add swerve drive & angle motors

        //create orchestras
        sfxControl = new Orchestra(sfxInstruments);
        mixer = new Orchestra(trackInstruments);

        loadTrack(currentTrack); //queue up first song
        loadSFX(sfx[0]); //queue up sfx
    }

    /* Mixer Commands */
    public Command runMixerPlay() {
        return runOnce(() -> playTrack()).withName("AudioControl.play").ignoringDisable(true);
    }

    public Command runMixerPause() {
        return runOnce(() -> pause()).withName("AudioControl.pause").ignoringDisable(true);
    }

    public Command runMixerSkip() {
        return runOnce(() -> continueToNextTrack()).withName("AudioControl.skip").ignoringDisable(true);
    }

    /* SFX Commands */
    public Command runSFXPlay() {
        return runOnce(() -> playSFX()).withName("AudioControl.playSFX").ignoringDisable(true);
    }

    public Command runSFXPlay(String filepath) {
        return runOnce(() -> playSFX(filepath)).withName("AudioControl.playSFX").ignoringDisable(true);
    }

    public Command runSFXStop() {
        return runOnce(() -> stopSFX()).withName("AudioControl.stopSFX").ignoringDisable(true);
    }


    /* SFX Wrapper */

    public void playSFX() {
        playSFX(sfx[0]);
    }

    public void playSFX(String filepath) {
        loadSFX(filepath);
        sfxControl.play();
    }

    public StatusCode loadSFX(String filepath) {
        return sfxControl.loadMusic("chirps/sfx/" + filepath);
    }

    public StatusCode stopSFX() {
        return sfxControl.stop();
    }

    /* Mixer Custom */

    //stop current song and continue to next song in playlist
    public void continueToNextTrack() {
        stop();
        playTrack();
    }

    public void skipTrack() {
        currentTrack = playlist[++currentTrackNumber];
        loadTrack(currentTrack);
    }

    /* Mixer Wrapper */

    public StatusCode addMixerInstrument(ParentDevice instrument) {
        return mixer.addInstrument(instrument);
    }

    public StatusCode addMixerInstrument(ParentDevice instrument, int trackNumber) {
        return mixer.addInstrument(instrument, trackNumber);
    }

    public StatusCode clearMixerInstruments() {
        return mixer.clearInstruments();
    }

    public void closeMixer() {
        mixer.close();
    }

    public double getCurrentTrackTime() {
        return mixer.getCurrentTime();
    }

    public boolean isTrackPlaying() {
        return mixer.isPlaying();
    }

    public StatusCode loadTrack(String filepath) {
        return mixer.loadMusic("chirps/tracks/" + filepath);
    }

    public StatusCode pause() {
        return mixer.pause();
    }

    public void playTrack() {
        if(canRun()) {
            mixer.play();
        }
    }

    //stop current song and continue to next song in playlist
    public StatusCode stop() {
        StatusCode status = mixer.stop();
        skipTrack();
        return status;
    }


    /* Audio Control */
    public boolean canRun() {
        if(DriverStation.isEnabled()) {
            if(!runWhileEnabled) {
                return false;
            }
        }
        return true;
    }
}
