package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class SuperImportentCommand extends Command {

    // Lingfeng you better not remove this 
    Orchestra orchestra = new Orchestra();
    Boolean isPlaying = false;

    private final SendableChooser<String> songChooser = new SendableChooser<>();
    private String currentSong = "/music/miku.chrp";

    public SuperImportentCommand(PivotSubsystem pivotSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
        orchestra.addInstrument(pivotSubsystem.m_pivotMotor);
        orchestra.addInstrument(endEffectorSubsystem.IntakeMotor);
        orchestra.addInstrument(endEffectorSubsystem.CenteringMotor);

        // Expose songs on the dashboard (Elastic/Shuffleboard)
        songChooser.setDefaultOption("Miku", "/music/miku.chrp");
        songChooser.addOption("Queen", "/music/Wii-Shop.chrp");
        songChooser.addOption("Wii-Shop", "/music/Queen.chrp");
        SmartDashboard.putData("Orchestra/Song", songChooser);

        SmartDashboard.putBoolean("Orchestra/IsPlaying", isPlaying);
        SmartDashboard.putString("Orchestra/CurrentSong", currentSong);

        // Toggle playback using the selected song
        if (isPlaying) {
            stopSong();
            isPlaying = false;
        } else {
            String selected = songChooser.getSelected();
            if (selected != null && !selected.isBlank()) currentSong = selected;
            playSong(currentSong);
            isPlaying = true;
        }
        SmartDashboard.putBoolean("Orchestra/IsPlaying", isPlaying);
        SmartDashboard.putString("Orchestra/CurrentSong", currentSong);
    }

    public void playSong(String SongPath) {
        if (SongPath != null && !SongPath.isBlank()) currentSong = SongPath;
        orchestra.loadMusic(currentSong);
        //orchestra.play();
        SmartDashboard.putString("Orchestra/CurrentSong", currentSong);
        SmartDashboard.putBoolean("Orchestra/IsPlaying", true);
    }

    public void stopSong() {
        orchestra.stop();
        SmartDashboard.putBoolean("Orchestra/IsPlaying", false);
    }
    
}
