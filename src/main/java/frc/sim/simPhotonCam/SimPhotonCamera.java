package frc.sim.simPhotonCam;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

import org.photonvision.PhotonTrackedTarget;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import frc.UnitUtils;

public class SimPhotonCamera {
    
    private final NetworkTableEntry rawBytesEntry;
    private final NetworkTableEntry driverModeEntry;
    private final NetworkTableEntry inputSaveImgEntry;
    private final NetworkTableEntry outputSaveImgEntry;
    private final NetworkTableEntry pipelineIndexEntry;
    private final NetworkTableEntry ledModeEntry;
  
    private final NetworkTable mainTable = NetworkTableInstance.getDefault().getTable("photonvision");
  
    private boolean driverMode;
    private int pipelineIndex;
    private LEDMode mode;
  
    private Packet packet = new Packet(1);

    LinkedList<PhotonTrackedTarget> trackedTargetList = new LinkedList<PhotonTrackedTarget>();
  
    /**
     * Constructs a PhotonCamera from a root table.
     *
     * @param rootTable The root table that the camera is broadcasting information
     *                  over.
     */
    public SimPhotonCamera(NetworkTable rootTable) {
      rawBytesEntry = rootTable.getEntry("rawBytes");
      driverModeEntry = rootTable.getEntry("driverMode");
      inputSaveImgEntry = rootTable.getEntry("inputSaveImgCmd");
      outputSaveImgEntry = rootTable.getEntry("outputSaveImgCmd");
      pipelineIndexEntry = rootTable.getEntry("pipelineIndex");
      ledModeEntry = mainTable.getEntry("ledMode");
  
      driverMode = driverModeEntry.getBoolean(false);
      pipelineIndex = pipelineIndexEntry.getNumber(0).intValue();
      getLEDMode();
      trackedTargetList.clear();
    }
  
    /**
     * Constructs a PhotonCamera from the name of the camera.
     *
     * @param cameraName The nickname of the camera (found in the PhotonVision
     *                   UI).
     */
    public SimPhotonCamera(String cameraName) {
      this(NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(cameraName));
    }


    public void reportDetectedTarget(Transform2d cameraToTarget){

      double yaw = UnitUtils.wrapAngleDeg(cameraToTarget.getRotation().getDegrees());
      double pitch = 0; //TODO based on height, camera pitch, distance;
      double area = 1.0; //TODO based on distance and known target size;
      double skew = 0; //Todo - maybe? depends on the year
      var reportedTarget = new PhotonTrackedTarget(yaw, pitch, area, skew, cameraToTarget);
      trackedTargetList.add(reportedTarget);
    }

    public void clearAllTargets(){
      trackedTargetList.clear();
    }

    public void update(){
      //Simulate one run of a vision processing pipleine, putting one result to NT.
      
      PhotonPipelineResult newResult = new PhotonPipelineResult(0,trackedTargetList);
      var newPacket = new org.photonvision.Packet(newResult.getPacketSize());
      newResult.populatePacket(newPacket);
      rawBytesEntry.setRaw(newPacket.getData());
    }

  /**
   * Returns the current LED mode.
   *
   * @return The current LED mode.
   */
  public LEDMode getLEDMode() {
    int value = ledModeEntry.getNumber(-1).intValue();
    switch (value) {
      case 0:
        mode = LEDMode.kOff;
        break;
      case 1:
        mode = LEDMode.kOn;
        break;
      case 2:
        mode = LEDMode.kBlink;
        break;
      case -1:
      default:
        mode = LEDMode.kDefault;
        break;
    }
    return mode;
  }

    
}