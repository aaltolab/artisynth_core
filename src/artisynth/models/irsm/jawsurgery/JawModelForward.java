package artisynth.models.irsm.jawsurgery;

import java.awt.Color;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;

import javax.swing.JSeparator;

import artisynth.core.driver.Main;
import artisynth.core.gui.ControlPanel;
import artisynth.core.inverse.TrackingController;
// artisynth mechmodels imports
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MotionTargetComponent;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.Muscle;
import maspack.matrix.RigidTransform3d;

import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.util.ArtisynthIO;

// artisynth utilites
import artisynth.core.util.ArtisynthPath;
// artisynth workspace imports
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;

// artisynth maspack imports
import maspack.matrix.Point3d;
import maspack.properties.Property;
import maspack.render.RenderProps;

public class JawModelForward extends RootModel {
  JawModelBaseline myJawModel;
  String probesFilename;
  String workingDirname;
  TrackingController myTrackingController;
  String excitersFile = "muscleList.txt";
  ArrayList<String> MuscleAbbreviation = new ArrayList<String> ();
  public static boolean debug = false;
  boolean addIncisorTrace = false;
  double dt = 0.001;
  double t = 0.75;


   // Condyle const
   boolean useCondyleCapsule = false;
   boolean usePlaneJoint = false;
   boolean useRevoluteJoint = false;
   boolean useCondyleTargets = false;
   boolean useForceEffectorCons = true;

  @Override
  public void build (String[] args) throws IOException {
    super.build (args);
    setWorkingDir();

    myJawModel = new JawModelBaseline ("jawmodel",usePlaneJoint, useRevoluteJoint, useCondyleCapsule, useCondyleTargets, useForceEffectorCons);
    addModel (myJawModel);

    removeAllInputProbes ();
    removeAllOutputProbes ();  

    if (usePlaneJoint || useForceEffectorCons) {
      myJawModel.setRLtrlWallAngle(0);
      myJawModel.setLLtrlWallAngle(0);
   }

  }

  public void attach (DriverInterface driver) {
    workingDirname = "data";
    probesFilename = "excitationInputProbes.art";
    loadProbes ();

    super.attach (driver);

    ControlPanel simulationPanel = new ControlPanel ("Simulation Controls");

    addSimulationControls (myJawModel, simulationPanel);
    addControlPanel (simulationPanel);
    addMuscleProbe (t, dt, "excitation", "Excitations");
    addIncisorProbe (t, dt, "position");
    addIncisorProbe (t, dt, "velocity");

    RigidTransform3d T = new RigidTransform3d (0, 0, -0.001);
    myJawModel.rigidBodies ().get ("jaw").transformGeometry (T);

    setIncisorVisible ();

  //   if (useRevoluteJoint == false) {
  //     myJawModel.bodyConnectors().get("LTMJ").setEnabled(true);
  //     myJawModel.bodyConnectors().get("RTMJ").setEnabled(true);
  //     myJawModel.bodyConnectors().get("LLTRL").setEnabled(true);
  //     myJawModel.bodyConnectors().get("RLTRL").setEnabled(true);
  //     myJawModel.bodyConnectors().get("LPOST").setEnabled(true);
  //     myJawModel.bodyConnectors().get("RPOST").setEnabled(true);
  //     myJawModel.bodyConnectors().get("LBITE").setEnabled(true);
  //     myJawModel.bodyConnectors().get("RBITE").setEnabled(true);
  //     myJawModel.bodyConnectors().get("LMED").setEnabled(false);
  //     myJawModel.bodyConnectors().get("RMED").setEnabled(false);
  //  }
    
    // addBreakPoint (t);

    Main.getMain().getViewer().setBackgroundColor (new Color(1f,1f,1f));

    Color myRed = new Color(255,102,102,255);
    myJawModel.bodyConnectors().get(0).getRenderProps().setFaceColor(myRed);
    myJawModel.bodyConnectors().get(0).getRenderProps().setAlpha(1);
    myJawModel.bodyConnectors().get(1).getRenderProps().setFaceColor(myRed);
    myJawModel.bodyConnectors().get(1).getRenderProps().setAlpha(1);

    if (myJawModel.showCons == false) {
      myJawModel.bodyConnectors().get(0).getRenderProps().setVisible(false);
      myJawModel.bodyConnectors().get(1).getRenderProps().setVisible(false);
   }

  }
  
  public void addTrackingController (double t) throws IOException {
    // set up tracking controller
    myTrackingController =
       new TrackingController (myJawModel, "myTrackingController");

    MotionTargetComponent target =
       myJawModel.frameMarkers ().get ("lowerincisor");
    myTrackingController.setNormalizeH (true);
    myTrackingController.addL2RegularizationTerm (0.0025);
    myTrackingController.addDampingTerm (1.0);
    myTrackingController.addMotionTarget (target, 1.0);
    myTrackingController.setProbeDuration (t);
    myTrackingController.setExcitationBounds (0, 1);
    myTrackingController.setMaxExcitationJump (0.0001);
    addController (myTrackingController);
 }

  public void loadProbes () {
    if (probesFilename == null || !myInputProbes.isEmpty ()
    || !myOutputProbes.isEmpty ())
      return;

    String osType = System.getProperty("os.name");
    ArtisynthPath.setWorkingDir(new File(ArtisynthPath.getWorkingDir().toPath() + (osType.equals("Mac OS X") ? "/" : "\\") + workingDirname));
    String probeFileFullPath =
      ArtisynthPath.getWorkingDir () + (osType.equals("Mac OS X") ? "/" : "\\") +  probesFilename;
    System.out.println(ArtisynthPath.getWorkingDir ());
    System.out.println(probeFileFullPath);

    try {
      scanProbes (ArtisynthIO.newReaderTokenizer (probeFileFullPath));
    }
    catch (Exception e) {
      System.out.println ("Error reading probe file");
      e.printStackTrace ();
    }
  }

  public void setWorkingDir () {
    // set default working directory to repository location
    File workingDir =
      new File (ArtisynthPath.getSrcRelativePath (JawModelForward.class, workingDirname));
    
    String path = ArtisynthPath.getHomeDir().replace("artisynth_core", "Patient Data");
    File patientDataDir = new File (path);

    if (patientDataDir.isDirectory()) {
      workingDir = patientDataDir;
      ArtisynthPath.setWorkingDir (workingDir);
    } else {
       ArtisynthPath.setWorkingDir (workingDir);
    }

    System.out.println(workingDir);
  }

  public void addSimulationControls (JawModel jaw, ControlPanel panel) {
    if (panel == null)
      return;
    panel.addWidget (new JSeparator ());
    panel.addWidget ("Integration Method", jaw, "integrator");
    panel.addWidget ("Maximum Step Size", jaw, "maxStepSize");

    addBodyDampingControls (
      jaw, "jaw", panel, new double[] { 0.0, 20000.0 },
      new double[] { 0.0, 40.0 });
    addBodyDampingControls (
      jaw, "hyoid", panel, new double[] { 0.0, 20000.0 },
      new double[] { 0.0, 40.0 });

  }

  public static void addBodyDampingControls (
    JawModel jaw, String bodyName, ControlPanel panel, double[] rotLimits,
    double[] transLimits) {
    if (panel == null
    || jaw.findComponent ("rigidBodies/" + bodyName) == null) {
      return;
    }
    panel.addWidget (new JSeparator ());

    panel
      .addWidget (
        bodyName.toUpperCase () + " Rot Damping", jaw,
        "rigidBodies/" + bodyName + ":rotaryDamping", rotLimits[0],
        rotLimits[1]);
    panel
      .addWidget (
        bodyName.toUpperCase () + " Trans Damping", jaw,
        "rigidBodies/" + bodyName + ":frameDamping", transLimits[0],
        transLimits[1]);
  }

  public void addMuscleProbe (
    double duration, double interval, String propName, String name) {
    ArrayList<Property> props = new ArrayList<Property> ();
    for (AxialSpring m : myJawModel.axialSprings ()) {
      if (m instanceof Muscle)
        if (((Muscle)m).isEnabled ()) {
          props.add (m.getProperty (propName));
        }
    }

    Property[] proparray = new Property[props.size ()];
    for (int i = 0; i < props.size (); i++) {
      proparray[i] = props.get (i);
    }

    NumericOutputProbe p = new NumericOutputProbe (proparray, interval);
    p.setStartStopTimes (0, duration);
    p.setName (name);
    p.setAttachedFileName (name + "_output.txt");
    p.setDefaultDisplayRange (-0.1, 0.1);
    addOutputProbe (p);
  }

  public void addIncisorProbe (
    double duration, double interval, String propName) {
    String name = "incisor_" + propName;
    NumericOutputProbe p =
      new NumericOutputProbe (
        new Property[] { myJawModel
          .frameMarkers ().get ("lowerincisor").getProperty (propName) },
        interval);
    p.setStartStopTimes (0.0, duration);
    p.setName (name);
    p.setAttachedFileName (name + "_output.txt");
    p.setDefaultDisplayRange (-0.1, 0.1);
    addOutputProbe (p);

  }

  public void setIncisorVisible () {
    FrameMarker inc = myJawModel.frameMarkers ().get ("lowerincisor");
    if (inc == null)
      return;
      
    RenderProps.setLineColor (inc, Color.RED);
    RenderProps.setPointColor (inc, Color.CYAN);
    Point3d loc = new Point3d ();
    inc.getLocation (loc);
    inc.setLocation (loc);
    enableTracing (inc);
    addIncisorTrace = true;

  }
}
