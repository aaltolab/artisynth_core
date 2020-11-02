package artisynth.models.irsm.jawsurgery;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;

import javax.swing.JSeparator;

import org.omg.CORBA.FloatSeqHolder;
import org.python.antlr.ast.Num;

import artisynth.core.driver.Main;
import artisynth.core.gui.ControlPanel;
import artisynth.core.inverse.TrackingController;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.BodyConnector;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MotionTargetComponent;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.SegmentedPlanarConnector;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.modelbase.ComponentListView;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.properties.Property;
import maspack.render.RenderProps;

public class JawModelInversePlayground extends RootModel {
   JawModelBaseline myJawModel;
   TrackingController myTrackingController;
   ArrayList<String> MuscleAbbreviation = new ArrayList<String> ();
   String excitersFile = "muscleList.txt";
   protected String workingDirname = "data/";
   List<String> ForceTargetNames = Arrays.asList ("Brux_C"); // "CANINE" or
                                                             // "LBITE"
   // String incisorTargetpathFileName = "ref_targetPos_input.txt";
   String incisorTargetpathFileName = "lowerincisor_position.txt";
   String leftTmjTargetpathFileName = "ltmj_position.txt";
   String rightTmjTargetpathFileName = "rtmj_position.txt";
   double t = 0.5; 
   double timestep = 0.001;

   // tracking method
   Boolean condyleTargets = false;
   Boolean frameTarget = false;
   Boolean useIncisorTarget = true;

   // Condyle const
   boolean useCondyleCapsule = false;
   boolean usePlaneJoint = false;
   boolean useRevoluteJoint = false;
   boolean useCondyleTargets = false;
   boolean useForceEffectorCons = true;
   
   public JawModelInversePlayground () {
      // TODO Auto-generated constructor stub
   }

   public JawModelInversePlayground (String name) {
      super (null);
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      setWorkingDir ();

      myJawModel = new JawModelBaseline ("jawmodel",usePlaneJoint, useRevoluteJoint, useCondyleCapsule, useCondyleTargets, useForceEffectorCons);

      addModel (myJawModel);
      getRoot (this).setMaxStepSize (timestep);
      myJawModel.setIntegrator (Integrator.ConstrainedBackwardEuler);

      ControlPanel simulationPanel = new ControlPanel ("Simulation Controls");
      addSimulationControls (myJawModel, simulationPanel);
      addControlPanel (simulationPanel);

      // System.out.println ("Jaw pose: "+ myJawModel.rigidBodies ().get ("jaw").getPosition ().toString ());
      
      RigidTransform3d T = new RigidTransform3d (0, 0, -0.0001);
      myJawModel.rigidBodies ().get ("jaw").transformGeometry (T);

      // System.out.println ("Transformed");
      // System.out.println ("Jaw pose: " + myJawModel.rigidBodies ().get ("jaw").getPosition ().toString ());

      addTrackingController (t);

      // for (double i=0.001; i<=t; i=i+0.001 ){
      //    addWayPoint (i);
      // }

      if (usePlaneJoint || useForceEffectorCons) {
         myJawModel.setRLtrlWallAngle(0);
         myJawModel.setLLtrlWallAngle(0);
      }
      
      addBreakPoint (t);
      
      addMuscleProbe (t, 0.001, "forceNorm", "Muscle Forces");
      addIncisorProbe (t, 0.001, "position");
      addLeftTmjProbe(t, 0.001, "position");
      addRightTmjProbe(t, 0.001, "position");
      addIncisorProbe (t, 0.001, "velocity");

      // System.out.println(myJawModel.rigidBodies().get("jaw").getPose().toString());
      // myJawModel.rigidBodies().get("jaw").transformGeometry(new RigidTransform3d(0,0,-0.1,0,0,0));
      // System.out.println(myJawModel.rigidBodies().get("jaw").getPose().toString());

      ComponentListView<BodyConnector> bcs = myJawModel.bodyConnectors();
      addMonitor(new TMJConstraintMonitor(bcs));

   }

   @Override
   public void attach (DriverInterface driver){
      workingDirname = "data";
      setWorkingDir();

      setIncisorVisible ();

      try {
         if (condyleTargets) {
            buildTargetProbeFile();
            loadTargetPositionProbe(ArtisynthPath.getSrcRelativePath (JawModelBaseline.class, "data/target_positions.txt"));
         } else if (frameTarget) {
            loadTargetPositionProbe(ArtisynthPath.getSrcRelativePath (JawModelBaseline.class, "data/jaw_pose_output.txt"));
         } else if (useIncisorTarget) {
            loadTargetPositionProbe(ArtisynthPath.getSrcRelativePath (JawModelInversePlayground.class, "data/") + incisorTargetpathFileName);
            // addClosingToTargerPositionProbe(ArtisynthPath.getSrcRelativePath (JawModelInversePlayground.class, "data/") + incisorTargetpathFileName);
         }
      }
      catch (IOException e) {
          // TODO Auto-generated catch block
          e.printStackTrace ();
      }

      // for (PlanarConnector c : myJawModel.con) {
      //    if (c.getName() == "LLTRL" || c.getName() == "RLTRL") {
      //       c.setUnilateral(false);
      //    }
      //    if (c.getName() == "LTMJ" || c.getName() == "RTMJ") {
      //       c.setUnilateral(false);
      //    }
      // }

      if (usePlaneJoint) {
         for (BodyConnector c : myJawModel.bodyConnectors()) {
            if (c.getName() == "LTMJ" || c.getName() == "RTMJ") {
               SegmentedPlanarConnector tmj = (SegmentedPlanarConnector)c;
               tmj.setUnilateral(false);
               System.out.println(c.getName() + " isUnilateral = " + tmj.isUnilateral());
            }
         }
      }

      Main.getMain().getViewer().setBackgroundColor (new Color(1f,1f,1f));

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

   public void addControlPanel () {

      ControlPanel panel = new ControlPanel ();
      panel.addWidget (myJawModel, "integrator");

      panel.addWidget (new JSeparator ());

      panel.addWidget (this, "E");
      panel.addWidget (this, "nu");
      panel.addWidget (this, "damping");
      panel.addWidget (this, "thickness");
   }

   public ArrayList<Point3d> loadTargetProbeData (String targetpath) throws IOException {

      File f = new File (targetpath);
      ArrayList<Point3d> dataList = new ArrayList<Point3d>();
      BufferedReader bReader = new BufferedReader(new FileReader(f.getAbsolutePath()));
      System.out.println(targetpath);
      String line;
      int linenum = 0;

      while ((line = bReader.readLine()) != null) {

         if (linenum != 0 && linenum != 1) {
            String dataVal[] = line.split("\\s+");
            String x = dataVal[2];
            String y = dataVal[3];
            String z = dataVal[4];

            Point3d p = new Point3d(Double.parseDouble(x), Double.parseDouble(y), Double.parseDouble(z));

            dataList.add(p);
         }
         linenum++;
      }
      bReader.close();
      return dataList;

   }

   public void loadTargetPositionProbe (String targetpath) throws IOException {

      File jawPosition = new File (targetpath);
      NumericInputProbe motionTargetProbe =
         (NumericInputProbe)getInputProbes ().get ("target positions");
      motionTargetProbe.setAttachedFileName (jawPosition.getPath ());
      motionTargetProbe.load ();
      motionTargetProbe.setActive (true);
   }


   public void buildTargetProbeFile() {
      File f = new File(ArtisynthPath.getSrcRelativePath (JawModelBaseline.class, "/data/target_positions.txt"));
      
      try {
         String line1 = "0.0 0.5 1.0";
         String line2 = "Linear 9 0.001";

         ArrayList<Point3d> ltmjPoints = loadTargetProbeData (ArtisynthPath.getSrcRelativePath (JawModelInversePlayground.class, "data/") + leftTmjTargetpathFileName);
         ArrayList<Point3d> rtmjPoints = loadTargetProbeData (ArtisynthPath.getSrcRelativePath (JawModelInversePlayground.class, "data/") + rightTmjTargetpathFileName);
         ArrayList<Point3d> incisorPoints = loadTargetProbeData (ArtisynthPath.getSrcRelativePath (JawModelInversePlayground.class, "data/") + incisorTargetpathFileName);

         FileOutputStream fos = new FileOutputStream(f);
         BufferedWriter bw = new BufferedWriter(new OutputStreamWriter(fos));

         bw.write(line1);
         bw.newLine();
         bw.write(line2);
         bw.newLine();

         for (int i = 0; i < incisorPoints.size(); i++) {
            String line = Double.toString(incisorPoints.get(i).x) + "\t" +
                          Double.toString(incisorPoints.get(i).y) + "\t" +
                          Double.toString(incisorPoints.get(i).z) + "\t" +
                          Double.toString(ltmjPoints.get(i).x) + "\t" +
                          Double.toString(ltmjPoints.get(i).y) + "\t" +
                          Double.toString(ltmjPoints.get(i).z) + "\t" +
                          Double.toString(rtmjPoints.get(i).x) + "\t" +
                          Double.toString(rtmjPoints.get(i).y) + "\t" +
                          Double.toString(rtmjPoints.get(i).z);
            bw.write(line);
            bw.newLine();
         }
         bw.close();
      }
      catch (IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace ();
      }
   }

   // public void addTargetPositionData() {
   //    // NumericInputProbe ltmjProbe = (NumericInputProbe)getInputProbes ().get ("ltmj_target");
   //    // ltmjProbe.
   // }

   public void addIncisorTarget (String targetpath) throws IOException {

      File jawPosition = new File (targetpath);
      System.out.println(targetpath);
      String compName = "controllers/myTrackingController/motionTerm/targetPoints/ltmj_ref";
      String prop = "position";
      String name = compName + prop;
      ModelComponent comp = myJawModel.findComponent(compName);
      NumericInputProbe p = new NumericInputProbe(myJawModel, "frameMarkers/lowerincisor:position", 0, t);
      p.setAttachedFileName (jawPosition.getPath ());
      p.setName("lowerincisor_target");
      p.load();
      addInputProbe(p);
   }

   public void addLeftTmjTarget (String targetpath) throws IOException {

      File jawPosition = new File (targetpath);
      System.out.println(targetpath);
      String compName = "controllers/myTrackingController/motionTerm/targetPoints/ltmj_ref";
      String prop = "position";
      String name = compName + prop;
      ModelComponent comp = myJawModel.findComponent(compName);
      NumericInputProbe p = new NumericInputProbe(myJawModel, "frameMarkers/ltmj:position", 0, t);
      p.setAttachedFileName (jawPosition.getPath ());
      p.setName("ltmj_target");
      p.load();
      addInputProbe(p);
   }

   public void addRightTmjTarget (String targetpath) throws IOException {

      File jawPosition = new File (targetpath);
      System.out.println(targetpath);
      String compName = "controllers/myTrackingController/motionTerm/targetPoints/ltmj_ref";
      String prop = "position";
      String name = compName + prop;
      ModelComponent comp = myJawModel.findComponent(compName);
      NumericInputProbe p = new NumericInputProbe(myJawModel, "frameMarkers/rtmj:position", 0, t);
      p.setAttachedFileName (jawPosition.getPath ());
      p.setName("rtmj_target");
      p.load();
      addInputProbe(p);

   }

   public void setIncisorVisible () {
      FrameMarker inc = myJawModel.frameMarkers ().get ("lowerincisor");
      if (inc == null)
         return;
      RenderProps.setLineColor (inc, Color.CYAN);
      RenderProps.setPointColor (inc, Color.CYAN);
      Point3d loc = new Point3d ();
      inc.getLocation (loc);
      inc.setLocation (loc);
      enableTracing (inc);
   }

   public void addTrackingController (double t) throws IOException {
      // set up tracking controller
      myTrackingController =
         new TrackingController (myJawModel, "myTrackingController");

         Scanner s =
         new Scanner (
            new FileReader (
               ArtisynthPath
                  .getSrcRelativePath (
                     JawModelBaselineMotion.class,
                     "geometry/" + excitersFile)));
      MuscleAbbreviation = new ArrayList<String> ();
      while (s.hasNext ()) {
         MuscleAbbreviation.add (s.next ());
      }
      s.close ();

      for (String name : MuscleAbbreviation) {
         Muscle m = (Muscle)myJawModel.axialSprings ().get (name);
         if (m == null) {
            MultiPointMuscle mm =
               (MultiPointMuscle)myJawModel.multiPointSprings ().get (name);
            myTrackingController.addExciter (mm);
            continue;
         }
         myTrackingController.addExciter (m);
      }
      
      MotionTargetComponent incisorTarget =
         myJawModel.frameMarkers ().get ("lowerincisor");

      MotionTargetComponent ltmjTarget =
         myJawModel.frameMarkers ().get ("ltmj");

      MotionTargetComponent rtmjTarget =
         myJawModel.frameMarkers ().get ("rtmj");


      myTrackingController.setNormalizeH (true);
      myTrackingController.addL2RegularizationTerm (4);
      if (condyleTargets) {
         myTrackingController.addMotionTarget(incisorTarget);
         myTrackingController.setMotionTargetWeight(incisorTarget, 10.0);
         myTrackingController.addMotionTarget (ltmjTarget);
         myTrackingController.addMotionTarget (rtmjTarget);
         myTrackingController.setMotionTargetWeight(ltmjTarget, 8.0);
         myTrackingController.setMotionTargetWeight(rtmjTarget, 8.0);
      }
      if (frameTarget) {
         myTrackingController.addMotionTarget(myJawModel.rigidBodies().get("jaw"));
      }
      if (useIncisorTarget) {
         myTrackingController.addMotionTarget(incisorTarget);
         myTrackingController.setMotionTargetWeight(incisorTarget, 5.0);
      }

      myTrackingController.setProbeDuration (t);
      myTrackingController.setExcitationBounds (0, 1);
      myTrackingController.setMaxExcitationJump (0.001);
      myTrackingController.setProbeUpdateInterval(0.001);
      myTrackingController.createProbesAndPanel(this);
      addController (myTrackingController);
   }

   public void addMuscleExciters (String filename, String name)
      throws IOException {
      MuscleExciter ex = new MuscleExciter (name);
      Scanner s =
         new Scanner (
            new FileReader (
               ArtisynthPath
                  .getSrcRelativePath (
                     JawModelInversePlayground.class, "data/muscles/" + filename)));
      MuscleAbbreviation = new ArrayList<String> ();
      while (s.hasNext ()) {
         MuscleAbbreviation.add (s.next ());
      }
      s.close ();
      for (int k = 0; k < MuscleAbbreviation.size (); k++) {
         String nom = MuscleAbbreviation.get (k);
         ExcitationComponent c = (Muscle)myJawModel.axialSprings ().get (nom);
         ex.addTarget (c, 1.0);
      }
      myJawModel.addMuscleExciter (ex);
   }

   public void setWorkingDir () {
      // set default working directory to repository location
    File workingDir =
    new File (
      ArtisynthPath
        .getSrcRelativePath (JawModelForward.class, ""));
  
   String path = ArtisynthPath.getHomeDir().replace("artisynth_core", "Patient Data");
   File patientDataDir = new File (path );
   if (patientDataDir.isDirectory()) {
     workingDir = patientDataDir;
     ArtisynthPath.setWorkingDir (workingDir);
   } else {
      ArtisynthPath.setWorkingDir (workingDir);
   }
   System.out.println(workingDir);
  
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

   public void addLeftTmjProbe (
      double duration, double interval, String propName) {
      String name = "ltmj_" + propName;
      NumericOutputProbe p =
         new NumericOutputProbe (
            new Property[] { myJawModel
               .frameMarkers ().get ("ltmj").getProperty (propName) },
            interval);
      p.setStartStopTimes (0.0, duration);
      p.setName (name);
      p.setAttachedFileName (name + "_output.txt");
      p.setDefaultDisplayRange (-0.1, 0.1);
      addOutputProbe (p);

   }

   public void addRightTmjProbe (
      double duration, double interval, String propName) {
      String name = "rtmj" + propName;
      NumericOutputProbe p =
         new NumericOutputProbe (
            new Property[] { myJawModel
               .frameMarkers ().get ("rtmj").getProperty (propName) },
            interval);
      p.setStartStopTimes (0.0, duration);
      p.setName (name);
      p.setAttachedFileName (name + "_output.txt");
      p.setDefaultDisplayRange (-0.1, 0.1);
      addOutputProbe (p);
   }

   private class TMJConstraintMonitor extends MonitorBase {

      int numUnilateralConstraints;
      ComponentListView<BodyConnector> bcs;

      TMJConstraintMonitor(ComponentListView<BodyConnector> bodyConnectors) {
         bcs = bodyConnectors;
         System.out.println("Monitors created for...");
         for (BodyConnector bc : bcs) {
            String name = bc.getName();
            System.out.print(name + ", ");
         }
         System.out.println("\n\n");
      }

      private void printNumUnilateralConstraints(ComponentListView<BodyConnector> bcs) {
         for (BodyConnector bc : bcs) {
            numUnilateralConstraints = bc.numUnilateralConstraints();
            String name = bc.getName();
            System.out.print(name + " : " + numUnilateralConstraints + "  ");
         }
         System.out.print("\n");
      }

      @Override
      public void apply (double t0, double t1) {
         printNumUnilateralConstraints(bcs);
      }
   }
}
