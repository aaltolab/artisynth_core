package artisynth.models.irsm.jawsurgery;

import java.awt.Color;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import javax.swing.JFileChooser;
import javax.swing.JFrame;

import java.awt.event.ActionEvent;

import artisynth.core.driver.Main;
import artisynth.core.inverse.ForceTargetTerm;
import artisynth.core.inverse.TrackingController;
import artisynth.core.mechmodels.BodyConnector;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.SegmentedPlanarConnector;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.Property;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer.LineStyle;
import maspack.widgets.GuiUtils;

public class JawModelBaselineMotion extends RootModel {
   JawModelBaseline myJawModel;
   TrackingController myTrackingController;
   ArrayList<String> MuscleAbbreviation = new ArrayList<String> ();
   String excitersFile = "inv2.txt";
   protected String workingDirname = "data";

   double t = 0.5; // 0.5 prot; 0.75 open; 0.225 brux


   boolean useCondyleCapsule = false;
   boolean usePlaneJoint = false;
   boolean useRevoluteJoint = false;
   boolean useCondyleTargets = false;
   boolean useForceEffectorCons = true;

   public JawModelBaselineMotion () {
      // TODO Auto-generated constructor stub
   }

   public JawModelBaselineMotion (String name) {
      super (null);
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      setWorkingDir ();

      myJawModel = new JawModelBaseline ("jawmodel", usePlaneJoint, useRevoluteJoint, useCondyleCapsule, useCondyleTargets, useForceEffectorCons);

      addModel (myJawModel);
      getRoot (this).setMaxStepSize (0.01);

      // addPreload(0,t/6);

      // addProtrusion();
      // addLaterotrusionRight();
      // addLaterotrusionLeft();

      // for (double i = 0.001; i <= t; i = i + 0.001) {
      //    addWayPoint (i);
      // }

      if (usePlaneJoint || useForceEffectorCons) {
         myJawModel.setRLtrlWallAngle(0);
         myJawModel.setLLtrlWallAngle(0);
      }

      addBreakPoint (t);
      addOutPutProbes (t);

      // System.out.println("Setting view for cam pose");
      // RigidTransform3d camPose = new RigidTransform3d(new Vector3d(0,0,0), new AxisAngle(0.28155, 0.04734, 0.95838, 27.733));
      // myJawModel.rigidBodies().get("jaw").setPose(camPose);
      // myJawModel.rigidBodies().get("skull").setPose(camPose);
      // myJawModel.rigidBodies().get("hyoid").setPose(camPose);
      // System.out.println(myJawModel.rigidBodies().get("jaw").getPose().toString());
   }
   
   @Override
   public void attach (DriverInterface driver){

      setIncisorVisible();

      try {
         addOpening (0, 0.25);
         addClosingForce(0.25,0.5);
      }
      catch (IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace ();
      }


      if (usePlaneJoint) {
         myJawModel.bodyConnectors().get("LTMJ").setEnabled(true);
         myJawModel.bodyConnectors().get("RTMJ").setEnabled(true);

         for (BodyConnector c : myJawModel.bodyConnectors()) {
            if (c.getName() == "LTMJ" || c.getName() == "RTMJ") {
               SegmentedPlanarConnector tmj = (SegmentedPlanarConnector)c;
               tmj.setUnilateral(false);
               System.out.println(c.getName() + " isUnilateral = " + tmj.isUnilateral());
            }
         }
      }

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

   @Override
   public boolean getMenuItems (List<Object> items) {
      items.add (GuiUtils.createMenuItem (this, "Save Muscle Points", ""));
      items.add (GuiUtils.createMenuItem (this, "Import Muscle Points", ""));

      return true;
   }

   public void actionPerformed (ActionEvent event) {
      String cmd = event.getActionCommand();
      if (cmd.equals ("Save Muscle Points")) {
         try {
            saveMuscleFrameMarkers ();
         }
         catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace ();
         }
      }
   }

   // Save Muscle Frame Marker Origin and Insertion Points 
   public void saveMuscleFrameMarkers() throws IOException {
      File fileToSave;
      // parent component of the dialog
      JFrame parentFrame = new JFrame();

      JFileChooser fileChooser = new JFileChooser();
      fileChooser.setDialogTitle("Specify a file to save");   

      int userSelection = fileChooser.showSaveDialog(parentFrame);

      if (userSelection == JFileChooser.APPROVE_OPTION) {
          fileToSave = fileChooser.getSelectedFile();
          System.out.println("Save as file: " + fileToSave.getAbsolutePath());

          FileOutputStream fos = new FileOutputStream(fileToSave);
          BufferedWriter bw = new BufferedWriter(new OutputStreamWriter(fos));

          RenderableComponentList<FrameMarker> frameMarkers =  myJawModel.frameMarkers();
      
          for (FrameMarker fm : frameMarkers) {
             String name = fm.getName();
             Point3d pnt = fm.getPosition();
    
             bw.write(name + "," + Double.toString(pnt.x) + "," + Double.toString(pnt.y) + "," + Double.toString(pnt.z));
             bw.newLine();
          }
    
          bw.close();
      }
   }
   
   public void addOutPutProbes (double t) {

      double sampleRate = 0.001;

      addRightTmjProbe(t, sampleRate, "position");
      addLeftTmjProbe(t, sampleRate, "position");

      NumericOutputProbe incisor =
         new NumericOutputProbe (
            myJawModel, "frameMarkers/lowerincisor:position",
            ArtisynthPath.getWorkingDir ().getPath () + "\\", sampleRate);
      incisor.setName ("lowerincisor_position");
      incisor.setAttachedFileName ("lowerincisor_position.txt");
      incisor.setDefaultDisplayRange (-1, 1);
      incisor.setStopTime (t);
      addOutputProbe (incisor);
      addMandPoseOutputProbes(t);
   }

   public void addClosingForce (double ti, double tf) throws IOException {

      MuscleExciter mex = myJawModel.getMuscleExciters ().get ("bi_close");
      String f = ArtisynthPath.getSrcRelativePath (JawModelBaseline.class, "/data/close_activation.txt");
      String prop = "excitation";
      NumericInputProbe probe = new NumericInputProbe(mex, prop, ti , tf);
      probe.setAttachedFileName(f);
      probe.setName ("Closing Muscle Activation");
      probe.load();
      addInputProbe (probe);
   }

   public void addOpening (double ti, double tf) throws IOException {

      MuscleExciter mex = myJawModel.getMuscleExciters ().get ("bi_open");
      String f = ArtisynthPath.getSrcRelativePath (JawModelBaseline.class, "/data/open_activation.txt");
      String prop = "excitation";
      NumericInputProbe probe = new NumericInputProbe(myJawModel, "exciters/bi_open:excitation", ti , tf);
      probe.setAttachedFileName(f);
      probe.setName ("Opening Muscle Activation");
      probe.load();
      addInputProbe (probe);
   }

   public void addPreload (double ti, double tf) throws IOException {
      for (BodyConnector p : myJawModel.bodyConnectors ()) {

         if (p.getName ().equals ("BiteICP") == false) {
            p.setEnabled (false);
            p.getRenderProps ().setVisible (false);
         }

      }

      MuscleExciter mex = myJawModel.getMuscleExciters ().get ("bi_close");

      NumericInputProbe probe = new NumericInputProbe (mex, "excitation", ArtisynthPath.getSrcRelativePath (JawModelBaseline.class, "/data/preload_activation.txt"));
      
      probe.setStartStopTimes (ti, tf);
      probe.setName ("Preload Muscle Activation");
      addInputProbe (probe);
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
      p.setAttachedFileName (name + ".txt");
      p.setDefaultDisplayRange (-0.1, 0.1);
      addOutputProbe (p);

   }

   public void addRightTmjProbe (
      double duration, double interval, String propName) {
      String name = "rtmj_" + propName;
      NumericOutputProbe p =
         new NumericOutputProbe (
            new Property[] { myJawModel
               .frameMarkers ().get ("rtmj").getProperty (propName) },
            interval);
      p.setStartStopTimes (0.0, duration);
      p.setName (name);
      p.setAttachedFileName (name + ".txt");
      p.setDefaultDisplayRange (-0.1, 0.1);
      addOutputProbe (p);
   }

   public void addMandPoseOutputProbes (double stoptime) {
      RigidBody mand = myJawModel.rigidBodies().get("jaw");
      String propNames[] = {"position", "orientation"};
      
      if (mand != null) {
         NumericOutputProbe op;
         op =
            new NumericOutputProbe (
               mand, propNames, null, myJawModel.getMaxStepSize ());
         op.setName (mand.getName() + " pose");
         op.setStartStopTimes (0, stoptime);
         op.setAttachedFileName (mand.getName() + "_pose_output.txt");
         addOutputProbe (op);
      }
   }
   
   public void addProtrusion () throws IOException {

      for (BodyConnector p : myJawModel.bodyConnectors ()) {

         if (p.getName ().equals ("BiteICP") == false) {
            p.setEnabled (false);
            p.getRenderProps ().setVisible (false);
         }

      }
      MuscleExciter mex = myJawModel.getMuscleExciters ().get ("bi_pro");

      NumericInputProbe probe = new NumericInputProbe (mex, "excitation", ArtisynthPath.getSrcRelativePath (JawModelBaseline.class, "/data/input_activation.txt"));

      probe.setStartStopTimes (0, 0.5);
      probe.setName ("Protrusion Muscle Activation");
      addInputProbe (probe);
   }

   public void addLaterotrusionRight () throws IOException {
      MuscleExciter mex = new MuscleExciter ("lat_right");
      mex.addTarget ((Muscle)myJawModel.axialSprings ().get ("lsp"), 1);
      mex.addTarget ((Muscle)myJawModel.axialSprings ().get ("lip"), 1);
      mex.addTarget ((Muscle)myJawModel.axialSprings ().get ("lmp"), 0.2);
      mex.addTarget ((Muscle)myJawModel.axialSprings ().get ("lad"), 0.4);
      mex.addTarget ((Muscle)myJawModel.axialSprings ().get ("rad"), 0.4);
      mex.addTarget ((Muscle)myJawModel.axialSprings ().get ("rmt"), 0.2);
      mex.addTarget ((Muscle)myJawModel.axialSprings ().get ("rat"), 0.2);
      myJawModel.addMuscleExciter (mex);
      // mex.setName ("lat_right");
      NumericInputProbe probe =
         new NumericInputProbe (
            mex, "excitation",
            ArtisynthPath
               .getSrcRelativePath (
                  JawModelBaseline.class, "/data/input_activation.txt"));
      probe.setStartStopTimes (0, 1);
      probe.setName ("Laterotrusion Right Activation");
      addInputProbe (probe);
   }

   public void addLaterotrusionLeft () throws IOException {
      MuscleExciter mex = new MuscleExciter ("lat_left");
      mex.addTarget ((Muscle)myJawModel.axialSprings ().get ("rsp"), 1);
      mex.addTarget ((Muscle)myJawModel.axialSprings ().get ("rip"), 1);
      mex.addTarget ((Muscle)myJawModel.axialSprings ().get ("rmp"), 0.4);
      // mex.addTarget ((Muscle)myJawModel.axialSprings ().get ("rad"),0.3);
      mex.addTarget ((Muscle)myJawModel.axialSprings ().get ("lad"), 0.3);
      mex.addTarget ((Muscle)myJawModel.axialSprings ().get ("lmt"), 0.1);
      mex.addTarget ((Muscle)myJawModel.axialSprings ().get ("lat"), 0.1);
      myJawModel.addMuscleExciter (mex);
      NumericInputProbe probe =
         new NumericInputProbe (
            mex, "excitation",
            ArtisynthPath
               .getSrcRelativePath (
                  JawModelBaseline.class, "/data/input_activation.txt"));
      probe.setStartStopTimes (0, 1);
      probe.setName ("Laterotrusion Left Activation");

      addInputProbe (probe);

   }

   public void addTrackingController () throws IOException {
      // set up tracking controller
      myTrackingController = new TrackingController (myJawModel, "tcon");
      Scanner s =
         new Scanner (
            new FileReader (
               ArtisynthPath
                  .getSrcRelativePath (
                     JawModelBaselineMotion.class,
                     "data/muscles/" + excitersFile)));
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

      myTrackingController.setNormalizeH (true);
      myTrackingController.addL2RegularizationTerm (0.0025);
      myTrackingController.addDampingTerm ();
      myTrackingController
         .addMotionTarget (myJawModel.rigidBodies ().get ("jaw"));
      myTrackingController.setProbeDuration (t);
      myTrackingController.setProbeUpdateInterval (0.001);
      myTrackingController.createProbesAndPanel (this);
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
                     JawModelBaseline.class, "data/muscles/" + filename)));
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

   public void addForceTargets (List<String> for_tar_nam) throws IOException {
      ForceTargetTerm mft = myTrackingController.addForceTargetTerm();
      for (String name : for_tar_nam) {
         ((PlanarConnector)myJawModel.bodyConnectors ().get (name))
            .setUnilateral (false);
         mft
            .addForceTarget (
               ((PlanarConnector)myJawModel.bodyConnectors ().get (name)));
      }
      myTrackingController.setNormalizeH (true);
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

   }
   
   public void setWorkingDir () {
      // set default working directory to repository location
      File workingDir =
         new File (
            ArtisynthPath
               .getSrcRelativePath (JawModelBaselineMotion.class, ""));

      String path = ArtisynthPath.getHomeDir().replace("artisynth_core", "Patient Data");
      File patientDataDir = new File (path );

      if (patientDataDir.isDirectory()) {
         ArtisynthPath.setWorkingDir (patientDataDir);
      } else {
         ArtisynthPath.setWorkingDir (workingDir);
      }
   }
}