package artisynth.models.irsm.jawsurgery;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Scanner;

import artisynth.core.materials.AxialMuscleMaterial;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.BodyConnector;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.PointPlaneForce;
import artisynth.core.mechmodels.RevoluteJoint;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.RigidCylinder;
import artisynth.core.mechmodels.Wrappable;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.util.ArtisynthPath;
import artisynth.models.irsm.jawsurgery.JawModel;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;

public class JawModelBaseline extends JawModel {

   public static final String muscleListFilename = "muscleList.txt";

   public static final String wrappedMuscleListFilename =
      "wrappedMuscleList.txt";

   public static final String bodyListFilename = "bodyList.txt";

   public static final String muscleInfoFilename = "muscleInfo.txt";

   public static ArrayList<String> wrappedMuscleList = new ArrayList<String> ();

   public static final String muscleGroupInfoFilename = "muscleGroupsInfo.txt";

   protected ArrayList<Muscle> myMuscles = new ArrayList<Muscle> ();

   ArrayList<String> MuscleAbbreviation = new ArrayList<String> ();

   String excitersFile = "muscleList.txt";

   protected HashMap<String,String> muscleGroupNames =
      new LinkedHashMap<String,String> ();

   public boolean showCons = false;

   protected double myParticleDamping = 40;

   protected double myStiffnessDamping = 10;

   protected static PropertyList myProps =
      new PropertyList (JawModelBaseline.class, JawModel.class);

   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

   private void setupRenderProps () {

      // Line Renderprops
      RenderProps props = createRenderProps ();
      props.setLineRadius (2.0);
      props.setLineWidth (3);
      props.setLineStyle (Renderer.LineStyle.LINE);
      props.setLineColor (Color.WHITE);
      // RenderProps.setLineColor()

      // Mesh RenderProps
      props.setShading (Renderer.Shading.SMOOTH);
      props.setFaceColor (new Color (1f, 0.8f, 0.6f));
      props.setFaceStyle (Renderer.FaceStyle.FRONT_AND_BACK);
      setRenderProps (props);

      // Spring Render Props
      RenderProps.setLineRadius (myAxialSprings, 2.0);
      RenderProps.setLineStyle (myAxialSprings, Renderer.LineStyle.SPINDLE);
      RenderProps.setLineColor (myAxialSprings, Color.WHITE);

      // Marker RenderProps
      frameMarkers ().getRenderProps ().setPointStyle (PointStyle.SPHERE);
      frameMarkers ().getRenderProps ().setPointSize (1);
      frameMarkers ().getRenderProps ().setPointColor (Color.PINK);

      // tmj point render props
      RenderProps.setPointColor(frameMarkers().get("ltmj"), Color.GREEN);
      RenderProps.setPointColor(frameMarkers().get("rtmj"), Color.GREEN);

      // Force Effector and BodyCon Colors
      // ArrayList<PointPlaneForce> forceEffectorCon = new ArrayList<PointPlaneForce>(NUM_CON);
      // ArrayList<PlanarConnector> bodyCon = new ArrayList<PlanarConnector>();

      // forceEffectorCon = this.forceEffectorCon;
      // bodyCon = this.con;

      Color myPurple = new Color(204,204,255,255);
      Color myGrey = new Color(153,153,153,255);
      Color myRed = new Color(255,102,102,255);

      for (PointPlaneForce ppf :  this.forceEffectorCon) {
         if(ppf.getName() == "LPOST" || ppf.getName() == "RPOST") {
            ppf.getRenderProps().setFaceColor(myPurple);
			   ppf.getRenderProps().setAlpha(1);
         }
         if(ppf.getName() == "LLTRL" || ppf.getName() == "RLTRL") {
            ppf.getRenderProps().setFaceColor(myGrey);
			   ppf.getRenderProps().setAlpha(1);
         }
         if (showCons == false) {
            ppf.getRenderProps().setVisible(false);
         }
      }
      for (BodyConnector bc : this.con) {
         if (bc.getName() == "LTMJ" || bc.getName() == "RTMJ") {
            bc.getRenderProps().setFaceColor(myRed);
            bc.getRenderProps().setAlpha(1);
         }
         if (showCons == false) {
            bc.getRenderProps().setVisible(false);
         }
      }

      for (FrameMarker fm : frameMarkers()) {
         if (fm.getName() == "ltmj" || fm.getName() == "rtmj") {
            fm.getRenderProps().setPointColor(Color.GREEN);
         }
      }

      // for (PlanarConnector pbc : bodyCon) {
      //    if (pbc.getName() == "LTMJ" || pbc.getName() == "RTMJ") {
      //       pbc.getRenderProps().setFaceColor(myRed);
      //       pbc.getRenderProps().setAlpha(1);
      //    }
      // }
   }

   // update intertia and center of mass for the new geometry
   public void setNewJawDynamicProps () {
      rigidBodies ().get ("jaw").setInertiaFromMass (0.2);
      // rigidBodies().get("jaw").setCenterOfMass(new Point3d(6.954, -61.147,
      // 44.602));
      rigidBodies ().get ("jaw").setRotaryDamping (100);
      rigidBodies ().get ("jaw").setFrameDamping (500);
   }

   // Load Mesh files
   public PolygonalMesh loadFemMesh (String meshName, double scale) {
      String meshFilename =
         ArtisynthPath
            .getSrcRelativePath (
               JawModelBaseline.class, "geometry/" + meshName);
      PolygonalMesh mesh = new PolygonalMesh ();

      try {
         mesh.read (new BufferedReader (new FileReader (meshFilename)));
      }
      catch (IOException e) {
         e.printStackTrace ();
      }

      mesh.scale (scale);
      mesh.setFixed (true);
      // mesh.transform (amiraTranformation);

      return mesh;
   }

   /**
    * used to translate the frames of each body to the center of the body and
    * translate components accordingly
    */
   public void translateFrame (RigidBody body) {
      Vector3d centroid = new Vector3d ();
      body.getSurfaceMesh ().computeCentroid (centroid);
      RigidTransform3d XComToBody = new RigidTransform3d ();
      XComToBody.p.set (centroid);

      RigidTransform3d XBodyToWorld = new RigidTransform3d ();
      body.getPose (XBodyToWorld);

      RigidTransform3d XComToWorld = new RigidTransform3d ();
      XComToWorld.mul (XBodyToWorld, XComToBody);
      body.setPose (XComToWorld);

      RigidTransform3d XMeshToCom = new RigidTransform3d ();
      if (body.getSurfaceMesh () != null) {
         PolygonalMesh mesh = body.getSurfaceMesh ();
         XMeshToCom.invert (XComToWorld);
         mesh.transform (XMeshToCom);
         body.setSurfaceMesh (mesh, null);
      }

      for (FrameMarker mrk : frameMarkers ()) {
         if (mrk.getFrame () == body) {
            Point3d loc = new Point3d ();
            mrk.getLocation (loc);
            loc.transform (XMeshToCom);
            mrk.setLocation (loc);
         }
      }

      for (BodyConnector con : bodyConnectors ()) {
         if (con.getBodyA () == body) {
            con.transformGeometry (XComToWorld);
         }
      }
   }

   public void addWrappedMuscles (
      ArrayList<String> wrappedMuscleList,
      HashMap<String,ExcitationComponent> myMuscles) {
      HashMap<String,RigidBody> wrappingBodies =
         new LinkedHashMap<String,RigidBody> ();
      /*
       * as of now only the superior head of the lateral pterygoid has wrapping
       * geometry wrapping bodies are created and only added if the related
       * muscles in part of wrappedMuscleList
       */
      double size = 10;
      double density = 150;
      RigidCylinder cylinder =
         new RigidCylinder ("cylinder_rsp", size / 1.1, 2 * size, density, 50);
      cylinder
         .setPose (
            new RigidTransform3d (
               new Vector3d (-40.1763, -10.8551, 2.3), new AxisAngle (
                  -0.12701, 0.98571, 0.11067, Math.toRadians (89.728))));
      cylinder.setDynamic (false);
      cylinder.getRenderProps ().setFaceColor (Color.BLUE);
      cylinder.getRenderProps ().setFaceColor (Color.BLUE);
      cylinder.getRenderProps ().setFaceStyle (FaceStyle.NONE);
      cylinder.getRenderProps ().setLineColor (Color.BLUE);
      cylinder.getRenderProps ().setDrawEdges (true);

      wrappingBodies.put ("rsp", cylinder);

      RigidCylinder cylinder2 =
         new RigidCylinder ("cylinder_lsp", size / 1.1, 2 * size, density, 50);
      cylinder2.setDynamic (false);
      cylinder2
         .setPose (
            new RigidTransform3d (
               new Vector3d (49.0791, -14.5805, 4.64117), new AxisAngle (
                  0.14726, 0.98318, -0.10801, Math.toRadians (90.734))));
      cylinder2.getRenderProps ().setFaceColor (Color.BLUE);
      cylinder2.getRenderProps ().setFaceColor (Color.BLUE);
      cylinder2.getRenderProps ().setFaceStyle (FaceStyle.NONE);
      cylinder2.getRenderProps ().setLineColor (Color.BLUE);
      cylinder2.getRenderProps ().setDrawEdges (true);
      wrappingBodies.put ("lsp", cylinder2);

      ArrayList<AxialSpring> wrappedMuscles = new ArrayList<AxialSpring> ();

      for (int i = 0; i < wrappedMuscleList.size (); i++) {
         wrappedMuscles.add (axialSprings ().get (wrappedMuscleList.get (i)));
         addRigidBody (wrappingBodies.get (wrappedMuscles.get (i).getName ()));
      }

      /*
       * sets up MultiPointMuscle using the same material properties that have
       * been used for the respective axial spring muscle and deletes axial
       * spring muscle + replaces it in myMuscles List
       */
      for (int i = 0; i < wrappedMuscles.size (); i++) {
         MultiPointMuscle m = new MultiPointMuscle ();
         m
            .addPoint (
               ((Muscle)axialSprings ().get (wrappedMuscleList.get (i)))
                  .getFirstPoint ());
         m.setSegmentWrappable (20, new Point3d[] { new Point3d (0, 0, 0) });
         m
            .addWrappable (
               (Wrappable)wrappingBodies.get (wrappedMuscleList.get (i)));
         m
            .addPoint (
               ((Muscle)axialSprings ().get (wrappedMuscleList.get (i)))
                  .getSecondPoint ());
         m.updateWrapSegments ();
         m
            .setMaterial (
               ((Muscle)axialSprings ().get (wrappedMuscleList.get (i)))
                  .getMaterial ());
         String name =
            ((Muscle)axialSprings ().get (wrappedMuscleList.get (i)))
               .getName ();

         double length = m.getLength ();
         double optLength =
            ((AxialMuscleMaterial)m.getMaterial ()).getOptLength ();
         double maxLength =
            ((AxialMuscleMaterial)m.getMaterial ()).getMaxLength ();
         double maxOptRatio = (optLength != 0.0) ? maxLength / optLength : 1.0;
         ((AxialMuscleMaterial)m.getMaterial ()).setOptLength (length);
         ((AxialMuscleMaterial)m.getMaterial ())
            .setMaxLength (length * maxOptRatio);
         ((AxialMuscleMaterial)m.getMaterial ())
            .setMaxForce (
               ((AxialMuscleMaterial)m.getMaterial ()).getMaxForce () * 0.001);

         m.setName (name);
         m.setExcitationColor (Color.RED);
         m.getRenderProps ().setLineStyle (LineStyle.CYLINDER);
         m.getRenderProps ().setLineRadius (0.75);
         addMultiPointSpring (m);

         myMuscles.remove (name);
         myMuscles.put (name, m);
      }

      ComponentUtils.removeComponents (wrappedMuscles, null);
   }

   public void addAllExciters () {
      Scanner s;
      try {
         s =
            new Scanner (
               new FileReader (
                  ArtisynthPath
                     .getSrcRelativePath (
                        JawModelBaseline.class, "geometry/" + excitersFile)));
         MuscleAbbreviation = new ArrayList<String> ();
         while (s.hasNext ()) {
            MuscleAbbreviation.add (s.next ());
         }
         s.close ();
      }
      catch (FileNotFoundException e) {
         // TODO Auto-generated catch block
         e.printStackTrace ();
      }

      for (String name : MuscleAbbreviation) {
         Muscle m = (Muscle)axialSprings ().get (name);
         MuscleExciter mex = new MuscleExciter (name);
         mex.addTarget (m);
         addMuscleExciter(mex);
      }
   }

   // Model Constructor
   public JawModelBaseline (String name, boolean usePlaneJoint, boolean useRevoluteJoint, boolean useCondyleCapsule, boolean useCondyleTargets, boolean useForceEffectorCons) throws IOException {
      super ();
      this.setName (name);
      setGravity (0, 0, -gravityVal * unitConversion);

      String osType = System.getProperty("os.name");
      String geometryFilename = "/geometry/";

      if (osType.equals("Mac OS X"))
      {
         geometryFilename = "/geometry/";
      }
      else if (osType.startsWith("Windows"))
      {
         geometryFilename = "\\geometry\\";
      } 
     
      JawModel.muscleList = readStringList (ArtisynthPath.getWorkingDirPath() + geometryFilename + muscleListFilename);
      JawModel.bodyInfoList = readBodyInfoList (ArtisynthPath.getWorkingDirPath() + geometryFilename + bodyListFilename);

      JawModel.muscleInfo = readMuscleInfo (ArtisynthPath.getWorkingDirPath() + geometryFilename + muscleInfoFilename);
      JawModel.muscleGroupInfo = readMuscleGroupsInfo (ArtisynthPath.getWorkingDirPath() + geometryFilename + muscleGroupInfoFilename);
      ArrayList<RigidBody> bodies = JawModel.assembleRigidBodies ( bodyInfoList, ArtisynthPath.getWorkingDirPath() +  (osType.startsWith("Windows") ? "\\" : "/"));

      // for (RigidBody body : bodies) {
      //    addRigidBody (body);
      // }

      setNewJawDynamicProps ();

      ArrayList<FrameMarker> markers = JawModel.assembleMarkersForMuscles ( muscleList, muscleInfo, myRigidBodies,ArtisynthPath.getWorkingDirPath() +  geometryFilename + "modelFrameMarkers.csv");
      HashMap<String,FrameMarker> myMarkerInfo = new LinkedHashMap<String,FrameMarker> ();

      for (FrameMarker marker : markers) {
         addFrameMarker (marker);
         myMarkerInfo.put (marker.getName (), marker);
      }

      ArrayList<Muscle> myAssembledMuscles = JawModel.assembleandreturnMuscles ();
      ArrayList<Muscle> myAttachedMuscles = JawModel.attachMuscles (muscleList, muscleInfo, myMarkerInfo, myAssembledMuscles);
      
      HashMap<String,ExcitationComponent> myMuscles = new LinkedHashMap<String,ExcitationComponent> ();

      for (Muscle muscle : myAttachedMuscles) {
         muscle.setExcitationColor (Color.RED);
         muscle.setMaxColoredExcitation (1);
         addAxialSpring (muscle);
         myMuscles.put (muscle.getName (), muscle);
      }

      wrappedMuscleList = readStringList (ArtisynthPath.getWorkingDirPath() + geometryFilename + wrappedMuscleListFilename);

      addIncisorMarker(ArtisynthPath.getWorkingDirPath() + geometryFilename + "modelFrameMarkers.csv");

      addBiteMarkers(ArtisynthPath.getWorkingDirPath() + geometryFilename + "modelFrameMarkers.csv");

      addCondyleMarkers(ArtisynthPath.getWorkingDirPath() + geometryFilename + "modelFrameMarkers.csv");

      for (RigidBody body : bodies) {
         translateFrame (body);
      }

      closerMuscleList = JawModel.createMuscleList(readStringList(ArtisynthPath.getWorkingDirPath() + geometryFilename+"closerMuscleList.txt"), muscleInfo, myAttachedMuscles);
      assembleBilateralExcitors(muscleList, muscleInfo, myMuscles, muscleAbbreviations);
      HashMap<String,MuscleExciter>  exciters = new LinkedHashMap<String,MuscleExciter>  (); 


      for(MuscleGroupInfo info : muscleGroupInfo){
         exciters=assembleMuscleGroups(info, myMuscles, getMuscleExciters (),  muscleAbbreviations);
         addMuscleExciter (exciters.get ("l"+info.name));
         addMuscleExciter (exciters.get ("r"+info.name));
      }
      
      ArrayList<MuscleExciter> bilateral_exciters= new ArrayList<> ();
      bilateral_exciters=assemblebilateralMuscleGroups(muscleGroupInfo, getMuscleExciters (), muscleAbbreviations);      
      
      for(MuscleExciter exciter : bilateral_exciters){
         addMuscleExciter (exciter);
       }

      //  addAllExciters();

      JawModel.updateMuscleLengthProps (myAttachedMuscles);


      constrainedBody = myRigidBodies.get("jaw");
      if (constrainedBody == null) {
	      System.err.println("JawModel: unable to get jaw rigidbody");
	      return;
      }

      if (useRevoluteJoint) {
         initRevoluteCons();

      } else if (useCondyleCapsule && !useCondyleTargets) {
         setCondylarCapsule();
      } else if (useCondyleCapsule && useCondyleTargets) {
         setCondylarCapsuleWithTmjPlane();
      }
      else if (usePlaneJoint) {
         setCondyleConstraint();
         
      } else if (!useCondyleCapsule && useCondyleTargets) {
         setCondyleTargets();
      } else if (useForceEffectorCons) {
         setCondyleForceEffectorCons();
         // set point damping for tmj frame markers
         myFrameMarkers.get("ltmj").setPointDamping(1);
         myFrameMarkers.get("rtmj").setPointDamping(1);
      }
      else {
         initCons();
         removeBodyConnector(con.get(0));
         removeBodyConnector(con.get(1));
         addCurvilinearTmjs(); 
      }
   
      setupRenderProps ();

   }
}
