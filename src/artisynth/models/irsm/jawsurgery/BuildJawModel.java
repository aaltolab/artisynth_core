package artisynth.models.irsm.jawsurgery;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JSeparator;
import javax.swing.filechooser.FileNameExtensionFilter;

import maspack.geometry.PolygonalMesh;
import maspack.geometry.io.GenericMeshReader;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyMode;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import maspack.render.Renderer.Shading;
import maspack.util.ReaderTokenizer;
import maspack.widgets.GuiUtils;
import artisynth.core.driver.MainFrame;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.ComponentListView;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.RootModel;


public class BuildJawModel extends RootModel {

   public class MuscleInfo {
      public String name;
   
      public String origin;
   
      public String insertion;
   
      public String fullName;
   
      boolean pairedFlag; // true == left-right paired muscle
   
      public boolean isPaired () {
         return pairedFlag;
      }
   
      public void scan (ReaderTokenizer rtok) throws IOException {
         name = rtok.sval;
         rtok.nextToken ();
         origin = rtok.sval;
         rtok.nextToken ();
         insertion = rtok.sval;
         rtok.nextToken ();
         pairedFlag = (rtok.sval.compareTo ("paired") == 0.0);
         rtok.nextToken ();
         fullName = rtok.sval;
      }
   
   }
   
   private MainFrame myFrame;
   protected BuildJawModel myBuildJawModel;
   private String workDir = "";
    
   MechModel mech;
   Vector3d gravity = new Vector3d(0.0,0.0,-9.81);

   // Bones
   RigidBody skull;
   RigidBody mandible;
   RigidBody hyoid;

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);

      mech = new MechModel ("mech");
      addModel (mech);
      mech.setGravity (gravity);
      mech.setIntegrator (Integrator.ConstrainedBackwardEuler);
      mech.setMaxStepSize (0.01);

      // Rigid Bodies
      addBones();
      setRenderProperties();
    
   }

   @Override
   public boolean getMenuItems(List<Object> items) {
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
      if (cmd.equals ("Import Muscle Points")) {
         try {
            loadMuscleFrameMarkers ();
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
      fileChooser.setCurrentDirectory(new File(this.workDir));

      int userSelection = fileChooser.showSaveDialog(parentFrame);

      if (userSelection == JFileChooser.APPROVE_OPTION) {
          fileToSave = fileChooser.getSelectedFile();
          System.out.println("Save as file: " + fileToSave.getAbsolutePath());

          FileOutputStream fos = new FileOutputStream(fileToSave);
          BufferedWriter bw = new BufferedWriter(new OutputStreamWriter(fos));

          RenderableComponentList<FrameMarker> frameMarkers =  mech.frameMarkers();
      
          for (FrameMarker fm : frameMarkers) {
             String name = fm.getName();
             Point3d pnt = fm.getPosition();
    
             bw.write(name + "," + Double.toString(pnt.x) + "," + Double.toString(pnt.y) + "," + Double.toString(pnt.z));
             bw.newLine();
          }
    
          bw.close();
      }
   }

   public void loadMuscleFrameMarkers() throws IOException {
      FrameMarker myMarker;
      FrameMarker modelMarker;
      ArrayList<FrameMarker> myMarkers = new ArrayList<> ();
      ArrayList<FrameMarker> modelMarkers = new ArrayList<> ();
      File[] frameMarkerFiles = selectFiles (this.workDir);
      ArrayList<String> muscleList = new ArrayList<String> ();
      HashMap<String,MuscleInfo> muscleInfo = new HashMap<String,MuscleInfo>();
      ComponentListView<RigidBody> myRigidBodies = mech.rigidBodies();

      muscleList = readStringList (ArtisynthPath.getSrcRelativePath(BuildJawModel.class, "/geometry/") +  "muscleList.txt");
      muscleInfo = readMuscleInfo (ArtisynthPath.getSrcRelativePath(BuildJawModel.class, "/geometry/") + "muscleInfo.txt");

      
      try {
         System.out.println(frameMarkerFiles[0].getAbsolutePath());
         File csvFile = new File (frameMarkerFiles[0].getAbsolutePath());
         // create BufferedReader and read data from csv
         BufferedReader csvReader;
         csvReader = new BufferedReader (new FileReader (csvFile.getAbsolutePath()));
         String row = new String ();
         while ((row = csvReader.readLine ()) != null) {
            String[] data = row.split (",");
            System.out.println(data);
            myMarker = new FrameMarker();
            myMarker.setName(data[0]);
            Point3d pnt = new Point3d(Double.parseDouble(data[1]),Double.parseDouble(data[2]),Double.parseDouble(data[3]));
            myMarker.setPosition(pnt);

            System.out.println(myMarker.getName() + ": (" + myMarker.getPosition().x + "," + myMarker.getPosition().y + "," + myMarker.getPosition().z + ")" );
            myMarkers.add(myMarker);
         }
         csvReader.close ();
      }
      catch (IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace ();
      }
      
      for (int k = 0; k < muscleList.size(); k++) {
         String name = muscleList.get(k);
         BuildJawModel.MuscleInfo info = muscleInfo.get(name);
         RigidBody origin = myRigidBodies.get(info.origin);
         RigidBody insertion = myRigidBodies.get(info.insertion);

         try {
            for (FrameMarker fm : myMarkers) {
               // System.out.println(fm.getName().substring(0, 3) + " == " + name);
               if (fm.getName().substring(0, 3).equals(name)) {
                  if (fm.getName().endsWith("_insertion")) {
                     modelMarker = new FrameMarker();
                     modelMarker.setName(fm.getName());
                     modelMarker.setFrame(insertion);
                     modelMarker.setLocation(fm.getPosition());
                     modelMarker.setRefPos(fm.getPosition());
                     // modelMarkers.add(modelMarker);
                     mech.addFrameMarker(modelMarker);
                  } else if (fm.getName().endsWith("_origin")) {
                     modelMarker = new FrameMarker();
                     modelMarker.setName(fm.getName());
                     modelMarker.setFrame(origin);
                     modelMarker.setLocation(fm.getPosition());
                     modelMarker.setRefPos(fm.getPosition());
                     // modelMarkers.add(modelMarker);
                     mech.addFrameMarker(modelMarker);
                  }
               }
            }
         } catch (Exception e) {
            System.out.println(e.getMessage());
            System.out.println("unable to add markers for muscle, removing: "
                  + info.fullName);
            muscleList.remove(k);
            k--; // update index to reflect purged kth muscle
            continue;
         }
      }

      addIncisorMarker(frameMarkerFiles[0].getAbsolutePath());
      addCondyleMarkers(frameMarkerFiles[0].getAbsolutePath());
      addBiteMarkers(frameMarkerFiles[0].getAbsolutePath());

   }

   public void addBones() {
      String meshFile = "";
      String[] filename;
      File[] meshFiles = selectFiles ();
      this.workDir = meshFiles[0].getParent();

      if (meshFiles.length == 3) {
         for (int i = 0; i < meshFiles.length; i++) {
               meshFile = meshFiles[i].toString ();
               if (meshFile.contains("Max")) {
                   System.out.println("Loading: " + meshFile);
                   skull = addBody("skull", meshFile, /* dynamic */false);
                   transformFrame(skull);
               } else if (meshFile.contains("Hyoid")) {
                   System.out.println("Loading: " + meshFile);
                   hyoid = addBody( "hyoid", meshFile, /* dynamic */false);
                   transformFrame(hyoid);
               } else if (meshFile.contains("Mand")) {
                   System.out.println("Loading: " + meshFile);
                   mandible = addBody("jaw", meshFile, /* dynamic */false);
                   transformFrame(mandible);
               }
         }
      }
      else {
         JOptionPane.showMessageDialog (
            null,
            "A Skull, Mandible, and Hyoid mesh must be selected to build the model");
         addBones();
      }

      // Once done adding bone attach mand frame and hyoid to max frame.
      // Transfrom skull frame to world coordinate origin
      // transformToWorld(skull);
      
   }

   public void transformFrame(RigidBody body) {
      // Compute centroid of bodies mesh and set translation vector to its value
      Vector3d centroid = new Vector3d ();
      body.getSurfaceMesh ().computeCentroid (centroid);
      RigidTransform3d XComToBody = new RigidTransform3d ();
      XComToBody.p.set (centroid);

      // Get transform matrix of bodies mesh and set it to XBodyToWorld
      RigidTransform3d XBodyToWorld = new RigidTransform3d ();
      body.getPose (XBodyToWorld);

      // Calculate transfrom matrix from XComToWorld
      RigidTransform3d XComToWorld = new RigidTransform3d ();
      XComToWorld.mul (XBodyToWorld, XComToBody);
      body.setPose (XComToWorld);

      // Transform mesh to Com and update surface mesh of body
      RigidTransform3d XMeshToCom = new RigidTransform3d ();
      PolygonalMesh mesh = body.getSurfaceMesh ();
      XMeshToCom.invert (XComToWorld);
      mesh.transform (XMeshToCom);
      body.setSurfaceMesh (mesh, null);
   }
  
   public RigidBody addBody (String name, String meshFileName, boolean dynamic) {

      RigidBody body = new RigidBody (name);
      try {
         PolygonalMesh mesh =
            (PolygonalMesh)GenericMeshReader.readMesh (meshFileName);
         ;
         body.setSurfaceMesh (mesh, meshFileName);   
         body.setDynamic (dynamic);
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
      mech.addRigidBody (body);

      return mech.rigidBodies().get(name);
   }

   private File[] selectFiles () {
      File dir = new File(ArtisynthPath.getHomeDir());
          
      int retval = 0;
      JFileChooser chooser = new JFileChooser (dir);

      chooser.setMultiSelectionEnabled (true);
      chooser.setFileHidingEnabled (true);
      retval = chooser.showOpenDialog (myFrame);

      File[] files = chooser.getSelectedFiles ();
      return ((retval == JFileChooser.APPROVE_OPTION) ? files : null);
   }

   private File[] selectFiles (String path) {
      File dir = new File(path);
          
      int retval = 0;
      JFileChooser chooser = new JFileChooser (dir);

      chooser.setMultiSelectionEnabled (true);
      chooser.setFileHidingEnabled (true);
      retval = chooser.showOpenDialog (myFrame);

      File[] files = chooser.getSelectedFiles ();
      return ((retval == JFileChooser.APPROVE_OPTION) ? files : null);
   }

   public ArrayList<String> readStringList (String filename)
     throws IOException {
     ArrayList<String> stringList = new ArrayList<String> ();
     ReaderTokenizer rtok = new ReaderTokenizer (new FileReader (filename));

     while (rtok.nextToken () != ReaderTokenizer.TT_EOF) {
        if (rtok.ttype != ReaderTokenizer.TT_WORD) {
           throw new IOException (
              "readMarkerList Expecting word, got " + rtok.tokenName ());
        }
        stringList.add (rtok.sval);
     }
     return stringList;
   }

   public HashMap<String,MuscleInfo> readMuscleInfo (String filename)
      throws IOException {
      HashMap<String,MuscleInfo> infoList =
         new LinkedHashMap<String,MuscleInfo> ();
      ReaderTokenizer rtok = new ReaderTokenizer (new FileReader (filename));

      while (rtok.nextToken () != ReaderTokenizer.TT_EOF) {
         MuscleInfo mi = new MuscleInfo ();
         mi.scan (rtok);
         infoList.put (mi.name, mi);
      }
      return infoList;
   }

   public void addIncisorMarker(String myPath) {
      FrameMarker myMarker;
      ComponentListView<RigidBody> myRigidBodies = mech.rigidBodies();
      try {
         // System.out.println(myPath);
         File csvFile = new File (myPath);
         // create BufferedReader and read data from csv
         BufferedReader csvReader;
         csvReader = new BufferedReader (new FileReader (csvFile.getAbsolutePath()));
         String row = new String ();
         while ((row = csvReader.readLine ()) != null) {
            String[] data = row.split (",");
            // System.out.println(data);
            if (data[0].equals("lowerincisor")) {
               Point3d pnt = new Point3d(Double.parseDouble(data[1]),Double.parseDouble(data[2]),Double.parseDouble(data[3]));
               myMarker = new FrameMarker();
               myMarker.setPosition(pnt);
               myMarker.setName(data[0]);
               myMarker.setFrame(myRigidBodies.get("jaw"));
               myMarker.setLocation(pnt);
               myMarker.setRefPos(pnt);
               mech.addFrameMarker(myMarker);
            }    
         }
         csvReader.close ();
      }
      catch (IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace ();
      }
      
   }

   public void addBiteMarkers(String myPath) {
      FrameMarker myMarker;
      ComponentListView<RigidBody> myRigidBodies = mech.rigidBodies();
      try {
         // System.out.println(myPath);
         File csvFile = new File (myPath);
         // create BufferedReader and read data from csv
         BufferedReader csvReader;
         csvReader = new BufferedReader (new FileReader (csvFile.getAbsolutePath()));
         String row = new String ();
         while ((row = csvReader.readLine ()) != null) {
            String[] data = row.split (",");
            // System.out.println(data);
            if (data[0].equals("rbite")) {
               Point3d pnt = new Point3d(Double.parseDouble(data[1]),Double.parseDouble(data[2]),Double.parseDouble(data[3]));
               myMarker = new FrameMarker();
               myMarker.setPosition(pnt);
               myMarker.setName(data[0]);
               myMarker.setFrame(myRigidBodies.get("jaw"));
               myMarker.setLocation(pnt);
               myMarker.setRefPos(pnt);
               mech.addFrameMarker(myMarker);
            }  
            if (data[0].equals("lbite")) {
               Point3d pnt = new Point3d(Double.parseDouble(data[1]),Double.parseDouble(data[2]),Double.parseDouble(data[3]));
               myMarker = new FrameMarker();
               myMarker.setPosition(pnt);
               myMarker.setName(data[0]);
               myMarker.setFrame(myRigidBodies.get("jaw"));
               myMarker.setLocation(pnt);
               myMarker.setRefPos(pnt);
               mech.addFrameMarker(myMarker);
            }    
         }
         csvReader.close ();
      }
      catch (IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace ();
      }
      
   }

   public void addCondyleMarkers(String myPath)
   {
      FrameMarker myMarker;
      ComponentListView<RigidBody> myRigidBodies = mech.rigidBodies();
      try {
         System.out.println(myPath);
         File csvFile = new File (myPath);
         // create BufferedReader and read data from csv
         BufferedReader csvReader;
         csvReader = new BufferedReader (new FileReader (csvFile.getAbsolutePath()));
         String row = new String ();
         while ((row = csvReader.readLine ()) != null) {
            String[] data = row.split (",");
            // System.out.println(data);
            if (data[0].equals("ltmj") || data[0].equals("rtmj")) {
               Point3d pnt = new Point3d(Double.parseDouble(data[1]),Double.parseDouble(data[2]),Double.parseDouble(data[3]));
               myMarker = new FrameMarker();
               myMarker.setPosition(pnt);
               myMarker.setName(data[0]);
               myMarker.setFrame(myRigidBodies.get("jaw"));
               myMarker.setLocation(pnt);
               myMarker.setRefPos(pnt);
               mech.addFrameMarker(myMarker);
            }
            if (data[0].equals("rTmjOuterAnterior") || data[0].equals("lTmjOuterAnterior") || data[0].equals("rTmjOuterPosterior") || data[0].equals("lTmjOuterPosterior")) {
               Point3d pnt = new Point3d(Double.parseDouble(data[1]),Double.parseDouble(data[2]),Double.parseDouble(data[3]));
               myMarker = new FrameMarker();
               myMarker.setPosition(pnt);
               myMarker.setName(data[0]);
               myMarker.setFrame(myRigidBodies.get("jaw"));
               myMarker.setLocation(pnt);
               myMarker.setRefPos(pnt);
               mech.addFrameMarker(myMarker);
            }
            if (data[0].equals("rTmjInnerAnterior") || data[0].equals("lTmjInnerAnterior") || data[0].equals("rTmjInnerPosterior") || data[0].equals("lTmjInnerPosterior")) {
               Point3d pnt = new Point3d(Double.parseDouble(data[1]),Double.parseDouble(data[2]),Double.parseDouble(data[3]));
               myMarker = new FrameMarker();
               myMarker.setPosition(pnt);
               myMarker.setName(data[0]);
               myMarker.setFrame(myRigidBodies.get("jaw"));
               myMarker.setLocation(pnt);
               myMarker.setRefPos(pnt);
               mech.addFrameMarker(myMarker);
            }

            if (data[0].equals("rCapsulePosterior") || data[0].equals("lCapsulePosterior"))  {
               Point3d pnt = new Point3d(Double.parseDouble(data[1]),Double.parseDouble(data[2]),Double.parseDouble(data[3]));
               myMarker = new FrameMarker();
               myMarker.setPosition(pnt);
               myMarker.setName(data[0]);
               myMarker.setFrame(myRigidBodies.get("skull"));
               myMarker.setLocation(pnt);
               myMarker.setRefPos(pnt);
               mech.addFrameMarker(myMarker);
            }
          }
         csvReader.close ();
      }
      catch (IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace ();
      }
      
   }

   public void setRenderProperties () {
      for (RigidBody body : mech.rigidBodies ()) {
         // Mesh render props
         RenderProps.setFaceColor (body, new Color(1f, 0.8f, 0.6f));
         RenderProps.setAlphaMode (body, PropertyMode.Inherited);
         RenderProps.setFaceStyle (body, FaceStyle.FRONT_AND_BACK);
         // Frame marker render props
         RenderProps.setPointStyle (mech.frameMarkers(), PointStyle.SPHERE);
         RenderProps.setPointColor(mech.frameMarkers(), Color.PINK);
         RenderProps.setPointRadius(mech.frameMarkers(),1);
         // Muscle props
         RenderProps.setLineColor(mech.axialSprings(), Color.WHITE);
         RenderProps.setShading(mech.axialSprings(), Shading.SMOOTH);
         RenderProps.setLineStyle(mech.axialSprings(), LineStyle.SPINDLE);
         RenderProps.setLineWidth(mech.axialSprings(), 3);
         RenderProps.setLineRadius(mech.axialSprings(), 2);
      }
   }
}