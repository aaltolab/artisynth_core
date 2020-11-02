#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include "tetgenJNI.h"

int main (int argc, char**argv) {


   double coords[] = {
     1.5, 1, 0.5,
     -1.5, 1, 0.5,
     -1.5, -1, 0.5,
     1.5, -1, 0.5,
     1.5, 1, -0.5,
     -1.5, 1, -0.5,
     -1.5, -1, -0.5,
     1.5, -1, -0.5
   };

   double coords2[] = {
     21.0, 0.0, 0.0,
     0.0, 21.0, 0.0,
     0.0, 0.0, 0.0,
     18.0, 2.0, 6.0,
     1.0, 18.0, 5.0,
     2.0, 1.0, 3.0,
     14.0, 3.0, 10.0,
     4.0, 14.0, 14.0,
     3.0, 4.0, 10.0,
     10.0, 6.0, 12.0,
     5.0, 10.0, 15.0,
   };

   double coords3[] = {
0.029999999999999995, 0.0, 0.014999999999999998,
0.019999999999999997, 0.0, 0.006666666666666665,
0.009999999999999998, 0.0, 0.0016666666666666663,
-3.469446951953614E-18, 0.0, 2.006177025403371E-34,
-0.010000000000000002, 0.0, 0.0016666666666666676,
-0.019999999999999997, 0.0, 0.006666666666666665,
//-0.029999999999999995, 0.0, 0.014999999999999998,

0.029999999999999995, 0.009999999999999998, 0.016666666666666666,
0.019999999999999997, 0.009999999999999998, 0.008333333333333331,
0.009999999999999998, 0.009999999999999998, 0.0033333333333333327,
-3.469446951953614E-18, 0.009999999999999998, 0.0016666666666666663,
-0.010000000000000002, 0.009999999999999998, 0.003333333333333334,
-0.019999999999999997, 0.009999999999999998, 0.008333333333333331,
//-0.029999999999999995, 0.009999999999999998, 0.016666666666666666,

0.029999999999999995, 0.0, 0.03333333333333333,
0.019999999999999997, 0.0, 0.03333333333333333,
0.009999999999999998, 0.0, 0.03333333333333333,
-3.469446951953614E-18, 0.0, 0.03333333333333333,
-0.010000000000000002, 0.0, 0.03333333333333333,
-0.019999999999999997, 0.0, 0.03333333333333333,
//-0.029999999999999995, 0.0, 0.03333333333333333,

0.029999999999999995, 0.009999999999999998, 0.03333333333333333,
0.019999999999999997, 0.009999999999999998, 0.03333333333333333,
0.009999999999999998, 0.009999999999999998, 0.03333333333333333,
-3.469446951953614E-18, 0.009999999999999998, 0.03333333333333333,
-0.010000000000000002, 0.009999999999999998, 0.03333333333333333,
-0.019999999999999997, 0.009999999999999998, 0.03333333333333333,
//-0.029999999999999995, 0.009999999999999998, 0.03333333333333333
   };

   double coords4[] = {
     0x1.eb851eb851eb7p-6, 0x0p+0, 0x1.eb851eb851eb7p-7,
     0x1.47ae147ae147ap-6, 0x0p+0, 0x1.b4e81b4e81b4dp-8,
     0x1.47ae147ae147ap-7, 0x0p+0, 0x1.b4e81b4e81b4dp-10,
      -0x1p-58, 0x0p+0, 0x1.0aaaaaaaaaaabp-112,
      -0x1.47ae147ae147cp-7, 0x0p+0, 0x1.b4e81b4e81b53p-10,
      -0x1.47ae147ae147ap-6, 0x0p+0, 0x1.b4e81b4e81b4dp-8,
     // -0x1.eb851eb851eb7p-6, 0x0p+0, 0x1.eb851eb851eb7p-7,
     0x1.eb851eb851eb7p-6, 0x1.47ae147ae147ap-7, 0x1.1111111111111p-6,
     0x1.47ae147ae147ap-6, 0x1.47ae147ae147ap-7, 0x1.111111111111p-7,
     0x1.47ae147ae147ap-7, 0x1.47ae147ae147ap-7, 0x1.b4e81b4e81b4dp-9,
      -0x1p-58, 0x1.47ae147ae147ap-7, 0x1.b4e81b4e81b4dp-10,
      -0x1.47ae147ae147cp-7, 0x1.47ae147ae147ap-7, 0x1.b4e81b4e81b5p-9,
      -0x1.47ae147ae147ap-6, 0x1.47ae147ae147ap-7, 0x1.111111111111p-7,
     // -0x1.eb851eb851eb7p-6, 0x1.47ae147ae147ap-7, 0x1.1111111111111p-6,

     0x1.eb851eb851eb7p-6, 0x0p+0, 0x1.1111111111111p-5,
     0x1.47ae147ae147ap-6, 0x0p+0, 0x1.1111111111111p-5,
     0x1.47ae147ae147ap-7, 0x0p+0, 0x1.1111111111111p-5,
      -0x1p-58, 0x0p+0, 0x1.1111111111111p-5,
      -0x1.47ae147ae147cp-7, 0x0p+0, 0x1.1111111111111p-5,
      -0x1.47ae147ae147ap-6, 0x0p+0, 0x1.1111111111111p-5,
     // -0x1.eb851eb851eb7p-6, 0x0p+0, 0x1.1111111111111p-5,
     0x1.eb851eb851eb7p-6, 0x1.47ae147ae147ap-7, 0x1.1111111111111p-5,
     0x1.47ae147ae147ap-6, 0x1.47ae147ae147ap-7, 0x1.1111111111111p-5,
     0x1.47ae147ae147ap-7, 0x1.47ae147ae147ap-7, 0x1.1111111111111p-5,
     -0x1p-58, 0x1.47ae147ae147ap-7, 0x1.1111111111111p-5,
      -0x1.47ae147ae147cp-7, 0x1.47ae147ae147ap-7, 0x1.1111111111111p-5,
      -0x1.47ae147ae147ap-6, 0x1.47ae147ae147ap-7, 0x1.1111111111111p-5,
     // -0x1.eb851eb851eb7p-6, 0x1.47ae147ae147ap-7, 0x1.1111111111111p-5,
   };

   double coords5[] = {
0.03, 0.0, 0.014999999999999998,
0.02, 0.0, 0.006666666666666665,
0.01, 0.0, 0.0016666666666666663,
0, 0, 0,
-0.01, 0.0, 0.0016666666666666676,
-0.02, 0.0, 0.006666666666666665,
//-0.03, 0.0, 0.014999999999999998,

0.03, 0.01, 0.016666666666666666,
0.02, 0.01, 0.008333333333333331,
0.01, 0.01, 0.0033333333333333327,
0, 0.01, 0.0016666666666666663,
-0.01, 0.01, 0.003333333333333334,
-0.02, 0.01, 0.008333333333333331,
//-0.03, 0.01, 0.016666666666666666,

0.03, 0.0, 0.03333333333333333,
0.02, 0.0, 0.03333333333333333,
0.01, 0.0, 0.03333333333333333,
0, 0.0, 0.03333333333333333,
-0.01, 0.0, 0.03333333333333333,
-0.02, 0.0, 0.03333333333333333,
//-0.03, 0.0, 0.03333333333333333,

0.03, 0.01, 0.03333333333333333,
0.02, 0.01, 0.03333333333333333,
0.01, 0.01, 0.03333333333333333,
0, 0.01, 0.03333333333333333,
-0.01, 0.01, 0.03333333333333333,
-0.02, 0.01, 0.03333333333333333,
//-0.03, 0.01, 0.03333333333333333
   };


   int indices[] = {
     4, 0, 4, 5, 1, 
     4, 1, 5, 6, 2,
     4, 2, 6, 7, 3, 
     4, 3, 7, 4, 0, 
     4, 0, 1, 2, 3, 
     4, 7, 6, 5, 4, 
   };

   TetgenTessellator *tt = new TetgenTessellator();

   tt->buildFromMesh (coords, 8, indices, 6, 24, 0.0);
   assert (tt->out->numberofpoints == 8);
   assert (tt->out->numberoftetrahedra == 6);
   assert (tt->out->numberoftrifaces == 12);
   printf ("number of points=%d\n", tt->out->numberofpoints);
   printf ("number of tets=%d\n", tt->out->numberoftetrahedra);
   printf ("number of hull faces=%d\n", tt->out->numberoftrifaces);
	   
   tt->buildFromPoints (coords2, 11);
   assert (tt->out->numberofpoints == 11);
   assert (tt->out->numberoftetrahedra == 19);
   assert (tt->out->numberoftrifaces == 16);
   printf ("\n");
   printf ("number of points=%d\n", tt->out->numberofpoints);
   printf ("number of tets=%d\n", tt->out->numberoftetrahedra);
   printf ("number of hull faces=%d\n", tt->out->numberoftrifaces);

   tt->buildFromPoints (coords5, 24);
   assert (tt->out->numberofpoints == 24);
   assert (tt->out->numberoftetrahedra == 32);
   assert (tt->out->numberoftrifaces == 44);
   printf ("\n");
   printf ("number of points=%d\n", tt->out->numberofpoints);
   printf ("number of tets=%d\n", tt->out->numberoftetrahedra);
   printf ("number of hull faces=%d\n", tt->out->numberoftrifaces);

   printf ("\nPASSED\n\n");

   return 0;   
}