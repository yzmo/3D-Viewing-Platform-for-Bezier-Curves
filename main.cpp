/**
 * Class description
 * This program displays and interactively manipulates Bezier and B-Spline Curves 
 * and 3D surface, and contains model construction,hidden surface removal in a polygon mesh, 
 * and 2-D/3D viewing platform integration
 * Yongzhao Mo
 */

/* constants */
#ifndef piValue
#define piValue            (3.14159265358979f)
#endif

#include <GL/glut.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <stdarg.h>
#include <time.h>	
#include <windows.h>
#include <stdlib.h>
#include "bmp.h"
#include <gl/glaux.h>

/*Define variables*/
GLint winWidth = 600, winHeight = 600;
GLfloat xeye = 0.0, yeye = 0.0, zeye = 8.0; // Viewing-coordinate origin.
GLfloat xref = 0.0, yref = 0.0, zref = 0.0; // Look-at point.
GLfloat Vx = 0.0, Vy = 1.0, Vz = 0.0; // View up vector.
GLfloat tx= 100.0,ty= 100.0,tz= 0.0;
GLfloat sx= 0.5,sy= 0.5,sz= 0.5;
GLint axis = 1;    //1 x; 2 y; 3 z
GLint mouseX, mouseY;
GLint startX, startY;
bool sflag = true;
bool autoRun = false;
bool showCoordinates = true;
bool isdrawing =false;
bool showLight = false;
bool shadding = false;
bool solarSystem = false;
bool drawingPoints = true;
bool drawBezierCurve = false;
bool solarFirst = false;
bool solid = false;
GLint drawType = 1; //1 rotate; 2 translate; 3 scale; 4 clipping near; 5 clipping far; 6 angle
                    //7 Ambiant Ka; 8 Ambiant B; 9 Point light Kb, 10 Point intensity p
                    //11 Solar  System; 12 X-axis Rotatate; 13 Mesh without light; 14 Solid with lighting
GLint type = 6;             //1 MC; 2 WC; 3 VC; 4 light; 5 Option; 6 A4subMenu
/* Set coordinate limits for the clipping window: */
GLfloat xwMin = -40.0, ywMin = -60.0, xwMax = 40.0, ywMax = 60.0;
/* Set positions for near and far clipping planes: */
GLfloat vangle = 40.0, dnear = 1.0, dfar = 20.0;
GLfloat rx = 1.0, ry = 0.0, rz = 0.0, s=0.8;
const GLfloat pi = 3.1415926;
GLfloat theta = -pi/180;
bool first = true; 
GLfloat cubeShade[6];
GLuint    textures[3];//Storage for three texture. 
GLfloat P = 0.8, B = 0.3, Ka = 1.0, Kd = 1.0;
GLfloat lx =1.5, ly = 1.5, lz=1.5; // point light position coordinates
GLuint   texture[3];        // Storage For Our Texture(Used in Load Texture)
GLint      LoadGLTextures();  //Function to Load the Images
GLfloat LMC[4][4] = {
{1.0,0.0,0.0, lx},
{0.0,1.0,0.0, ly},
{0.0,0.0,1.0, lz},
{0.0,0.0,0.0, 1.0}
}; //  this sets the light MC for the convenience of light transformation
GLfloat SMC[4][4] = {{1.0,0.0,0.0,0.0} ,{0.0,1.0,0.0,0.0},
{0.0,0.0,1.0,0.0},{0.0,0.0,0.0,1.0}}; 
GLfloat EMC[4][4] = {
{1.0,0.0,0.0, 6},
{0.0,1.0,0.0, 6},
{0.0,0.0,1.0, 6},
{0.0,0.0,0.0, 1.0}
};
GLfloat red = 1.0, green = 1.0, blue = 1.0; //color
GLfloat sunPosition[] = {0.0, 0.0, 0.0, 1.0};
GLfloat position[] = {lx, ly, lz, 1.0};
GLfloat diffuse[] = {red*P*Kd, green*P*Kd, blue*P*Kd, 1};
GLfloat ambient[] = {red*Ka*B, green*Ka*B, blue*Ka*B, 1};
// cube model
 GLfloat MC[4][4] = {{1.0,0.0,0.0,0.0} ,{0.0,1.0,0.0,0.0},
{0.0,0.0,1.0,0.0},{0.0,0.0,0.0,1.0}};  // Model coordinate system

GLfloat cube[8][3] = {{-1, -1, -1}, {-1, 1, -1}, {1, -1, -1}, {1, 1, -1},
{-1, -1, 1}, {-1, 1, 1}, {1, -1, 1}, {1, 1, 1} };  

GLint cube_faces[6][4] = { {0, 1, 3, 2}, {3, 7, 6, 2}, {7, 5, 4, 6},
{4, 5, 1, 0}, {5, 7, 3, 1}, {6, 4, 0, 2} };
GLint faces_copy[6][4] = { {0, 1, 3, 2}, {3, 7, 6, 2}, {7, 5, 4, 6},
{4, 5, 1, 0}, {5, 7, 3, 1}, {6, 4, 0, 2} };

GLint face_order[6] = {0,3,5,4,1,2};
GLint face_index[6] = {0,3,5,4,1,2};
GLfloat light_order[6];
GLfloat light_index[6];
GLint nCtrlPts = 0;  // number control points selected
GLint R = 45, S = 5, T = 5;  // Rotation variables
GLint Ptgen = 1, BCgen = 0, BCRotation = 0; // state variables for control point generation, 
                                            //Bezier curve generation, Bezier curve rotation
 class wcPt3D {
      public:                         //define structure to hold data
         GLfloat x, y, z;
   };
 wcPt3D ctrlPts[10]; // to store control points,
 wcPt3D setctrlPts[10] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},
 {0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
 wcPt3D RBM[73][26], RBC_face_norm[73][26];  // to store the mesh points and norm at each points  
   typedef float Matrix4x4 [4][4];
   typedef float Matrix4x1 [4][1];
   Matrix4x4 matRot;
   
   GLint nVerts= 8;
     wcPt3D verts[8] = {{-0.7, -0.7, -0.7}, {-0.7, 0.7, -0.7}, {0.7, -0.7, -0.7}, {0.7, 0.7, -0.7},
{-0.7, -0.7, 0.7}, {-0.7, 0.7, 0.7}, {0.7, -0.7, 0.7}, {0.7, 0.7, 0.7} };        // cube vertices
wcPt3D setVerts[8] = {{-0.7, -0.7, -0.7}, {-0.7, 0.7, -0.7}, {0.7, -0.7, -0.7}, {0.7, 0.7, -0.7},
{-0.7, -0.7, 0.7}, {-0.7, 0.7, 0.7}, {0.7, -0.7, 0.7}, {0.7, 0.7, 0.7} }; 
     wcPt3D p1 = {0.0,0.0,0.0};
     wcPt3D p2 = {1.0,0.0,0.0};
     wcPt3D p1Light = {0.0,0.0,0.0};
     wcPt3D p2Light = {1.0,0.0,0.0};
     wcPt3D p1Auto = {0.0,0.0,0.0};
     wcPt3D p2Auto = {1.0,0.0,0.0};
     wcPt3D p1Solar = {0.0,0.0,0.0};
     wcPt3D p2Solar = {1.0,0.0,0.0};
     wcPt3D originCopy = {0.0,0.0,0.0};
     
     /*This method draws sphere for sun and earth with texture*/
     void drawSphere(double r, int lats, int longs) {
       int i, j;
       for(i = 0; i <= lats; i++) {
           double lat0 = piValue * (-0.5 + (double) (i - 1) / lats);
          double z0  = sin(lat0);
          double zr0 =  cos(lat0);
    
          double lat1 = piValue * (-0.5 + (double) i / lats);
          double z1 = sin(lat1);
          double zr1 = cos(lat1);
    
          glBegin(GL_QUAD_STRIP);
          for(j = 0; j <= longs; j++) {
               double lng = 2 * piValue * (double) (j - 1) / longs;
               double x = cos(lng);
               double y = sin(lng);
    
               glNormal3f(x * zr0, y * zr0, z0);
               glTexCoord3d(x * zr0, y * zr0, z0);
              glVertex3f(x * zr0, y * zr0, z0);
              glNormal3f(x * zr1, y * zr1, z1);
              glTexCoord3d(x * zr1, y * zr1, z1);
              glVertex3f(x * zr1, y * zr1, z1);
          }
          glEnd();
      }
  }
/* Construct the 4 by 4 identity matrix. */
   void matrix4x4SetIdentity (Matrix4x4 matIdent4x4)
   {
      GLint row, col;

      for (row = 0; row < 4; row++)
         for (col = 0; col < 4 ; col++)
            matIdent4x4 [row][col] = (row == col);
   }

   /* Premultiply matrix m1 times matrix m2, store result in m2. */
   void matrix4x4PreMultiply (Matrix4x4 m1, Matrix4x4 m2)
   {
      GLint row, col;
      Matrix4x4 matTemp;

      for (row = 0; row < 4; row++)
         for (col = 0; col < 4 ; col++)
            matTemp [row][col] = m1 [row][0] * m2 [0][col] + m1 [row][1] *
                               m2 [1][col] + m1 [row][2] * m2 [2][col] +
                               m1 [row][3] * m2 [3][col];
      for (row = 0; row < 4; row++)
         for (col = 0; col < 4; col++)
            m2 [row][col] = matTemp [row][col];
   }
   /* Premultiply matrix m1 times matrix m2, store result in m2. */
   void matrix4x1PreMultiply (Matrix4x4 m1, Matrix4x1 m2)
   {
      GLint row;
      Matrix4x1 matTemp;

      for (row = 0; row < 4; row++)
            matTemp [row][0] = m1 [row][0] * m2 [0][0] + m1 [row][1] *
                               m2 [1][0] + m1 [row][2] * m2 [2][0] +
                               m1 [row][3] * m2 [3][0];
      for (row = 0; row < 4; row++)
            m2 [row][0] = matTemp [row][0];
   }
   
   /*Transpose matrix m to itself*/
   void MatrixTranspose(Matrix4x4 m)
   {
      GLint row, col;
      Matrix4x4 matTemp;

      for (row = 0; row < 4; row++)
         for (col = 0; col < 4 ; col++)
            matTemp [row][col] = m[col][row];
            
      for (row = 0; row < 4; row++)
         for (col = 0; col < 4; col++)
            m [row][col] = matTemp [row][col];
   }

   /*Translate matRot according to tx, ty, tz*/
   void translate3D (GLfloat tx, GLfloat ty, GLfloat tz)
   {
      Matrix4x4 matTransl3D;

      /*  Initialize translation matrix to identity.  */
      matrix4x4SetIdentity (matTransl3D);

      matTransl3D [0][3] = tx;
      matTransl3D [1][3] = ty;
      matTransl3D [2][3] = tz;

      /*  Concatenate translation matrix with matRot.  */
      matrix4x4PreMultiply (matTransl3D, matRot);
   }
   
   /*Rotate matRot accoring the p1 and p2 with radianAngle*/
   void rotate3D (wcPt3D p1, wcPt3D p2, GLfloat radianAngle)
   {
      Matrix4x4 matQuaternionRot;

      GLfloat axisVectLength = sqrt ((p2.x - p1.x) * (p2.x - p1.x) +
                           (p2.y - p1.y) * (p2.y - p1.y) +
                           (p2.z - p1.z) * (p2.z - p1.z));
      GLfloat cosA = cos (radianAngle);
      GLfloat oneC = 1 - cosA;
      GLfloat sinA = sin (radianAngle);
      GLfloat ux = (p2.x - p1.x) / axisVectLength;
      GLfloat uy = (p2.y - p1.y) / axisVectLength;
      GLfloat uz = (p2.z - p1.z) / axisVectLength;

      /*  Set up translation matrix for moving p1 to origin.  */
      translate3D (-p1.x, -p1.y, -p1.z);

      /*  Initialize matQuaternionRot to identity matrix.  */
      matrix4x4SetIdentity (matQuaternionRot);

      matQuaternionRot [0][0] = ux*ux*oneC + cosA;
      matQuaternionRot [0][1] = ux*uy*oneC - uz*sinA;
      matQuaternionRot [0][2] = ux*uz*oneC + uy*sinA;
      matQuaternionRot [1][0] = uy*ux*oneC + uz*sinA;
      matQuaternionRot [1][1] = uy*uy*oneC + cosA;
      matQuaternionRot [1][2] = uy*uz*oneC - ux*sinA;
      matQuaternionRot [2][0] = uz*ux*oneC - uy*sinA;
      matQuaternionRot [2][1] = uz*uy*oneC + ux*sinA;
      matQuaternionRot [2][2] = uz*uz*oneC + cosA;

      /*  Combine matQuaternionRot with translation matrix.  */
      matrix4x4PreMultiply (matQuaternionRot, matRot);

      /*  Set up inverse matTransl3D and concatenate with 
       *  product of previous two matrices.  
       */
      translate3D (p1.x, p1.y, p1.z);
   }

/* Procedure for generating a 3D scaling matrix. */
void scale3D (GLfloat sx,GLfloat sy,GLfloat sz, wcPt3D fixedPt)
{
        Matrix4x4 matScale3D;
        /* Initialize scaling matrix to identity. */
        matrix4x4SetIdentity (matScale3D);
        matScale3D [0][0] =sx;
        matScale3D [0][3] = (1 -sx) *fixedPt.x;
        matScale3D [1][1] =sy;
        matScale3D [1][3] = (1 -sy) *fixedPt.y;
        matScale3D [2][2] =sz;
        matScale3D [2][3] = (1 -sz) *fixedPt.z;
        /* Concatenate matScale3D with composite matrix. */
        matrix4x4PreMultiply (matScale3D,matRot);
}

/*Print matrix*/
void printMatrix(GLint a,GLint b,Matrix4x4 ma)
{
    GLint i = 0;
    GLint j = 0;
    for(i=0;i<a;i++)
    {
        for(j=0;j<b;j++)
        {
            printf ("%lf      ",ma[i][j]);          
        }
        printf ("\n");                                 
    }                    
     
}

/*Make objects moving to create animation*/
void move(int x) {
GLfloat speed = 0.001;
static int oldTime = clock(), newTime;
newTime = clock();
theta = (newTime - oldTime) * speed;
oldTime = newTime;

if (x == 1){
glutPostRedisplay();
glutTimerFunc(40, move, x);
}
}
// normalize MC coordinate system matrix.
void normalizeMC ()
{
GLfloat norm;

norm = sqrt(MC[0][0]*MC[0][0] + MC[1][0]*MC[1][0]+MC[2][0]*MC[2][0]);

MC[0][0] = MC[0][0]/norm;
MC[1][0] = MC[1][0]/norm;
MC[2][0] = MC[2][0]/norm;

MC[0][2] = MC[1][0]*MC[2][0]-MC[2][0]*MC[1][1];
MC[1][2] = MC[2][0]*MC[0][1]-MC[0][0]*MC[2][1];
MC[3][2] = MC[0][0]*MC[1][1]-MC[0][1]*MC[1][0];

norm = sqrt(MC[0][2]*MC[0][2] + MC[1][2]*MC[1][2]+MC[2][2]*MC[2][2]);

MC[0][2] = MC[0][2]/norm;
MC[1][2] = MC[1][2]/norm;
MC[2][2] = MC[2][2]/norm;

MC[0][1] = MC[1][2]*MC[2][0]-MC[1][0]*MC[2][2];
MC[1][1] = MC[2][2]*MC[0][0]-MC[2][0]*MC[0][2];
MC[2][1] = MC[0][2]*MC[1][0]-MC[0][0]*MC[1][2];


MC[3][0] = 0;
MC[3][1] = 0;
MC[3][2] = 0;
MC[3][3] = 1;
}

void draw2DFrame(){
// draw the 2D coordinate frame at the middle of the display window
   glLineWidth(2.5);
   glBegin(GL_LINES);
   glColor3f(0.8,0.0,0.0);
   glVertex3f(-1.5,0.0,0.0); 
   glVertex3f(1.5,0.0,0.0);    
   glColor3f(0.0,0.8,0.0);
   glVertex3f(0,-1.5,0);
   glVertex3f(0,1.5,0);    
   glEnd();
}

void drawCPts(){
// draw the list of control points
   glColor3f(1.0,0.0,0.0);
   glPointSize(3);
   glBegin(GL_POINTS);

   for (int i = 0;i < nCtrlPts;i++)
   {
       glVertex3f(ctrlPts[i].x, ctrlPts[i].y, ctrlPts[i].z);    
   }
   glEnd( );
   glPointSize(1);
}

void displayCPts(){
//glLoadIdentity();
glMatrixMode (GL_PROJECTION);
//gluOrtho2D (-winWidth/2, winWidth/2, -winHeight/2, winHeight/2); 
glClear (GL_COLOR_BUFFER_BIT);
glClearColor (0.0, 0.0, 0.0, 1.0); // Set display-window color to black
draw2DFrame();
drawCPts();   
}

/*Plot a point*/
void plotPoint (wcPt3D bezCurvePt)
{
    glBegin (GL_POINTS);
        glVertex2f (bezCurvePt.x, bezCurvePt.y);
    glEnd ( );
}
/*  Compute binomial coefficients C for given value of n.  */
void binomialCoeffs (GLint n, GLint * C)
{
   GLint k, j;

   for (k = 0;  k <= n;  k++) {
      /*  Compute n!/(k!(n - k)!).  */
      C [k] = 1;
      for (j = n;  j >= k + 1;  j--)
        C [k] *= j;
      for (j = n - k;  j >= 2;  j--)
        C [k] /= j;
   }
}

/*Compute Bezier points*/
void computeBezPt (GLfloat u, wcPt3D * bezPt, GLint nCtrlPts,
                    wcPt3D * ctrlPts, GLint * C)
{
   GLint k, n = nCtrlPts - 1;
   GLfloat bezBlendFcn;

   bezPt->x = bezPt->y = bezPt->z = 0.0;

   for (k = 0; k < nCtrlPts; k++) {
      bezBlendFcn = C [k] * pow (u, k) * pow (1 - u, n - k); //Compute blending functions
      bezPt->x += ctrlPts [k].x * bezBlendFcn;    //blend control points
      bezPt->y += ctrlPts [k].y * bezBlendFcn;
      bezPt->z += ctrlPts [k].z * bezBlendFcn;
   }
}

/*Draw the Bezier curve*/
void drawBezCurve (wcPt3D * ctrlPts, GLint nCtrlPts, GLint nBezCurvePts)
{
   wcPt3D bezCurvePt;
   GLfloat u;
   GLint *C, k;
   C = new GLint [nCtrlPts];

   binomialCoeffs (nCtrlPts - 1, C);
   for (k = 0;  k <= nBezCurvePts;  k++) {
      u = GLfloat (k) / GLfloat (nBezCurvePts);
      computeBezPt (u, &bezCurvePt, nCtrlPts, ctrlPts, C);
      plotPoint (bezCurvePt);
   }
   delete [ ] C;
}

void displayBezCurve(){
glClear (GL_COLOR_BUFFER_BIT);
glLoadIdentity();
glMatrixMode (GL_PROJECTION);
gluOrtho2D (-winWidth/2, winWidth/2, -winHeight/2, winHeight/2);
draw2DFrame();
displayCPts();
}

/*Compute the mesh points of the surface, and their norms*/
void RotateBezier (wcPt3D * ctrlPts, GLint nCtrlPts, GLint nBezCurvePts)
{
   wcPt3D bezCurvePt;
   GLfloat u;
   GLint *C, i,j,k;
   GLfloat theta=0;
   C = new GLint [nCtrlPts];

   binomialCoeffs (nCtrlPts - 1, C);
   for (k = 0;  k <= T;  k++)
   {
      u = GLfloat (k) / GLfloat (T);
      computeBezPt (u, &bezCurvePt, nCtrlPts, ctrlPts, C);
      RBM[0][k].x = bezCurvePt.x;
      RBM[0][k].y = bezCurvePt.y;
      RBM[0][k].z = bezCurvePt.z;
   }
   
   for (i = 1; i < R/S+2; i++) 
   {
          for (j = 0; j < nBezCurvePts+1; j++) 
          {
               RBM[i][j].x = RBM[0][j].x;
               RBM[i][j].y = RBM[0][j].y*cos(theta) + RBM[0][j].z*sin(theta);
               RBM[i][j].z = RBM[0][j].y*sin(theta) + RBM[0][j].z*cos(theta);
          }
          theta = theta + S*pi/180;
     }
     
     for (int i = 0; i < R/S+2; i++) 
     {
         for (j = 0; j < T+1; j++) 
         {
               GLfloat v1x = RBM[i][j+1].x - RBM[i][j].x;
               GLfloat v1y = RBM[i][j+1].y - RBM[i][j].y;
               GLfloat v1z = RBM[i][j+1].z - RBM[i][j].z;
               GLfloat v2x = RBM[i][j+1].x - RBM[i+1][j+1].x;
               GLfloat v2y = RBM[i][j+1].y - RBM[i+1][j+1].y;
               GLfloat v2z = RBM[i][j+1].z - RBM[i+1][j+1].z;
               GLfloat  vx =  v1y * v2z - v1z * v2y;
               GLfloat  vy =  v1z * v2x - v1x * v2z;
               GLfloat  vz =  v1x * v2y - v1y * v2x;
               GLfloat norm = sqrt(vx*vx+vy*vy+vz*vz);
               if(norm==0)norm = 1;
               RBC_face_norm[i][j].x =  vx/norm;      //cross product                       
               RBC_face_norm[i][j].y =  vy/norm;
               RBC_face_norm[i][j].z =  vz/norm;
          }
     }
              
     theta = 0;
   delete [ ] C;
}

/*Draw the surface*/
void drawRBC()
{
   int i,j;
   glColor3f(red, green, blue);
   for (i = 0; i < R/S+1; i++) 
   {
          for (j = 0; j < T; j++) 
          {
               if(drawType==3||drawType==4)glBegin(GL_LINE_LOOP);
               else    glBegin(GL_POLYGON);
                  glNormal3f(RBC_face_norm[i][j].x,RBC_face_norm[i][j].y,RBC_face_norm[i][j].z);
                  glVertex3f(RBM[i][j].x,RBM[i][j].y,RBM[i][j].z);
                  glVertex3f(RBM[i][j+1].x,RBM[i][j+1].y,RBM[i][j+1].z);
                  glVertex3f(RBM[i+1][j+1].x,RBM[i+1][j+1].y,RBM[i+1][j+1].z);
                  glVertex3f(RBM[i+1][j].x,RBM[i+1][j].y,RBM[i+1][j].z);
               glEnd();    
          }
   }
}
     
/*Draw the world coordinates and model coordinates*/
void drawAxis(int coordinateType)
{
  glLineWidth(3);
  glPushMatrix();
  float length = 2.0;
  glBegin(GL_LINES);
  if(coordinateType==1)  //World Coordinates
  {
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(0,0,0);
        glVertex3f(length,0,0);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3f(0,0,0);
        glVertex3f(0,length,0);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(0,0,0);
        glVertex3f(0,0,length);               
  }
  else if(coordinateType==2)  //Model Coordinates
  {
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f((verts[0].x+verts[7].x)/2,(verts[0].y+verts[7].y)/2,(verts[0].z+verts[7].z)/2);
        glVertex3f((verts[0].x+verts[7].x)/2+1,(verts[0].y+verts[7].y)/2,(verts[0].z+verts[7].z)/2);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3f((verts[0].x+verts[7].x)/2,(verts[0].y+verts[7].y)/2,(verts[0].z+verts[7].z)/2);
        glVertex3f((verts[0].x+verts[7].x)/2,(verts[0].y+verts[7].y)/2+1,(verts[0].z+verts[7].z)/2);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3f((verts[0].x+verts[7].x)/2,(verts[0].y+verts[7].y)/2,(verts[0].z+verts[7].z)/2);
        glVertex3f((verts[0].x+verts[7].x)/2,(verts[0].y+verts[7].y)/2,(verts[0].z+verts[7].z)/2+1);
  }       
  glEnd();
  glColor3f(red, green, blue);
  glPopMatrix();
  glLineWidth(2.5);
}
    
// selection sort function to sort the face order for hidden surface removal
void selectionSort(GLfloat data[], int count)
{
        int i, j, max, temp2;
        GLfloat temp1,temp3;
        for (i = 0; i < count - 1; i++) {
                // find the minimum
                max = i;
                for (j = i+1; j < count; j++) {
                    if (data[j] > data[max]) {
                        max = j;
                    }
                }
                // swap data[i] and data[max], face_order[i] and face_order[max]
                temp1 = data[i];
                data[i] = data[max];
                data[max] = temp1;
                
                temp2 = face_order[i];
                face_order[i] = face_order[max];
                face_order[max] = temp2;    
        }            
        
}
/*Sort the faces in decresaing order of the Z values of the faces
 *by Depth Sort Algorithm  
 */
void sortFaces(void)
{    
     //calculate each conter of the 6 faces
     int i,j;
     GLfloat order[6];
     GLfloat orderLight[6];
     for(i=0;i<6;i++){face_order[i]=face_index[i];}
     for (j = 0; j < 6; j++) 
     {
          i=face_order[j];
          GLfloat x = ((verts[cube_faces[i][0]].x + verts[cube_faces[i][1]].x +
          verts[cube_faces[i][2]].x + verts[cube_faces[i][3]].x)/4);
          GLfloat y = ((verts[cube_faces[i][0]].y + verts[cube_faces[i][1]].y +
          verts[cube_faces[i][2]].y + verts[cube_faces[i][3]].y)/4);
          GLfloat z = ((verts[cube_faces[i][0]].z + verts[cube_faces[i][1]].z +
          verts[cube_faces[i][2]].z + verts[cube_faces[i][3]].z)/4);
          Matrix4x1 MM = {{x},{y},{z},{1.0}};
          matrix4x1PreMultiply (MC,MM);
          x = MM[0][0];y=MM[1][0];z=MM[2][0];
          GLfloat temp = ((x-xeye)*(xref-xeye)+(y-yeye)*(yref-yeye)+(z-zeye)*(zref-zeye))/(
  sqrt((xref-xeye)*(xref-xeye)+(yref-yeye)*(yref-yeye)+(zref-zeye)*(zref-zeye)));
          order[j] = temp;
          
          //Calculate the cos value of the angle between the face normal vector and the vector from 
          //center point to source light
          if(showLight)                                                                       
          {     //printf("\n");                                                                                          
                GLfloat v1x =  verts[cube_faces[i][1]].x - verts[cube_faces[i][0]].x;
                GLfloat v1y =  verts[cube_faces[i][1]].y - verts[cube_faces[i][0]].y;
                GLfloat v1z =  verts[cube_faces[i][1]].z - verts[cube_faces[i][0]].z;  
                GLfloat v2x =  verts[cube_faces[i][2]].x - verts[cube_faces[i][1]].x;
                GLfloat v2y =  verts[cube_faces[i][2]].y - verts[cube_faces[i][1]].y;
                GLfloat v2z =  verts[cube_faces[i][2]].z - verts[cube_faces[i][1]].z;
                GLfloat  vx =  v1y * v2z - v1z * v2y;
                GLfloat  vy =  v1z * v2x - v1x * v2z;
                GLfloat  vz =  v1x * v2y - v1y * v2x;
                GLfloat nvx =  vx/sqrt(vx*vx+vy*vy+vz*vz);      //cross product                       
                GLfloat nvy =  vy/sqrt(vx*vx+vy*vy+vz*vz);
                GLfloat nvz =  vz/sqrt(vx*vx+vy*vy+vz*vz);
                Matrix4x1 MMM = {{nvx},{nvy},{nvz},{1.0}};
                matrix4x1PreMultiply (MC,MMM);
                nvx = MMM[0][0];nvy=MMM[1][0];nvz=MMM[2][0];
                
                GLfloat  ux =  lx-x;GLfloat  uy =  ly-y;GLfloat  uz =  lz-z; 
               
                ux = ux/sqrt(ux*ux+uy*uy+uz*uz);    //dot product
                uy = uy/sqrt(ux*ux+uy*uy+uz*uz);
                uz = uz/sqrt(ux*ux+uy*uy+uz*uz);                                                                

                GLfloat tempLight = (ux*nvx+uy*nvy+uz*nvz);
                
                if (tempLight > 0 )
                {
                   cubeShade[i] = B*Ka + P*Kd*tempLight; // shade of the face i
                }
                else
                cubeShade[i] = B*Ka;
          
                //printf ("x = %lf , y = %lf , z = %lf     cosO = %lf",x,y,z,tempLight);
          }
                
  }
  //printf("\n");
  selectionSort(order,6);
}       

void colorChosen(int i)
{
   if(i==0){red = 1.0;  green = 0.0;  blue = 0.0;}
   else if(i==1){red = 0.0;  green = 1.0;  blue = 0.0;}
   else if(i==2){red = 0.0;  green = 0.0;  blue = 1.0;} 
   else if(i==3){red = 1.0;  green = 1.0;  blue = 0.0;} 
   else if(i==4){red = 1.0;  green = 0.0;  blue = 1.0;} 
   else if(i==5){red = 0.0;  green = 1.0;  blue = 1.0;}         
}     

void drawCube(void);
static void (*drawObject)(void) = drawCube;
/*Draw a cube*/
void drawCube(void)
{
     if(solid){
     int i,j;
     for (j = 0; j < 6; j++)
     {
      i=face_order[j];colorChosen(i);glColor3f(red, green, blue);
      if(shadding)    //Calculate the shading               
      {
         glColor3f(cubeShade[i]*red, cubeShade[i]*green, cubeShade[i]*blue);     
      }
      glBegin(GL_QUADS);
      glVertex3f (verts[cube_faces[i][0]].x,verts[cube_faces[i][0]].y,verts[cube_faces[i][0]].z);
      glVertex3f (verts[cube_faces[i][1]].x,verts[cube_faces[i][1]].y,verts[cube_faces[i][1]].z);
      glVertex3f (verts[cube_faces[i][2]].x,verts[cube_faces[i][2]].y,verts[cube_faces[i][2]].z);
      glVertex3f (verts[cube_faces[i][3]].x,verts[cube_faces[i][3]].y,verts[cube_faces[i][3]].z);
      glEnd();
     }
     }
     else
     {
       glColor3f(red, green, blue);
       int i;
     for (i = 0; i < 6; i++) 
     {
      glBegin(GL_LINE_LOOP);
      glVertex3f (verts[cube_faces[i][0]].x,verts[cube_faces[i][0]].y,verts[cube_faces[i][0]].z);
      glVertex3f (verts[cube_faces[i][1]].x,verts[cube_faces[i][1]].y,verts[cube_faces[i][1]].z);
      glVertex3f (verts[cube_faces[i][2]].x,verts[cube_faces[i][2]].y,verts[cube_faces[i][2]].z);
      glVertex3f (verts[cube_faces[i][3]].x,verts[cube_faces[i][3]].y,verts[cube_faces[i][3]].z);
      glEnd();
     }    
     }    
}
/*Draw other objects*/
void drawTeapot(void) {
    glColor3f(red, green, blue);
	if(solid) glutSolidTeapot(1.0);
    else glutWireTeapot(1.0);
}
void drawSphere(void) {
     glColor3f(red, green, blue);
    if(solid) glutSolidSphere(1.0,20,20);
    else glutWireSphere(1.0,20,20);
}
void drawCone(void) {
     glColor3f(red, green, blue);
     if(solid) glutSolidCone(1.5,3.0,10,20);
    else glutWireCone(1.5, 3.0, 10, 20);
}
void drawTorus(void) {
     glColor3f(red, green, blue);
     if(solid) glutSolidTorus(0.5, 1.0, 10, 20);
    else glutWireTorus(0.5, 1.0, 10, 20);
}
void drawIcos(void) {
    glColor3f(red, green, blue);
    if(solid) glutSolidIcosahedron();
    else glutWireIcosahedron();

}
void drawDodecahedron(void)
{
      glColor3f(red, green, blue);
      if(solid) glutSolidDodecahedron();
      else glutWireDodecahedron();  
}     
    
void drawTetrahedron(void)
{
      glColor3f(red, green, blue);
      if(solid) glutSolidTetrahedron();
      else glutWireTetrahedron(); 
}     
              
void drawOctahedron(void)
{
      glColor3f(red, green, blue);
      if(solid) glutSolidOctahedron();
      else glutWireOctahedron(); 
}     

void drawIcosahedron(void)
{
      glColor3f(red, green, blue);
      if(solid) glutSolidIcosahedron();
      else glutWireIcosahedron(); 
}



/*Display function to display the drawing*/                     
void display(void)
{
     glLineWidth(2.5);
     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
     glMatrixMode(GL_PROJECTION);
     glLoadIdentity ( );
     gluPerspective(vangle, 1.0, dnear, dfar);
     glMatrixMode(GL_MODELVIEW);
     glLoadIdentity ( );
     gluLookAt(xeye, yeye, zeye, xref, yref, zref, Vx, Vy, Vz);
     
     if(type==6&&drawType == 1)        //draw the Bezier points
     {    
           displayCPts();
     }
     else if(type==6&&drawType == 2)   //draw the bezier curve
     {
           displayCPts();
           glColor3f(1.0, 0.0, 0.0);
           GLint nBezCurvePts = 1000;
           glPointSize(3);     
           drawBezCurve (ctrlPts, nCtrlPts, nBezCurvePts); 
     }             
     else
     {
         
     
         if(solarSystem==false)
         {
               if(showCoordinates)drawAxis(1);
               if(autoRun)             //cube rotates automatically with respect to the z axis of MC       
               {
                     matrix4x4SetIdentity(matRot);
                     GLfloat x0 = MC[0][3],y0 = MC[1][3],z0 = MC[2][3];            
                     rotate3D(p1Auto,p2Auto,theta);
            
                     matrix4x4PreMultiply (matRot, MC);
                     MC[0][3] = x0;
                     MC[1][3] = y0;
                     MC[2][3] = z0; 
                     sortFaces();     
                }
                
                
                if(showLight)
                {
                     position[0] = lx;position[0] = ly;position[0] = lz;
                     glEnable(GL_LIGHT0);
                     glEnable(GL_COLOR_MATERIAL);
                     glEnable(GL_DEPTH_TEST);
                     glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
                     glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
                     glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
                     glLightfv(GL_LIGHT0, GL_POSITION, position); 
                     glEnable(GL_LIGHTING);            
                }
                if(type==6&&drawType==5) glEnable(GL_LIGHTING);
                //draw the object
                glPushMatrix();
                MatrixTranspose(MC);
                glMultMatrixf(&MC[0][0]);
                MatrixTranspose(MC);
                
                if(showCoordinates)drawAxis(2);
                (*drawObject)();
                glPopMatrix();
                if(type==6&&drawType==5) glDisable(GL_LIGHTING);

                      if(showLight){
                      sortFaces();         
                      glPushMatrix();
                      MatrixTranspose(LMC);
                      glMultMatrixf(&LMC[0][0]);
                      MatrixTranspose(LMC);
                      glColor3f(1.0*(B*Ka + P*Kd*0.5), 0.0*(B*Ka + P*Kd*0.5), 0.0*(B*Ka + P*Kd*0.5));
                      glutSolidSphere( 0.1, 20, 20 );
                      glColor3f(red, green, blue);    
                      glPopMatrix();
                      glEnable(GL_LIGHTING);
                      }
                      

          }
          else    /*Draw sloar system*/    
          {    	                                             
                 glLineWidth(1.5);
                 matrix4x4SetIdentity(matRot);
                 GLfloat x0 = SMC[0][3],y0 = SMC[1][3],z0 = SMC[2][3];            
                 rotate3D(p1Solar,p2Solar,theta);   //The sun rotating at the center of WC
            
                 matrix4x4PreMultiply (matRot, SMC);
                 SMC[0][3] = x0;
                 SMC[1][3] = y0;
                 SMC[2][3] = z0;
                 
                 glEnable(GL_TEXTURE_2D);// Enable 2D texturing
                 glBindTexture(GL_TEXTURE_2D, textures[0]);
                 glPushMatrix();
                 MatrixTranspose(SMC);
                 glMultMatrixf(&SMC[0][0]);
                 MatrixTranspose(SMC);
	             glColor3f (1, 1, 1);
                 
                 if(solarFirst){matrix4x4SetIdentity(matRot);scale3D(0.6,0.6,0.6,originCopy);
                matrix4x4PreMultiply (matRot, SMC);}
                
		         drawSphere(0.2, 30, 30);
                 glPopMatrix();
                 glDisable(GL_TEXTURE_2D);
                 
                 glEnable(GL_LIGHT0);
                glEnable(GL_COLOR_MATERIAL);
                glEnable(GL_DEPTH_TEST);
                glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
                glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
                glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
                glLightfv(GL_LIGHT0, GL_POSITION, sunPosition);
                glEnable(GL_LIGHTING);
                 matrix4x4SetIdentity(matRot);
                 rotate3D(p1Solar,p2Solar,theta); //The earth rotates around the sun
                 matrix4x4PreMultiply (matRot, EMC); 
                 
                 if(solarFirst){matrix4x4SetIdentity(matRot);scale3D(0.2,0.2,0.2,originCopy);
                matrix4x4PreMultiply (matRot, EMC);solarFirst=false;}    
                 matrix4x4SetIdentity(matRot);
                 x0 = EMC[0][3];y0 = EMC[1][3];z0 = EMC[2][3];            
                 rotate3D(p1Solar,p2Solar,theta);  //The earth rotating its own z axis
            
                 matrix4x4PreMultiply (matRot, EMC);
                 EMC[0][3] = x0;
                 EMC[1][3] = y0;
                 EMC[2][3] = z0;
                 
                 glEnable(GL_TEXTURE_2D);// Enable 2D texturing
                 glBindTexture(GL_TEXTURE_2D, textures[1]);
                 
                 glPushMatrix();
                 MatrixTranspose(EMC);
                 glMultMatrixf(&EMC[0][0]);
                 MatrixTranspose(EMC);
                 
                 
                 drawSphere(0.2, 30, 30);
                 glPopMatrix();
                 glDisable(GL_TEXTURE_2D);
                 glDisable(GL_LIGHTING);
          }
     }    
     glutSwapBuffers();   // draw images in two buffer alternatively reduce flicker.
}

/*Drawing accoding to the mouse motion*/
void mouseMotion (GLint x, GLint y)
{
     isdrawing = true;
     sortFaces();
     if(isdrawing == true)
     {
         if(type==1) //Model Transformations
         {
              matrix4x4SetIdentity(matRot); 
              if(drawType == 1)
              {    
                  
                  GLfloat x0 = MC[0][3],y0 = MC[1][3],z0 = MC[2][3];    
                  if(x>mouseX)rotate3D(p1,p2,theta);
                  else rotate3D(p1,p2,-theta);
                  matrix4x4PreMultiply (matRot, MC);
                  MC[0][3] = x0;
                  MC[1][3] = y0;
                  MC[2][3] = z0; 
              }    
              else if(drawType == 3)
              {
                  wcPt3D origin = {0.0,0.0,0.0};
                  
                  if(sflag==true)
                  {     
                        if(x>mouseX)
                        {
                              scale3D(1.1,1.1,1.1,origin);
                        }
                        else
                        {
                              scale3D(0.9,0.9,0.9,origin); 
                              if(MC[0][0]*MC[1][1]*MC[2][2]<0.0000005)sflag=false;
                        }
                  }
                  else
                  {
                      if(x>mouseX)
                      {
                             scale3D(0.9,0.9,0.9,origin); 
                             if(MC[0][0]*MC[1][1]*MC[2][2]<0.0000005)sflag=true;      
                      }
                      else
                      {
                             scale3D(1.1,1.1,1.1,origin); 
                      }
                  } 
                  matrix4x4PreMultiply (matRot, MC);                    
                  
              }        
                        
         }
         else if(type==2)   //World Transformations
         {
               matrix4x4SetIdentity(matRot);
               if(drawType == 1)
               {
                     if(x>mouseX)rotate3D(p1,p2,theta); 
                     else rotate3D(p1,p2,-theta);
                     matrix4x4PreMultiply (matRot, MC);        
               }
               else if(drawType == 2)
               {
                     if(axis ==1)
                     {
                            if(x>mouseX){MC[0][3] += 0.05;}
                            else        {MC[0][3] -= 0.05;}
                     }
                     else if(axis==2)
                     {
                            if(winHeight-y>winHeight-mouseY){MC[1][3] += 0.05;}
                            else        {MC[1][3] -= 0.05;}
                     }
                     else if(axis==3)
                     {
                     if(x-mouseX+(winHeight-y)-(winHeight-mouseY)<0){MC[2][3] += 0.05;}
                            else        {MC[2][3] -= 0.05;}
                     } 
                     
                 
               }                        
         }
         else if(type == 3)  //View Transformations
         {
               if(drawType == 1)
               {
                   GLfloat xCopy=xeye, yCopy=yeye, zCopy=zeye;
                           
                   if(axis == 1)
                   {
                           yeye = yCopy * cos(theta) - zCopy * sin(theta);
                           zeye = yCopy * sin(theta) + zCopy * cos(theta);      
                   }
                   else if(axis == 2)
                   {
                           zeye = zCopy * cos(theta) - xCopy * sin(theta);
                           xeye = zCopy * sin(theta) + xCopy * cos(theta); 
                   }      
                   else if(axis == 3)
                   {
                           xeye = xCopy * cos(theta) - yCopy * sin(theta);
                           yeye = xCopy * sin(theta) + yCopy * cos(theta);    
                   }                    
               }
               else if(drawType == 2)
               {
                   if(axis == 1)
                   {
                           if(x>mouseX){xeye += 0.1;}
                           else        {xeye -= 0.1;}     
                   }
                   else if(axis == 2)
                   {
                           if(winHeight-y>winHeight-mouseY){yeye += 0.1;}
                           else        {yeye -= 0.1;}
                   }      
                   else if(axis == 3)
                   {
                           if(x-mouseX+(winHeight-y)-(winHeight-mouseY)<0){zeye += 0.1;}
                           else        {zeye -= 0.1;}     
                   }

               }
               else if(drawType == 4)
               {
                    if(x-mouseX+(winHeight-y)-(winHeight-mouseY)<0){dnear = dnear - 0.3;}
                    else        {dnear = dnear + 0.3;}
               }
               else if(drawType == 5)
               {
                    if(x-mouseX+(winHeight-y)-(winHeight-mouseY)<0){dfar = dfar + 0.3;}
                    else        {dfar = dfar - 0.3;}
               }
               else if(drawType == 6)
               {
                    if(x-mouseX+(winHeight-y)-(winHeight-mouseY)<0){vangle = vangle + 0.5;}
                    else        {vangle = vangle - 0.5;}
               }
               else if(drawType == 7)
               {
                    if(x>mouseX){Vx += 0.1;}
                    else        {Vx -= 0.1;} 
               }
               else if(drawType == 8)
               {
                    if(winHeight-y>winHeight-mouseY){Vy += 0.1;}
                    else        {Vy -= 0.1;}
               }
               else if(drawType == 9)
               {
                    if(x-mouseX+(winHeight-y)-(winHeight-mouseY)<0){Vz += 0.1;}
                    else        {Vz -= 0.1;} 
               }                       
                             
         }
         else if(type == 4)  //Light Transformations
         {
               matrix4x4SetIdentity(matRot);
               if(drawType == 1)
               {
                     if(x>mouseX)rotate3D(p1Light,p2Light,theta); 
                     else rotate3D(p1Light,p2Light,-theta); 
                     matrix4x4PreMultiply (matRot, LMC);     
               }
               else if(drawType == 2)
               {
                     if(axis ==1)
                     {
                            if(x>mouseX){LMC[0][3] += 0.05;}
                            else        {LMC[0][3] -= 0.05;}
                     }
                     else if(axis==2)
                     {
                            if(winHeight-y>winHeight-mouseY){LMC[1][3] += 0.05;}
                            else        {LMC[1][3] -= 0.05;}
                     }
                     else if(axis==3)
                     {
                     if(x-mouseX+(winHeight-y)-(winHeight-mouseY)<0){LMC[2][3] += 0.05;}
                            else        {LMC[2][3] -= 0.05;}
                     } 
                     
                 
               }
               else if(drawType == 7)
               {
                   if(x>mouseX){Ka += 0.05;}
                   else        {Ka -= 0.05;}
               
               }
               else if(drawType == 8)
               {
                   if(x>mouseX){B += 0.05;}
                   else        {B -= 0.05;}
               
               }
               else if(drawType == 9)
               {
                   if(x>mouseX){Kd += 0.05;}
                   else        {Kd -= 0.05;}
               
               }
               else if(drawType == 10)
               {
                   if(x>mouseX){P += 0.05;}
                   else        {P -= 0.05;}
               
               }
               
               lx = LMC[0][3];ly = LMC[1][3];lz = LMC[2][3];
               
         }
         if(type==6) //Model Transformations
         {
              matrix4x4SetIdentity(matRot); 
              if(drawType == 3||drawType == 4||drawType == 5)
              {    
                  
                  GLfloat x0 = MC[0][3],y0 = MC[1][3],z0 = MC[2][3];    
                  if(x>mouseX)rotate3D(p1,p2,theta);
                  else rotate3D(p1,p2,-theta);
                  matrix4x4PreMultiply (matRot, MC);
                  MC[0][3] = x0;
                  MC[1][3] = y0;
                  MC[2][3] = z0; 
              }                               
         }            
     } 
                 
     //normalizeMC ();
     mouseX = x;
     mouseY = y;
     glutPostRedisplay ( );
     
}

/*Record mouse action*/
void mouseAction(int btn, int state, int x, int y)
{
     if(btn==GLUT_LEFT_BUTTON && state == GLUT_UP)
      {
            first = true;
            isdrawing = false;
                                 
      }   
      else if(btn==GLUT_LEFT_BUTTON && state == GLUT_DOWN)
      {
            if(drawingPoints&&nCtrlPts<10&&!drawBezierCurve)
            {
                  GLdouble xPos, yPos, zPos;
                  GLdouble fx, fy, fz;
                  GLdouble modelViewMatrix[16];
                  GLdouble projectionMatrix[16];
                  GLint viewportMatrix[4];
                  glGetDoublev(GL_MODELVIEW_MATRIX, modelViewMatrix);
                  glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);
                  glGetIntegerv(GL_VIEWPORT, viewportMatrix);
                  gluProject(0.0f, 0.0f, 0.0f, modelViewMatrix, projectionMatrix, viewportMatrix, &xPos, &yPos, &zPos );
                  gluUnProject(x,winHeight-y,zPos,modelViewMatrix, projectionMatrix, viewportMatrix,&fx, &fy, &fz);
                  ctrlPts[nCtrlPts].x = (float)fx;
                  ctrlPts[nCtrlPts].y = (float)fy;
                  ctrlPts[nCtrlPts].z = (float)fz;
                  nCtrlPts = nCtrlPts + 1;                      
            } 
            first = false;  
            startX = x;
            startY = winHeight - y;
      
      }
	  glutPostRedisplay();
	  
}

void winReshapeFcn (GLint newWidth, GLint newHeight)
{
// Reset viewport and projection parameters 
glViewport (0, 0, newWidth, newHeight);
glMatrixMode (GL_PROJECTION);
glLoadIdentity ( );
gluPerspective( 60.0, (GLdouble)newWidth/(GLdouble)newHeight, 0.1, 40.0);
winWidth = newWidth;
winHeight = newHeight;
glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0.0, 0.0, 20.0, // eye at (0,0,20)
			  0.0, 0.0, 0.0, // lookat point
			  0.0, 1.0, 0.0); // up is in +ive y 
glOrtho(xwMin,xwMax,ywMin,ywMax,dnear,dfar);
glClear(GL_COLOR_BUFFER_BIT);
}

/*Initial*/
void init (void)
{
glClearColor (0.0, 0.0, 0.0, 1.0); // Set display-window color to black

glMatrixMode(GL_PROJECTION);
gluPerspective(vangle, 1.0, dnear, dfar);

glMatrixMode(GL_MODELVIEW);
gluLookAt(xeye, yeye, zeye, xref, yref, zref, Vx, Vy, Vz);

}

   void mainMenu (GLint allMode)
   {
       switch (allMode) {
           case 1:    glClearColor (0.0, 0.0, 0.0, 1.0); // Set display-window 
                                                         //color to black
                      glClear(GL_COLOR_BUFFER_BIT);
                      matrix4x4SetIdentity(MC); 
                      matrix4x4SetIdentity(matRot); 
                      for(int i =0;i<8;i++)                          //reset
                      {
                              verts[i] = setVerts[i];
                      }
                      drawType = 1;
                      type = 6;  sflag = true;
                      xeye = 0.0, yeye = 0.0, zeye = 8.0;
                      xref = 0.0; yref = 0.0; zref = 0.0; 
                      Vx = 0.0; Vy = 1.0;Vz = 0.0;
                      lx =1.5; ly = 1.5; lz=1.5;
                      vangle = 40.0; dnear = 1.0; dfar = 20.0;
                      rx = 1.0; ry = 0.0; rz = 0.0;
                      p2.x = 1.0; p2.y = 0.0; p2.z = 0.0;
                      red = 1.0; green = 0.0; blue = 0.0;
                      sortFaces();
                      theta = -pi/180;
                      R = 45; S = 5; T = 5;
                      solarSystem = false;
                      solarFirst = false;
                      nCtrlPts = 0;
                      drawBezierCurve = false;
                      drawingPoints = true;
                      for(int j =0;j<10;j++)                          //reset
                      {
                              ctrlPts[j] = setctrlPts[j];
                      }
                      glutPostRedisplay ( );   
                      break;
           case 2:   exit(0);
           
       }
       
   }
   
   void modelSubMenu (GLint modelOption)
   {
        drawType = 1;
        switch(modelOption)
        {
          case 1:
                p2.x = MC[0][0];p2.y = MC[1][0]; p2.z = MC[2][0]; break;  // rotate x  
          case 2:
                p2.x = MC[0][1];p2.y = MC[1][1]; p2.z = MC[2][1]; break;  // rotate y 
          case 3:
                p2.x = MC[0][2];p2.y = MC[1][2]; p2.z = MC[2][2]; break;  // rotate z     
          case 4: 
               drawType = 3;                                  //scale
               
        } 
        type = 1;       
   }
   
   void WCSubMenu (GLint WCOption)
   {
        drawType = 1;
        switch (WCOption)
        {
           case 1:
               p2.x = 1.0; p2.y = 0.0; p2.z = 0.0; break;    //rotate x
           case 2: 
               p2.x = 0.0; p2.y = 1.0; p2.z = 0.0; break;    //rotate y
           case 3: 
               p2.x = 0.0; p2.y = 0.0; p2.z = 1.0; break;    //rotate z
           case 4:
                                                                          // translate x
                axis = 1;
                drawType = 2;
                break;
           case 5:                                                        // translate y
                axis = 2;
                drawType = 2;
                break;
           case 6:                                                         // translate z
                axis = 3;
                drawType = 2;                      
        } 
        type = 2;        
   }
   void viewSubMenu (GLint viewOption)
   {
        drawType = 1;
        switch (viewOption)
        {
           case 1:
                axis = 1; break;  // rotate x  
           case 2:
                axis = 2; break;  // rotate y 
           case 3:
                axis = 3; break;  // rotate z 
           case 4:
                                                                          // translate x
                axis = 1;
                drawType = 2;
                break;
           case 5:
                axis = 2;                                                 // translate y
                drawType = 2;
                break;
           case 6:                                                        // translate z
                axis = 3;                                                 
                drawType = 2;
                break;
           case 7:
                drawType = 4;                                             // clipping near 
                break;
           case 8:
                drawType = 5;                                             // clipping far
                break;
           case 9:
                drawType = 6;                                             // angle
                break;
                 
        }    
        type = 3;    
   }
   
   void lightSubMenu (GLint lightOption)
   {
        drawType = 1;
        showLight = true;
        switch (lightOption)
        {
           case 1:
               p2Light.x = 1.0; p2Light.y = 0.0; p2Light.z = 0.0; break;    //rotate x
           case 2: 
               p2Light.x = 0.0; p2Light.y = 1.0; p2Light.z = 0.0; break;    //rotate y
           case 3: 
               p2Light.x = 0.0; p2Light.y = 0.0; p2Light.z = 1.0; break;    //rotate z
           case 4:
                                                                          // translate x
                axis = 1;
                drawType = 2;
                break;
           case 5:                                                        // translate y
                axis = 2;
                drawType = 2;
                break;
           case 6:                                                         // translate z
                axis = 3;
                drawType = 2; 
                break;
           case 7:  
                drawType = 7;  // Ka
                
                break;
           case 8:  
                drawType = 8;  // B
                
                break;
           case 9:   
                drawType = 9;  // Kd
                
                break;                    
           case 10:                                                       
                drawType = 10; // P
                                     
        } 
        
        type = 4;        
   }
   void optionSubMenu (GLint optionOption)
   {
        GLint dT = drawType;
        GLint tp = type;
        drawType = 1;
        type = 5;  
        switch (optionOption)
        {

           case 1: 
               showCoordinates = true;   //show coordinate
               drawType = dT;type = tp;
               glutPostRedisplay();
               break;
           case 2: 
               showCoordinates = false;  //hide coordinate
               drawType = dT;type = tp;
               glutPostRedisplay();
               break;
           case 3:
                showLight = true;         //show light
                drawType = dT;type = tp;
                glutPostRedisplay();
                break;
           case 4:                                                 
                showLight = false;        //hide light
                drawType = dT;type = tp;
                glutPostRedisplay();                     
        } 
              
   }
   void objectSubMenu (GLint objectOption)
   {
        switch (objectOption)
        {
               case 1: drawObject = drawCube;
                       break;
               case 2: drawObject = drawDodecahedron;
                       break;        
               case 3: drawObject = drawTetrahedron;
                       break;
               case 4: drawObject = drawOctahedron;
                       break;
               case 5: drawObject = drawIcosahedron;
                       break;
               case 6: drawObject = drawCone;
                       break;
               case 7: drawObject = drawTorus;
                       break;
               case 8: drawObject = drawTeapot;
                       break;
               case 9: drawObject = drawSphere;
                       break;   
               case 10: drawObject = drawRBC;
                       break;              
               
        }  
        glutPostRedisplay();
   }

   void colorsSubMenu (GLint colorsOption)
   {
        switch (colorsOption){
               case 1:                              
                    {red = 1.0;  green = 0.0;  blue = 0.0;} //Red
                    break;
               case 2:
                    {red = 0.0;  green = 1.0;  blue = 0.0;} //Green
                    break;
               case 3:
                    {red = 0.0;  green = 0.0;  blue = 1.0;} //Blue
                    break;
               case 4:
                    {red = 1.0;  green = 1.0;  blue = 1.0;} //White
                    break;
               case 5:
                    {red = 0.0;  green = 0.0;  blue = 0.0;} //Black
                    break;
               case 6:
                    {red = 1.0;  green = 1.0;  blue = 0.0;} //Yellow
                    break;
               case 7:
                    {red = 0.0;  green = 1.0;  blue = 1.0;} //Cyan
                    break;  
               case 8:
                    {red = 1.0;  green = 0.0;  blue = 1.0;} //Magenta
                                        
        }
        glutPostRedisplay(); 
   } 
   void rSubMenu (GLint rOption)
   {
        switch (rOption){
               case 1:                              
                    R = 45;
                    break;
               case 2:
                    R = 90;
                    break;
               case 3:
                    R = 135;
                    break;
               case 4:
                    R = 180;
                    break;
               case 5:
                    R = 225;
                    break;
               case 6:
                    R = 270;
                    break;
               case 7:
                    R = 360;
                    break;  
               case 8:
                    S = 5;
                    break;
               case 9:
                    S = 10;
                    break;    
               case 10:
                    S = 15;
                    break;
               case 11:
                    T = 5;
                    break;      
               case 12:
                    T = 10;
                    break;   
               case 13:
                    T = 15;
                    break; 
               case 14:
                    T = 20;
                    break;   
               case 15:
                    T = 25;
                    break;                         
                                        
        }
        //Recompute the surface values if the parameters changed
        if (nCtrlPts > 0 && (type == 6&&(drawType==3||drawType==4||drawType==5)))RotateBezier(ctrlPts, nCtrlPts, T);
        glutPostRedisplay();
   }
        
   void A4SubMenu (GLint A4Option)
   {
        GLint dT = drawType;
        GLint tp = type;
        drawType = 1;
        type = 6;  
        switch (A4Option)
        {
           case 1:                      //Control point selection
                drawingPoints = true;
                drawBezierCurve = false;
                drawType = 1;
                solarSystem = false;
                solarFirst = false;
                showLight = false;
                solid = false;
                glutPostRedisplay();
                break;
           case 2:                      //Bezier curve generation
                drawingPoints = false;
                drawBezierCurve = true;
                drawType = 2;
                solarFirst = false;
                solarSystem = false;
                showLight = false;
                solid = false;
                glutPostRedisplay();
                break;
           case 3:                      //X-axis Rotatate
                red = 0.0;green = 0.0;blue = 1.0;
                p2.x = MC[0][0];p2.y = MC[1][0]; p2.z = MC[2][0];  // rotate x  
                drawObject = drawRBC;
                showCoordinates = true;
                drawingPoints = false;
                drawType = 3;
                solid = false;
                drawBezierCurve = false;
                RotateBezier(ctrlPts, nCtrlPts, T);
                solarFirst = false;
                showLight = false;
                solarSystem = false;
                xeye = 3.0f; yeye = 3.0f; zeye = 7.0f;
                glutPostRedisplay();
                break;
           case 4:                     //Mesh without light
                
                scale3D(0.7,0.7,0.7,originCopy);
                matrix4x4PreMultiply (matRot, MC);
                red = 1.0;green = 0.0;blue = 0.0;
                p2.x = MC[0][0];p2.y = MC[1][0]; p2.z = MC[2][0];  // rotate x  
                drawObject = drawRBC;
                showCoordinates = true;
                solid = false;
                drawingPoints = false;
                drawType = 4;
                showLight = false;
                drawBezierCurve = false;
                RotateBezier(ctrlPts, nCtrlPts, T);
                solarFirst = false;
                solarSystem = false;
                xeye = 3.0f; yeye = 3.0f; zeye = 7.0f;
                glutPostRedisplay();
                break;
           case 5:                     //Solid with lighting
                red = 1.0;green = 0.0;blue = 0.0;
                p2.x = MC[0][0];p2.y = MC[1][0]; p2.z = MC[2][0];  // rotate x  
                drawObject = drawRBC;
                showCoordinates = true;
                drawingPoints = false;
                drawType = 5;
                solid = true;
                showLight = true;
                shadding = true;
                drawBezierCurve = false;
                
                solarFirst = false;
                solarSystem = false;
                RotateBezier(ctrlPts, nCtrlPts, T);
                xeye = 3.0f; yeye = 3.0f; zeye = 7.0f;
                glutPostRedisplay();
                break;
           case 6:                     //Improved Solar System
                drawingPoints = false;
                drawType = 6;
                solarFirst = true;
                solarSystem = true;
                p2Solar.x = SMC[0][2];p2Solar.y = SMC[1][2]; p2Solar.z = SMC[2][2];
                loadbmp(textures, "sun.bmp",0);   
                loadbmp(textures, "earth.bmp",1); 
                loadbmp(textures, "moon.bmp",2); 
                glutTimerFunc(40, move, 1);
                drawBezierCurve = false;
                glutPostRedisplay();  
        } 
                
   }
   
int main (int argc, char** argv)
{
         GLint subMenu1;
         GLint subMenu2;
         GLint subMenu3;
         GLint subMenu4;
         GLint subMenu5;
         GLint subMenu6;
         GLint subMenu7;
         GLint subMenu8;
         GLint rotationSubMenu;
         glutInit (&argc, argv);
         glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
         glutInitWindowPosition (100, 100);
         glutInitWindowSize (winWidth, winHeight);
         glutCreateWindow ("My 3D Viewing Platform for Bezier Curve (by Y. Mo   moxx4685)");
         init( );
         glutDisplayFunc (display);
         //glutReshapeFunc (winReshapeFcn);
         glutMotionFunc (mouseMotion);
         glutMouseFunc (mouseAction);
         
         
         rotationSubMenu = glutCreateMenu(rSubMenu);
               glutAddMenuEntry("R_45",1);
               glutAddMenuEntry("R_90",2);
               glutAddMenuEntry("R_135",3);
               glutAddMenuEntry("R_180",4);
               glutAddMenuEntry("R_225",5);
               glutAddMenuEntry("R_270",6);
               glutAddMenuEntry("R_360",7);
               glutAddMenuEntry("S_5",8);
               glutAddMenuEntry("S_10",9);
               glutAddMenuEntry("S_15",10);
               glutAddMenuEntry("T_5",11);
               glutAddMenuEntry("T_10",12);
               glutAddMenuEntry("T_15",13);
               glutAddMenuEntry("T_20",14);
               glutAddMenuEntry("T_25",15);
         subMenu1 = glutCreateMenu(A4SubMenu);
                glutAddMenuEntry("Control point selection",1);
                glutAddMenuEntry("Bezier curve generation",2);
                glutAddSubMenu("Rotation options",rotationSubMenu);    
                glutAddMenuEntry("X-axis Rotatate",3);
                glutAddMenuEntry("Mesh without light",4);
                glutAddMenuEntry("Solid with lighting",5);
                glutAddMenuEntry("A4P2:Improved Solar System",6);
         subMenu2 = glutCreateMenu(modelSubMenu);
                glutAddMenuEntry("Rotate x",1);
                glutAddMenuEntry("Rotate y",2);
                glutAddMenuEntry("Rotate z",3);
                glutAddMenuEntry("Scale",4);
         subMenu3 = glutCreateMenu(WCSubMenu);
                glutAddMenuEntry("Rotate x",1);
                glutAddMenuEntry("Rotate y",2);
                glutAddMenuEntry("Rotate z",3);
                glutAddMenuEntry("Translate x",4);
                glutAddMenuEntry("Translate y",5);
                glutAddMenuEntry("Translate z",6);
         subMenu4 = glutCreateMenu(viewSubMenu);
                glutAddMenuEntry("Rotate x",1);
                glutAddMenuEntry("Rotate y",2);
                glutAddMenuEntry("Rotate z",3);
                glutAddMenuEntry("Translate x",4);
                glutAddMenuEntry("Translate y",5);
                glutAddMenuEntry("Translate z",6);
                glutAddMenuEntry("Clipping Near",7);
                glutAddMenuEntry("Clipping Far",8);
                glutAddMenuEntry("Angle",9);        
         subMenu5 = glutCreateMenu(lightSubMenu);
                glutAddMenuEntry("Rotate x",1);
                glutAddMenuEntry("Rotate y",2);
                glutAddMenuEntry("Rotate z",3);
                glutAddMenuEntry("Translate x",4);
                glutAddMenuEntry("Translate y",5);
                glutAddMenuEntry("Translate z",6);
                glutAddMenuEntry("Ambient Ka",7);
                glutAddMenuEntry("Ambient B",8);
                glutAddMenuEntry("Point light Kd",9);
                glutAddMenuEntry("Point Intensity P",10);
         subMenu6 = glutCreateMenu(objectSubMenu);
                glutAddMenuEntry("MyCube",1);
                glutAddMenuEntry("Glut Dodecahedron",2);
                glutAddMenuEntry("Glut Tetrahedron",3);
                glutAddMenuEntry("Glut Octahedron",4);
                glutAddMenuEntry("Glut Icosahedron",5);
                glutAddMenuEntry("Glut Cone",6);
                glutAddMenuEntry("Glut Torus",7);
                glutAddMenuEntry("Glut Teapot",8);
                glutAddMenuEntry("Glut Sphere",9);    
                glutAddMenuEntry("Rotated Bezier Curve",10);   
         subMenu7 = glutCreateMenu(optionSubMenu);
                glutAddMenuEntry("Add Coordinate Frame",1);    
                glutAddMenuEntry("Remove Coordinate Frame",2);
                glutAddMenuEntry("Show Point Light",3);
                glutAddMenuEntry("Hide Point Light",4);
         subMenu8 = glutCreateMenu(colorsSubMenu);
                glutAddMenuEntry("Red",1);
                glutAddMenuEntry("Green",2);    
                glutAddMenuEntry("Blue",3);
                glutAddMenuEntry("White",4);
                glutAddMenuEntry("Black",5);
                glutAddMenuEntry("Yellow",6);
                glutAddMenuEntry("Cyan",7);
                glutAddMenuEntry("Magenta",8);         
                
         glutCreateMenu (mainMenu);      // Create main pop-up menu.
           glutAddSubMenu("A4",subMenu1);
           glutAddSubMenu("Model Transformations",subMenu2);
           glutAddSubMenu("WC Transformations",subMenu3);
           glutAddSubMenu("View Transformations",subMenu4);
           glutAddSubMenu("Light Transformation",subMenu5);
           glutAddSubMenu("GLUT Objects",subMenu6);
           glutAddSubMenu("View Options",subMenu7);
           glutAddSubMenu("Colors Options",subMenu8);
           glutAddMenuEntry("Reset",1);
           glutAddMenuEntry("Quit",2);
           
           
       //  Select menu option using right mouse button.  
       glutAttachMenu (GLUT_RIGHT_BUTTON);

       glutMainLoop ( );
}  







