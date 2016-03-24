# UR5 drawing project 
ADrawingUserInput.m , AResolvedrate.m and A_UR5_IK_Icecream.m are the main files
that make the UR5 draw from user input (through mouse), a given outline (in this 
case a batman logo that has been traced using bwboundary in MATLAB), or an ice-cream
cone (using exact geometric equations) <br />
ur5fwdtrans gives the forward kinematics using DH parameters <br />
ur5fwdtwist gives the forward kinematics using twists (screw theory) <br />
ur5inv gives the inverse kinematics using DH <br />
Adjoi and ijack give the adjoint and inverse jacobian matrices for resolved rate control <br />
All other functions are used to support the forward and inverse kinematics <br />

###Note:
The main functions communicate with VREP simulator via a MATLAB API. If those sections of code are ommitted, 
then the package can be used to operate the real UR5 <br />
The subfolders have a PDF with a link to the videos <br />
The scene file for VREP has also been included
