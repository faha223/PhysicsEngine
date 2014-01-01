PhysicsEngine
=============

An Object-Oriented Physics Engine that abstracts the PhysX (and maybe eventually Havok) API

Compilation:

  This project requires access to the Nvidia PhysX SDK version 3.3.0
  
  To compile, open the project in VS2013 and edit the Include and Lib folders to include the
  locations of your PhysX SDK Include and Lib folders.
  
  Then just build and run.
  
Additional Info:

  This project includes a driver.cpp which creates an SDL window in which to demonstrate the
  physics engine using OpenGL.
  
  This project was built using C++11 standards for timing and multithreading so it is
  exception safe and portable. 
  
Future Work:
  
  This is in its early alpha stages, and as such does not actually support any primitives yet.
  That is the next thing I need to implement. I plan to implement 3 types of bounding volumes:
    Convex Mesh,
    Capsule,
    and Sphere.
    
  These or any combination of these should be sufficient to model any real-world physics you
  would need.
    
  I also plan to write an alternate implementation based on the Havok Physics API for if I want
  to write a game that runs on AMD hardware as well as it does on Nvidia hardware.
