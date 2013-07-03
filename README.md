SPC
===

**Stratigraphy from Point Clouds**


Introduction
------------
SPC Is a small project that puts together tools and methods for the extraction of stratigraphic information from point clouds, both from lidar and photogrammetry.

The project started with the aim of applying modern remote sensing techniques for extracting stratigraphic information from TLS collected data, in particular from intensity data. 

Final goal is to provide a set of command line tools for the processing of pointclouds both from TLS and from Photogrammetry for extracting stratigraphic logs to be used as numerical series i.e. in cyclostratigraphy.

What is inside
--------------
1. A library **SPC** (actually a set of modular libs) with various aims, corresponding to folders:
	* __common__, some helpers and common algorithms
	* __geology__, methods that are strictly geology-related
	* __io__, io functions for data import/export
	* __methods__, more generic numerical methods as *kernel smoothing*, *interpolators*, *radial basis functions models*, *etc...* 
	* __time series__, Time series related objects, used by __methods__ for having a simple representation of time-series (*stratigraphic logs* in the specific case)
	* __micmac__, contains code interfacing with [micmac](http://www.micmac.ign.fr/) (aka culture3d) in particular for accessing some micmac features from python

    *Note that some of these modules can be disabled at compile time. For example for compiling the micmac part you would need to have a micmac (now culture3d) folder with compiled binaries inside somewhere*

2. **qGEO** a qt plugin for [cloudcompare](http://www.danielgm.net/cc/) that permits to do some of the operations done by the lib in a graphical context

3. **Tools** contains the code for the executables. Actually a set of tools for performing some operations on clouds

Dependencies
------------
The code here (mainly badly written C++) is composed both of some libraries "libspc_*" and a set of tools for point cloud processing in the context of geology/stratigraphy. Both depend on PCL (and its natural dependencies, as VTK, Boost, CMake, FLANN, etc)

Should compile fine on linux, but I never tested on win or mac. If you try let me know!

Note that the project is just started so sometimes the code may not compile as expected! In such a case contact me so we can see what is the problem :-)

I am making use of c++11 standard somewhere... so you need an updated compiler.

<!---
Some numerical methods have been implemented:

- `Kernel Smoothing`_ (KS): used for target-to-sensor intensity loss detrending (we do not have any yet implemented numerical model for compensating the loss of intensity due to the distance and the scattering angle). KS is also used for the reconstruction of stratigraphic logs from the intensity value from TLS data, ora also from RGB informations , i.e. from photogrammetric models.
-->

Looking for docs?
-----------------

A fully-featured project page with docs etc is in preparation, and will be accessible from github project pages, [here](http://luca-penasa.github.io/spc/ "SPC Project Page")


contact me via github or by: luca.penasa@gmail.com

