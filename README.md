spc
===

Stratigraphy from Point Clouds

is a small project that puts together tools and methods for the extraction of stratigraphic information from point clouds, both from lidar and photogrammetry.

The code here (mainly badly written C++) is composed both of a library "libspc" and a set of tools for point cloud processing in the context of geology/stratigraphy. Both depend on PCL_ (and its natural dependencies, as VTK_, Boost_, CMake_, FLANN_, etc)

The project started with the aim of applying modern remote sensing techniques for extracting stratigraphic information from TLS collected data, in particular from intensity data. 

Final goal is to provide a set of command line tools for the processing of pointclouds both from TLS and from Photogrammetry for extracting stratigraphic logs to be used as numerical series i.e. in cyclostratigraphy.

<!---
Some numerical methods have been implemented:

- `Kernel Smoothing`_ (KS): used for target-to-sensor intensity loss detrending (we do not have any yet implemented numerical model for compensating the loss of intensity due to the distance and the scattering angle). KS is also used for the reconstruction of stratigraphic logs from the intensity value from TLS data, ora also from RGB informations , i.e. from photogrammetric models.
-->


contact me via github or by: luca.penasa@gmail.com

