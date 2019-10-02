%module micmac
%{ 
    #define SWIG_FILE_WITH_INIT
    #include <general/ptxd.h>
	
%}
%include "numpy.i"
%init %{
import_array();
%}

%include <general/ptxd.h>
