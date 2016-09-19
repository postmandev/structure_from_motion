# Makefile modified to build on OSX 10.9

MATLAB_HOME=/Applications/MATLAB_R2014b.app
MEX=${MATLAB_HOME}/bin/mex
MEXCFLAGS=-I.. -largeArrayDims -O
INTFACESRCS=sba.c
INTFACESOBJ=sba.mexglx
LIBS=-L.. -lsba -llapack

dummy: $(INTFACESOBJ) projac.so

$(INTFACESOBJ): $(INTFACESRCS)
	$(MEX) $(MEXCFLAGS) $(INTFACESRCS) $(LIBS)

projac.so: projac.c
	gcc -Wall -fPIC -O3 -shared -o projac.so projac.c

clean:
	@rm -f $(INTFACESOBJ) projac.so

realclean cleanall: clean

depend:
	makedepend -f Makefile $(INTFACESRCS)

# DO NOT DELETE THIS LINE -- make depend depends on it.

