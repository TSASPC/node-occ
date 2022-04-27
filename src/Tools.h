#include "OCC.h"
#include "NodeV8.h"
#define NO_CADEX
#if !defined(NO_CADEX)
    #if __has_include("cadex/cadex_license.cxx") && __has_include("cadex/Base_String.hxx")
        #define CADEXCHANGER
    #endif
#endif

NAN_METHOD(writeSTEP);
NAN_METHOD(readSTEP);
NAN_METHOD(writeBREP);
NAN_METHOD(readBREP);
NAN_METHOD(writeSTL);
NAN_METHOD(writeGLTF);

#if !defined(NO_CADEX)
    #ifdef CADEXCHANGER
        NAN_METHOD(writeParasolid);
        NAN_METHOD(readParasolid);
        NAN_METHOD(convertParasolid2GLTF);
    #endif
#endif


