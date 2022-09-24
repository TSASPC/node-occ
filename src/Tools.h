#include "OCC.h"
#include "NodeV8.h"
//#define NO_CADEX
#if !defined(NO_CADEX) && !defined(CADEXCHANGER)
    #if __has_include("cadex/cadex_license.cxx") && __has_include("cadex/Base_String.hxx")
        #define CADEXCHANGER
    #else
        #define NO_CADEX
    #endif
#endif

#if !defined(NO_PAID) && !defined(PAID_OCCT)
    #if __has_include("products/OCCLicense_Activate.hxx")
        #define PAID_OCCT
        #pragma message "Paid Modules Found!"
        #if __has_include("products/XtControl_Reader.hxx")
            #pragma message "Paid Parasolid Module Found!"
            #define PARASOLID
        #endif
    #else
        #define NO_PAID
        #pragma message "No Paid Modules Found!"
    #endif
#endif

NAN_METHOD(writeSTEP);
NAN_METHOD(readSTEP);
NAN_METHOD(writeBREP);
NAN_METHOD(readBREP);
NAN_METHOD(writeSTL);
NAN_METHOD(writeGLTF);

#ifdef CADEXCHANGER
    NAN_METHOD(writeParasolid);
    NAN_METHOD(readParasolid);
    NAN_METHOD(convertParasolid2GLTF);
#else
    #ifdef PARASOLID
        //NAN_METHOD(writeParasolid);
        NAN_METHOD(readParasolid);
        //NAN_METHOD(convertParasolid2GLTF);
    #endif
#endif

