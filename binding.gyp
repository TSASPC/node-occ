{
  "targets": [
    {
      "target_name": "occ",
      "cflags!"    : [ '-fno-exceptions' ],
      "cflags_cc!" : ['-fno-exceptions' ],

	  "msvs_settings": {

		'VCLinkerTool': {					
			'AdditionalLibraryDirectories': [
				#	"/projects/oce-0.11/Win64/lib/",
					"../deps/Win64/lib",
			],
			# 'AdditionalOptions': ["/FORCE",],						
		},	  
	  },
		
      "sources": [
			"src/Base.h",
			"src/Base.cc",
			"src/BoundingBox.h",
			"src/BoundingBox.cc",
			"src/Edge.h",
			"src/Edge.cc",
			"src/Face.h",
			"src/Face.cc",
			"src/GeometryBuilder.cc",
			"src/GeometryBuilder.h",
			"src/Mesh.h",
			"src/Mesh.cc",
			"src/NodeV8.h",
			"src/OCC.h",
			"src/Point3Wrap.h",
			"src/Point3Wrap.cc",
			"src/Shape.h",
			"src/Shape.cc",
			"src/ShapeFactory.h",
			"src/ShapeFactory.cc",
			"src/ShapeIterator.h",
			"src/ShapeIterator.cc",
			"src/Shell.h",
			"src/Shell.cc",
			"src/Solid.h",
			"src/Solid.cc",
			"src/Tools.h",
			"src/Tools.cc",
			"src/Transformation.h",
			"src/Transformation.cc",
			"src/Util.h",
			"src/Util.cc",
			"src/Vertex.h",
			"src/Vertex.cc",
			"src/Wire.h",
			"src/Wire.cc",
			"src/V8Wrapper.cc",
			],
			
	  "defines": [
		 "OCE_HAVE_IOMANIP",
		 "OCE_HAVE_IOSTREAM",
		 "OCE_HAVE_CLIMITS" 
	  ],

	  "library_dirs": [
	     "/usr/local/libs",
	  ],
	  "include_dirs": [  
           "/usr/local/include/oce",
           "../oce/inc",
		   "/home/pi/oce/inc",
           "/projects/oce-0.11/include/oce",
	       "/projet/oce/inc",			    
	  ],				

	  "libraries": [ 

		'-lTKAdvTools',
		'-lTKMath',
		'-lTKernel',
		'-lTKG2d',
		'-lTKG3d',
		'-lTKIGES',
		'-lTKSTEP',
		'-lTKFillet',
		'-lTKMesh',

		],
    }
  ]
}
