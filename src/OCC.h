// OpenCASCADE
#pragma once

#include <Standard_Version.hxx>

#include <Bnd_Box.hxx>

#include <BRep_Tool.hxx>
#include <BRepCheck_Analyzer.hxx>

//#include <BRepMesh.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepMesh_FastDiscret.hxx>

#include <BRepTools.hxx>
#include <BRepLib.hxx>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepPrimAPI_MakeWedge.hxx>

#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Common.hxx>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>

#include <BRepCheck_Analyzer.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepOffsetAPI_MakeDraft.hxx>
#include <BRepOffsetAPI_DraftAngle.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>


#include <BRepClass3d_SolidExplorer.hxx>
#include <BRepClass3d_SolidClassifier.hxx>

#include <ElCLib.hxx>

//xx #include <FSD_BinaryFile.hxx>

#include <Geom_BezierCurve.hxx>
#include <Geom_Circle.hxx>
#include <Geom_TrimmedCurve.hxx>

#include <GCPnts_UniformDeflection.hxx>

#include <gp.hxx>
#include <gp_Pnt.hxx>
#include <gp_Elips.hxx>
#include <gp_Sphere.hxx>
#include <gp_Circ.hxx>

#include <gce_MakeCirc.hxx>
#include <gce_MakePln.hxx>
#include <gce_MakeLin.hxx>

#include <GC_MakeSegment.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GC_MakeArcOfEllipse.hxx>
#include <GC_MakeLine.hxx>
#include <GC_MakePlane.hxx>
#include <GC_MakeTrimmedCone.hxx>




#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>

#include <GeomLProp_SLProps.hxx>
#include <GeomLib_IsPlanarSurface.hxx>

#include <GProp_GProps.hxx>


#include <Interface_Static.hxx>
#include <Interface_InterfaceModel.hxx>

#include "Message_ProgressIndicator.hxx"

//xx #include <MgtBRep.hxx>

#include <Poly_Triangulation.hxx>
#include <Poly_Connect.hxx>
#include <Poly_PolygonOnTriangulation.hxx>

#include <ShapeFix_ShapeTolerance.hxx>
#include <ShapeFix_Shape.hxx>
#include <ShapeFix_Solid.hxx>

#include <STEPControl_Writer.hxx>
#include <STEPControl_Reader.hxx>

//xx #include <ShapeSchema.hxx>

// #include <StdPrs_ToolShadedShape.hxx>

#include <Storage_Data.hxx>
#include <Storage_HSeqOfRoot.hxx>
#include <Storage_HSeqOfRoot.hxx>
#include <Storage_Root.hxx>

#include <StlAPI_Writer.hxx>
//xx #include <StlAPI_Reader.hxx>

#include <TColgp_Array1OfDir.hxx>
#include <TColgp_Array1OfPnt2d.hxx>

#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>

#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_MapIteratorOfMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <Transfer_TransientProcess.hxx>
#include <XSControl_WorkSession.hxx>
#include <XSControl_TransferReader.hxx>
#include <StepBasic_ProductDefinition.hxx>
#include <StepShape_TopologicalRepresentationItem.hxx>
#include <StepGeom_GeometricRepresentationItem.hxx>
#include <StepBasic_Product.hxx>
#include <StepBasic_ProductDefinitionFormation.hxx>
#include <StepRepr_NextAssemblyUsageOccurrence.hxx>
#include <StepRepr_ProductDefinitionShape.hxx>
#include <Interface_EntityIterator.hxx>
#include <TransferBRep.hxx>
//xx #include <TDF_Label.hxx>
//xx #include <TDF_ChildIDIterator.hxx>
//xx #include <TDataStd_Name.hxx>
//xx #include <TNaming_Builder.hxx>
#include <Interface_Graph.hxx>

#include <Poly_Polygon3D.hxx>
#include <BRep_TEdge.hxx>
#include <BRep_ListIteratorOfListOfCurveRepresentation.hxx>
#include <BRep_PolygonOnTriangulation.hxx>

#include <RWGltf_CafWriter.hxx>
#include <Message_ProgressRange.hxx>
#include <TColStd_IndexedDataMapOfStringString.hxx>
# include <TDocStd_Document.hxx>
#include <TDocStd_Application.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_Location.hxx>
#include <TDF_Label.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDataStd_Name.hxx>
#include <Quantity_Color.hxx>
#include <Interface_Static.hxx>
#include <Prs3d.hxx>
#include <Prs3d_Drawer.hxx>
#include <STEPCAFControl_Reader.hxx>


#include <TColStd_Array1OfReal.hxx>

//Surface Geometries
#include <gp_Pln.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Cone.hxx>
#include <gp_Sphere.hxx>
#include <gp_Torus.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierSurface.hxx>
//Curve Geometries
#include <gp_Lin.hxx>
#include <gp_Circ.hxx>
#include <gp_Elips.hxx>
#include <gp_Hypr.hxx>
#include <gp_Parab.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_OffsetCurve.hxx>

// Transformations:
#include <gp_Trsf.hxx>
#include <gp_Quaternion.hxx>

// Compatibility 6.5 and above
#if (OCC_VERSION_MAJOR * 10 + OCC_VERSION_MINOR )  < 66

// this makes some adjustemnts to make sure node-occ can be
// build with older version of OCC.
#include <TNaming_NamedShape.hxx>
#define BOPAlgo_Operation BOP_Operation
#define BOPAlgo_SECTION BOP_SECTION
#define BOPAlgo_COMMON  BOP_COMMON
#define BOPAlgo_FUSE    BOP_FUSE
#define BOPAlgo_CUT     BOP_CUT
#define BOPAlgo_CUT21   BOP_CUT21
#define BOPAlgo_UNKNOWN BOP_UNKNOWN
#define OUTER_SHELL(x)  BRepTools::OuterShell(x)
#else

#include <BRepClass3d.hxx>
#define OUTER_SHELL(x)  BRepClass3d::OuterShell(x)
#endif

#undef Handle
