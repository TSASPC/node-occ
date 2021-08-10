#include "Face.h"
#include "Wire.h"
#include "Edge.h"
#include "Util.h"
#include "Mesh.h"
#include <string>
#include <sstream>

Face::~Face() {
	m_cacheMesh.Reset();
};

const TopoDS_Shape&  Face::shape() const
{
  return face();
}

void Face::setShape( const TopoDS_Shape& shape)
{
  m_face = TopoDS::Face(shape);
}

int Face::numWires()
{
  if(shape().IsNull()) return 0;
  TopTools_IndexedMapOfShape anIndices;
  TopExp::MapShapes(shape(), TopAbs_WIRE, anIndices);
  return anIndices.Extent();
}

NAN_METHOD(Face::getWires)
{
  Face* pThis = UNWRAP(Face);
  auto arr = extract_shapes_as_javascript_array(pThis,TopAbs_WIRE);
  info.GetReturnValue().Set(arr);
}

bool Face::fixShape()
{
  return true;
}

double Face::area()
{
  GProp_GProps prop;
  BRepGProp::SurfaceProperties(shape(), prop);
  return prop.Mass();
}

bool Face::hasMesh()
{
  TopLoc_Location loc;
  occHandle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(this->face(), loc);
  return triangulation.IsNull()?false:true;
}

std::vector<double> Face::inertia()
{
  std::vector<double> ret;
  GProp_GProps prop;
  BRepGProp::SurfaceProperties(this->shape(), prop);
  gp_Mat mat = prop.MatrixOfInertia();
  ret.push_back(mat(1,1)); // Ixx
  ret.push_back(mat(2,2)); // Iyy
  ret.push_back(mat(3,3)); // Izz
  ret.push_back(mat(1,2)); // Ixy
  ret.push_back(mat(1,3)); // Ixz
  ret.push_back(mat(2,3)); // Iyz
  return ret;
}


const gp_XYZ Face::centreOfMass() const
{

  GProp_GProps prop;
  BRepGProp::SurfaceProperties(this->shape(), prop);
  gp_Pnt cg = prop.CentreOfMass();

  return cg.Coord();
}

bool Face::isPlanar()
{

  Handle_Geom_Surface surf = BRep_Tool::Surface(this->m_face);
  GeomLib_IsPlanarSurface tool(surf);
  return tool.IsPlanar() ? true : false;
}

Nan::Persistent<v8::FunctionTemplate> Face::_template;

bool Face::buildFace(std::vector<Wire*>& wires)
{
  if (wires.size()==0) return false;

  // checling that all wires are closed
  for (uint32_t i = 0; i < wires.size(); i++) {
    if (!wires[i]->isClosed()) {
      Nan::ThrowError("Some of the wires are not closed");
      return false;
    }
  }

  try {
    const TopoDS_Wire& outerwire = wires[0]->wire();

    BRepBuilderAPI_MakeFace MF(outerwire);

    // add optional holes
    for (unsigned i = 1; i < wires.size(); i++) {
      const TopoDS_Wire& wire = wires[i]->wire();

      if (wire.Orientation() != outerwire.Orientation()) {
        MF.Add(TopoDS::Wire(wire.Reversed()));
      } else {
        MF.Add(wire);
      }
    }
    this->setShape(MF.Shape());

    // possible fix shape
    if (!this->fixShape()) {
      StdFail_NotDone::Raise("Shapes not valid");
    }

  }
  CATCH_AND_RETHROW_NO_RETURN("Failed to create a face");
  return true;
}

NAN_METHOD(Face::NewInstance) { _NewInstance<Face>(info); }

NAN_METHOD(Face::New)
{
  if (!info.IsConstructCall()) {
   return Nan::ThrowError(" use new occ.Face() to construct a Face");
  }

  Face* pThis = new Face();
  pThis->Wrap(info.This());
  pThis->InitNew(info);

  std::vector<Wire*> wires;
  extractArgumentList(info,wires);
  pThis->buildFace(wires);

  info.GetReturnValue().Set(info.This());

}

v8::Local<v8::Object> Face::Clone() const
{
  Face* obj = new Face();
  v8::Local<v8::Object> instance = makeInstance(_template);
  obj->Wrap(instance);
  obj->setShape(this->shape());
  return instance;
}

v8::Local<v8::Object> Face::NewInstance(const TopoDS_Face& face)
{
  Face* obj = new Face();
  v8::Local<v8::Object> instance = makeInstance(_template);
  obj->Wrap(instance);
  obj->setShape(face);
  return instance;
}

NAN_PROPERTY_GETTER(Face::_mesh)
{
  Face* pThis = UNWRAP(Face)

  if (pThis->m_cacheMesh.IsEmpty()) {
	  pThis->m_cacheMesh.Reset(pThis->createMesh(1,0.5, true));
  }
  info.GetReturnValue().Set(Nan::New(pThis->m_cacheMesh));
}

v8::Local<v8::Object> Face::createMesh(double factor, double angle, bool qualityNormals)
{
  Nan::EscapableHandleScope scope;
  const unsigned argc = 0;
  v8::Local<v8::Value> argv[1] = {  };
  
  v8::Local<v8::Object> theMesh = makeInstance(Mesh::_template);

  Mesh *mesh =  Mesh::Unwrap<Mesh>(theMesh);

  const TopoDS_Shape& shape = this->shape();

  try {

    TopLoc_Location loc;
    occHandle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(this->face(), loc);
    if (triangulation.IsNull()) {
       BRepMesh_IncrementalMesh MSH(shape,factor,Standard_True,angle,Standard_True);
    }


    // this code assume that the triangulation has been created
    // on the parent object
    mesh->extractFaceMesh(this->face(), qualityNormals);
    mesh->optimize();

  } CATCH_AND_RETHROW_NO_RETURN("Failed to mesh solid ");
  return scope.Escape(theMesh);
}

v8::Local<v8::String> Face::getType()
{
  Nan::EscapableHandleScope scope;
  v8::Local<v8::String> Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(), "NaN").ToLocalChecked();

  const TopoDS_Face& face = this->face();

  try {
    BRepAdaptor_Surface surface(face);
    switch(surface.GetType())
    {
      case (GeomAbs_Plane):
        Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(), "Plane").ToLocalChecked();
      break;
      case (GeomAbs_Cylinder):
        Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(),  "Cylinder").ToLocalChecked();
      break;
      case (GeomAbs_Cone):
        Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(),  "Cone").ToLocalChecked();
      break;
      case (GeomAbs_Sphere):
        Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(),  "Sphere").ToLocalChecked();
      break;
      case (GeomAbs_Torus):
        Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(),  "Torus").ToLocalChecked();
      break;
      case (GeomAbs_BezierSurface):
        Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(),  "BezierSurface").ToLocalChecked();
      break;
      case (GeomAbs_BSplineSurface):
        Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(),  "BSplineSurface").ToLocalChecked();
      break;
      case (GeomAbs_SurfaceOfRevolution):
        Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(),  "SurfaceOfRevolution").ToLocalChecked();
      break;
      case (GeomAbs_SurfaceOfExtrusion):
        Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(),  "SurfaceOfExtrusion").ToLocalChecked();
      break;
      case (GeomAbs_OffsetSurface):
        Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(),  "OffsetSurface").ToLocalChecked();
      break;
      case (GeomAbs_OtherSurface):
        Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(),  "OtherSurface").ToLocalChecked();
      break;
    }

  } CATCH_AND_RETHROW_NO_RETURN("Failed to find Surface Type ");
  return scope.Escape(Type);
}

v8::Local<v8::String> Face::getTypeJSON()
{
  Nan::EscapableHandleScope scope;
  v8::Local<v8::String> Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(), "{err:\"Not Supported\"}").ToLocalChecked();

  const TopoDS_Face& face = this->face();

  try {
    BRepAdaptor_Surface surface(face);
    std::stringstream s;
    switch(surface.GetType())
    {
      case (GeomAbs_Plane):
      {
        Standard_Real A,B,C,D;
        //A * X + B * Y + C * Z + D = 0.
        gp_Pln c = surface.Plane();
        const gp_Ax3& Pos = c.Position();
        const gp_XYZ& XDir = Pos.XDirection().XYZ();
        const gp_XYZ& YDir = Pos.YDirection().XYZ();
        const gp_XYZ& ZDir = Pos.Direction ().XYZ();
        const gp_XYZ& PLoc = Pos.Location  ().XYZ();
        s << "{\"class\":\"Plane\"";
        s << ",\"XDir\":{\"x\":" << XDir.X() <<",\"y\":" << XDir.Y() << ",\"z\":" << XDir.Z()<<"}";
        s << ",\"YDir\":{\"x\":" << YDir.X() <<",\"y\":" << YDir.Y() << ",\"z\":" << YDir.Z()<<"}";
        s << ",\"ZDir\":{\"x\":" << ZDir.X() <<",\"y\":" << ZDir.Y() << ",\"z\":" << ZDir.Z()<<"}";
        s << ",\"PLoc\":{\"x\":" << PLoc.X() <<",\"y\":" << PLoc.Y() << ",\"z\":" << PLoc.Z()<<"}";
        s << "}";
        Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(), s.str().c_str()).ToLocalChecked();
      
      break;
      }
      case (GeomAbs_Cylinder):
      {
        gp_Cylinder c = surface.Cylinder();
        const gp_Ax3& Pos = c.Position();
        const Standard_Real Radius = c.Radius();
        const gp_XYZ& XDir = Pos.XDirection().XYZ();
        const gp_XYZ& YDir = Pos.YDirection().XYZ();
        const gp_XYZ& ZDir = Pos.Direction ().XYZ();
        const gp_XYZ& PLoc = Pos.Location  ().XYZ();
        s << "{\"class\":\"Cylinder\"";
        s << ",\"Radius\":" << Radius;
        s << ",\"XDir\":{\"x\":" << XDir.X() <<",\"y\":" << XDir.Y() << ",\"z\":" << XDir.Z()<<"}";
        s << ",\"YDir\":{\"x\":" << YDir.X() <<",\"y\":" << YDir.Y() << ",\"z\":" << YDir.Z()<<"}";
        s << ",\"ZDir\":{\"x\":" << ZDir.X() <<",\"y\":" << ZDir.Y() << ",\"z\":" << ZDir.Z()<<"}";
        s << ",\"PLoc\":{\"x\":" << PLoc.X() <<",\"y\":" << PLoc.Y() << ",\"z\":" << PLoc.Z()<<"}";
        s << "}";
        // A1.X**2 + A2.Y**2 + A3.Z**2 + 2.(B1.X.Y + B2.X.Z + B3.Y.Z) +
        // 2.(C1.X + C2.Y + C3.Z) + D = 0.0
        Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(), s.str().c_str()).ToLocalChecked();
      
      break;
      }
      case (GeomAbs_Cone):
      {
        gp_Cone c = surface.Cone();
        const gp_Ax3& Pos = c.Position();
        const Standard_Real Radius = c.RefRadius();
        const Standard_Real SAngle = c.SemiAngle();
        const gp_XYZ& XDir = Pos.XDirection().XYZ();
        const gp_XYZ& YDir = Pos.YDirection().XYZ();
        const gp_XYZ& ZDir = Pos.Direction ().XYZ();
        const gp_XYZ& PLoc = Pos.Location  ().XYZ();
        s << "{\"class\":\"Cone\"";
        s << ",\"Radius\":" << Radius;
        s << ",\"SAngle\":" << SAngle;
        s << ",\"XDir\":{\"x\":" << XDir.X() <<",\"y\":" << XDir.Y() << ",\"z\":" << XDir.Z()<<"}";
        s << ",\"YDir\":{\"x\":" << YDir.X() <<",\"y\":" << YDir.Y() << ",\"z\":" << YDir.Z()<<"}";
        s << ",\"ZDir\":{\"x\":" << ZDir.X() <<",\"y\":" << ZDir.Y() << ",\"z\":" << ZDir.Z()<<"}";
        s << ",\"PLoc\":{\"x\":" << PLoc.X() <<",\"y\":" << PLoc.Y() << ",\"z\":" << PLoc.Z()<<"}";
        s << "}";
      //A1.X**2 + A2.Y**2 + A3.Z**2 + 2.(B1.X.Y + B2.X.Z + B3.Y.Z) + 2.(C1.X + C2.Y + C3.Z) + D = 0.0.
        Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(), s.str().c_str()).ToLocalChecked();
      break;
      }
      case (GeomAbs_Sphere):
      {
        
        gp_Sphere c = surface.Sphere();
        const gp_Ax3& Pos = c.Position();
        const Standard_Real Radius = c.Radius();
        const gp_XYZ& XDir = Pos.XDirection().XYZ();
        const gp_XYZ& YDir = Pos.YDirection().XYZ();
        const gp_XYZ& ZDir = Pos.Direction ().XYZ();
        const gp_XYZ& PLoc = Pos.Location  ().XYZ();
        s << "{\"class\":\"Sphere\"";
        s << ",\"Radius\":" << Radius;
        s << ",\"XDir\":{\"x\":" << XDir.X() <<",\"y\":" << XDir.Y() << ",\"z\":" << XDir.Z()<<"}";
        s << ",\"YDir\":{\"x\":" << YDir.X() <<",\"y\":" << YDir.Y() << ",\"z\":" << YDir.Z()<<"}";
        s << ",\"ZDir\":{\"x\":" << ZDir.X() <<",\"y\":" << ZDir.Y() << ",\"z\":" << ZDir.Z()<<"}";
        s << ",\"PLoc\":{\"x\":" << PLoc.X() <<",\"y\":" << PLoc.Y() << ",\"z\":" << PLoc.Z()<<"}";
        s << "}";
        //A1.X**2 + A2.Y**2 + A3.Z**2 + 2.(B1.X.Y + B2.X.Z + B3.Y.Z) +
        //2.(C1.X + C2.Y + C3.Z) + D = 0.0
        Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(), s.str().c_str()).ToLocalChecked();
      break;
      }
      case (GeomAbs_Torus):
      {
        
        gp_Torus c = surface.Torus();
        const gp_Ax3& Pos = c.Position();
        const Standard_Real R_major = c.MajorRadius();
        const Standard_Real R_minor = c.MinorRadius();
        const gp_XYZ& XDir = Pos.XDirection().XYZ();
        const gp_XYZ& YDir = Pos.YDirection().XYZ();
        const gp_XYZ& ZDir = Pos.Direction ().XYZ();
        const gp_XYZ& PLoc = Pos.Location  ().XYZ();
        s << "{\"class\":\"Torus\"";
        s << ",\"R_major\":" << R_major;
        s << ",\"R_minor\":" << R_minor;
        s << ",\"XDir\":{\"x\":" << XDir.X() <<",\"y\":" << XDir.Y() << ",\"z\":" << XDir.Z()<<"}";
        s << ",\"YDir\":{\"x\":" << YDir.X() <<",\"y\":" << YDir.Y() << ",\"z\":" << YDir.Z()<<"}";
        s << ",\"ZDir\":{\"x\":" << ZDir.X() <<",\"y\":" << ZDir.Y() << ",\"z\":" << ZDir.Z()<<"}";
        s << ",\"PLoc\":{\"x\":" << PLoc.X() <<",\"y\":" << PLoc.Y() << ",\"z\":" << PLoc.Z()<<"}";
        s << "}";
        /*
        Coef(1) * X^4 + Coef(2) * Y^4 + Coef(3) * Z^4 +
        Coef(4) * X^3 * Y + Coef(5) * X^3 * Z + Coef(6) * Y^3 * X +
        Coef(7) * Y^3 * Z + Coef(8) * Z^3 * X + Coef(9) * Z^3 * Y +
        Coef(10) * X^2 * Y^2 + Coef(11) * X^2 * Z^2 +
        Coef(12) * Y^2 * Z^2 + Coef(13) * X^2 * Y * Z +
        Coef(14) * X * Y^2 * Z + Coef(15) * X * Y * Z^2 +
        Coef(16) * X^3 + Coef(17) * Y^3 + Coef(18) * Z^3 + 
        Coef(19) * X^2 * Y + Coef(20) * X^2 * Z + Coef(21) * Y^2 * X +
        Coef(22) * Y^2 * Z + Coef(23) * Z^2 * X + Coef(24) * Z^2 * Y +
        Coef(25) * X * Y * Z +
        Coef(26) * X^2 + Coef(27) * Y^2 + Coef(28) * Z^2 +
        Coef(29) * X * Y + Coef(30) * X * Z + Coef(31) * Y * Z +
        Coef(32) * X + Coef(33) * Y + Coef(34) *  Z + 
        Coef(35) = 0.0
        */
        Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(), s.str().c_str()).ToLocalChecked();
      break;
      }
      case (GeomAbs_BezierSurface):
      {
        occHandle(Geom_BezierSurface) bezier = surface.Bezier();
        const TColgp_Array2OfPnt poles = bezier->Poles();
        TColStd_Array2OfReal Weights;
        bezier->Weights(Weights);
        s << "{\"class\":\"BezierSurface\",";
        s << "\"Poles\":[";
         for (Standard_Integer i = poles.LowerRow(); i <= poles.UpperRow(); i++){
          if (i != 1)
            s << ",";
          s << "[";
          for (Standard_Integer j = poles.LowerCol(); j <= poles.UpperCol(); j++){
            gp_Pnt pole = poles.Value(i,j);
            if (j!=1)
              s << ",";
            s << "{\"x\":" << pole.X() << ",\"y\":" << pole.Y() << ",\"z\":" << pole.Z() <<"}";
          }
          s << "]";
         }
         s << "],";
         s << "\"Weights\":[";
         for (Standard_Integer i = Weights.LowerRow(); i <= Weights.UpperRow(); i++){
          for (Standard_Integer j = Weights.LowerCol(); j <= Weights.UpperCol(); j++){
            if (j!=1)
              s << ",";
            s << Weights.Value(i,j);
          }
         }
         s << "]";
        s << "}";
        Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(),  s.str().c_str()).ToLocalChecked();
      break;
      }
      case (GeomAbs_BSplineSurface):
      {
         occHandle(Geom_BSplineSurface) bspline = surface.BSpline();
         //The Spline Data:
         s << "{\"class\":\"BSplineSurface\",";
         s << "\"Poles\":[";
         const TColgp_Array2OfPnt poles = bspline->Poles();
         for (Standard_Integer i = poles.LowerRow(); i <= poles.UpperRow(); i++){
          if (i != 1)
            s << ",";
          s << "[";
          for (Standard_Integer j = poles.LowerCol(); j <= poles.UpperCol(); j++){
            gp_Pnt pole = poles.Value(i,j);
            if (j!=1)
              s << ",";
            s << "{\"x\":" << pole.X() << ",\"y\":" << pole.Y() << ",\"z\":" << pole.Z() <<"}";
          }
          s << "]";
         
         }
         s << "],";
         TColStd_Array2OfReal Weights;
         bspline->Weights(Weights);
         s << "\"Weights\":[";
         
         for (Standard_Integer i = Weights.LowerRow(); i <= Weights.UpperRow(); i++){
           if (i != 1)
            s << ",";
          s << "[";
          for (Standard_Integer j = Weights.LowerCol(); j <= Weights.UpperCol(); j++){
            if (!(i==1 && j==1))
              s << ",";
            s <<Weights.Value(i,j);
          }
          s << "]";
         }
         s << "],";
         const TColStd_Array1OfReal UKnots = bspline->UKnotSequence();
         s << "\"UKnots\":[";
         for (Standard_Integer i = UKnots.Lower(); i <= UKnots.Upper(); i++){
           if (i != 1)
            s << ",";
          s << UKnots.Value(i);
         }
         s << "],";
         const TColStd_Array1OfReal VKnots = bspline->VKnotSequence();
         s << "\"VKnots\":[";
         for (Standard_Integer i = VKnots.Lower(); i <= VKnots.Upper(); i++){
           if (i != 1)
            s << ",";
          s << VKnots.Value(i);
         }
         s << "],";
         /*const TColStd_Array1OfInteger UMults = bspline->UMultiplicities();
         s << "\"UMults\":[";
         for (Standard_Integer i = UMults.Lower(); i <= UMults.Upper(); i++){
           if (i != 1)
            s << ",";
          s << UMults.Value(i);
         }
         s << "],";
         const TColStd_Array1OfInteger VMults = bspline->VMultiplicities();
         s << "\"VMults\":[";
         for (Standard_Integer i = VMults.Lower(); i <= VMults.Upper(); i++){
           if (i != 1)
            s << ",";
          s << VMults.Value(i);
         }
         s << "],";*/
         const Standard_Integer UDegree = bspline->UDegree();
         s << "\"UDegree\":"<< UDegree<<",";
         const Standard_Integer VDegree = bspline->VDegree();
         s << "\"VDegree\":"<< VDegree<<",";
         const Standard_Boolean UPeriodic = bspline->IsUPeriodic();
         s << "\"UPeriodic\":"<< UPeriodic<<",";
         const Standard_Boolean VPeriodic = bspline->IsVPeriodic();
         s << "\"VPeriodic\":"<< VPeriodic;
         s << "}";
        Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(),  s.str().c_str()).ToLocalChecked();
      break;
      }
      //These will need work in their JSON Rep.
      case (GeomAbs_SurfaceOfRevolution):
        //Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(),  "SurfaceOfRevolution Not Implemented");
      break;
      case (GeomAbs_SurfaceOfExtrusion):
       // Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(),  "SurfaceOfExtrusion Not Implemented");
      break;
      case (GeomAbs_OffsetSurface):
      //  Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(),  "OffsetSurface Not Implemented");
      break;
      case (GeomAbs_OtherSurface):
       // Type = v8::String::NewFromUtf8(v8::Isolate::GetCurrent(),  "OtherSurface Not Implemented");
      break;
    }

  } CATCH_AND_RETHROW_NO_RETURN("Failed to find Surface Type ");
  return scope.Escape(Type);
}


void Face::InitNew(_NAN_METHOD_ARGS)
{
  Base::InitNew(info);
  REXPOSE_READ_ONLY_PROPERTY_DOUBLE(Face,area);
  REXPOSE_READ_ONLY_PROPERTY_INTEGER(Face,numWires);
  REXPOSE_READ_ONLY_PROPERTY_DOUBLE(Face,area);
  REXPOSE_READ_ONLY_PROPERTY_BOOLEAN(Face,isPlanar);
  REXPOSE_READ_ONLY_PROPERTY_BOOLEAN(Face,hasMesh);
}

void Face::Init(v8::Local<v8::Object> target)
{
  // Prepare constructor template
  v8::Local<v8::FunctionTemplate> tpl = Nan::New<v8::FunctionTemplate>(Face::New);
  tpl->SetClassName(Nan::New("Face").ToLocalChecked());

  // object has one internal filed ( the C++ object)
  tpl->InstanceTemplate()->SetInternalFieldCount(1);

  _template.Reset(tpl);

  // Prototype
  v8::Local<v8::ObjectTemplate> proto = tpl->PrototypeTemplate();

  Base::InitProto(proto);

  EXPOSE_METHOD(Face,getWires);
  EXPOSE_METHOD(Face,createMesh);
  EXPOSE_METHOD(Face,getType);
  EXPOSE_METHOD(Face,getTypeJSON);
  EXPOSE_READ_ONLY_PROPERTY_INTEGER(Face,numWires);
  EXPOSE_READ_ONLY_PROPERTY_DOUBLE(Face,area);
  EXPOSE_READ_ONLY_PROPERTY_BOOLEAN(Face,isPlanar);
  EXPOSE_READ_ONLY_PROPERTY_BOOLEAN(Face,hasMesh);
  EXPOSE_READ_ONLY_PROPERTY(_mesh,mesh);
  EXPOSE_TEAROFF(Face,centreOfMass);
  Nan::Set(target,Nan::New("Face").ToLocalChecked(), Nan::GetFunction(tpl).ToLocalChecked());
}

NAN_METHOD(Face::createMesh)
{
  Face* pThis = UNWRAP(Face);
  v8::Local<v8::Object> mesh = pThis->createMesh(1,0.5,true);
  info.GetReturnValue().Set(mesh);
}
NAN_METHOD(Face::getType)
{
  Face* pThis = UNWRAP(Face);
  v8::Local<v8::String> Type = pThis->getType();
  info.GetReturnValue().Set(Type);
}
NAN_METHOD(Face::getTypeJSON)
{
  Face* pThis = UNWRAP(Face);
  v8::Local<v8::Object>  Type = v8::Local<v8::Object>::Cast(v8::JSON::Parse(v8::Isolate::GetCurrent()->GetCurrentContext(),  pThis->getTypeJSON()).ToLocalChecked());
  //v8::Local<v8::String>  Type = pThis->getTypeJSON();
  info.GetReturnValue().Set(Type);
}