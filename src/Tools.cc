#include "Tools.h"

#include "IFSelect_ReturnStatus.hxx"

#include "Shape.h"
#include "Solid.h"

#include <list>
#include <sstream>
#include "AsyncWorkerWithProgress.h"

//Checkout:
//https://docs.cadexchanger.com/sdk/exploring_2brepgeometry_2main_8cxx-example.html
#include <iostream>

#ifdef CADEXCHANGER 
  //Note you will need to put the Licence into the CADEX include Folder
  #include <cadex/cadex_license.cxx>

  #include <cadex/Base_String.hxx>
  #include <cadex/LicenseManager_Activate.h>
  #include <cadex/ModelData_Model.hxx>
  #include <cadex/ModelData_ModelWriter.hxx>
  #include <cadex/STEP_Reader.hxx>
  #include <cadex/STEP_ReaderParameters.hxx>
  #include <cadex/Para_Reader.hxx>
  #include <cadex/Para_Writer.hxx>
  #include <cadex/GLTF_Writer.hxx>
  #include <cadex/ModelData_ShapeConverter.hxx>
  #include <cadex/ModelData_Model.hxx>
  #include <cadex/ModelData_Shape.hxx>
  #include <cadex/ModelData_SceneGraphElement.hxx>
  
  using namespace cadex;
//SOLIDWORKS
//#include <cadex/SDL_Reader.hxx>
#endif
#ifdef PAID_OCCT
  //Note you will need to put the Licence into the CADEX include Folder
  #include <products/OCCLicense_Activate.hxx>
//SOLIDWORKS
//#include <cadex/SDL_Reader.hxx>
#endif
#ifdef PARASOLID 
  //Note you will need to put the Licence into the CADEX include Folder
  
  //#include <XSControl_Reader.hxx>
  #include <products/XtControl_Reader.hxx>
//SOLIDWORKS
//#include <cadex/SDL_Reader.hxx>
#endif

//
// ref : http://nikhilm.github.io/uvbook/threads.html
//
void extractShapes(v8::Local<v8::Value> value, std::list<Shape*>& shapes)
{
  if (value->IsArray()) {

    v8::Local<v8::Array> arr = v8::Local<v8::Array>::Cast(value);
    for (uint32_t i = 0; i < arr->Length(); i++) {
      auto elementI = Nan::Get(arr,i).ToLocalChecked();
      extractShapes(elementI, shapes);
    }
  }
  else if (value->IsObject()) {
    // it must be of type
    v8::Local<v8::Object> obj = Nan::To<v8::Object>(value).ToLocalChecked();
    if (IsInstanceOf<Solid>(obj)) {
      shapes.push_back(Nan::ObjectWrap::Unwrap<Shape>(obj));
    }

  }
}

static bool extractFileName(const v8::Local<v8::Value>& value, std::string& filename)
{
  // first argument is filename
  if (!value->IsString()) {
    return false;
  }
  Nan::Utf8String str(value);
  filename = *str;
  return true;
}

static bool extractCallback(const v8::Local<v8::Value>& value, v8::Local<v8::Function>& callback)
{
  if (!value->IsFunction()) {
    return false;
  }
  callback = Nan::To<v8::Function>(value).ToLocalChecked();
  return true;
}


NAN_METHOD(writeSTEP)
{
  std::string filename;
  if (!extractFileName(info[0], filename)) {
    return Nan::ThrowError("expecting a file name");
  }

  std::list<Shape*>  shapes;
  for (int i = 1; i < info.Length(); i++) {
    extractShapes(info[i], shapes);
  }

  if (shapes.size() == 0) {
    return info.GetReturnValue().Set(Nan::New<v8::Boolean>(false));
  }

  try {
    STEPControl_Writer writer;
    IFSelect_ReturnStatus status;

    //xx Interface_Static::SetCVal("xstep.cascade.unit","M");
    //xx Interface_Static::SetIVal("read.step.nonmanifold", 1);

    for (std::list<Shape*>::iterator it = shapes.begin(); it != shapes.end(); it++) {
      status = writer.Transfer((*it)->shape(), STEPControl_AsIs);
      if (status != IFSelect_RetDone) {
        return Nan::ThrowError("Failed to write STEP file");
      }
    }
    status = writer.Write(filename.c_str());
  } CATCH_AND_RETHROW("Failed to write STEP file ");

  info.GetReturnValue().Set(Nan::New<v8::Boolean>(true));
}

NAN_METHOD(writeBREP)
{
  std::string filename;
  if (!extractFileName(info[0], filename)) {
    return Nan::ThrowError("expecting a file name");
  }
  std::list<Shape*>  shapes;
  for (int i = 1; i < info.Length(); i++) {
    extractShapes(info[i], shapes);
  }
  if (shapes.size() == 0) {
    info.GetReturnValue().Set(Nan::New<v8::Boolean>(false));
    return;
  }

  try {
    BRep_Builder B;
    TopoDS_Compound C;
    B.MakeCompound(C);
    for (std::list<Shape*>::iterator it = shapes.begin(); it != shapes.end(); it++) {
      TopoDS_Shape shape = (*it)->shape();
      B.Add(C, shape);
    }
    BRepTools::Write(C, filename.c_str());
  } CATCH_AND_RETHROW("Failed to write BREP file ");
  info.GetReturnValue().Set(Nan::New<v8::Boolean>(true));
}

NAN_METHOD(writeSTL)
{
  std::string filename;
  if (!extractFileName(info[0], filename)) {
    return Nan::ThrowError("expecting a file name");
  }
  std::list<Shape*>  shapes;
  for (int i = 1; i < info.Length(); i++) {
    extractShapes(info[i], shapes);
  }
  if (shapes.size() == 0) {
    info.GetReturnValue().Set(Nan::New<v8::Boolean>(false));
    return;
  }
  try {
    BRep_Builder B;
    TopoDS_Compound C;
    B.MakeCompound(C);
    for (std::list<Shape*>::iterator it = shapes.begin(); it != shapes.end(); it++) {
      TopoDS_Shape shape = (*it)->shape();
      B.Add(C, shape);
    }

    StlAPI_Writer writer;
    writer.ASCIIMode() = Standard_False;
    writer.Write(C, filename.c_str());

  } CATCH_AND_RETHROW("Failed to write STL file ");
  info.GetReturnValue().Set(Nan::New<v8::Boolean>(true));
}



static int extractSubShape(const TopoDS_Shape& shape, std::list<v8::Local<v8::Object> >& shapes)
{
  TopAbs_ShapeEnum type = shape.ShapeType();
  switch (type)
  {
  case TopAbs_COMPOUND:
    return 0;
  case TopAbs_COMPSOLID:
  case TopAbs_SOLID:
  {
    shapes.push_back(Nan::To<v8::Object>(Solid::NewInstance(shape)).ToLocalChecked());
    break;
  }
  case TopAbs_FACE:
  case TopAbs_SHELL:
  {
    break;
  }
  case TopAbs_WIRE:
  {

    break;
  }
  case TopAbs_EDGE:
  {
    break;
  }
  case TopAbs_VERTEX:
  {
    break;
  }
  default:
    return 0;
  }
  return 1;
}

static int extractShape(const TopoDS_Shape& shape, std::list<v8::Local<v8::Object> >& shapes)
{
  TopAbs_ShapeEnum type = shape.ShapeType();

  if (type != TopAbs_COMPOUND) {
    extractSubShape(shape, shapes);
    return 0;
  }

  TopExp_Explorer ex;
  int ret = 0;

  // extract compund
  for (ex.Init(shape, TopAbs_COMPOUND); ex.More(); ex.Next())
    ret += extractSubShape(ex.Current(), shapes);

  // extract solids
  for (ex.Init(shape, TopAbs_COMPSOLID); ex.More(); ex.Next())
    ret += extractSubShape(ex.Current(), shapes);
  for (ex.Init(shape, TopAbs_SOLID); ex.More(); ex.Next())
    ret += extractSubShape(ex.Current(), shapes);

  // extract free faces
  for (ex.Init(shape, TopAbs_SHELL, TopAbs_SOLID); ex.More(); ex.Next())
    ret += extractSubShape(ex.Current(), shapes);
  for (ex.Init(shape, TopAbs_FACE, TopAbs_SOLID); ex.More(); ex.Next())
    ret += extractSubShape(ex.Current(), shapes);

  // extract free wires
  for (ex.Init(shape, TopAbs_WIRE, TopAbs_FACE); ex.More(); ex.Next())
    ret += extractSubShape(ex.Current(), shapes);

  // extract free edges
  for (ex.Init(shape, TopAbs_EDGE, TopAbs_WIRE); ex.More(); ex.Next())
    ret += extractSubShape(ex.Current(), shapes);

  // extract free vertices
  for (ex.Init(shape, TopAbs_VERTEX, TopAbs_EDGE); ex.More(); ex.Next())
    ret += extractSubShape(ex.Current(), shapes);

  return ret;
}

static v8::Local<v8::Array> convert(std::list<v8::Local<v8::Object> > & shapes) {
  v8::Local<v8::Array> arr = Nan::New<v8::Array>((int)shapes.size());
  int i = 0;
  for (std::list<v8::Local<v8::Object> >::iterator it = shapes.begin(); it != shapes.end(); it++) {
    Nan::Set(arr,i, *it);
    i++;
  }
  return arr;
}


bool mutex_initialised = false;
uv_mutex_t stepOperation_mutex = { 0 };

class MutexLocker
{
  uv_mutex_t& m_mutex;
public:
  MutexLocker(uv_mutex_t& mutex)
    :m_mutex(mutex)
  {
    uv_mutex_lock(&m_mutex);
  }
  ~MutexLocker()
  {
    uv_mutex_unlock(&m_mutex);
  }
};


/*

class MyProgressIndicator : public Message_ProgressIndicator
{
  ProgressData* m_data;
  AsyncWorkerWithProgress* _worker;
public:
  MyProgressIndicator(AsyncWorkerWithProgress* worker);

  void notify_progress();
  virtual Standard_Boolean Show(const Standard_Boolean force);
};


MyProgressIndicator::Message_ProgressIndicator(AsyncWorkerWithProgress* worker)
  :Message_ProgressIndicator(), _worker(worker)
{
  m_data = &_worker->m_data;
}


Standard_Boolean MyProgressIndicator::Show(const Standard_Boolean force)
{

  double value = this->GetPosition();
  double delta = (value - this->m_data->m_lastValue);

  if (delta > 0.01) {
    this->m_data->m_percent = value;
    this->m_data->m_progress = int(delta * 1000);// this->GetPosition();
    this->m_data->m_lastValue = value;
    this->_worker->send_notify_progress();

  }
  return Standard_False;
}
*/




//http://free-cad.sourceforge.net/SrcDocu/df/d7b/ImportStep_8cpp_source.html


class StepBrepAsyncReadWorker : public AsyncWorkerWithProgress {
protected:
  StepBrepAsyncReadWorker(Nan::Callback *callback, Nan::Callback* progressCallback, std::string* pfilename)
    : AsyncWorkerWithProgress(callback, progressCallback, pfilename)
  {
  }  
  virtual void WorkComplete();
protected:
  int _retValue;
  std::list<TopoDS_Shape > shapes;
};

void StepBrepAsyncReadWorker::WorkComplete() {

  Nan::HandleScope scope;
  if (this->_retValue == 0) {

    try {

      std::list<v8::Local<v8::Object> > jsshapes;

      for (std::list<TopoDS_Shape >::iterator it = shapes.begin(); it != shapes.end(); it++) {
        const TopoDS_Shape& aShape = (*it);
        extractShape(aShape, jsshapes);
      }

      v8::Local<v8::Array> arr = convert(jsshapes);

      v8::Local<v8::Value> err = Nan::Null(); 
      v8::Local<v8::Value> argv[2] = { err, arr };

      callback->Call(2, argv, async_resource);

    }
    catch (...) {
      this->SetErrorMessage(" exception in trying to build shapes");
      return this->AsyncWorkerWithProgress::WorkComplete();
    }
  }
  else {  
    return this->AsyncWorkerWithProgress::WorkComplete();
  }
}

class StepAsyncReadWorker : public StepBrepAsyncReadWorker {
public:
  StepAsyncReadWorker(Nan::Callback *callback, Nan::Callback* progressCallback, std::string* pfilename)
    : StepBrepAsyncReadWorker(callback, progressCallback, pfilename)
  {
  }
  ~StepAsyncReadWorker() {
  }

  virtual void Execute();
};



void StepAsyncReadWorker::Execute() {

  MutexLocker _locker(stepOperation_mutex);

  void* data = request.data;
  this->_retValue = 0;

  //occHandle(Message_ProgressRange) progress = new MyProgressIndicator(this);

  //progress->SetScale(1, 100, 1);
  //progress->Show();

  try {
   /* occHandle(TDocStd_Document) hDoc;
    occHandle(XCAFApp_Application) hApp = XCAFApp_Application::GetApplication();
    hApp->NewDocument(TCollection_ExtendedString("MDTV-XCAF"), hDoc);
    occHandle(XCAFDoc_ShapeTool) Assembly = XCAFDoc_DocumentTool::ShapeTool (hDoc->Main()); 
    
      STEPCAFControl_Reader aReader;
    Interface_Static::SetCVal("xstep.cascade.unit",    "mm");
    Interface_Static::SetIVal("read.step.nonmanifold",  1);
    Interface_Static::SetIVal("read.step.product.mode", 1);
      aReader.SetColorMode(true);
      aReader.SetNameMode(true);
      aReader.SetLayerMode(true);
      if (aReader.ReadFile((Standard_CString)(_filename.c_str())) != IFSelect_RetDone) {

      std::stringstream str;
    std::cerr << " (1) cannot read STEP file "  << std::endl;
    this->SetErrorMessage("2 - caught C++ exception in readStep");
    this->_retValue = 2;

      // Local<Value> argv[] = { Local<Value>(String::New())  };
      //  Local<Value>  res =  callback->Call(global, 1, argv);
      // NanReturnUndefined();
      //progress->EndScope();
      //progress->SetValue(105.0);
      //progress->Show();
      this->_retValue = 1;
      return;

    }
    aReader.Transfer(hDoc);

    TDF_LabelSequence frshapes;
    Assembly->GetShapes(frshapes);
    if (frshapes.Length() == 0) {
      std::stringstream str;
    std::cerr << " (1) cannot read STEP file "  << std::endl;
    this->SetErrorMessage("2 - caught C++ exception in readStep (No Data)");
    this->_retValue = 2;
    return;
    } else if (frshapes.Length() == 1) {
        TopoDS_Shape shape = Assembly->GetShape(frshapes.Value(1));
                    this->shapes.push_back(shape);
    } else {
        for (Standard_Integer i=1; i<frshapes.Length(); i++) {
            TopoDS_Shape S = Assembly->GetShape(frshapes.Value(i));

            TDF_Label aLabel = Assembly->FindShape(S, Standard_False);
            if ( (!aLabel.IsNull()) && (Assembly->IsShape(aLabel)) ) {
                if (Assembly->IsFree(aLabel) ) {
                    this->shapes.push_back(S);
                }
            }
        }
    }
    */

    STEPControl_Reader aReader;
    //TODO:
    //See https://gitlab.onelab.info/gmsh/gmsh/-/blob/master/src/geo/GModelIO_OCC.cpp 
    //For reference.
    //CAF is for Labels + Colors
    //STEPCAFControl_Reader aReader;



    Interface_Static::SetCVal("xstep.cascade.unit",    "mm");
    Interface_Static::SetIVal("read.step.nonmanifold",  1);
    Interface_Static::SetIVal("read.step.product.mode", 1);

    //progress->NewScope(5, "reading");
      std::cout << " STEP INITIALIZED." << std::endl;

    if (aReader.ReadFile(_filename.c_str()) != IFSelect_RetDone) {

      std::stringstream str;
      str << " (1) cannot read STEP file " << _filename << std::ends;
      this->SetErrorMessage(str.str().c_str());

      // Local<Value> argv[] = { Local<Value>(String::New())  };
      //  Local<Value>  res =  callback->Call(global, 1, argv);
      // NanReturnUndefined();
      //progress->EndScope();
      //progress->SetValue(105.0);
      //progress->Show();
      this->_retValue = 1;
      return;

    }
      std::cout << " STEP READ." << std::endl;
    //progress->EndScope();
    //progress->Show();


    //progress->NewScope(95, "transfert");
    //progress->Show();
    //aReader.WS()->MapReader()->SetProgress(progress);


    // Root transfers
    int nbr = aReader.NbRootsForTransfer();
      std::cout << "# Roots: "<< nbr << std::endl;

    Standard_Boolean failsonly = Standard_False;
    aReader.PrintCheckTransfer(failsonly, IFSelect_ItemsByEntity);
    
    std::cout << "PrintCheckTransfer: "<< failsonly << std::endl;
    //progress->SetRange(0, nbr);
    int mod = nbr / 10 + 1;
    for (int n = 1; n <= nbr; n++) {

      Standard_Boolean ok = aReader.TransferRoot(n);
      std:: cout << n << std::endl;
      Standard_Integer nbs = aReader.NbShapes();
      if (!ok || nbs == 0) {
        continue; // skip empty root
      }
      if ((n + 1) % mod == 0) { 
        //progress->Increment(); 
      }
    }
    //aReader.TransferRoots();
      std::cout << " STEP Roots Transferred." << std::endl;

    //aReader.WS()->MapReader()->SetProgress(0);
  
    //progress->SetValue(100);
    //progress->EndScope();
    //progress->Show();

    TopoDS_Shape aResShape;
    BRep_Builder B;
    TopoDS_Compound compound;
    B.MakeCompound(compound);
      std::cout << " STEP Shape Compounded." << std::endl;



    int nbs = aReader.NbShapes();
    for (int i = 1; i <= nbs; i++) {
      const TopoDS_Shape& aShape = aReader.Shape(i);             


      if (aShape.ShapeType() == TopAbs_SOLID) {
        ShapeFix_Solid fix((const TopoDS_Solid&)aShape);
        fix.Perform();
      }
      B.Add(compound, aShape);
      this->shapes.push_back(aShape);
    }
      std::cout << " STEP Shapes Pushed." << std::endl;

    aResShape = compound;

    TopTools_IndexedMapOfShape anIndices;
    TopExp::MapShapes(aResShape, anIndices);

    occHandle(Interface_InterfaceModel) Model = aReader.WS()->Model();
    occHandle(XSControl_TransferReader) TR = aReader.WS()->TransferReader();

    if (!TR.IsNull()) {
      occHandle(Transfer_TransientProcess) TP = TR->TransientProcess();
      occHandle(Standard_Type) tPD = STANDARD_TYPE(StepBasic_ProductDefinition);
      occHandle(Standard_Type) tNAUO = STANDARD_TYPE(StepRepr_NextAssemblyUsageOccurrence);
      occHandle(Standard_Type) tShape = STANDARD_TYPE(StepShape_TopologicalRepresentationItem);
      occHandle(Standard_Type) tGeom = STANDARD_TYPE(StepGeom_GeometricRepresentationItem);

      Standard_Integer nb = Model->NbEntities();


      for (Standard_Integer ie = 1; ie <= nb; ie++) {

        occHandle(Standard_Transient) enti = Model->Value(ie);

        occHandle(TCollection_HAsciiString) aName;

        if (enti->DynamicType() == tNAUO) {
          occHandle(StepRepr_NextAssemblyUsageOccurrence) NAUO = occHandle(StepRepr_NextAssemblyUsageOccurrence)::DownCast(enti);
          if (NAUO.IsNull()) continue;

          // Interface_EntityIterator subs = aReader.WS()->Graph().Sharings(NAUO);
          auto  subs = aReader.WS()->Graph().Sharings(NAUO);
          for (subs.Start(); subs.More(); subs.Next()) {
            occHandle(StepRepr_ProductDefinitionShape) PDS = occHandle(StepRepr_ProductDefinitionShape)::DownCast(subs.Value());
            if (PDS.IsNull()) continue;
            occHandle(StepBasic_ProductDefinitionRelationship) PDR = PDS->Definition().ProductDefinitionRelationship();
            if (PDR.IsNull()) continue;
            if (PDR->HasDescription() && PDR->Description()->Length() > 0) {
              aName = PDR->Description();
            }
            else if (PDR->Name()->Length() > 0) {
              aName = PDR->Name();
            }
            else {
              aName = PDR->Id();
            }
          }
          // find proper label
          TCollection_ExtendedString str(aName->String());
        }
        else  if (enti->IsKind(tShape) || enti->IsKind(tGeom)) {
          aName = occHandle(StepRepr_RepresentationItem)::DownCast(enti)->Name();
        }
        else if (enti->DynamicType() == tPD) {
          occHandle(StepBasic_ProductDefinition) PD = occHandle(StepBasic_ProductDefinition)::DownCast(enti);
          if (PD.IsNull()) continue;
          occHandle(StepBasic_Product) Prod = PD->Formation()->OfProduct();
          aName = Prod->Name();
        }
        else {
          continue;
        }
        if (aName->UsefullLength() < 1)
          continue;
        // skip 'N0NE' name
        if (aName->UsefullLength() == 4 && toupper(aName->Value(1)) == 'N' &&toupper(aName->Value(2)) == 'O' && toupper(aName->Value(3)) == 'N' && toupper(aName->Value(4)) == 'E')
          continue;
        TCollection_ExtendedString aNameExt(aName->ToCString());

        // find target shape
        occHandle(Transfer_Binder) binder = TP->Find(enti);
        if (binder.IsNull()) continue;

        TopoDS_Shape S = TransferBRep::ShapeResult(binder);
        if (S.IsNull()) continue;
        // as PRODUCT can be included in the main shape
        // several times, we look here for all inclusions.
        Standard_Integer isub, nbSubs = anIndices.Extent();
        for (isub = 1; isub <= nbSubs; isub++) {
          TopoDS_Shape aSub = anIndices.FindKey(isub);
          if (aSub.IsPartner(S)) {

            //xx cout << " name of part =" << aName->ToCString() << "  shape " << HashCode(aSub, -1) << " " << aSub.ShapeType() << endl;
          }
        }

      }
      // END: Store names
    }
      std::cout << " STEP Finalized." << std::endl;
    
  }
  /*catch (OSD_Exception& e) {
      Base::Console().Error("%s\n", e.GetMessageString());
      Base::Console().Message("Try to load STEP file without colors...\n");

      Part::ImportStepParts(pcDoc,Utf8Name.c_str());
      pcDoc->recompute();
  }*/
  catch (...) {
    std::cerr << " EXCEPTION in READ STEP" << std::endl;
    this->SetErrorMessage("2 - caught C++ exception in readStep");
    this->_retValue = 2;
    return;
  }

}

void readStepAsync(const std::string& filename, v8::Local<v8::Function> _callback, v8::Local<v8::Function> _progressCallback)
{
  Nan::Callback* callback = new Nan::Callback(_callback);
  Nan::Callback* progressCallback = _progressCallback.IsEmpty() ? nullptr : new Nan::Callback(_progressCallback);
  std::string* pfilename = new std::string(filename);
  Nan::AsyncQueueWorker(new StepAsyncReadWorker(callback, progressCallback, pfilename));
}

NAN_METHOD(readSTEP)
{
  if (!mutex_initialised) { uv_mutex_init(&stepOperation_mutex); mutex_initialised = true; }

  std::string filename;
  if (!extractFileName(info[0], filename)) {
    return Nan::ThrowError("expecting a file name");
  }


  v8::Local<v8::Function> callback;
  if (!extractCallback(info[1], callback)) {
    return Nan::ThrowError("expecting a callback function");
  }


  v8::Local<v8::Function> progressCallback;
  if (!extractCallback(info[2], progressCallback)) {
    // OPTIONAL !!!
    // Nan::ThrowError("expecting a callback function");
  }

  readStepAsync(filename, callback, progressCallback);
}



class BRepAsyncReadWorker : public StepBrepAsyncReadWorker {
public:
  BRepAsyncReadWorker(Nan::Callback *callback, Nan::Callback* progressCallback, std::string* pfilename)
    : StepBrepAsyncReadWorker(callback, progressCallback, pfilename)
  {
  }
  ~BRepAsyncReadWorker() {

  }

  void Execute();

};


void BRepAsyncReadWorker::Execute()
{

  this->_retValue = 0;

  std::string filename = this->_filename;

  try {
    //occHandle(Message_ProgressIndicator) progress = new MyProgressIndicator(this);
    //progress->SetScale(1, 100, 1);
    //progress->Show();

    // read brep-file
    TopoDS_Shape shape;
    BRep_Builder aBuilder;
    if (!BRepTools::Read(shape, filename.c_str(), aBuilder, Message_ProgressRange())) {

      std::stringstream str;
      str << "1- cannot read BREP file : '" << filename << "'" << std::ends;
      this->SetErrorMessage(str.str().c_str());
      this->_retValue = 1;

      //progress->SetValue(100.0);
      //progress->Show();
      return;
    }
    this->shapes.push_back(shape);
    //progress->SetValue(100.0);
    //progress->Show();
  }
  catch (...) {
    this->SetErrorMessage("2 ( caught C++ exception in _readBREPAsync");
    this->_retValue = 2;
    return;
  }
}

void readBREPAsync(const std::string& filename, v8::Local<v8::Function> _callback, v8::Local<v8::Function> _progressCallback)
{
  Nan::Callback* callback = new Nan::Callback(_callback);
  Nan::Callback* progressCallback = _progressCallback.IsEmpty() ? nullptr : new Nan::Callback(_progressCallback);
  std::string* pfilename = new std::string(filename);

  Nan::AsyncQueueWorker(new BRepAsyncReadWorker(callback, progressCallback, pfilename));

}


NAN_METHOD(readBREP)
{

  std::string filename;
  if (!extractFileName(info[0], filename)) {
    return Nan::ThrowError("expecting a file name");
  }
  v8::Local<v8::Function> callback;
  if (!extractCallback(info[1], callback)) {
    return Nan::ThrowError("expecting a callback function");
  }
  v8::Local<v8::Function> progressCallback;
  if (!extractCallback(info[2], progressCallback)) {
    // OPTIONAL !!!
    // return Nan::ThrowError("expecting a callback function");
  }
  readBREPAsync(filename, callback, progressCallback);

  info.GetReturnValue().SetUndefined();
}

#ifdef CADEXCHANGER

class ParasolidAsyncReadWorker : public StepBrepAsyncReadWorker {
public:
  ParasolidAsyncReadWorker(Nan::Callback *callback, Nan::Callback* progressCallback, std::string* pfilename)
    : StepBrepAsyncReadWorker(callback, progressCallback, pfilename)
  {
  }
  ~ParasolidAsyncReadWorker() {

  }

  void Execute();

};


void ParasolidAsyncReadWorker::Execute()
{

  this->_retValue = 0;

  std::string filename = this->_filename;

  try {
    //occHandle(Message_ProgressIndicator) progress = new MyProgressIndicator(this);
    //progress->SetScale(1, 100, 1);
    //progress->Show();

    // read parasolid-file
    auto aKey = cadex::LicenseKey::Value();

    // Activate the license (aKey must be defined in cadex_license.cxx)
    if (!CADExLicense_Activate (aKey)) {
        std::stringstream str;
        str << "Failed to activate CAD Exchanger license." <<  std::ends;
        this->SetErrorMessage(str.str().c_str());
        this->_retValue = 1;
    }
    cadex::Para_Reader aReader;
    TopoDS_Shape aShape;
    if (!aReader.ReadFile (filename.c_str()) || !aReader.Transfer (aShape)) 
    {
      
      std::stringstream str;
      str << "1- cannot read Parasolid file : '" << filename << "'" << std::ends;
      this->SetErrorMessage(str.str().c_str());
      this->_retValue = 1;

      //progress->SetValue(100.0);
      //progress->Show();
      return;
    }
    this->shapes.push_back(aShape);
    //progress->SetValue(100.0);
    //progress->Show();
  }
  catch (...) {
    this->SetErrorMessage("2 ( caught C++ exception in _readParasolidAsync");
    this->_retValue = 2;
    return;
  }
}


  void readParasolidAsync(const std::string& filename, v8::Local<v8::Function> _callback, v8::Local<v8::Function> _progressCallback)
  {
    Nan::Callback* callback = new Nan::Callback(_callback);
    Nan::Callback* progressCallback = _progressCallback.IsEmpty() ? nullptr : new Nan::Callback(_progressCallback);
    std::string* pfilename = new std::string(filename);

    Nan::AsyncQueueWorker(new ParasolidAsyncReadWorker(callback, progressCallback, pfilename));

  }
  NAN_METHOD(readParasolid)
  {

    std::string filename;
    if (!extractFileName(info[0], filename)) {
      return Nan::ThrowError("expecting a file name");
    }
    v8::Local<v8::Function> callback;
    if (!extractCallback(info[1], callback)) {
      return Nan::ThrowError("expecting a callback function");
    }
    v8::Local<v8::Function> progressCallback;
    if (!extractCallback(info[2], progressCallback)) {
      // OPTIONAL !!!
      // return Nan::ThrowError("expecting a callback function");
    }
    readParasolidAsync(filename, callback, progressCallback);

    info.GetReturnValue().SetUndefined();
  }

NAN_METHOD(writeParasolid)
{
  std::string filename;
  if (!extractFileName(info[0], filename)) {
    return Nan::ThrowError("expecting a file name");
  }
  std::list<Shape*>  shapes;
  for (int i = 1; i < info.Length(); i++) {
    extractShapes(info[i], shapes);
  }
  if (shapes.size() == 0) {
    info.GetReturnValue().Set(Nan::New<v8::Boolean>(false));
    return;
  }

  try {
    auto aKey = cadex::LicenseKey::Value();

    // Activate the license (aKey must be defined in cadex_license.cxx)
    if (!CADExLicense_Activate (aKey)) {
        return Nan::ThrowError("Failed to activate CAD Exchanger license.");
    }
    cadex::ModelData_Model aModel;
    cadex::Para_Writer aWriter;
    for (std::list<Shape*>::iterator it = shapes.begin(); it != shapes.end(); it++) {
      TopoDS_Shape shape = (*it)->shape();
      cadex::ModelData_ShapeConverter::Add (shape, aModel);
    }
    if (!aWriter.Transfer (aModel) || !aWriter.WriteFile (filename.c_str())) {
        //some error happened
        return Nan::ThrowError("Error in Parasolid Write!");
    }
  } CATCH_AND_RETHROW("Failed to write Parasolid file ");
  info.GetReturnValue().Set(Nan::New<v8::Boolean>(true));
}
NAN_METHOD(convertParasolid2GLTF)
{
  std::string inputfilename;
  if (!extractFileName(info[0], inputfilename)) {
    return Nan::ThrowError("expecting an input file name");
  }
  std::string outputfilename;
  if (!extractFileName(info[1], outputfilename)) {
    return Nan::ThrowError("expecting an output file name");
  }

  try {
    auto aKey = cadex::LicenseKey::Value();

    // Activate the license (aKey must be defined in cadex_license.cxx)
    if (!CADExLicense_Activate (aKey)) {
        return Nan::ThrowError("Failed to activate CAD Exchanger license.");
    }
    cadex::ModelData_Model aModel;
    cadex::Para_Reader aParasolidReader;
    aParasolidReader.ReadFile (inputfilename.c_str()) && aParasolidReader.Transfer (aModel);
    cadex::GLTF_Writer aWriter;
    if (!aWriter.Transfer (aModel) || !aWriter.WriteFile (outputfilename.c_str())) {
        //some error happened
        return Nan::ThrowError("Error in Parasolid Write!");
    }
  } CATCH_AND_RETHROW("Failed to write Parasolid file ");
  info.GetReturnValue().Set(Nan::New<v8::Boolean>(true));
}

#endif
#ifdef PARASOLID

class ParasolidAsyncReadWorker : public StepBrepAsyncReadWorker {
public:
  ParasolidAsyncReadWorker(Nan::Callback *callback, Nan::Callback* progressCallback, std::string* pfilename)
    : StepBrepAsyncReadWorker(callback, progressCallback, pfilename)
  {
  }
  ~ParasolidAsyncReadWorker() {

  }

  void Execute();

};


void ParasolidAsyncReadWorker::Execute() {

  MutexLocker _locker(stepOperation_mutex);

  void* data = request.data;
  this->_retValue = 0;

  //occHandle(Message_ProgressRange) progress = new MyProgressIndicator(this);

  //progress->SetScale(1, 100, 1);
  //progress->Show();

  try {
   /* occHandle(TDocStd_Document) hDoc;
    occHandle(XCAFApp_Application) hApp = XCAFApp_Application::GetApplication();
    hApp->NewDocument(TCollection_ExtendedString("MDTV-XCAF"), hDoc);
    occHandle(XCAFDoc_ShapeTool) Assembly = XCAFDoc_DocumentTool::ShapeTool (hDoc->Main()); 
    
      STEPCAFControl_Reader aReader;
    Interface_Static::SetCVal("xstep.cascade.unit",    "mm");
    Interface_Static::SetIVal("read.step.nonmanifold",  1);
    Interface_Static::SetIVal("read.step.product.mode", 1);
      aReader.SetColorMode(true);
      aReader.SetNameMode(true);
      aReader.SetLayerMode(true);
      if (aReader.ReadFile((Standard_CString)(_filename.c_str())) != IFSelect_RetDone) {

      std::stringstream str;
    std::cerr << " (1) cannot read STEP file "  << std::endl;
    this->SetErrorMessage("2 - caught C++ exception in readStep");
    this->_retValue = 2;

      // Local<Value> argv[] = { Local<Value>(String::New())  };
      //  Local<Value>  res =  callback->Call(global, 1, argv);
      // NanReturnUndefined();
      //progress->EndScope();
      //progress->SetValue(105.0);
      //progress->Show();
      this->_retValue = 1;
      return;

    }
    aReader.Transfer(hDoc);

    TDF_LabelSequence frshapes;
    Assembly->GetShapes(frshapes);
    if (frshapes.Length() == 0) {
      std::stringstream str;
    std::cerr << " (1) cannot read STEP file "  << std::endl;
    this->SetErrorMessage("2 - caught C++ exception in readStep (No Data)");
    this->_retValue = 2;
    return;
    } else if (frshapes.Length() == 1) {
        TopoDS_Shape shape = Assembly->GetShape(frshapes.Value(1));
                    this->shapes.push_back(shape);
    } else {
        for (Standard_Integer i=1; i<frshapes.Length(); i++) {
            TopoDS_Shape S = Assembly->GetShape(frshapes.Value(i));

            TDF_Label aLabel = Assembly->FindShape(S, Standard_False);
            if ( (!aLabel.IsNull()) && (Assembly->IsShape(aLabel)) ) {
                if (Assembly->IsFree(aLabel) ) {
                    this->shapes.push_back(S);
                }
            }
        }
    }
    */

    XtControl_Reader aReader;


    Interface_Static::SetCVal("xstep.cascade.unit",    "mm");
    Interface_Static::SetIVal("read.step.nonmanifold",  1);
    Interface_Static::SetIVal("read.step.product.mode", 1);

    //progress->NewScope(5, "reading");

    if (aReader.ReadFile(_filename.c_str()) != IFSelect_RetDone) {

      std::stringstream str;
      str << " (1) cannot read STEP file " << _filename << std::ends;
      this->SetErrorMessage(str.str().c_str());

      // Local<Value> argv[] = { Local<Value>(String::New())  };
      //  Local<Value>  res =  callback->Call(global, 1, argv);
      // NanReturnUndefined();
      //progress->EndScope();
      //progress->SetValue(105.0);
      //progress->Show();
      this->_retValue = 1;
      return;

    }
    //progress->EndScope();
    //progress->Show();


    //progress->NewScope(95, "transfert");
    //progress->Show();
    //aReader.WS()->MapReader()->SetProgress(progress);


    // Root transfers
    int nbr = aReader.NbRootsForTransfer();

    Standard_Boolean failsonly = Standard_False;
    aReader.PrintCheckTransfer(failsonly, IFSelect_ItemsByEntity);

    //progress->SetRange(0, nbr);
    // int mod = nbr / 10 + 1;
    // for (int n = 1; n <= nbr; n++) {
    //   aReader.ClearShapes();
    //   Standard_Boolean ok = aReader.TransferRoot(n);

    //   Standard_Integer nbs = aReader.NbShapes();
    //   if (!ok || nbs == 0) {
    //     continue; // skip empty root
    //   }
    //   if ((n + 1) % mod == 0) { 
    //     //progress->Increment(); 
    //   }
    // }
    aReader.TransferRoots();
    //aReader.WS()->MapReader()->SetProgress(0);
  
    //progress->SetValue(100);
    //progress->EndScope();
    //progress->Show();

    TopoDS_Shape aResShape;
    BRep_Builder B;
    TopoDS_Compound compound;
    B.MakeCompound(compound);



    int nbs = aReader.NbShapes();
    for (int i = 1; i <= nbs; i++) {
      const TopoDS_Shape& aShape = aReader.Shape(i);             


      if (aShape.ShapeType() == TopAbs_SOLID) {
        ShapeFix_Solid fix((const TopoDS_Solid&)aShape);
        fix.Perform();
      }
      B.Add(compound, aShape);
      this->shapes.push_back(aShape);
    }

    aResShape = compound;

    TopTools_IndexedMapOfShape anIndices;
    TopExp::MapShapes(aResShape, anIndices);

    occHandle(Interface_InterfaceModel) Model = aReader.WS()->Model();
    occHandle(XSControl_TransferReader) TR = aReader.WS()->TransferReader();

    if (!TR.IsNull()) {
      occHandle(Transfer_TransientProcess) TP = TR->TransientProcess();
      occHandle(Standard_Type) tPD = STANDARD_TYPE(StepBasic_ProductDefinition);
      occHandle(Standard_Type) tNAUO = STANDARD_TYPE(StepRepr_NextAssemblyUsageOccurrence);
      occHandle(Standard_Type) tShape = STANDARD_TYPE(StepShape_TopologicalRepresentationItem);
      occHandle(Standard_Type) tGeom = STANDARD_TYPE(StepGeom_GeometricRepresentationItem);

      Standard_Integer nb = Model->NbEntities();


      for (Standard_Integer ie = 1; ie <= nb; ie++) {

        occHandle(Standard_Transient) enti = Model->Value(ie);

        occHandle(TCollection_HAsciiString) aName;

        if (enti->DynamicType() == tNAUO) {
          occHandle(StepRepr_NextAssemblyUsageOccurrence) NAUO = occHandle(StepRepr_NextAssemblyUsageOccurrence)::DownCast(enti);
          if (NAUO.IsNull()) continue;

          // Interface_EntityIterator subs = aReader.WS()->Graph().Sharings(NAUO);
          auto  subs = aReader.WS()->Graph().Sharings(NAUO);
          for (subs.Start(); subs.More(); subs.Next()) {
            occHandle(StepRepr_ProductDefinitionShape) PDS = occHandle(StepRepr_ProductDefinitionShape)::DownCast(subs.Value());
            if (PDS.IsNull()) continue;
            occHandle(StepBasic_ProductDefinitionRelationship) PDR = PDS->Definition().ProductDefinitionRelationship();
            if (PDR.IsNull()) continue;
            if (PDR->HasDescription() && PDR->Description()->Length() > 0) {
              aName = PDR->Description();
            }
            else if (PDR->Name()->Length() > 0) {
              aName = PDR->Name();
            }
            else {
              aName = PDR->Id();
            }
          }
          // find proper label
          TCollection_ExtendedString str(aName->String());
        }
        else  if (enti->IsKind(tShape) || enti->IsKind(tGeom)) {
          aName = occHandle(StepRepr_RepresentationItem)::DownCast(enti)->Name();
        }
        else if (enti->DynamicType() == tPD) {
          occHandle(StepBasic_ProductDefinition) PD = occHandle(StepBasic_ProductDefinition)::DownCast(enti);
          if (PD.IsNull()) continue;
          occHandle(StepBasic_Product) Prod = PD->Formation()->OfProduct();
          aName = Prod->Name();
        }
        else {
          continue;
        }
        if (aName->UsefullLength() < 1)
          continue;
        // skip 'N0NE' name
        if (aName->UsefullLength() == 4 && toupper(aName->Value(1)) == 'N' &&toupper(aName->Value(2)) == 'O' && toupper(aName->Value(3)) == 'N' && toupper(aName->Value(4)) == 'E')
          continue;
        TCollection_ExtendedString aNameExt(aName->ToCString());

        // find target shape
        occHandle(Transfer_Binder) binder = TP->Find(enti);
        if (binder.IsNull()) continue;

        TopoDS_Shape S = TransferBRep::ShapeResult(binder);
        if (S.IsNull()) continue;
        // as PRODUCT can be included in the main shape
        // several times, we look here for all inclusions.
        Standard_Integer isub, nbSubs = anIndices.Extent();
        for (isub = 1; isub <= nbSubs; isub++) {
          TopoDS_Shape aSub = anIndices.FindKey(isub);
          if (aSub.IsPartner(S)) {

            //xx cout << " name of part =" << aName->ToCString() << "  shape " << HashCode(aSub, -1) << " " << aSub.ShapeType() << endl;
          }
        }

      }
      // END: Store names
    }
    
  }
  /*catch (OSD_Exception& e) {
      Base::Console().Error("%s\n", e.GetMessageString());
      Base::Console().Message("Try to load STEP file without colors...\n");

      Part::ImportStepParts(pcDoc,Utf8Name.c_str());
      pcDoc->recompute();
  }*/
  catch (...) {
    std::cerr << " EXCEPTION in READ STEP" << std::endl;
    this->SetErrorMessage("2 - caught C++ exception in readStep");
    this->_retValue = 2;
    return;
  }

}



  void readParasolidAsync(const std::string& filename, v8::Local<v8::Function> _callback, v8::Local<v8::Function> _progressCallback)
  {
    Nan::Callback* callback = new Nan::Callback(_callback);
    Nan::Callback* progressCallback = _progressCallback.IsEmpty() ? nullptr : new Nan::Callback(_progressCallback);
    std::string* pfilename = new std::string(filename);

    Nan::AsyncQueueWorker(new ParasolidAsyncReadWorker(callback, progressCallback, pfilename));

  }
  NAN_METHOD(readParasolid)
  {

    std::string filename;
    if (!extractFileName(info[0], filename)) {
      return Nan::ThrowError("expecting a file name");
    }
    v8::Local<v8::Function> callback;
    if (!extractCallback(info[1], callback)) {
      return Nan::ThrowError("expecting a callback function");
    }
    v8::Local<v8::Function> progressCallback;
    if (!extractCallback(info[2], progressCallback)) {
      // OPTIONAL !!!
      // return Nan::ThrowError("expecting a callback function");
    }
    readParasolidAsync(filename, callback, progressCallback);

    info.GetReturnValue().SetUndefined();
  }

// NAN_METHOD(writeParasolid)
// {
//   std::string filename;
//   if (!extractFileName(info[0], filename)) {
//     return Nan::ThrowError("expecting a file name");
//   }
//   std::list<Shape*>  shapes;
//   for (int i = 1; i < info.Length(); i++) {
//     extractShapes(info[i], shapes);
//   }
//   if (shapes.size() == 0) {
//     info.GetReturnValue().Set(Nan::New<v8::Boolean>(false));
//     return;
//   }

//   try {
//     auto aKey = cadex::LicenseKey::Value();

//     // Activate the license (aKey must be defined in cadex_license.cxx)
//     if (!CADExLicense_Activate (aKey)) {
//         return Nan::ThrowError("Failed to activate CAD Exchanger license.");
//     }
//     cadex::ModelData_Model aModel;
//     cadex::Para_Writer aWriter;
//     for (std::list<Shape*>::iterator it = shapes.begin(); it != shapes.end(); it++) {
//       TopoDS_Shape shape = (*it)->shape();
//       cadex::ModelData_ShapeConverter::Add (shape, aModel);
//     }
//     if (!aWriter.Transfer (aModel) || !aWriter.WriteFile (filename.c_str())) {
//         //some error happened
//         return Nan::ThrowError("Error in Parasolid Write!");
//     }
//   } CATCH_AND_RETHROW("Failed to write Parasolid file ");
//   info.GetReturnValue().Set(Nan::New<v8::Boolean>(true));
// }
// NAN_METHOD(convertParasolid2GLTF)
// {
//   std::string inputfilename;
//   if (!extractFileName(info[0], inputfilename)) {
//     return Nan::ThrowError("expecting an input file name");
//   }
//   std::string outputfilename;
//   if (!extractFileName(info[1], outputfilename)) {
//     return Nan::ThrowError("expecting an output file name");
//   }

//   try {
//     auto aKey = cadex::LicenseKey::Value();

//     // Activate the license (aKey must be defined in cadex_license.cxx)
//     if (!CADExLicense_Activate (aKey)) {
//         return Nan::ThrowError("Failed to activate CAD Exchanger license.");
//     }
//     cadex::ModelData_Model aModel;
//     cadex::Para_Reader aParasolidReader;
//     aParasolidReader.ReadFile (inputfilename.c_str()) && aParasolidReader.Transfer (aModel);
//     cadex::GLTF_Writer aWriter;
//     if (!aWriter.Transfer (aModel) || !aWriter.WriteFile (outputfilename.c_str())) {
//         //some error happened
//         return Nan::ThrowError("Error in Parasolid Write!");
//     }
//   } CATCH_AND_RETHROW("Failed to write Parasolid file ");
//   info.GetReturnValue().Set(Nan::New<v8::Boolean>(true));
// }

#endif
//#ifndef CADEXCHANGER
NAN_METHOD(writeGLTF)
{
  std::string filename;
  if (!extractFileName(info[0], filename)) {
    return Nan::ThrowError("expecting a file name");
  }

  std::list<Shape*>  shapes;
  for (int i = 1; i < info.Length(); i++) {
    extractShapes(info[i], shapes);
  }

  if (shapes.size() == 0) {
    return info.GetReturnValue().Set(Nan::New<v8::Boolean>(false));
  }

  try {
    TColStd_IndexedDataMapOfStringString aMetadata;

    //occHandle(Message_ProgressIndicator) progress = new MyProgressIndicator(this);
    occHandle(TDocStd_Document) hDoc;
    occHandle(XCAFApp_Application) hApp = XCAFApp_Application::GetApplication();
    hApp->NewDocument(TCollection_ExtendedString("MDTV-XCAF"), hDoc);
    occHandle(XCAFDoc_ShapeTool) myAssembly = XCAFDoc_DocumentTool::ShapeTool (hDoc->Main()); 
    //TDF_Label aLabel = myAssembly->NewShape() 
    RWGltf_CafWriter aWriter (filename.c_str(), filename.substr(filename.find_last_of(".") + 1)=="glb");
    aWriter.SetTransformationFormat (RWGltf_WriterTrsfFormat_Compact);
    // https://github.com/KhronosGroup/glTF/blob/master/specification/2.0/README.md#coordinate-system-and-units
    aWriter.ChangeCoordinateSystemConverter().SetInputLengthUnit (1); //in MM
    aWriter.ChangeCoordinateSystemConverter().SetInputCoordinateSystem (RWMesh_CoordinateSystem_Zup);
    Standard_Boolean makeAssembly = Standard_True; //default, if true, increases hierarchy size.
    for (std::list<Shape*>::iterator it = shapes.begin(); it != shapes.end(); it++) {
      TDF_Label aLabel = myAssembly->AddShape((*it)->shape(), makeAssembly); 
    }
    TDF_LabelSequence aRootLabels;
    myAssembly->GetFreeShapes(aRootLabels);
    TopoDS_Compound aCompound;
    BRep_Builder    aBuildTool;
    aBuildTool.MakeCompound (aCompound);
    for (TDF_LabelSequence::Iterator aRootIter (aRootLabels); aRootIter.More(); aRootIter.Next())
    {
      const TDF_Label& aRootLabel = aRootIter.Value();
      TopoDS_Shape aRootShape;
      if (XCAFDoc_ShapeTool::GetShape (aRootLabel, aRootShape))
      {
        aBuildTool.Add (aCompound, aRootShape);
      }
    }

    // perform meshing
    occHandle(Prs3d_Drawer) aDrawer = new Prs3d_Drawer(); // holds visualization defaults
    BRepMesh_IncrementalMesh anAlgo;
    //anAlgo.ChangeParameters().Deflection = Prs3d::GetDeflection (aCompound, aDrawer);
    anAlgo.ChangeParameters().Angle      = 20.0 * M_PI / 180.0; // 20 degrees
    anAlgo.ChangeParameters().Deflection               = 0.01;
    anAlgo.ChangeParameters().MinSize               = 0.01;
    anAlgo.ChangeParameters().InParallel = true;
    anAlgo.SetShape (aCompound);
    anAlgo.Perform();
    bool status = aWriter.Perform (hDoc, aMetadata, Message_ProgressRange());

    hApp->Close(hDoc);
    if (!status) {
        return Nan::ThrowError("Failed to write GLTF");
      } 
  } CATCH_AND_RETHROW("Failed to write GLTF");

  info.GetReturnValue().Set(Nan::New<v8::Boolean>(true));
}