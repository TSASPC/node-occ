const nodeocc = require("..");
const occ = nodeocc.occ;


occ.readSTEP("VAT_Test.step", function (err, shapes) {
    //console.log(shapes)
    //console.log(err)
    occ.writeGLTF("VAT_Test.gltf",shapes)
});
