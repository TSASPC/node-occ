const nodeocc = require("..");
const occ = nodeocc.occ;


function clock(start) {
    if ( !start ) return process.hrtime();
    var end = process.hrtime(start);
    return Math.round((end[0]*1000) + (end[1]/1000000));
}
const N = 2;
const dats = ['ssbarb','socketcap','rail']
const start = clock();
for (var p =0; p < N; p++)
    for (var i = 0; i <dats.length; i++)
        occ.convertParasolid2GLTF(dats[i]+".x_t", dats[i]+".glb");
const end = clock(start)
console.log('Took '+end+'ms for '+(N*dats.length)+' Conversions');
console.log('Took '+(end/(N*dats.length))+'ms per Conversion');
