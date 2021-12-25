const nodeocc = require("..");
const occ = nodeocc.occ;


function clock(start) {
    if ( !start ) return process.hrtime();
    var end = process.hrtime(start);
    return Math.round((end[0]*1000) + (end[1]/1000000));
}
const N = 2;
const dat = 'ssbarb'
const start = clock();
occ.readParasolid(dat+".x_t", function (err, shapes) {
    console.log(shapes)
    const end = clock(start)
    console.log('Took '+end+'ms');
});
