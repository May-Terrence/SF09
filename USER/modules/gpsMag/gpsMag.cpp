/*
 * gpsMag.cpp
 *
 *  Created on: 2020年11月28日
 *      Author: 刘成吉
 */

#include "gpsMag.hpp"
#include "string.h"

const float SAMPLING_RES = 10;
const float SAMPLING_MIN_LAT = -90;
const float SAMPLING_MAX_LAT = 90;
const float SAMPLING_MIN_LON = -180;
const float SAMPLING_MAX_LON = 180;

const float declination_table[19][37] = {
    {149.42982,139.42982,129.42982,119.42982,109.42982,99.42982,89.42982,79.42982,69.42982,59.42982,49.42982,39.42982,29.42982,19.42982,9.42982,-0.57018,-10.57018,-20.57018,-30.57018,-40.57018,-50.57018,-60.57018,-70.57018,-80.57018,-90.57018,-100.57018,-110.57018,-120.57018,-130.57018,-140.57018,-150.57018,-160.57018,-170.57018,179.42982,169.42982,159.42982,149.42982},
    {129.72350,117.44264,106.27675,96.07611,86.65367,77.82604,69.43354,61.34790,53.47334,45.74345,38.11493,30.55915,23.05266,15.56845,8.07026,0.51157,-7.16035,-14.99671,-23.03882,-31.31253,-39.82756,-48.58229,-57.57257,-66.80224,-76.29309,-86.09269,-96.27941,-106.96368,-118.28316,-130.38657,-143.39712,-157.34798,-172.09985,172.71041,157.61759,143.16328,129.72350},
    {85.55217,77.63610,71.28198,65.85129,60.92943,56.19924,51.40394,46.35152,40.93254,35.13240,29.02800,22.76471,16.51219,10.40361,4.47591,-1.36061,-7.29972,-13.56717,-20.33231,-27.64625,-35.43223,-43.52666,-51.74560,-59.94560,-68.06018,-76.11386,-84.22919,-92.65008,-101.81721,-112.57190,-126.68545,-148.05164,177.80280,138.85306,112.24808,96.22652,85.55217},
    {47.46676,46.20752,44.77315,43.38132,42.07405,40.72123,39.02862,36.61723,33.16561,28.53811,22.85208,16.48376,9.99764,3.97129,-1.24934,-5.73392,-9.95537,-14.56642,-20.09169,-26.69103,-34.10667,-41.82261,-49.31316,-56.20179,-62.27798,-67.43190,-71.55604,-74.40916,-75.36987,-72.73151,-60.81180,-21.51651,25.67221,42.27293,47.13896,48.09081,47.46676},
    {30.85230,31.07383,30.82762,30.44253,30.14685,30.03972,29.94303,29.32537,27.46672,23.77665,18.08319,10.82311,3.04859,-3.93382,-9.21525,-12.73761,-15.21249,-17.71784,-21.33214,-26.67316,-33.41515,-40.52179,-46.98754,-52.17899,-55.71832,-57.28691,-56.42962,-52.33208,-43.85402,-30.45586,-13.95751,1.69556,13.74137,21.89083,26.92848,29.67895,30.85230},
    {22.27872,22.80208,22.89144,22.73115,22.48110,22.36292,22.48106,22.51808,21.62358,18.70580,13.04246,4.91899,-4.12159,-11.97999,-17.35339,-20.33078,-21.74028,-22.30429,-22.99797,-25.48758,-30.37748,-36.24796,-41.36613,-44.68747,-45.62236,-43.81812,-39.04370,-31.23361,-21.30619,-11.43220,-3.04586,3.96581,9.92051,14.80522,18.51928,20.97339,22.27872},
    {16.77535,17.26078,17.48701,17.50586,17.28046,16.91869,16.66947,16.54681,15.86669,13.28797,7.65685,-0.80738,-10.09007,-17.67695,-22.39570,-24.71604,-25.51571,-24.75384,-22.31254,-20.27272,-21.40935,-25.31718,-29.47979,-31.85235,-31.48877,-28.41892,-23.12231,-16.18014,-9.02321,-3.45191,0.46489,3.92873,7.47714,10.84926,13.70257,15.70340,16.77535},
    {13.11722,13.37999,13.53697,13.64213,13.51862,13.08139,12.58669,12.22195,11.40281,8.74064,3.03248,-5.27164,-13.84131,-20.33623,-23.89768,-24.94427,-24.16696,-21.44804,-16.65468,-11.57062,-9.17726,-10.67932,-14.35074,-17.28133,-17.77932,-15.94890,-12.44438,-7.73479,-3.08253,-0.10308,1.47778,3.23739,5.69958,8.33884,10.69634,12.34763,13.11722},
    {10.86027,10.84311,10.78802,10.86614,10.82660,10.44376,9.97329,9.56517,8.51030,5.51344,-0.27795,-8.04143,-15.48088,-20.66310,-22.77532,-21.94614,-18.98400,-14.68117,-9.75013,-5.14534,-2.07246,-1.81242,-4.23526,-7.17511,-8.61911,-8.27316,-6.53525,-3.65636,-0.68530,0.85477,1.25432,2.22687,4.25977,6.61864,8.78455,10.29691,10.86027},
    {9.65412,9.47055,9.21866,9.26610,9.31617,9.03106,8.63298,8.13549,6.68936,3.23076,-2.55638,-9.56906,-15.79936,-19.60429,-20.16306,-17.72398,-13.51036,-8.94319,-4.99959,-1.80533,0.70520,1.63736,0.32148,-2.02391,-3.63410,-4.02175,-3.40313,-1.86837,-0.15286,0.47666,0.26260,0.84732,2.70977,5.05917,7.34111,9.03687,9.65412},
    {8.97014,9.00488,8.80979,8.96359,9.19654,9.04844,8.56530,7.64578,5.51969,1.46185,-4.34083,-10.58923,-15.55806,-17.91473,-17.18297,-14.00887,-9.67704,-5.46497,-2.28363,-0.02111,1.89784,2.95296,2.25306,0.43899,-1.01576,-1.61934,-1.61055,-1.06870,-0.42999,-0.57588,-1.26445,-1.05555,0.57935,2.99639,5.63172,7.86731,8.97014},
    {8.05364,8.90038,9.27928,9.81741,10.36068,10.39013,9.68557,8.05395,4.96137,0.07375,-5.95698,-11.57288,-15.26102,-16.24039,-14.65177,-11.37263,-7.38273,-3.56765,-0.72868,1.13805,2.65859,3.63912,3.29650,1.92360,0.69270,0.06311,-0.26069,-0.47485,-0.85717,-1.88327,-3.18863,-3.51318,-2.26377,0.14436,3.15228,6.07110,8.05364},
    {6.50676,8.60131,10.09409,11.34973,12.28177,12.46194,11.52588,9.17003,5.01214,-0.92961,-7.49086,-12.75176,-15.40894,-15.33506,-13.21707,-9.96509,-6.25217,-2.66248,0.13977,1.99766,3.36674,4.31107,4.34970,3.54042,2.61979,1.95032,1.27943,0.31325,-1.17765,-3.29530,-5.41927,-6.36449,-5.53191,-3.19867,0.05852,3.54451,6.50676},
    {4.72297,8.05666,10.83494,13.02130,14.42516,14.72066,13.56011,10.55536,5.32509,-1.84890,-9.22855,-14.49666,-16.58473,-15.89094,-13.41064,-10.03642,-6.31692,-2.68976,0.34194,2.55093,4.17653,5.41501,6.09964,6.11745,5.65956,4.85558,3.55177,1.51138,-1.37406,-4.81860,-7.87000,-9.36323,-8.80100,-6.47499,-3.01085,0.91836,4.72297},
    {3.37834,7.62747,11.43831,14.53532,16.55914,17.10928,15.75231,11.96543,5.32788,-3.51576,-12.05022,-17.59555,-19.44526,-18.41349,-15.62077,-11.92636,-7.86545,-3.83213,-0.19787,2.82486,5.30947,7.42114,9.13275,10.21765,10.41233,9.48469,7.23089,3.55521,-1.29658,-6.44422,-10.49765,-12.34822,-11.76790,-9.22856,-5.44532,-1.08105,3.37834},
    {2.74348,7.62543,12.14084,15.97571,18.69139,19.70118,18.21773,13.23975,4.12042,-7.61873,-17.81916,-23.55512,-24.98204,-23.43924,-20.11619,-15.80905,-11.02265,-6.11388,-1.36465,3.04381,7.06375,10.70545,13.87807,16.28311,17.42655,16.70088,13.51831,7.64077,-0.16718,-7.83650,-13.15188,-15.19775,-14.31554,-11.33845,-7.10164,-2.26053,2.74348},
    {2.39274,7.82484,12.91652,17.34244,20.60856,21.88978,19.76035,12.03027,-2.57373,-19.28852,-30.38750,-34.61212,-34.22997,-31.16364,-26.55257,-21.04868,-15.04630,-8.80967,-2.53355,3.63374,9.57427,15.16558,20.22250,24.42837,27.25922,27.88349,25.07001,17.49446,5.52718,-6.65781,-14.50735,-17.27006,-16.30170,-13.00872,-8.39222,-3.11386,2.39274},
    {1.41048,6.79922,11.76290,15.71616,17.64723,15.50964,5.20845,-16.74575,-39.26184,-50.10475,-52.24001,-49.91343,-45.26415,-39.31139,-32.58025,-25.36820,-17.86089,-10.18671,-2.44480,5.27857,12.89790,20.31585,27.40587,33.98419,39.75848,44.22469,46.44725,44.61461,35.58987,17.59449,-1.23274,-11.49558,-14.19386,-12.63756,-8.86147,-3.96333,1.41048},
    {179.28905,-170.71095,-160.71095,-150.71095,-140.71095,-130.71095,-120.71095,-110.71095,-100.71095,-90.71095,-80.71095,-70.71095,-60.71095,-50.71095,-40.71095,-30.71095,-20.71095,-10.71095,-0.71095,9.28905,19.28905,29.28905,39.28905,49.28905,59.28905,69.28905,79.28905,89.28905,99.28905,109.28905,119.28905,129.28905,139.28905,149.28905,159.28905,169.28905,179.28905}
};

const float inclination_table[19][37] = {
    {-72.15514,-72.15514,-72.15514,-72.15514,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15514,-72.15514,-72.15514,-72.15514,-72.15514,-72.15514,-72.15514},
    {-78.41938,-77.65720,-76.73750,-75.70236,-74.58798,-73.42713,-72.25182,-71.09474,-69.98907,-68.96648,-68.05411,-67.27158,-66.62917,-66.12826,-65.76418,-65.53067,-65.42438,-65.44784,-65.60989,-65.92339,-66.40121,-67.05164,-67.87473,-68.86017,-69.98700,-71.22453,-72.53382,-73.86899,-75.17816,-76.40410,-77.48547,-78.36007,-78.97179,-79.28132,-79.27661,-78.97577,-78.41938},
    {-81.01394,-79.19354,-77.36330,-75.50423,-73.59085,-71.60725,-69.56243,-67.50284,-65.51562,-63.71750,-62.22814,-61.13417,-60.45695,-60.14119,-60.07436,-60.13141,-60.22358,-60.32959,-60.49886,-60.83015,-61.43611,-62.40625,-63.78154,-65.54891,-67.65384,-70.02042,-72.56767,-75.21637,-77.88752,-80.49381,-82.91772,-84.94371,-86.09650,-85.83265,-84.51758,-82.81598,-81.01394},
    {-77.57314,-75.57627,-73.65643,-71.75236,-69.77902,-67.63834,-65.25509,-62.63653,-59.92397,-57.39957,-55.42857,-54.33546,-54.24860,-55.00475,-56.20273,-57.38532,-58.22377,-58.60633,-58.63690,-58.59002,-58.82471,-59.65214,-61.21772,-63.47750,-66.26844,-69.40327,-72.72828,-76.12771,-79.49774,-82.71529,-85.55816,-87.23537,-86.33673,-84.19492,-81.90400,-79.68142,-77.57314},
    {-71.61543,-69.68071,-67.81295,-65.98732,-64.14688,-62.17325,-59.89199,-57.17213,-54.08269,-51.00980,-48.64526,-47.75928,-48.75785,-51.33161,-54.60713,-57.67200,-59.93603,-61.13032,-61.22418,-60.51276,-59.68523,-59.55885,-60.61820,-62.82724,-65.81275,-69.14491,-72.49609,-75.61467,-78.21163,-79.94564,-80.58474,-80.18788,-79.04223,-77.44162,-75.58435,-73.60578,-71.61543},
    {-64.37232,-62.41896,-60.47622,-58.52696,-56.58944,-54.65086,-52.56912,-50.08837,-47.05221,-43.75107,-41.15259,-40.61471,-42.92315,-47.48780,-52.84135,-57.78264,-61.78868,-64.59420,-65.79355,-65.17341,-63.32395,-61.60337,-61.26328,-62.62104,-65.13165,-67.99298,-70.59799,-72.54035,-73.51467,-73.54501,-73.01716,-72.21990,-71.17485,-69.83733,-68.20192,-66.33391,-64.37232},
    {-54.95221,-52.86419,-50.76591,-48.58110,-46.34373,-44.17713,-42.09147,-39.78009,-36.80878,-33.28666,-30.49763,-30.50812,-34.40344,-41.04847,-48.30470,-54.79768,-60.23560,-64.58921,-67.32150,-67.73041,-65.88657,-62.97458,-60.81524,-60.54948,-61.84643,-63.65901,-65.18925,-65.95250,-65.66438,-64.64265,-63.60342,-62.78558,-61.89618,-60.67851,-59.04993,-57.06784,-54.95221},
    {-42.11386,-39.71540,-37.42483,-35.05675,-32.54684,-30.08394,-27.80378,-25.32349,-22.02374,-18.05872,-15.18383,-16.05640,-21.78336,-30.70367,-40.14151,-48.30425,-54.72071,-59.52309,-62.50036,-63.16678,-61.44170,-58.08476,-54.78220,-53.13352,-53.22163,-54.08263,-54.91880,-55.11744,-54.20645,-52.62937,-51.43900,-50.82759,-50.12042,-48.88316,-47.03524,-44.65772,-42.11386},
    {-25.13413,-22.26490,-19.81259,-17.42498,-14.84507,-12.27503,-9.86112,-7.06257,-3.32091,0.81170,3.26607,1.47837,-5.45280,-16.03006,-27.48020,-37.20828,-44.04254,-48.15139,-50.04572,-50.01060,-48.00052,-44.32786,-40.52423,-38.33249,-37.93574,-38.43154,-39.10891,-39.27530,-38.24513,-36.52908,-35.53036,-35.38136,-34.94364,-33.61633,-31.38812,-28.37515,-25.13413},
    {-4.98724,-1.67113,0.80624,2.98017,5.33226,7.69596,9.97866,12.77254,16.36048,19.83531,21.34544,19.04912,12.24385,1.74844,-10.05341,-20.06978,-26.53723,-29.54293,-30.15639,-29.36839,-27.14298,-23.31481,-19.29463,-16.98941,-16.55873,-17.01490,-17.72494,-18.08276,-17.33194,-15.95689,-15.51123,-16.06549,-16.12696,-14.92427,-12.47974,-8.91893,-4.98724},
    {14.90282,18.28085,20.60765,22.44210,24.40105,26.45274,28.51977,30.96055,33.77883,36.09590,36.59075,34.17570,28.42190,19.80101,10.09244,1.82326,-3.38282,-5.35317,-5.09095,-3.87579,-1.76843,1.66543,5.30533,7.38022,7.73795,7.35305,6.77420,6.35804,6.66144,7.31673,7.03597,5.77710,4.97055,5.51060,7.51077,10.90541,14.90282},
    {31.18963,34.07820,36.15493,37.77321,39.49122,41.42181,43.45993,45.61640,47.69248,49.01037,48.72897,46.33216,41.81270,35.71328,29.23542,23.81452,20.40725,19.30431,19.92626,21.21332,22.94355,25.46085,28.10581,29.64612,29.92208,29.67696,29.36743,29.13603,29.17665,29.18105,28.33513,26.63499,25.13382,24.70085,25.65298,28.02186,31.18963},
    {43.43755,45.49190,47.26481,48.85551,50.58716,52.57872,54.71110,56.79027,58.49411,59.27330,58.58690,56.31729,52.84224,48.81000,44.95503,41.89331,40.02366,39.54270,40.19143,41.32256,42.64128,44.25362,45.87796,46.89272,47.16813,47.12227,47.08047,47.09378,47.09015,46.74024,45.58111,43.68407,41.77157,40.56696,40.45969,41.53519,43.43755},
    {53.14782,54.39692,55.85421,57.48060,59.33861,61.41997,63.58932,65.61909,67.16297,67.73632,66.96414,64.95612,62.26740,59.52539,57.18039,55.46617,54.49933,54.33769,54.84891,55.69458,56.62748,57.61136,58.56598,59.29096,59.71825,59.99071,60.24551,60.46371,60.46654,59.94194,58.64378,56.71646,54.68166,53.08630,52.27188,52.34019,53.14782},
    {61.94434,62.65988,63.82448,65.37386,67.23521,69.29012,71.36784,73.24154,74.58711,74.99756,74.23745,72.52856,70.40872,68.38080,66.74355,65.60955,64.99647,64.87946,65.16376,65.68049,66.27974,66.90598,67.55683,68.22109,68.88444,69.53923,70.14291,70.56735,70.59156,69.97960,68.65713,66.83530,64.91593,63.28781,62.20398,61.76373,61.94434},
    {70.63808,71.09315,71.98608,73.26619,74.84990,76.61252,78.38590,79.94294,80.96893,81.12757,80.33123,78.87452,77.18440,75.59163,74.28473,73.34040,72.76654,72.53101,72.57154,72.80778,73.17108,73.63386,74.21058,74.92988,75.79757,76.76288,77.69093,78.35319,78.47680,77.89101,76.66981,75.09384,73.49103,72.12315,71.15227,70.65240,70.63808},
    {78.93654,79.23546,79.83613,80.70281,81.78075,82.98979,84.21001,85.24697,85.79655,85.59093,84.72327,83.52137,82.24759,81.05849,80.04256,79.24469,78.67956,78.34092,78.20960,78.26251,78.48242,78.86522,79.41957,80.15662,81.07098,82.11600,83.17407,84.02661,84.37742,84.04550,83.16374,82.03536,80.91903,79.98035,79.31253,78.96013,78.93654},
    {86.10236,86.22193,86.48572,86.87541,87.36129,87.89241,88.36587,88.57214,88.32223,87.73845,87.01883,86.26928,85.54489,84.88026,84.29972,83.82070,83.45516,83.21059,83.09120,83.09908,83.23514,83.49941,83.89025,84.40254,85.02486,85.73566,86.49754,87.24606,87.86481,88.16518,88.02685,87.59994,87.10131,86.65607,86.32486,86.13654,86.10236},
    {88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643}
};

const float intensity_table[19][37] = {
    {0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779},
    {0.60837,0.60220,0.59450,0.58547,0.57534,0.56430,0.55260,0.54051,0.52830,0.51629,0.50481,0.49417,0.48467,0.47656,0.47008,0.46544,0.46285,0.46247,0.46446,0.46890,0.47579,0.48502,0.49634,0.50939,0.52367,0.53860,0.55356,0.56792,0.58108,0.59253,0.60189,0.60890,0.61344,0.61552,0.61525,0.61280,0.60837},
    {0.63258,0.61970,0.60507,0.58890,0.57125,0.55217,0.53178,0.51039,0.48854,0.46699,0.44657,0.42804,0.41193,0.39851,0.38787,0.38006,0.37526,0.37389,0.37653,0.38383,0.39624,0.41387,0.43636,0.46284,0.49210,0.52268,0.55299,0.58146,0.60660,0.62725,0.64260,0.65237,0.65672,0.65618,0.65146,0.64336,0.63258},
    {0.62095,0.60243,0.58290,0.56243,0.54073,0.51730,0.49170,0.46391,0.43471,0.40564,0.37864,0.35546,0.33702,0.32321,0.31311,0.30575,0.30078,0.29875,0.30106,0.30949,0.32559,0.34998,0.38207,0.42017,0.46201,0.50518,0.54726,0.58585,0.61871,0.64402,0.66081,0.66905,0.66955,0.66358,0.65259,0.63798,0.62095},
    {0.58620,0.56373,0.54114,0.51857,0.49563,0.47134,0.44448,0.41429,0.38135,0.34789,0.31726,0.29272,0.27609,0.26667,0.26176,0.25848,0.25542,0.25307,0.25351,0.26004,0.27617,0.30384,0.34230,0.38856,0.43871,0.48915,0.53690,0.57926,0.61368,0.63832,0.65262,0.65718,0.65336,0.64282,0.62713,0.60777,0.58620},
    {0.54054,0.51627,0.49228,0.46880,0.44572,0.42234,0.39721,0.36894,0.33742,0.30457,0.27434,0.25147,0.23907,0.23614,0.23823,0.24087,0.24203,0.24166,0.24097,0.24339,0.25474,0.27997,0.31964,0.36973,0.42394,0.47661,0.52408,0.56374,0.59341,0.61236,0.62157,0.62234,0.61581,0.60319,0.58561,0.56419,0.54054},
    {0.48867,0.46503,0.44165,0.41863,0.39623,0.37454,0.35283,0.32976,0.30421,0.27678,0.25090,0.23196,0.22388,0.22576,0.23267,0.24037,0.24755,0.25361,0.25697,0.25833,0.26348,0.28050,0.31390,0.36106,0.41386,0.46412,0.50708,0.53992,0.56069,0.57069,0.57365,0.57144,0.56381,0.55092,0.53338,0.51203,0.48867},
    {0.43253,0.41175,0.39135,0.37126,0.35187,0.33373,0.31703,0.30085,0.28340,0.26400,0.24494,0.23085,0.22546,0.22878,0.23747,0.24859,0.26158,0.27535,0.28592,0.29056,0.29243,0.29975,0.32084,0.35718,0.40141,0.44430,0.48014,0.50509,0.51651,0.51743,0.51457,0.51016,0.50215,0.48970,0.47325,0.45356,0.43253},
    {0.37920,0.36357,0.34861,0.33428,0.32094,0.30906,0.29901,0.29027,0.28111,0.27011,0.25800,0.24747,0.24170,0.24271,0.25038,0.26289,0.27848,0.29521,0.30916,0.31675,0.31844,0.31999,0.33017,0.35323,0.38456,0.41649,0.44359,0.46117,0.46580,0.46094,0.45426,0.44791,0.43910,0.42690,0.41213,0.39570,0.37920},
    {0.34153,0.33271,0.32464,0.31758,0.31215,0.30839,0.30613,0.30492,0.30318,0.29885,0.29113,0.28131,0.27228,0.26771,0.27053,0.28020,0.29350,0.30751,0.31975,0.32797,0.33146,0.33325,0.33952,0.35374,0.37352,0.39442,0.41263,0.42404,0.42554,0.41934,0.41078,0.40157,0.39035,0.37734,0.36408,0.35186,0.34153},
    {0.32870,0.32607,0.32446,0.32437,0.32687,0.33172,0.33779,0.34389,0.34801,0.34747,0.34080,0.32911,0.31570,0.30498,0.30097,0.30427,0.31213,0.32189,0.33188,0.34049,0.34689,0.35278,0.36094,0.37200,0.38471,0.39787,0.40970,0.41726,0.41829,0.41286,0.40244,0.38841,0.37237,0.35653,0.34312,0.33381,0.32870},
    {0.34033,0.34108,0.34427,0.35011,0.35952,0.37200,0.38564,0.39812,0.40667,0.40818,0.40093,0.38646,0.36911,0.35402,0.34504,0.34287,0.34584,0.35257,0.36183,0.37142,0.38023,0.38954,0.40019,0.41097,0.42118,0.43143,0.44120,0.44828,0.45031,0.44540,0.43237,0.41263,0.39030,0.36959,0.35349,0.34378,0.34033},
    {0.37286,0.37425,0.38045,0.39099,0.40563,0.42319,0.44141,0.45760,0.46878,0.47164,0.46425,0.44822,0.42852,0.41089,0.39901,0.39341,0.39308,0.39740,0.40545,0.41491,0.42425,0.43421,0.44546,0.45716,0.46876,0.48057,0.49201,0.50099,0.50465,0.49993,0.48499,0.46152,0.43456,0.40947,0.38986,0.37762,0.37286},
    {0.42301,0.42396,0.43139,0.44439,0.46152,0.48059,0.49920,0.51504,0.52566,0.52833,0.52129,0.50580,0.48620,0.46770,0.45379,0.44539,0.44205,0.44336,0.44854,0.45590,0.46409,0.47337,0.48463,0.49801,0.51297,0.52857,0.54321,0.55444,0.55930,0.55497,0.54021,0.51688,0.48967,0.46376,0.44295,0.42921,0.42301},
    {0.48379,0.48443,0.49110,0.50289,0.51807,0.53427,0.54920,0.56099,0.56797,0.56854,0.56177,0.54851,0.53155,0.51445,0.50002,0.48959,0.48345,0.48151,0.48332,0.48795,0.49463,0.50341,0.51494,0.52962,0.54684,0.56497,0.58158,0.59393,0.59937,0.59603,0.58366,0.56421,0.54142,0.51947,0.50159,0.48950,0.48379},
    {0.53971,0.53998,0.54408,0.55129,0.56040,0.56987,0.57817,0.58402,0.58645,0.58476,0.57870,0.56880,0.55636,0.54317,0.53091,0.52082,0.51365,0.50978,0.50925,0.51182,0.51725,0.52552,0.53678,0.55096,0.56731,0.58420,0.59944,0.61067,0.61601,0.61448,0.60643,0.59353,0.57838,0.56374,0.55175,0.54361,0.53971},
    {0.57270,0.57191,0.57270,0.57465,0.57722,0.57979,0.58173,0.58248,0.58164,0.57897,0.57445,0.56828,0.56094,0.55308,0.54545,0.53875,0.53362,0.53052,0.52975,0.53146,0.53567,0.54232,0.55121,0.56189,0.57362,0.58531,0.59567,0.60348,0.60783,0.60838,0.60542,0.59985,0.59290,0.58587,0.57979,0.57531,0.57270},
    {0.57801,0.57670,0.57563,0.57472,0.57387,0.57296,0.57187,0.57049,0.56875,0.56662,0.56412,0.56133,0.55839,0.55547,0.55278,0.55055,0.54900,0.54830,0.54859,0.54996,0.55239,0.55581,0.56003,0.56482,0.56984,0.57472,0.57910,0.58266,0.58517,0.58654,0.58683,0.58619,0.58487,0.58317,0.58133,0.57957,0.57801},
    {0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621}
};
GPSMAG gpsmag(UART4,(char*) "GPSMAG",(char*) "+X-Y-Z");
/***********************************驱动程序*******************************************/
void UART4_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_IDLE(gpsmag.huart))
	{
		LL_USART_ClearFlag_IDLE(gpsmag.huart);
		LL_DMA_DisableStream(DMA1,LL_DMA_STREAM_2);
		LL_DMA_ClearFlag_DME2(DMA1);
		LL_DMA_ClearFlag_HT2(DMA1);
		LL_DMA_ClearFlag_TC2(DMA1);
		LL_DMA_ClearFlag_TE2(DMA1);
		LL_DMA_ClearFlag_FE2(DMA1);
		LL_USART_ClearFlag_CM(UART4);
		LL_USART_ClearFlag_EOB(UART4);
		LL_USART_ClearFlag_FE(UART4);
		LL_USART_ClearFlag_LBD(UART4);
		LL_USART_ClearFlag_NE(UART4);
		LL_USART_ClearFlag_ORE(UART4);
		LL_USART_ClearFlag_PE(UART4);
		LL_USART_ClearFlag_RTO(UART4);
		LL_USART_ClearFlag_TC(UART4);
		LL_USART_ClearFlag_WKUP(UART4);
		LL_USART_ClearFlag_nCTS(UART4);
		LL_USART_ClearFlag_IDLE(UART4);
		gpsmag.RxDataSize = GPS_RX_LEN - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_2);
		do{
			if(gpsmag.RxRawDat[0]!='$'|| gpsmag.RxRawDat[gpsmag.RxDataSize-1]!='\n') break;
			if(gpsmag.LockRx == HAL_LOCKED) break;
			gpsmag.LockRx = HAL_LOCKED;
			memcpy(gpsmag.RxDat, gpsmag.RxRawDat, gpsmag.RxDataSize);
			gpsmag.RxDat[gpsmag.RxDataSize] = 0;
			gpsmag.LockRx = HAL_UNLOCKED;
			gpsmag.RxFlag = true;   //收到完整一帧

			BaseType_t YieldRequired = xTaskResumeFromISR(gpsmagTaskHandle);
			if(YieldRequired==pdTRUE)
			{
				/*如果函数xTaskResumeFromISR()返回值为pdTRUE，那么说明要恢复的这个
				任务的任务优先级等于或者高于正在运行的任务(被中断打断的任务),所以在
				退出中断的时候一定要进行上下文切换！*/
				portYIELD_FROM_ISR(YieldRequired);
			}
		}while(0);
		LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_2, GPS_RX_LEN);
		LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_2);
	}
}

//bool TxFlag;
void DMA1_Stream4_IRQHandler(void)  //发送DMA中断
{
	LL_DMA_DisableStream(DMA1,LL_DMA_STREAM_4);
	LL_DMA_ClearFlag_TC4(DMA1);
	gpsmag.TxFlag = false;
}

bool uart4_Send_DMA(uint8_t * pData,uint16_t Size)	//DMA发送
{
//	static int error_cnt = 0;
//	if(error_cnt>10)
//	{
//		LL_DMA_DisableStream(DMA1,LL_DMA_STREAM_4);
//		LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_4, (uint32_t)UART4->TDR);
//		LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4);
//		error_cnt = 0;
//	}

	if(gpsmag.TxFlag == true)
	{
//		error_cnt++;
		return false;	//串口发送忙,放弃发送该帧数据
	}

	LL_DMA_DisableStream(DMA1,LL_DMA_STREAM_4);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, Size);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_4, (uint32_t)pData);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4);
	gpsmag.TxFlag = true;
	return true;
}
void GPSMAG::GpsMag_Init(void)
{
	LockRx = HAL_UNLOCKED;
	RxFlag = false;
	TxFlag = false;
	RxDataSize = 0;
	executionTime_us = 0;
	Sta = STA_INI;
	Err = ERR_NONE;

	MagUpdate = false;
	MagErr = ERR_NONE;
	MagSta = STA_INI;
	if(Dir_Trans(Dir, DirChr)==false) Err = ERR_SOFT;

	MagOff[0] = 59.7f;//1509.6f;
	MagOff[1] = -271.9;//-494.4f;
	MagOff[2] = -2492.2;//-1942.4f;	//RTK4

	for(u8 i=0;i<3;i++)
	{
		Re2t[i][0] = 0.0f; Re2t[i][1] = 0.0f;Re2t[i][2] = 0.0f;
	}

	GpsUpdate = false;
	ECEF_Init_Flag = false;

	gpsPosAccuracy = 10;
	gpsHeiAccuracy = 10;
	gpsSpdAccuracy = 10;
	star = 0;
	lat = 0;
	lng = 0;
	alti = 0;
	gps_update_time_us = 0;
	mag_update_time_us = 0;

	gps.timestamp = gps_update_time_us;
	mag.timestamp = mag_update_time_us;
	gps.star = star;
	gps.lat = lat;
	gps.lng = lng;
	gps.alti = alti;
	gps.gpsPosAccuracy = gpsPosAccuracy;
	gps.gpsHeiAccuracy = gpsHeiAccuracy;
	gps.gpsSpdAccuracy = gpsSpdAccuracy;
	gps.isReady = false;
	for(uint8_t i=0;i<3;i++)
	{
		gps.NED[i] = NED[i] = 0;
		gps.NED_spd[i] = NED_spd[i] = 0;
		mag.MagRel[i] = MagRel[i] = 0;
		gps.Zero_LLH[i] = Zero_LLH[i] = 0;
	}
	xQueueOverwrite(queueGps,&gps);
//	xQueueOverwrite(queueMag,&mag);
	osDelay(250);

	/* 配置接收DMA */
	LL_DMA_DisableStream(DMA1,LL_DMA_STREAM_2);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_2, (uint32_t)&huart->RDR);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_2, (uint32_t)RxRawDat);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_2, GPS_RX_LEN);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_2);
	/* 配置接收DMA */

	/* 配置发送DMA */
	LL_DMA_DisableStream(DMA1,LL_DMA_STREAM_4);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_4, (uint32_t)&huart->TDR);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_4, (uint32_t)TxDat);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, GPS_TX_LEN);
	LL_DMA_ClearFlag_TC4(DMA1);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_4);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4);
	/* 配置发送DMA */

	LL_DMA_ClearFlag_DME2(DMA1);
	LL_DMA_ClearFlag_HT2(DMA1);
	LL_DMA_ClearFlag_TC2(DMA1);
	LL_DMA_ClearFlag_TE2(DMA1);
	LL_DMA_ClearFlag_FE2(DMA1);

	LL_USART_ClearFlag_CM(huart);
	LL_USART_ClearFlag_EOB(huart);
	LL_USART_ClearFlag_FE(huart);
	LL_USART_ClearFlag_LBD(huart);
	LL_USART_ClearFlag_NE(huart);
	LL_USART_ClearFlag_ORE(huart);
	LL_USART_ClearFlag_PE(huart);
	LL_USART_ClearFlag_RTO(huart);
	LL_USART_ClearFlag_TC(huart);
	LL_USART_ClearFlag_WKUP(huart);
	LL_USART_ClearFlag_nCTS(huart);
	LL_USART_ClearFlag_IDLE(huart);

	LL_USART_EnableDMAReq_RX(huart);
	LL_USART_EnableDMAReq_TX(huart);
	LL_USART_ClearFlag_IDLE(huart);
	LL_USART_EnableIT_IDLE(huart);
	Sta = STA_RUN;
	MagSta = STA_RUN;
	GpsCal = true;
	osDelay(250);
}

bool GPSMAG::Gps_Calc()
{
	if(Sta == STA_INI)return false;

	startTimerLast = startTimer;
	getTimer_us(&startTimer);
	cycleTime_us = startTimer - startTimerLast;

	if(RxFlag == false)
	{
		return false;	//无GPS更新
	}

	RxFlag = false;

	if(LockRx == HAL_LOCKED)return false;
	LockRx = HAL_LOCKED;

	bool GpsFlag = false,MagFlag = false;

	do{
		char *RxType[4];
		RxType[0]=(char*)RxDat-1;  //第一个存放RxDat的前一个地址以便和下面的格式一致
		u8 num = StrSeg((char*)RxDat,'\n',&RxType[1],3);
		if(num==0)break;   //分割不同类型，这里只有$GPGGA,$GPVTG,$HMC,$GPS 4种
		for(u8 i=0;i<num;i++)
		{
			char *RxPara[17];
			char Dat[GPS_RX_LEN];
			if(*(RxType[i]+4)=='G')
			{
				//$GPGGA,102134.00,1306.9847,N,11402.2986,W,2,5,1.0,50.3,M,-16.27,M,,*61\r\n
				memcpy(Dat,RxType[i]+1,RxType[i+1]-RxType[i]);
				if(StrSeg(Dat,',',RxPara,14)!=14)break;
				time=atof(RxPara[0]);         //第二位为时间hhmmss.ss
				int time_int=(int)time;
				time = (int)(time_int/10000*3600+time_int%10000/100*60+time_int%100)+(time-time_int);   //时分秒转化成秒
				lat = atof(RxPara[1]);        //第三位纬度，格式为ddmm.mmmm
				int lat_int=(int)lat;
				lat = (int)(lat_int/100)+atof(RxPara[1]+2)/60.0f;
				if(*(RxPara[2])=='S') lat=-lat;    //第四位纬度N或S（南纬或北纬）
				lng = atof(RxPara[3]);         //第五位经度，格式dddmm.mmmm
				int lng_int=(int)lng;
				lng = (int)(lng_int/100)+atof(RxPara[3]+3)/60.0f;
				if(*(RxPara[4])=='W') lng=-lng;    //第六位经度E或W（东经或西经）
				status=atoi(RxPara[5]);           //第七位GPS状态
				star=atoi(RxPara[6]);          //第八位星数
				hdop=atof(RxPara[7]);          //第九位水平精度
				alti=atof(RxPara[8]);          //第十位椭球高
				wgs_alt=atoi(RxPara[10]);      //第十二位水准平面高
				GpsFlag = true;
			}
			if(*(RxType[i]+4)=='V')
			{
				//$GPVTG,<1>,T,<2>,M,<3>,N,<4>,K,<5>*hh\r\n
				memcpy(Dat,RxType[i]+1,RxType[i+1]-RxType[i]);
				if(StrSeg(Dat,',',RxPara,9)!=9)break;
				vtg_dir = atof(RxPara[0]);     //第二位为真北方向
				vtg_spd = atof(RxPara[6]);     //第八位为地面速率，单位：公里/小时
				GpsFlag = true;
			}
			if(*(RxType[i]+4)=='C')
			{
				//$HMC,mx,my,mz\r\n
				memcpy(Dat,RxType[i]+1,RxType[i+1]-RxType[i]);
				if(StrSeg(Dat,',',RxPara,3)!=3)break;
				MagRaw[0] = atoi(RxPara[0]);
				MagRaw[1] = atoi(RxPara[1]);
				MagRaw[2] = atoi(RxPara[2]);
				MagFlag = true;
			}
			if(*(RxType[i]+4)=='S')
			{
				//$GPS,%1d%1d,%d,%d,%d,%f,%f,%f,%f,%d\r\n\r\n
				memcpy(Dat,RxType[i]+1,RxType[i+1]-RxType[i]);
				if(StrSeg(Dat,',',RxPara,17)!=17)break;
				if(*(RxPara[0])=='1'){
					MagFlag=true;
					MagRaw[0] = atoi(RxPara[1]);
					MagRaw[1] = atoi(RxPara[2]);
					MagRaw[2] = atoi(RxPara[3]);
					getTimer_us(&mag_update_time_us);
					MagUpdate = true;
				}
				else
				{
					MagUpdate = false;
				}

				if(*(RxPara[0]+1)=='1'){
					GpsFlag = true;
					lat = atof(RxPara[4]);
					lng = atof(RxPara[5]);
					NED_spd[0] = atof(RxPara[6]);
					NED_spd[1] = atof(RxPara[7]);
					NED_spd[2] = atof(RxPara[8]);
					alti = atof(RxPara[9]);
					star = atof(RxPara[10]);
					gpsPosAccuracy = atof(RxPara[11]);
					gpsHeiAccuracy = atof(RxPara[12]);//..高度精度
					gpsSpdAccuracy = atof(RxPara[13]);
					gps_headAccuracy = atof(RxPara[14]);
					gps_heading =  atof(RxPara[15]);
					status = atoi(RxPara[16]);
					getTimer_us(&gps_update_time_us);
					GpsUpdate = true;
				}
			}
		}
	}while(0);
	LockRx = HAL_UNLOCKED;

	if(MagFlag==true)
	{
		//方向变换
		MagRot[0]=Dir[0]*(MagRaw[Dir[1]]-MagOff[Dir[1]]);
		MagRot[1]=Dir[2]*(MagRaw[Dir[3]]-MagOff[Dir[3]]);
		MagRot[2]=Dir[4]*(MagRaw[Dir[5]]-MagOff[Dir[5]]);
		//实际值转换
		MagFil[0]=MagRel[0]=MagRot[0];
		MagFil[1]=MagRel[1]=MagRot[1];
		MagFil[2]=MagRel[2]=MagRot[2];

		if(MagSta == STA_RUN)MagUpdate = true;
	//	float sqrNorm=SQR(MagFil[0])+SQR(MagFil[1])+SQR(MagFil[2]);
	//	if(sqrNorm>360000||sqrNorm<40000)MagUpdate = false;
	}
	static u8 count=0;

	if(GpsFlag == true)
	{
		//是否收到有效数据
		if(star<6)
		{
			Err = ERR_UPDT;
			return false;
		}
		//初始化ECEF坐标系
		if (!ECEF_Init_Flag)
		{
			Gps_Cali();
			if (!ECEF_Init_Flag)
				return false;
		}

		if(Sta==STA_RUN)Update = true;


		//转北东地坐标
		LLH[0] = lat * D2R;
		LLH[1] = lng * D2R;
		LLH[2] = alti;

		N = C_WGS84_a/sqrt( 1.0 - SQR(C_WGS84_e)*SQR(sin(LLH[0])));
		ECFF[0]= (N + LLH[2]) * cos(LLH[0]) * cos(LLH[1]);
		ECFF[1]= (N + LLH[2]) * cos(LLH[0]) * sin(LLH[1]);
		ECFF[2]= (N*(1-SQR(C_WGS84_e)) +LLH[2]) * sin(LLH[0]);

		ECFF[0] -= Zero_ECFF[0];
		ECFF[1] -= Zero_ECFF[1];
		ECFF[2] -= Zero_ECFF[2];

		NED[0] = Re2t[0][0]*ECFF[0]+Re2t[0][1]*ECFF[1]+Re2t[0][2]*ECFF[2];
		NED[1] = Re2t[1][0]*ECFF[0]+Re2t[1][1]*ECFF[1]+Re2t[1][2]*ECFF[2];
		NED[2] = Re2t[2][0]*ECFF[0]+Re2t[2][1]*ECFF[1]+Re2t[2][2]*ECFF[2];
	}
	else
	{
		if(++count>20)
		{
			count = 0;
			GpsUpdate = false;
		}
	}
	gps.timestamp = gps_update_time_us;
	mag.timestamp = mag_update_time_us;

	gps.status = status;
	gps.star = star;
	gps.lat = lat;
	gps.lng = lng;
	gps.alti = alti;
	gps.gpsPosAccuracy = gpsPosAccuracy;
	gps.gpsHeiAccuracy = gpsHeiAccuracy;
	gps.gpsSpdAccuracy = gpsSpdAccuracy;
	gps.isReady = ECEF_Init_Flag;
	gps.update = GpsUpdate;
	for(uint8_t i=0;i<3;i++)
	{
		gps.NED[i] = NED[i];
		gps.NED_spd[i] = NED_spd[i];
		gps.Zero_LLH[i] = Zero_LLH[i];
		mag.MagRaw[i] = MagRaw[i];
		mag.MagRel[i] = MagRel[i];
	}
	xQueueOverwrite(queueGps,&gps);
//	xQueueOverwrite(queueMag,&mag);
	getTimer_us(&stopTimer);
	executionTime_us = stopTimer - startTimer;
	return Update;
}


bool GPSMAG::Gps_Cali()		//获取飞机初始位置（原点）
{
	static bool GpsIsCali = false;
	if(GpsCal==true)
	{
		GpsCal = false;
		Sta = STA_CAL;
		GpsIsCali = true;
	}
	if(GpsIsCali == false)return true;
// 下次进来就不用再等
	if(star<=18||gpsPosAccuracy > 0.1f)
	{
			return false;
	}

	/****计算磁偏角****/
	yaw_declination = get_declination(lat,lng);
	/****计算位置*****/
	LLH_Init[0] = lat*D2R;         //纬度
	LLH_Init[1] = lng*D2R;         //经度
	LLH_Init[2] = alti;  			//高度
	Zero_LLH[0] = lat;
	Zero_LLH[1] = lng;
	Zero_LLH[2] = alti;

	//LLH转ECEF（地球中心坐标系）
	N_Init = C_WGS84_a/sqrt( 1.0 - SQR(C_WGS84_e)*SQR(sin(LLH_Init[0])));		//地球椭圆体法线长度
	ECFF_Init[0]= (N_Init + LLH_Init[2]) * cos(LLH_Init[0]) * cos(LLH_Init[1]);
	ECFF_Init[1]= (N_Init + LLH_Init[2]) * cos(LLH_Init[0]) * sin(LLH_Init[1]);
	ECFF_Init[2]= (N_Init * (1-SQR(C_WGS84_e)) +LLH_Init[2]) * sin(LLH_Init[0]);
	Zero_ECFF[0] = ECFF_Init[0];
	Zero_ECFF[1] = ECFF_Init[1];
	Zero_ECFF[2] = ECFF_Init[2];
	//Re2t矩阵，ECEF转NED矩阵，因为没飞那么远，认为该矩阵不变
	double clat=cos(lat*D2R),slat=sin(lat*D2R),clng=cos(lng*D2R),slng=sin(lng*D2R);
	Re2t[0][0] = -slat*clng;
	Re2t[0][1] = -slat*slng;
	Re2t[0][2] =  clat;
	Re2t[1][0] = -slng;
	Re2t[1][1] =  clng;
	Re2t[1][2] =  0.0;
	Re2t[2][0] = -clat*clng;
	Re2t[2][1] = -clat*slng;
	Re2t[2][2] = -slat;
	alti_off = alti;
	Sta=STA_RUN;
	GpsIsCali = false;
	ECEF_Init_Flag = true;
	return true;
}

//OrdinaryGps OrdinaryGps(USART6,(char*) "OrdinaryGps",(char*) "+X-Y-Z");
//void USART6_IRQHandler(void)
//{
//	if(LL_USART_IsActiveFlag_IDLE(OrdinaryGps.huart))
//	{
//		LL_USART_ClearFlag_IDLE(OrdinaryGps.huart);
//		LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_1);
//		LL_DMA_ClearFlag_DME1(DMA2);
//		LL_DMA_ClearFlag_HT1(DMA2);
//		LL_DMA_ClearFlag_TC1(DMA2);
//		LL_DMA_ClearFlag_TE1(DMA2);
//		LL_DMA_ClearFlag_FE1(DMA2);
//
//		LL_USART_ClearFlag_CM(USART6);
//		LL_USART_ClearFlag_EOB(USART6);
//		LL_USART_ClearFlag_FE(USART6);
//		LL_USART_ClearFlag_LBD(USART6);
//		LL_USART_ClearFlag_NE(USART6);
//		LL_USART_ClearFlag_ORE(USART6);
//		LL_USART_ClearFlag_PE(USART6);
//		LL_USART_ClearFlag_RTO(USART6);
//		LL_USART_ClearFlag_TC(USART6);
//		LL_USART_ClearFlag_WKUP(USART6);
//		LL_USART_ClearFlag_nCTS(USART6);
//		LL_USART_ClearFlag_IDLE(USART6);
//		OrdinaryGps.RxDataSize = GPS_RX_LEN - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_1);
//
//		do{
//			if(OrdinaryGps.RxRawDat[0]!='$'|| OrdinaryGps.RxRawDat[OrdinaryGps.RxDataSize-1]!='\n') break;
//			if(OrdinaryGps.LockRx == HAL_LOCKED) break;
//			OrdinaryGps.LockRx = HAL_LOCKED;
//			memcpy(OrdinaryGps.RxDat, OrdinaryGps.RxRawDat, OrdinaryGps.RxDataSize);
//			OrdinaryGps.RxDat[OrdinaryGps.RxDataSize] = 0;
//			OrdinaryGps.LockRx = HAL_UNLOCKED;
//			OrdinaryGps.RxFlag = true;   //收到完整一帧
//
//			BaseType_t YieldRequired = xTaskResumeFromISR(OrdinaryGpsTaskHandle);
//			if(YieldRequired==pdTRUE)
//			{
//				/*如果函数xTaskResumeFromISR()返回值为pdTRUE，那么说明要恢复的这个
//				任务的任务优先级等于或者高于正在运行的任务(被中断打断的任务),所以在
//				退出中断的时候一定要进行上下文切换！*/
//				portYIELD_FROM_ISR(YieldRequired);
//			}
//		}while(0);
//		LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, GPS_RX_LEN);
//		LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
//	}
//}
////bool TxFlag;
//void DMA2_Stream6_IRQHandler(void)  //发送DMA中断
//{
//	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_6);
//	LL_DMA_ClearFlag_TC6(DMA2);
//	OrdinaryGps.TxFlag = false;
//}
//
//bool usart6_Send_DMA(uint8_t * pData,uint16_t Size)	//DMA发送
//{
//	if(OrdinaryGps.TxFlag == true) return false;	//串口发送忙,放弃发送该帧数据
//	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_6);
//	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_6, Size);
//	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_6, (uint32_t)pData);
//	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_6);
//	OrdinaryGps.TxFlag = true;
//	return true;
//}
//void OrdinaryGps::GpsMag_Init(void)
//{
//	LockRx = HAL_UNLOCKED;
//	RxFlag = false;
//	TxFlag = false;
//	RxDataSize = 0;
//	executionTime_us = 0;
//	Sta = STA_INI;
//	Err = ERR_NONE;
//
//	MagUpdate = false;
//	MagErr = ERR_NONE;
//	MagSta = STA_INI;
//	if(Dir_Trans(Dir, DirChr)==false) Err = ERR_SOFT;
//
//	MagOff[0] = 59.7f;//1509.6f;
//	MagOff[1] = -271.9;//-494.4f;
//	MagOff[2] = -2492.2;//-1942.4f;	//RTK4
//
//	for(u8 i=0;i<3;i++)
//	{
//		Re2t[i][0] = 0.0f; Re2t[i][1] = 0.0f;Re2t[i][2] = 0.0f;
//	}
//
//	GpsUpdate = false;
//	ECEF_Init_Flag = false;
//
//	gpsPosAccuracy = 10;
//	gpsHeiAccuracy = 10;
//	gpsSpdAccuracy = 10;
//	star = 0;
//	lat = 0;
//	lng = 0;
//	alti = 0;
//	gps_update_time_us = 0;
//	mag_update_time_us = 0;
//
//	gps.timestamp = gps_update_time_us;
//	mag.timestamp = mag_update_time_us;
//	gps.star = star;
//	gps.lat = lat;
//	gps.lng = lng;
//	gps.alti = alti;
//	gps.gpsPosAccuracy = gpsPosAccuracy;
//	gps.gpsHeiAccuracy = gpsHeiAccuracy;
//	gps.gpsSpdAccuracy = gpsSpdAccuracy;
//	gps.isReady = false;
//	for(uint8_t i=0;i<3;i++)
//	{
//		gps.NED[i] = NED[i] = 0;
//		gps.NED_spd[i] = NED_spd[i] = 0;
//		mag.MagRel[i] = MagRel[i] = 0;
//		gps.Zero_LLH[i] = Zero_LLH[i] = 0;
//	}
//	xQueueOverwrite(queueOrdinaryGps,&gps);
////	xQueueOverwrite(queueMag,&mag);
//
//	osDelay(250);
//
//	/* 配置接收DMA */
//	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_1);
//	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_1, (uint32_t)&huart->RDR);
//	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_1, (uint32_t)RxRawDat);
//	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, GPS_RX_LEN);
//	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
//	/* 配置接收DMA */
//
//	/* 配置发送DMA */
//	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_6);
//	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_6, (uint32_t)&huart->TDR);
//	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_6, (uint32_t)TxDat);
//	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_6, GPS_TX_LEN);
//	LL_DMA_ClearFlag_TC6(DMA2);
//	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_6);
//	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_6);
//	/* 配置发送DMA */
//
//	LL_DMA_ClearFlag_DME1(DMA2);
//	LL_DMA_ClearFlag_HT1(DMA2);
//	LL_DMA_ClearFlag_TC1(DMA2);
//	LL_DMA_ClearFlag_TE1(DMA2);
//	LL_DMA_ClearFlag_FE1(DMA2);
//
//	LL_USART_ClearFlag_CM(huart);
//	LL_USART_ClearFlag_EOB(huart);
//	LL_USART_ClearFlag_FE(huart);
//	LL_USART_ClearFlag_LBD(huart);
//	LL_USART_ClearFlag_NE(huart);
//	LL_USART_ClearFlag_ORE(huart);
//	LL_USART_ClearFlag_PE(huart);
//	LL_USART_ClearFlag_RTO(huart);
//	LL_USART_ClearFlag_TC(huart);
//	LL_USART_ClearFlag_WKUP(huart);
//	LL_USART_ClearFlag_nCTS(huart);
//	LL_USART_ClearFlag_IDLE(huart);
//
//	LL_USART_EnableDMAReq_RX(huart);
//	LL_USART_EnableDMAReq_TX(huart);
//	LL_USART_ClearFlag_IDLE(huart);
//	LL_USART_EnableIT_IDLE(huart);
//	Sta = STA_RUN;
//	MagSta = STA_RUN;
//	GpsCal = true;
//	osDelay(250);
//}
//bool OrdinaryGps::Gps_Calc()
//{
//	if(Sta == STA_INI)return false;
//	startTimerLast = startTimer;
//	getTimer_us(&startTimer);
//	cycleTime_us = startTimer - startTimerLast;
//
//	if(RxFlag == false)
//	{
//		return false;	//无GPS更新
//	}
//	RxFlag = false;
//	if(LockRx == HAL_LOCKED)return false;
//	LockRx = HAL_LOCKED;
//	bool GpsFlag = false,MagFlag = false;
//	do{
//		char *RxType[4];
//		RxType[0]=(char*)RxDat-1;  //第一个存放RxDat的前一个地址以便和下面的格式一致
//		u8 num = StrSeg((char*)RxDat,'\n',&RxType[1],3);
//		if(num==0)break;   //分割不同类型，这里只有$GPGGA,$GPVTG,$HMC,$GPS 4种
//		for(u8 i=0;i<num;i++)
//		{
//			char *RxPara[17];
//			char Dat[GPS_RX_LEN];
//			if(*(RxType[i]+4)=='G')
//			{
//				//$GPGGA,102134.00,1306.9847,N,11402.2986,W,2,5,1.0,50.3,M,-16.27,M,,*61\r\n
//				memcpy(Dat,RxType[i]+1,RxType[i+1]-RxType[i]);
//				if(StrSeg(Dat,',',RxPara,14)!=14)break;
//				time=atof(RxPara[0]);         //第二位为时间hhmmss.ss
//				int time_int=(int)time;
//				time = (int)(time_int/10000*3600+time_int%10000/100*60+time_int%100)+(time-time_int);   //时分秒转化成秒
//				lat = atof(RxPara[1]);        //第三位纬度，格式为ddmm.mmmm
//				int lat_int=(int)lat;
//				lat = (int)(lat_int/100)+atof(RxPara[1]+2)/60.0f;
//				if(*(RxPara[2])=='S') lat=-lat;    //第四位纬度N或S（南纬或北纬）
//				lng = atof(RxPara[3]);         //第五位经度，格式dddmm.mmmm
//				int lng_int=(int)lng;
//				lng = (int)(lng_int/100)+atof(RxPara[3]+3)/60.0f;
//				if(*(RxPara[4])=='W') lng=-lng;    //第六位经度E或W（东经或西经）
//				status=atoi(RxPara[5]);           //第七位GPS状态
//				star=atoi(RxPara[6]);          //第八位星数
//				hdop=atof(RxPara[7]);          //第九位水平精度
//				alti=atof(RxPara[8]);          //第十位椭球高
//				wgs_alt=atoi(RxPara[10]);      //第十二位水准平面高
//				GpsFlag = true;
//			}
//			if(*(RxType[i]+4)=='V')
//			{
//				//$GPVTG,<1>,T,<2>,M,<3>,N,<4>,K,<5>*hh\r\n
//				memcpy(Dat,RxType[i]+1,RxType[i+1]-RxType[i]);
//				if(StrSeg(Dat,',',RxPara,9)!=9)break;
//				vtg_dir = atof(RxPara[0]);     //第二位为真北方向
//				vtg_spd = atof(RxPara[6]);     //第八位为地面速率，单位：公里/小时
//				GpsFlag = true;
//			}
//			if(*(RxType[i]+4)=='C')
//			{
//				//$HMC,mx,my,mz\r\n
//				memcpy(Dat,RxType[i]+1,RxType[i+1]-RxType[i]);
//				if(StrSeg(Dat,',',RxPara,3)!=3)break;
//				MagRaw[0] = atoi(RxPara[0]);
//				MagRaw[1] = atoi(RxPara[1]);
//				MagRaw[2] = atoi(RxPara[2]);
//				MagFlag = true;
//			}
//			if(*(RxType[i]+4)=='S')
//			{
//				//$GPS,%1d%1d,%d,%d,%d,%f,%f,%f,%f,%d\r\n\r\n
//				memcpy(Dat,RxType[i]+1,RxType[i+1]-RxType[i]);
//				if(StrSeg(Dat,',',RxPara,17)!=17)break;
//				if(*(RxPara[0])=='1'){
//					MagFlag=true;
//					MagRaw[0] = atoi(RxPara[1]);
//					MagRaw[1] = atoi(RxPara[2]);
//					MagRaw[2] = atoi(RxPara[3]);
//					getTimer_us(&mag_update_time_us);
//					MagUpdate = true;
//				}
//				else{
//					MagUpdate = false;
//				}
//				if(*(RxPara[0]+1)=='1'){
//					GpsFlag = true;
//					lat = atof(RxPara[4]);
//					lng = atof(RxPara[5]);
//					NED_spd[0] = atof(RxPara[6]);
//					NED_spd[1] = atof(RxPara[7]);
//					NED_spd[2] = atof(RxPara[8]);
//					alti = atof(RxPara[9]);
//					star = atof(RxPara[10]);
//					gpsPosAccuracy = atof(RxPara[11]);
//					gpsHeiAccuracy = atof(RxPara[12]);//..高度精度
//					gpsSpdAccuracy = atof(RxPara[13]);
//					gps_headAccuracy = atof(RxPara[14]);
//					gps_heading =  atof(RxPara[15]);
//					status = atoi(RxPara[16]);
//					getTimer_us(&gps_update_time_us);
//					GpsUpdate = true;
//				}
//			}
//		}
//	}while(0);
//	LockRx = HAL_UNLOCKED;
//	if(GpsFlag == true)
//	{
//		//是否收到有效数据
//		if(star<6)
//		{
//			Err = ERR_UPDT;
//			return false;
//		}
//		//初始化ECEF坐标系
//		if (!ECEF_Init_Flag)
//		{
//			Gps_Cali();
//			if (!ECEF_Init_Flag)
//				return false;
//		}
//		if(Sta==STA_RUN)Update = true;
//		//转北东地坐标
//		LLH[0] = lat * D2R;
//		LLH[1] = lng * D2R;
//		LLH[2] = alti;
//
//		N = C_WGS84_a/sqrt( 1.0 - SQR(C_WGS84_e)*SQR(sin(LLH[0])));
//		ECFF[0]= (N + LLH[2]) * cos(LLH[0]) * cos(LLH[1]);
//		ECFF[1]= (N + LLH[2]) * cos(LLH[0]) * sin(LLH[1]);
//		ECFF[2]= (N*(1-SQR(C_WGS84_e)) +LLH[2]) * sin(LLH[0]);
//
//		ECFF[0] -= Zero_ECFF[0];
//		ECFF[1] -= Zero_ECFF[1];
//		ECFF[2] -= Zero_ECFF[2];
//
//		NED[0] = Re2t[0][0]*ECFF[0]+Re2t[0][1]*ECFF[1]+Re2t[0][2]*ECFF[2];
//		NED[1] = Re2t[1][0]*ECFF[0]+Re2t[1][1]*ECFF[1]+Re2t[1][2]*ECFF[2];
//		NED[2] = Re2t[2][0]*ECFF[0]+Re2t[2][1]*ECFF[1]+Re2t[2][2]*ECFF[2];
//	}
//
//	gps.timestamp = gps_update_time_us;
//	mag.timestamp = mag_update_time_us;
//
//	gps.status = status;
//	gps.star = star;
//	gps.lat = lat;
//	gps.lng = lng;
//	gps.alti = alti;
//	gps.gpsPosAccuracy = gpsPosAccuracy;
//	gps.gpsHeiAccuracy = gpsHeiAccuracy;
//	gps.gpsSpdAccuracy = gpsSpdAccuracy;
//	gps.isReady = ECEF_Init_Flag;
//	for(uint8_t i=0;i<3;i++)
//	{
//		gps.NED[i] = NED[i];
//		gps.NED_spd[i] = NED_spd[i];
//		gps.Zero_LLH[i] = Zero_LLH[i];
//		mag.MagRaw[i] = MagRaw[i];
//		mag.MagRel[i] = MagRel[i];
//	}
//	xQueueOverwrite(queueOrdinaryGps,&gps);
//	getTimer_us(&stopTimer);
//	executionTime_us = stopTimer - startTimer;
//	return Update;
//}
//
//
//bool OrdinaryGps::Gps_Cali()		//获取飞机初始位置（原点）
//{
//	static bool GpsIsCali = false;
//	if(GpsCal==true)
//	{
//		GpsCal = false;
//		Sta = STA_CAL;
//		GpsIsCali = true;
//	}
//	if(GpsIsCali == false)return true;
//// 下次进来就不用再等
//	if(star<=14||gpsPosAccuracy > 1.5f)//14-1.5
//	{
//			return false;
//	}
//
//	/****计算磁偏角****/
//	yaw_declination = get_declination(lat,lng);
//	/****计算位置*****/
//	LLH_Init[0] = lat*D2R;         //纬度
//	LLH_Init[1] = lng*D2R;         //经度
//	LLH_Init[2] = alti;  			//高度
//	Zero_LLH[0] = lat;
//	Zero_LLH[1] = lng;
//	Zero_LLH[2] = alti;
//
//	//LLH转ECEF（地球中心坐标系）
//	N_Init = C_WGS84_a/sqrt( 1.0 - SQR(C_WGS84_e)*SQR(sin(LLH_Init[0])));		//地球椭圆体法线长度
//	ECFF_Init[0]= (N_Init + LLH_Init[2]) * cos(LLH_Init[0]) * cos(LLH_Init[1]);
//	ECFF_Init[1]= (N_Init + LLH_Init[2]) * cos(LLH_Init[0]) * sin(LLH_Init[1]);
//	ECFF_Init[2]= (N_Init * (1-SQR(C_WGS84_e)) +LLH_Init[2]) * sin(LLH_Init[0]);
//	Zero_ECFF[0] = ECFF_Init[0];
//	Zero_ECFF[1] = ECFF_Init[1];
//	Zero_ECFF[2] = ECFF_Init[2];
//	//Re2t矩阵，ECEF转NED矩阵，因为没飞那么远，认为该矩阵不变
//	double clat=cos(lat*D2R),slat=sin(lat*D2R),clng=cos(lng*D2R),slng=sin(lng*D2R);
//	Re2t[0][0] = -slat*clng;
//	Re2t[0][1] = -slat*slng;
//	Re2t[0][2] =  clat;
//	Re2t[1][0] = -slng;
//	Re2t[1][1] =  clng;
//	Re2t[1][2] =  0.0;
//	Re2t[2][0] = -clat*clng;
//	Re2t[2][1] = -clat*slng;
//	Re2t[2][2] = -slat;
//	alti_off = alti;
//	Sta=STA_RUN;
//	GpsIsCali = false;
//	ECEF_Init_Flag = true;
//	return true;
//}
bool get_mag_field_ef(float latitude_deg, float longitude_deg, float *intensity_gauss, float *declination_deg, float *inclination_deg)
{
    bool valid_input_data = true;
    /* round down to nearest sampling resolution */
    int32_t min_lat = (int32_t)((int32_t)(latitude_deg / SAMPLING_RES) * SAMPLING_RES);
    int32_t min_lon = (int32_t)((int32_t)(longitude_deg / SAMPLING_RES) * SAMPLING_RES);

    /* for the rare case of hitting the bounds exactly
     * the rounding logic wouldn't fit, so enforce it.
     */

    /* limit to table bounds - required for maxima even when table spans full globe range */
    if (latitude_deg <= SAMPLING_MIN_LAT) {
        min_lat = (int32_t)(SAMPLING_MIN_LAT);
        valid_input_data = false;
    }

    if (latitude_deg >= SAMPLING_MAX_LAT) {
        min_lat = (int32_t)((int32_t)(latitude_deg / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES);
        valid_input_data = false;
    }

    if (longitude_deg <= SAMPLING_MIN_LON) {
        min_lon = (int32_t)(SAMPLING_MIN_LON);
        valid_input_data = false;
    }

    if (longitude_deg >= SAMPLING_MAX_LON) {
        min_lon = (int32_t)((int32_t)(longitude_deg / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES);
        valid_input_data = false;
    }

    /* find index of nearest low sampling point */
    uint32_t min_lat_index = (uint32_t)((-(SAMPLING_MIN_LAT) + min_lat)  / SAMPLING_RES);
    uint32_t min_lon_index = (uint32_t)((-(SAMPLING_MIN_LON) + min_lon) / SAMPLING_RES);

    /* calculate intensity */

    float data_sw = intensity_table[min_lat_index][min_lon_index];
    float data_se = intensity_table[min_lat_index][min_lon_index + 1];;
    float data_ne = intensity_table[min_lat_index + 1][min_lon_index + 1];
    float data_nw = intensity_table[min_lat_index + 1][min_lon_index];

    /* perform bilinear interpolation on the four grid corners */

    float data_min = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_se - data_sw) + data_sw;
    float data_max = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_ne - data_nw) + data_nw;

    *intensity_gauss = ((latitude_deg - min_lat) / SAMPLING_RES) * (data_max - data_min) + data_min;

    /* calculate declination */

    data_sw = declination_table[min_lat_index][min_lon_index];
    data_se = declination_table[min_lat_index][min_lon_index + 1];;
    data_ne = declination_table[min_lat_index + 1][min_lon_index + 1];
    data_nw = declination_table[min_lat_index + 1][min_lon_index];

    /* perform bilinear interpolation on the four grid corners */

    data_min = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_se - data_sw) + data_sw;
    data_max = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_ne - data_nw) + data_nw;

    *declination_deg = ((latitude_deg - min_lat) / SAMPLING_RES) * (data_max - data_min) + data_min;

    /* calculate inclination */
    data_sw = inclination_table[min_lat_index][min_lon_index];
    data_se = inclination_table[min_lat_index][min_lon_index + 1];;
    data_ne = inclination_table[min_lat_index + 1][min_lon_index + 1];
    data_nw = inclination_table[min_lat_index + 1][min_lon_index];

    /* perform bilinear interpolation on the four grid corners */
    data_min = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_se - data_sw) + data_sw;
    data_max = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_ne - data_nw) + data_nw;

    *inclination_deg = ((latitude_deg - min_lat) / SAMPLING_RES) * (data_max - data_min) + data_min;

    return valid_input_data;
}
/*
 calculate magnetic field intensity and orientation
*/
float get_declination(float latitude_deg, float longitude_deg)
{
    float declination_deg=0, inclination_deg=0, intensity_gauss=0;
    if(get_mag_field_ef(latitude_deg, longitude_deg, &intensity_gauss, &declination_deg, &inclination_deg)){
			return declination_deg;
		}
		else{
			return -2.85f;
		}
}

extern "C" void gpsmag_main(void *argument)
{
	osDelay(500);//等待系统完成初始化
	gpsmag.GpsMag_Init();	//参数初始化
	for(;;)
	{
		vTaskSuspend(gpsmagTaskHandle);
		gpsmag.Gps_Calc();
	}
}
//extern "C" void OrdinaryGps_main(void *argument)
//{
//	osDelay(500);//等待系统完成初始化
//	OrdinaryGps.GpsMag_Init();	//参数初始化
//	for(;;)
//	{
//		vTaskSuspend(OrdinaryGpsTaskHandle);
//		OrdinaryGps.Gps_Calc();
//	}
//}
