#include "ProbabilisticParser.h"

#include <map>
#include <string>

using namespace std;
using namespace hd;

map<string,string> idiomModelDesc;

void IdiomModelInit() {

    idiomModelDesc[string("gcc")]=string(
"12 \
P:320004fffe_24fffdffff 11.557510 \
P:18b00010004_24fffdffff 10.317698 \
P:18b00020004_24fffdffff 9.772404 \
P:18b00030004_24fffdffff 10.726118 \
P:18b00060004_24fffdffff 10.332256 \
P:18b00070004_24fffdffff 10.064688 \
1b500050004_10300010000 9.458438 \
1b500050004_10300050004 17.071176 \
1b500050004_aaaaffffffff_10300050004 15.485654 \
1f90004fffe_1b500050004 10.302958 \
1f90004fffe_aaaaffffffff_1b500050004 8.679270 \
21700050005_aaaaffffffff_390004fffe 12.309351 \
-6.222232");


}
