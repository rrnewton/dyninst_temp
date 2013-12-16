#include "ProbabilisticParser.h"

#include <map>
#include <string>

using namespace std;
using namespace hd;

map<string,string> idiomModelDesc;

void IdiomModelInit() {
    idiomModelDesc[string("32-bit-gcc")]=string(
"6 \
P:320004fffe_24fffdffff 8.810099 \
P:18b00030004_24fffdffff 7.794592 \
1b500050004_10300050004 14.488899 \
1b500050004_aaaaffffffff_10300050004 12.061317 \
1f90004fffe_1b500050004 8.240142 \
21700050005_18b00060004_10300010004 9.212425 \
-5.346398 \
0.46258599565");

    idiomModelDesc[string("32-bit-icc")]=string(
"9 \
P:70010ffff_130ffffffff 8.575495 \
P:24fffdffff_130ffffffff 9.255394 \
P:320004fffe_130ffffffff 7.483178 \
130ffffffff -8.663364 \
P:130ffffffff_130ffffffff 7.877774 \
1b500050004_10300050004 8.087816 \
1b500070004_1b500060004 6.511418 \
1f90004fffe_103fffd0005 8.085095 \
aaaaffffffff_1b500060004_1b500050004 3.053964 \
-2.429746 \
0.98852706135");

    idiomModelDesc[string("64-bit-gcc")]=string(
"20 \
P:70010ffff_130ffffffff -0.534462 \
P:32fffd0203_190010ffff 7.115012 \
1030000fffd_320000fffe 5.231150 \
103fffd0003 4.943136 \
P:130ffffffff 6.781068 \
1b500000004 3.803109 \
1b500030004 4.986216 \
1b500050004 3.823901 \
1b5000c0004 5.598772 \
1b5000d0004 6.243245 \
1b5000e0004 6.303827 \
1b5000f0004 8.861184 \
1f90004fffe 6.079618 \
P:202fffd0100_32fffd0100_32fffd0100 1.912703 \
aaaaffffffff_150010ffff 0.705742 \
aaaaffffffff_24fffdffff_130ffffffff 2.267716 \
aaaaffffffff_d30100fffe -6.113787 \
aaaaffffffff_1b500030004 2.102403 \
aaaaffffffff_aaaaffffffff_70010ffff 0.153313 \
aaaaffffffff_aaaaffffffff_110010ffff -0.837408 \
-4.978386 \
0.9899876364");

    idiomModelDesc[string("64-bit-icc")]=string(
"12 \
P:70010ffff_130ffffffff 5.826891 \
P:24fffdffff_130ffffffff 6.234387 \
P:32fffd0100 1.624691 \
e700060006 -5.916771 \
P:e700060006_e700070007 7.645867 \
1030000fffd_aaaaffffffff_2020000fffe 8.375661 \
130ffffffff -9.000042 \
P:130ffffffff 3.422269 \
P:130ffffffff_e700070007 8.381890 \
P:130ffffffff_130ffffffff 4.154446 \
1b5000f0004_1b5000e0004 8.446606 \
aaaaffffffff_10afffd2000 -5.568011 \
-4.842990 \
0.957217146");
}
