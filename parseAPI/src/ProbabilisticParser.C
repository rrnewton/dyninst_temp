/*
 * See the dyninst/COPYRIGHT file for copyright information.
 * 
 * We provide the Paradyn Tools (below described as "Paradyn")
 * on an AS IS basis, and do not warrant its validity or performance.
 * We reserve the right to update, modify, or discontinue this
 * software at any time.  We shall have no obligation to supply such
 * updates or modifications or any other form of support to you.
 * 
 * By your use of Paradyn, you understand and agree that we (or any
 * other person or entity with proprietary rights in Paradyn) are
 * under no obligation to provide either maintenance services,
 * update services, notices of latent defects, or correction of
 * defects for Paradyn.
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#if defined(cap_stripped_binaries)

#include <cstdio>
#include <cmath>
#include <algorithm>

#include "entryIDs.h"
#include "dyn_regs.h"
#include "InstructionDecoder.h"
#include "Instruction.h"

#include "ProbabilisticParser.h"

using namespace std;
using namespace Dyninst;
using namespace Dyninst::ParseAPI;
using namespace Dyninst::InstructionAPI;
using namespace NS_x86;
using namespace hd; 
bool IdiomTerm::operator == (const IdiomTerm &it) const {
    if (entry_id == 0x130 && it.entry_id == 0x130) return true; // Nops match without considering arguments
    if (entry_id == 0x1b5 && arg1 == 0xffff && arg2 == 0xffff) // This is a push callee saved register term
        if (it.entry_id == 0x1b5 && ((it.arg1 <= 0xf && it.arg1 >= 0xc) || it.arg1 == 0x5 || it.arg1 == 0x3)) // Push callee saved registers
	    return true;
    return (entry_id == it.entry_id) && (arg1 == it.arg1) && (arg2 == it.arg2);
}

bool IdiomTerm::operator < (const IdiomTerm &it) const {
    if (entry_id != it.entry_id) return entry_id < it.entry_id;
    if (arg1 != it.arg1) return arg1 < it.arg1;
    return arg2 < it.arg2;
}

static string HandleAnOperand(unsigned short arg, int style) {
    if(arg != NOARG) {
	if (arg == MEMARG) {
	    if (style) return "MEM"; else return ":A";
	}
	else if (arg == IMMARG) {
	    if (style) return "IMM"; else return ":B";
	}
	else if (arg == MULTIREG) {
	    if (style) return "MULTIREG"; else return ":C";
	}
	else {
	    // the human_readble format is still broken
	    if (style) {
		if (arg == 0x0010) return x86_64::rip.name();
	        switch (arg & 0xf000) {
		    case 0x0000:
		        //GPR
			return MachRegister(arg | x86_64::GPR | Arch_x86_64).name();
		    case 0x1000:
		        //mmx
			return MachRegister(arg | x86_64::MMX | Arch_x86_64).name();
		    case 0x2000:
		        //xxm
			return MachRegister(arg | x86_64::XMM | Arch_x86_64).name();
		    case 0x4000:
		        // flag bit
			return MachRegister(arg | x86_64::FLAG | Arch_x86_64).name();
		}

	      
	        return ":" + MachRegister(arg).name();
	    }
	    char buf[64];
	    snprintf(buf, 64, "%x", arg);
	    return string(":") + string(buf);
	}
    }
    return "";
}

static IdiomTerm WILDCARD_TERM(0xaaaa, 0xffff, 0xffff);

string IdiomTerm::human_format() const {
    string entryname;
    if(*this == WILDCARD_TERM)
        entryname = "*";
    else if(entryNames_IAPI.find((entryID)(entry_id-1)) == entryNames_IAPI.end()) {
        // Just not lack of entry in entryNames_IAPI, once found such situation, should add an entry
        entryname = "[UNMAPED]";
	fprintf(stderr, "Found entryID not mapped in entryNames_IAPI %d\n", entry_id - 1);
    }
    else {
        entryname = entryNames_IAPI[(entryID)(entry_id-1)];
	if (arg1 != NOARG) entryname += " " + HandleAnOperand(arg1, 1);
	if (arg2 != NOARG) entryname += "," + HandleAnOperand(arg2, 1);
    }

    return entryname;        

}

bool Idiom::operator < (const Idiom &i) const {
    if (terms.size() != i.terms.size()) return terms.size() < i.terms.size();
    for (size_t index = 0; index < terms.size(); ++index)
        if (terms[index] < i.terms[index]) return true;
	else if (i.terms[index] < terms[index]) return false;
    return false;
}

static void split(const char * str, vector<uint64_t> & terms)
{
    const char *s = str, *e = NULL;
    char buf[32];
    
    while((e = strchr(s,'_')) != NULL) {
        assert(e-s < 32);
	strncpy(buf,s,e-s);
	buf[e-s] = '\0';
	terms.push_back(strtoull(buf,NULL,16));
	
	s = e+1;
    }
    // last one
    if(strlen(s)) {
        terms.push_back(strtoull(s,NULL,16));
    }
}
Idiom::Idiom(string format, double weight, bool pre): 
    w(weight), prefix(pre)
{
    vector<uint64_t> items;
    split(format.c_str(), items);

    unsigned short entry_id, arg1, arg2;
    for (size_t i = 0; i < items.size(); ++i) {
        uint64_t t = items[i];
        if(!(t & ENTRY_MASK)) {
	    t = t << ARG_SIZE;
	    t |= NOARG;
	}
	if(!(t & ENTRY_MASK)) {
	    t = t << ARG_SIZE;
	    t |= NOARG;
	}
	
	entry_id = (t>>32) & 0xffffffff;
	arg1 = (t>>16) & 0xffff;
	arg2 = t & 0xffff;
	terms.push_back(IdiomTerm(entry_id, arg1, arg2));

    }
    if (prefix) reverse(terms.begin(), terms.end());       
}

string Idiom::human_format() const {
    string ret = "";

    //printf("formatting %s\n",format().c_str());
    for(unsigned i=0;i<terms.size();++i) {
        ret += terms[i].human_format();
        if(i<terms.size()-1)
            ret += "|";
    }
    return ret;

}

void IdiomSet::addIdiom(const Idiom& idiom) {
    idioms.push_back(idiom);
}

void IdiomSet::deleteUnmatch(size_t pos, unsigned short entry_id) {
    for (size_t i = 0; i < idioms.size(); ) {
        bool match = false;
        if (idioms[i].terms.size() > pos) {
	    const Idiom& cur_idiom = idioms[i];
	    if (cur_idiom.terms[pos].entry_id == entry_id || cur_idiom.terms[pos].entry_id == 0xaaaa) match = true;
	}
	if (!match) {
	    idioms.erase(idioms.begin() + i);
	} else {
	    ++i;
	}
    }
}

void IdiomSet::deleteUnmatch(size_t pos, unsigned short arg1, unsigned short arg2) {
    for (size_t i = 0; i < idioms.size(); ) {
        bool match = false;
        if (idioms[i].terms.size() > pos) {
	    const Idiom& cur_idiom = idioms[i];
	    if (cur_idiom.terms[pos].entry_id == 0xaaaa || // Wildcard ignores arugments
	        cur_idiom.terms[pos].entry_id == 0x130 || // Nop ignores arguments
		(cur_idiom.terms[pos].entry_id == 0x1b5 && cur_idiom.terms[pos].arg1 == 0xffff && cur_idiom.terms[pos].arg2 == 0xffff && ((arg1 <= 0xf && arg1 >= 0xc) || arg1 == 0x5 || arg1 == 0x3)) || // Push callee saved registers
		(cur_idiom.terms[pos].arg1 == arg1 && cur_idiom.terms[pos].arg2 == arg2)) match = true;
	}
	if (!match) {
	    idioms.erase(idioms.begin() + i);
	} else {
	    ++i;
	}
    }
}

double IdiomSet::matchForwardAndDelete(size_t pos, unsigned short entry_id, unsigned short arg1, unsigned short arg2) {
    double w = 0;
    const IdiomTerm it(entry_id, arg1, arg2);
    for (size_t i = 0; i < idioms.size(); ) {
        bool match = false;
        if (idioms[i].terms.size() == pos + 1) {
	    const Idiom& cur_idiom = idioms[i];
	    if (cur_idiom.terms[pos] == it) {
	        match = true;
		w += cur_idiom.w;
            }
	}
	if (match) {
	    idioms.erase(idioms.begin() + i);
	} else {
	    ++i;
	}
    }
    return w;
}

double IdiomSet::matchBackwardAndDelete(size_t pos, 
                                        unsigned short entry_id, 
					unsigned short arg1, 
					unsigned short arg2, 
					set<Idiom>& matched) {
    double w = 0;
    const IdiomTerm it(entry_id, arg1, arg2);
    for (size_t i = 0; i < idioms.size(); ) {
        bool match = false;
        if (idioms[i].terms.size() == pos + 1) {
	    const Idiom& cur_idiom = idioms[i];
	    if (cur_idiom.terms[pos] == it) {
	        match = true;
		if (matched.find(cur_idiom) == matched.end()) {
		    w += cur_idiom.w;
		    matched.insert(cur_idiom);
		}
	    }
	}
	if (match) {
	    idioms.erase(idioms.begin() + i);
	} else {
	    ++i;
	}
    }
    return w;
}
IdiomModel::IdiomModel(string model_spec) {

    if (model_spec != "gcc" && model_spec != "icc") {
        assert("We currently only support the 'gcc' model and 'icc' model\n" && 0);
    }
    
    int totalFeatures;
    char buf[1024];
    double weights;

    FILE *f;
    if (model_spec == "gcc") f = fopen("gcc_model", "r");
    if (model_spec == "icc") f = fopen("icc_model", "r");
    if (f == NULL) {
        fprintf(stderr, "Cannot %s model file!\n", model_spec.c_str());
	assert(0);
    }
    fscanf(f, "%d", &totalFeatures);
    for (int i = 0; i < totalFeatures; ++i) {
        fscanf(f, "%s %lf", buf, &weights);
	if (buf[0] == 'P')
	    prefix.addIdiom(Idiom(buf+2, weights, true));
	else
	    normal.addIdiom(Idiom(buf, weights, false));
    }
    fscanf(f, "%lf", &bias);
    // Probability threshold is default to be 0.5
    if (fscanf(f, "%lf", &prob_threshold) == EOF) prob_threshold = 0.5;
    fclose(f);
}

IdiomSet IdiomModel::copyNormalIdioms() {
    return normal;
}

IdiomSet IdiomModel::copyPrefixIdioms() {
    return prefix;
}

ProbabilityCalculator::ProbabilityCalculator(CodeRegion *reg, CodeSource *source, string model_spec):
    model(model_spec), cr(reg), cs(source) {}

double ProbabilityCalculator::calcProbByMatchingIdioms(Address addr) {
    if (!cr->contains(addr)) return -1; 
    double w = model.getBias();
//    if (addr == 0x42edc8) {
    IdiomSet is = model.copyNormalIdioms();
    w += calcForwardWeights(0, addr, is);
//    fprintf(stderr, "w = %.6lf\n", w);
    is = model.copyPrefixIdioms();
    set<Idiom> matched;
    w += calcBackwardWeights(0, addr, is, matched);
//    fprintf(stderr, "w = %.6lf\n", w);
//    for (auto mit = matched.begin(); mit != matched.end(); ++mit) {
//        fprintf(stderr, "prefix match %s\n", mit->human_format().c_str());
//    }
//    }
    double prob = ((double)1) / (1 + exp(-w));
    return first_prob[addr] = prob;
}

void ProbabilityCalculator::calcProbByEnforcingConstraints() {


}

double ProbabilityCalculator::getProb(Address addr) {
    if (final_prob.find(addr) != final_prob.end()) return final_prob[addr];
    if (first_prob.find(addr) != first_prob.end()) return first_prob[addr];
    return -1;
}

bool ProbabilityCalculator::isFEP(Address addr) {
    double prob = getProb(addr);
    if (prob >= model.getProbThreshold()) return true; else return false;
}

double ProbabilityCalculator::calcForwardWeights(int cur, Address addr, IdiomSet &is) {
    if (addr >= cr->high()) return 0;
    if (is.size() == 0) return 0;
    Instruction::Ptr insn;
    InstructionDecoder dec((unsigned char*)(cs->getPtrToInstruction(addr)),  30, cs->getArch());
    
    unsigned short entry_id, len;
    if (opcodeCache.find(addr) != opcodeCache.end()) {
        const pair<unsigned short, unsigned short> &val = opcodeCache[addr];
	entry_id = val.first;
	len = val.second;
	if (len == 0) return 0;
    } else {
        insn = dec.decode();
	if (!insn) {
	    opcodeCache[addr] = make_pair(0xffff, 0);
	    return 0;
	}

        len = insn->size();
	if (len == 0) return 0;
        const Operation & op = insn->getOperation();
        entry_id = op.getID() + 1;
	opcodeCache[addr] = make_pair(entry_id, len);
    }

    is.deleteUnmatch(cur, entry_id);
    if (is.size() == 0) return 0;

    unsigned short arg1, arg2;
    if (operandCache.find(addr) != operandCache.end()) {
        const pair<unsigned short, unsigned short> &val = operandCache[addr];
	arg1 = val.first;
	arg2 = val.second;
    } else {

        if (!insn) insn = dec.decode();

        vector<Operand> ops;
        insn->getOperands(ops);
        int args[2] = {NOARG,NOARG};
	
	for(unsigned int i=0;i<2 && i<ops.size();++i) {
	    Operand & op = ops[i];
	    if(!op.readsMemory() && !op.writesMemory()) {
	        // register or immediate
                set<RegisterAST::Ptr> regs;
		op.getReadSet(regs);
		op.getWriteSet(regs);

                if(!regs.empty()) {
		    if (regs.size() > 1) {
		        args[i] = MULTIREG;
		    } else {
		        args[i] = (*regs.begin())->getID();
		    }
		} else {
                    // immediate
                    args[i] = IMMARG;
                }
            } else {
	        args[i] = MEMARG; 
            }
        }
	arg1 = args[0];
	arg2 = args[1];
	operandCache[addr] = make_pair(arg1, arg2);
    }
    is.deleteUnmatch(cur, arg1, arg2);
    if (is.size() == 0) return 0;

    double w = is.matchForwardAndDelete(cur, entry_id, arg1, arg2);
    return w + calcForwardWeights(cur + 1, addr + len , is);
}

double ProbabilityCalculator::calcBackwardWeights(int cur, Address addr, IdiomSet &is, set<Idiom> &matched) {
    if (is.size() == 0) return 0;
    double w;
    for (Address prevAddr = addr - 1; prevAddr >= cr->low() && addr - prevAddr <= 15; --prevAddr) {
        Instruction::Ptr insn;
	InstructionDecoder dec((unsigned char*)(cs->getPtrToInstruction(prevAddr)),  30, cs->getArch()); 
	unsigned short entry_id, len;
	if (opcodeCache.find(prevAddr) != opcodeCache.end()) {
	    const pair<unsigned short, unsigned short> &val = opcodeCache[prevAddr];
	    entry_id = val.first;
	    len = val.second;
	    if (len == 0) continue;
	} else {
	    insn = dec.decode();
	    if (!insn) {
	        opcodeCache[prevAddr] = make_pair(0xffff, 0);
		continue;
	    }
	    
	    len = insn->size();
	    if (len == 0) continue;
	    const Operation & op = insn->getOperation();
	    entry_id = op.getID() + 1;
	    opcodeCache[prevAddr] = make_pair(entry_id, len);
	}
	if (prevAddr + len != addr) continue;
	IdiomSet is_copy = is;
	is_copy.deleteUnmatch(cur, entry_id);
	if (is_copy.size() == 0) continue;
	
	unsigned short arg1, arg2;
	if (operandCache.find(prevAddr) != operandCache.end()) {
	    const pair<unsigned short, unsigned short> &val = operandCache[prevAddr];
	    arg1 = val.first;
	    arg2 = val.second;
	} else {

            if (!insn) insn = dec.decode();

	    vector<Operand> ops;
	    insn->getOperands(ops);
	    int args[2] = {NOARG,NOARG};
	    
	    for(unsigned int i=0;i<2 && i<ops.size();++i) {
	        Operand & op = ops[i];
		if(!op.readsMemory() && !op.writesMemory()) {
		    // register or immediate
                    set<RegisterAST::Ptr> regs;
		    op.getReadSet(regs);
		    op.getWriteSet(regs);
		    
		    if(!regs.empty()) {
		        if (regs.size() > 1) {
			    args[i] = MULTIREG;
			} else {
			    args[i] = (*regs.begin())->getID();
			}
		    } else {
		        // immediate
                        args[i] = IMMARG;
                    }
                } else {
		    args[i] = MEMARG; 
                }
            }
	    arg1 = args[0];
            arg2 = args[1];
	    operandCache[prevAddr] = make_pair(arg1, arg2);
        }
        is_copy.deleteUnmatch(cur, arg1, arg2);
        if (is_copy.size() == 0) return 0;

        w += is_copy.matchBackwardAndDelete(cur, entry_id, arg1, arg2, matched);
        w += calcBackwardWeights(cur + 1, prevAddr , is_copy, matched);

    }
    return w;
}

#endif

