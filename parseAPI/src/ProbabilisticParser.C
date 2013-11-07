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
#include <queue>
#include <iostream>

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

extern map<std::string, std::string> idiomModelDesc;
void IdiomModelInit();

#define NOP_ENTRY_ID 0x130
#define WILDCARD_ENTRY_ID 0xaaaa
// Precision error allowed in double precision float number
#define ZERO 1e-8
static int double_cmp(double a, double b) {
    double delta = a - b;
    if (fabs(delta) < ZERO) return 0;
    else if (delta > ZERO) return 1;
    else return -1;
}

struct ProbAndAddr {
    Address addr;
    double prob;

    ProbAndAddr(Address a, double p): addr(a), prob(p) {}
    bool operator < (const ProbAndAddr &p) const {
        if (double_cmp(prob, p.prob) == 0) return addr > p.addr;
	return prob < p.prob;
    }
};

static unsigned short RegType(unsigned short arg) {
    int reg = arg & 0xf;
    if (reg == x86_64::BASEB || reg == x86_64::BASESP || reg == x86_64::BASEBP ||
        (reg <= x86_64::BASE15 && reg >= x86_64::BASE12)) return CALLEESAVEREG;    
    if (reg == x86_64::BASEA || reg == x86_64::BASE10 || reg == x86_64::BASE11) return OTHERREG;
    return ARGUREG;   
}

static bool IsSingleReg(unsigned short arg) {
    if (arg == NOARG || arg == IMMARG || arg == MEMARG || arg == MULTIREG
        || arg == CALLEESAVEREG || arg == ARGUREG || arg == OTHERREG) return false; return true;
}

static bool IsCategoryReg(unsigned short arg) {
    if (arg == CALLEESAVEREG || arg == ARGUREG || arg == OTHERREG) return true; return false;
}

static bool MatchArgs(unsigned short arg1, unsigned short arg2) {
    if (IsCategoryReg(arg1) && IsSingleReg(arg2)) return arg1 == RegType(arg2);
    if (IsCategoryReg(arg2) && IsSingleReg(arg1)) return arg2 == RegType(arg1);
    return arg1 == arg2;
}

bool IdiomTerm::operator == (const IdiomTerm &it) const {    
    if (entry_id == WILDCARD_ENTRY_ID || it.entry_id == WILDCARD_ENTRY_ID) return true; // Wildcard matches everything
    if (entry_id == NOP_ENTRY_ID && it.entry_id == NOP_ENTRY_ID) return true; // Nops match without considering arguments
    if (entry_id != it.entry_id) return false;
    return MatchArgs(arg1, it.arg1) && MatchArgs(arg2, it.arg2);
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
	else if (arg == CALLEESAVEREG) {
	    if (style) return "Callee saved reg"; else return ":D";
	} 
	else if (arg == ARGUREG) {
	    if (style) return "Argu passing reg"; else return ":E";
	}
	else if (arg == OTHERREG) {
	    if (style) return "Other reg"; else return ":F";
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

static IdiomTerm WILDCARD_TERM(WILDCARD_ENTRY_ID, NOARG, NOARG);

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
    // only work if the idiom contains single wildcard!
    for (size_t i = 0 ; i < idiom.terms.size(); ++i)
        if (idiom.terms[i].entry_id == WILDCARD_ENTRY_ID) {
	    Idiom new_idiom = idiom;
	    new_idiom.terms.insert(new_idiom.terms.begin() + i, WILDCARD_TERM);
	    idioms.push_back(new_idiom);
	    break;
	}
}

void IdiomSet::deleteUnmatch(size_t pos, unsigned short entry_id) {
    for (size_t i = 0; i < idioms.size(); ) {
        bool match = false;
        if (idioms[i].terms.size() > pos) {
	    const Idiom& cur_idiom = idioms[i];
	    if (cur_idiom.terms[pos].entry_id == entry_id || cur_idiom.terms[pos].entry_id == WILDCARD_ENTRY_ID) match = true;
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
	    if (cur_idiom.terms[pos].entry_id == WILDCARD_ENTRY_ID || // Wildcard ignores arugments
	        cur_idiom.terms[pos].entry_id == NOP_ENTRY_ID || // Nop ignores arguments
		(MatchArgs(cur_idiom.terms[pos].arg1, arg1) && MatchArgs(cur_idiom.terms[pos].arg2, arg2))) // Otherwise, match two operands
		    match = true;
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
//		fprintf(stderr, "forward match %s\n", cur_idiom.human_format().c_str());
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
    IdiomModelInit();
    if (model_spec != "gcc" && model_spec != "icc") {
        assert("We currently only support the 'gcc' model and 'icc' model\n" && 0);
    }
    
    int totalFeatures;
    char buf[1024];
    double weights;

    string desc = idiomModelDesc[model_spec];
    istringstream iss(desc);

    iss >> totalFeatures;
    for (int i = 0; i < totalFeatures; ++i) {
        iss >> buf >> weights;
	if (buf[0] == 'P')
	    prefix.addIdiom(Idiom(buf+2, weights, true));
	else
	    normal.addIdiom(Idiom(buf, weights, false));
    }
    iss >> bias;
    // Probability threshold is default to be 0.5
    if (!(iss >> prob_threshold)) prob_threshold = 0.5;
}

IdiomSet IdiomModel::copyNormalIdioms() {
    return normal;
}

IdiomSet IdiomModel::copyPrefixIdioms() {
    return prefix;
}

ProbabilityCalculator::ProbabilityCalculator(CodeRegion *reg, CodeSource *source, Parser* p, string model_spec):
    model(model_spec), cr(reg), cs(source), parser(p) 
{
}

double ProbabilityCalculator::calcProbByMatchingIdioms(Address addr) {
    if (!cr->contains(addr)) return -1; 
    double w = model.getBias();
    IdiomSet is = model.copyNormalIdioms();
    bool valid = true;
    w += calcForwardWeights(0, addr, is, valid);
//    fprintf(stderr, "w = %.6lf\n", w);
    if (valid) {
        is = model.copyPrefixIdioms();
	set<Idiom> matched;
	w += calcBackwardWeights(0, addr, is, matched);
	/*
        fprintf(stderr, "w = %.6lf\n", w);
        for (auto mit = matched.begin(); mit != matched.end(); ++mit) {
            fprintf(stderr, "prefix match %s\n", mit->human_format().c_str());
        }
	*/
        double prob = ((double)1) / (1 + exp(-w));
        return first_prob[addr] = prob;
    } else return 0;
//    }
}

void ProbabilityCalculator::calcProbByEnforcingConstraints() {

    priority_queue<ProbAndAddr> q;
    double threshold = model.getProbThreshold();

    // Use the prob from matching idioms and push all the FEP candidates into the priority queue
    for (auto pit = first_prob.begin(); pit != first_prob.end(); ++pit)
        if (pit->second >= threshold) 
	    q.push(ProbAndAddr(pit->first, pit->second));
    
    while (!q.empty()) {
        ProbAndAddr pa = q.top();
	q.pop();

	//This is an out-dated item. The address has either improved prob by
	//applying call consistency constraints or 0 prob by applying 
	//overlapping constraints.
	if (final_prob.find(pa.addr) != final_prob.end() && double_cmp(final_prob[pa.addr], pa.prob) != 0) continue;

	parser->parse_at(cr, pa.addr, false, GAP);

	Function *f = parser->findFuncByEntry(cr, pa.addr);
	assert(f != NULL);

//	fprintf(stderr, "Enforcing constraints at %lx with prob %.6lf\n", pa.addr, pa.prob);
	// Enforce the overlapping constraints
        if (enforceOverlappingConstraints(f, pa.addr, pa.prob)) {
	    // Enforce the call consistency constraints
	    auto call_edges = f->callEdges();
	    for (auto eit = call_edges.begin(); eit != call_edges.end(); ++eit) {
	        // TODO: Can I assume that the first address of the target block of a call edge is a FEP?
		Address target = (*eit)->trg()->start();
//		fprintf(stderr, "    find direct call target %lx with prob %.6lf\n", target, getProb(target));
		if (first_prob.find(target) == first_prob.end()) continue;
		if (target == pa.addr) continue;
		double prob = getProb(target);
		if (double_cmp(pa.prob, prob) > 0) {
		    final_prob[target] = pa.prob;
		    q.push(ProbAndAddr(target, pa.prob));
		}
	    }
	}	      
    }

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

double ProbabilityCalculator::calcForwardWeights(int cur, Address addr, IdiomSet &is, bool &valid) {
    if (addr >= cr->high()) return 0;
    if (is.size() == 0) return 0;
    Instruction::Ptr insn;
    InstructionDecoder dec((unsigned char*)(cs->getPtrToInstruction(addr)),  30, cs->getArch());
    
    unsigned short entry_id, len;
    if (opcodeCache.find(addr) != opcodeCache.end()) {
        const pair<unsigned short, unsigned short> &val = opcodeCache[addr];
	entry_id = val.first;
	len = val.second;
	// We have decoded it before and it is invalid
	if (len == 0) { 
	    valid = false;
	    return 0;
	}
    } else {
        insn = dec.decode();
	if (!insn) {
	    // We decoded junk
	    opcodeCache[addr] = make_pair(0xffff, 0);
	    valid = false;
	    return 0;
	}

        len = insn->size();
	// This is still invalid
	if (len == 0) {
	    valid = false;
	    return 0;
	}
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
	    if (op.getValue()->size() == 0) {
	        // This is actually an invalid instruction with valid opcode
		// so modify the opcode cache to make it invalid
		opcodeCache[addr] = make_pair(0xffff,0);
		valid = false;
		return 0;
	    }
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
    // the return value is not important if "valid" becomes false
    return w + calcForwardWeights(cur + 1, addr + len , is, valid);
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
	    bool validOperand = true;
	    for(unsigned int i=0;i<2 && i<ops.size();++i) {
	        Operand & op = ops[i];
		if (op.getValue()->size() == 0) {
		    // This is actually an invalid instruction with valid opcode
    		    // so modify the opcode cache to make it invalid
		    opcodeCache[prevAddr] = make_pair(0xffff,0);
		    validOperand = false;
		    break;
	        }
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
	    if (!validOperand) continue;
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

bool ProbabilityCalculator::enforceOverlappingConstraints(Function *f, Address cur_addr, double cur_prob) {
    vector<FuncExtent*> extents = f->extents();
    for (auto eit = extents.begin(); eit != extents.end(); ++eit) 
        for (Address addr = (*eit)->start(); addr < (*eit)->end(); ++addr) {
	    if (addr == cur_addr) continue;
	    double prob = getProb(addr);
	    
	    if (double_cmp(cur_prob, prob) < 0 || (double_cmp(cur_prob, prob) == 0 && addr < cur_addr)) {
	        // The address cur_addr conflicts with the address addr.
		// The address addr has larger prob to be a FEP or they have the same prob but addr is before cur_addr, 
		// then we don't think cur_addr to be a FEP
		final_prob[cur_addr] = 0;
		parser->remove_func(f);
//		fprintf(stderr, "    meet %lx with prob %.6lf\n", addr, prob);
//		fprintf(stderr, "NOT FEP\n");
		// We don't have to continue because we must have enforced the constraints at addr before
		return false;
	    } else {
	        // The address addr should not be a FEP
		final_prob[addr] = 0;
//		fprintf(stderr, "    enforce %lx to be not a FEP\n", addr);

		Function *addr_f = parser->findFuncByEntry(cr, addr);
		if (addr_f != NULL) {
//		    fprintf(stderr, "WARNING: address %lx with prob %.6lf conflicts with address %lx with prob %.6lf\n", cur_addr, cur_prob, addr, prob);
//		    fprintf(stderr, "         and address %lx shouldn't have been parsed\n", addr);
		    parser->remove_func(addr_f);
		}
	    }
	}
    return true;
}

#endif

