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

/*
 * The classes used for probabilistic gap parsing
 */

#ifndef _PROBABILISTIC_PARSER_H_
#define _PROBABILISTIC_PARSER_H_

#if defined(cap_stripped_binaries)

#include <string>
#include <vector>
#include <set>
#include <map>

#include "common/h/dyntypes.h"
#include "CodeSource.h"
#include "entryIDs.h"
#include "RegisterIDs.h"
#include "Parser.h"
#include "CFG.h"

using Dyninst::Address;
using Dyninst::ParseAPI::CodeRegion;
using Dyninst::ParseAPI::CodeSource;
using Dyninst::ParseAPI::Parser;
using Dyninst::ParseAPI::Function;

namespace hd {

#define NOARG 0xffff
#define IMMARG (NOARG-1)
#define MEMARG (NOARG-2)
#define MULTIREG (NOARG-3)
#define CALLEESAVEREG (NOARG-4)
#define ARGUREG (NOARG-5)
#define OTHERREG (NOARG-6)


#define ENTRY_SHIFT 32ULL
#define ARG1_SHIFT 16ULL
#define ARG2_SHIFT 0ULL
#define ENTRY_SIZE 16ULL
#define ARG_SIZE 16ULL

#define ENTRY_MASK (((uint64_t)(1<<ENTRY_SIZE)-1) << ENTRY_SHIFT)
#define ARG1_MASK (((uint64_t)(1<<ARG_SIZE)-1) << ARG1_SHIFT)
#define ARG2_MASK (((uint64_t)(1<<ARG_SIZE)-1) << ARG2_SHIFT)

struct IdiomTerm {
    unsigned short entry_id, arg1, arg2;
    IdiomTerm() {}
    IdiomTerm(unsigned short a, unsigned short b, unsigned short c):
        entry_id(a), arg1(b), arg2(c) {}
    bool operator == (const IdiomTerm & it) const;
    bool operator < (const IdiomTerm & it) const;
    std::string human_format() const;
};

struct Idiom {
    std::vector<IdiomTerm> terms;
    double w;
    bool prefix;
    Idiom(std::string f, double weight, bool pre);
    bool operator < (const Idiom& i) const;
    std::string human_format() const;
};

class IdiomSet {
    std::vector<Idiom> idioms;

public:
    void addIdiom(const Idiom& idiom);
    void deleteUnmatch(size_t pos, unsigned short entry_id);
    void deleteUnmatch(size_t pos, unsigned short arg1, unsigned short arg2);
    double matchForwardAndDelete(size_t pos, unsigned short entry_id, unsigned short arg1, unsigned short arg2);
    // A prefix idiom may happen multiple times at a give byte. We only want to count its weight once
    double matchBackwardAndDelete(size_t pos, 
                                  unsigned short entry_id, 
				  unsigned short arg1, 
				  unsigned short arg2, 
				  std::set<Idiom> &matched); 
    int size() {return idioms.size(); }
};

class IdiomModel {
    IdiomSet normal;
    IdiomSet prefix;

    double bias;
    double prob_threshold;

public:
    IdiomModel() {}
    IdiomModel(std::string model_spec);
    IdiomSet copyNormalIdioms();
    IdiomSet copyPrefixIdioms();
    double getBias() {return bias; }
    double getProbThreshold() { return prob_threshold; }
};

class ProbabilityCalculator {
    IdiomModel model;
    CodeRegion* cr;
    CodeSource* cs;
    Parser* parser;
    // The probability calculated by applying idiom model
    dyn_hash_map<Address, double> first_prob;
    // The probability calculated by enforcing overlapping and caller-callee constraints
    dyn_hash_map<Address, double> final_prob;

    // To avoid reduntant decoding
    dyn_hash_map<Address, std::pair<unsigned short, unsigned short> > opcodeCache, operandCache;

    // Recursively mathcing normal idioms and calculate weights
    double calcForwardWeights(int cur, Address addr, IdiomSet &is, bool &valid);
    // Recursively mathcing prefix idioms and calculate weights
    double calcBackwardWeights(int cur, Address addr, IdiomSet &is, std::set<Idiom> &matched);
    // Enforce the overlapping constraints and
    // return true if the cur_addr doesn't conflict with other identified functions,
    // otherwise return false
    bool enforceOverlappingConstraints(Function *f, Address cur_addr, double cur_prob);

public:
    ProbabilityCalculator(CodeRegion *reg, CodeSource *source, Parser *parser, std::string model_spec);
    double calcProbByMatchingIdioms(Address addr);
    void calcProbByEnforcingConstraints();
    double getProb(Address addr);
    bool isFEP(Address addr);
};

};

#endif

#endif
