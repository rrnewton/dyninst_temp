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

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <string>

#include <unordered_map>
#include <stack>
#include "CFG.h"

#include "LoopAnalyzer.h"

using namespace Dyninst;
using namespace Dyninst::ParseAPI;

// constructor of the class. It creates the CFG and
LoopAnalyzer::LoopAnalyzer(Function *f)
  : func(f) 
{
    for (auto bit = f->blocks().begin(); bit != f->blocks().end(); ++bit) {
        Block* b = *bit;
	DFSP_pos[b] = 0;
	header[b] = NULL;
    }
       
}


struct loop_sort {
   Address getLoopSmallestEntry(Loop *l) const {
       vector<Block*> entries;
       l->getLoopEntries(entries);
       Address min = 0;
       for (auto bit = entries.begin(); bit != entries.end(); ++bit)
           if (min == 0 || (*bit)->start() < min) min = (*bit)->start();
       return min;          
   }
   bool operator()(Loop *l, Loop *r) const {
      Address lentry = getLoopSmallestEntry(l);
      Address rentry = getLoopSmallestEntry(r);
      return lentry < rentry; 
   }
};

void LoopAnalyzer::dfsCreateLoopHierarchy(LoopTreeNode * parent,
                            vector<Loop *> &loops,
                            std::string level)
{
  for (unsigned int i = 0; i < loops.size(); i++) {
    // loop name is hierarchical level
    std::string clevel = (level != "")
      ? level + "." + utos(i+1)
      : utos(i+1);

    // add new tree nodes to parent
    LoopTreeNode * child =
      new LoopTreeNode(loops[i], (std::string("loop_"+clevel)).c_str());

    parent->children.push_back(child);

    // recurse with this child's outer loops
    vector<Loop*> outerLoops;
    loops[i]->getOuterLoops(outerLoops);
    loop_sort l;
    std::sort(outerLoops.begin(), outerLoops.end(), l);
    dfsCreateLoopHierarchy(child, outerLoops, clevel);
  }
}

void LoopAnalyzer::createLoopHierarchy()
{
  LoopTreeNode* loopRoot = new LoopTreeNode(NULL, NULL);
  func->_loop_root = loopRoot;

  vector<Loop *> outerLoops;
  func->getOuterLoops(outerLoops);

  loop_sort l;
  std::sort(outerLoops.begin(), outerLoops.end(), l);
  // Recursively build the loop nesting tree
  dfsCreateLoopHierarchy(loopRoot, outerLoops, string(""));

  // Enumerate every basic blocks in the functions to find all create 
  // call information for each loop
  for (auto bit = func->blocks().begin(); bit != func->blocks().end(); ++bit) {
    Block *b = *bit;
    for (auto eit = b->targets().begin(); eit != b->targets().end(); ++eit) {
      // Can tail call happen here?
      if ((*eit)->type() == CALL) {
          Block *target = (*eit)->trg();
	  vector<Function*> callees;
	  target->getFuncs(callees);
	  for (auto fit = callees.begin(); fit != callees.end(); ++fit)
	      insertCalleeIntoLoopHierarchy(*fit, b->last());
      }
    }
  }
}


// try to insert func into the appropriate spot in the loop tree based on
// address ranges. if succesful return true, return false otherwise.
bool LoopAnalyzer::dfsInsertCalleeIntoLoopHierarchy(LoopTreeNode *node,
                                                        Function *callee,
                                                        unsigned long addr)
{
  // if this node contains func then insert it
  if ((node->loop != NULL) && node->loop->containsAddress(addr)) {
    node->callees.push_back(callee);
    return true;
  }

  // otherwise recur with each of node's children
  bool success = false;

  for (unsigned int i = 0; i < node->children.size(); i++) {
     success |= dfsInsertCalleeIntoLoopHierarchy(node->children[i], callee, addr);
  }

  return success;
}


void LoopAnalyzer::insertCalleeIntoLoopHierarchy(Function *callee,
                                                     unsigned long addr)
{
  // try to insert func into the loop hierarchy
  bool success = dfsInsertCalleeIntoLoopHierarchy(func->_loop_root, callee, addr);

  // if its not in a loop make it a child of the root
  if (!success) {
    func->_loop_root->callees.push_back(callee);
  }
}




bool LoopAnalyzer::analyzeLoops() {
    WMZC_DFS(func->entry(), 1);

    for (auto bit = func->blocks().begin(); bit != func->blocks().end(); ++bit) {
        Block* b = *bit;
	if (header[b] == NULL) continue;
	loop_tree[header[b]].insert(b);
    }

    for (auto bit = func->blocks().begin(); bit != func->blocks().end(); ++bit) {
        Block* b = *bit;
        if (header[b] == NULL) {
	    // if header[b] == NULL, b is either the header of a outermost loop, or not in any loop
	    createLoops(b);
	}
    }

    // The WMZC algorithm only identifies back edges 
    // to the loop head, which is the first node of the loop 
    // visited in the DFS.
    // Add other back edges that targets other entry blocks
    for (auto bit = func->blocks().begin(); bit != func->blocks().end(); ++bit) {
        Block* b = *bit;
	if (loops[b] != NULL) FillMoreBackEdges(loops[b]);
    }
    // Finish constructing all loops in the function.
    // Now populuate the loop data structure of the function.
    for (auto bit = func->blocks().begin(); bit != func->blocks().end(); ++bit) {
        Block* b = *bit;
	if (loops[b] != NULL)
	   func->_loops.insert(loops[b]); 
    }
        

    return true;
}

struct edge_sort {
    bool operator() (Edge *l, Edge *r) const {
        return l->trg()->start() < r->trg()->start();
    }
};

Block* LoopAnalyzer::WMZC_DFS(Block* b0, int pos) {
    visited.insert(b0);
    DFSP_pos[b0] = pos;
    // The final loop nesting structure depends on
    // the order of DFS. To guarantee that we get the 
    // same loop nesting structure for an individual binary 
    // in all executions, we sort the target blocks using
    // the start adress.
    vector<Edge*> visitOrder;
    edge_sort es;
    visitOrder.insert(visitOrder.end(), b0->targets().begin(), b0->targets().end());
    sort(visitOrder.begin(), visitOrder.end(), es);
    for (auto eit = b0->targets().begin(); eit != b0->targets().end(); ++eit) {
        if ((*eit)->interproc() || (*eit)->sinkEdge()) continue;
	Block* b = (*eit)->trg();	
	if (visited.find(b) == visited.end()) {
	    // case A, new
	    Block* nh = WMZC_DFS(b, pos + 1);
	    WMZC_TagHead(b0, nh);
	} else {
	    if (DFSP_pos[b] > 0) {
	        // case B
		if (loops[b] == NULL)
		    loops[b] = new Loop(func);
		WMZC_TagHead(b0, b);
		loops[b]->entries.insert(b);
		loops[b]->backEdges.insert(*eit);
	    }
	    else if (header[b] == NULL) {
	        // case C, do nothing
	    } else {
	        Block* h = header[b];
		if (DFSP_pos[h] > 0) {
		    // case D
		    WMZC_TagHead(b0, h);
		} else {
		    // case E
		    // Mark b and (b0,b) as re-entry
		    assert(loops[h]);
		    loops[h]->entries.insert(b);
		    while (header[h] != NULL) {
		        h = header[h];
			if (DFSP_pos[h] > 0) {
			    WMZC_TagHead(b0, h);
			    break;
		        }	
			assert(loops[h]);
			loops[h]->entries.insert(b);

		    }
		}
	    }
	}
    }
    DFSP_pos[b0] = 0;
    return header[b0];
}

void LoopAnalyzer::WMZC_TagHead(Block* b, Block* h) {
    if (b == h || h == NULL) return;
    Block *cur1, *cur2;
    cur1 = b; cur2 = h;
    while (header[cur1] != NULL) {
        Block* ih = header[cur1];
	if (ih == cur2) return;
	if (DFSP_pos[ih] < DFSP_pos[cur2]) { // Can we guarantee both are not 0?
	    header[cur1] = cur2;
	    cur1 = cur2;
	    cur2 = ih;
	} else cur1 = ih;
    }
    header[cur1] = cur2;
}

// Recursively build the basic blocks in a loop
// and the contained loops in a loop
void LoopAnalyzer::createLoops(Block* cur) {   
    if (loops[cur] != NULL)
        loops[cur]->basicBlocks.insert(cur);

    for (auto bit = loop_tree[cur].begin(); bit != loop_tree[cur].end(); ++bit) {
        Block* child = *bit;
	createLoops(child);
	if (loops[cur] != NULL && loops[child] != NULL) {
	    loops[cur]->containedLoops.insert(loops[child]->containedLoops.begin(), loops[child]->containedLoops.end());
	    loops[cur]->containedLoops.insert(loops[child]);
	    loops[child]->parent = loops[cur];
	}
	if (loops[cur] != NULL) {
	    loops[cur]->basicBlocks.insert(child);
	    if (loops[child] != NULL)
	        loops[cur]->basicBlocks.insert(loops[child]->basicBlocks.begin(), loops[child]->basicBlocks.end());
	}
    }

}

void LoopAnalyzer::FillMoreBackEdges(Loop *loop) {
    // All back edges to the header of the loop have been identified.
    // Now find all back edges to the other entries of the loop.
    for (auto bit = loop->basicBlocks.begin(); bit != loop->basicBlocks.end(); ++bit) {
        Block* b = *bit;
	for (auto eit = b->targets().begin(); eit != b->targets().end(); ++eit) {
	    Edge *e = *eit;
	    if (e->interproc() || e->sinkEdge()) continue;
	    if (loop->entries.find(e->trg()) != loop->entries.end())
	        loop->backEdges.insert(e);
	}	
    }
}

