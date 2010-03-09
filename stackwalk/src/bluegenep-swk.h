/*
 * Copyright (c) 1996-2009 Barton P. Miller
 * 
 * We provide the Paradyn Parallel Performance Tools (below
 * described as "Paradyn") on an AS IS basis, and do not warrant its
 * validity or performance.  We reserve the right to update, modify,
 * or discontinue this software at any time.  We shall have no
 * obligation to supply such updates or modifications or any other
 * form of support to you.
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

#ifndef BLUEGENEP_SWK_H
#define BLUEGENEP_SWK_H

#include "bluegene-swk.h"
#include <string>
#include <vector>

namespace Dyninst {
  namespace Stackwalker {
    
    class ProcDebugBGP : public ProcDebugBG {
      friend class ProcDebugBG;
      friend class ProcDebug;

    public:
      ProcDebugBGP(PID pid, std::string executable);
      virtual ~ProcDebugBGP();
      virtual bool isLibraryTrap(Dyninst::THR_ID thrd);
      virtual bool getThreadIds(std::vector<THR_ID> &threads);

    protected:
      virtual bool pollForNewThreads();
      virtual bool debug_post_attach(ThreadState *ts);
      virtual void registerLibSpotter();
      virtual bool debug_handle_event(DebugEvent ev);
      virtual void translate_event(const DebuggerInterface::BG_Debugger_Msg& msg, 
                                   DebugEvent& ev);
      
      /* Protected Data */
      bool gprs_set;  
      Address lib_load_trap;
    };
    
  } // Stackwalker
} // Dyninst

#endif // BLUEGENEP_SWK_H