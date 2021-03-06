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

#include "InstructionDecoderImpl.h"
#include "common/src/singleton_object_pool.h"
#include "InstructionDecoder-x86.h"
#include "InstructionDecoder-power.h"
#include "BinaryFunction.h"
#include "Dereference.h"

using namespace std;
namespace Dyninst
{
    namespace InstructionAPI
    {
        Instruction* InstructionDecoderImpl::makeInstruction(entryID opcode, const char* mnem,
            unsigned int decodedSize, const unsigned char* raw)
        {
            Operation::Ptr tmp(make_shared(singleton_object_pool<Operation>::construct(opcode, mnem, m_Arch)));
            return singleton_object_pool<Instruction>::construct(tmp, decodedSize, raw, m_Arch);
        }
    
    
        Instruction::Ptr InstructionDecoderImpl::decode(InstructionDecoder::buffer& b)
        {
            //setMode(m_Arch == Arch_x86_64);
            const unsigned char* start = b.start;
            decodeOpcode(b);
            unsigned int decodedSize = b.start - start;

            return make_shared(singleton_object_pool<Instruction>::construct(
                                   m_Operation, decodedSize, start, m_Arch));
        }

        std::map<Architecture, InstructionDecoderImpl::Ptr> InstructionDecoderImpl::impls;
        InstructionDecoderImpl::Ptr InstructionDecoderImpl::makeDecoderImpl(Architecture a)
        {
            if(impls.empty())
            {
                impls[Arch_x86] = Ptr(new InstructionDecoder_x86(Arch_x86));
                impls[Arch_x86_64] = Ptr(new InstructionDecoder_x86(Arch_x86_64));
                impls[Arch_ppc32] = Ptr(new InstructionDecoder_power(Arch_ppc32));
                impls[Arch_ppc64] = Ptr(new InstructionDecoder_power(Arch_ppc64));
            }
            std::map<Architecture, Ptr>::const_iterator foundImpl = impls.find(a);
            if(foundImpl == impls.end())
            {
                return Ptr();
            }
            return foundImpl->second;
        }
        Expression::Ptr InstructionDecoderImpl::makeAddExpression(Expression::Ptr lhs,
                Expression::Ptr rhs, Result_Type resultType)
        {
            BinaryFunction::funcT::Ptr adder(new BinaryFunction::addResult());

            return make_shared(singleton_object_pool<BinaryFunction>::construct(lhs, rhs, resultType, adder));
        }
        Expression::Ptr InstructionDecoderImpl::makeMultiplyExpression(Expression::Ptr lhs, Expression::Ptr rhs,
                Result_Type resultType)
        {
            BinaryFunction::funcT::Ptr multiplier(new BinaryFunction::multResult());
            return make_shared(singleton_object_pool<BinaryFunction>::construct(lhs, rhs, resultType, multiplier));
        }
        Expression::Ptr InstructionDecoderImpl::makeDereferenceExpression(Expression::Ptr addrToDereference,
                Result_Type resultType)
        {
            return make_shared(singleton_object_pool<Dereference>::construct(addrToDereference, resultType));
        }
        Expression::Ptr InstructionDecoderImpl::makeRegisterExpression(MachRegister registerID)
        {
            int newID = registerID.val();
            int minusArch = newID & ~(registerID.getArchitecture());
            int convertedID = minusArch | m_Arch;
            MachRegister converted(convertedID);
            return make_shared(singleton_object_pool<RegisterAST>::construct(converted, 0, registerID.size() * 8));
        }
    };
};

