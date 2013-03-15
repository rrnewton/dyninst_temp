# CMake configuration for symtabAPI directory

include_directories (
    ${PROJECT_SOURCE_DIR}/symtabAPI/src
  )


set (SRC_LIST
                src/Object.C 
                src/Aggregate.C 
                src/Function.C 
                src/Variable.C 
                src/Symbol.C 
                src/LineInformation.C 
                src/Symtab.C 
                src/Symtab-edit.C 
                src/Symtab-lookup.C 
                src/Symtab-deprecated.C 
                src/Module.C 
                src/Region.C 
                src/Collections.C 
                src/Type.C 
                src/AddrLookup.C 
                src/annotations.C 
                src/debug.C 
                src/SymtabReader.C 
                src/dwarfWalker.C
  )

if (PLATFORM MATCHES freebsd OR 
    PLATFORM MATCHES linux OR 
    PLATFORM MATCHES bgq OR
    PLATFORM MATCHES cnl)

set (SRC_LIST ${SRC_LIST}
    src/Object-elf.C 
    src/Archive.C 
    src/Archive-elf.C 
    src/parseDwarf.C 
    src/parseStab.C 
    src/emitElf.C 
    src/LinkMap.C 
    src/emitElfStatic.C 
)

if (PLATFORM MATCHES x86_64 OR PLATFORM MATCHES amd64)
set (SRC_LIST ${SRC_LIST}
    src/emitElf-64.C
    src/emitElfStatic-x86.C 
    src/relocationEntry-elf-x86.C
    )
elseif (PLATFORM MATCHES i386)
set (SRC_LIST ${SRC_LIST}
    src/emitElfStatic-x86.C
    src/relocationEntry-elf-x86.C
    )
elseif (PLATFORM MATCHES ppc32)
set (SRC_LIST ${SRC_LIST}
    src/emitElfStatic-ppc32.C 
    src/relocationEntry-elf-ppc32.C
    )
elseif (PLATFORM MATCHES ppc64)
set (SRC_LIST ${SRC_LIST}
    src/emitElf-64.C
    src/emitElfStatic-ppc64.C
    src/relocationEntry-elf-ppc64.C
    )
endif()

endif()
  

add_library (symtabAPI ${SRC_LIST})
target_link_libraries (symtabAPI common)

if (UNIX)
  target_link_libraries (symtabAPI ${LIBELF_LIBRARIES})
  target_link_libraries (symtabAPI ${LIBDWARF_LIBRARIES})
endif()

INSTALL (DIRECTORY h/ DESTINATION include FILES_MATCHING PATTERN "*.h")
INSTALL (TARGETS symtabAPI
        LIBRARY DESTINATION lib)