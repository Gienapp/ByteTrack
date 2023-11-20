# 
# To be defined header with copyright information
# 

# Specifies compiler warnings (using 'target_compile_options')
# for a given pseudo-target. This target should then be consumed 
# by the actual library/executable using 'target_link_libraries'.
function( set_custom_compiler_warnings target )

    option( TREAT_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" FALSE )

    # List of warning options copied from
    # https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html (2023.06.01)
    set( compiler_warnings 
        -Wall 
        # enables (removed non-C++ ones):
        # -Waddress
        # -Warray-bounds=1 (only with -O2)
        # -Warray-compare
        # -Wbool-compare
        # -Wbool-operation
        # -Wc++11-compat  
        # -Wc++14-compat
        # -Wcatch-value (C++ and Objective-C++ only)
        # -Wchar-subscripts
        # -Wcomment
        # -Wdangling-pointer=2
        # -Wformat
        # -Wformat-overflow
        # -Wformat-truncation
        # -Wint-in-bool-context
        # -Winit-self (only for C++)
        # -Wlogical-not-parentheses
        # -Wmain (only for C/ObjC and unless -ffreestanding)
        # -Wmaybe-uninitialized
        # -Wmemset-elt-size
        # -Wmemset-transposed-args
        # -Wmisleading-indentation (only for C/C++)
        # -Wmismatched-dealloc
        # -Wmismatched-new-delete (only for C/C++)
        # -Wmissing-attributes
        # -Wmultistatement-macros
        # -Wnarrowing (only for C++)
        # -Wnonnull
        # -Wnonnull-compare
        # -Wopenmp-simd
        # -Wparentheses
        # -Wpessimizing-move (only for C++)
        # -Wpointer-sign
        # -Wrange-loop-construct (only for C++)
        # -Wreorder
        # -Wrestrict
        # -Wreturn-type
        # -Wself-move (only for C++)
        # -Wsequence-point
        # -Wsign-compare (only in C++)
        # -Wsizeof-array-div
        # -Wsizeof-pointer-div
        # -Wsizeof-pointer-memaccess
        # -Wstrict-aliasing
        # -Wstrict-overflow=1
        # -Wswitch
        # -Wtautological-compare
        # -Wtrigraphs
        # -Wuninitialized
        # -Wunknown-pragmas
        # -Wunused-function
        # -Wunused-label
        # -Wunused-value
        # -Wunused-variable
        # -Wuse-after-free=2
        # -Wvolatile-register-var
        # -Wzero-length-bounds
        -Wextra
        # enables (removed non-C++ ones):
        # -Wclobbered
        # -Wcast-function-type
        # -Wdeprecated-copy (C++ only)
        # -Wempty-body
        # -Wignored-qualifiers
        # -Wimplicit-fallthrough=3
        # -Wmissing-field-initializers
        # -Woverride-init
        # -Wstring-compare
        # -Wredundant-move (only for C++)
        # -Wtype-limits
        # -Wuninitialized
        # -Wshift-negative-value (in C++11 to C++17 and in C99 and newer)
        # -Wunused-parameter (only with -Wunused or -Wall)
        # -Wunused-but-set-parameter (only with -Wunused or -Wall)
        -Wpedantic 
        # not covered by -Wall and/or -Wextra
        -Wcast-align 
        -Wcast-qual 
        -Wconversion 
        -Wdouble-promotion
        -Wduplicated-branches 
        -Wduplicated-cond 
        -Wformat=2
        -Wlogical-op
        -Wmissing-declarations 
        -Wnon-virtual-dtor 
        -Wnull-dereference
        -Wold-style-cast 
        -Woverloaded-virtual
        -Wredundant-decls 
        -Wshadow 
        -Wsign-conversion 
        -Wswitch-default 
        -Wundef 
        -Wunused 
    )

    target_compile_options( ${target} INTERFACE ${compiler_warnings} )

endfunction()