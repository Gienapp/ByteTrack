# 
# To be defined header with copyright information
# 

# Specifies compiler options (using 'target_compile_options')
# for a given pseudo-target. This target should then be consumed 
# by the actual library/executable using 'target_link_libraries'.
function( set_custom_compiler_options target )

    set( compiler_options 
        -Wno-long-long 
        -fPIC
        -pthread
    )

    target_compile_options( ${target} INTERFACE ${compiler_options} )

endfunction()