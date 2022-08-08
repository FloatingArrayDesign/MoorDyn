file(GLOB FUNCTIONS_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.mex)
foreach(FUNCTION_SRC ${FUNCTIONS_SRCS})
    matlab_add_mex(NAME ${FUNCTION_SRC} SRC ${FUNCTION_SRC}.mex LINK_TO ${Matlab_LIBRARIES} moordyn)
    install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${FUNCTION_SRC}.${Matlab_MEX_EXTENSION}
    DESTINATION
        matlab
    )
endforeach()
