file(GLOB FUNCTIONS_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.mex)
foreach(FUNCTION_SRC ${FUNCTIONS_SRCS})
    get_filename_component(OUTPUT_FILE_WLE ${FUNCTION_SRC} FUNCTION_NAME)
    matlab_add_mex(NAME ${FUNCTION_NAME} SRC ${FUNCTION_SRC} LINK_TO ${Matlab_LIBRARIES} moordyn)
    install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${FUNCTION_NAME}.${Matlab_MEX_EXTENSION}
    DESTINATION
        matlab
    )
endforeach()
