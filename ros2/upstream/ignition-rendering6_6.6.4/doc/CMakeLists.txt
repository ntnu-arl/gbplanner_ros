find_package(Doxygen)

set(IGNITION_DOXYGEN_TAGFILES
  "\"${CMAKE_BINARY_DIR}/doc/ogre_1-9.tag.xml=https://www.ogre3d.org/docs/api/1.9/\"")

if (DOXYGEN_FOUND)
  configure_file(${IGNITION_CMAKE_DOXYGEN_DIR}/api.in
                 ${CMAKE_BINARY_DIR}/api.dox @ONLY)

  configure_file(${IGNITION_CMAKE_DOXYGEN_DIR}/tutorials.in
                 ${CMAKE_BINARY_DIR}/tutorials.dox @ONLY)

  # Find tar
  find_program(TAR_PATH tar)
  set (DOC_DEPEND "")

  # If tar exists, then untar the ogre xml tag file
  if (TAR_PATH)
    add_custom_target(doc_xml ALL
      ${TAR_PATH} xvf ${CMAKE_SOURCE_DIR}/doc/ogre_1-9.tag.xml.tgz -C ${CMAKE_BINARY_DIR}/doc
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
    set (DOC_DEPEND doc_xml)
  endif()

  # Generate the API and tutorial documentation
  add_custom_target(doc ALL
    ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/tutorials.dox
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/api.dox
    DEPENDS ${DOC_DEPEND}
    COMMENT "Generating API documentation with Doxygen" VERBATIM)

  install(FILES ${CMAKE_BINARY_DIR}/doc/${PROJECT_NAME_LOWER}.tag.xml
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/ignition/${PROJECT_NAME_LOWER}_${PROJECT_VERSION_MINOR})
endif()
