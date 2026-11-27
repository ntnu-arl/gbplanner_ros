file(GLOB files "*.glsl")

install(FILES ${files} DESTINATION ${IGN_RENDERING_RESOURCE_PATH}/ogre/media/materials/programs)

