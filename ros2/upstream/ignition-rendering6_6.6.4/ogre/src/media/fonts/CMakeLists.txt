file(GLOB files "*ttf" "*png" "*fontdef")

install(FILES ${files} DESTINATION ${IGN_RENDERING_RESOURCE_PATH}/ogre/media/fonts)
install(DIRECTORY liberation-sans DESTINATION ${IGN_RENDERING_RESOURCE_PATH}/ogre/media/fonts)
