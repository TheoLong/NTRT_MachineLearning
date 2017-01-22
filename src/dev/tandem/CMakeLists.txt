Project(tandem)

link_libraries(sensors
		tgcreator
		core
		util
		terrain
		Adapters
		Configuration
		AnnealEvolution
		FileHelpers
		tgOpenGLSupport)

add_executable(TandemTest
    TandemModel6.cpp
    AppTANDEMball.cpp
    #T6Controller.cpp
) 
