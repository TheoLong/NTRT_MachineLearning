Project(cpeterson)

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

add_executable(CPetersonTest
    SuperBall/T6Model.cpp
    SuperBall/AppSUPERball.cpp
    SuperBall/T6Controller.cpp
) 

add_executable(TandemTest
    Tandem/AppTANDEMball.cpp
    Tandem/TandemModel6.cpp
    Tandem/TandemController.cpp
)

add_executable(TandemD2L
    TandemD2L/AppTANDEMball.cpp
    TandemD2L/TandemModel6.cpp
    TandemD2L/TandemController.cpp
)
