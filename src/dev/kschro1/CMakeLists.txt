Project(kschro1)

link_libraries(tgcreator core Adapters)

add_executable(AppSuperBall_VTech
    SUPERball_VTech/T6Model.cpp
    SUPERball_VTech/AppSUPERball_VTech.cpp
) 

add_executable(AppSensoredSuperBallPayload
    SensoredSuperBallPayload/SensoredSuperBallPayload.cpp
    SensoredSuperBallPayload/SensoredSuperBallPayloadController.cpp
    SensoredSuperBallPayload/AppSensoredSuperBallPayload.cpp
)

add_executable(AppCurvedSuperBallPayload
    CurvedSuperBallPayload/CurvedSuperBallPayload.cpp
    CurvedSuperBallPayload/CurvedSuperBallPayloadController.cpp
    CurvedSuperBallPayload/AppCurvedSuperBallPayload.cpp
)
