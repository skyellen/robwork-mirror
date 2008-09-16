add_library(rw_sandbox
        Dummy.cpp
        pathanalysis/PathAnalyzer.cpp
        VelocityRamps/SyncVelocityRamp.cpp
        control/JointController.cpp
        drawable/TactileArrayRender.cpp
        TactileArrayUtil.cpp
		FrameMap.cpp
		${RW_SANDBOX_USER_FILES}
# This template stuff doesn't compile with VC++.
)
